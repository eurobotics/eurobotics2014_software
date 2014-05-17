/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 *  Javier Baliñas Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
//#include <trajectory_manager_core.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <scheduler.h>


#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "../maindspic/strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* Add here the main strategic, the inteligence of robot */

/**
 * STRAT EVENTS 
 */

/* schedule a single strat tevent */
void strat_schedule_single_event(void (*f)(void *), void * data)
{
	uint8_t flags;
	int8_t ret;

	/* delete current event */
	scheduler_del_event(mainboard.strat_event);

	/* add event */
	IRQ_LOCK(flags);
	ret  = scheduler_add_single_event_priority(f, data, 
											   EVENT_PERIOD_STRAT/SCHEDULER_UNIT, EVENT_PRIORITY_STRAT_EVENT);
	mainboard.strat_event = ret;
	IRQ_UNLOCK(flags);
}

/* schedule a periodical strat tevent */
void strat_schedule_periodical_event(void (*f)(void *), void * data)
{
	uint8_t flags;
	int8_t ret;

	/* delete current event */
	scheduler_del_event(mainboard.strat_event);

	/* add event */
	IRQ_LOCK(flags);
	ret  = scheduler_add_periodical_event_priority(f, data,
			EVENT_PERIOD_STRAT/SCHEDULER_UNIT, EVENT_PRIORITY_STRAT_EVENT);
	mainboard.strat_event = ret;
	IRQ_UNLOCK(flags);
}


/* wait for traj end event */
void strat_wait_traj_end_event (void *data)
{
	uint16_t *__data = data;

	uint8_t ret = wait_traj_end ((uint8_t)__data[0]);

	/* return thru bt protocol */	
	bt_set_cmd_ret (ret);
}

/* auto possition depending on color */
void strat_auto_position (void)
{
#define AUTOPOS_SPEED_FAST 	1000
#define BASKET_WIDTH		300

	uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save & set speeds */
	interrupt_traj_reset();
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

	/* goto blocking to y axis */
	trajectory_d_rel(&mainboard.traj, -200);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);

	/* set y */
	strat_reset_pos(DO_NOT_SET_POS, BASKET_WIDTH + ROBOT_CENTER_TO_BACK, 90);

	/* prepare to x axis */
	trajectory_d_rel(&mainboard.traj, 45);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;

	trajectory_a_rel(&mainboard.traj, COLOR_A_REL(-90));
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;


	/* goto blocking to x axis */
	trajectory_d_rel(&mainboard.traj, -700);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	
	/* set x and angle */
	strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_BACK), DO_NOT_SET_POS, COLOR_A_ABS(0));
	
	/* goto start position */
	trajectory_d_rel(&mainboard.traj, 175);
	err = wait_traj_end(END_INTR|END_TRAJ);
	if (err == END_INTR)
		goto intr;
	wait_ms(100);
	
	/* restore speeds */	
	strat_set_speed(old_spdd, old_spda);
	return;

intr:
	strat_hardstop();
	strat_set_speed(old_spdd, old_spda);
}


/* auto position event */
void strat_auto_position_event (void *data)
{
	strat_auto_position ();

	/* return thru bt protocol */	
	bt_set_cmd_ret (END_TRAJ);

	/* print end pos */
	printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"), 
		 position_get_x_double(&mainboard.pos),
		 position_get_y_double(&mainboard.pos),
		 DEG(position_get_a_rad_double(&mainboard.pos)));
}


/* patrol between 2 points depending on nearest opponent */
uint8_t strat_patrol_between(int16_t x1, int16_t y1,int16_t x2, int16_t y2)
{


	int8_t opp_there, r2nd_there;
	int16_t opp_x, opp_y, x_aux;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
	uint8_t err = 0;
	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
  	strat_limit_speed_disable ();
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_VERY_SLOW);
	/* get robot coordenates */
	opp_there = get_opponent1_xy(&opp_x, &opp_y);

	if(opp_there == -1 )
		printf_P("return 0;");
 	if(x1>x2) x_aux=x1;
	else{
		x_aux=x2;
	}
	if(opp1_x_is_more_than(COLOR_X(x_aux+600))){
		if(opp1_y_is_more_than(y1)&&opp1_y_is_more_than(y2)){
			printf_P("More than Ys opp 1.\n");
		}
		else if((opp1_y_is_more_than(y1)&&!opp1_y_is_more_than(y2))||(!opp1_y_is_more_than(y1)&&opp1_y_is_more_than(y2))){
			printf_P("Between Ys opp 1.\n");
			trajectory_goto_xy_abs (&mainboard.traj,(x1+x2)/2,opp_y);
		}
	}else{
	printf_P("More than X opp 1.\n");
	}
		printf_P("X: %d Y: %d.\n",opp_x,opp_y);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);

	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
end:
	strat_set_speed(old_spdd, old_spda);
	strat_limit_speed_enable();
    return err;
}


#if notyet /* TODO 2014 */

/* return 1 if is a valid zone and 0 otherwise */
uint8_t strat_is_valid_zone(uint8_t zone_num)
{
#define OPP_WAS_IN_ZONE_TIMES	

	//static uint16_t opp_times[ZONES_MAX];
	//static microseconds opp_time_us = 0;

	/* discard actual zone */
	//if(strat_infos.current_zone == zone_num)
	//	return 0;

	/* discard down side zones depends on strat config */
	if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) == 0 
		&& strat_infos.zones[zone_num].init_y < (AREA_Y/2)
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
/*
	else if((strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES) 
		&& strat_infos.zones[zone_num].init_y > (AREA_Y/2) 
		&& zone_num != ZONE_SHIP_OUR_DECK_2
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )
		return 0;
*/
	/* discard if opp is in zone */
	if(opponent_is_in_area(	COLOR_X(strat_infos.zones[zone_num].x_up),strat_infos.zones[zone_num].y_up,
								COLOR_X(strat_infos.zones[zone_num].x_down),	strat_infos.zones[zone_num].y_down)) {

#if 0
		if(time_get_us2() - opp_time_us < 100000UL)
		{
			opp_time_us = time_get_us2();

			opp_times[zone_num]++;
			if(opp_times[zone_num] > OPP_WAS_IN_ZONE_TIMES)
				strat_infos.zones[zone_num].flags |= ZONE_CHECKED_OPP;
		}
#endif
		DEBUG(E_USER_STRAT, "Discarted zone %s, opp inside", numzone2name[zone_num]);
		return 0;
	}

	/* discard our checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
		return 0;	

	/* discard opp checked zones */
	if((strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
		&& zone_num != ZONE_SHIP_OUR_CAPTAINS_BEDRROM 
		&& zone_num != ZONE_SHIP_OUR_HOLD 
		&& zone_num != ZONE_SHIP_OUR_DECK_2 
		&& zone_num != ZONE_SHIP_OUR_DECK_1 )

		return 0;	

	/* discard avoid zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
		return 0;	

	/* if we have treasure on mouth, we only can send messages, save treasure on ship or pickup middle coins group */
	if(strat_infos.treasure_in_mouth) {
		if(zone_num != ZONE_SHIP_OUR_CAPTAINS_BEDRROM
			&& zone_num != ZONE_SHIP_OUR_DECK_1
			&& zone_num != ZONE_SHIP_OUR_DECK_2
			&& zone_num != ZONE_SHIP_OUR_HOLD
			&& zone_num != ZONE_SAVE_TREASURE
			&& zone_num != ZONE_MIDDLE_COINS_GROUP
			&& zone_num != ZONE_OUR_BOTTLE_1
			&& zone_num != ZONE_OUR_BOTTLE_2 )
		
		return 0;
	}
	/* if we have not treasure on mouth, we have not to save treasure any where */
	else {
		if(zone_num == ZONE_SHIP_OUR_CAPTAINS_BEDRROM
		|| zone_num == ZONE_SHIP_OUR_DECK_1
		|| zone_num == ZONE_SHIP_OUR_DECK_2
		|| zone_num == ZONE_SAVE_TREASURE)
		
		return 0;
	}

	/* TODO depending on goldbars in boot */
	if(!strat_infos.treasure_in_boot && zone_num == ZONE_SHIP_OUR_HOLD)
		return 0;

	return 1;
}

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void)
{
	uint16_t prio_max = 0;
	int8_t zone_num = -1;
	int8_t i;
	
	/* evaluate zones */
	for(i=0; i < ZONES_MAX; i++) 
	{
		/* check if is a valid zone */
		if(!strat_is_valid_zone(i))	
			continue;

		/* check priority */
		if(strat_infos.zones[i].prio > prio_max) {

			/* update zone candidate params */
			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}
	}

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t zone_num)
{
	//double d_rel = 0.0, a_rel_rad = 0.0;
	//uint8_t arm_type = 0;
	int8_t err;

	/* special cases */

	if(zone_num == ZONE_TOTEM_OPP_SIDE_2) {

		if(!opp_y_is_more_than(1200) && !opp2_y_is_more_than(1200))
			strat_limit_speed_disable();

		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

		strat_limit_speed_enable();
	}

	else if(zone_num == ZONE_MIDDLE_COINS_GROUP) {
		if(position_get_x_s16(&mainboard.pos) > (AREA_X/2))
			err = goto_and_avoid_forward((AREA_X - strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		else
			err = goto_and_avoid_forward(strat_infos.zones[zone_num].init_x, strat_infos.zones[zone_num].init_y, 
								TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_BOTTLE) {
		err = goto_and_avoid_backward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	else if(zone_num == ZONE_SHIP_OUR_DECK_1 || zone_num == ZONE_SHIP_OUR_DECK_2) {
		err = goto_and_avoid_forward(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

	}
	/* by default */
	else {
		err = goto_and_avoid(COLOR_X(strat_infos.zones[zone_num].init_x), strat_infos.zones[zone_num].init_y, 
							TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	}

	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	int16_t x = strat_infos.zones[zone_num].x;
	int16_t y = strat_infos.zones[zone_num].y;

	static uint8_t first_time = 1;

#ifdef DEBUG_STRAT_SMART
	return END_TRAJ;
#endif

	if(strat_infos.zones[zone_num].type == ZONE_TYPE_TOTEM) {

#if 0
		if(zone_num == ZONE_TOTEM_OUR_SIDE_1) {
			err = strat_empty_totem_side(COLOR_X(x), y, STORE_BOOT, step_our_totem_1);

		}
		else if(zone_num == ZONE_TOTEM_OUR_SIDE_2) {

		}
		else if(zone_num == ZONE_TOTEM_OPP_SIDE_1) {

		}
		else if(zone_num == ZONE_TOTEM_OPP_SIDE_2) {

		}
#else
		err = strat_empty_totem_side(COLOR_X(x), y, STORE_BOOT, 0);
#endif
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_GOLDBAR) {
		err = strat_pickup_goldbar_floor(COLOR_X(x), y, STORE_BOOT);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_MAP) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_BOTTLE) {
		if(zone_num == ZONE_OUR_BOTTLE_1)
			x = x + 80;

		err = strat_send_message_bottle(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COIN) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, ONE);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_COINS_GROUP) {
		err = strat_pickup_coins_floor(COLOR_X(x), y, GROUP);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_HOLD) {
		err = strat_save_treasure_in_hold_back(COLOR_X(x), y);
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_DECK) {

		if(zone_num == ZONE_SHIP_OUR_DECK_2) {
			
			if(time_get_s() < LAST_SECONDS_TIME + 5) {

				if(mainboard.our_color == I2C_COLOR_PURPLE)
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);
				else
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
	
				if(!(first_time) && strat_infos.treasure_in_boot)
					err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
				first_time = 0;
			}
			else {
				if(!(first_time) && strat_infos.treasure_in_boot)
					err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
				first_time = 0;

				if(mainboard.our_color == I2C_COLOR_PURPLE)
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);
				else
					err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
			}
		}
		else if(zone_num == ZONE_SHIP_OUR_DECK_1) {
			
			if(mainboard.our_color == I2C_COLOR_PURPLE)
				err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_LEFT);
			else
				err = strat_save_treasure_arms(COLOR_X(x), y, I2C_ARM_TYPE_RIGHT);

			if(strat_infos.treasure_in_boot)
				err = strat_save_treasure_in_deck_back_blowing(COLOR_X(x), y);
		}
		else {
			err = strat_stole_opp_treasure(COLOR_X(x), y);
		}	
	}


	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		/* TODO */
		err = END_TRAJ;
	}
	else if(strat_infos.zones[zone_num].type == ZONE_TYPE_SAVE) {
		err = strat_save_treasure_generic(COLOR_X(x), y);
	}

	return err;

}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!strat_infos.debug_step)
		return;

	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* smart play */
uint8_t strat_smart(void)
{
	int8_t zone_num;
	uint8_t err;

	/* XXX DEBUG STEP BY STEP */
	state_debug_wait_key_pressed();


	/* get new zone */
	if(time_get_s() > LAST_SECONDS_TIME 
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_1].flags & ZONE_CHECKED)
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_2].flags & ZONE_CHECKED))
		zone_num = ZONE_SHIP_OUR_DECK_2;
	else
		zone_num = strat_get_new_zone();

		
	if(zone_num == -1) {
		DEBUG(E_USER_STRAT, "No zone is found");
		DEBUG(E_USER_STRAT, "Down side zones enabled");	
		strat_infos.conf.flags |= ENABLE_DOWN_SIDE_ZONES;	
		strat_infos.conf.flags |= ENABLE_R2ND_POS;

		/* Enable all paths */
		DEBUG(E_USER_STRAT, "All possible paths enabled");	
		strat_set_bounding_box(-1);	
		return END_TRAJ;
	}

	/* goto zone */
	strat_infos.goto_zone = zone_num;
	strat_dump_infos(__FUNCTION__);
	
	err = strat_goto_zone(strat_infos.goto_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Can't reach zone %d", strat_infos.goto_zone);
		return END_TRAJ;
	}

	/* work on zone */
	strat_infos.last_zone = strat_infos.current_zone;
	strat_infos.current_zone = strat_infos.goto_zone;
	strat_dump_infos(__FUNCTION__);

	err = strat_work_on_zone(strat_infos.current_zone);
	if (!TRAJ_SUCCESS(err)) {
		DEBUG(E_USER_STRAT, "Work on zone %d fails", strat_infos.current_zone);

		if(strat_infos.zones[zone_num].type == ZONE_TYPE_TOTEM) 
		{
			strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
         i2c_slavedspic_mode_turbine_angle(0,200);
         i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_lift_height(30);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM,I2C_FINGERS_MODE_HOLD,0);
			i2c_slavedspic_wait_ready();
         i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR,I2C_FINGERS_MODE_CLOSE,0);
         i2c_slavedspic_wait_ready();
		}
		return err;
	}

	/* special case */
//	if(strat_infos.current_zone == ZONE_SHIP_OUR_DECK_2 
//		&& strat_infos.zones[ZONE_SHIP_OUR_DECK_2].prio == ZONE_PRIO_80)
//		strat_infos.zones[ZONE_SHIP_OUR_DECK_2].prio = ZONE_PRIO_50;

	/* check the zone */

 	/* don't check if it's saving zone */
	if(strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_HOLD 
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_DECK
		|| strat_infos.zones[strat_infos.current_zone].type == ZONE_TYPE_CAPTAINS_BEDROOM) {
		DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
		return END_TRAJ;
	}
	else	{
		strat_infos.zones[strat_infos.current_zone].flags |= ZONE_CHECKED;
	}

	DEBUG(E_USER_STRAT, "Work on zone %d successed!", strat_infos.current_zone);
	return END_TRAJ;
}


#endif /* notyet TODO 2014 */



