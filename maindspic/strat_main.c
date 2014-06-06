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
 *  Javier Bali�as Santos <javier@arc-robots.org> and Silvia Santano
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
#include <dac_mc.h>
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


#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"
#include "bt_protocol.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* Add here the main strategy, the intelligence of robot */


/* return 1 if is a valid zone and 0 otherwise */
uint8_t strat_is_valid_zone(uint8_t zone_num)
{
#define OPP_WAS_IN_ZONE_TIMES	

	//static uint16_t opp_times[ZONES_MAX];
	//static microseconds opp_time_us = 0;

	/* discard current zone */
	if(strat_infos.current_zone == zone_num)
		return 0;

	/* discard if opp is in zone */
	if(opponents_are_in_area(COLOR_X(strat_infos.zones[zone_num].x_up), strat_infos.zones[zone_num].y_up,
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
		printf_P(PSTR("Discarded zone %s, opp inside\r\n"), numzone2name[zone_num]);
		return 0;
	}

	/* discard avoid and checked zones */
	if(strat_infos.zones[zone_num].flags & ZONE_AVOID)
		return 0;	

	if(strat_infos.zones[zone_num].type==ZONE_TYPE_BASKET)
	{
		if(strat_infos.harvested_trees==0) 
			return 0;
	}
	else
	{
		if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
			return 0;	
	}

	if((strat_infos.zones[zone_num].type==ZONE_TYPE_TREE) && (strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP))
		return 0;
		
	return 1;
}

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void)
{
	uint8_t prio_max = 0;
	int8_t zone_num = -1;
	int8_t i=0;
	
	/* evaluate zones */
	for(i=0; i < ZONES_MAX; i++) 
	{
		/* check if is a valid zone */
		if(!strat_is_valid_zone(i))	
			continue;

		/* compare current priority */
		if(strat_infos.zones[i].prio >= prio_max) {

			prio_max = strat_infos.zones[i].prio;
			zone_num = i;
		}

        /* XXX force go to basket if timeout */
		if((time_get_s() > 75) && (strat_infos.harvested_trees))
			zone_num = ZONE_BASKET_2;
	}

	return zone_num;
}

/* return END_TRAJ if zone is reached, err otherwise */
uint8_t strat_goto_zone(uint8_t zone_num)
{
#define BASKET_OFFSET_SIDE 175
#define BEGIN_LINE_Y 	450
#define BEGIN_FRESCO_X	1295
#define PROTECT_H1_X 400
#define PROTECT_H1_Y 1500

	int8_t err=0;
	
	/* update strat_infos */
	strat_infos.current_zone=-1;
	strat_infos.goto_zone=zone_num;
	
	/* Secondary robot */
	if(strat_infos.zones[zone_num].robot==SEC_ROBOT)
	{
		if(strat_infos.zones[zone_num].type==ZONE_TYPE_FRESCO)
			trajectory_goto_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), BEGIN_LINE_Y);
			
		else if(strat_infos.zones[zone_num].type==ZONE_TYPE_MAMOOTH);
			//Not needed, goto mamooth and work in mamooth are in the same function in sec robot
			
		else if(strat_infos.zones[zone_num].type==ZONE_TYPE_HEART)
		{
			if(zone_num==ZONE_HEART_1)
			{
				bt_robot_2nd_goto_xy_abs(COLOR_X(FIRE_1_X),FIRE_1_Y);
				bt_robot_2nd_wait_end();
				bt_robot_2nd_goto_xy_abs(COLOR_X(PROTECT_H1_X),PROTECT_H1_Y);
				bt_robot_2nd_wait_end();
			}
			else if  (zone_num==ZONE_HEART_3)
			{
				bt_robot_2nd_goto_xy_abs(COLOR_X(3000-FIRE_1_X),FIRE_1_Y);
				bt_robot_2nd_wait_end();
				bt_robot_2nd_goto_xy_abs(COLOR_X(3000-PROTECT_H1_X),PROTECT_H1_Y);
				bt_robot_2nd_wait_end();
			}
			
			else if(zone_num==ZONE_HEART_2_DOWN || zone_num==ZONE_HEART_2_UP || zone_num==ZONE_HEART_2_RIGHT || zone_num==ZONE_HEART_2_LEFT)
			{
			 /* TODO : remove fires from opponent. At the moment only protect ours */
			}
		}
	}
	
	else
	{
		/* go */
		if (zone_num == ZONE_TREE_1 || zone_num == ZONE_TREE_2 
			|| zone_num == ZONE_TREE_3 || zone_num == ZONE_TREE_4) 	{
			err = goto_and_avoid_forward (COLOR_X(strat_infos.zones[zone_num].init_x), 
										strat_infos.zones[zone_num].init_y,  
										TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		}
		else if (zone_num == ZONE_BASKET_2) 	{
			if (opp1_x_is_more_than(3000-750) || opp2_x_is_more_than(3000-750) ) {
				err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x - BASKET_OFFSET_SIDE), 
											strat_infos.zones[zone_num].init_y,  
											TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
			}
			else {
				err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x + BASKET_OFFSET_SIDE), 
											strat_infos.zones[zone_num].init_y,  
											TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
			}

		}
		else if (strat_infos.zones[zone_num].type == ZONE_TYPE_FIRE) {
		err = strat_goto_orphan_fire (zone_num);
		}
		else if (strat_infos.zones[zone_num].type == ZONE_TYPE_TORCH) {
		err = strat_goto_torch (zone_num);
		}
		else if (strat_infos.zones[zone_num].type == ZONE_TYPE_HEART) {
		err = strat_goto_heart_fire (zone_num);
		}
		else if (strat_infos.zones[zone_num].type == ZONE_TYPE_M_TORCH) {
			strat_goto_mobile_torch (zone_num);
		}
		else {
			if(strat_infos.zones[zone_num].robot==MAIN_ROBOT)
				err = goto_and_avoid (COLOR_X(strat_infos.zones[zone_num].init_x),  strat_infos.zones[zone_num].init_y,  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		}	
	}
	
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
	/* update strat_infos */
	strat_infos.last_zone=strat_infos.current_zone;
	strat_infos.goto_zone=-1;
	if (!TRAJ_SUCCESS(err)) {
		strat_infos.current_zone=-1;
	}
	else{
		strat_infos.current_zone=zone_num;
	}

end:
    /* TODO XXX if error put arm in safe position */
	return err;
}


/* return END_TRAJ if the work is done, err otherwise */
uint8_t strat_work_on_zone(uint8_t zone_num)
{
	uint8_t err = END_TRAJ;
	
#ifdef HOST_VERSION
	printf_P(PSTR("strat_work_on_zone %s: press a key\r\n"),numzone2name[zone_num]);
	while(!cmdline_keypressed());
#endif

    /* XXX if before the tree harvesting was interrupted by opponent */    
    if (strat_infos.tree_harvesting_interrumped) {
        strat_infos.tree_harvesting_interrumped = 0;
        i2c_slavedspic_wait_ready();
        i2c_slavedspic_mode_harvest_fruits (I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_END);
    }
    
	/* Secondary robot */
	if(strat_infos.zones[zone_num].robot==SEC_ROBOT)
	{
		if(strat_infos.zones[zone_num].type==ZONE_TYPE_FRESCO)
			bt_robot_2nd_bt_fresco();
			
		else if(strat_infos.zones[zone_num].type==ZONE_TYPE_MAMOOTH)
			bt_robot_2nd_bt_task_mamooth(6,0);
		
		if(strat_infos.zones[zone_num].type==ZONE_TYPE_HEART)
		{
			if(zone_num==ZONE_HEART_1)
			{
				bt_robot_2nd_bt_protect_h(1);
				strat_infos.zones[ZONE_HEART_1].prio=ZONE_PRIO_0;
			}
			else if(zone_num==ZONE_HEART_3)
			{
				bt_robot_2nd_bt_protect_h(3);
				strat_infos.zones[ZONE_HEART_3].prio=ZONE_PRIO_0;
			}
				
			else if(zone_num==ZONE_HEART_2_DOWN || zone_num==ZONE_HEART_2_UP || zone_num==ZONE_HEART_2_RIGHT || zone_num==ZONE_HEART_2_LEFT)
			{
			/* TODO : remove fires from opponent. At the moment only protect ours */
			}
		}
	}
	
	else
	{
		switch(zone_num)
		{
			case ZONE_TREE_1:
			case ZONE_TREE_2:
				err = strat_harvest_fruits (COLOR_X (strat_infos.zones[zone_num].x), strat_infos.zones[zone_num].y, 0);
				if(TRAJ_SUCCESS(err))
				{
					strat_infos.harvested_trees++;
					strat_infos.zones[ZONE_BASKET_2].prio+=PRIO_BASKET_AFTER_ONE_TREE;
				}
				break;

			case ZONE_TREE_3:
			case ZONE_TREE_4:
				err = strat_harvest_fruits (COLOR_X (strat_infos.zones[zone_num].x), strat_infos.zones[zone_num].y, 1);
				if(TRAJ_SUCCESS(err))
				{
					strat_infos.harvested_trees++;
					strat_infos.zones[ZONE_BASKET_2].prio+=PRIO_BASKET_AFTER_ONE_TREE;
				}
				break;
				
			case ZONE_FIRE_1:
			case ZONE_FIRE_2:
			case ZONE_FIRE_3:
			case ZONE_FIRE_4:
			case ZONE_FIRE_5:
			case ZONE_FIRE_6:
			err = strat_harvest_orphan_fire (zone_num);
				break;
				
			case ZONE_TORCH_1:
			case ZONE_TORCH_2:
			case ZONE_TORCH_3:
			case ZONE_TORCH_4:
				err = strat_harvest_torch (zone_num);
				break;
				
			case ZONE_HEART_1:
			case ZONE_HEART_3:
				err = strat_make_puzzle_on_heart (zone_num);
				if(TRAJ_SUCCESS(err))
				{
					strat_infos.zones[zone_num].prio=PRIO_HEART_AFTER_PUZZLE;
					strat_infos.zones[zone_num].robot=SEC_ROBOT;
				}
				break;

			case ZONE_HEART_2_UP:
			case ZONE_HEART_2_LEFT:
			case ZONE_HEART_2_DOWN:
			case ZONE_HEART_2_RIGHT:
				/* wipe by main or by sec  */
				break;
				
			case ZONE_M_TORCH_1:
			case ZONE_M_TORCH_2:
				err = strat_pickup_mobile_torch_top(zone_num);
				err = strat_pickup_mobile_torch_mid(zone_num);
				err = strat_pickup_mobile_torch_bot(zone_num);
				break;
				
			case ZONE_BASKET_1:
			case ZONE_BASKET_2:
				err = goto_basket_best_path(zone_num);
				
				/* update strat_infos */
				if(TRAJ_SUCCESS(err))
				{
					strat_infos.harvested_trees=0;
					strat_infos.zones[ZONE_BASKET_2].prio=ZONE_PRIO_0;
					break;
				}
		}
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


void recalculate_priorities(void)
{
	#if 0
	uint8_t zone_num;
	
	for(zone_num=0; zone_num<ZONES_MAX; zone_num++)
	{
		/* 1. opp checked zone AFTER us */
		if((strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)&&
		(strat_infos.zones[zone_num].flags & ZONE_CHECKED))
		{
				//TODO
		}
		
		/* 2. checked zone */
		else if(strat_infos.zones[zone_num].flags & ZONE_CHECKED)
		{
				//TODO
		}
		
		/* 3. Points we can get now in this zone */
		switch(strat_infos.zones[zone_num].type)
		{
			case ZONE_TYPE_HEART:
				if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					strat_zones_points[zone_num]=4;
					
				/* The robot has fires inside */
				strat_zones_points[zone_num]+=(strat_infos.fires_inside*2);
				break;
				
			case ZONE_TYPE_TREE:
				/* If visited by the opponent: no points */
				if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					strat_zones_points[zone_num]=0;
				break;
				
			case ZONE_TYPE_FIRE:
				if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					strat_zones_points[zone_num]=0;
				break;
				
			case ZONE_TYPE_TORCH:
				if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					strat_zones_points[zone_num]=0;
				break;
				
			case ZONE_TYPE_M_TORCH:
				if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					strat_zones_points[zone_num]=0;
				break;
				
				
				
			case ZONE_TYPE_BASKET:
				/* Save fruits on basket */
				strat_zones_points[zone_num]=strat_infos.harvested_trees*3;
				
				/* Opponent has given us toxic fruits */
				// XXX Remove toxic fruits available???
				if(((mainboard.our_color==I2C_COLOR_YELLOW) && (zone_num==ZONE_BASKET_2)) ||
				((mainboard.our_color==I2C_COLOR_RED) && (zone_num==ZONE_BASKET_1)))
				{
					if(strat_infos.zones[zone_num].flags & ZONE_CHECKED_OPP)
					{
						if(strat_infos.zones[ZONE_TREE_1].flags & ZONE_CHECKED_OPP)
							strat_zones_points[zone_num]+=2;
						if(strat_infos.zones[ZONE_TREE_2].flags & ZONE_CHECKED_OPP)
							strat_zones_points[zone_num]+=2;
						if(strat_infos.zones[ZONE_TREE_3].flags & ZONE_CHECKED_OPP)
							strat_zones_points[zone_num]+=2;
						if(strat_infos.zones[ZONE_TREE_4].flags & ZONE_CHECKED_OPP)
							strat_zones_points[zone_num]+=2;
					}
				}
				break;
				
			/* Remain the same */
			case ZONE_TYPE_FRESCO:
			case ZONE_TYPE_MAMOOTH:
			case ZONE_TYPE_HOME:
				break;
				
			default:
				break;
		}
		
		/* 4. Distance from robot to the zone */
		// TODO
		//Give points to closest zones
		
		
		
		/* Recalculate priorities depending on strategy TODO*/
		/***********************************************************/
		/* Defensive: protect our points */
		
		/* Risky: try to get the maximum points without protecting */
		
		/* Moderate: combine defense and risk */
		
		/* Offensive: make the opponent lose points */
		/***********************************************************/
	}
	#endif
}


/* smart play */
uint8_t strat_smart(void)
{
	int8_t zone_num;
	uint8_t err;

	/* recalculate priorities NOTYET */
	//recalculate_priorities();
	
	/* get new zone */
	zone_num = strat_get_new_zone();
		
	if(zone_num == -1) {
		printf_P(PSTR("No zone is found\r\n"));
		return END_TRAJ;
	}

	else
	{
		/* goto zone */
		printf_P(PSTR("Going to zone %s.\r\n"),numzone2name[zone_num]);
		strat_infos.goto_zone = zone_num;
		strat_dump_infos(__FUNCTION__);
		
		err = strat_goto_zone(zone_num);
		if (!TRAJ_SUCCESS(err)) {
			printf_P(PSTR("Can't reach zone %d.\r\n"), zone_num);
			return END_TRAJ;
		}

		/* work on zone */
		strat_infos.last_zone = strat_infos.current_zone;
		strat_infos.current_zone = strat_infos.goto_zone;
		strat_dump_infos(__FUNCTION__);

		err = strat_work_on_zone(zone_num);
		if (!TRAJ_SUCCESS(err)) {
			printf_P(PSTR("Work on zone %s fails.\r\n"), numzone2name[zone_num]);
		}
		else
		{
			// Switch off devices, go back to normal state if anything was deployed
		}

		/* mark the zone as checked */
		if(strat_infos.zones[zone_num].type!=ZONE_TYPE_BASKET)
			strat_infos.zones[zone_num].flags |= ZONE_CHECKED;
		//strat_infos.zones[zone_num].flags &= ~(ZONE_CHECKED_OPP);

		//printf_P(PSTR("Work on zone %s succeeded!\r\n"), numzone2name[zone_num]);
		return END_TRAJ;
	}
}


void strat_opp_tracking (void) 
{
#define MAX_TIME_BETWEEN_VISITS_MS	4000
#define TIME_MS_TREE				1500
#define TIME_MS_HEART				1500
#define TIME_MS_BASKET				1000
#define UPDATE_ZONES_PERIOD_MS		25	
	
	uint8_t flags;
	uint8_t zone_opp;
	
    /* check if there are opponents in every zone */
    for(zone_opp = 0; zone_opp <  ZONES_MAX-1; zone_opp++)
    {
	   
	if(opponents_are_in_area(COLOR_X(strat_infos.zones[zone_opp].x_up), strat_infos.zones[zone_opp].y_up,
                                     COLOR_X(strat_infos.zones[zone_opp].x_down), strat_infos.zones[zone_opp].y_down)){
			
			if(!(strat_infos.zones[zone_opp].flags & (ZONE_CHECKED_OPP)))
			{
				IRQ_LOCK(flags);
				strat_infos.zones[zone_opp].last_time_opp_here=time_get_us2();
				IRQ_UNLOCK(flags);
				if((time_get_us2() - strat_infos.zones[zone_opp].last_time_opp_here) < MAX_TIME_BETWEEN_VISITS_MS*1000L)
				{
					/* Opponent continues in the same zone: */
					/* update zone time */ 
					IRQ_LOCK(flags);
					strat_infos.zones[zone_opp].opp_time_zone_us += UPDATE_ZONES_PERIOD_MS*1000L;
					IRQ_UNLOCK(flags);

					/* Mark zone as checked and sum points */
					switch(strat_infos.zones[zone_opp].type)
					{
						case ZONE_TYPE_TREE:
							if(strat_infos.zones[zone_opp].opp_time_zone_us>=TIME_MS_TREE*1000L)
							{
								strat_infos.zones[zone_opp].flags |= ZONE_CHECKED_OPP;
								strat_infos.opp_harvested_trees++;
								printf_P("opp_harvested_trees=%d\n",strat_infos.opp_harvested_trees);
								printf_P("OPP approximated score: %d\n", strat_infos.opp_score);
							}
							break;
						case ZONE_TYPE_BASKET:
							if(((mainboard.our_color==I2C_COLOR_YELLOW) && (zone_opp==ZONE_BASKET_1)) ||
							((mainboard.our_color==I2C_COLOR_RED) && (zone_opp==ZONE_BASKET_2)))
							{
								if(strat_infos.zones[zone_opp].opp_time_zone_us>=TIME_MS_BASKET*1000L)
								{
									if(strat_infos.opp_harvested_trees!=0)
									{
										strat_infos.opp_score += strat_infos.opp_harvested_trees * 3;
										strat_infos.opp_harvested_trees=0;
										printf_P("opp_harvested_trees=%d\n",strat_infos.opp_harvested_trees);
										printf_P("OPP approximated score: %d\n", strat_infos.opp_score);
									}
								}
							}
							break;
						case ZONE_TYPE_HEART:
							if(strat_infos.zones[zone_opp].opp_time_zone_us>= TIME_MS_HEART*1000L)
							{
								strat_infos.zones[zone_opp].flags |= ZONE_CHECKED_OPP;
								strat_infos.opp_score += 4;
								printf_P("OPP approximated score: %d\n", strat_infos.opp_score);
							}
							break;
						default:
							break;
					}
				}
							
				/* Zone has changed */
				else
				{
					/* reset zone time */
					IRQ_LOCK(flags);
					strat_infos.zones[zone_opp].opp_time_zone_us = 0;
					IRQ_UNLOCK(flags);
				}
			}
		}
	}

}


void strat_homologation(void)
{
	uint8_t err;
	uint8_t i=0;
	#define ZONES_SEQUENCE_LENGTH 6
	uint8_t zones_sequence[ZONES_SEQUENCE_LENGTH] = 						
	{ZONE_TORCH_1,ZONE_FIRE_1,ZONE_FIRE_3,ZONE_TORCH_2,ZONE_TORCH_3,ZONE_FIRE_5};
	
	/* Secondary robot */
	bt_robot_2nd_bt_patrol_fr_mam(6,0);
	time_wait_ms(2000);
	trajectory_d_rel(&mainboard.traj,250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	
	for(i=0; i<ZONES_SEQUENCE_LENGTH; i++)
	{
		/* goto zone */
		//printf_P(PSTR("Going to zone %s.\r\n"),numzone2name[zones_sequence[i]]);
		strat_dump_infos(__FUNCTION__);
		strat_infos.current_zone=-1;
		strat_infos.goto_zone=i;
		err = goto_and_avoid(COLOR_X(strat_infos.zones[i].init_x), strat_infos.zones[i].init_y,  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		strat_infos.last_zone=strat_infos.current_zone;
		strat_infos.goto_zone=-1;
		if (!TRAJ_SUCCESS(err)) {
			strat_infos.current_zone=-1;
			printf_P(PSTR("Can't reach zone %s.\r\n"), numzone2name[zones_sequence[i]]);
		}
		else{
			strat_infos.current_zone=i;
		}

		/* work on zone */
		strat_dump_infos(__FUNCTION__);
		err = strat_work_on_zone(zones_sequence[i]);
		if (!TRAJ_SUCCESS(err)) {
			printf_P(PSTR("Work on zone %s fails.\r\n"), numzone2name[zones_sequence[i]]);
		}

		/* mark the zone as checked */
		strat_infos.zones[i].flags |= ZONE_CHECKED;
	}
}


uint8_t goto_basket_best_path(uint8_t protect_zone_num)
{

	/*bt_robot_2nd_goto_xy_abs(COLOR_X(FIRE_1_X),FIRE_1_Y);
	bt_robot_2nd_wait_end();
	bt_robot_2nd_goto_xy_abs(COLOR_X(PROTECT_H1_X),PROTECT_H1_Y);
	bt_robot_2nd_wait_end();
	bt_robot_2nd_bt_protect_h1();
	bt_robot_2nd_wait_end();*/
	
	#define PROTECT_H1_X 400
	#define PROTECT_H1_Y 1600
	int8_t err;	
	int8_t opp1_there, opp2_there;
	int16_t opp_x, opp_y;
	int16_t opp2_x, opp2_y;

	/* get robot coordenates */
	opp1_there = get_opponent1_xy(&opp_x, &opp_y);
	opp2_there = get_opponent2_xy(&opp2_x, &opp2_y);
	
	/* Divide field in 2 parts (Y) and see where are the opponents: */
	/* Both opponents lower part */
	if(!opp1_y_is_more_than(CENTER_Y+50) && !opp2_y_is_more_than(CENTER_Y+50))
	{
		/* up */
		err=goto_basket_path_up();
	}
	/* Both opponents upper part */
	else if(opp1_y_is_more_than(CENTER_Y+50) && opp2_y_is_more_than(CENTER_Y+50))
	{
		/* down */
		err=goto_basket_path_down();
	}
	else if(opp1_there == -1 && opp2_there == -1){
		err=goto_basket_path_up();
	}

	/* One opponents upper part and one lower part */
	else
	{
		/* Check if there is a robot blocking the way up */
		if(opponents_are_in_area(COLOR_X(2100),2000,COLOR_X(900),1700))
		{
			/* up */
			err=goto_basket_path_up();
		}
		
		/* Check if there is a robot blocking the way down */
		else if(opponents_are_in_area(COLOR_X(1900),400,COLOR_X(1100),0))
		{
			/*down*/
			err=goto_basket_path_down();
		}
		else
		{/* Alternative strategy */
			printf_P("Oh no, both robots are blocking me :(.\n");
			err=goto_basket_path_up();
			
		}
	}
end:
	return err;
}
uint8_t goto_basket_path_up(void)
{
	int8_t opp1_there, opp2_there;
	int16_t opp_x, opp_y;
	int16_t opp2_x, opp2_y;

	/* get robot coordenates */
	opp1_there = get_opponent1_xy(&opp_x, &opp_y);
	opp2_there = get_opponent2_xy(&opp2_x, &opp2_y);
	
	printf_P("UP\n");
	int8_t err;
	
	err=goto_and_avoid (COLOR_X(FIRE_3_X),FIRE_3_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_STD);	
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);

	err=goto_and_avoid (COLOR_X(FIRE_5_X),FIRE_5_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_STD);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	
	err=goto_and_avoid (COLOR_X(FIRE_6_X),FIRE_6_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_STD);	
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	if(opponents_are_in_area(COLOR_X(3000),800,COLOR_X(2250),0)){
		strat_leave_fruits_from_fresco();
	}else{
		strat_leave_fruits_from_home_red();
	}
	
	
	end:
		return err;
}
uint8_t strat_leave_fruits_from_home_red(){
	//printf_P("Home red\n");
	#define BASKET_INIT_X 	 2700
	#define BASKET_INIT_Y 	 543
	
	int8_t err;
	int16_t x;
	
	err=goto_and_avoid (COLOR_X(BASKET_INIT_X),BASKET_INIT_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);	
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	err=goto_and_avoid (COLOR_X(BASKET_INIT_X-300),BASKET_INIT_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err); 
		
	x = 2600 - ROBOT_WIDTH/2;
	err=goto_and_avoid (COLOR_X(x),position_get_y_s16(&mainboard.pos),TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	
	err=strat_leave_fruits();
end:
	return err;
}
uint8_t strat_leave_fruits_from_fresco(){
	//printf_P("Fresco\n");
	#define BASKET_INIT_X 	 1650
	#define BASKET_INIT_Y 	 543
	int8_t err;
	int16_t x;
	
	err=goto_and_avoid (COLOR_X(BASKET_INIT_X),BASKET_INIT_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	err=goto_and_avoid (COLOR_X(BASKET_INIT_X+300),BASKET_INIT_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);
	x = 1900 + ROBOT_WIDTH/2;
	err=goto_and_avoid (COLOR_X(x),position_get_y_s16(&mainboard.pos),TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
	
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	
	err=strat_leave_fruits();
end:
	return err;
}
uint8_t goto_basket_path_down(void)
{
//printf_P("Down\n");
	int8_t err;
	int16_t x;
	err=goto_and_avoid (COLOR_X(FIRE_3_X),FIRE_3_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_STD);	
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	err=goto_and_avoid (COLOR_X(FIRE_2_X),FIRE_2_Y,TRAJ_FLAGS_STD,TRAJ_FLAGS_STD);
	//if (!TRAJ_SUCCESS(err))
	//	ERROUT(err);
	if(opponents_are_in_area(COLOR_X(3000),800,COLOR_X(2250),0)){
		strat_leave_fruits_from_fresco();
	}else{
		strat_leave_fruits_from_home_red();
	}
	
end:
	return err;
}

uint8_t strat_wipe_out(void){
	int16_t x,y,x_final,y_final;
	int8_t err,first_point=0, direction=0;
	
	#define H1_RADIUS 150
	#define MARGIN 10

	
	//printf_P("WIPE_OUT\n");
		if(x_is_more_than(COLOR_X(CENTER_X)))
		{
			if(y_is_more_than(CENTER_Y+50)){
				//RIGHT-UP 4
				if(opponents_are_in_area(COLOR_X(CENTER_X+H1_RADIUS+ROBOT_WIDTH+MARGIN), CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH+MARGIN,COLOR_X(CENTER_X),CENTER_Y+50)){
					if(!x_is_more_than(COLOR_X(2100))&&y_is_more_than(1600))
					{
						first_point=1;
					}else{
						first_point=3;
						}
				}else{
					first_point=4;
					
				}
				
			}else{
				//RIGHT-DOWN 3
				if(opponents_are_in_area(COLOR_X(CENTER_X+H1_RADIUS+ROBOT_WIDTH-MARGIN),CENTER_Y+50 ,COLOR_X(CENTER_X),CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH-MARGIN)){
					first_point=4;
				}else{
					first_point=3;
				}
				
	
			}
		}
		else {
			if(y_is_more_than(CENTER_Y+50)){
				//LEFT-UP 1
				if(opponents_are_in_area(COLOR_X(CENTER_X),CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH+MARGIN ,COLOR_X(CENTER_X-H1_RADIUS-ROBOT_WIDTH-MARGIN),CENTER_Y+50)){
					if(x_is_more_than(COLOR_X(900))&&y_is_more_than(1600))
					{
						first_point=4;
					}else{
						first_point=2;
					}
				}else{
					first_point=1;
				}
				
			
			}else{
				//LEFT-DOWN 2
				if(opponents_are_in_area(COLOR_X(CENTER_X),CENTER_Y+50 ,COLOR_X(CENTER_X-H1_RADIUS-ROBOT_WIDTH-MARGIN),CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH-MARGIN)){
					first_point=1;
				}else{
					first_point=2;
				}
			}
		}
		switch(first_point){
			
			case 1:
				x=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
				y=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
				
				if(opponents_are_in_area(COLOR_X(CENTER_X),CENTER_Y+50 ,COLOR_X(CENTER_X-H1_RADIUS-ROBOT_WIDTH-MARGIN),CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH-MARGIN)){
					x_final=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					y_final=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					//printf_P("LEFT-UP OPP\n");
					direction=0;
				}else{
					x_final=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					y_final=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					//printf_P("LEFT-UP NO OPP\n");
					direction=1;
				}
			break;
			case 2:
				x=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
				y=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
				if(opponents_are_in_area(COLOR_X(CENTER_X+H1_RADIUS+ROBOT_WIDTH-MARGIN),CENTER_Y+50 ,COLOR_X(CENTER_X),CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH-MARGIN)){
					x_final=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					y_final=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					//printf_P("LEFT-DOWN OPP\n");
					direction=0;
				}else{
					x_final=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					y_final=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					//printf_P("LEFT-DOWN NO OPP\n");
					direction=1;
				}
			break;
			case 3:
				x=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
				y=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
				if(opponents_are_in_area(COLOR_X(CENTER_X+H1_RADIUS+ROBOT_WIDTH+MARGIN), CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH+MARGIN,COLOR_X(CENTER_X),CENTER_Y+50)){
					x_final=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					y_final=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					//printf_P("RIGHT-DOWN OPP\n");
					direction=0;
				}else{
					x_final=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					y_final=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					//printf_P("RIGHT-DOWN NO OPP\n");
					direction=1;
				}
			break;
			case 4:
				x=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
				y=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
				if(opponents_are_in_area(COLOR_X(CENTER_X),CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH+MARGIN ,COLOR_X(CENTER_X-H1_RADIUS-ROBOT_WIDTH-MARGIN),CENTER_Y+50)){
					x_final=CENTER_X+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					y_final=CENTER_Y+50-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					//printf_P("RIGHT-UP OPP\n");
					direction=0;
				}else{
					x_final=CENTER_X-H1_RADIUS-ROBOT_WIDTH/2-MARGIN;
					y_final=CENTER_Y+50+H1_RADIUS+ROBOT_WIDTH/2+MARGIN;
					//printf_P("RIGHT-UP NO OPP\n");
					direction=1;
				}
	
				//printf_P("RIGHT-UP\n");
			break;
		}
		goto_and_avoid (x,y,TRAJ_FLAGS_STD,TRAJ_FLAGS_SMALL_DIST);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
										 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();
		//printf_P("Stick \n");
		
		
		if (direction==1){
			trajectory_goto_forward_xy_abs (&mainboard.traj, x_final,y_final);
			//printf_P("forward \n");
			}
		else{
			trajectory_goto_backward_xy_abs (&mainboard.traj, x_final,y_final);
			//printf_P("backward \n");
			}
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
	end:
	return err;	
	
	


}

