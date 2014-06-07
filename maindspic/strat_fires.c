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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano
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


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* goto orphan fire */
uint8_t strat_goto_orphan_fire (uint8_t zone_num) 
{
#define DIST_ORPHAN_FIRE_PUSH	360
#define DIST_ORPHAN_FIRE_PULL	210

    int16_t robot_x, robot_y, x, y;
	uint8_t color, level, err = 0;

    /* depending on robot and fire position */
    robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);

	/* position depending on color */
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    if ((zone_num == ZONE_FIRE_1) || (zone_num == ZONE_FIRE_6)) {
        if (robot_y > y) {
            
            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_1: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_6: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }

            /* y correction */  
            //if (color == mainboard.our_color)
                y += DIST_ORPHAN_FIRE_PUSH;
            //else
            //    y += DIST_ORPHAN_FIRE_PULL;

        }
        else {  

            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_1: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_6: color = COLOR_INVERT(I2C_COLOR_RED); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }

            /* y correction */   
            //if (color == mainboard.our_color)         
                y -= DIST_ORPHAN_FIRE_PUSH;
            //else
            //    y -= DIST_ORPHAN_FIRE_PULL;
        } 
    }
    else {
        if (robot_x > x) {

            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_2: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_3: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_4: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_5: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }

            /* x correction */  
            //if (color == mainboard.our_color)
                x += DIST_ORPHAN_FIRE_PUSH;
            //else
            //    x += DIST_ORPHAN_FIRE_PULL;
        }
        else {
            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_2: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_3: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_4: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_5: color = COLOR_INVERT(I2C_COLOR_RED); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }

            /* x correction */  
            //if (color == mainboard.our_color)
                x -= DIST_ORPHAN_FIRE_PUSH;
            //else
            //    x -= DIST_ORPHAN_FIRE_PULL;
        }
    }

    DEBUG (E_USER_STRAT, "fire is %s", color==I2C_COLOR_RED? "R":"Y");

    /* select level of sucker depending on infront color */
    if (color == mainboard.our_color)
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_PUSH_PULL;
    else
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_STANDUP;

    
    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);

    /* go near */
    err = goto_and_avoid_forward (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

    /* TODO XXX if error put arm in safe position */
    return err;
}



/* harvest orphan fires  */
uint8_t strat_harvest_orphan_fire (uint8_t zone_num)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

#define DIST_ORPHAN_FIRE_OVER_PUSH 150 //140
#define DIST_ORPHAN_FIRE_OVER_PULL 150

    uint8_t err = 0;
	uint16_t old_spdd, old_spda;
    int16_t d;
	uint8_t level, color;
    int16_t robot_x, robot_y, x, y;

	/* position depending on color */
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* depending on robot and fire position */
    robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to infront of fire */
    trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

    /* get the in front color */
    if ((zone_num == ZONE_FIRE_1) || (zone_num == ZONE_FIRE_6)) {
        if (robot_y > y) {
            switch (zone_num) {
                case ZONE_FIRE_1: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_6: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }
        }
        else {  
            switch (zone_num) {
                case ZONE_FIRE_1: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_6: color = COLOR_INVERT(I2C_COLOR_RED); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }
        } 
    }
    else {
        if (robot_x > x) {
            switch (zone_num) {
                case ZONE_FIRE_2: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_3: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_4: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_5: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }
        }
        else {
            /* infront color */
            switch (zone_num) {
                case ZONE_FIRE_2: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_3: color = COLOR_INVERT(I2C_COLOR_RED); break;
                case ZONE_FIRE_4: color = COLOR_INVERT(I2C_COLOR_YELLOW); break;
                case ZONE_FIRE_5: color = COLOR_INVERT(I2C_COLOR_RED); break;
                default: color = COLOR_INVERT(I2C_COLOR_YELLOW);
            }
        }
    }

    /* in case it's not our color go to pull distance */
    if (color != mainboard.our_color) {
        trajectory_d_rel(&mainboard.traj, 
                         ABS(DIST_ORPHAN_FIRE_PUSH-DIST_ORPHAN_FIRE_PULL) );
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
	        ERROUT(err);
    }

    /* ready for push/pull */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, 
                                              I2C_SLAVEDSPIC_LEVEL_FIRE_PUSH_PULL);
    i2c_slavedspic_wait_ready();

    /* push/pull fire */
    if (ABS(distance_from_robot (x,y) - DIST_ORPHAN_FIRE_PUSH) > 30) {
        d = -DIST_ORPHAN_FIRE_OVER_PULL;
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_GROUND_PULL;
    }
    else {
        d = DIST_ORPHAN_FIRE_OVER_PUSH;
        level = I2C_SLAVEDSPIC_LEVEL_FIRE_GROUND_PUSH;
    }
    trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* pickup fire */
    i2c_slavedspic_mode_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);
    i2c_slavedspic_wait_ready();


    /* XXX set color of fire */
    mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = mainboard.our_color;

	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

end:
    /* TODO XXX if error put arm in safe position */
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}


/* goto torch */
uint8_t strat_goto_torch (uint8_t zone_num)
{
    int16_t x, y;
	uint8_t err;

    /*  position */
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* init x,y correction */
    if (zone_num == ZONE_TORCH_1)       { x += COLOR_SIGN(DIST_ORPHAN_FIRE_PUSH); }//y+=5; }
    else if (zone_num == ZONE_TORCH_4)  { x -= COLOR_SIGN(DIST_ORPHAN_FIRE_PUSH); }//y+=5; }

    else if ((zone_num == ZONE_TORCH_2) || (zone_num == ZONE_TORCH_3))
    	{ y-=DIST_ORPHAN_FIRE_PUSH; x +=10;}

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

    /* go near */
    err = goto_and_avoid (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

    /* TODO XXX if error put arm in safe position */

	return err;
}


/* harvest torch  */
uint8_t strat_harvest_torch (uint8_t zone_num)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

#define DIST_TORCH_OVER 70

    uint8_t err = 0;
	uint16_t old_spdd, old_spda;
    int16_t x, y;

    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to infront of torch */
    trajectory_turnto_xy (&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);
    i2c_slavedspic_wait_ready();

    /* push/pull fire */
    trajectory_d_rel(&mainboard.traj, DIST_TORCH_OVER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

    /* pickup fire */
    i2c_slavedspic_mode_pickup_torch(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);
    i2c_slavedspic_wait_ready();

    /* XXX set color of fire */
    if (zone_num == ZONE_TORCH_1 || zone_num == ZONE_TORCH_3)
        mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = COLOR_INVERT(I2C_COLOR_RED);
    else
       mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = COLOR_INVERT(I2C_COLOR_YELLOW);
        
	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

    /* XXX return inside boundinbox */
    trajectory_d_rel(&mainboard.traj, -DIST_TORCH_OVER);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);
    
end:
    /* TODO XXX if error put arm in safe position */
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}

/* goto mobile torch */
uint8_t strat_goto_mobile_torch (uint8_t zone_num)
{
    int16_t robot_x, robot_y, x, y, x1, y2;
    double temp_x, temp_y;
	uint8_t err = 0;
	uint16_t a_abs;

#define MOBIL_TORCH_X_OFFSET    (45.0) //45
#define MOBIL_TORCH_Y_OFFSET    (255.0)
#define DIST_MOBIL_TORCH_SAFE	(360.0)
#define K_DIST_SAFE				(1.4)

    /* depending on robot and mobiel torch position */
    robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* 3 init points depending on robot position */

    /* top */
    temp_x = -(MOBIL_TORCH_X_OFFSET*K_DIST_SAFE);
    temp_y = (MOBIL_TORCH_Y_OFFSET*K_DIST_SAFE);

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
        rotate(&temp_x, &temp_y, RAD(-120));

    /* bottom-left */
    else if ((robot_y < y) && (robot_x < x))
        rotate(&temp_x, &temp_y, RAD(120));

    x += temp_x;
    y += temp_y;

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG,
                                              I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_TOP);

	x1 = x;
	y2 = y;

 	x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

   /* top */
    temp_x = -MOBIL_TORCH_X_OFFSET;
    temp_y = MOBIL_TORCH_Y_OFFSET;

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
        rotate(&temp_x, &temp_y, RAD(-120));

    /* bottom-left */
    else if ((robot_y < y) && (robot_x < x))
        rotate(&temp_x, &temp_y, RAD(120));

    x += temp_x;
    y += temp_y;

    /* go near */
    err = goto_and_avoid_forward (x1, y2, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

 

	trajectory_goto_forward_xy_abs(&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	   
	/* turn to infront of torch */
    /* top */
	a_abs = 270;

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
		a_abs = 120;
    /* bottom-left */
    else if ((robot_y < y) && (robot_x < x))
		a_abs = 30;

	printf ("a_abs = %d\n\f", a_abs);

	trajectory_a_abs (&mainboard.traj, a_abs);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

end:
    /* TODO XXX if error put arm in safe position */
	return err;
}


/* pickup mobile torch fires */
static uint8_t __strat_pickup_mobile_torch (uint8_t zone_num, uint8_t level)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
	uint16_t old_spdd, old_spda;
    int16_t x, y, robot_x, robot_y;
	uint8_t color;
	uint16_t a_abs;
	//double a_rel_rad, d_rel, d;
    double temp_x, temp_y;

    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);

	/* go to pickup position */
#if 0
    /* top */
    temp_x = -MOBIL_TORCH_X_OFFSET;
    temp_y = MOBIL_TORCH_Y_OFFSET;

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
        rotate(&temp_x, &temp_y, RAD(-120));

    /* bottom-left */
    else if ((robot_y < y) && (robot_x < x))
        rotate(&temp_x, &temp_y, RAD(120));

    x += temp_x;
    y += temp_y;

	trajectory_goto_xy_abs(&mainboard.traj, x, y);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	   
	/* turn to infront of torch */
    /* top */
	a_abs = 270;

    /* bottom-right */
    if ((robot_y < y) && (robot_x > x))
		a_abs = 120;
    /* bottom-left */
    else if ((robot_y < y) && (robot_x < x))
		a_abs = 30;

	printf ("a_abs = %d\n\f", a_abs);

	trajectory_a_abs (&mainboard.traj, a_abs);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
#endif
#if 0
    /* go over fires if we are not yet */
	abs_xy_to_rel_da(x, y, &d_rel, &a_rel_rad);
	d = d_rel * cos(a_rel_rad);

    if (ABS((int16_t)d - MOBIL_TORCH_Y_OFFSET) > 30) 
    {
        trajectory_d_rel(&mainboard.traj, 
                        ABS((int16_t)d - MOBIL_TORCH_Y_OFFSET));
        err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
           ERROUT(err);    
    }
#endif

    /* ready for pickup */
    i2c_slavedspic_wait_ready();
    i2c_slavedspic_mode_ready_for_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG,
                                              level);

    /* pickup fire */
    i2c_slavedspic_mode_pickup_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG, level);
    i2c_slavedspic_wait_ready();

    /* XXX set color of fire */
    if (zone_num == ZONE_M_TORCH_1) 
    {
        if (level == I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MIDDLE) 
             color = COLOR_INVERT(I2C_COLOR_RED);
        else color = COLOR_INVERT(I2C_COLOR_YELLOW);
    }
    else {
        if (level == I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MIDDLE) 
             color = COLOR_INVERT(I2C_COLOR_YELLOW);
        else color = COLOR_INVERT(I2C_COLOR_RED);
    }

    mainboard.stored_fire_color[slavedspic.nb_stored_fires+1] = color;

	/* store fire */
    i2c_slavedspic_mode_store_fire(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);
	i2c_slavedspic_wait_ready();

end:
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
    /* TODO XXX if error put arm in safe position */
}

/* pickup mobile torch, top fire */
inline uint8_t strat_pickup_mobile_torch_top (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_TOP);
}

/* pickup mobile torch, middle fire */
inline uint8_t strat_pickup_mobile_torch_mid (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MIDDLE);
}

/* pickup mobile torch, bottom fire */
inline uint8_t strat_pickup_mobile_torch_bot (uint8_t zone_num) {
    return __strat_pickup_mobile_torch (zone_num, I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_DOWN);
}


/* goto heart of fire */
uint8_t strat_goto_heart_fire (uint8_t zone_num)
{
    int16_t robot_x, robot_y, x, y;
	double temp_x, temp_y;
    uint8_t err;

#define HEART_FIRE_OFFSET_X 290
#define HEART_FIRE_OFFSET_Y 425

    /* depending on robot and mobiel torch position */
    robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
    x = COLOR_X(strat_infos.zones[zone_num].x);
    y = strat_infos.zones[zone_num].y;

    /* color inversion */
    if (mainboard.our_color == I2C_COLOR_RED) {
        if (zone_num == ZONE_HEART_1)
            zone_num = ZONE_HEART_3;
        else if (zone_num == ZONE_HEART_3)
            zone_num = ZONE_HEART_1;
       	else if (zone_num == ZONE_HEART_2_RIGHT)
            zone_num = ZONE_HEART_2_LEFT;
       	else if (zone_num == ZONE_HEART_2_LEFT)
            zone_num = ZONE_HEART_2_RIGHT;
    }

    /* corner hearts */
    if (zone_num == ZONE_HEART_1) {
        x = HEART_FIRE_OFFSET_X;
        y = AREA_Y - HEART_FIRE_OFFSET_Y;
    }
    else if (zone_num == ZONE_HEART_3) {
        x = AREA_X - HEART_FIRE_OFFSET_Y;
        y = AREA_Y - HEART_FIRE_OFFSET_X;
    }
    /* central heart */
    else {
        temp_x = HEART_FIRE_OFFSET_X;
        temp_y = -HEART_FIRE_OFFSET_Y;
        
        if (zone_num == ZONE_HEART_2_UP)
            rotate(&temp_x, &temp_y, RAD(135));
        else if (zone_num == ZONE_HEART_2_DOWN)
            rotate(&temp_x, &temp_y, RAD(-45));
        else if (zone_num == ZONE_HEART_2_RIGHT)
            rotate(&temp_x, &temp_y, RAD(45));
        else if (zone_num == ZONE_HEART_2_LEFT)
            rotate(&temp_x, &temp_y, RAD(-135));

        x = HEART_2_X + temp_x;
        y = HEART_2_Y + temp_y;
    }

    /* go near */
    err = goto_and_avoid_forward (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);

    /* TODO XXX if error put arm in safe position */
	return err;
}

/* dump stored fires on heart of fire making a puzzle */
uint8_t strat_make_puzzle_on_heart (uint8_t zone_num)
{
//#define DEBUG_STRAT_FIRES
#ifdef DEBUG_STRAT_FIRES
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0, i;
	uint16_t old_spdd, old_spda;
    int16_t a_abs;
	uint8_t sucker_type;
    int16_t arm_a, arm_y;

	int16_t fire_x[5] = 	{+55,	+30, 	-35, 	+162,   +80	};
	int8_t  fire_a[5] = 	{-20, 	+45, 	+10, 	+15,    -35 };
	int16_t fire_d_fw[5] = 	{+90,	+50,	+95,	+30,	+100};
	int16_t fire_d_bw[5] = 	{-90,	-50,	-125, 	-130,	-100};


	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_VERY_SLOW,SPEED_ANGLE_FAST);
   
    /* color inversion */
    if (mainboard.our_color == I2C_COLOR_RED) {
        if (zone_num == ZONE_HEART_1)
            zone_num = ZONE_HEART_3;
        else if (zone_num == ZONE_HEART_3)
            zone_num = ZONE_HEART_1;
       	else if (zone_num == ZONE_HEART_2_RIGHT)
            zone_num = ZONE_HEART_2_LEFT;
       	else if (zone_num == ZONE_HEART_2_LEFT)
            zone_num = ZONE_HEART_2_RIGHT;
    }

	/* set proper angle */
	if (zone_num == ZONE_HEART_1)
		a_abs = 135;
	else if (zone_num == ZONE_HEART_3)
		a_abs = 45;
    else
        a_abs = 90;


	trajectory_a_abs (&mainboard.traj, a_abs);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);


	i2c_slavedspic_wait_ready();

	/* make puzzle :) */
	for (i=0; i<5; i++) 
	{	
		if (i==0) {
			/* load 1st fire */
			if (slavedspic.nb_stored_fires > 6)
				 sucker_type = I2C_SLAVEDSPIC_SUCKER_TYPE_SHORT;
			else sucker_type = I2C_SLAVEDSPIC_SUCKER_TYPE_LONG;

			i2c_slavedspic_mode_load_fire(sucker_type);
		}
		i2c_slavedspic_wait_ready();

		/* putdown */
		i2c_slavedspic_mode_putdown_fire (sucker_type, 
										  I2C_SLAVEDSPIC_LEVEL_FIRE_HEART, 
										  fire_x[i], &arm_y, &arm_a, fire_a[i]);
		i2c_slavedspic_wait_ready();

		/* go forward */
	   	trajectory_d_rel(&mainboard.traj, fire_d_fw[i]);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//   ERROUT(err);

		/* release */
		i2c_slavedspic_mode_release_fire (sucker_type);
		i2c_slavedspic_wait_ready();

		/* load fire */
		i2c_slavedspic_mode_load_fire(sucker_type);

		/* go backward */
retry_bw:
	   	trajectory_d_rel(&mainboard.traj, fire_d_bw[i]);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (err & END_BLOCKING)
			goto retry_bw;
		//if (!TRAJ_SUCCESS(err))
		//   ERROUT(err);

		DEBUG (E_USER_STRAT, "fire %d", i);
	}

	/* hide arm */
	i2c_slavedspic_mode_hide_arm(I2C_SLAVEDSPIC_SUCKER_TYPE_LONG);

end:
    /* TODO XXX if error put arm in safe position */
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}

#if 0
uint8_t strat_harvest_mobile_torch (uint8_t zone_num) 
{
	uint8_t flags;

	/* pickup top fire */
	IRQ_LOCK(flags);
	nb_stored_fires_saved = mainboard.nb_stored_fires;
	IRQ_UNLOCK(flags);
		
	err = strat_pickup_mobile_torch_top (zone_num);
	i2c_slavedspic_wait_ready();
	i2c_slavedspic_wait_ready();

	if (manboard.nb_stored_fires <= nb_stored_fires_saved)
	

}
#endif



