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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano and Miguel Ortiz
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


/* 
 * decrease gain on angle PID, and go forward until we reach the
 * border.
 */
 
//#if 0
uint8_t strat_calib_tree(int16_t dist, uint8_t flags)
{
	int32_t p = pid_get_gain_P(&mainboard.angle.pid);
	int32_t i = pid_get_gain_I(&mainboard.angle.pid);
	int32_t d = pid_get_gain_D(&mainboard.angle.pid);
	uint8_t err;

	bd_set_current_thresholds(&mainboard.distance.bd, 100, 2000, 1000000, 10); //20, 8000, 1000000, 50);

	pid_set_gains(&mainboard.angle.pid, 50, 0, 1000);
	trajectory_d_rel(&mainboard.traj, dist);
    err = WAIT_COND_OR_TRAJ_END(sensor_get (S_TREE_TOP), flags);
    strat_hardstop();
	pid_set_gains(&mainboard.angle.pid, p, i, d);

	bd_set_current_thresholds(&mainboard.distance.bd, 100, 2000, 1000000, 25); //20, 8000, 1000000, 50);
	return err;
}

/* harvest fruits from trees */
uint8_t strat_harvest_fruits(int16_t x, int16_t y, uint8_t clean_before)
{
//#define DEBUG_STRAT_HARVEST_FRUITS
#ifdef DEBUG_STRAT_HARVEST_FRUITS 
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif

    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d, clean_floor_a_rel;
	uint8_t stick_type;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   
    /* clean floor if requested */
    if (clean_before)
    {
	    /* depending on tree type */

	    /* tree 1 */
	    if (position_get_x_s16(&mainboard.pos) < TREE_2_X) {
		    stick_type = I2C_STICK_TYPE_RIGHT;
		    clean_floor_a_rel = 180; 
	    }
	    /* tree 4 */
	    else 	if (position_get_x_s16(&mainboard.pos) > TREE_3_X) {
		    stick_type = I2C_STICK_TYPE_LEFT;
		    clean_floor_a_rel = -180; 
	    }

	    /* tree 2 */
	    else if (position_get_x_s16(&mainboard.pos) < CENTER_X
				    && position_get_y_s16(&mainboard.pos) > TREE_1_Y) {
		    stick_type = I2C_STICK_TYPE_LEFT;
		    clean_floor_a_rel = -180; 
	    }
	    /* tree 3 */	
	    else { /* if (position_get_x_s16(&mainboard.pos) > CENTER_X
				    && position_get_y_s16(&mainboard.pos) > TREE_4_Y) */
		    stick_type = I2C_STICK_TYPE_RIGHT;
		    clean_floor_a_rel = 180; 
	    }

	    /* turn in front of tree with stick deployed */
        i2c_slavedspic_wait_ready();
	    i2c_slavedspic_mode_stick (stick_type, I2C_STICK_MODE_CLEAN_FLOOR, 0);

        /* turn to infront of tree */
	    trajectory_turnto_xy (&mainboard.traj, x, y);
	    err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	    if (!TRAJ_SUCCESS(err))
		    ERROUT(err);

 	    /* XXX somtimes the stick doesn't reach the possition */
	    i2c_slavedspic_wait_ready();
	    //time_wait_ms (100);

	    wait_press_key();

	    /* clean floor */
	    trajectory_a_rel (&mainboard.traj, clean_floor_a_rel);
	    err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	    if (!TRAJ_SUCCESS(err))
			    ERROUT(err);

	    i2c_slavedspic_mode_stick ( stick_type,	 I2C_STICK_MODE_HIDE, 0);
	    i2cproto_wait_update ();
	    i2c_slavedspic_mode_stick ( 
         (stick_type == I2C_STICK_TYPE_RIGHT? I2C_STICK_TYPE_LEFT:I2C_STICK_TYPE_RIGHT),
     	 I2C_STICK_MODE_HIDE, 0);

	    //i2cproto_wait_update ();
	    wait_press_key();
    }

	/* prepare for for harvesting */
#define HARVEST_TREE_D_NEAR		(350)
#define HARVEST_TREE_SPEED_DIST	(500)
#define HARVEST_TREE_D_CLOSE	 	(200)
#define HARVEST_TREE_D_FAR			(500-160)
#define HARVEST_TREE_D_BLOCKING	(250)

    /* open back system */
	i2c_slavedspic_wait_ready();
	i2c_slavedspic_mode_harvest_fruits(I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_READY);
	wait_press_key();

    /* turn to behind tree */
    trajectory_turnto_xy (&mainboard.traj, x, y);
    err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	    ERROUT(err);	

    /* center to trunk if it's necessary */
    if (!sensor_get (S_TREE_TRUNK))
    {
	    trajectory_a_rel (&mainboard.traj, 15);
	    err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	    if (!TRAJ_SUCCESS(err))
			    ERROUT(err);

        /* set slow angle speed */
        strat_get_speed (&temp_spdd, &temp_spda);
	    strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_VERY_SLOW);

	    trajectory_a_rel (&mainboard.traj, -30);

        /* return 0 if cond become true */
        err = WAIT_COND_OR_TRAJ_END(sensor_get (S_TREE_TRUNK), TRAJ_FLAGS_SMALL_DIST);
        strat_hardstop();
        
        /* restore speed */
        strat_set_speed (temp_spdd,temp_spda);

        /* if not sensor detected, it's supposed the tree is centered */
        if (err != 0) {
            /* turn to behind tree */
            trajectory_turnto_xy (&mainboard.traj, x, y);
            err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
            if (!TRAJ_SUCCESS(err))
	            ERROUT(err);	
        }  
    }


	/* go to near tree */
	d = distance_from_robot(x,y);
	trajectory_d_rel(&mainboard.traj, -(d - HARVEST_TREE_D_NEAR));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);


	/* XXX go to near truck, ends with sensor detection or blocking */
	strat_get_speed (&temp_spdd, &temp_spda);
	strat_set_speed (HARVEST_TREE_SPEED_DIST, temp_spda);

    /* return 0 if cond become true */
	err = strat_calib_tree(-HARVEST_TREE_D_BLOCKING, TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed( temp_spdd, temp_spda);

    /* check stop condition */
    if (err != 0) {
        /* no sensor detecton, ends in blocking, go backward a bit */
	    trajectory_d_rel(&mainboard.traj, 10);
	    err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
        if (!TRAJ_SUCCESS(err))
	       ERROUT(err);

        /* we just blocking wit truck, wait for fruit stabilization */	
        time_wait_ms (300);
    }

	/* should stay very close to tree, not centering blocking case */
	if (distance_from_robot(x,y) > HARVEST_TREE_D_CLOSE)
		goto end_harvesting;

	wait_press_key();

	/* pick up the fruits and go backward */
	i2c_slavedspic_mode_harvest_fruits(I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_DO);
	i2c_slavedspic_wait_ready();

	wait_press_key();

end_harvesting:

	/* check if opponent is behind and harvest fruits */
	strat_set_speed (HARVEST_TREE_SPEED_DIST, temp_spda);

    /* check opponent behind */
	if (opponent1_is_infront() || opponent2_is_infront()) {
        time_wait_ms (2000);

        if (opponent1_is_infront() || opponent2_is_infront())
		    strat_set_speed (SPEED_DIST_SLOW, SPEED_ANGLE_VERY_SLOW);

        strat_infos.tree_harvesting_interrumped = 1;
	}

    /* return to init position */
	trajectory_d_rel(&mainboard.traj, HARVEST_TREE_D_FAR);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);

	/* wait for fruits fall into */
	time_wait_ms (250);


	/* hide tools */
end:
	i2c_slavedspic_mode_harvest_fruits (I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_END);
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}
//#endif


/* leave fruits from trees on basket*/
/* TODO two different positions in basket */

uint8_t strat_leave_fruits(void)
{
#ifdef DEBUG_STRAT_HARVEST_FRUITS 
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif
#define BASKET_1_CENTER_X	750
#define BASKET_2_CENTER_X	2250
    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);

	/* turn */
	trajectory_a_abs(&mainboard.traj, 90);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	
	/* go near basket */
	d = distance_from_robot(position_get_x_s16(&mainboard.pos), 300);
	trajectory_d_rel(&mainboard.traj, -(d - 150));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
    if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* go backwards until blocking */
	strat_get_speed (&temp_spdd, &temp_spda);
	strat_set_speed (500, temp_spda);
	strat_calib(-100, TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed( temp_spdd, temp_spda);

	/* dump fruits do */
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO);
	i2c_slavedspic_wait_ready();
	time_wait_ms (2000);


	/* go forward */
	trajectory_d_rel(&mainboard.traj, 80);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* dump fruits end */
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END);
	i2c_slavedspic_wait_ready();


	/* go backwards until blocking */
	strat_get_speed (&temp_spdd, &temp_spda);
	strat_set_speed (500, temp_spda);
	strat_calib(-100, TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed( temp_spdd, temp_spda);

	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO);
	i2c_slavedspic_wait_ready();

	trajectory_d_rel(&mainboard.traj, 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);


end:
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END);
	i2c_slavedspic_wait_ready();

	strat_set_speed(old_spdd, old_spda);
   strat_limit_speed_enable();
	return err;
}




/* leave fruits from trees on basket*/
/* TODO two different positions in basket */

uint8_t strat_leave_fruits_clean(void)
{
#ifdef DEBUG_STRAT_HARVEST_FRUITS 
#define wait_press_key() state_debug_wait_key_pressed();
	strat_infos.debug_step = 1;
#else
#define wait_press_key()
#endif
#define BASKET_1_CENTER_X	750
#define BASKET_2_CENTER_X	2250
    uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
    int16_t d, clean_floor_a_abs,clean_floor_a_rel;
	uint8_t stick_type;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);

	/* depending on basket type */

	/* basket 1 left */
	if (position_get_x_s16(&mainboard.pos) < BASKET_1_CENTER_X) {
		stick_type = I2C_STICK_TYPE_LEFT;
		clean_floor_a_abs = 180+20; 
		clean_floor_a_rel = -90-20;
	}
	/* basket 1 right */
	else 	if(position_get_x_s16(&mainboard.pos) > BASKET_1_CENTER_X
				&& position_get_x_s16(&mainboard.pos) < CENTER_X){
		stick_type = I2C_STICK_TYPE_RIGHT;
		clean_floor_a_abs = 0-20; 
		clean_floor_a_rel = 90+20;
	}

	/* basket 2 left */
	else if (position_get_x_s16(&mainboard.pos) < BASKET_2_CENTER_X
				&& position_get_x_s16(&mainboard.pos) > CENTER_X) {
		stick_type = I2C_STICK_TYPE_LEFT;
		clean_floor_a_abs = 180+20; 
		clean_floor_a_rel = -90-20;
	}
	/* basket 2 right */	
	else { /* if (position_get_x_s16(&mainboard.pos) > CENTER_X
				&& position_get_y_s16(&mainboard.pos) > BASKET_2_CENTER_X) */
		stick_type = I2C_STICK_TYPE_RIGHT;
		clean_floor_a_abs = 0-20; 
		clean_floor_a_rel = 90+20;
	}


	/* clean floor */
	trajectory_a_abs (&mainboard.traj, clean_floor_a_abs);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* turn in front of basket with stick deployed */
	i2c_slavedspic_mode_stick (stick_type,
 										I2C_STICK_MODE_CLEAN_FLOOR, 0);
	i2c_slavedspic_wait_ready();

	trajectory_a_rel (&mainboard.traj, clean_floor_a_rel);
	err = wait_traj_end (TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	i2c_slavedspic_mode_stick ( I2C_STICK_TYPE_LEFT,
 										 I2C_STICK_MODE_HIDE, 0);
	i2c_slavedspic_wait_ready();
	i2c_slavedspic_mode_stick ( I2C_STICK_TYPE_RIGHT,
 										 I2C_STICK_MODE_HIDE, 0);
	i2c_slavedspic_wait_ready();


	/* go near basket */
	d = distance_from_robot(position_get_x_s16(&mainboard.pos), 300);
	trajectory_d_rel(&mainboard.traj, -(d - 150));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* go backwards until blocking */
	strat_get_speed (&temp_spdd, &temp_spda);
	strat_set_speed (500, temp_spda);
	strat_calib(-100, TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed( temp_spdd, temp_spda);

	/* dump fruits do */
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO);
	i2c_slavedspic_wait_ready();
	time_wait_ms (2000);


	/* go forward */
	trajectory_d_rel(&mainboard.traj, 80);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);

	/* dump fruits end */
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END);
	i2c_slavedspic_wait_ready();


	/* go backwards until blocking */
	strat_get_speed (&temp_spdd, &temp_spda);
	strat_set_speed (500, temp_spda);
	strat_calib(-100, TRAJ_FLAGS_SMALL_DIST);
	strat_set_speed( temp_spdd, temp_spda);

	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO);
	i2c_slavedspic_wait_ready();

	trajectory_d_rel(&mainboard.traj, 250);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);


end:
	i2c_slavedspic_mode_dump_fruits(I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END);
	i2c_slavedspic_wait_ready();

	strat_set_speed(old_spdd, old_spda);
   strat_limit_speed_enable();
	return err;
}






