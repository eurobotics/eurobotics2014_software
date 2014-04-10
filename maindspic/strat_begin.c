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



uint8_t strat_begin(void)
{   
	uint8_t err;
	uint8_t zone_num=0;
	#define ZONES_SEQUENCE_LENGTH 6
	uint8_t zones_sequence[ZONES_SEQUENCE_LENGTH] = 						
	{ZONE_TORCH_1,ZONE_FIRE_1,ZONE_FIRE_2,ZONE_MOBILE_TORCH_1,ZONE_FIRE_3,ZONE_TORCH_3};
	
	if(mainboard.our_color==I2C_COLOR_RED)
	{
		zones_sequence[0]=ZONE_TORCH_4; zones_sequence[1]=ZONE_FIRE_6; zones_sequence[2]=ZONE_FIRE_4; 
		zones_sequence[3]=ZONE_MOBILE_TORCH_2; zones_sequence[4]=ZONE_FIRE_5; zones_sequence[5]=ZONE_TORCH_2; 
	}
	
	for(zone_num=0; zone_num<ZONES_SEQUENCE_LENGTH; zone_num++)
	{
		/* goto zone */
		printf_P(PSTR("Going to zone %s.\r\n"),numzone2name[zones_sequence[zone_num]]);
		strat_dump_infos(__FUNCTION__);
		strat_infos.current_zone=-1;
		strat_infos.goto_zone=zones_sequence[zone_num];
		err = goto_and_avoid(strat_infos.zones[zones_sequence[zone_num]].init_x, strat_infos.zones[zones_sequence[zone_num]].init_y,  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		trajectory_a_abs(&mainboard.traj, strat_infos.zones[zones_sequence[zone_num]].init_a);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		strat_infos.last_zone=strat_infos.current_zone;
		strat_infos.goto_zone=-1;
		if (!TRAJ_SUCCESS(err)) {
			strat_infos.current_zone=-1;
			printf_P(PSTR("Can't reach zone %s.\r\n"),numzone2name[zones_sequence[zone_num]]);
		}
		else{
			strat_infos.current_zone=zones_sequence[zone_num];
		}

		/* work on zone */
		strat_dump_infos(__FUNCTION__);
		err = strat_work_on_zone(zones_sequence[zone_num]);
		if (!TRAJ_SUCCESS(err)) {
			printf_P(PSTR("Work on zone %s fails.\r\n"),numzone2name[zones_sequence[zone_num]]);
		}
		else
		{
			// Switch off devices, go back to normal state if anything was deployed
		}

		/* mark the zone as checked */
		strat_infos.zones[zones_sequence[zone_num]].flags |= ZONE_CHECKED;
		printf_P(PSTR("Work on zone %s succeeded!\r\n"),numzone2name[zones_sequence[zone_num]]);
	}
	return err;
}

#define DEBUG_BEGIN
#ifdef DEBUG_BEGIN 
#define wait_press_key() state_debug_wait_key_pressed();
#else
#define wait_press_key()
#endif


/* begin alcabot */
uint8_t strat_begin_alcabot (void)
{
   uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
   int16_t d;
	static uint8_t state = 0;

	strat_infos.debug_step = 0;
 
	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   strat_limit_speed_enable ();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);
   

	wait_press_key();

   switch(state)
   {
		/* torch 1 */
      case 0:
#define TORCH_1_D_STICK 340
#define TORCH_1_A_STICK -80

		d = TORCH_1_Y - position_get_y_s16(&mainboard.pos) - TORCH_1_D_STICK;
		trajectory_d_rel(&mainboard.traj, d);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);				

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(TORCH_1_A_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		strat_infos.zones[ZONE_TORCH_1].flags |= ZONE_CHECKED;

		state ++;	
		break;			

		/* fire 2 */
      case 1:
#define FIRE_2_Y_STICK 			860
#define FIRE_2_A_ABS_STICK		90


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (FIRE_1_X + (MOBILE_TORCH_1_X-FIRE_1_X)/2),
										FIRE_2_Y_STICK);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(FIRE_2_A_ABS_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		strat_infos.zones[ZONE_FIRE_2].flags |= ZONE_CHECKED;

		state ++;
		break;

		/* fire 1 */
      case 2:
#define FIRE_1_Y_STICK 
		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (FIRE_1_X+(MOBILE_TORCH_1_X-FIRE_1_X)/2),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_1].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* push m torch 1 */
      case 3:
#define M_TORCH_ABS_PUSH 		0
#define M_TORCH_X_PUSH			1250
#define M_TORCH_1_SPEED_DIST	1250
		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(M_TORCH_ABS_PUSH));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_CLEAN_HEART, 0);
		i2c_slavedspic_wait_ready();

		strat_get_speed(&temp_spdd, &temp_spda);
		strat_set_speed(M_TORCH_1_SPEED_DIST,SPEED_ANGLE_SLOW);

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (M_TORCH_X_PUSH),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		strat_set_speed(temp_spdd,temp_spda);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		time_wait_ms (200);

		state ++;
		break;

		/* fire fire 6 */
      case 4:
#define TORCH_2_D_MARGIN 	50
#define FIRE_6_A_REL_STICK	-140

		/* avoid heart fire 2 */
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X(CENTER_X),
		HEART_FIRE_2_Y + HEART_FIRE_2_RAD +  ROBOT_WIDTH/2 + TORCH_2_D_MARGIN);
		
		err = wait_traj_end(TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		/* goto near fire 6 */
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X(MOBILE_TORCH_2_X+(FIRE_6_X-MOBILE_TORCH_2_X)/2),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(FIRE_6_A_REL_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_6].flags |= ZONE_CHECKED;
		state ++;
		break;
		
		/* fire 5 */
      case 5:
#define FIRE_5_D_STICK	75
		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(FIRE_5_X - FIRE_5_D_STICK),
								MOBILE_TORCH_1_Y + (FIRE_3_Y-MOBILE_TORCH_1_Y)/2);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_5].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* push m toch 2 */
      case 6:
		state ++;
		break;

		/* torch 3 */
      case 7:
#define TORCH_3_X_STICK			(TORCH_3_X - 270)
#define TORCH_3_Y_STICK			(AREA_Y - 200)
#define TORCH_3_A_ABS_STICK	(-45)
#define TORCH_3_A_REL_STICK 	(-30)

		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(TORCH_3_X_STICK),
								TORCH_3_Y_STICK);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		//trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(TORCH_3_A_ABS_STICK));
		//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//		ERROUT(err);

		//i2c_slavedspic_wait_ready();

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		wait_ms (100);

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(TORCH_3_A_REL_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_TORCH_3].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* fire 3 */
      case 8:
#define FIRE_3_TIMEOUT	1000
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
								COLOR_X(MOBILE_TORCH_1_X),
								MOBILE_TORCH_1_Y);


		WAIT_COND_OR_TIMEOUT(!x_is_more_than(1400),
									FIRE_3_TIMEOUT);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_3].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* m torch 1*/
      case 99:
#define M_TORCH_1_Y_STICK 	(AREA_Y - 600)
#define M_TORCH_X_STICK		(CENTER_X - 480)


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(CENTER_X - 440),
								AREA_Y - 515);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);


		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(180));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_TORCH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		wait_ms (200);


		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(CENTER_X-150),
								AREA_Y - 550);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_MOBILE_TORCH_1].flags |= ZONE_CHECKED;
		state ++;
		break;
		
      default:
			i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
	 											 I2C_STICK_MODE_HIDE, 0);
			i2c_slavedspic_wait_ready();
	
			i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
	 											 I2C_STICK_MODE_HIDE, 0);
			i2c_slavedspic_wait_ready();
			err = END_RESERVED;
         break;
   }

end:

	if (err & END_OBSTACLE)
		time_wait_ms (2100);
	
	if (err & END_BLOCKING) {

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		err = END_RESERVED;
	}

	strat_set_speed(old_spdd, old_spda);	
   strat_limit_speed_enable();
   return err;
}





/***********************************************************************/

#if 0
/* begin alcabot */
uint8_t strat_begin_alcabot_v2 (void)
{
   uint8_t err = 0;
	uint16_t old_spdd, old_spda, temp_spdd, temp_spda;
   int16_t d;
	static uint8_t state = 0;

	strat_infos.debug_step = 0;
 
	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   strat_limit_speed_enable ();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);
   

	wait_press_key();

   switch(state)
   {
		/* torch 1 */
      case 0:
#define TORCH_1_D_STICK 340
#define TORCH_1_A_STICK -80

		d = TORCH_1_Y - position_get_y_s16(&mainboard.pos) - TORCH_1_D_STICK;
		trajectory_d_rel(&mainboard.traj, d);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);				

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(TORCH_1_A_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		strat_infos.zones[ZONE_TORCH_1].flags |= ZONE_CHECKED;

		state ++;	
		break;			

		/* fire 2 */
      case 1:
#define FIRE_2_Y_STICK 			860
#define FIRE_2_A_ABS_STICK		90


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (FIRE_1_X + (MOBILE_TORCH_1_X-FIRE_1_X)/2),
										FIRE_2_Y_STICK);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(FIRE_2_A_ABS_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
		strat_infos.zones[ZONE_FIRE_2].flags |= ZONE_CHECKED;

		state ++;
		break;

		/* fire 1 */
      case 2:
#define FIRE_1_Y_STICK 
		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (FIRE_1_X+(MOBILE_TORCH_1_X-FIRE_1_X)/2),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_1].flags |= ZONE_CHECKED;
		state ++;
		break;


		/* push m torch 1 */
      case 33:
#define M_TORCH_ABS_PUSH 		0
#define M_TORCH_X_PUSH			1250
#define M_TORCH_1_SPEED_DIST	1250
		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(M_TORCH_ABS_PUSH));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_CLEAN_HEART, 0);
		i2c_slavedspic_wait_ready();

		strat_get_speed(&temp_spdd, &temp_spda);
		strat_set_speed(M_TORCH_1_SPEED_DIST,SPEED_ANGLE_SLOW);

		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X (M_TORCH_X_PUSH),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		strat_set_speed(temp_spdd,temp_spda);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		time_wait_ms (200);

		state ++;
		break;

		/* fire fire 6 */
      case 4:
#define TORCH_2_D_MARGIN 	50
#define FIRE_6_A_REL_STICK	-140

		/* avoid heart fire 2 */
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X(CENTER_X),
		HEART_FIRE_2_Y + HEART_FIRE_2_RAD +  ROBOT_WIDTH/2 + TORCH_2_D_MARGIN);
		
		err = wait_traj_end(TRAJ_FLAGS_STD);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		/* goto near fire 6 */
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
										COLOR_X(MOBILE_TORCH_2_X+(FIRE_6_X-MOBILE_TORCH_2_X)/2),
										MOBILE_TORCH_1_Y+(FIRE_3_Y-MOBILE_TORCH_1_Y)/2);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(FIRE_6_A_REL_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_6].flags |= ZONE_CHECKED;
		state ++;
		break;
		
		/* fire 5 */
      case 5:
#define FIRE_5_D_STICK	75
		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(FIRE_5_X - FIRE_5_D_STICK),
								MOBILE_TORCH_1_Y + (FIRE_3_Y-MOBILE_TORCH_1_Y)/2);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_5].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* push m toch 2 */
      case 6:
		state ++;
		break;

		/* torch 3 */
      case 7:
#define TORCH_3_X_STICK			(TORCH_3_X - 270)
#define TORCH_3_Y_STICK			(AREA_Y - 200)
#define TORCH_3_A_ABS_STICK	(-45)
#define TORCH_3_A_REL_STICK 	(-30)

		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(TORCH_3_X_STICK),
								TORCH_3_Y_STICK);	
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		//trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(TORCH_3_A_ABS_STICK));
		//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		//if (!TRAJ_SUCCESS(err))
		//		ERROUT(err);

		//i2c_slavedspic_wait_ready();

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		wait_ms (100);

		trajectory_a_rel (&mainboard.traj, COLOR_A_REL(TORCH_3_A_REL_STICK));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_TORCH_3].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* fire 3 */
      case 8:
#define FIRE_3_TIMEOUT	1000
		trajectory_goto_forward_xy_abs (&mainboard.traj, 
								COLOR_X(MOBILE_TORCH_1_X),
								MOBILE_TORCH_1_Y);


		WAIT_COND_OR_TIMEOUT(!x_is_more_than(1400),
									FIRE_3_TIMEOUT);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_PUSH_FIRE, 0);

		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_FIRE_3].flags |= ZONE_CHECKED;
		state ++;
		break;

		/* m torch 1*/
      case 9:
#define M_TORCH_1_Y_STICK 	(AREA_Y - 600)
#define M_TORCH_X_STICK		(CENTER_X - 480)


		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(CENTER_X - 440),
								AREA_Y - 550);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);


		trajectory_a_abs (&mainboard.traj, COLOR_A_ABS(180));
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_PUSH_TORCH_FIRE, 0);
		i2c_slavedspic_wait_ready();

		wait_ms (200);


		trajectory_goto_backward_xy_abs (&mainboard.traj, 
								COLOR_X(CENTER_X-150),
								AREA_Y - 550);
		err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		strat_infos.zones[ZONE_MOBILE_TORCH_1].flags |= ZONE_CHECKED;
		state ++;
		break;
		
      default:
			i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
	 											 I2C_STICK_MODE_HIDE, 0);
			i2c_slavedspic_wait_ready();
	
			i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
	 											 I2C_STICK_MODE_HIDE, 0);
			i2c_slavedspic_wait_ready();
			err = END_RESERVED;
         break;
   }

end:

	if (err & END_OBSTACLE)
		time_wait_ms (2100);
	
	if (err & END_BLOCKING) {

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_LEFT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		i2c_slavedspic_mode_stick ( COLOR_INVERT(I2C_STICK_TYPE_RIGHT),
 											 I2C_STICK_MODE_HIDE, 0);
		i2c_slavedspic_wait_ready();

		err = END_RESERVED;
	}

	strat_set_speed(old_spdd, old_spda);	
   strat_limit_speed_enable();
   return err;
}
#endif









