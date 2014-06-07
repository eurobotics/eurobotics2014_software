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
#include "../common/bt_commands.h"

#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "../maindspic/strat_avoid.h"
#include "../maindspic/strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)

/* Add here the main strategic, the inteligence of robot */


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


uint8_t strat_patrol_fresco_mamooth(uint8_t balls_mamooth_1, uint8_t balls_mamooth_2)
{
	#define BEGIN_FRESCO_X	1295
	#define REFERENCE_DISTANCE 600
	static uint8_t fresco_done =0;
	static uint8_t mamooth_done=0;
	static uint8_t initialized=0;
	int16_t d1,d2;
	int16_t a1,a2;
	int16_t opp1_x, opp1_y, opp2_x, opp2_y;
	
	get_opponent1_da(&d1,&a1);
	get_opponent2_da(&d2,&a2);
	
	if(initialized==0)
	{
        blade_push_fire ();
		strat_initial_move();
		initialized=1;
		printf_P("initialized");
	}
		
	if(((d2>REFERENCE_DISTANCE)||(!opp2_x_is_more_than(BEGIN_FRESCO_X))) && ((d1>REFERENCE_DISTANCE) || (!opp1_x_is_more_than(BEGIN_FRESCO_X))))
	{
		if(fresco_done!=1)
		{
			fresco_done=strat_paint_fresco();
			return (fresco_done);
		}
		else if(mamooth_done!=1){
			mamooth_done=strat_shoot_mamooth(balls_mamooth_1,balls_mamooth_2);
			return (mamooth_done);
		}
	}
	else
	{
		get_opponent1_xy(&opp1_x, &opp1_y);
		get_opponent2_xy(&opp2_x, &opp2_y);
		if(((opp1_y<300 && opp1_y>0) || (opp2_y<300 && opp2_y>0)) && ((fresco_done!=1)||(mamooth_done!=1)))
		{
			if(fresco_done!=1)
			{
				fresco_done=strat_paint_fresco();
				return (fresco_done);
			}
			else if(mamooth_done!=1)
			{
				mamooth_done=strat_shoot_mamooth(balls_mamooth_1,balls_mamooth_2);
				return (mamooth_done);
			}
		}
			
		else if((opp1_x_is_more_than(BEGIN_FRESCO_X)) || (opp2_x_is_more_than(BEGIN_FRESCO_X)))
		{
			printf_P("PATROL\n");
			return strat_patrol_between(COLOR_X(BEGIN_FRESCO_X),300,COLOR_X(BEGIN_FRESCO_X),900);
		}
	}

    return 0;
}


void strat_initial_move(void)
{
#define BEGIN_LINE_Y 	450
#define BEGIN_FRESCO_X	1295
	static uint8_t state = 0;
	uint8_t err = 0;
	
	
	while(1)
	{
		switch(state)
		{
			/* go in front of fresco */
			case 0:
				//printf_P("init case 0\n");
				trajectory_goto_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), BEGIN_LINE_Y);
				state++;
				break;
			
			case 1:
				//printf_P("init case 1 - x: %d y: %d - opp %d\n", abs(position_get_x_double(&mainboard.pos)-COLOR_X(BEGIN_FRESCO_X)),abs(position_get_y_double(&mainboard.pos)-BEGIN_LINE_Y) , (opponent1_is_infront() || opponent2_is_infront()));
				if(opponent1_is_infront()==0 && opponent2_is_infront()==0)
				{
					//err = test_traj_end(TRAJ_FLAGS_STD);

					if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(BEGIN_FRESCO_X))<30) && (abs(position_get_y_double(&mainboard.pos)-BEGIN_LINE_Y)<30))
					{
						state ++;
					}
				}
				else
				{
					if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(BEGIN_FRESCO_X))<30) && (abs(position_get_y_double(&mainboard.pos)-BEGIN_LINE_Y)<30))
					{
						state ++;
						break;
					}
					strat_hardstop();
					state=0;
				}	
				break;
				
			case 2:
				trajectory_a_abs (&mainboard.traj, 90);
				err = wait_traj_end(TRAJ_FLAGS_STD);
				if (!TRAJ_SUCCESS(err))
						break;
				return;
				break;
			default:
				break;
		}
	}
}


uint8_t strat_paint_fresco(void)
{
	static uint8_t state = 0;
	uint16_t old_spdd, old_spda;
	//int16_t opp_d, opp_a,opp2_d,opp2_a;
	uint8_t err = 0;
#define BEGIN_LINE_Y 	450
#define BEGIN_FRESCO_X	1295
#define BEGIN_FRESCO_Y	450


	switch (state)
	{
		/* turn to fresco */
		case 0:
			//printf_P("fresco case 0");
			trajectory_a_abs (&mainboard.traj, 90);
			err = wait_traj_end(TRAJ_FLAGS_STD);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);

			state ++;
			return 0;		
			break;

		/* paint fresco */
		case 1:
			//printf_P("fresco case 1");
			sensor_obstacle_enable();
			if (sensor_get (S_OBS_REAR_L) || sensor_get (S_OBS_REAR_R))
				ERROUT(END_OBSTACLE);
				
			/* go backwards */
			trajectory_goto_backward_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);
			strat_get_speed(&old_spdd, &old_spda);
			strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);
			trajectory_d_rel(&mainboard.traj, -200);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);


			/* go forward */
			strat_set_speed(old_spdd, old_spda);
			trajectory_goto_forward_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
			err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
			if (!TRAJ_SUCCESS(err))
					ERROUT(err);
					
				
			state ++;	
			return 0;	
			break;

		/* leave fresco */
		case 2:	
			//printf_P("fresco case 2");
			if(opponent1_is_infront()==0 && opponent2_is_infront()==0)
			{
				trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(BEGIN_FRESCO_X),600);
				err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
				if (!TRAJ_SUCCESS(err))
						ERROUT(err);
						
				state ++;
				//printf_P("fresco done");
				return 1;
			}
			
			else
				return 0;
			break;
			
		default:
			break;
	}
end:
	return err;
}


uint8_t strat_patrol_between(int16_t x1, int16_t y1,int16_t x2, int16_t y2)
{	
	#define REFERENCE_DISTANCE_TO_ROBOT 800
	int16_t opp1_x, opp1_y, opp2_x, opp2_y;
	static int16_t opp1_y_saved, opp_y_saved;
	uint16_t old_spdd, old_spda;
	uint8_t err = 0;
	int16_t d1,d2,a1,a2;

	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_VERY_SLOW);
	
	/* get robot coordinates */
	get_opponent1_xy(&opp1_x, &opp1_y);
	get_opponent2_xy(&opp2_x, &opp2_y);
	get_opponent1_da(&d1,&a1);
	get_opponent2_da(&d2,&a2);
	
	if(d1<(REFERENCE_DISTANCE_TO_ROBOT) && d1>0)
	{
		if((opp1_y_is_more_than(y1)&&!opp1_y_is_more_than(y2))||(!opp1_y_is_more_than(y1)&&opp1_y_is_more_than(y2)))
		{
			//trajectory_goto_xy_abs(&mainboard.traj,(x1+x2)/2,opp1_y);
			//err = goto_and_avoid ((x1+x2)/2, opp1_y,TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
			opp_y_saved = opp1_y;
		}

		//err = goto_and_avoid ((x1+x2)/2, opp1_y_saved,TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
		
	}
	
	else if(d2<(REFERENCE_DISTANCE_TO_ROBOT) && d2>0)
	{
		if((opp2_y_is_more_than(y1)&&!opp2_y_is_more_than(y2))||(!opp2_y_is_more_than(y1)&&opp2_y_is_more_than(y2)))
		{
			//trajectory_goto_xy_abs(&mainboard.traj,(x1+x2)/2,opp2_y); 
			//err = goto_and_avoid ((x1+x2)/2, opp2_y,TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
			opp_y_saved = opp2_y;
		}

	}
	
	time_wait_ms (100);

	err = goto_and_avoid ((x1+x2)/2, opp_y_saved,TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);	
	if (!TRAJ_SUCCESS(err))
		ERROUT(err);
	
end:
	strat_set_speed(old_spdd, old_spda);	
	return err;
}

uint8_t strat_shoot_mamooth(uint8_t balls_mamooth_1, uint8_t balls_mamooth_2)
{
    uint8_t err = 0;
	static uint8_t state=0;

#define __BEGIN_LINE_Y 	460
#define BEGIN_MAMOOTH_X	750
#define SERVO_SHOOT_POS_UP 80
#define SERVO_SHOOT_POS_DOWN 300

	switch(state)
	{
		case 0:
			if(balls_mamooth_1 > 0)
			{	
				trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(BEGIN_MAMOOTH_X), __BEGIN_LINE_Y);
				state++;
				break;
			}
			else if(balls_mamooth_2 > 0)
			{
				trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(3000-BEGIN_MAMOOTH_X), __BEGIN_LINE_Y);
				state=3;
			}
			break;
		
		/* Goto mamooth  2 */
		case 1:
			if(opponent1_is_infront()==0 && opponent2_is_infront()==0)
			{
				//err = test_traj_end(TRAJ_FLAGS_STD);
				if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(BEGIN_MAMOOTH_X))<30) && (abs(position_get_y_double(&mainboard.pos)-__BEGIN_LINE_Y)<30))
				{
					trajectory_a_abs (&mainboard.traj, -90);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
							ERROUT(err);
					state ++;
				}
			}
			else
			{
				if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(BEGIN_MAMOOTH_X))<30) && (abs(position_get_y_double(&mainboard.pos)-__BEGIN_LINE_Y)<30))
				{
					trajectory_a_abs (&mainboard.traj, -90);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
							ERROUT(err);
					state ++;
					break;
				}
				strat_hardstop();
				state=0;
			}	
			break;
			
		/* Shoot to mamooth 2 */
		case 2:
				#ifndef HOST_VERSION
					pwm_servo_set(&gen.pwm_servo_oc3, SERVO_SHOOT_POS_UP);
					pwm_servo_set(&gen.pwm_servo_oc4, SERVO_SHOOT_POS_DOWN);
				#endif
				time_wait_ms(1000);
				
				if(balls_mamooth_2 == 0)
					return 1;
				else 
					state++;
			
			break;
			
		case 3:
				trajectory_goto_forward_xy_abs (&mainboard.traj, COLOR_X(3000-BEGIN_MAMOOTH_X), __BEGIN_LINE_Y);
				state++;
				break;
				
		/* Goto mamooth  1 */
		case 4:
			if(opponent1_is_infront()==0 && opponent2_is_infront()==0)
			{
				//err = test_traj_end(TRAJ_FLAGS_STD);
				if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(3000-BEGIN_MAMOOTH_X))<30) && (abs(position_get_y_double(&mainboard.pos)-__BEGIN_LINE_Y)<30))
				{
					trajectory_a_abs (&mainboard.traj, -90);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
							ERROUT(err);
					state ++;
				}
			}
			else
			{
				if ((abs(position_get_x_double(&mainboard.pos)-COLOR_X(3000-BEGIN_MAMOOTH_X))<30) && (abs(position_get_y_double(&mainboard.pos)-__BEGIN_LINE_Y)<30))
				{
					trajectory_a_abs (&mainboard.traj, -90);
					err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
					if (!TRAJ_SUCCESS(err))
							ERROUT(err);
					state ++;
					break;
				}
				strat_hardstop();
				state=3;
			}	
			break;
			
		/* Shoot to mamooth 1 */
		case 5:
				#ifndef HOST_VERSION
					pwm_servo_set(&gen.pwm_servo_oc3, SERVO_SHOOT_POS_UP);
					pwm_servo_set(&gen.pwm_servo_oc4, SERVO_SHOOT_POS_DOWN);
				#endif
				time_wait_ms(1000);
				return 1;
			
			break;
			
		default:
			break;
	}
	return 0;
	
	
end:	
	return err;
}

/* goto fresco */
uint8_t strat_goto_fresco (void) 
{
    int16_t x, y;
	uint8_t err = 0;
	static int8_t init = 0;

	/* position depending on color */
    x = COLOR_X(BEGIN_FRESCO_X);
    y = BEGIN_LINE_Y;

retry:
	if (init == 0) {
		init = 1;

		/* goto in front of fresco */
		trajectory_goto_forward_xy_abs (&mainboard.traj,x,y);
	    err = wait_traj_end(TRAJ_FLAGS_STD);

        if (!TRAJ_SUCCESS(err))
		   goto retry;
	}
	else {
		/* go with avoid */
		err = goto_and_avoid_forward (x, y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	}

end:
    return err;
}

/* paint fresco v2  */
uint8_t strat_paint_fresco2 (void)
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
	int16_t opp_d, opp_a;


	/* save speed */
	strat_get_speed (&old_spdd, &old_spda);
    strat_limit_speed_disable ();
	strat_set_speed (SPEED_DIST_SLOW,SPEED_ANGLE_FAST);
   

	/* turn to fresco */
	trajectory_a_abs (&mainboard.traj, 90);
	err = wait_traj_end(TRAJ_FLAGS_STD);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* check opp is not infront */
	while ( (opponent1_is_behind() && (get_opponent1_da(&opp_d, &opp_a)!=-1)) || 
			(opponent2_is_behind() && (get_opponent2_da(&opp_d, &opp_a)!=-1)) );

	/* go to near fresco */
	trajectory_goto_backward_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* set low speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_VERY_SLOW, SPEED_ANGLE_FAST);

	/* paint fresco 1/1 */
	trajectory_d_rel(&mainboard.traj, -200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);

	/* task flag done */
	robot_2nd.done_flags |= BT_FRESCO_DONE;

	/* check opponent */
	if (opponent1_is_infront() || opponent2_is_infront())
	{
		/* scape */
scape:
		trajectory_d_rel(&mainboard.traj, ROBOT_CENTER_TO_BACK + 100);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err)) {
			wait_ms (1000);
			goto scape;
		}

		/* second way */
		trajectory_goto_forward_xy_abs (&mainboard.traj,  COLOR_X(1650), ROBOT_CENTER_TO_BACK + 100);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
			ERROUT(err);

		/* return to start position */
		strat_set_speed(old_spdd, old_spda);
		trajectory_goto_forward_xy_abs (&mainboard.traj,  COLOR_X(1650), BEGIN_FRESCO_Y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

	}
	else {

		/* paint fresco 2/2 */
		trajectory_goto_forward_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), ROBOT_CENTER_TO_BACK + 100);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);

		/* return to start position */
		strat_set_speed(old_spdd, old_spda);
		trajectory_goto_forward_xy_abs (&mainboard.traj,  COLOR_X(BEGIN_FRESCO_X), BEGIN_FRESCO_Y);
		err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
		if (!TRAJ_SUCCESS(err))
				ERROUT(err);
	}


end:
	strat_set_speed(old_spdd, old_spda);	
    strat_limit_speed_enable();
    return err;
}

uint8_t strat_goto_and_paint_fresco (void)
{
	volatile uint8_t err = 0;
	int16_t opp_d, opp_a;


	/* goto fresco */
	err = strat_goto_fresco();
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	/* check opponent */	
	if ((robot_2nd.done_flags & BT_FRESCO_DONE) == 0) {
		if ((get_opponent1_da(&opp_d, &opp_a)!=-1) && (get_opponent2_da(&opp_d, &opp_a)!=-1)) {
			err = strat_paint_fresco2();
		}
		else if ((opp1_x_is_more_than(1800) && opp2_x_is_more_than(1800)) ||
				 !opp1_y_is_more_than(400) || !opp2_x_is_more_than(400) )
		{
			 err = strat_paint_fresco2();
		}
	}

#if 0
	while ((err & END_INTR) == 0) 
	{
		if ((robot_2nd.done_flags & BT_FRESCO_DONE) == 0)
		{
			if ((get_opponent1_da(&opp_d, &opp_a)!=-1) && (get_opponent2_da(&opp_d, &opp_a)!=-1)) {;
				err = strat_paint_fresco2();
			}
			else if ((opp1_x_is_more_than(1800) && opp2_x_is_more_than(1800)) ||
				     !opp1_y_is_more_than(400) || !opp2_x_is_more_than(400) )
			{
				 err = strat_paint_fresco2();
			}

			else {
				err = strat_patrol_between(COLOR_X(BEGIN_FRESCO_X),300,COLOR_X(BEGIN_FRESCO_X),900);
			}
		}
		else {
			err = strat_patrol_between(COLOR_X(BEGIN_FRESCO_X),300,COLOR_X(BEGIN_FRESCO_X),900);
		}
	}
#endif

end:
	return err;
}



