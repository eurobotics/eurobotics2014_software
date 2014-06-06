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
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <scheduler.h>


#include "../common/i2c_commands.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "../maindspic/strat_avoid.h"
#include "../maindspic/strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"

#include "strat.h"
#include "bt_protocol.h"

/*************** STRAT EVENT SCHEDULE FUNCTIONS *******************************/

/* schedule a single strat tevent */
void strat_event_schedule_single (void (*f)(void *), void * data)
{
	uint8_t flags;
	int8_t ret;

	/* stop robot */
	strat_hardstop();
	
	/* delete current event */
	scheduler_del_event(mainboard.strat_event);

	/* add event */
	IRQ_LOCK(flags);
	ret  = scheduler_add_single_event_priority(f, data, 
			EVENT_PERIOD_STRAT/SCHEDULER_UNIT, EVENT_PRIORITY_STRAT_EVENT);
	mainboard.strat_event = ret;
	IRQ_UNLOCK(flags);
}

/* schedule a periodical strat event */
void strat_event_schedule_periodical(void (*f)(void *), void * data)
{
	uint8_t flags;
	int8_t ret;

	/* stop robot */
	strat_hardstop();

	/* delete current event */
	scheduler_del_event(mainboard.strat_event);

	/* add event */
	IRQ_LOCK(flags);
	ret  = scheduler_add_periodical_event_priority(f, data,
			EVENT_PERIOD_STRAT/SCHEDULER_UNIT, EVENT_PRIORITY_STRAT_EVENT);
	mainboard.strat_event = ret;
	IRQ_UNLOCK(flags);
}


/*************** STRAT EVENT WRAPPER FUNCTIONS  *******************************/

/* auto position event */
void strat_auto_position_event (void *data)
{
	strat_auto_position ();

	/* return value */
	bt_status_set_cmd_ret (END_TRAJ);
}


void strat_goto_xy_abs_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	trajectory_goto_xy_abs(&mainboard.traj, arg[0], arg[1]);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}
void strat_goto_forward_xy_abs_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	trajectory_goto_forward_xy_abs(&mainboard.traj, arg[0], arg[1]);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}
void strat_goto_backward_xy_abs_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	trajectory_goto_backward_xy_abs(&mainboard.traj, arg[0], arg[1]);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}
void strat_goto_avoid_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	goto_and_avoid(arg[0], arg[1],  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}
void strat_goto_avoid_forward_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	goto_and_avoid_forward(arg[0], arg[1],  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}
void strat_goto_avoid_backward_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	goto_and_avoid_backward(arg[0], arg[1],  TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
	err = wait_traj_end(TRAJ_FLAGS_STD);

	if (err != END_TRAJ && err != END_NEAR)
		strat_hardstop();

	/* return value */
	bt_status_set_cmd_ret (err);
}


void strat_patrol_fresco_mamooth_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
	err=strat_patrol_fresco_mamooth(arg[0], arg[1]);

	/* return value */
	bt_status_set_cmd_ret (err);
}


void strat_fresco_event (void *data)
{
	uint8_t err;
 
	err=strat_paint_fresco();

	/* return value */
	bt_status_set_cmd_ret (END_TRAJ);
}


void strat_patrol_event (void *data)
{	
	//int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	//err=strat_patrol_between(arg[0], arg[1],arg[2], arg[3]);

	/* return value */
	bt_status_set_cmd_ret (err);
}

void strat_mamooth_event (void *data)
{
	int16_t *arg = (int16_t*)data;
	uint8_t err;
 
	err=strat_shoot_mamooth(arg[0], arg[1]);

	/* return value */
	bt_status_set_cmd_ret (err);
}

void strat_protect_h1_event (void *data)
{
	uint8_t err;
 
	err=strat_patrol_between(400,1300,400,1800);

	/* return value */
	bt_status_set_cmd_ret (err);
}

void strat_net_event (void *data)
{
	/* TODO */
}




