/*  
 *  Copyright Droids Corporation
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: cs.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  cs.c,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include <parse.h>
#include <rdline.h>

#include "main.h"
#include "beacon.h"

/* actual beacon position and speed */
static volatile int32_t beacon_pos;
static volatile int32_t beacon_speed = 0;


/* beacon speed calculation based on encoder position,
 * used by cs as feedback. Must be compatible format with cs */
int32_t encoders_update_beacon_speed(void * number)
{
	int32_t ret;
	uint8_t flags;

	/* critical section */
	IRQ_LOCK(flags);
	
	/* get encoder position */
	ret = encoders_dspic_get_value(number);

	/* calulate speed */
	beacon_speed = ret - beacon_pos;
	beacon_pos = ret;

	IRQ_UNLOCK(flags);
	
	return beacon_speed;
}

/* read actual beacon speed */
int32_t encoders_get_beacon_speed(void * dummy)
{
	return beacon_speed;
}



/* cs processing called periodically */
static void do_cs(void *dummy) 
{
	/* read encoders */
	if (beaconboard.flags & DO_ENCODERS) {
		encoders_dspic_manage(NULL);
	}

	/* control system */
	if (beaconboard.flags & DO_CS) {
		if (beaconboard.speed.on)
			cs_manage(&beaconboard.speed.cs);
	}

	/* power enable */
	if (beaconboard.flags & DO_POWER)
		BRAKE_OFF();
	else
		BRAKE_ON();
}

/* dump check points of cs */
void dump_cs(const char *name, struct cs *cs)
{
	printf_P(PSTR("%s cons=% .5ld fcons=% .5ld err=% .5ld "
		      "in=% .5ld out=% .5ld\r\n"), 
		 name, cs_get_consign(cs), cs_get_filtered_consign(cs),
		 cs_get_error(cs), cs_get_filtered_feedback(cs),
		 cs_get_out(cs));
}

/* dump processing PID results */
void dump_pid(const char *name, struct pid_filter *pid)
{
	printf_P(PSTR("%s P=% .8ld I=% .8ld D=% .8ld out=% .8ld\r\n"),
		 name,
		 pid_get_value_in(pid) * pid_get_gain_P(pid),
		 pid_get_value_I(pid) * pid_get_gain_I(pid),
		 pid_get_value_D(pid) * pid_get_gain_D(pid),
		 pid_get_value_out(pid));
}

/* init beacon cs */
void beacon_cs_init(void)
{
	/* PID */
	pid_init(&beaconboard.speed.pid);
	pid_set_gains(&beaconboard.speed.pid, 80, 80, 250);
	pid_set_maximums(&beaconboard.speed.pid, 0, 10000, 2600);
	pid_set_out_shift(&beaconboard.speed.pid, 6);
	pid_set_derivate_filter(&beaconboard.speed.pid, 6);

	/* CS */
	cs_init(&beaconboard.speed.cs);
	cs_set_correct_filter(&beaconboard.speed.cs, pid_do_filter, &beaconboard.speed.pid);
	cs_set_process_in(&beaconboard.speed.cs, pwm_mc_set, BEACON_PWM);
	cs_set_process_out(&beaconboard.speed.cs, encoders_update_beacon_speed, BEACON_ENCODER);
	cs_set_consign(&beaconboard.speed.cs, 0);

	/* set it on !! */
	beaconboard.speed.on = 0;

	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_cs, NULL, 
						EVENT_PERIOD_CS / SCHEDULER_UNIT, EVENT_PRIO_CS);
}




