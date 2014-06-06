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
 *  Javier Bali?as Santos <javier@arc-robots.org>
 */

#include <aversive.h>
#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <ax12.h>

#include "i2c_protocol.h"
#include "actuator.h"
#include "main.h"
#include "robotsim.h"
#include "beacon.h"
#include "ax12_user.h"

void dac_set_and_save(void *pwm_mc, int32_t val)
{
	/* save value */	
	if (pwm_mc == LEFT_MOTOR)
		mainboard.pwm_l = val;
	else if (pwm_mc == RIGHT_MOTOR)
		mainboard.pwm_r = val;


	/* set value */
#ifdef HOST_VERSION
	robotsim_pwm(pwm_mc, val);
#else
	pwm_mc_set(pwm_mc, val);
#endif
}



/* actual beacon position and speed */
static volatile int32_t beacon_pos;
static volatile int32_t beacon_speed = 0;

/* beacon speed calculation based on encoder position,
 * used by cs as feedback. Must be compatible format with cs */
int32_t encoders_update_beacon_speed(void * dummy)
{
	int32_t ret;
	uint8_t flags;



	/* critical section */
	IRQ_LOCK(flags);

#ifndef HOST_VERSION

	/* get encoder position */
	ret = (int32_t)beacon_encoder_get_value();
#endif
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

uint8_t blade_hide (void) {
#ifndef HOST_VERSION
	ax12_user_write_int(&gen.ax12, 1, AA_GOAL_POSITION_L, 261);
#endif
    return 0;
}

uint8_t blade_push_fire (void) {
#ifndef HOST_VERSION
	ax12_user_write_int(&gen.ax12, 1, AA_GOAL_POSITION_L, 556);
#endif
    return 0;
}


