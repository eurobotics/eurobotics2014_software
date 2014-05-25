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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <stdint.h>

/* used by cs, correct offset and save values */
void dac_set_and_save(void *dac, int32_t val);

/* beacon speed calculation based on encoder position,
 * used by cs as feedback. Must be compatible format with cs */
int32_t encoders_update_beacon_speed(void * number);

/* read actual beacon speed */
int32_t encoders_get_beacon_speed(void * dummy);


#endif

