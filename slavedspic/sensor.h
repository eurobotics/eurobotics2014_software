/*  
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
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
 *  Revision : $Id: sensor.h,v 1.4 2009/04/24 19:30:42 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.h,v 1.4 2009/04/24 19:30:42 zer0 Exp.
 */


/* synchronize with sensor.c */
#define SENSOR1       0
#define SENSOR2       1
#define SENSOR3       2
#define SENSOR4       3
#define SENSOR5       4
#define SENSOR6       5
//#define SENSOR7     6
#define SENSOR_MAX    6


void sensor_init(void);

/* get filtered values of boolean sensors */
uint16_t sensor_get_all(void);
uint8_t sensor_get(uint8_t i);

/* called every X ms as a scheduler task */
void do_sensors(__attribute__((unused)) void *dummy);

uint8_t sensor_object_is_catched(void);
