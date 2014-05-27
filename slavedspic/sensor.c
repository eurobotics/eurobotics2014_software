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
 *  Revision : $Id$
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.c,v 1.4 2009/04/24 19:30:42 zer0 Exp.
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>

#include <scheduler.h>
#include <ax12.h>
#include <parse.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "state.h"
#include "sensor.h"
#include "main.h"

/* boolean sensors ********************************************/

struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[SENSOR1] 	= { 11, 0, 8, 10, 0, 1 },
	[SENSOR2] 	= { 11, 0, 8, 10, 0, 1 },
	[SENSOR3] 	= { 11, 0, 8, 10, 0, 1 },
	[SENSOR4] 	= { 11, 0, 8, 10, 0, 1 },
	[SENSOR5]	= { 11, 0, 8, 10, 0, 1 },
	[SENSOR6]	= { 11, 0, 8, 10, 0, 1 },
	//[SENSOR7] 	= { 1, 0, 0, 1, 0, 1 },
};

/* value of filtered sensors */
static uint16_t sensor_filtered = 0;

uint16_t sensor_get_all(void)
{
	uint16_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
	uint16_t tmp = sensor_get_all();
	return !!(tmp & _BV(i));
}

/* get the physical value of pins */
static uint16_t sensor_read(void)
{

/* sensor mapping : 
 * SENSOR1: RB11
 * SENSOR2: RB10
 * SENSOR3: RC8
 * SENSOR4: RA8
 * SENSOR5: RC3
 * SENSOR6: RB4	 XXX servos AX12 in 2011 board
 * SENSOR7: RC2 	 XXX broken pin in 2011 board, pwm servo on 2012 board
 */

	uint16_t tmp = 0;
	//XXX tmp |= (uint16_t)((PORTB & (_BV(11)))>> 11)<< 0;
	//XXX tmp |= (uint16_t)((PORTB & (_BV(10)))>> 10)<< 1;
	tmp |= (uint16_t)((PORTC & (_BV(8)))>> 8)<< 2;
	tmp |= (uint16_t)((PORTA & (_BV(8)))>> 8)<< 3;
	tmp |= (uint16_t)((PORTC & (_BV(3)))>> 3)<< 4;
#ifndef EUROBOT_2011_BOARD
	tmp |= (uint16_t)((PORTB & (_BV(4)))>> 4)<< 5;
#ifndef EUROBOT_2011_BOARD
	tmp |= (uint16_t)((PORTC & (_BV(2)))>> 2)<< 6;
#endif
#endif

	/* add more sensors here */
	
	return tmp;
}

/* called every X ms, see below */
static void do_boolean_sensors(__attribute__((unused)) void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint16_t sensor = sensor_read();
	uint16_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		if ((1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}
		
		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= (1UL << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= (1UL << i);
		}
	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}


/* called every X ms as a scheduler task */
void do_sensors(__attribute__((unused)) void *dummy)
{
	do_boolean_sensors(NULL);
}


/* return true if object is cached by turbine */
uint8_t sensor_object_is_catched(void) {

//	return ((sensor_get_all() & S_TURBINE_LINE_A) && (sensor_get_all() & S_TURBINE_LINE_B));
	return 0;
}
