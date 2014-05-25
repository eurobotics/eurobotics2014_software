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
 *  Revision : $Id: sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>

#include "main.h"
#include "sensor.h"
#include "strat.h"
#include "../maindspic/strat_utils.h"


#ifndef HOST_VERSION

/************ ADC */
/* config init */
static void adc_init(void)
{
	/* adc off */
	_ADON = 0;

	/* 3V external reference  */
	_VCFG = 0b011;


	/* by default: 10 bit mode and ADCLK from TCY */
	
	/* Adquisition and conversion time (TSAM, TAD, TCONV):

		_SAMC =	11111 = 31 TAD = TSAM
					•
					•
					•
					00001 = 1 TAD
					00000 = 0 TAD

		_ADCS =	00111111 = TCY · (ADCS<7:0> + 1) = 64 · TCY = TAD
					.
					.
					. 	
					00000010 = TCY · (ADCS<7:0> + 1) = 3 · TCY = TAD 
					00000001 = TCY · (ADCS<7:0> + 1) = 2 · TCY = TAD
					00000000 = TCY · (ADCS<7:0> + 1) = 1 · TCY = TAD

		TADmin = 76 ns

		TCONV = 12 • TAD

		FCONVmax = 1.1 Msps 

	*/

	_SSRC = 0b111;			/* TSAM auto with internal counter */
	_SAMC = 0b11111;		/* TSAM = 31· TCY = 775 ns */
	_ADCS = 0b00111111;	/* TAD = 64· TCY = 1.6 us, TCONV = 19.2 us (50 Ksps max) */ 

	_ASAM = 0;
	
	/* interrupt */
	_AD1IF = 0;
	_AD1IE = 1;

	/* adc on */
	_ADON = 1;
}


/* launch new adquisition */
void adc_launch(uint16_t conf)
{
	AD1CHS0 = conf;
	
	/* lauch conversion */
	_SAMP = 1;
}


/* adc interrupt */
static void adc_event(uint16_t result);
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
  	_AD1IF=0;

	/* XXX suposse _DONE = 1 */
	adc_event(ADCBUF0);
}

struct adc_infos {
	uint16_t config;
	int16_t value;
	int16_t prev_val;
  int16_t (*filter)(struct adc_infos *, int16_t);
};


/* reach 90% of the value in 4 samples */
int16_t rii_light(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + (int32_t)adc->prev_val / 2;
	return adc->prev_val / 2;
}

/* reach 90% of the value in 8 samples */
int16_t rii_medium(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 3) / 4;
	return adc->prev_val / 4;
}

/* reach 90% of the value in 16 samples */
int16_t rii_strong(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 7) / 8;
	return adc->prev_val / 8;
}

#define ADC_CONF(x) (x)

/* define which ADC to poll, see in sensor.h */
static struct adc_infos adc_infos[ADC_MAX] = { 
	[ADC_LASER_1] = { .config = ADC_CONF(7), .filter = NULL },
	[ADC_LASER_2] = { .config = ADC_CONF(6), .filter = NULL },
};

/* call on adc interrupt */
static void adc_event(uint16_t result)
{
	static uint8_t i = 0;

	/* filter value if needed */
	if (adc_infos[i].filter)
		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i], result);
	else
		adc_infos[i].value = result;

	i ++;
	if (i >= ADC_MAX)
		i = 0;
	else
		adc_launch(adc_infos[i].config);
}

/* called every 10 ms, see init below */
static void do_adc(void *dummy) 
{
	/* launch first conversion */
	adc_launch(adc_infos[0].config);
}
#endif /* !HOST_VERSION */

/* get analog sensor value */
int16_t sensor_get_adc(uint8_t i)
{
#ifdef HOST_VERSION
	return 0;
#else
	int16_t tmp;
	uint8_t flags;

	IRQ_LOCK(flags);
	tmp = adc_infos[i].value;
	IRQ_UNLOCK(flags);
	return tmp;
#endif
}


/************ boolean sensors */

#ifndef HOST_VERSION
struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_SENSOR_1] =  { 1, 0, 0, 1, 0, 0 }, /* 0 */
	[S_SENSOR_2] =  { 1, 0, 0, 1, 0, 0 }, /* 1 */
	[S_SENSOR_3] =  { 1, 0, 0, 1, 0, 0 }, /* 2 */
	[S_RESERVED4] = { 0, 0, 0, 1, 0, 0 }, /* 3 */
	[S_RESERVED5] = { 0, 0, 0, 1, 0, 0 }, /* 4 */
	[S_RESERVED6] = { 0, 0, 0, 1, 0, 0 }, /* 5 */
	[S_RESERVED7] = { 0, 0, 0, 1, 0, 0 }, /* 6 */
	[S_RESERVED8] = { 0, 0, 0, 1, 0, 0 }, /* 7 */
		
	[S_GP0_0] = { 1, 0, 0, 1, 0, 1 }, /* 8 */
	[S_GP0_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_4] = { 1, 0, 0, 1, 0, 0 }, /*  */
	[S_GP0_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP0_7] = { 1, 0, 0, 1, 0, 1 }, /* 15 */

	[S_GP1_0] = { 1, 0, 0, 1, 0, 1 }, /* 16 */
	[S_GP1_1] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_2] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_3] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_4] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_5] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_6] = { 1, 0, 0, 1, 0, 1 }, /*  */
	[S_GP1_7] = { 1, 0, 0, 1, 0, 1 }, /* 23 */
};
#endif /* !HOST_VERSION */

/* value of filtered sensors */
static uint64_t sensor_filtered = 0;

/* sensor mapping : 
 * 0:  	 SENSOR 1 (RC9)
 * 1:  	 SENSOR 2 (RB2)
 * 2:  	 SENSOR 3 (RA8)
 * 4-7:   reserved
 * 8-15:  i2c GP0
 * 16-23: i2c GP1
 */

uint64_t sensor_get_all(void)
{
	uint64_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
#ifdef HOST_VERSION
	return 0;
#else
	uint64_t tmp = sensor_get_all();
	return (uint8_t)((tmp & ((uint64_t)1 << i))>>i);
#endif
}

#ifndef HOST_VERSION
/* get the physical value of pins */
static uint64_t sensor_read(void)
{
	uint64_t tmp = 0;

	tmp |= (uint64_t)((PORTC & (_BV(9))) >> 9) << 0;
	tmp |= (uint64_t)((PORTB & (_BV(2))) >> 2) << 1;
	tmp |= (uint64_t)((PORTA & (_BV(8))) >> 8) << 2;
	/* 1 to 7 reserved */

	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio0))<< 8;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio1))<< 16;

	/* add reserved sensors here */
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint64_t sensor = sensor_read();
	uint64_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {
		
		if(sensor_filter[i].filter == 0)
			continue;
		
		if (((uint64_t)1 << i) & sensor) {
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
			tmp |= ((uint64_t)1 << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}

	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}
#endif /* !HOST_VERSION */

/* virtual obstacle */
#define DISABLE_CPT_MAX 200 /* XXX T_SENSORS = 10ms */
static uint16_t disable = 0; 	/* used to disable obstacle detection 
			   				 		           * during some time */

/* called every 10 ms */
void sensor_obstacle_update(void)
{
	if (disable > 0) {
		disable --;
		if (disable == 0)
			DEBUG(E_USER_STRAT, "re-enable sensor");
	}
}

void sensor_obstacle_disable(void)
{
	DEBUG(E_USER_STRAT, "disable sensor");
	disable = DISABLE_CPT_MAX;
}

void sensor_obstacle_enable(void)
{
	DEBUG(E_USER_STRAT, "re-enable sensor");
	disable = 0;
}

uint8_t sensor_obstacle_is_disabled(void)
{
	return disable;
}


/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
#ifndef HOST_VERSION
	do_adc(NULL);
	do_boolean_sensors(NULL);
#endif
	sensor_obstacle_update();
}

void sensor_init(void)
{
#ifndef HOST_VERSION
	adc_init();
#endif	
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						EVENT_PERIOD_SENSORS / SCHEDULER_UNIT, EVENT_PRIORITY_SENSORS);
}

