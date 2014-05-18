/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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

#ifndef _MAIN_H_
#define _MAIN_H_


#include <aversive.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <rdline.h>

#include <encoders_dspic.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <ax12.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <trajectory_manager.h>

#include "../common/i2c_commands.h"


/* SOME USEFUL MACROS AND VALUES  *********************************************/

/* NUMBER OF ROBOTS TO TRACK */
#define TWO_OPPONENTS
#define ROBOT_2ND

/* uart 0 is for cmds and uart 1 is 
 * multiplexed between beacon and slavedspic */
#define CMDLINE_UART 	0
#define MUX_UART	1

/* generic led toggle macro */
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		    \
			port &= ~_BV(bit);	        \
		else				            \
			port |= _BV(bit);	        \
	} while(0)

#ifdef HOST_VERSION
#define LED1_ON()
#define LED1_OFF()
#define LED1_TOGGLE()

#define BRAKE_DDR()
#define BRAKE_ON()
#define BRAKE_OFF()

#else

/* leds manage */
#define LED1_ON() 		cbi(LATA, 4)
#define LED1_OFF() 		sbi(LATA, 4)
#define LED1_TOGGLE() 	LED_TOGGLE(LATA, 4)

/* brake motors */
#define BRAKE_ON()      do {_LATA0 = 0; _LATA1 = 0; _LATA7 = 0; _LATB15 = 0; _LATB13 = 0; _LATA10 = 0;} while(0)
#define BRAKE_OFF()     //do {_LATA0 = 1; _LATA1 = 1; _LATA7 = 1; _LATB15 = 1; _LATB13 = 1; _LATA10 = 1;} while(0)

#endif /* !HOST_VERSION */

/* only 90 seconds, don't forget it :) */
#define MATCH_TIME 89


/* ROBOT PARAMETERS *************************************************/

/* distance between encoders weels,
 * decrease track to decrease angle */
#define EXT_TRACK_MM      219.67487845028
#define VIRTUAL_TRACK_MM  EXT_TRACK_MM

/* robot dimensions */
#define IM_SECONDARY_ROBOT
#define ROBOT_LENGTH      	    150.
#define ROBOT_WIDTH 	    	230.
#define ROBOT_CENTER_TO_FRONT   75.
#define ROBOT_CENTER_TO_BACK    75.
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE            (137.+10.)

/* Some calculus:
 * it is a 5000 imps -> 20000 because we see 1/4 period
 * and diameter: 53mm -> perimeter 166.5mm 
 * 20000/163.5 -> 1201 imps/10 mm */

/* increase it to go further */
#define IMP_ENCODERS 		  	    5000.0
#define WHEEL_DIAMETER_MM 		    53.0
#define WHEEL_PERIM_MM 	    	    (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 			        10.0
#define DIST_IMP_MM 		    	(((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)

/* encoders handlers */
#define LEFT_ENCODER        ((void *)1)
#define RIGHT_ENCODER       ((void *)2)

/* motor handles */
#define MOTOR_1     	((void *)&gen.pwm_mc_1)
#define MOTOR_2     	((void *)&gen.pwm_mc_2)
#define MOTOR_3     	((void *)&gen.pwm_mc_3)

#define LEFT_MOTOR	MOTOR_2
#define RIGHT_MOTOR	MOTOR_3


/** ERROR NUMS */
#define E_USER_STRAT        194
#define E_USER_I2C_PROTO    195
#define E_USER_SENSOR       196
#define E_USER_CS           197
#define E_USER_BEACON       198
#define E_USER_AX12         199
#define E_USER_BT_WT11      199
#define E_USER_BT_PROTO     200

/* EVENTS PRIORITIES */
#define EVENT_PRIORITY_LED 			170
#define EVENT_PRIORITY_TIME         160
#define EVENT_PRIORITY_I2C_POLL     140
#define EVENT_PRIORITY_SENSORS      120
#define EVENT_PRIORITY_CS           100
#define EVENT_PRIORITY_STRAT        30
#define EVENT_PRIORITY_BEACON_POLL  20
#define EVENT_PRIORITY_CMDLINE      15
#define EVENT_PRIORITY_STRAT_EVENT	10



/* EVENTS PERIODS */
#define EVENT_PERIOD_LED 	        1000000L
#define EVENT_PERIOD_CMDLINE 		50000L
#define EVENT_PERIOD_STRAT		  	25000L
#define EVENT_PERIOD_STRAT_EVENT  	25000L
#define EVENT_PERIOD_BEACON_PULL	10000L
#define EVENT_PERIOD_SENSORS	  	10000L
#define EVENT_PERIOD_I2C_POLL	  	8000L
#define EVENT_PERIOD_CS 		    5000L

#define CS_PERIOD   ((EVENT_PERIOD_CS/SCHEDULER_UNIT)*SCHEDULER_UNIT) /* in microsecond */
#define CS_HZ       (1000000. / CS_PERIOD)


/* dynamic logs */
#define NB_LOGS 10

/* MAIN DATA STRUCTURES **************************************/

/* cs data */
struct cs_block {
	uint8_t on;
	struct cs cs;
  struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* genboard */
struct genboard
{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct pwm_mc pwm_mc_1;
	struct pwm_mc pwm_mc_2;
	struct pwm_mc pwm_mc_3;

	/* servos */
	struct pwm_servo pwm_servo_oc1;
	struct pwm_servo pwm_servo_oc2;
	struct pwm_servo pwm_servo_oc3;	
	struct pwm_servo pwm_servo_oc4;

	/* ax12 servos */
	AX12 ax12;

	/* i2c gpios */
	uint8_t i2c_gpio0;
	uint8_t i2c_gpio1;

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

/* maindspic */
struct mainboard 
{
	/* events flags */
	uint8_t flags;                
#define DO_ENCODERS   1
#define DO_CS         2
#define DO_RS         4
#define DO_POS        8
#define DO_BD         16
#define DO_TIMER      32
#define DO_POWER      64
#define DO_OPP        128

	/* control systems */
	struct cs_block angle;
	struct cs_block distance;

	/* x,y positionning and traj*/
	struct robot_system rs;
	struct robot_position pos;
   	struct trajectory traj;

	/* robot status */
	uint8_t our_color;

	volatile int16_t speed_a;  /* current angle speed */
	volatile int16_t speed_d;  /* current dist speed */

	int32_t pwm_l;  /* current left dac */
	int32_t pwm_r;  /* current right dac */

	/* strat events */
	int8_t  strat_event;
	int16_t strat_event_data[3];
};


/* state of beaconboard, synchronized through i2c */
struct beaconboard 
{
	/* status and color */
	uint8_t status;
	uint8_t color;
	
	/* opponent pos */
	int16_t opponent1_x;
	int16_t opponent1_y;
	int16_t opponent1_a;
	int16_t opponent1_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;
	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif
};

/* main robot structure parameters */
struct robot_2nd
{
	/* command requested */
	uint8_t cmd_id;					/* for ack test */
	uint8_t cmd_ret; 					/* for end traj test, 
												follows END_TRAJ flags rules, 
												see strat_base.h */
	uint8_t cmd_args_checksum;		/* checksum of cmd arguments*/

	/* strat info */
	uint16_t done_flags;

  	/* robot position */
	int16_t x;
	int16_t y;
  	int16_t a_abs;
	int16_t a;
	int16_t d;

  	/* opponent pos */
	int16_t opponent1_x;
	int16_t opponent1_y;
	int16_t opponent1_a;
	int16_t opponent1_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;
	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif
};

extern struct genboard gen;
extern struct mainboard mainboard;
extern struct beaconboard beaconboard;
extern struct robot_2nd robot_2nd;


/* set bt cmd id and args checksum */
static inline void bt_set_cmd_id_and_checksum (uint8_t cmd_id, uint8_t args_sumatory) {
	uint8_t flags;
	IRQ_LOCK (flags);
	robot_2nd.cmd_id = cmd_id;
	robot_2nd.cmd_args_checksum = (uint8_t)(cmd_id + args_sumatory);
	IRQ_UNLOCK (flags);
}

/* set bt cmd id and args checksum */
static inline void bt_set_cmd_ret (uint8_t ret) {
	uint8_t flags;
	IRQ_LOCK (flags);
	robot_2nd.cmd_ret = ret;
	IRQ_UNLOCK (flags);
}

///* TODO start the bootloader */
//void bootloader(void);

#ifndef HOST_VERSION
/* swap UART 2 between beacon and slavedspic */ 
static inline void set_uart_mux(uint8_t channel)
{
#define BEACON_CHANNEL			0
#define SLAVEDSPIC_CHANNEL		1

	uint8_t flags;

	IRQ_LOCK(flags);


	if(channel == BEACON_CHANNEL){
		_U2RXR 	= 9;	  /* U2RX <- RP9(RB9)  <- BEACON_UART_RX */
		_TRISB9 	= 1;	/* U2RX is input								*/
	  	_RP25R 	= 5;	  /* U2TX -> RP25(RC9) -> BEACON_UART_TX */
		_TRISC9	= 0;	  /* U2TX is output								*/
	}
	else
	{
		_U2RXR 	= 2;	  /* U2RX <- RP2(RB2) <- SLAVE_UART_TX	*/
		_TRISB2 	= 1;	/* U2RX is input								*/
	  	_RP3R 	= 5;	  /* U2TX -> RP3(RB3) -> SLAVE_UART_RX	*/
		_TRISB3	= 0;	  /* U2TX is output								*/
	}


	IRQ_UNLOCK(flags);
	Nop();
}
#endif

#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
	if (__ret)					      \
		DEBUG(E_USER_STRAT, "cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})
#endif

