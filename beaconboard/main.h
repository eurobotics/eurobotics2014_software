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

/* NUMBER OF ROBOTS TO TRACK */
#define TWO_OPPONENTS
#define ROBOT_2ND

#define BRAKE_ON()      do { _LATC6 = 0; _LATC7	= 0; } while(0)
#define BRAKE_OFF()     do { Nop(); } while(0)

#define BEACON_ENCODER  ((void *)1)
#define BEACON_PWM      ((void *)&beaconboard.pwm_mc_mod2_ch1)

/** ERROR NUMS */
#define E_USER_SENSOR	196
#define E_USER_BEACON   197

/* EVENTS PRIORITY */
#define EVENT_PRIO_TIME 		160
#define EVENT_PRIO_SENSOR     120
#define EVENT_PRIO_CS         100
#define EVENT_PRIO_BEACON	    80

/* EVENTS PERIODS */
#define EVENT_PERIOD_BEACON 	20000L
#define EVENT_PERIOD_SENSOR   10000L
#define EVENT_PERIOD_CS        5000L

/* NUMBER OF DINAMIC LOGS */
#define NB_LOGS 4

/* structure of beacon control system */
struct cs_block 
{
	uint8_t on;
  	struct cs cs;
  	struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* main data structure of beacon */
struct beaconboard 
{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct pwm_mc pwm_mc_mod2_ch1;

	/* control systems */
 	struct cs_block speed;

	/* events flags */
	uint8_t flags;  
#define DO_ENCODERS  1
#define DO_CS        2
#define DO_BD        4
#define DO_POWER     8
#define DO_BEACON	  	16

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;

	/* robot play color */
	uint8_t our_color;

	/* beacon running watchdog */
	uint8_t watchdog_enable;
	uint16_t watchdog;
#define WATCHDOG_NB_TIMES	10

};

extern struct beaconboard beaconboard;


/* usefull macro */
#define wait_cond_or_timeout(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
        __ret;                                                \
})

