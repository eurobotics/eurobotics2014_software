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

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <configuration_bits_config.h>

#include <uart.h>
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

#include "../common/i2c_commands.h"

#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "cs.h"
#include "beacon.h"

/* main data structure of beacon */
struct beaconboard beaconboard;


/* init timer 1, used as main time base */
void main_timer_init(void)
{
	T1CON = 0;              
	IFS0bits.T1IF = 0;      
	IEC0bits.T1IE = 1;      
	TMR1 = 0x0000;  	
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;  
}

/* to call on main timer interrupt */
static void main_timer_interrupt(void)
{
	/* manage task */	
	sei();
	scheduler_interrupt();
}

/* timer 1 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  _T1IF = 0;
  main_timer_interrupt();
}

/***********************************************************
 * init pins: address (input/output), remapeable (RPx),		
 * digital or analog funcion, open collector and init value 
 ***********************************************************/ 
void io_pins_init(void)
{

	/*********************************************
 	*  IO portmap and config
 	*/
	
	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
		and has disabled the read operation
	*/

	// mostfet H bridge (outputs)
	_TRISC6 = 0;	// PH1
	_ODCC6 	= 1;	// PH1 is open collector
	_TRISC7 = 0;	// PH2
	_ODCC7 	= 1;	// PH2 is open collector
	
	_LATC6 	= 0;	// motor stoped
	_LATC7	= 0;
	
	// keyence sensors (inputs capture)
	_TRISC4 = 1;	// PZ2_1 (IC1)
	_IC1R		= 20;	// IC1 <- RP20
	_TRISC5 = 1;	// PZ2_2 (IC2)
	_IC2R		= 21;	// IC2 <- RP21
	
	// ee-sx671a sensors (inputs capture)
	_TRISC9 = 1;	// HERR1 (IC7)
	_IC7R		= 25; // IC7 <- RP25
	_TRISC8 = 1;	// HERR2 (IC8)
	_IC8R		= 24; // IC8 <- RP24
	
	// encoder (inputs)
	_QEA1R 	= 10;	// QEA1 <- RP10
	_TRISB10= 1;	// QEA1
	_QEB1R 	= 11;	// QEB1 <- RP11
	_TRISB11= 1;	// QEB1

	// stop bumper (input)
	// XXX: NOT USED
	_TRISB9 = 1;	// BUMPER
	
	// wt11 reset (output)
	_TRISB13= 0;	// RESET_BLUE
	_LATB13	= 0;	// RESET_BLUE OFF
	
	// i2c (in/out open collector??)
	// TODO: i2c
	
	// uart
	_U1RXR 	= 8;	// U1RX <- RP8
	_TRISB8  = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7
	_TRISB7	= 0;	// U1TX is output
}

int main(void)
{
	/* disable interrupts */
	cli();

	/* TODO: eemprom magic number */
	
	/* remapeable pins */
	io_pins_init();
	
	/* oscillator */
	oscillator_init();

	/* reset data structures */
	memset(&beaconboard, 0, sizeof(beacon));

	/* initial events */
	beaconboard.flags = DO_ENCODERS | DO_CS | DO_POWER;
	
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

	/* ENCODERS */
	encoders_dspic_init();

	/* PWM */
	pwm_mc_channel_init(&beaconboard.pwm_mc_mod2_ch1,
	                    PWM_MC_MODE_BIPOLAR|PWM_MC_MODE_SIGN_INVERTED, 
	                    2, 1, NULL, 0, NULL, 0);
	pwm_mc_init(&beaconboard.pwm_mc_mod2_ch1, 15000, CH1_COMP&PEN1H&PEN1L);
	pwm_mc_set(&beaconboard.pwm_mc_mod2_ch1, 0);		


	/* MAIN TIMER */
	main_timer_init();

	/* SCHEDULER */
	scheduler_init();

	/* all cs management */
	beacon_cs_init();

	/* sensors */
	/* XXX: not necesary, uncoment for testing */
	//sensor_init();
	
	/* TIME */
	time_init(EVENT_PRIO_TIME);

	/* beacon */
	beacon_init();

	/* enable interrupts */
	sei();

	/* DINAMIC LOGS */
	//beaconboard.log_level = 5;
	//beaconboard.logs[0] = E_USER_BEACON;

	/* program WT-11 */
#if 0
	time_wait_ms (1000);
	printf ("+++\n\r");	  
	time_wait_ms (1000);
	printf ("SET BT NAME beacon\n\r");	
	time_wait_ms (1000);
	printf ("SET BT AUTH * gomaespuminos\n\r");
	time_wait_ms (1000);
#endif

	/* hello */
	printf_P(PSTR("\r\n"));
	printf_P(PSTR("Don't turn it on, take it a part!!\r\n"));
	
	/* command line, never returns */
	cmdline_interact();

	return 0;
}
