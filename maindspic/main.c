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

#ifndef HOST_VERSION
#include <configuration_bits_config.h>
#endif

#include <uart.h>
#include <i2c_mem.h>

#include <encoders_dspic.h>
#include <dac_mc.h>
#include <pwm_servo.h>


#include <scheduler.h>
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

#include <parse.h>
#include <rdline.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"
#include "i2c_protocol.h"
#include "beacon.h"
#include "bt_protocol.h"
#include "robotsim.h"
#include "strat_base.h"


struct genboard gen;
struct mainboard mainboard;
struct slavedspic slavedspic;
struct beaconboard beaconboard;
struct robot_2nd robot_2nd;

/***************************************************************/

#ifndef HOST_VERSION
void do_led_blink(void *dummy)
{
	/* simple blink */
	LED1_TOGGLE();
}

static void main_timer_interrupt(void)
{
	/* scheduler tasks */
	sei();
	scheduler_interrupt();	
}

/* main timer */
void main_timer_init(void)
{
	/* use timer 1 */
	T1CON = 0;              
	IFS0bits.T1IF = 0;      
	IEC0bits.T1IE = 1;      
	TMR1 = 0x0000;  	
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;  
}

/* timer 1 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  _T1IF=0;
  main_timer_interrupt();
}

void io_pins_init(void)
{
	/***************************************
 	*  IO portmap and config
 	*/

	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
	 *		  and has disabled the read operation
	 */

	/* analog inputs */
	/* by default all analog pins are digital */
	AD1PCFGL = 0xFF;	

	/* leds */
	_TRISA4 = 0;	/* MAIN_LED1 */
	_TRISA8 = 0;	/* MAIN_LED2 */
	_TRISC2 = 0;	/* MAIN_LED3 */
	_TRISC8 = 0;	/* MAIN_LED4 */
	
	/* brushless motors */
	_TRISA10 = 0; 	/* L_MOT_REV	*/
	_TRISA7  = 0;	/* L_MOT_BREAK	*/
	_LATA7 	 = 0;

	_TRISB10 = 0; 	/* R_MOT_REV	*/
	_TRISB11 = 0; 	/* R_MOT_BREAK	*/
	_LATB11  = 0;

	/* servos */
	_RP22R = 0b10010; /* OC1 -> RP22(RC6) -> MAIN_SERVO_PWM_1 */
	_RP23R = 0b10011; /* OC2 -> RP23(RC7) -> MAIN_SERVO_PWM_2 */
	_TRISC6 = 0;
	_TRISC7	= 0;
	
	/* encoders */	
	_QEA1R 	= 21;	/* QEA1 <- RP21(RC5) <- R_ENC_CHA */
	_TRISC5 = 1;	
	_QEB1R 	= 20;	/* QEB1 <- RP20(RC4) <- R_ENC_CHB */
	_TRISC4	= 1;

	_QEA2R 	= 19;	/* QEA2 <- RP19(RC3) <- L_ENC_CHA */
	_TRISC3 = 1;	
	_QEB2R 	= 4;	/* QEB2 <- RP4(RB4)  <- L_ENC_CHB */
	_TRISB4	= 1;	
	
	/* lasers */
	AD1PCFGL &= ~(_BV(7));	/* AN7 <- MAIN_LASER_1 */
	AD1PCFGL &= ~(_BV(6));	/* AN6 <- MAIN_LASER_2 */
			
	/* i2c */
	/* XXX open collector */
	_ODCB6 = 1;
	_ODCB5 = 1;

	/* uarts */
	/* U1 is for cmdline and bootloader */
	_U1RXR 	= 8;	/* U1RX <- RP8(RB8) <- MAIN_UART_RX	*/
	_TRISB8 	= 1;	/* U1RX is input	*/
  	_RP7R 	= 3;	/* U1TX -> RP7(RB7) -> MAIN_UART_TX	*/
	_TRISB7	= 0;	/* U1TX is output	*/
	
	/* UART2 swap between BEACON and SLAVEDSPIC */
	set_uart_mux(BEACON_CHANNEL);

//#if 1
//	_U2RXR 	= 9;	/* U2RX <- RP9(RB9)  <- BEACON_UART_RX */
//	_TRISB9 	= 1;	/* U2RX is input								*/
//  	_RP25R 	= 5;	/* U2TX -> RP25(RC9) -> BEACON_UART_TX */
//	_TRISC9	= 0;	/* U2TX is output								*/
//#else
//	_U2RXR 	= 2;	/* U2RX <- RP2(RB2) <- SLAVE_UART_TX	*/
//	_TRISB2 	= 1;	/* U2RX is input								*/
//  	_RP3R 	= 5;	/* U2TX -> RP3(RB3) -> SLAVE_UART_RX	*/
//	_TRISB3	= 0;	/* U2TX is output								*/
//#endif
}
#endif /* !HOST_VERSION */


int main(void)
{	
	/* disable interrupts */
	cli();

	/* TODO: eeprom magic number */

#ifndef HOST_VERSION
	/* remapeable pins */
	io_pins_init();

	/* brake motors */
	BRAKE_ON();
	
	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
#endif

	/* reset data structures */
	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));
	memset(&slavedspic, 0, sizeof(slavedspic));
	memset(&beaconboard, 0, sizeof(beaconboard));

	/* init flags */
  	mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
		DO_POS | DO_POWER | DO_BD;
	
	/* bt protocol */
	beaconboard.link_id = 0xFF;
	robot_2nd.link_id = 0xFF;

	/* opponents and robot 2nd positions */
	beaconboard.opponent1_x = I2C_OPPONENT_NOT_THERE;

#ifdef TWO_OPPONENTS
    beaconboard.opponent2_x = I2C_OPPONENT_NOT_THERE;
#endif

#ifdef ROBOT_2ND
	robot_2nd.x = I2C_OPPONENT_NOT_THERE;
#endif

#ifndef HOST_VERSION
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);
#endif 

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

#ifndef HOST_VERSION
	/* ENCODERS */
	encoders_dspic_init();

	/* I2C */
	i2c_init();
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	i2c_protocol_init();

	/* DAC_MC */
	dac_mc_channel_init(&gen.dac_mc_right, 1, CHANNEL_R,
											DAC_MC_MODE_SIGNED|DAC_MC_MODE_SIGN_INVERTED,
										 	&LATB, 10, NULL, 0);
	

	dac_mc_channel_init(&gen.dac_mc_left, 1, CHANNEL_L,
											DAC_MC_MODE_SIGNED,
										 	&LATA, 10, NULL, 0);
	
	dac_mc_set(&gen.dac_mc_right, 0);
	dac_mc_set(&gen.dac_mc_left, 0);


	/* MAIN TIMER */
	main_timer_init();
#endif

	/* SCHEDULER */
	scheduler_init();
#ifdef HOST_VERSION
	hostsim_init();
	robotsim_init();
#endif

	/* EVENTS OR INIT MODULES THAT INCLUDE EVENTS */
#ifndef HOST_VERSION
	scheduler_add_periodical_event_priority(do_led_blink, NULL, 
						EVENT_PERIOD_LED / SCHEDULER_UNIT, EVENT_PRIORITY_LED);
#endif

	/* time */
	time_init(EVENT_PRIORITY_TIME);

	/* all cs management */
	maindspic_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();

#ifndef HOST_VERSION
	/* i2c slaves polling (gpios and slavedspic) */
	scheduler_add_periodical_event_priority(i2c_poll_slaves, NULL,
						EVENT_PERIOD_I2C_POLL / SCHEDULER_UNIT, EVENT_PRIORITY_I2C_POLL);

	/* beacon commnads and polling */
	//scheduler_add_periodical_event_priority(beacon_protocol, NULL,
	//				EVENT_PERIOD_BEACON_PULL / SCHEDULER_UNIT, EVENT_PRIORITY_BEACON_POLL);
#endif
	/* beacon and robot 2nd commnads and polling */
	scheduler_add_periodical_event_priority(bt_protocol, NULL,
					EVENT_PERIOD_BEACON_PULL / SCHEDULER_UNIT, EVENT_PRIORITY_BEACON_POLL);

	/* strat-related event */
	scheduler_add_periodical_event_priority(strat_event, NULL,
						EVENT_PERIOD_STRAT / SCHEDULER_UNIT, EVENT_PRIORITY_STRAT);

	/* log setup */
 	gen.logs[0] = E_USER_STRAT;
 	//gen.logs[1] = E_USER_BEACON;
 	gen.logs[1] = E_USER_I2C_PROTO;
 	//gen.logs[3] = E_OA;
 	gen.logs[2] = E_USER_WT11;
 	//gen.logs[4] = E_USER_BT_PROTO;
 	gen.log_level = 5;
	
	/* reset strat infos */
	strat_reset_infos();

	/* enable interrupts */
	sei();

	/* wait to init of slavedspic */
	wait_ms(2000);
	
	/* say hello */
	printf("\r\n");
	printf("Don't turn it on, take it a part!!\r\n");

#ifdef HOST_VERSION
	mainboard.our_color = I2C_COLOR_YELLOW;
	strat_reset_pos(COLOR_X(200), 500, COLOR_A_ABS(0));
	//strat_event_enable();
#endif

	/* program WT-11 */
#if 0
	time_wait_ms (1000);
	printf ("+++\n\r");	  
	time_wait_ms (1000);
	printf ("SET BT NAME Grosnik\n\r");	
	time_wait_ms (1000);
	printf ("SET BT AUTH * gomaespuminos\n\r");
	time_wait_ms (1000);
#endif
  
	/* process commands, never returns */
	cmdline_interact();

	return 0;
}










