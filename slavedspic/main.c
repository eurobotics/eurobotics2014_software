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



#include <configuration_bits_config.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <encoders_dspic.h>

#include <dac_mc.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <ax12.h>
#include <uart.h>
#include <i2c_slave_lite.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>
#include <parse.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "state.h"
#include "actuator.h"
#include "i2c_protocol.h"
#include "cs.h"
#include "main.h"

struct genboard gen;
struct slavedspic slavedspic;

extern uint8_t i2c_watchdog_cnt;

#undef EUROBOT_2011_BOARD
#ifdef EUROBOT_2011_BOARD
#warning COMPILATION FOR EUROBOT 2011 BOARD!!
#endif

void do_led_blink(__attribute__((unused)) void *dummy){
#if 1 /* simple blink */
	static uint8_t a=0;

	if(a){
		LED1_ON();
	}	
	else{
		LED1_OFF();
	}
	a = !a;
#endif
}

void do_i2c_watchdog(void *dummy){
	if(i2c_watchdog_cnt == 0){
		i2c_init(I2C_SLAVEDSPIC_ADDR);
		ERROR(E_USER_I2C_PROTO,"I2C watchdog triggered, reinit i2c hw");
	}
	
	i2c_watchdog_cnt--;		
}

static void main_timer_interrupt(void){
	sei();
	scheduler_interrupt();
}

void timer_init(void)
{
	/* Scheduler uses timer1 */
	T1CON = 0;              
	IFS0bits.T1IF = 0;      
	IEC0bits.T1IE = 1;      
	TMR1 = 0x0000;  	
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;  
}

/* Timer 1 interrupt handler */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  IFS0bits.T1IF=0;
    
  /* scheduler interrupt */
  main_timer_interrupt();
}


void io_pins_init(void)
{
	/***************************************
 	*  IO portmap and config
 	*/
	
	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
		and has disabled the read operation
	*/
	
	/* leds */
	_TRISC9 = 0;	// SLAVE_LED1

	/* sensors */
	AD1PCFGL = 0xFF;	// all analog pins are digital

	/* used as GPO (EV E2040)*/
	//XXX _TRISB11 = 0;		// SENSOR1
	//XXX _TRISB10 = 0;		// SENSOR2
	_TRISC8 	= 1;		// SENSOR3
	_TRISA8 	= 1;		// SENSOR4
	_TRISC3 	= 1;		// SENSOR5
	_TRISB4 	= 1;		// SENSOR6
#ifndef EUROBOT_2012_BOARD
	_TRISC2 	= 1;		// SENSOR7
#endif


	/* dc motors */
	_TRISB12 = 0;	// SLAVE_MOT_2_INA
	_TRISB13 = 0;	// SLAVE_MOT_2_INB
	_LATB12  = 0;	// initialy breaked
	_LATB13  = 0;

	_TRISC6 = 0;	// SLAVE_MOT_1_INA
	_TRISC7 = 0;	// SLAVE_MOT_1_INB
	_LATC6  = 0;	// initialy braked
	_LATC7  = 0;

	/* other power outputs */
	_TRISA9 = 0;	// SLAVE_RELE_OUT
	_ODCA9  = 1;	// is open drain
	_LATA9  = 1;	// initialy Vload = 0

	_TRISA4 = 0;	// SLAVE_MOST_OUT
	_ODCA4  = 1; 	// is open drain
	_LATA4  = 1;	// initialy Vload = 0


	/* brushless motors */
	_TRISA10 = 0; 	// SLAVE_MOT_BRUSH_REV
	_TRISA7  = 0; 	// SLAVE_MOT_BRUSH_BREAK
	_LATA7   = 0;	// initialy breaked
	
	/* servos  */

	/* XXX vacum motors */
	_TRISC1	= 0;
	_LATC1 = 1;
	_TRISB3 = 0;
	_LATB3 = 1;

	/* XXX electro valvules */
	_TRISB11 = 0;		// SENSOR1
	_LATB11 = 0;
	_TRISB10 = 0;		// SENSOR2
	_LATB10 = 0;

	/* servos  */
	// XXX _RP17R = 0b10010; // OC1 -> RP17(RC1) -> SLAVE_SERVO_PWM_1
	// XXX _RP3R  = 0b10100; // OC3 -> RP3(RB3)  -> SLAVE_SERVO_PWM_3
	_RP16R = 0b10011; // OC2 -> RP16(RC0) -> SLAVE_SERVO_PWM_2
#ifdef EUROBOT_2012_BOARD
	_RP18R = 0b10101; // OC4 -> RP18(RC2) -> SLAVE_SENSOR_7
#endif
		
	/* encoders */
	_QEA1R 	= 21;	// QEA1 <- RP21 <- SLAVE_ENC_CHA
	_TRISC5  = 1;	
	_QEB1R 	= 20;	// QEB1 <- RP20 <- SLAVE_ENC_CHB
	_TRISC4	= 1;
		
	/* i2c */
	_ODCB6 = 1;
	_ODCB5 = 1;
	
	/* uarts */
	_U1RXR 	= 8;	// U1RX <- RP8 <- SLAVE_UART_RX
	_TRISB8  = 1;	// U1RX is input
  	_RP7R 	= 3;	// U1TX -> RP7 -> SLAVE_UART_TX
	_TRISB7	= 0;	// U1TX is output

#ifndef EUROBOT_2011_BOARD	
	_U2RXR 	= 9;	// U2RX <- RP9 <- SERVOS_AX12_UART
  	_RP9R 	= 5;	// U2TX -> RP9 -> SERVOS_AX12_UART
	_TRISB9	= 0;	// U2TX is output
 	_ODCB9 	= 1;	// For half-duplex mode RP9 is open collector
#else
	_U2RXR 	= 9;	// U2RX <- RP4 <- SERVOS_AX12_UART
  	_RP4R 	= 5;	// U2TX -> RP4 -> SERVOS_AX12_UART
	_TRISB4	= 0;	// U2TX is output
 	_ODCB4 	= 1;	// For half-duplex mode RP4 is open collector
#endif
}

int main(void)
{
	/* disable interrupts */
	cli();

	/* remapeable pins */
	io_pins_init();

	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();

	/* clear structures */
	memset(&gen, 0, sizeof(gen));
	memset(&slavedspic, 0, sizeof(slavedspic));
	
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

	/* CS */
	slavedspic_cs_init();

	/* I2C */
	i2c_init(I2C_SLAVEDSPIC_ADDR);
	i2c_register_read_event(i2c_read_event);
	i2c_register_write_event(i2c_write_event);
	i2c_protocol_init();

	/* TIMER */
	timer_init();

	/* PWM_MC */
	pwm_mc_channel_init(&gen.pwm_mc_mod2_ch1,
	                    PWM_MC_MODE_BIPOLAR, 
	                    2, 1, NULL, 0, NULL, 0);
	pwm_mc_init(&gen.pwm_mc_mod2_ch1, 19000, 
							CH1_COMP&PEN1H&PEN1L);
	pwm_mc_set(&gen.pwm_mc_mod2_ch1, 0);


	/* DO FLAGS */
	/* note: cs is enabled after calibration */
	slavedspic.flags = DO_ENCODERS | DO_POWER | DO_BD;


	/* DAC_MC */
	dac_mc_channel_init(&gen.dac_mc_left, 1, CHANNEL_L,
							  DAC_MC_MODE_SIGNED | DAC_MC_MODE_SIGN_INVERTED, &LATA, 10, NULL, 0);
	dac_mc_set(&gen.dac_mc_left, 0);

	/* servos */
	pwm_servo_init(&gen.pwm_servo_oc1, 1, 600, 2400);
	pwm_servo_init(&gen.pwm_servo_oc2, 2, 600, 2400);
	pwm_servo_init(&gen.pwm_servo_oc3, 3, 600, 2400);
	pwm_servo_init(&gen.pwm_servo_oc4, 4, 600, 2400);
	pwm_servo_enable();
	pwm_servo_set(&gen.pwm_servo_oc1, 1000);
	pwm_servo_set(&gen.pwm_servo_oc2, 1000);
	pwm_servo_set(&gen.pwm_servo_oc3, 1000);
	pwm_servo_set(&gen.pwm_servo_oc4, 1000);

	/* SCHEDULER */
	scheduler_init();

	//scheduler_add_periodical_event_priority(do_led_blink, NULL, 
	//					100000L / SCHEDULER_UNIT, 
	//					LED_PRIO);

	scheduler_add_periodical_event_priority(do_cs, NULL, 
						CS_PERIOD / SCHEDULER_UNIT, 
						CS_PRIO);

	//scheduler_add_periodical_event_priority(do_i2c_watchdog, NULL,
	//					8000L / SCHEDULER_UNIT,
	//					I2C_POLL_PRIO);

	scheduler_add_periodical_event_priority(do_sensors, NULL, 
						10000L / SCHEDULER_UNIT, 
						SENSOR_PRIO);
	/* TIME */
	time_init(TIME_PRIO);

	/* SERVOS AX12 */
	ax12_user_init();

	/* LOGS */
 	gen.logs[0] = E_USER_ST_MACH;
	gen.logs[1] = E_USER_ACTUATORS;
	//gen.logs[2] = E_USER_CS;
	gen.log_level = 5;

	/* enable interrupt */
	sei();

	/* wait some ms */
	wait_ms(500);

	/* ACTUATORS */
	actuator_init();

	printf("\r\n");
	printf("Don't turn it on, take it a part!! \n\r");
	
	/* init cmdline */
	cmdline_init();

	/* main loop */
	while(1)
	{
		state_machines();
		cmdline_interact_nowait();
	}

	return 0;
}
