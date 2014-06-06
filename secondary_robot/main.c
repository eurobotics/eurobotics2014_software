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
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <ax12.h>


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
#include "robotsim.h"
#include "strat_base.h"
#include "ax12_user.h"
#include "beacon.h"


struct genboard gen;
struct mainboard mainboard;
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
    *        and has disabled the read operation
    */

   /* analog inputs */
   /* XXX by default all analog pins are digital */
   AD1PCFGL = 0xFF;

   /* leds */
   _TRISA4 = 0;   /* LED5 */

   /* sensor inputs */
   _TRISC9   = 1;   /* SENSOR_1 */
	_IC7R		 = 25;  // IC7 <- RP25

   _TRISB2   = 1;   /* SENSOR_2 */
   _IC2R		 = 2;  // IC2 <- RP2

   _TRISA8   = 1;   /* SENSOR_3 */

   /* L6203 H bridges (outputs) */
   _TRISA0    = 0;   // MOT_1_IN1
   _TRISA1    = 0;   // MOT_1_IN2
   _TRISB14   = 0;   // MOT_1_EN

   _TRISA7    = 0;   // MOT_2_IN1
   _TRISB15   = 0;   // MOT_2_IN2
   _TRISB12   = 0;   // MOT_2_EN

   _TRISB13   = 0;   // MOT_3_IN1
   _TRISA10   = 0;   // MOT_3_IN2
   _TRISB10   = 0;   // MOT_3_EN

   /* wt11 */
   _TRISA9 = 0;
   _LATA9 = 0;

   /* servos */
   _RP18R = 0b10010; /* OC1 -> RP18(RC2) -> SERVO_1_PWM */
   _RP17R = 0b10011; /* OC2 -> RP17(RC1) -> SERVO_2_PWM */
   _RP16R = 0b10100; /* OC3 -> RP16(RC0) -> SERVO_3_PWM */
   _RP3R  = 0b10101; /* OC4 -> RP3(RB3)  -> SERVO_4_PWM */

   _TRISC2 = 0;
   _TRISC1 = 0;
   _TRISC0 = 0;
   _TRISB3 = 0;

   /* encoders */

   /* XXX encoder 1 channels has been inverted for march with the other one */

   _QEA1R    = 20;   /* QEA1 <- RP21(RC5) <- ENC_1_CHA */
   _TRISC5  = 1;
   _QEB1R    = 21;   /* QEB1 <- RP20(RC4) <- ENC_1_CHB */
   _TRISC4   = 1;

   _QEA2R    = 19;   /* QEA2 <- RP19(RC3) <- ENC_2_CHA */
   _TRISC3  = 1;
   _QEB2R    = 4;   /* QEB2 <- RP4(RB4)  <- ENC_2_CHB */
   _TRISB4   = 1;

   /* ENC 3 */
	_TRISB11 = 1;
	_T4CKR = 11;	/* T3CK <-- RP11 */

   /* i2c */
   /* XXX open collector */
   _ODCB6 = 1;
   _ODCB5 = 1;

   /* uarts */
   /* U1 is for cmdline and bootloader */
   _U1RXR    = 8;   /* U1RX <- RP8(RB8) <- UART_RX   */
   _TRISB8 = 1;   /* U1RX is input   */
   _RP7R    = 3;   /* U1TX -> RP7(RB7) -> UART_TX   */
   _TRISB7   = 0;   /* U1TX is output   */

   /* UART SERVO AX12 */
   _U2RXR    = 9;   // U2RX <- RP9 <- SERVOS_AX12_UART
   _RP9R    = 5;   // U2TX -> RP9 -> SERVOS_AX12_UART
   _TRISB9   = 0;   // U2TX is output
   _ODCB9    = 1;   // For half-duplex mode RP9 is open collector

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
#endif

   /* reset data structures */
   memset(&gen, 0, sizeof(gen));
   memset(&mainboard, 0, sizeof(mainboard));
   memset(&beaconboard, 0, sizeof(beaconboard));
   memset(&robot_2nd, 0, sizeof(robot_2nd));

   /* init flags */
#ifdef HOST_VERSION
  mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
      DO_POS | DO_POWER | DO_BD;
#else
  mainboard.flags = DO_ENCODERS  | DO_RS |
      DO_POS | DO_POWER | DO_BD| DO_CS ;
#endif

   beaconboard.opponent1_x = I2C_OPPONENT_NOT_THERE;
   robot_2nd.opponent1_x = I2C_OPPONENT_NOT_THERE;
   robot_2nd.x = I2C_OPPONENT_NOT_THERE;

#ifdef TWO_OPPONENTS
   beaconboard.opponent2_x = I2C_OPPONENT_NOT_THERE;
   robot_2nd.opponent2_x = I2C_OPPONENT_NOT_THERE;
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
   pwm_mc_channel_init(MOTOR_1,
                       PWM_MC_MODE_SIGNED,
                       1, 1, &PORTA, 0, &PORTA, 1);

   pwm_mc_channel_init(MOTOR_2,
                       PWM_MC_MODE_SIGNED | PWM_MC_MODE_SIGN_INVERTED,
                       1, 2, &PORTA, 7, &PORTB, 15);

   pwm_mc_channel_init(MOTOR_3,
                       PWM_MC_MODE_SIGNED,
                       1, 3, &PORTB, 13, &PORTA, 10);

   pwm_mc_init(MOTOR_1, 15000, CH1_IND&PEN1H&PDIS1L &
                               CH2_IND&PEN2H&PDIS2L &
                               CH3_IND&PEN3H&PDIS3L);
   pwm_mc_init(MOTOR_2, 15000, CH1_IND&PEN1H&PDIS1L &
                               CH2_IND&PEN2H&PDIS2L &
                               CH3_IND&PEN3H&PDIS3L);
   pwm_mc_init(MOTOR_3, 15000, CH1_IND&PEN1H&PDIS1L &
                               CH2_IND&PEN2H&PDIS2L &
                               CH3_IND&PEN3H&PDIS3L);

   pwm_mc_set(MOTOR_1, 0);
   pwm_mc_set(MOTOR_2, 0);
   pwm_mc_set(MOTOR_3, 0);

   /* servos */
   pwm_servo_init(&gen.pwm_servo_oc1, 1, 500, 2400);
   pwm_servo_init(&gen.pwm_servo_oc2, 2, 500, 2400);
   pwm_servo_init(&gen.pwm_servo_oc3, 3, 300, 2400);
   pwm_servo_init(&gen.pwm_servo_oc4, 4, 300, 2400);
   pwm_servo_enable();

   pwm_servo_set(&gen.pwm_servo_oc1, 0);
   pwm_servo_set(&gen.pwm_servo_oc2, 0);
   pwm_servo_set(&gen.pwm_servo_oc3, 1200);
   pwm_servo_set(&gen.pwm_servo_oc4, 1300);

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

#ifndef HOST_VERSION
   /* SERVOS AX12 */
   ax12_user_init();

#endif



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
   // EVENT_PERIOD_BEACON_PULL / SCHEDULER_UNIT, EVENT_PRIORITY_BEACON_POLL);
#endif

   /* strat-related event */
   scheduler_add_periodical_event_priority(strat_event, NULL,
    EVENT_PERIOD_STRAT / SCHEDULER_UNIT, EVENT_PRIORITY_STRAT);

#ifndef HOST_VERSION
	beacon_init();
#endif

   /* log setup */
    gen.logs[0] = E_USER_STRAT;
    gen.logs[1] = E_USER_BEACON;
    //gen.logs[2] = E_USER_I2C_PROTO;
    //gen.logs[3] = E_OA;
    gen.logs[2] = E_USER_BT_PROTO;
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
   strat_reset_pos(COLOR_X(520), 420, COLOR_A_ABS(90));
   //strat_event_enable();
#endif

	/* program WT-11 */
#if 0
	time_wait_ms (1000);
	printf ("+++\n\r");
	time_wait_ms (1000);
	printf ("SET BT NAME Seskapa\n\r");
	time_wait_ms (1000);
	printf ("SET BT AUTH * gomaespuminos\n\r");
	time_wait_ms (1000);
#endif

   /* start */
   //strat_start_match(1);

   /* process commands, never returns */
   //cmdline_interact(NULL);

#ifndef HOST_VERSION
    blade_hide ();
	beacon_start ();
#endif
	cmdline_init();

	/* command line event */
   	scheduler_add_periodical_event_priority(cmdline_interact_nowait, NULL,
    		EVENT_PERIOD_CMDLINE / SCHEDULER_UNIT, EVENT_PRIORITY_CMDLINE);
	
	while (1);

   return 0;
}










