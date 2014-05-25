/*
 *  Copyright Droids Corporation (2009)
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
 *  Revision : $Id: commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>
#include <i2c_mem.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_core.h>
#include <trajectory_manager_utils.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#ifdef HOST_VERSION
#include <hostsim.h>
#endif

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"
//#include "../common/bt_commands.h"

#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "strat.h"
#include "../maindspic/strat_utils.h"
#include "strat_base.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "beacon.h"
#include "robotsim.h"
#include "bt_protocol.h"

#ifdef HOST_VERSION
#define COMPILE_COMMANDS_MAINBOARD
#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#endif

extern int8_t beacon_connected;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS |
			DO_BD | DO_TIMER | DO_POWER | DO_BEACON;
		if (!strcmp_P(res->arg2, PSTR("on")))
			mainboard.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			mainboard.flags &= bit;
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("rs is %s\r\n"), 
				 (DO_RS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("pos is %s\r\n"), 
				 (DO_POS & mainboard.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & mainboard.flags) ? "on":"off");
			printf_P(PSTR("timer is %s\r\n"), 
				 (DO_TIMER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & mainboard.flags) ? "on":"off");
			printf_P(PSTR("opp is %s\r\n"), 
				 (DO_BEACON & mainboard.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	else if (!strcmp_P(res->arg1, PSTR("cs"))) {
		strat_hardstop();
		bit = DO_CS;
	}
	else if (!strcmp_P(res->arg1, PSTR("rs")))
		bit = DO_RS;
	else if (!strcmp_P(res->arg1, PSTR("pos")))
		bit = DO_POS;
	else if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	else if (!strcmp_P(res->arg1, PSTR("timer"))) {
		time_reset();
		bit = DO_TIMER;
	}
	else if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;
	else if (!strcmp_P(res->arg1, PSTR("opp")))
		bit = DO_BEACON;

	if (!strcmp_P(res->arg2, PSTR("on")))
		mainboard.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		if (!strcmp_P(res->arg1, PSTR("cs"))) {
#ifdef HOST_VERSION
			robotsim_pwm(LEFT_MOTOR, 0);
			robotsim_pwm(RIGHT_MOTOR, 0);
#else
		 	pwm_mc_set(LEFT_MOTOR, 0);
			pwm_mc_set(RIGHT_MOTOR, 0);
#endif
		}
		mainboard.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & mainboard.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power#beacon";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
	.f = cmd_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_event_arg0, 
		(prog_void *)&cmd_event_arg1, 
		(prog_void *)&cmd_event_arg2, 
		NULL,
	},
};


/**********************************************************/
/* Opponent tests */

/* this structure is filled when cmd_opponent is parsed successfully */
struct cmd_opponent_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
	int32_t arg3;
};

/* function called when cmd_opponent is parsed successfully */
static void cmd_opponent_parsed(void *parsed_result, void *data)
{
	struct cmd_opponent_result * res = parsed_result;

	int16_t x,y,d,a;
	int16_t x2, y2, d2, a2;
	int16_t x_r2nd,y_r2nd,d_r2nd,a_r2nd, a_abs_r2nd;
	int8_t opp1, opp2, r2nd;
//	microseconds us;

	if(data)
	{
		beaconboard.opponent1_d = res->arg2;
		beaconboard.opponent1_a = res->arg3;
		beaconboard.opponent1_x = 0;
	}
	else
		do {
	
	#ifdef TWO_OPPONENTS
		opp1 = get_opponent1_xyda(&x, &y, &d, &a);
		opp2 = get_opponent2_xyda(&x2, &y2, &d2, &a2);
		r2nd = get_robot_2nd_xyda(&x_r2nd, &y_r2nd, &d_r2nd, &a_r2nd);
    	get_robot_2nd_a_abs(&a_abs_r2nd);
		
		if (opp1 == -1)
			printf_P(PSTR("opp1 not there"));
		else
			printf_P(PSTR("opp1 x=%.4d y=%.4d, d=%.4d a=%+.3d"), x, y, d, a);
	
		if (opp2 == -1)
			printf_P(PSTR(" / opp2 not there"));
		else
			printf_P(PSTR(" / opp2 x=%.4d y=%.4d, d=%.4d a=%+.3d"), x2, y2, d2, a2);
	
	#ifdef ROBOT_2ND
		if (r2nd == -1)
			printf_P(PSTR(" / r2nd not there"));
		else
			printf_P(PSTR(" / r2nd x=%.4d y=%.4d, d=%.4d a=%+.3d (%+.3d)"), x_r2nd, y_r2nd, d_r2nd, a_r2nd, a_abs_r2nd);
	#endif
	
		printf("\n\r");
	
	#else
		if (get_opponent_xyda(&x, &y, &d, &a) == -1)
			printf_P(PSTR("No opponent\r\n"));
		else
			printf_P(PSTR("x=%d y=%d, d=%d a=%d\r\n"), x, y, d, a);
	#endif
	
	//	us = time_get_us2();
	//	while(time_get_us2()-us < 200000L) {
	//		beacon_opponent_pulling();
	//	}
	
		wait_ms(200);
	
		} while (!cmdline_keypressed());
}

prog_char str_opponent_arg0[] = "opponent";
parse_pgm_token_string_t cmd_opponent_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg0, str_opponent_arg0);
prog_char str_opponent_arg1[] = "show";
parse_pgm_token_string_t cmd_opponent_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1);

prog_char help_opponent[] = "Show (x,y) opponent";
parse_pgm_inst_t cmd_opponent = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_opponent,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0, 
		(prog_void *)&cmd_opponent_arg1, 
		NULL,
	},
};


prog_char str_opponent_arg1_set[] = "set";
parse_pgm_token_string_t cmd_opponent_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_opponent_result, arg1, str_opponent_arg1_set);
parse_pgm_token_num_t cmd_opponent_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg2, INT32);
parse_pgm_token_num_t cmd_opponent_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_opponent_result, arg3, INT32);

prog_char help_opponent_set[] = "Set (x,y) opponent";
parse_pgm_inst_t cmd_opponent_set = {
	.f = cmd_opponent_parsed,  /* function to call */
	.data = (void*)1,      /* 2nd arg of func */
	.help_str = help_opponent_set,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_opponent_arg0, 
		(prog_void *)&cmd_opponent_arg1_set,
		(prog_void *)&cmd_opponent_arg2, 
		(prog_void *)&cmd_opponent_arg3, 
		NULL,
	},
};



/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result {
	fixed_string_t arg0;
	fixed_string_t color;
	fixed_string_t debug;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{
	struct cmd_start_result *res = parsed_result;
	uint8_t old_level = gen.log_level;
	int8_t c;
	
	gen.logs[NB_LOGS] = E_USER_STRAT;
	if (!strcmp_P(res->debug, PSTR("debug"))) {
		strat_infos.dump_enabled = 1;
		gen.log_level = 5;
	}
	else if (!strcmp_P(res->debug, PSTR("debug_step"))) {
		strat_infos.dump_enabled = 1;
		strat_infos.debug_step = 1;	
		gen.log_level = 5;
	}
	else {
		strat_infos.dump_enabled = 0;
		gen.log_level = 0;
	}

#if 0
#ifndef HOST_VERSION
retry:
	printf_P(PSTR("Press a key when beacon ready, 'q' for skip \r\n"));
	c = -1;
	while(c == -1){
		c = cmdline_getchar();
	}
	
	if(c == 'q'){
		printf("Play without beacon\r\n");
	}
	else{
		beacon_cmd_wt11_call();
		WAIT_COND_OR_TIMEOUT((beacon_connected==1),5000);
		if(!beacon_connected){
			printf("Beacon connection FAIL, reseting local wt11\r\n");
			beacon_cmd_wt11_local_reset();
			goto retry;
		}
		else{
			printf("Beacon connection SUCCESS!\r\n");		
retry_on:
			beacon_cmd_beacon_on_watchdog();
			
			printf("is beacon running? (s/n)\n\r");
			c = -1;
			while(c == -1){
				c = cmdline_getchar();
			}
			if(c == 'n'){
				wait_ms(100);
				goto retry_on;	
			}
		}
	}
#endif
#endif

	if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
		//beacon_cmd_color();
	}
	else if (!strcmp_P(res->color, PSTR("yellow"))) {
		mainboard.our_color = I2C_COLOR_YELLOW;
		//beacon_cmd_color();
	}

	strat_start();

	gen.logs[NB_LOGS] = 0;
	gen.log_level = old_level;
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_color[] = "red#yellow";
parse_pgm_token_string_t cmd_start_color = TOKEN_STRING_INITIALIZER(struct cmd_start_result, color, str_start_color);
prog_char str_start_debug[] = "debug#debug_step#match";
parse_pgm_token_string_t cmd_start_debug = TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
	.f = cmd_start_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_start,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_start_arg0, 
		(prog_void *)&cmd_start_color, 
		(prog_void *)&cmd_start_debug, 
		NULL,
	},
};


/**********************************************************/
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

void bt_send_status (void);

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;

	if (!strcmp_P(res->color, PSTR("yellow"))) {
		mainboard.our_color = I2C_COLOR_YELLOW;	}
	else if (!strcmp_P(res->color, PSTR("red"))) {
		mainboard.our_color = I2C_COLOR_RED;
	}
	else if (!strcmp_P(res->color, PSTR("show"))) {
		if(mainboard.our_color == I2C_COLOR_YELLOW)
			printf("color is YELLOW\n\r");
		else
			printf("color is RED\n\r");
	}

	bt_set_color (mainboard.our_color);

	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "yellow#red#show";
parse_pgm_token_string_t cmd_color_color = TOKEN_STRING_INITIALIZER(struct cmd_color_result, color, str_color_color);

prog_char help_color[] = "Set our color";
parse_pgm_inst_t cmd_color = {
	.f = cmd_color_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_color,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_color_arg0, 
		(prog_void *)&cmd_color_color, 
		NULL,
	},
};

/**********************************************************/
/* TODO beacon */
#if 0
/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_beacon_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

static void cmd_beacon_parsed(void * parsed_result, void * data)
{
	int16_t c;
	int8_t cmd = 0;
	struct vt100 vt100;

	struct cmd_beacon_result *res = parsed_result;

	vt100_init(&vt100);
	
	if(!strcmp_P(res->arg1, "raw"))
	{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
		/* init vt100 character set */
		vt100_init(&vt100);
		
		/* interact */
		while(cmd != KEY_CTRL_C) 
		{
			/* received from slave */
			if((c = uart_recv_nowait(MUX_UART))!= -1)
				/* echo */
				uart_send_nowait(CMDLINE_UART,c);
			
			/* send to slavedspic */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}

			/* check exit cmd */
			cmd = vt100_parser(&vt100, c);

			/* send to slave */
			uart_send_nowait(MUX_UART,c);	
		}
#endif
	}	
	else if(!strcmp_P(res->arg1, "wt11_reset")){
		beacon_cmd_wt11_local_reset();
	}
	else if(!strcmp_P(res->arg1, "call")){
		beacon_cmd_wt11_call();
	}
	else if(!strcmp_P(res->arg1, "wt11_close")){
		beacon_cmd_wt11_close();
	}
	else if(!strcmp_P(res->arg1, "on")){
		beacon_cmd_beacon_on();
	}
	else if(!strcmp_P(res->arg1, "watchdog_on")){
		beacon_cmd_beacon_on();
	}
	else if(!strcmp_P(res->arg1, "off")){
		beacon_cmd_beacon_off();
	}
	else if(!strcmp_P(res->arg1, "color")){
		beacon_cmd_color();
	}
	else if(!strcmp_P(res->arg1, "opponent")){
		//beacon_cmd_opponent();
		beacon_pull_opponent();
	}

	printf("Done\n\r");
}

prog_char str_beacon_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg0, str_beacon_arg0);
prog_char str_beacon_arg1[] = "raw#wt11_reset#call#wt11_close#on#watchdog_on#off#color#opponent";
parse_pgm_token_string_t cmd_beacon_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg1, str_beacon_arg1);

prog_char help_beacon[] = "beacon commads";
parse_pgm_inst_t cmd_beacon = {
	.f = cmd_beacon_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_arg0, 
		(prog_void *)&cmd_beacon_arg1, 
		NULL,
	},
};
#endif



#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

/**********************************************************/
/* Interact */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_interact_result {
	fixed_string_t arg0;
};

static void print_cs(void)
{
	printf_P(PSTR("cons_d=% .8"PRIi32" cons_a=% .8"PRIi32" fil_d=% .8"PRIi32" fil_a=% .8"PRIi32" "
		      "err_d=% .8"PRIi32" err_a=% .8"PRIi32" out_d=% .8"PRIi32" out_a=% .8"PRIi32"\r\n"),
		 cs_get_consign(&mainboard.distance.cs),
		 cs_get_consign(&mainboard.angle.cs),
		 cs_get_filtered_consign(&mainboard.distance.cs),
		 cs_get_filtered_consign(&mainboard.angle.cs),
		 cs_get_error(&mainboard.distance.cs),
		 cs_get_error(&mainboard.angle.cs),
		 cs_get_out(&mainboard.distance.cs),
		 cs_get_out(&mainboard.angle.cs));
}

static void print_pos(void)
{
	printf_P(PSTR("x=% .8d y=% .8d a=% .8d\r\n"),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
}

static void print_time(void)
{
	printf_P(PSTR("time %d\r\n"),(int16_t)time_get_s());
}


static void print_sensors(void)
{
#ifdef notyet
	if (sensor_start_switch())
		printf_P(PSTR("Start switch | "));
	else
		printf_P(PSTR("             | "));

	if (IR_DISP_SENSOR())
		printf_P(PSTR("IR disp | "));
	else
		printf_P(PSTR("        | "));

	printf_P(PSTR("\r\n"));
#endif
}

static void print_pid(void)
{
	printf_P(PSTR("P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32" | "
		      "P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32"\r\n"),
		 pid_get_value_in(&mainboard.distance.pid) * pid_get_gain_P(&mainboard.distance.pid),
		 pid_get_value_I(&mainboard.distance.pid) * pid_get_gain_I(&mainboard.distance.pid),
		 pid_get_value_D(&mainboard.distance.pid) * pid_get_gain_D(&mainboard.distance.pid),
		 pid_get_value_out(&mainboard.distance.pid),
		 pid_get_value_in(&mainboard.angle.pid) * pid_get_gain_P(&mainboard.angle.pid),
		 pid_get_value_I(&mainboard.angle.pid) * pid_get_gain_I(&mainboard.angle.pid),
		 pid_get_value_D(&mainboard.angle.pid) * pid_get_gain_D(&mainboard.angle.pid),
		 pid_get_value_out(&mainboard.angle.pid));
}

#define PRINT_POS       (1<<0)
#define PRINT_PID       (1<<1)
#define PRINT_CS        (1<<2)
#define PRINT_SENSORS   (1<<3)
#define PRINT_TIME      (1<<4)
#define PRINT_BLOCKING  (1<<5)

static void cmd_interact_parsed(void * parsed_result, void * data)
{
	int c;
	int8_t cmd;
	uint8_t print = 0;
	struct vt100 vt100;

	vt100_init(&vt100);

	printf_P(PSTR("Display debugs:\r\n"
		      "  1:pos\r\n"
		      "  2:pid\r\n"
		      "  3:cs\r\n"
		      "  4:sensors\r\n"
		      "  5:time\r\n"
		      /* "  6:blocking\r\n" */
		      "Commands:\r\n"
		      "  arrows:move\r\n"
		      "  space:stop\r\n"
		      "  q:quit\r\n"));

	/* stop motors */
	mainboard.flags &= (~DO_CS);
	dac_set_and_save(LEFT_MOTOR, 0);
	dac_set_and_save(RIGHT_MOTOR, 0);

	while(1) {
		if (print & PRINT_POS) {
			print_pos();
		}

		if (print & PRINT_PID) {
			print_pid();
		}

		if (print & PRINT_CS) {
			print_cs();
		}

		if (print & PRINT_SENSORS) {
			print_sensors();
		}

		if (print & PRINT_TIME) {
			print_time();
		}
/* 		if (print & PRINT_BLOCKING) { */
/* 			printf_P(PSTR("%s %s blocking=%d\r\n"),  */
/* 				 mainboard.blocking ? "BLOCK1":"      ", */
/* 				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ", */
/* 				 rs_get_blocking(&mainboard.rs)); */
/* 		} */

		c = cmdline_getchar();
		if (c == -1) {
			wait_ms(10);
			continue;
		}
		cmd = vt100_parser(&vt100, c);
		if (cmd == -2) {
			wait_ms(10);
			continue;
		}

		if (cmd == -1) {
			switch(c) {
			case '1': print ^= PRINT_POS; break;
			case '2': print ^= PRINT_PID; break;
			case '3': print ^= PRINT_CS; break;
			case '4': print ^= PRINT_SENSORS; break;
			case '5': print ^= PRINT_TIME; break;
			case '6': print ^= PRINT_BLOCKING; break;

			case 'q':
				if (mainboard.flags & DO_CS)
					strat_hardstop();
				dac_set_and_save(LEFT_MOTOR, 0);
				dac_set_and_save(RIGHT_MOTOR, 0);
				return;
			case ' ':
				dac_set_and_save(LEFT_MOTOR, 0);
				dac_set_and_save(RIGHT_MOTOR, 0);
				break;
			default:
				break;
			}
		}
		else {
#ifdef HOST_VERSION
#define PWM_INTERACT 1200
#else
#define PWM_INTERACT 1200
#endif
			switch(cmd) {
			case KEY_UP_ARR:
				dac_set_and_save(LEFT_MOTOR, PWM_INTERACT);
				dac_set_and_save(RIGHT_MOTOR, PWM_INTERACT);
				break;
			case KEY_LEFT_ARR:
				dac_set_and_save(LEFT_MOTOR, -PWM_INTERACT);
				dac_set_and_save(RIGHT_MOTOR, PWM_INTERACT);
				break;
			case KEY_DOWN_ARR:
				dac_set_and_save(LEFT_MOTOR, -PWM_INTERACT);
				dac_set_and_save(RIGHT_MOTOR, -PWM_INTERACT);
				break;
			case KEY_RIGHT_ARR:
				dac_set_and_save(LEFT_MOTOR, PWM_INTERACT);
				dac_set_and_save(RIGHT_MOTOR, -PWM_INTERACT);
				break;
			}
		}
		wait_ms(10);
	}
}

prog_char str_interact_arg0[] = "interact";
parse_pgm_token_string_t cmd_interact_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);

prog_char help_interact[] = "Interactive mode";
parse_pgm_inst_t cmd_interact = {
	.f = cmd_interact_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_interact,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_interact_arg0,
		NULL,
	},
};



/**********************************************************/
/* Rs tests */

/* this structure is filled when cmd_rs is parsed successfully */
struct cmd_rs_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_rs is parsed successfully */
static void cmd_rs_parsed(void *parsed_result, void *data)
{
	//	struct cmd_rs_result *res = parsed_result;
	do {
		printf_P(PSTR("angle cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
			 cs_get_consign(&mainboard.angle.cs),
			 cs_get_filtered_feedback(&mainboard.angle.cs),
			 cs_get_out(&mainboard.angle.cs));
		printf_P(PSTR("distance cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
			 cs_get_consign(&mainboard.distance.cs),
			 cs_get_filtered_feedback(&mainboard.distance.cs),
			 cs_get_out(&mainboard.distance.cs));
		printf_P(PSTR("l=% .4"PRIi32" r=% .4"PRIi32"\r\n"), mainboard.pwm_l,
			 mainboard.pwm_r);
		wait_ms(100);
	} while(!cmdline_keypressed());
}

prog_char str_rs_arg0[] = "rs";
parse_pgm_token_string_t cmd_rs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
prog_char str_rs_arg1[] = "show";
parse_pgm_token_string_t cmd_rs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);

prog_char help_rs[] = "Show rs (robot system) values";
parse_pgm_inst_t cmd_rs = {
	.f = cmd_rs_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_rs,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_rs_arg0,
		(prog_void *)&cmd_rs_arg1,
		NULL,
	},
};

/**********************************************************/
/* Clitoid */

/* this structure is filled when cmd_clitoid is parsed successfully */
struct cmd_clitoid_result {
	fixed_string_t arg0;
	float alpha_deg;
	float beta_deg;
	float R_mm;
	float Vd;
	float Amax;
	float d_inter_mm;
};

/* function called when cmd_test is parsed successfully */
static void cmd_clitoid_parsed(void *parsed_result, void *data)
{
	struct cmd_clitoid_result *res = parsed_result;
/* 	clitoid(res->alpha_deg, res->beta_deg, res->R_mm, */
/* 		res->Vd, res->Amax, res->d_inter_mm); */
	double x = position_get_x_double(&mainboard.pos);
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);

	strat_set_speed(res->Vd, SPEED_ANGLE_FAST);
	trajectory_clitoid(&mainboard.traj, x, y, a, 150.,
			   res->alpha_deg, res->beta_deg, res->R_mm,
			   res->d_inter_mm);
}

prog_char str_clitoid_arg0[] = "clitoid";
parse_pgm_token_string_t cmd_clitoid_arg0 =
	TOKEN_STRING_INITIALIZER(struct cmd_clitoid_result,
				 arg0, str_clitoid_arg0);
parse_pgm_token_num_t cmd_clitoid_alpha_deg =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      alpha_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_beta_deg =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      beta_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_R_mm =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      R_mm, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Vd =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      Vd, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Amax =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      Amax, FLOAT);
parse_pgm_token_num_t cmd_clitoid_d_inter_mm =
	TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
			      d_inter_mm, FLOAT);

prog_char help_clitoid[] = "do a clitoid (alpha, beta, R, Vd, Amax, d_inter)";
parse_pgm_inst_t cmd_clitoid = {
	.f = cmd_clitoid_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_clitoid,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_clitoid_arg0,
		(prog_void *)&cmd_clitoid_alpha_deg,
		(prog_void *)&cmd_clitoid_beta_deg,
		(prog_void *)&cmd_clitoid_R_mm,
		(prog_void *)&cmd_clitoid_Vd,
		(prog_void *)&cmd_clitoid_Amax,
		(prog_void *)&cmd_clitoid_d_inter_mm,
		NULL,
	},
};

/**********************************************************/
/* Time_Monitor */

/* this structure is filled when cmd_time_monitor is parsed successfully */
struct cmd_time_monitor_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_time_monitor is parsed successfully */
static void cmd_time_monitor_parsed(void *parsed_result, void *data)
{
#ifndef HOST_VERSION
/*
	struct cmd_time_monitor_result *res = parsed_result;
	uint16_t seconds;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		eeprom_write_word(EEPROM_TIME_ADDRESS, 0);
	}
	seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
	printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
*/
#endif
}

prog_char str_time_monitor_arg0[] = "time_monitor";
parse_pgm_token_string_t cmd_time_monitor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg0, str_time_monitor_arg0);
prog_char str_time_monitor_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_time_monitor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg1, str_time_monitor_arg1);

prog_char help_time_monitor[] = "Show since how long we are running";
parse_pgm_inst_t cmd_time_monitor = {
	.f = cmd_time_monitor_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_time_monitor,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_time_monitor_arg0,
		(prog_void *)&cmd_time_monitor_arg1,
		NULL,
	},
};

/**********************************************************/
/* Sleep */

/* this structure is filled when cmd_sleep is parsed successfully */
struct cmd_sleep_result {
	fixed_string_t arg0;
	uint32_t ms;
};

/* function called when cmd_sleep is parsed successfully */
static void cmd_sleep_parsed(void *parsed_result, void *data)
{
	struct cmd_sleep_result *res = parsed_result;
	time_wait_ms(res->ms);
}

prog_char str_sleep_arg0[] = "sleep";
parse_pgm_token_string_t cmd_sleep_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sleep_result, arg0, str_sleep_arg0);
parse_pgm_token_num_t cmd_sleep_ms = TOKEN_NUM_INITIALIZER(struct cmd_sleep_result, ms, UINT32);

prog_char help_sleep[] = "Sleep during some miliseconds";
parse_pgm_inst_t cmd_sleep = {
	.f = cmd_sleep_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sleep,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sleep_arg0,
		(prog_void *)&cmd_sleep_ms,
		NULL,
	},
};
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/


/**********************************************************/
/* Strat_Event */

/* this structure is filled when cmd_strat_event is parsed successfully */
struct cmd_strat_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_strat_event is parsed successfully */
static void cmd_strat_event_parsed(void *parsed_result, void *data)
{
	struct cmd_strat_event_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("on")))
		strat_event_enable();
	else
		strat_event_disable();
}

prog_char str_strat_event_arg0[] = "strat_event";
parse_pgm_token_string_t cmd_strat_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_event_result, arg0, str_strat_event_arg0);
prog_char str_strat_event_arg1[] = "on#off";
parse_pgm_token_string_t cmd_strat_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_event_result, arg1, str_strat_event_arg1);

prog_char help_strat_event[] = "Enable/disable strat_event callback";
parse_pgm_inst_t cmd_strat_event = {
	.f = cmd_strat_event_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_strat_event,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_strat_event_arg0,
		(prog_void *)&cmd_strat_event_arg1,
		NULL,
	},
};



/**********************************************************/
/* status */

/* this structure is filled when cmd_status is parsed successfully */
struct cmd_status_result {
	fixed_string_t arg0;
	int16_t robot_x;
	int16_t robot_y;
	int16_t robot_a_abs;

	int16_t opp1_x;
	int16_t opp1_y;

	int16_t opp2_x;
	int16_t opp2_y;

	uint16_t checksum;
};


/* function called when cmd_opponent is parsed successfully */
static void cmd_status_parsed(void * parsed_result, void *data)
{
	struct cmd_status_result *res = parsed_result;
	uint8_t flags;
	int16_t checksum = 0;
	double x, y, a, d;

	/* received status from main robot */

	/* test checksum */
	checksum = res->robot_x + res->robot_y + res->robot_a_abs;
	checksum += res->opp1_x + res->opp1_y;
	checksum += res->opp2_x + res->opp2_y;

	if (res->checksum != (uint16_t)(checksum)) {
		ERROR (E_USER_BT_PROTO, "checksum ERROR");
		goto send_status;
	}

	/* get main robot possition */
	x = (int32_t)res->robot_x;
	y = (int32_t)res->robot_y;
	abs_xy_to_rel_da(x, y, &d, &a);

	IRQ_LOCK(flags);
	robot_2nd.x = res->robot_x;
	robot_2nd.y = res->robot_y;
	robot_2nd.a_abs = res->robot_a_abs;
	robot_2nd.a = DEG(a);
	robot_2nd.d = d;
	IRQ_UNLOCK(flags);

	/* get opponent 1 possition */
	x = (int32_t)res->opp1_x;
	y = (int32_t)res->opp1_y;
	abs_xy_to_rel_da(x, y, &d, &a);

	IRQ_LOCK(flags);
	robot_2nd.opponent1_x = res->opp1_x;
	robot_2nd.opponent1_y = res->opp1_y;
	robot_2nd.opponent1_a = DEG(a);
	robot_2nd.opponent1_d = d;
	IRQ_UNLOCK(flags);

	/* get opponent 2 possition */
	x = (int32_t)res->opp2_x;
	y = (int32_t)res->opp2_y;
	abs_xy_to_rel_da(x, y, &d, &a);

	IRQ_LOCK(flags);
	robot_2nd.opponent2_x = res->opp2_x;
	robot_2nd.opponent2_y = res->opp2_y;
	robot_2nd.opponent2_a = DEG(a);
	robot_2nd.opponent2_d = d;
	IRQ_UNLOCK(flags);

	/* send status of sencondary robot */
send_status:
	bt_send_status();
}


prog_char str_status_arg0[] = "status";
parse_pgm_token_string_t cmd_status_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_status_result, arg0, str_status_arg0);
parse_pgm_token_num_t cmd_status_robot_x = TOKEN_NUM_INITIALIZER(struct cmd_status_result, robot_x, INT16);
parse_pgm_token_num_t cmd_status_robot_y = TOKEN_NUM_INITIALIZER(struct cmd_status_result, robot_y, INT16);
parse_pgm_token_num_t cmd_status_robot_a_abs = TOKEN_NUM_INITIALIZER(struct cmd_status_result, robot_a_abs, INT16);
parse_pgm_token_num_t cmd_status_opp1_x = TOKEN_NUM_INITIALIZER(struct cmd_status_result, opp1_x, INT16);
parse_pgm_token_num_t cmd_status_opp1_y = TOKEN_NUM_INITIALIZER(struct cmd_status_result, opp1_y, INT16);
parse_pgm_token_num_t cmd_status_opp2_x = TOKEN_NUM_INITIALIZER(struct cmd_status_result, opp2_x, INT16);
parse_pgm_token_num_t cmd_status_opp2_y = TOKEN_NUM_INITIALIZER(struct cmd_status_result, opp2_y, INT16);
parse_pgm_token_num_t cmd_status_checksum = TOKEN_NUM_INITIALIZER(struct cmd_status_result, checksum, INT16);

prog_char help_status[] = "Get robot status (x,y,a_abs,checksum)";
parse_pgm_inst_t cmd_status = {
	.f = cmd_status_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_status,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_status_arg0, 
		(prog_void *)&cmd_status_robot_x, 
		(prog_void *)&cmd_status_robot_y, 
		(prog_void *)&cmd_status_robot_a_abs,
		(prog_void *)&cmd_status_opp1_x, 
		(prog_void *)&cmd_status_opp1_y, 
		(prog_void *)&cmd_status_opp2_x, 
		(prog_void *)&cmd_status_opp2_y, 
		(prog_void *)&cmd_status_checksum, 
		NULL,
	},
};

/**********************************************************/
/* Beacon */

/* this structure is filled when cmd_beacon is parsed successfully */
struct cmd_beacon_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_beacon is parsed successfully */
static void cmd_beacon_parsed(void *parsed_result, void *data)
{
	
	struct cmd_beacon_result *res = (struct cmd_beacon_result *) parsed_result;
	if (!strcmp_P(res->arg1, PSTR("on"))) {
		beacon_start();
	}
	else if (!strcmp_P(res->arg1, PSTR("off"))) {
		beacon_stop();
	}

	printf_P(PSTR("Done\r\n"));
}

prog_char str_beacon_arg0[] = "beacon";
parse_pgm_token_string_t cmd_beacon_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg0, str_beacon_arg0);
prog_char str_beacon_arg1[] = "on#off";
parse_pgm_token_string_t cmd_beacon_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_beacon_result, arg1, str_beacon_arg1);

prog_char help_beacon[] = "Enable/Disable Beacon (on/off)";
parse_pgm_inst_t cmd_beacon = {
	.f = cmd_beacon_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_beacon,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_beacon_arg0, 
		(prog_void *)&cmd_beacon_arg1, 
		NULL,
	},
};

/* TODO 2014 */
#if 0


/**********************************************************/
/* Robot sensors test */

/* this structure is filled when cmd_sensor is parsed successfully */
struct cmd_sensor_robot_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};


#define _BV64(i) ((uint64_t)1 << i)

/* function called when cmd_sensor is parsed successfully */
static void cmd_sensor_robot_parsed(void *parsed_result, void *data)
{
	struct cmd_sensor_robot_result *res = parsed_result;
	volatile int64_t sensor_prev = 0;
	uint8_t sensor_event = 0;
	uint8_t i;
	char sensor_string[15];

	if (!strcmp(res->arg1, "robot"))
	{	
		do {
			for (i=0; i<SENSOR_MAX; i++) {

				if(sensor_get(i) && ((sensor_prev & _BV64(i))==0) ){
					sensor_event = 1;
					sensor_prev |= _BV64(i);
				}
				else if (!sensor_get(i) && ((sensor_prev & _BV64(i))!=0)){
					sensor_event = 1;
					sensor_prev &= (~_BV64(i));
				}
			
				if(sensor_event){

					sensor_event = 0;

					switch(i){
						case S_START_SWITCH: strcpy(sensor_string, "START"); break;

						case S_TOKEN_FRONT_R: 			strcpy(sensor_string, "T_FRONT_R"); break;
						case S_TOKEN_FRONT_L: 			strcpy(sensor_string, "T_FRONT_L"); break;
						case S_TOKEN_FRONT_45R: 		strcpy(sensor_string, "T_FRONT_45R"); break;
						case S_TOKEN_FRONT_45L: 		strcpy(sensor_string, "T_FRONT_45L"); break;
						case S_TOKEN_FRONT_TOWER2H: 	strcpy(sensor_string, "S_TOKEN_FRONT_TOWER2H"); break;
						case S_TOKEN_FRONT_TOWER1H: 	strcpy(sensor_string, "S_TOKEN_FRONT_TOWER1H"); break;
						case S_TOKEN_FRONT_FIGURE: 	strcpy(sensor_string, "S_TOKEN_FRONT_FIGURE"); break;

						case S_TOKEN_REAR_R: 			strcpy(sensor_string, "T_REAR_R"); break;
						case S_TOKEN_REAR_L: 			strcpy(sensor_string, "T_REAR_L"); break;
						case S_TOKEN_REAR_45R: 			strcpy(sensor_string, "T_REAR_45R"); break;
						case S_TOKEN_REAR_45L: 			strcpy(sensor_string, "T_REAR_45L"); break;
						case S_TOKEN_REAR_TOWER2H: 	strcpy(sensor_string, "S_TOKEN_REAR_TOWER2H"); break;
						case S_TOKEN_REAR_TOWER1H: 	strcpy(sensor_string, "S_TOKEN_REAR_TOWER1H"); break;
						case S_TOKEN_REAR_FIGURE: 		strcpy(sensor_string, "S_TOKEN_REAR_FIGURE"); break;

						case S_OPPONENT_FRONT_R: 		strcpy(sensor_string, "OP_FRONT_R"); break;
						case S_OPPONENT_FRONT_L: 		strcpy(sensor_string, "OP_FRONT_L"); break;
						case S_OPPONENT_REAR_R: 		strcpy(sensor_string, "OP_REAR_R"); break;
						case S_OPPONENT_REAR_L: 		strcpy(sensor_string, "OP_REAR_L"); break;
						case S_OPPONENT_RIGHT: 			strcpy(sensor_string, "OP_RIGHT"); break;
						case S_OPPONENT_LEFT: 			strcpy(sensor_string, "OP_LEFT"); break;

						default: strcpy(sensor_string, "S_UNKNOW"); continue;
					}

					printf("%s %s!\n\r", sensor_string, (sensor_prev & _BV64(i))? "on":"off" ); 

				}
			}

			wait_ms(100);
		} while (!cmdline_keypressed());
	}
}

prog_char str_sensor_robot_arg0[] = "sensor";
parse_pgm_token_string_t cmd_sensor_robot_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_robot_result, arg0, str_sensor_robot_arg0);
prog_char str_sensor_robot_arg1[] = "robot";
parse_pgm_token_string_t cmd_sensor_robot_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_robot_result, arg1, str_sensor_robot_arg1);

prog_char help_sensor_robot[] = "Show robot sensors";
parse_pgm_inst_t cmd_sensor_robot = {
	.f = cmd_sensor_robot_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sensor_robot,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sensor_robot_arg0, 
		(prog_void *)&cmd_sensor_robot_arg1, 
		NULL,
	},
};

#endif /* TODO 2014*/




