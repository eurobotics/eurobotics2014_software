/*
 *  Copyright Droids Corporation (2008)
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
 *  Revision : $Id: commands_gen.c,v 1.7 2009/05/27 20:04:07 zer0 Exp $
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_gen.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>
#include <encoders_dspic.h>

#include <scheduler.h>
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

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "wt11.h"

/**********************************************************/
/* Reset */

/* this structure is filled when cmd_reset is parsed successfully */
struct cmd_reset_result {
	fixed_string_t arg0;
};

/* function called when cmd_reset is parsed successfully */
static void cmd_reset_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
	hostsim_exit();
#else
	asm("Reset");
#endif
}

prog_char str_reset_arg0[] = "reset";
parse_pgm_token_string_t cmd_reset_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_reset_result, arg0, str_reset_arg0);

prog_char help_reset[] = "Reset the board";
parse_pgm_inst_t cmd_reset = {
	.f = cmd_reset_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_reset,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_reset_arg0, 
		NULL,
	},
};

///**********************************************************/
///* Bootloader */
//
///* this structure is filled when cmd_bootloader is parsed successfully */
//struct cmd_bootloader_result {
//	fixed_string_t arg0;
//};
//
///* function called when cmd_bootloader is parsed successfully */
//static void cmd_bootloader_parsed(void *parsed_result, void *data)
//{
//	bootloader();
//}
//
//prog_char str_bootloader_arg0[] = "bootloader";
//parse_pgm_token_string_t cmd_bootloader_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_bootloader_result, arg0, str_bootloader_arg0);
//
//prog_char help_bootloader[] = "Launch the bootloader";
//parse_pgm_inst_t cmd_bootloader = {
//	.f = cmd_bootloader_parsed,  /* function to call */
//	.data = NULL,      /* 2nd arg of func */
//	.help_str = help_bootloader,
//	.tokens = {        /* token list, NULL terminated */
//		(prog_void *)&cmd_bootloader_arg0, 
//		NULL,
//	},
//};

/**********************************************************/
/* Encoders tests */

/* this structure is filled when cmd_encoders is parsed successfully */
struct cmd_encoders_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_encoders is parsed successfully */
static void cmd_encoders_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_encoders_result *res = parsed_result;

	if (!strcmp_P(res->arg1, PSTR("reset"))) {
		
		encoders_dspic_set_value((void *)1, 0);
		encoders_dspic_set_value((void *)2, 0);
		return;
	}

	/* show */
	while(!cmdline_keypressed()) {
		//printf_P(PSTR("% .8ld % .8ld\r\n"), 
		printf("% .8ld % .8ld\r\n",
			 encoders_dspic_get_value((void *)1),
			 encoders_dspic_get_value((void *)2));

		wait_ms(50);
	}
#endif
}

prog_char str_encoders_arg0[] = "encoders";
parse_pgm_token_string_t cmd_encoders_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg0, str_encoders_arg0);
prog_char str_encoders_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_encoders_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_encoders_result, arg1, str_encoders_arg1);

prog_char help_encoders[] = "Show encoders values";
parse_pgm_inst_t cmd_encoders = {
	.f = cmd_encoders_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_encoders,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_encoders_arg0, 
		(prog_void *)&cmd_encoders_arg1, 
		NULL,
	},
};

/**********************************************************/
/* Scheduler show */

/* this structure is filled when cmd_scheduler is parsed successfully */
struct cmd_scheduler_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_scheduler is parsed successfully */
static void cmd_scheduler_parsed(void *parsed_result, void *data)
{
	scheduler_dump_events();
}

prog_char str_scheduler_arg0[] = "scheduler";
parse_pgm_token_string_t cmd_scheduler_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_scheduler_result, arg0, str_scheduler_arg0);
prog_char str_scheduler_arg1[] = "show";
parse_pgm_token_string_t cmd_scheduler_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_scheduler_result, arg1, str_scheduler_arg1);

prog_char help_scheduler[] = "Show scheduler events";
parse_pgm_inst_t cmd_scheduler = {
	.f = cmd_scheduler_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_scheduler,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_scheduler_arg0, 
		(prog_void *)&cmd_scheduler_arg1, 
		NULL,
	},
};

/**********************************************************/
/* pwm_servo tests */

/* this structure is filled when cmd_pwm is parsed successfully */
struct cmd_pwm_servo_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint16_t arg2;
};

/* function called when cmd_pwm is parsed successfully */
static void cmd_pwm_servo_parsed(void *parsed_result, void *show_range)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	void * pwm_servo_ptr = NULL;
	struct cmd_pwm_servo_result * res = parsed_result;

	if(show_range){
		if (!strcmp_P(res->arg1, PSTR("oc1")))
			printf("oc1 range = [0 ; %d]\r\n", gen.pwm_servo_oc1.range);
		else if (!strcmp_P(res->arg1, PSTR("oc2")))
			printf("oc2 range = [0 ; %d]\r\n", gen.pwm_servo_oc2.range);
	}
	else{
		if (!strcmp_P(res->arg1, PSTR("oc1")))
			pwm_servo_ptr = &gen.pwm_servo_oc1;
		else if (!strcmp_P(res->arg1, PSTR("oc2")))
			pwm_servo_ptr = &gen.pwm_servo_oc2;
	
		if (pwm_servo_ptr)
			pwm_servo_set(pwm_servo_ptr, res->arg2);
	
		printf("done\r\n");
	}
#endif
}

prog_char str_pwm_servo_arg0[] = "pwm_servo";
parse_pgm_token_string_t cmd_pwm_servo_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pwm_servo_result, arg0, str_pwm_servo_arg0);
prog_char str_pwm_servo_arg1[] = "oc1#oc2";
parse_pgm_token_string_t cmd_pwm_servo_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pwm_servo_result, arg1, str_pwm_servo_arg1);
parse_pgm_token_num_t cmd_pwm_servo_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pwm_servo_result, arg2, INT16);

prog_char help_pwm_servo[] = "Set pwm_servo values";
parse_pgm_inst_t cmd_pwm_servo = {
	.f = cmd_pwm_servo_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_pwm_servo,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pwm_servo_arg0, 
		(prog_void *)&cmd_pwm_servo_arg1, 
		(prog_void *)&cmd_pwm_servo_arg2, 
		NULL,
	},
};

/* show range */

/* this structure is filled when cmd_pwm_servo is parsed successfully */
struct cmd_pwm_servo_show_range_result {
	fixed_string_t arg0;
	fixed_string_t show_range;
	fixed_string_t arg1;
};
prog_char str_pwm_servo_show_range_arg[] = "show_range";
parse_pgm_token_string_t cmd_pwm_servo_show_range_arg = TOKEN_STRING_INITIALIZER(struct cmd_pwm_servo_show_range_result, show_range, str_pwm_servo_show_range_arg);

prog_char help_pwm_servo_show_range[] = "Show range values of pwm_servo channel";
parse_pgm_inst_t cmd_pwm_servo_show_range = {
	.f = cmd_pwm_servo_parsed,  /* function to call */
	.data = (void *)1,      /* 2nd arg of func */
	.help_str = help_pwm_servo_show_range,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_pwm_servo_arg0, 
		(prog_void *)&cmd_pwm_servo_show_range_arg,
		(prog_void *)&cmd_pwm_servo_arg1, 
		NULL,
	},
};

/**********************************************************/
/* dac_mc tests */

/* this structure is filled when cmd_dac_mc is parsed successfully */
struct cmd_dac_mc_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int32_t arg2;
};

/* function called when cmd_dac_mc is parsed successfully */
static void cmd_dac_mc_parsed(void * parsed_result, __attribute__((unused)) void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	void * dac_mc_ptr = NULL;
	struct cmd_dac_mc_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("left")))
		dac_mc_ptr = &gen.dac_mc_left;
	else if (!strcmp_P(res->arg1, PSTR("right")))
		dac_mc_ptr = &gen.dac_mc_right;

	
	if (dac_mc_ptr)
		dac_mc_set(dac_mc_ptr, res->arg2);

	printf_P(PSTR("done\r\n"));
#endif
}

prog_char str_dac_mc_arg0[] = "dac_mc";
parse_pgm_token_string_t cmd_dac_mc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_dac_mc_result, arg0, str_dac_mc_arg0);
prog_char str_dac_mc_arg1[] = "left#right";
parse_pgm_token_string_t cmd_dac_mc_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_dac_mc_result, arg1, str_dac_mc_arg1);
parse_pgm_token_num_t cmd_dac_mc_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_dac_mc_result, arg2, INT32);

prog_char help_dac_mc[] = "Set dac_mc values [-65535 ; 65535]";
parse_pgm_inst_t cmd_dac_mc = {
	.f = cmd_dac_mc_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_dac_mc,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_dac_mc_arg0, 
		(prog_void *)&cmd_dac_mc_arg1, 
		(prog_void *)&cmd_dac_mc_arg2, 
		NULL,
	},
};

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

/**********************************************************/
/* Adcs tests */

/* this structure is filled when cmd_adc is parsed successfully */
struct cmd_adc_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_adc is parsed successfully */
static void cmd_adc_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_adc_result *res = parsed_result;
	uint8_t i, loop = 0;

	if (!strcmp_P(res->arg1, PSTR("loop_show")))
		loop = 1;
	
	do {
		printf_P(PSTR("ADC values: "));
		for (i=0; i<ADC_MAX; i++) {
			printf_P(PSTR("%.4d "), sensor_get_adc(i));
		}
		printf_P(PSTR("\r\n"));
		wait_ms(100);
	} while (loop && !cmdline_keypressed());
#endif
}

#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS --------------------------------*/


prog_char str_adc_arg0[] = "adc";
parse_pgm_token_string_t cmd_adc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_adc_result, arg0, str_adc_arg0);
prog_char str_adc_arg1[] = "show#loop_show";
parse_pgm_token_string_t cmd_adc_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_adc_result, arg1, str_adc_arg1);

prog_char help_adc[] = "Show adc values";
parse_pgm_inst_t cmd_adc = {
	.f = cmd_adc_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_adc,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_adc_arg0, 
		(prog_void *)&cmd_adc_arg1, 
		NULL,
	},
};


/**********************************************************/
/* Sensors tests */

/* this structure is filled when cmd_sensor is parsed successfully */
struct cmd_sensor_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_sensor is parsed successfully */
static void cmd_sensor_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_sensor_result *res = parsed_result;
	uint8_t i, loop = 0;

	if (!strcmp_P(res->arg1, PSTR("loop_show")))
		loop = 1;
	
	do {
		printf_P(PSTR("SENSOR values: \r\n"));
		for (i=0; i<SENSOR_MAX; i++) {

			if(i<8)				
				printf_P(PSTR("S_CAP%.2d = %d "), i, !!sensor_get(i));
			else if (i<16)
				printf_P(PSTR("S_GPI%.2d = %d "), (i-8), !!sensor_get(i));
			else if (i<24)
				printf_P(PSTR("S_GPI%.2d = %d "), 10+(i-16), !!sensor_get(i));
			else if (i<32)
				printf_P(PSTR("S_GPI%.2d = %d "), 20+(i-24), !!sensor_get(i));
			else
				printf_P(PSTR("S_GPI%.2d = %d "), 30+(i-32), !!sensor_get(i));
			
			if((i+1)%8 == 0)
				printf("\n\r");
		}
		printf_P(PSTR("\r\n"));
		wait_ms(100);
	} while (loop && !cmdline_keypressed());
#endif
}

prog_char str_sensor_arg0[] = "sensor";
parse_pgm_token_string_t cmd_sensor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_result, arg0, str_sensor_arg0);
prog_char str_sensor_arg1[] = "show#loop_show";
parse_pgm_token_string_t cmd_sensor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_sensor_result, arg1, str_sensor_arg1);

prog_char help_sensor[] = "Show sensor values";
parse_pgm_inst_t cmd_sensor = {
	.f = cmd_sensor_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_sensor,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_sensor_arg0, 
		(prog_void *)&cmd_sensor_arg1, 
		NULL,
	},
};


/**********************************************************/
/* wt11 */

/* this structure is filled when cmd_sleep is parsed successfully */
struct cmd_wt11_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_wt11 is parsed successfully */
static void cmd_wt11_parsed(void *parsed_result, void *data)
{
#ifdef HOST_VERSION
	printf("not implemented\n");
#else
	struct cmd_wt11_result *res = parsed_result;
  	int16_t c;
	struct vt100 vt100;
	int8_t cmd = 0;
	static uint8_t beacon_link_id = 0;


#ifdef debug_wt11_commands
	static uint8_t beacon_addr [] = {0x00, 0x07 ,0x80, 0x85, 0x04, 0x70};
#endif
	
	if(!strcmp_P(res->arg1, "raw")) {

		/* init vt100 character set */
		vt100_init(&vt100);
		
		wt11_flush ();

		/* interact */
		while(cmd != KEY_CTRL_C) 
		{
			/* link --> cmd line */
			//if((c = uart_recv_nowait(MUX_UART))!= -1)
			//		uart_send_nowait(CMDLINE_UART, c);

			wt11_bypass_to_stdo (beacon_link_id);
			
			/* cmd line --> link */
			c = cmdline_getchar();
			if (c == -1) {
				continue;
			}

			/* check exit cmd */
			cmd = vt100_parser(&vt100, c);

			/* send to link */
			//uart_send_nowait(MUX_UART,c);	
			wt11_send_mux(beacon_link_id, (uint8_t *)&c, 1);
		}

  }
  else if (!strcmp_P(res->arg1, PSTR("reset"))) {
      //wt11_reset();
      wt11_reset_mux();
  }
#ifdef debug_wt11_commands
  else if (!strcmp_P(res->arg1, PSTR("mode_mux"))) {
      wt11_enable_mux_mode();
  }
  else if (!strcmp_P(res->arg1, PSTR("mode_normal"))) {
      wt11_disable_mux_mode();
  }
  else if (!strcmp_P(res->arg1, PSTR("mode_normal"))) {
      wt11_disable_mux_mode();
  }
  else if (!strcmp_P(res->arg1, PSTR("open_link"))) {
      wt11_open_link_mux(beacon_addr, &beacon_link_id);
  }
  else if (!strcmp_P(res->arg1, PSTR("close_link"))) {
      wt11_close_link_mux(beacon_link_id);
  }
#endif
#endif
}

prog_char str_wt11_arg0[] = "wt11";
parse_pgm_token_string_t cmd_wt11_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_wt11_result, arg0, str_wt11_arg0);
//prog_char str_wt11_arg1[] = "raw#reset#mode_mux#mode_normal#open_link#close_link";
prog_char str_wt11_arg1[] = "raw#reset";
parse_pgm_token_string_t cmd_wt11_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_wt11_result, arg1, str_wt11_arg1);

prog_char help_wt11[] = "wt11 functions";
parse_pgm_inst_t cmd_wt11 = {
	.f = cmd_wt11_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_wt11,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_wt11_arg0,
		(prog_void *)&cmd_wt11_arg1,
		NULL,
	},
};


/**********************************************************/
/* Log */

/* this structure is filled when cmd_log is parsed successfully */
struct cmd_log_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	uint8_t arg2;
	fixed_string_t arg3;
};

/* keep it sync with string choice */
static const prog_char uart_log[] = "uart";
static const prog_char rs_log[] = "rs";
static const prog_char traj_log[] = "traj";
//static const prog_char i2c_log[] = "i2c";
static const prog_char oa_log[] = "oa";
static const prog_char strat_log[] = "strat";
static const prog_char i2cproto_log[] = "i2cproto";
static const prog_char sensor_log[] = "sensor";
static const prog_char block_log[] = "bd";
static const prog_char cs_log[] = "cs";
static const prog_char btproto_log[] = "btproto";

struct log_name_and_num {
	const prog_char * name;
	uint8_t num;
};

static const struct log_name_and_num log_name_and_num[] = {
	{ uart_log, E_UART },
	{ rs_log, E_ROBOT_SYSTEM },
	{ traj_log, E_TRAJECTORY },
//	{ i2c_log, E_I2C },
	{ oa_log, E_OA },
	{ strat_log, E_USER_STRAT },
	{ i2cproto_log, E_USER_I2C_PROTO },
	{ sensor_log, E_USER_SENSOR },
	{ block_log, E_BLOCKING_DETECTION_MANAGER },
	{ cs_log, E_USER_CS },
	{ btproto_log, E_USER_BT_PROTO },
};

static uint8_t
log_name2num(const char * s)
{
	uint8_t i;
	
	for (i=0; i<sizeof(log_name_and_num)/sizeof(struct log_name_and_num); i++) {
		if (!strcmp_P(s, log_name_and_num[i].name)) {
			return log_name_and_num[i].num;
		}
	}
	return 0;
}

const prog_char *
log_num2name(uint8_t num)
{
	uint8_t i;
	
	for (i=0; i<sizeof(log_name_and_num)/sizeof(struct log_name_and_num); i++) {
		if (num ==  log_name_and_num[i].num) {
			return log_name_and_num[i].name;
		}
	}
	return NULL;
}

/* function called when cmd_log is parsed successfully */
static void cmd_log_do_show(void)
{
	uint8_t i, empty=1;
	const prog_char * name;

	printf_P(PSTR("log level is %d\r\n"), gen.log_level);
	for (i=0; i<NB_LOGS; i++) {
		name = log_num2name(gen.logs[i]);
		if (name) {
			printf_P(PSTR("log type %s is on\r\n"), name);
			empty = 0;
		}
	}
	if (empty)
		//printf_P(PSTR("no log configured\r\n"), gen.logs[i]);
		printf_P(PSTR("no log configured\r\n"));
}

/* function called when cmd_log is parsed successfully */
static void cmd_log_parsed(void * parsed_result, void * data)
{
	struct cmd_log_result *res = (struct cmd_log_result *) parsed_result;

	if (!strcmp_P(res->arg1, PSTR("level"))) {
		gen.log_level = res->arg2;
	}

	/* else it is a show */
	cmd_log_do_show();
}

prog_char str_log_arg0[] = "log";
parse_pgm_token_string_t cmd_log_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg0, str_log_arg0);
prog_char str_log_arg1[] = "level";
parse_pgm_token_string_t cmd_log_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg1, str_log_arg1);
parse_pgm_token_num_t cmd_log_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_log_result, arg2, INT32);

prog_char help_log[] = "Set log options: level (0 -> 5)";
parse_pgm_inst_t cmd_log = {
	.f = cmd_log_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_log,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_log_arg0, 
		(prog_void *)&cmd_log_arg1, 
		(prog_void *)&cmd_log_arg2, 
		NULL,
	},
};

prog_char str_log_arg1_show[] = "show";
parse_pgm_token_string_t cmd_log_arg1_show = TOKEN_STRING_INITIALIZER(struct cmd_log_result, arg1, str_log_arg1_show);

prog_char help_log_show[] = "Show configured logs";
parse_pgm_inst_t cmd_log_show = {
	.f = cmd_log_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_log_show,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_log_arg0, 
		(prog_void *)&cmd_log_arg1_show, 
		NULL,
	},
};

/* this structure is filled when cmd_log is parsed successfully */
struct cmd_log_type_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	fixed_string_t arg3;
};

/* function called when cmd_log is parsed successfully */
static void cmd_log_type_parsed(void * parsed_result, void * data)
{
	struct cmd_log_type_result *res = (struct cmd_log_type_result *) parsed_result;
	uint8_t lognum;
	uint8_t i;
	
	lognum = log_name2num(res->arg2);
	if (lognum == 0) {
		printf_P(PSTR("Cannot find log num\r\n"));
		return;
	}

	if (!strcmp_P(res->arg3, PSTR("on"))) {
		for (i=0; i<NB_LOGS; i++) {
			if (gen.logs[i] == lognum) {
				printf_P(PSTR("Already on\r\n"));
				return;
			}
		}
		for (i=0; i<NB_LOGS; i++) {
			if (gen.logs[i] == 0) {
				gen.logs[i] = lognum;
				break;
			}
		}
		if (i==NB_LOGS) {
			printf_P(PSTR("no more room\r\n"));
		}
	}
	else if (!strcmp_P(res->arg3, PSTR("off"))) {
		for (i=0; i<NB_LOGS; i++) {
			if (gen.logs[i] == lognum) {
				gen.logs[i] = 0;
				break;
			}
		}
		if (i==NB_LOGS) {
			printf_P(PSTR("already off\r\n"));
		}
	}
	cmd_log_do_show();
}

prog_char str_log_arg1_type[] = "type";
parse_pgm_token_string_t cmd_log_arg1_type = TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg1, str_log_arg1_type);
/* keep it sync with log_name_and_num above */
prog_char str_log_arg2_type[] = "uart#rs#traj#oa#ext#sensor#cs#bd#btproto";
parse_pgm_token_string_t cmd_log_arg2_type = TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg2, str_log_arg2_type);
prog_char str_log_arg3[] = "on#off";
parse_pgm_token_string_t cmd_log_arg3 = TOKEN_STRING_INITIALIZER(struct cmd_log_type_result, arg3, str_log_arg3);

prog_char help_log_type[] = "Set log type";
parse_pgm_inst_t cmd_log_type = {
	.f = cmd_log_type_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_log_type,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_log_arg0,
		(prog_void *)&cmd_log_arg1_type,
		(prog_void *)&cmd_log_arg2_type,
		(prog_void *)&cmd_log_arg3,
		NULL,
	},
};


