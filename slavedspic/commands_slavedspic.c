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
 *  Revision : $Id$
 *
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_mechboard.c,v 1.5 2009/05/27 20:04:07 zer0 Exp
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <clock_time.h>
#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "../common/i2c_commands.h"

#include "sensor.h"
#include "cmdline.h"
#include "state.h"
#include "i2c_protocol.h"
#include "actuator.h"
#include "cs.h"
#include "main.h"

extern uint16_t state_debug;

struct cmd_event_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
};


/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	u08 bit=0;

	struct cmd_event_result * res = parsed_result;
	
	if (!strcmp_P(res->arg1, PSTR("all"))) {
		bit = 0xFF;
		if (!strcmp_P(res->arg2, PSTR("on")))
			slavedspic.flags |= bit;
		else if (!strcmp_P(res->arg2, PSTR("off")))
			slavedspic.flags &= (~bit);
		else { /* show */
			printf_P(PSTR("encoders is %s\r\n"), 
				 (DO_ENCODERS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("cs is %s\r\n"), 
				 (DO_CS & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("bd is %s\r\n"), 
				 (DO_BD & slavedspic.flags) ? "on":"off");
			printf_P(PSTR("power is %s\r\n"), 
				 (DO_POWER & slavedspic.flags) ? "on":"off");
		}
		return;
	}

	if (!strcmp_P(res->arg1, PSTR("encoders")))
		bit = DO_ENCODERS;
	if (!strcmp_P(res->arg1, PSTR("cs")))
		bit = DO_CS;
	if (!strcmp_P(res->arg1, PSTR("bd")))
		bit = DO_BD;
	if (!strcmp_P(res->arg1, PSTR("power")))
		bit = DO_POWER;

	if (!strcmp_P(res->arg2, PSTR("on")))
		slavedspic.flags |= bit;
	else if (!strcmp_P(res->arg2, PSTR("off"))) {
		slavedspic.flags &= (~bit);
	}
	printf_P(PSTR("%s is %s\r\n"), res->arg1, 
		      (bit & slavedspic.flags) ? "on":"off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#bd#power";
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
/* Color */

/* this structure is filled when cmd_color is parsed successfully */
struct cmd_color_result {
	fixed_string_t arg0;
	fixed_string_t color;
};

/* function called when cmd_color is parsed successfully */
static void cmd_color_parsed(void *parsed_result, __attribute__((unused)) void *data)
{
	struct cmd_color_result *res = (struct cmd_color_result *) parsed_result;
	if (!strcmp_P(res->color, PSTR("yellow"))) {
		slavedspic.our_color = I2C_COLOR_YELLOW;
	}
	else if (!strcmp_P(res->color, PSTR("red"))) {
		slavedspic.our_color = I2C_COLOR_RED;
	}
	printf_P(PSTR("Done\r\n"));
}

prog_char str_color_arg0[] = "color";
parse_pgm_token_string_t cmd_color_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_color_result, arg0, str_color_arg0);
prog_char str_color_color[] = "yellow#red";
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
/* lift */

/* this structure is filled when cmd_lift is parsed successfully */
struct cmd_lift_result {
	fixed_string_t arg0;
	int32_t arg1;
};

/* function called when cmd_lift is parsed successfully */
static void cmd_lift_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{

	struct cmd_lift_result *res = (struct cmd_lift_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	microseconds t1, t2;



	if (!strcmp_P(res->arg0, PSTR("lift"))) {
#if 1
		lift_set_height(res->arg1);
	
		t1 = time_get_us2();
		while (!lift_check_height_reached()) {
			t2 = time_get_us2();
			if (t2 - t1 > 20000) {
				dump_cs_debug("lift", &slavedspic.lift.cs);
				t1 = t2;
			}
		}
#else
	command.mode = I2C_SLAVEDSPIC_MODE_LIFT_HEIGHT;
	command.lift.height = res->arg1;
	state_set_mode(&command);
#endif

	}
	else if (!strcmp_P(res->arg0, PSTR("lift_calibrate")))
		lift_calibrate();
}

prog_char str_lift_arg0[] = "lift#lift_calibrate";
parse_pgm_token_string_t cmd_lift_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_lift_result, arg0, str_lift_arg0);
parse_pgm_token_num_t cmd_lift_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_lift_result, arg1, INT32);

prog_char help_lift[] = "set lift height";
parse_pgm_inst_t cmd_lift = {
	.f = cmd_lift_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_lift,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_lift_arg0, 
		(prog_void *)&cmd_lift_arg1,
		NULL,
	},
};


/**********************************************************/
/* vacuum */

/* this structure is filled when cmd_vacuum is parsed successfully */
struct cmd_vacuum_result {
	fixed_string_t arg0;
	uint8_t arg1;
    fixed_string_t arg2;
};

/* function called when cmd_vacuum is parsed successfully */
static void cmd_vacuum_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_vacuum_result *res = (struct cmd_vacuum_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;
	uint8_t on = 0;

	if (!strcmp_P(res->arg2, PSTR("on"))) on = 1;
	else on = 0;

	if (!strcmp_P(res->arg0, PSTR("ev")))
		vacuum_ev_set (res->arg1, on);

	else if (!strcmp_P(res->arg0, PSTR("motor_vaccum")))
		vacuum_motor_set (res->arg1, on);

	else if (!strcmp_P(res->arg0, PSTR("vacuum"))) {

		if (!strcmp_P(res->arg2, PSTR("on"))) 
			vacuum_system_enable (res->arg1);
		else
			vacuum_system_disable (res->arg1);
	}
}

prog_char str_vacuum_arg0[] = "ev#motor_vaccum#vacuum";
parse_pgm_token_string_t cmd_vacuum_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_vacuum_result, arg0, str_vacuum_arg0);
parse_pgm_token_num_t cmd_vacuum_arg1 = TOKEN_NUM_INITIALIZER(struct cmd_vacuum_result, arg1, UINT8);
prog_char str_vacuum_arg2[] = "on#off";
parse_pgm_token_string_t cmd_vacuum_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_vacuum_result, arg2, str_vacuum_arg2);

prog_char help_vacuum[] = "manage vaccum system (1 or 2)";
parse_pgm_inst_t cmd_vacuum = {
	.f = cmd_vacuum_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_vacuum,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_vacuum_arg0, 
		(prog_void *)&cmd_vacuum_arg1,
        (prog_void *)&cmd_vacuum_arg2,
		NULL,
	},
};


/**********************************************************/
/* boot */

/* this structure is filled when cmd_boot is parsed successfully */
struct cmd_boot_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_boot is parsed successfully */
static void cmd_boot_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_boot_result *res = (struct cmd_boot_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("door_open"))) {
		command.boot_door.mode = I2C_BOOT_DOOR_MODE_OPEN;
		command.mode = I2C_SLAVEDSPIC_MODE_BOOT_DOOR;
	}	
	else if (!strcmp_P(res->arg1, PSTR("door_close"))) {
		command.boot_door.mode = I2C_BOOT_DOOR_MODE_CLOSE;	
		command.mode = I2C_SLAVEDSPIC_MODE_BOOT_DOOR;
	}	
	else if (!strcmp_P(res->arg1, PSTR("tray_vibrate"))) {
		command.boot_tray.mode = I2C_BOOT_TRAY_MODE_VIBRATE;	
		command.mode = I2C_SLAVEDSPIC_MODE_BOOT_TRAY;
	}	
	else if (!strcmp_P(res->arg1, PSTR("tray_down"))) {
		command.boot_tray.mode = I2C_BOOT_TRAY_MODE_DOWN;	
		command.mode = I2C_SLAVEDSPIC_MODE_BOOT_TRAY;
	}	

	state_set_mode(&command);

}

prog_char str_boot_arg0[] = "boot";
parse_pgm_token_string_t cmd_boot_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_boot_result, arg0, str_boot_arg0);
prog_char str_boot_arg1[] = "door_open#door_close#tray_vibrate#tray_down";
parse_pgm_token_string_t cmd_boot_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_boot_result, arg1, str_boot_arg1);

prog_char help_boot[] = "set boot mode";
parse_pgm_inst_t cmd_boot = {
	.f = cmd_boot_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_boot,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_boot_arg0, 
		(prog_void *)&cmd_boot_arg1,
		NULL,
	},
};


/**********************************************************/
/* combs */

/* this structure is filled when cmd_combs is parsed successfully */
struct cmd_combs_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_combs is parsed successfully */
static void cmd_combs_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_combs_result *res = (struct cmd_combs_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("hide")))
		command.combs.mode = I2C_COMBS_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("open")))
		command.combs.mode = I2C_COMBS_MODE_OPEN;	
	else if (!strcmp_P(res->arg1, PSTR("harvest_close")))
		command.combs.mode = I2C_COMBS_MODE_HARVEST_CLOSE;	
	else if (!strcmp_P(res->arg1, PSTR("harvest_open")))
		command.combs.mode = I2C_COMBS_MODE_HARVEST_OPEN;	

	command.combs.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_COMBS;
	state_set_mode(&command);

}

prog_char str_combs_arg0[] = "combs";
parse_pgm_token_string_t cmd_combs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_combs_result, arg0, str_combs_arg0);
prog_char str_combs_arg1[] = "hide#open#harvest_open#harvest_close";
parse_pgm_token_string_t cmd_combs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_combs_result, arg1, str_combs_arg1);
parse_pgm_token_num_t cmd_combs_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_combs_result, arg2, INT8);

prog_char help_combs[] = "set combs mode, offset";
parse_pgm_inst_t cmd_combs = {
	.f = cmd_combs_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_combs,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_combs_arg0, 
		(prog_void *)&cmd_combs_arg1,
		(prog_void *)&cmd_combs_arg2,
		NULL,
	},
};

/**********************************************************/
/* tree tray */

/* this structure is filled when cmd_tree_tray is parsed successfully */
struct cmd_tree_tray_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_tree_tray is parsed successfully */
static void cmd_tree_tray_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_tree_tray_result *res = (struct cmd_tree_tray_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("open")))
		command.tree_tray.mode = I2C_TREE_TRAY_MODE_OPEN;
	else if (!strcmp_P(res->arg1, PSTR("close")))
		command.tree_tray.mode = I2C_TREE_TRAY_MODE_CLOSE;	
	else if (!strcmp_P(res->arg1, PSTR("harvest")))
		command.tree_tray.mode = I2C_TREE_TRAY_MODE_HARVEST;

	command.tree_tray.offset = res->arg2;	

	command.mode = I2C_SLAVEDSPIC_MODE_TREE_TRAY;
	state_set_mode(&command);

}

prog_char str_tree_tray_arg0[] = "tree_tray";
parse_pgm_token_string_t cmd_tree_tray_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_tree_tray_result, arg0, str_tree_tray_arg0);
prog_char str_tree_tray_arg1[] = "open#close#harvest";
parse_pgm_token_string_t cmd_tree_tray_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_tree_tray_result, arg1, str_tree_tray_arg1);
parse_pgm_token_num_t cmd_tree_tray_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_tree_tray_result, arg2, INT8);

prog_char help_tree_tray[] = "set tree_tray mode, offset";
parse_pgm_inst_t cmd_tree_tray = {
	.f = cmd_tree_tray_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_tree_tray,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_tree_tray_arg0, 
		(prog_void *)&cmd_tree_tray_arg1,
		(prog_void *)&cmd_tree_tray_arg2,
		NULL,
	},
};

/**********************************************************/
/* stick */

/* this structure is filled when cmd_stick is parsed successfully */
struct cmd_stick_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
};

/* function called when cmd_stick is parsed successfully */
static void cmd_stick_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_stick_result *res = (struct cmd_stick_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg0, PSTR("stick_left")))
		command.stick.type = I2C_STICK_TYPE_LEFT;
	else if (!strcmp_P(res->arg0, PSTR("stick_right")))
		command.stick.type = I2C_STICK_TYPE_RIGHT;	

	if (!strcmp_P(res->arg1, PSTR("hide")))
		command.stick.mode = I2C_STICK_MODE_HIDE;
	else if (!strcmp_P(res->arg1, PSTR("push_fire")))
		command.stick.mode = I2C_STICK_MODE_PUSH_FIRE;	
	else if (!strcmp_P(res->arg1, PSTR("push_torch")))
		command.stick.mode = I2C_STICK_MODE_PUSH_TORCH_FIRE;	
	else if (!strcmp_P(res->arg1, PSTR("clean_floor")))
		command.stick.mode = I2C_STICK_MODE_CLEAN_FLOOR;	
	else if (!strcmp_P(res->arg1, PSTR("clean_heart")))
		command.stick.mode = I2C_STICK_MODE_CLEAN_HEART;

	command.stick.offset = res->arg2;

	command.mode = I2C_SLAVEDSPIC_MODE_STICK;
	state_set_mode(&command);
}

prog_char str_stick_arg0[] = "stick_left#stick_right";
parse_pgm_token_string_t cmd_stick_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_stick_result, arg0, str_stick_arg0);
prog_char str_stick_arg1[] = "hide#push_fire#push_torch#clean_floor#clean_heart";
parse_pgm_token_string_t cmd_stick_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_stick_result, arg1, str_stick_arg1);
parse_pgm_token_num_t cmd_stick_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_stick_result, arg2, INT8);

prog_char help_stick[] = "set stick mode, offset";
parse_pgm_inst_t cmd_stick = {
	.f = cmd_stick_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_stick,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_stick_arg0, 
		(prog_void *)&cmd_stick_arg1,
		(prog_void *)&cmd_stick_arg2,
		NULL,
	},
};

/**********************************************************/
/* harvest fruits */

/* this structure is filled when cmd_harvest_fruits is parsed successfully */
struct cmd_harvest_fruits_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_harvest_fruits is parsed successfully */
static void cmd_harvest_fruits_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_harvest_fruits_result *res = (struct cmd_harvest_fruits_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("ready")))
		command.harvest_fruits.mode = I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_READY;
	else if (!strcmp_P(res->arg1, PSTR("do")))
		command.harvest_fruits.mode = I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_DO;	
	else if (!strcmp_P(res->arg1, PSTR("end")))
		command.harvest_fruits.mode = I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_END;	

	command.mode = I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS;
	state_set_mode(&command);
}

prog_char str_harvest_fruits_arg0[] = "harvest_fruits";
parse_pgm_token_string_t cmd_harvest_fruits_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_harvest_fruits_result, arg0, str_harvest_fruits_arg0);
prog_char str_harvest_fruits_arg1[] = "ready#do#end";
parse_pgm_token_string_t cmd_harvest_fruits_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_harvest_fruits_result, arg1, str_harvest_fruits_arg1);

prog_char help_harvest_fruits[] = "set harvest tree mode";
parse_pgm_inst_t cmd_harvest_fruits = {
	.f = cmd_harvest_fruits_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_harvest_fruits,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_harvest_fruits_arg0, 
		(prog_void *)&cmd_harvest_fruits_arg1,
		NULL,
	},
};

/**********************************************************/
/* dump fruits */

/* this structure is filled when cmd_dump_fruits is parsed successfully */
struct cmd_dump_fruits_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
};

/* function called when cmd_dump_fruits is parsed successfully */
static void cmd_dump_fruits_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
	struct cmd_dump_fruits_result *res = (struct cmd_dump_fruits_result *) parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp_P(res->arg1, PSTR("do")))
		command.dump_fruits.mode = I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO;
	else if (!strcmp_P(res->arg1, PSTR("end")))
		command.dump_fruits.mode = I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END;	

	command.mode = I2C_SLAVEDSPIC_MODE_DUMP_FRUITS;
	state_set_mode(&command);
}

prog_char str_dump_fruits_arg0[] = "dump_fruits";
parse_pgm_token_string_t cmd_dump_fruits_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_dump_fruits_result, arg0, str_dump_fruits_arg0);
prog_char str_dump_fruits_arg1[] = "do#end";
parse_pgm_token_string_t cmd_dump_fruits_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_dump_fruits_result, arg1, str_dump_fruits_arg1);

prog_char help_dump_fruits[] = "set dump fruits mode";
parse_pgm_inst_t cmd_dump_fruits = {
	.f = cmd_dump_fruits_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_dump_fruits,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_dump_fruits_arg0, 
		(prog_void *)&cmd_dump_fruits_arg1,
		NULL,
	},
};



/**********************************************************/
/* State2 */

/* this structure is filled when cmd_state2 is parsed successfully */
struct cmd_state2_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	fixed_string_t arg2;
	uint8_t arg3;
	
};

/* function called when cmd_state2 is parsed successfully */
static void cmd_state2_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state2_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	if (!strcmp(res->arg1, "init")) {
		command.mode = I2C_SLAVEDSPIC_MODE_INIT;
		state_set_mode(&command);
	}	
	else if (!strcmp(res->arg1, "power_off")) {
		command.mode = I2C_SLAVEDSPIC_MODE_POWER_OFF;
		state_set_mode(&command);
	}
	else if (!strcmp(res->arg1, "status")) {

		printf("not implemented");
#if 0
		/* actuators blocking flags */
		printf("fingers_floor_blocked = %d\r\n", slavedspic.fingers_floor.blocking);
		printf("fingers_totem_blocked = %d\r\n", slavedspic.fingers_totem.blocking);
		printf("arm_right_blocked = %d\r\n", slavedspic.arm_right.blocking);
		printf("arm_left_blocked = %d\r\n", slavedspic.arm_left.blocking);
		printf("lift_blocked = %d\r\n", slavedspic.lift.blocking);
	
		/* sensors */
		printf("turbine_sensors = %d\r\n", sensor_get_all());
	
		/* infos */
		printf("status = %s\r\n", slavedspic.status == I2C_SLAVEDSPIC_STATUS_BUSY? "BUSY":"READY");
	
		printf("harvest_mode = %d\r\n", slavedspic.harvest_mode);
		printf("store_mode = %d\r\n", slavedspic.store_mode);
		printf("dump_mode = %d\r\n", slavedspic.dump_mode);
	
		printf("nb_goldbars_in_boot = %d\r\n", slavedspic.nb_goldbars_in_boot);
		printf("nb_goldbars_in_mouth = %d\r\n", slavedspic.nb_goldbars_in_mouth);
		printf("nb_coins_in_boot = %d\r\n", slavedspic.nb_coins_in_boot);
		printf("nb_coins_in_mouth = %d\r\n", slavedspic.nb_coins_in_mouth);
#endif
	}
}

prog_char str_state2_arg0[] = "state";
parse_pgm_token_string_t cmd_state2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg0, str_state2_arg0);
prog_char str_state2_arg1[] = "init#power_off#status";
parse_pgm_token_string_t cmd_state2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state2_result, arg1, str_state2_arg1);

prog_char help_state2[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state2 = {
	.f = cmd_state2_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state2,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state2_arg0, 
		(prog_void *)&cmd_state2_arg1,
		NULL,
	},
};

#if 0

/**********************************************************/
/* State3 */

/* this structure is filled when cmd_state3 is parsed successfully */
struct cmd_state3_result {
	fixed_string_t arg0;
	fixed_string_t arg1;
	int8_t arg2;
	
};

/* function called when cmd_state3 is parsed successfully */
static void cmd_state3_parsed(void *parsed_result,
			      __attribute__((unused)) void *data)
{
	struct cmd_state3_result *res = parsed_result;
	struct i2c_cmd_slavedspic_set_mode command;

	command.set_infos.nb_goldbars_in_boot = -1;
	command.set_infos.nb_goldbars_in_mouth = -1;
	command.set_infos.nb_coins_in_boot = -1;
	command.set_infos.nb_coins_in_mouth = -1;

	if (!strcmp(res->arg1, "nb_goldbars_in_boot")) {
		command.set_infos.nb_goldbars_in_boot = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_goldbars_in_mouth")) {
		command.set_infos.nb_goldbars_in_mouth = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_coins_in_boot")) {
		command.set_infos.nb_coins_in_boot = res->arg2;
	}
	else if (!strcmp(res->arg1, "nb_coins_in_mouth")) {
		command.set_infos.nb_coins_in_mouth = res->arg2;
	}

	command.mode = I2C_SLAVEDSPIC_MODE_SET_INFOS;
	state_set_mode(&command);
}

prog_char str_state3_arg0[] = "set_infos";
parse_pgm_token_string_t cmd_state3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg0, str_state3_arg0);
prog_char str_state3_arg1[] = "nb_goldbars_in_boot#nb_goldbars_in_mouth#nb_coins_in_boot#nb_coins_in_mouth";
parse_pgm_token_string_t cmd_state3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_state3_result, arg1, str_state3_arg1);
parse_pgm_token_num_t cmd_state3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_state3_result, arg2, INT8);

prog_char help_state3[] = "set slavedspic mode";
parse_pgm_inst_t cmd_state3 = {
	.f = cmd_state3_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state3,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state3_arg0, 
		(prog_void *)&cmd_state3_arg1, 
		(prog_void *)&cmd_state3_arg2,
		NULL,
	},
};

#ifdef notyet
/**********************************************************/
/* State_Machine */

/* this structure is filled when cmd_state_machine is parsed successfully */
struct cmd_state_machine_result {
	fixed_string_t arg0;
};

/* function called when cmd_state_machine is parsed successfully */
static void cmd_state_machine_parsed(__attribute__((unused)) void *parsed_result,
				     __attribute__((unused)) void *data)
{
	state_machines();
}

prog_char str_state_machine_arg0[] = "state_machine";
parse_pgm_token_string_t cmd_state_machine_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_machine_result, arg0, str_state_machine_arg0);

prog_char help_state_machine[] = "launch state machine";
parse_pgm_inst_t cmd_state_machine = {
	.f = cmd_state_machine_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_machine,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_machine_arg0, 
		NULL,
	},
};

/**********************************************************/
/* State_Debug */

/* this structure is filled when cmd_state_debug is parsed successfully */
struct cmd_state_debug_result {
	fixed_string_t arg0;
	uint8_t on;
};

/* function called when cmd_state_debug is parsed successfully */
static void cmd_state_debug_parsed(void *parsed_result,
				   __attribute__((unused)) void *data)
{
	//struct cmd_state_debug_result *res = parsed_result;
	//state_debug = res->on;
}

prog_char str_state_debug_arg0[] = "state_debug";
parse_pgm_token_string_t cmd_state_debug_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_state_debug_result, arg0, str_state_debug_arg0);
parse_pgm_token_num_t cmd_state_debug_on = TOKEN_NUM_INITIALIZER(struct cmd_state_debug_result, on, UINT8);

prog_char help_state_debug[] = "Set debug timer for state machine";
parse_pgm_inst_t cmd_state_debug = {
	.f = cmd_state_debug_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_state_debug,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_state_debug_arg0, 
		(prog_void *)&cmd_state_debug_on, 
		NULL,
	},
};

#endif

/**********************************************************/
/* Test */

/* this structure is filled when cmd_test is parsed successfully */
struct cmd_test_result {
	fixed_string_t arg0;
};

/* function called when cmd_test is parsed successfully */
static void cmd_test_parsed(__attribute__((unused)) void *parsed_result,
			    __attribute__((unused)) void *data)
{
}

prog_char str_test_arg0[] = "test";
parse_pgm_token_string_t cmd_test_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_test_result, arg0, str_test_arg0);

prog_char help_test[] = "Test function";
parse_pgm_inst_t cmd_test = {
	.f = cmd_test_parsed,  /* function to call */
	.data = NULL,      /* 2nd arg of func */
	.help_str = help_test,
	.tokens = {        /* token list, NULL terminated */
		(prog_void *)&cmd_test_arg0, 
		NULL,
	},
};

#endif
