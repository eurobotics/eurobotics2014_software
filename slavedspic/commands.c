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
 *  commands.c,v 1.5 2009/05/27 20:04:07 zer0 Exp  
 */

#include <stdlib.h>
#include <aversive/pgmspace.h>
#include <parse.h>

//#define COMPILE_COMMANDS_CS
#define COMPILE_COMMANDS_AX12

/* commands_gen.c */
extern parse_pgm_inst_t cmd_reset;
//extern parse_pgm_inst_t cmd_bootloader;
extern parse_pgm_inst_t cmd_encoders;
extern parse_pgm_inst_t cmd_pwm_mc;
extern parse_pgm_inst_t cmd_pwm_servo;
extern parse_pgm_inst_t cmd_pwm_servo_show_range;
extern parse_pgm_inst_t cmd_dac_mc;
extern parse_pgm_inst_t cmd_sensor;
extern parse_pgm_inst_t cmd_rele;
extern parse_pgm_inst_t cmd_log;
extern parse_pgm_inst_t cmd_log_show;
extern parse_pgm_inst_t cmd_log_type;
extern parse_pgm_inst_t cmd_scheduler;


/* commands_cs.c */
#ifdef COMPILE_COMMANDS_CS

#include "./commands_cs.c"

extern parse_pgm_inst_t cmd_gain;
extern parse_pgm_inst_t cmd_gain_show;
extern parse_pgm_inst_t cmd_speed;
extern parse_pgm_inst_t cmd_speed_show;
extern parse_pgm_inst_t cmd_derivate_filter;
extern parse_pgm_inst_t cmd_derivate_filter_show;
extern parse_pgm_inst_t cmd_consign;
extern parse_pgm_inst_t cmd_maximum;
extern parse_pgm_inst_t cmd_maximum_show;
extern parse_pgm_inst_t cmd_quadramp;
extern parse_pgm_inst_t cmd_quadramp_show;
extern parse_pgm_inst_t cmd_cs_status;
extern parse_pgm_inst_t cmd_blocking_i;
extern parse_pgm_inst_t cmd_blocking_i_show;

#endif /* COMPILE_COMMANDS_CS */

/* commands_ax12.c */
#ifdef COMPILE_COMMANDS_AX12

#include "./commands_ax12.c"

//extern parse_pgm_inst_t cmd_baudrate;
extern parse_pgm_inst_t cmd_uint16_read;
extern parse_pgm_inst_t cmd_uint16_write;
extern parse_pgm_inst_t cmd_uint8_read;
extern parse_pgm_inst_t cmd_uint8_write;
extern parse_pgm_inst_t cmd_ax12_stress;
extern parse_pgm_inst_t cmd_ax12_dump_stats;
#endif /* COMPILE_COMMANDS_AX12 */

/* commands_slavedspic.c */

extern parse_pgm_inst_t cmd_event;
extern parse_pgm_inst_t cmd_color;
extern parse_pgm_inst_t cmd_lift;
extern parse_pgm_inst_t cmd_vacuum;
extern parse_pgm_inst_t cmd_boot;
extern parse_pgm_inst_t cmd_combs;
extern parse_pgm_inst_t cmd_tree_tray;
extern parse_pgm_inst_t cmd_stick;
extern parse_pgm_inst_t cmd_arm;
extern parse_pgm_inst_t cmd_arm_goto;

extern parse_pgm_inst_t cmd_harvest_fruits;
extern parse_pgm_inst_t cmd_dump_fruits;
extern parse_pgm_inst_t cmd_state2;


#if 0
//extern parse_pgm_inst_t cmd_state_debug;
//extern parse_pgm_inst_t cmd_state1;
extern parse_pgm_inst_t cmd_state3;
//extern parse_pgm_inst_t cmd_state_machine;
//extern parse_pgm_inst_t cmd_test;
#endif

/* in progmem */
parse_pgm_ctx_t main_ctx[]  = {

	/* commands_gen.c */
	(parse_pgm_inst_t *)&cmd_reset,
	//(parse_pgm_inst_t *)&cmd_bootloader,
	(parse_pgm_inst_t *)&cmd_encoders,
	(parse_pgm_inst_t *)&cmd_pwm_mc,
	(parse_pgm_inst_t *)&cmd_pwm_servo,
	(parse_pgm_inst_t *)&cmd_pwm_servo_show_range,
	(parse_pgm_inst_t *)&cmd_dac_mc,
	(parse_pgm_inst_t *)&cmd_sensor,
	(parse_pgm_inst_t *)&cmd_rele,
	(parse_pgm_inst_t *)&cmd_log,
	(parse_pgm_inst_t *)&cmd_log_show,
	(parse_pgm_inst_t *)&cmd_log_type,
	(parse_pgm_inst_t *)&cmd_scheduler,

#ifdef COMPILE_COMMANDS_CS

	/* commands_cs.c */
	(parse_pgm_inst_t *)&cmd_gain,
	(parse_pgm_inst_t *)&cmd_gain_show,
	(parse_pgm_inst_t *)&cmd_speed,
	(parse_pgm_inst_t *)&cmd_speed_show,
	(parse_pgm_inst_t *)&cmd_consign,
	(parse_pgm_inst_t *)&cmd_derivate_filter,
	(parse_pgm_inst_t *)&cmd_derivate_filter_show,
	(parse_pgm_inst_t *)&cmd_maximum,
	(parse_pgm_inst_t *)&cmd_maximum_show,
	(parse_pgm_inst_t *)&cmd_quadramp,
	(parse_pgm_inst_t *)&cmd_quadramp_show,
	(parse_pgm_inst_t *)&cmd_cs_status,
	(parse_pgm_inst_t *)&cmd_blocking_i,
	(parse_pgm_inst_t *)&cmd_blocking_i_show,

#endif /* COMPILE_COMMANDS_CS */

#ifdef COMPILE_COMMANDS_AX12
	/* commands_ax12.c */
	//(parse_pgm_inst_t *)&cmd_baudrate,
	(parse_pgm_inst_t *)&cmd_uint16_read,
	(parse_pgm_inst_t *)&cmd_uint16_write,
	(parse_pgm_inst_t *)&cmd_uint8_read,
	(parse_pgm_inst_t *)&cmd_uint8_write,
	(parse_pgm_inst_t *)&cmd_ax12_stress,
	(parse_pgm_inst_t *)&cmd_ax12_dump_stats,

#endif

	/* commands_slavedspic.c */
	(parse_pgm_inst_t *)&cmd_event,
	(parse_pgm_inst_t *)&cmd_color,
	(parse_pgm_inst_t *)&cmd_lift,
	(parse_pgm_inst_t *)&cmd_vacuum,
	(parse_pgm_inst_t *)&cmd_boot,
	(parse_pgm_inst_t *)&cmd_combs,
	(parse_pgm_inst_t *)&cmd_tree_tray,
	(parse_pgm_inst_t *)&cmd_stick,
	(parse_pgm_inst_t *)&cmd_arm,
	(parse_pgm_inst_t *)&cmd_arm_goto,

	(parse_pgm_inst_t *)&cmd_harvest_fruits,
	(parse_pgm_inst_t *)&cmd_dump_fruits,
	(parse_pgm_inst_t *)&cmd_state2,

#if 0

	//(parse_pgm_inst_t *)&cmd_state_debug,
	//(parse_pgm_inst_t *)&cmd_state1,
	(parse_pgm_inst_t *)&cmd_state2,	
	(parse_pgm_inst_t *)&cmd_state3,
	//(parse_pgm_inst_t *)&cmd_state_machine,
	//(parse_pgm_inst_t *)&cmd_test,
#endif
	NULL,
};
