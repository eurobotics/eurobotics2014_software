/*  
 *  Copyright Droids Corporation, 
 *  Olivier Matz <zer0@droids-corp.org>
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
 *  Revision : $Id: cmdline.c,v 1.6 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  cmdline.c,v 1.6 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#ifdef HOST_VERSION
#include <hostsim.h>
#endif

#include <parse.h>
#include <rdline.h>

#include <uart.h>

#include "main.h"
#include "robotsim.h"
#include "beacon.h"
#include "cmdline.h"
#include "strat_base.h"


/* see in commands.c for the list of commands. */
extern parse_pgm_ctx_t main_ctx[];

/* generic write char function */
static void write_char(char c) 
{
	uart_send(CMDLINE_UART, c);
#ifdef HOST_VERSION
	robotsim_uart_send_BT(c);
#endif

}

/* process commands */
static void valid_buffer(const char *buf, uint8_t size) 
{
	int8_t ret;

	/* reset CTRL-C for trajectory interruption each time we
	 * receive a new command */
	interrupt_traj_reset();

	ret = parse(main_ctx, buf);
	
	if (ret != PARSE_SUCCESS)
		bt_status_set_cmd_ack (END_ERROR);

	if (ret == PARSE_AMBIGUOUS)
		printf_P(PSTR("Ambiguous command\r\n"));
	else if (ret == PARSE_NOMATCH)
		printf_P(PSTR("Command not found\r\n"));
	else if (ret == PARSE_BAD_ARGS)
		printf_P(PSTR("Bad arguments\r\n"));
}

/* complete commands */
static int8_t complete_buffer(const char *buf, char *dstbuf, uint8_t dstsize,
		int16_t *state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}


/* sending "pop" on cmdline uart resets the robot */
void emergency(char c)
{
	static uint8_t i = 0;
	
	/* interrupt traj here */
	if (c == '\003')
		interrupt_traj();
	
	if ((i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p')) 
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if (i == 3)
#ifdef HOST_VERSION
		hostsim_exit();
#else
		asm("Reset");
#endif
}

/* log function, add a command to configure
 * it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;
#ifndef HOST_VERSION
#ifndef DSPIC
	u16 stream_flags = stdout->flags;
#endif
#endif
	uint8_t i;
	time_h tv;

	if (e->severity > ERROR_SEVERITY_ERROR) {
		if (gen.log_level < e->severity)
			return;
		
		for (i=0; i<NB_LOGS+1; i++)
			if (gen.logs[i] == e->err_num)
				break;
		if (i == NB_LOGS+1)
			return;
	}

	va_start(ap, e);
	tv = time_get_time();
	printf_P(PSTR("%ld.%.3ld: "), (long int)tv.s, (tv.us/1000UL));
	
	printf_P(PSTR("(%d,%d,%d) "),
		 position_get_x_s16(&mainboard.pos),
		 position_get_y_s16(&mainboard.pos),
		 position_get_a_deg_s16(&mainboard.pos));
	
	/* XXX not secure vfprintf */
	vfprintf(stdout, e->text, ap);

	printf_P(PSTR("\r\n"));
	va_end(ap);
#ifndef HOST_VERSION
#ifndef DSPIC
	stdout->flags = stream_flags;
#endif
#endif
}

/* user interact */
void cmdline_interact(void *dummy)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	rdline_init(&gen.rdl, write_char, valid_buffer, complete_buffer);
	sprintf(gen.prompt, "secondary > ");	
	rdline_newline(&gen.rdl, gen.prompt);

	while (1){

		/* get character */
		c = uart_recv_nowait(CMDLINE_UART);
	
		if (c == -1) 
#ifndef HOST_VERSION	
			continue;
#else
    {
		  c = robotsim_uart_recv_BT();
		  if (c == -1) 
			  //continue;
			  return 0;    
    }
#endif

		/* process character in */
		ret = rdline_char_in(&gen.rdl, c);

		/* if command executes save in history and print prompt */
		if (ret != 2 && ret != 0) {
			buffer = rdline_get_buffer(&gen.rdl);
			history = rdline_get_history_item(&gen.rdl, 0);
			if (history) {
				same = !memcmp(buffer, history, strlen(history)) &&
					buffer[strlen(history)] == '\n';
			}
			else
				same = 0;
			if (strlen(buffer) > 1 && !same)
				rdline_add_history(&gen.rdl, buffer);
			rdline_newline(&gen.rdl, gen.prompt);
		}
	}
}

/* init cmdline */
void cmdline_init(void)
{
	rdline_init(&gen.rdl, write_char, valid_buffer, complete_buffer);	
	sprintf(gen.prompt, "secondary > ");	
	rdline_newline(&gen.rdl, gen.prompt);
}

/* user interact */
void cmdline_interact_nowait (void *dummy)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	while (1)
	{

		/* get character */
		c = uart_recv_nowait(CMDLINE_UART);
		if (c == -1) 
#ifndef HOST_VERSION	
			return;
#else
    	{
			c = robotsim_uart_recv_BT();
			if (c == -1) 
			  return;    
    	}
#endif

		/* process character in */
		ret = rdline_char_in(&gen.rdl, c);

		/* if command executes save in history and print prompt */
		if (ret != 2 && ret != 0) {
			buffer = rdline_get_buffer(&gen.rdl);
			history = rdline_get_history_item(&gen.rdl, 0);
			if (history) {
				same = !memcmp(buffer, history, strlen(history)) &&
					buffer[strlen(history)] == '\n';
			}
			else
				same = 0;
			if (strlen(buffer) > 1 && !same)
				rdline_add_history(&gen.rdl, buffer);
			rdline_newline(&gen.rdl, gen.prompt);
		}
	}
}


