/*  
 *  Copyright Droids Corporation
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
 *  Revision : $Id: cmdline.c,v 1.2 2009/04/07 20:03:48 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  cmdline.c,v 1.2 2009/04/07 20:03:48 zer0 Exp.
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include <aversive.h>
#include <aversive/error.h>

#include <parse.h>
#include <rdline.h>
#include <uart.h>
#include <pwm_mc.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>

#include "main.h"
#include "cmdline.h"


/* See in commands.c for the list of commands. */
extern parse_pgm_ctx_t main_ctx[];

static void write_char(char c) 
{
	uart_send(CMDLINE_UART, c);
}

/* rdline execute this function when has a line valid */
static void valid_buffer(const char *buf, uint8_t size) 
{
	int8_t ret;
	ret = parse(main_ctx, buf);
	if (ret == PARSE_AMBIGUOUS)
		printf_P(PSTR("Ambiguous command\r\n"));
	else if (ret == PARSE_NOMATCH)
		printf_P(PSTR("Command not found\r\n"));
	else if (ret == PARSE_BAD_ARGS)
		printf_P(PSTR("Bad arguments\r\n"));
}

/* rdline execute this function when autocompleted is demanded (TAB key) */
static int8_t complete_buffer(const char *buf, char *dstbuf, uint8_t dstsize,
		int16_t *state)
{
	return complete(main_ctx, buf, state, dstbuf, dstsize);
}


/* sending "pop" on cmdline uart resets the beacon */
void emergency(char c)
{
	static uint8_t i = 0;
	
	if ((i == 0 && c == 'p') ||
	    (i == 1 && c == 'o') ||
	    (i == 2 && c == 'p')) 
		i++;
	else if ( !(i == 1 && c == 'p') )
		i = 0;
	if (i == 3)
		asm("Reset");
}

/* log function, add a command to configure it dynamically */
void mylog(struct error * e, ...) 
{
	va_list ap;
	//u16 stream_flags = stdout->flags;
	uint8_t i;
	time_h tv;

	if (e->severity > ERROR_SEVERITY_ERROR) {
		if (beaconboard.log_level < e->severity)
			return;
		
		for (i=0; i<NB_LOGS+1; i++)
			if (beaconboard.logs[i] == e->err_num)
				break;
		if (i == NB_LOGS+1)
			return;
	}

	va_start(ap, e);
	tv = time_get_time();
	printf_P(PSTR("%ld.%.3ld: "), tv.s, (tv.us/1000UL));
	vfprintf_P(stdout, e->text, ap);
	printf_P(PSTR("\r\n"));
	va_end(ap);
}

#if 0
/* init rdline and loop waiting for commands, never returns */
int cmdline_interact(void)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	rdline_init(&beaconboard.rdl, write_char, valid_buffer, complete_buffer);
	sprintf(beaconboard.prompt, "beaconboard > ");
	rdline_newline(&beaconboard.rdl, beaconboard.prompt);

	while (1) {
		c = uart_recv_nowait(CMDLINE_UART);
		if (c == -1) 
			continue;
		ret = rdline_char_in(&beaconboard.rdl, c);
		if (ret != 2 && ret != 0) {
			buffer = rdline_get_buffer(&beaconboard.rdl);
			history = rdline_get_history_item(&beaconboard.rdl, 0);
			if (history) {
				same = !memcmp(buffer, history, strlen(history)) &&
					buffer[strlen(history)] == '\n';
			}
			else
				same = 0;
			if (strlen(buffer) > 1 && !same)
				rdline_add_history(&beaconboard.rdl, buffer);
			rdline_newline(&beaconboard.rdl, beaconboard.prompt);
		}
	}

	return 0;
}
#endif

/* init cmdline */
void cmdline_init(void)
{
	rdline_init(&gen.rdl, write_char, valid_buffer, complete_buffer);	
	sprintf(gen.prompt, "beaconboard > ");	
	rdline_newline(&gen.rdl, gen.prompt);
}

/* execute cmdline waiting to input commands */
int cmdline_interact(void)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	while(1)
	{
		c = uart_recv_nowait(CMDLINE_UART);
		if (c == -1) 
			continue;
		ret = rdline_char_in(&gen.rdl, c);
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
	return 0;
}

/* execute cmdline without waiting, pull input commands */
int cmdline_interact_nowait(void)
{
	const char *history, *buffer;
	int8_t ret, same = 0;
	int16_t c;
	
	c = uart_recv_nowait(CMDLINE_UART);
	if (c == -1) 
		return -1;
	ret = rdline_char_in(&gen.rdl, c);
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

	return 0;
}

