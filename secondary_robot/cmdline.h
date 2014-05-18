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
 *  Revision : $Id: cmdline.h,v 1.3 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  cmdline.h,v 1.3 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _CMDLINE_H_
#define _CMDLINE_H_

/* uart rx callback for reset() */
void emergency(char c);

/* log function */
void mylog(struct error * e, ...);

/* launch cmdline */
void cmdline_interact(void *dummy);

/* init cmdline */
void cmdline_init(void);

/* execute cmdline without waiting, pull input commands */
void cmdline_interact_nowait(void *dummy);

/* detect a key */
static inline uint8_t cmdline_keypressed(void) {
	return (uart_recv_nowait(CMDLINE_UART) != -1);
}

/* get one char if received */
static inline int16_t cmdline_getchar(void) {
	return uart_recv_nowait(CMDLINE_UART);
}

/* wait receive char and get it */
static inline uint8_t cmdline_getchar_wait(void) {
	return uart_recv(CMDLINE_UART);
}

#endif
