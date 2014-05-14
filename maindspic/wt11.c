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
#include <ctype.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <clock_time.h>

#include "main.h"
#include "wt11.h"

#ifdef HOST_VERSION
#include "robotsim.h"
#endif



uint8_t data[WT11_MUX_LENGTH_MAX];

/* send data in norma mode, any protocol is used */
void wt11_send (uint8_t *buff, uint16_t length) 
{
  uint16_t i;

	for(i=0; i<length; i++){
#ifndef HOST_VERSION
		uart_send(BT_UART, buff[i]);
#else
		robotsim_uart_send_BT(buff[i])
#endif
	}	
}

/* send data using multiplexing mode protocol */
void wt11_send_mux (uint8_t link_id, uint8_t *buff, uint16_t length)
{
#define WT11_MUX_SOF        0xBF

#ifndef HOST_VERSION
#define __uart_send(x)  uart_send(BT_UART,x)
#else
#define __uart_send(x)  robotsim_uart_send_BT((uint8_t)x);
#endif

  uint16_t i=0;

  /* check length */
  if (length > WT11_MUX_LENGTH_MAX) {
    ERROR(E_USER_BT_PROTO, "BT_PROTO: ERROR, length > max");
    return;
  }

  /* start of frame */
  __uart_send(WT11_MUX_SOF);

  /* link ID */
  __uart_send(link_id);

  /* flags and length */
  __uart_send(length >> 8);
  __uart_send(length & 0xFF);

  /* data */
  for(i=0; i<length; i++)
    __uart_send(buff[i]);

  /* nLINK */
  __uart_send(link_id ^ 0xFF);  
}

/* flush recevied buffer */
void wt11_flush (void) {
	while (uart_recv_nowait(BT_UART) != -1);
}

/* receive data using multiplexing mode protocol, 
   returns data length, -1 if no data received */
int16_t wt11_recv_mux (uint8_t *link_id, uint8_t *buff, uint16_t buff_size)
{
  int16_t ret = 0;
  uint8_t c = 0;
  static uint8_t state = 0;
  static uint8_t __link_id = 0;
  static uint16_t __length, i = 0;

  do {

    /* get byte */
#ifndef HOST_VERSION
		ret = uart_recv_nowait(BT_UART);
#else
		ret = robotsim_uart_recv_BT();  
#endif
  
		if (ret == -1)
			return ret;

		/* cast to uint8_t */
		c = (uint8_t)(ret & 0x00FF);

		switch (state) {

			/* start of frame */
			case 0:
				if (c == WT11_MUX_SOF)
				 state ++;
				break;

			/* link ID */
			case 1:
				if ((((int8_t)c >= 0) && (c <=8)) || (c == WT11_MUX_CTRL_CMD)) {
				 __link_id = c;
				 state ++;
				}
				else
				 state = 0;
				break;
				 
			/* flags and length */
			case 2:
				__length = ((uint16_t)c << 8);
				state ++;
				break;

			case 3:
			  	__length |= ((uint16_t)c & 0x00FF);

				//DEBUG (E_USER_WT11, "WT11 (%d): length =  %d\n", __link_id, __length);
			  	state ++;
			 	i = 0;
			  	break;

			/* data */
			case 4:
				//printf ("%c", (char)c);

				if (i < buff_size)
			  		buff[i++] = c;
				else {
						state = 0;
						i = 0;
						ERROR (E_USER_WT11, "WT11: ERROR buffer overflow");
						wt11_flush();
						return -1;
				}

				if (i == __length)
					state ++;
#if 0
					i++;
			  
				if (i == __length) {

					if (i < buff_size)
						state ++;
					else {
						state = 0;
						i = 0;
						ERROR (E_USER_WT11, "WT11: ERROR buffer overflow");
						wt11_flush();
						return -1;
					}
				}
#endif
				break;

			/* nLINK */    
			case 5:
				if ((c ^ 0xFF) == (__link_id)) {
					*link_id = __link_id;
					state = 0;
					i = 0;

					//DEBUG (E_USER_WT11, "WT11 (%d): received %s", *link_id, data);
					return __length;
				}
				else {
					state = 0;
					i = 0;
					//ERROR (E_USER_WT11, "WT11: link ID ERROR");
					return -1;
				}
				break;
			  
			default:
				state = 0;
				i = 0;
				break;
		}

	} while (ret != -1);

	return -1;
}

/* receive data using multiplexing mode protocol and redirect to STDO,
   returns data length, -1 if no data received */
int16_t wt11_bypass_to_stdo (uint8_t link_id)
{
  int16_t ret = 0;
  uint8_t c = 0;
  static uint8_t state = 0;
  static uint8_t __link_id = 0;
  static uint16_t __length, i = 0;

  do {

    /* get byte */
#ifndef HOST_VERSION
		ret = uart_recv_nowait(BT_UART);
#else
		ret = robotsim_uart_recv_BT();  
#endif
  
		if (ret == -1)
			return ret;

		/* cast to uint8_t */
		c = (uint8_t)(ret & 0x00FF);

		switch (state) {

			/* start of frame */
			case 0:
				if (c == WT11_MUX_SOF)
				 state ++;
				break;

			/* link ID */
			case 1:
				if ((((int8_t)c >= 0) && (c <=8)) || (c == WT11_MUX_CTRL_CMD)) {
				 	__link_id = c;

					if (__link_id == link_id)
					 state ++;
					else
					 state = 0;
				}
				else
				 state = 0;

				break;
				 
			/* flags and length */
			case 2:
				__length = ((uint16_t)c << 8);
				state ++;
				break;

			case 3:
			  	__length |= ((uint16_t)c & 0x00FF);

				//DEBUG (E_USER_WT11, "WT11 (%d): length =  %d\n", __link_id, __length);
			  	state ++;
			 	i = 0;
			  	break;

			/* data */
			case 4:
				uart_send_nowait(STDIO_UART, c);
				i++;
			  
				if (i == __length)
					state ++;
	
				break;

			/* nLINK */    
			case 5:
				if ((c ^ 0xFF) == (__link_id)) {
					state = 0;
					i = 0;

					//DEBUG (E_USER_WT11, "WT11 (%d): received %s", *link_id, data);
					return __length;
				}
				else {
					state = 0;
					i = 0;

					//ERROR (E_USER_WT11, "WT11: link ID ERROR");
					return -1;
				}
				break;
			  
			default:
				state = 0;
				i = 0;
				break;
		}

	} while (ret != -1);

	return -1;
}

/* return received char and link_id, -1 if not valid char */
int16_t wt11_recv_mux_char (uint8_t *link_id)
{
  int16_t ret = 0;
  uint8_t c = 0;
  static uint8_t state = 0;
  static uint8_t __link_id = 0;
  static uint16_t __length, i = 0;

  do {

    /* get byte */
#ifndef HOST_VERSION
		ret = uart_recv_nowait(BT_UART);
#else
		ret = robotsim_uart_recv_BT();  
#endif
  
		if (ret == -1)
			return ret;

		/* cast to uint8_t */
		c = (uint8_t)(ret & 0x00FF);

		switch (state) {

			/* start of frame */
			case 0:
				if (c == WT11_MUX_SOF)
				 state ++;
				break;

			/* link ID */
			case 1:
				if ((((int8_t)c >= 0) && (c <=8)) || (c == WT11_MUX_CTRL_CMD)) {
				 	__link_id = c;
					*link_id = __link_id;
					state ++;
				}
				else
				 state = 0;

				break;
				 
			/* flags and length */
			case 2:
				__length = ((uint16_t)c << 8);
				state ++;
				break;

			case 3:
			  	__length |= ((uint16_t)c & 0x00FF);

				//DEBUG (E_USER_WT11, "WT11 (%d): length =  %d\n", __link_id, __length);
			  	state ++;
			 	i = 0;
			  	break;

			/* data */
			case 4:
				//uart_send_nowait(STDIO_UART, c);
				i++;
			  
				if (i >= __length)
					state ++;

				return c;
				break;

			/* nLINK */    
			case 5:
				if ((c ^ 0xFF) == (__link_id)) {
					state = 0;
					i = 0;

					//DEBUG (E_USER_WT11, "WT11 (%d): received %s", *link_id, data);
					return -1;
				}
				else {
					state = 0;
					i = 0;

					//ERROR (E_USER_WT11, "WT11: link ID ERROR");
					return -1;

				}
				break;
			  
			default:
				state = 0;
				i = 0;
				break;
		}

	} while (ret != -1);

	return -1;
}


/* receive data in data mode, 
   returns data length, -1 if no data received */
int16_t wt11_rdline (uint8_t *buff, uint16_t buff_size)
{
	static uint16_t i = 0;
	int16_t length;
	int16_t ret;
	uint8_t c;

	do {

	/* get byte */
#ifndef HOST_VERSION
	ret = uart_recv_nowait(BT_UART);
#else
	ret = robotsim_uart_recv_BT();  
#endif
  
	if (ret == -1)
	  return ret;
	
	/* cast to uint8_t */
	c = (uint8_t)(ret & 0x00FF);

	if (c == '\r' || c == '\n') {
	  if (i!=0) {			
		  	buff[i] = '\0';
		  	length = i;        
	  		i=0;
			return length;
	  }
	}
	else {
		buff[i++] = c;
	  	if (i >= buff_size)
	  		i = 0;
	}	

  } while (ret != -1);

  return -1;	
}

/* reset wt11 */
void __wt11_reset (uint8_t mode)
{
	uint16_t i=0;
	uint16_t ret = 0;

	if (mode == WT11_MODE_NORMAL)
	{
		/* change to cmd mode */
		wait_ms(1200);
		wt11_send ((uint8_t *)"+++", 3);
		wait_ms(1200);
		wt11_send ((uint8_t *)"\n", 1);

		wt11_send ((uint8_t *)"at\n", 3);
		time_wait_ms (500);
	}

	/* flush input buffer */
	wt11_flush();

	/* send command */
	if (mode == WT11_MODE_NORMAL)
		wt11_send ((uint8_t *)"reset\n", 6);
	else if (mode == WT11_MODE_MUX)
		wt11_send_mux(WT11_MUX_CTRL_CMD, (uint8_t *)"reset", 5);

	/* parse answers */
	while (i<200) {
		
		if (mode == WT11_MODE_NORMAL)
			ret = wt11_rdline (data, 128);
		else if (mode == WT11_MODE_MUX){
			ret = wt11_recv_mux (NULL, data, WT11_MUX_LENGTH_MAX);
			data [ret] = '\0';
		}

		if (ret != -1)
			DEBUG (E_USER_WT11, "WT11: %s", data);

		time_wait_ms (10);
		i++;
	}
}

inline void wt11_reset (void) {
  __wt11_reset ((uint8_t)WT11_MODE_NORMAL);
}

inline void wt11_reset_mux (void) {
  __wt11_reset ((uint8_t)WT11_MODE_MUX);
}

/* open serial link in multiplexin mode */
void __wt11_open_link(uint8_t mode, uint8_t *addr, uint8_t *link_id)
{
	uint16_t __link_id;
	uint16_t size, i=0;
	uint16_t ret = 0;

	/* flush input buffer */
	wt11_flush();

	/* CALL {device address} {channel} {connect mode} */
	size = sprintf((char *)data, "CALL %.2X:%.2X:%.2X:%.2X:%.2X:%.2X 1 RFCOMM", 
		    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

	if (mode == WT11_MODE_NORMAL) {
	 	wt11_send (data, size);
	 	wt11_send ((uint8_t *)"\n", 1);
	}
	else if (mode == WT11_MODE_MUX)
	 	wt11_send_mux(WT11_MUX_CTRL_CMD, data, size);

	/* parse anwers */
	while (i<1000) {
		
		if (mode == WT11_MODE_NORMAL)
			ret = wt11_rdline (data, 128);
		else if (mode == WT11_MODE_MUX) {
			ret = wt11_recv_mux (NULL, data, WT11_MUX_LENGTH_MAX);
			data [ret] = '\0';
		}

		if (ret != -1) {
			DEBUG (E_USER_WT11, "WT11: %s", data);

			ret = sscanf((char *)data, "CALL %d", &__link_id);
			//if(ret == 1)
			//  NOTICE(E_USER_WT11, "WT11: link_id = %d", link_id);	
  			
			ret = sscanf((char *)data, "CONNECT %d RFCOMM 1", &__link_id);
 			if(ret == 1) {
				*link_id = __link_id;
			  	NOTICE(E_USER_WT11, "WT11: link_id = %d", *link_id);
				return;
			}	
		}
		time_wait_ms (10);
		i++;
	}

	ERROR(E_USER_WT11, "WT11: link KO");
}

inline void wt11_open_link(uint8_t *addr, uint8_t *link_id) {
  __wt11_open_link(WT11_MODE_NORMAL, addr, link_id);
}

inline void wt11_open_link_mux(uint8_t *addr, uint8_t *link_id) {
  __wt11_open_link(WT11_MODE_MUX, addr, link_id);
}

/* close serial link */
void __wt11_close_link(uint8_t mode, uint8_t link_id)
{
	uint16_t size, i=0;
	uint16_t ret = 0;
	int16_t error;

	if (mode == WT11_MODE_NORMAL)
	{
		/* change to cmd mode */
		wait_ms(1200);
		wt11_send ((uint8_t *)"+++", 3);
		wait_ms(1200);
		wt11_send ((uint8_t *)"\n", 3);

		wt11_send ((uint8_t *)"at\n", 3);
		time_wait_ms (500);
	}

	/* flush input buffer */
	wt11_flush();

	/* CLOSE {link_id} */
	size = sprintf((char *)data, "CLOSE %d", link_id);

	if (mode == WT11_MODE_NORMAL) {
	 wt11_send (data, size);
	 wt11_send ((uint8_t *)"\n", 1);
	}
	else if (mode == WT11_MODE_MUX)
	 wt11_send_mux(WT11_MUX_CTRL_CMD, data, size);

 
	/* parsing anwers */
	while (i<200) {
		
		if (mode == WT11_MODE_NORMAL)
			ret = wt11_rdline (data, 128);
		else if (mode == WT11_MODE_MUX) {
			ret = wt11_recv_mux (NULL, data, WT11_MUX_LENGTH_MAX);
			data [ret] = '\0';
		}

		if (ret != -1) {
			DEBUG (E_USER_WT11, "WT11: %s", data);

			ret = sscanf((char *)data, "NO CARRIER %d ERROR %d", (int *)&link_id, (int *)&error);
			if(ret == 2) {
			  	NOTICE(E_USER_WT11, "WT11: link_id = %d, error = %d", link_id, error);
				return;			
			}		
		}
		time_wait_ms (10);
		i++;
	}
}

inline void wt11_close_link(uint8_t link_id) {
  __wt11_close_link(WT11_MODE_NORMAL, link_id);
}

inline void wt11_close_link_mux(uint8_t link_id) {
  __wt11_close_link(WT11_MODE_MUX, link_id);
}

/* enable multiplexing mode */
void wt11_enable_mux_mode (void)
{
	uint16_t ret = 0, i=0;

	/* change to cmd mode */
	wait_ms(1200);
	wt11_send ((uint8_t *)"+++", 3);
	wait_ms(1200);
	wt11_send ((uint8_t *)"\n", 1);

	wt11_send ((uint8_t *)"at\n", 3);
	time_wait_ms (500);

	/* flush input buffer */
	wt11_flush();
 
  	/* SET CONTROL MUX 1 */
  	wt11_send ((uint8_t *)"SET CONTROL MUX 1\n", sizeof("SET CONTROL MUX 1\n"));

	/* parsing anwers */
	while (i<200) {
		ret = wt11_recv_mux (NULL, data, WT11_MUX_LENGTH_MAX);
		data [ret] = '\0';

		if (ret != -1)
			DEBUG (E_USER_WT11, "WT11: %s", data);

		time_wait_ms (10);
		i++;
	}

}

/* disable multiplexing mode */
void wt11_disable_mux_mode (void)
{
  uint16_t size, i=0;
  uint16_t ret = 0;

  /* SET CONTROL MUX 0 */
  size = sprintf((char *)data, "SET CONTROL MUX 0");

  wt11_send_mux(WT11_MUX_CTRL_CMD, data, size);

	/* parsing anwers */
	while (i<200) {
		ret = wt11_rdline (data, 128);

		if (ret != -1)
			DEBUG (E_USER_WT11, "WT11: %s", data);

		time_wait_ms (10);
		i++;
	}
}





