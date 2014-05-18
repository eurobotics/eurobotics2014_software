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

#ifndef __WT11_H__
#define __WT11_H__

/* UART used for send and receive data */
#define BT_UART   MUX_UART

/* number of bt devices, maximun 4 */
#define BT_PROTO_NUM_DEVICES 2

/* send and received modes */
#define WT11_MODE_NORMAL  0
#define WT11_MODE_MUX     1

#define WT11_MUX_CTRL_CMD   0xFF

/* maximun size of message */
#define WT11_MUX_LENGTH_MAX 100

/* send data in norma mode, any protocol is used */
void wt11_send (uint8_t *buff, uint16_t length);

/* send data using multiplexing mode protocol */
void wt11_send_mux (uint8_t link_id, uint8_t *buff, uint16_t length);

/* flush recevied buffer */
void wt11_flush (void);

/* receive data using multiplexing mode protocol, 
   returns data length, -1 if no data received */
int16_t wt11_recv_mux (uint8_t *link_id, uint8_t *buff, uint16_t buff_size);

/* receive data using multiplexing mode protocol and redirect to STDO,
   returns data length, -1 if no data received */
int16_t wt11_bypass_to_stdo (uint8_t link_id);

/* return received char and link_id, -1 if not valid char */
int16_t wt11_recv_mux_char (uint8_t *link_id);

/* receive data in data mode, 
   returns data length, -1 if no data received */
int16_t wt11_rdline (uint8_t *buff, uint16_t buff_size);

/* open serial link in multiplexin mode */
void __wt11_open_link(uint8_t mode, uint8_t *addr, uint8_t *link_id);
void wt11_open_link(uint8_t *addr, uint8_t *link_id);
void wt11_open_link_mux(uint8_t *addr, uint8_t *link_id);

/* close serial link */
void __wt11_close_link(uint8_t mode, uint8_t link_id);
void wt11_close_link(uint8_t link_id);
void wt11_close_link_mux(uint8_t link_id);

/* enable multiplexing mode */
void wt11_enable_mux_mode (void);

/* disable multiplexing mode */
void wt11_disable_mux_mode (void);

/* reset wt11 */
void __wt11_reset (uint8_t mode);
void wt11_reset (void);
void wt11_reset_mux (void);


#endif /* __WT11_H__ */

