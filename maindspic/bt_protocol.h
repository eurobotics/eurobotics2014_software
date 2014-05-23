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

#ifndef __BT_PROTOCOL_H__
#define __BT_PROTOCOL_H__

/* number of bt devices, maximun 4 */
#define BT_PROTO_NUM_DEVICES 2

/* send and receive commands to/from bt devices, periodic dev status pulling */
void bt_protocol (void * dummy);


/************************************************************
 * BEACON COMMANDS 
 ***********************************************************/

/* set color */
void bt_beacon_set_color (void);

/* beacon on */
void bt_beacon_set_on (void);

/* beacon on with watchdog */
void bt_beacon_set_on_watchdog (void);

/* beacon off*/
void bt_beacon_set_off (void);

/* request opponent position */
void bt_beacon_req_status(void);

/************************************************************
 * ROBOT 2ND COMMANDS 
 ***********************************************************/

/* send command, and return after received ack */
void bt_robot_2nd_cmd_no_wait_ack (uint8_t cmd_id, int16_t arg0, int16_t arg1);

/* send command, and return after received ack */
uint8_t bt_robot_2nd_cmd (uint8_t cmd_id, int16_t arg0, int16_t arg1);

/* auto set possition */
uint8_t bt_robot_2nd_autopos (void);

/* set color */
uint8_t bt_robot_2nd_set_color (void);

/* goto xy_abs */
uint8_t bt_robot_2nd_goto_xy_abs (int16_t x, int16_t y);

/* goto xy_rel */
uint8_t bt_robot_2nd_goto_xy_rel (int16_t x, int16_t y);

/* wait for robot 2nd ends */
uint8_t bt_robot_2nd_wait_end (void);

/* request opponent position */
void bt_robot_2nd_req_status(void);


#endif
