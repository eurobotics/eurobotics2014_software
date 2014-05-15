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
#include <stdarg.h>
#include <ctype.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <clock_time.h>

#include "main.h"
#include "strat_base.h"
#include "strat_utils.h"
#include "wt11.h"
#include "bt_protocol.h"
#include "../common/bt_commands.h"

#ifdef HOST_VERSION
#include "robotsim.h"
#endif

#define M_2PI (2*M_PI)

#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * (M_PI / 180.0))


static uint8_t cmd_data[BT_PROTO_NUM_DEVICES][WT11_MUX_LENGTH_MAX];
static uint8_t cmd_size[BT_PROTO_NUM_DEVICES] = {0, 0};
static uint8_t bt_errors_checksum = 0;


/* fill a link id buffer which has to be send */
uint8_t bt_send_cmd (uint8_t link_id, uint8_t *data, uint16_t size) 
{
	int16_t i;
  	uint8_t flags;

	/* check length */
	if(size > WT11_MUX_LENGTH_MAX){
		ERROR(E_USER_BT_PROTO, "Command size is too large");	
		return 1;
	}

  	/* XXX check if there is any pendant cmd */
  	//if (cmd_size[link_id])
	//	NOTICE (E_USER_BT_PROTO, "waiting free buffer... ");
	//while (cmd_size[link_id]);
		
	/* fill buffer */
	for(i=0; i<size; i++){
		cmd_data[link_id][i] = data[i];	
	}

	/* command size != 0 indicate 
   	 * that there is a command to send */	
	IRQ_LOCK(flags);
	cmd_size[link_id] = size;
  	IRQ_UNLOCK(flags);

  	return 0;
}

/* fill a link id buffer with formated data to be send */
uint8_t bt_send_ascii_cmd (uint8_t link_id, const char * format, ...)
{
	char buffer[WT11_MUX_LENGTH_MAX];
	va_list args;
	uint16_t n;
	uint8_t ret;

	va_start (args, format);
	n = vsprintf (&buffer[1],format, args);
	
	buffer[0]='\n';
	buffer[n+1]='\n';

	n += 2;

  	/* check length */
	if(n > WT11_MUX_LENGTH_MAX){
		ERROR(E_USER_BT_PROTO, "Command size is too large");
        va_end (args);
		return 1;
	}

	ret = bt_send_cmd (link_id, (uint8_t *)buffer, n);

	va_end (args);
	return ret;
}


/************************************************************
 * BEACON COMMANDS 
 ***********************************************************/

/* set color */
void bt_beacon_set_color (void)
{
	if(mainboard.our_color == I2C_COLOR_YELLOW)
   	bt_send_ascii_cmd (beaconboard.link_id, "color yellow");
  	else
		bt_send_ascii_cmd (beaconboard.link_id, "color red");
}


/* beacon on */
void bt_beacon_set_on (void) {
	bt_send_ascii_cmd (beaconboard.link_id, "beacon on");
}

/* beacon on with watchdog */
void bt_beacon_set_on_watchdog (void) {
	bt_send_ascii_cmd (beaconboard.link_id, "beacon watchdog_on");
}

/* beacon off*/
void bt_beacon_set_off (void) {
  	bt_send_ascii_cmd (beaconboard.link_id, "beacon off");
}


/* request opponent position */
void bt_beacon_req_status(void)
{
	int16_t robot_a;
	double robot_x, robot_y;
	uint8_t flags;
	uint8_t buff[32];
	uint8_t size;
   uint16_t checksum;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a = position_get_a_deg_s16(&mainboard.pos);
	IRQ_UNLOCK(flags);

  	rel_da_to_abs_xy(BEACON_OFFSET_D, RAD(BEACON_OFFSET_A), 
                  &robot_x, &robot_y);    

	/* checksum */
	checksum = (uint16_t)((int16_t)robot_x + (int16_t)robot_y + robot_a);

#if 0
  	bt_send_ascii_cmd (beaconboard.link_id, "opponent %d %d %d %d",
						    (int16_t)robot_x, (int16_t)robot_y, robot_a, checksum);

#else
	size = sprintf((char*)buff, "\nstatus %d %d %d %d\n",
						(int16_t)robot_x, (int16_t)robot_y, robot_a,
						checksum);
#endif

	wt11_send_mux (beaconboard.link_id, buff, size);
}

/* parse opponent position */
void bt_beacon_status_parser (int16_t c)
{
//#define debug_beacon_parser

	static uint8_t state = 0;
   static uint16_t i = 0;
	static struct bt_beacon_status_ans ans;
	static uint8_t *data = (uint8_t *) (&ans);

	uint8_t sync_header[] = BT_BEACON_SYNC_HEADER;
   uint8_t flags = 0;
    
	c &= 0x00FF;

	switch(state) 
	{
		case 0:
			/* sync header */
			if ((uint8_t)c == sync_header[i]) { i++; }
			else { i = 0; }
			
			if(i == sizeof(sync_header)) {
				i = 0;
				state ++;
				//DEBUG(E_USER_BT_PROTO,"beacon sync header detected");
			}
			break;

		case 1:
			data[i++] = (uint8_t)c;
			if (i < sizeof(ans))
				break;

			//DEBUG(E_USER_BT_PROTO,"beacon size %d", i);
			state = 0;

		case 2:
		{
		   double x, y, a, d;

			if (ans.checksum != bt_checksum(data, sizeof(ans)-sizeof(ans.checksum)))
				goto error_checksum;

			/* beacon correction */
			x = ans.opponent1_x;
			y = ans.opponent1_y;
#ifdef debug_beacon_parser
			a = ans.opponent1_a;
			d = ans.opponent1_d;
#else
			abs_xy_to_rel_da(x, y, &d, &a);
#endif
			IRQ_LOCK(flags);
			beaconboard.opponent1_x = (int16_t)x;
			beaconboard.opponent1_y = (int16_t)y;
			beaconboard.opponent1_a = (int16_t)DEG(a);
			beaconboard.opponent1_d = (int16_t)d;       
			IRQ_UNLOCK(flags);


			#ifdef TWO_OPPONENTS
			x = ans.opponent2_x;
			y = ans.opponent2_y;
#ifdef debug_beacon_parser
			a = ans.opponent2_a;
			d = ans.opponent2_d;
#else
			abs_xy_to_rel_da(x, y, &d, &a);
#endif
			IRQ_LOCK(flags);
			beaconboard.opponent2_x = (int16_t)x;
			beaconboard.opponent2_y = (int16_t)y;
			beaconboard.opponent2_a = (int16_t)DEG(a);
			beaconboard.opponent2_d = (int16_t)d;       
			IRQ_UNLOCK(flags);
			#endif

			//DEBUG (E_USER_BT_PROTO, "BT_PROTO: opp1 %d %d %d %d", 
			//		beaconboard.opponent_x, beaconboard.opponent_y,
			//		beaconboard.opponent_d, beaconboard.opponent_a);

			//DEBUG (E_USER_BT_PROTO, "BT_PROTO: opp2 %d %d %d %d", 
			//		beaconboard.opponent2_x, beaconboard.opponent2_y,
			//		beaconboard.opponent2_d, beaconboard.opponent2_a);

		 	break;
		}

		default:
			state = 0;
			break;
	}

	return;

 /* received errors */	
error_checksum:
	state = 0;
	bt_errors_checksum ++;
	NOTICE(E_USER_BT_PROTO, "beacon CHECKSUM error (%d)", bt_errors_checksum);
  	return;
}

/************************************************************
 * ROBOT_2ND COMMANDS 
 ***********************************************************/

/* return 1 if cmd arguments checksum matches */
uint8_t bt_robot_2nd_test_checksum (void) {
	return (robot_2nd.cmd_args_checksum_send == robot_2nd.cmd_args_checksum_recv);
}

/* send command, and return after received ack */
void bt_robot_2nd_cmd_no_wait_ack (uint8_t cmd_id, int16_t arg0, int16_t arg1)
{
    uint8_t flags;
	//DEBUG (E_USER_BT_PROTO, "TX cmd: id %d arg0 %d arg1 %d", cmd_id, arg0, arg1);

	/* command */
	if (cmd_id == BT_SET_COLOR) {
		arg0 = mainboard.our_color;

		if(arg0 == I2C_COLOR_YELLOW)
			bt_send_ascii_cmd (robot_2nd.link_id, "color yellow");
	  	else
			bt_send_ascii_cmd (robot_2nd.link_id, "color red");
	}

	else if (cmd_id == BT_GOTO_XY_ABS)
		bt_send_ascii_cmd (robot_2nd.link_id, "goto xy_abs %d %d", arg0, arg1);
	else if (cmd_id == BT_GOTO_XY_REL)
		bt_send_ascii_cmd (robot_2nd.link_id, "goto xy_rel %d %d", arg0, arg1);

	/* set feedback info */
	IRQ_LOCK (flags);
	robot_2nd.cmd_id = cmd_id;
	robot_2nd.cmd_args_checksum_send = (uint8_t) (cmd_id + arg0 + arg1);
	robot_2nd.cmd_args_checksum_recv = 0;
	robot_2nd.cmd_ret = 0;
	IRQ_UNLOCK (flags);

}

/* send command, and return after received ack */
void bt_robot_2nd_cmd (uint8_t cmd_id, int16_t arg0, int16_t arg1)
{
	uint8_t nb_tries = 3;
	int8_t ret;

retry:

	bt_robot_2nd_cmd_no_wait_ack (cmd_id, arg0, arg1);

	/* wait ack */
	ret = BT_WAIT_COND_OR_TIMEOUT( bt_robot_2nd_test_checksum (), 150);
	if (!ret && nb_tries--) {
		ERROR (E_USER_BT_PROTO, "ERROR sendind robot 2nd command (%d)", nb_tries);
		goto retry;
	}
}

/* set color */
inline void bt_robot_2nd_set_color (void) {
	//bt_robot_2nd_cmd_no_wait_ack (BT_SET_COLOR, 0, 0);
	bt_robot_2nd_cmd (BT_SET_COLOR, 0, 0);
}

/* goto xy_abs */
inline void bt_robot_2nd_goto_xy_abs (int16_t x, int16_t y) {
	bt_robot_2nd_cmd (BT_GOTO_XY_ABS, x, y);
}

/* goto xy_rel */
inline void bt_robot_2nd_goto_xy_rel (int16_t x, int16_t y) {
	bt_robot_2nd_cmd (BT_GOTO_XY_REL, x, y);
}


/* request opponent position */
void bt_robot_2nd_req_status(void)
{
	int16_t robot_a_abs, robot_x, robot_y;
	int16_t opp1_x, opp1_y, opp2_x, opp2_y;
	uint8_t flags;
	uint8_t buff[64];
	uint8_t size;
	int16_t checksum = 0;
	
	IRQ_LOCK(flags);
	robot_x = position_get_x_s16(&mainboard.pos);
	robot_y = position_get_y_s16(&mainboard.pos);
	robot_a_abs = position_get_a_deg_s16(&mainboard.pos);
	opp1_x = beaconboard.opponent1_x;
	opp1_y = beaconboard.opponent1_y;
	opp2_x = beaconboard.opponent2_x;
	opp2_y = beaconboard.opponent2_y;
	IRQ_UNLOCK(flags);

	/* checksum */
	checksum = robot_x + robot_y + robot_a_abs;
	checksum += opp1_x + opp1_y;
	checksum += opp2_x + opp2_y;

#if 0
  	bt_send_ascii_cmd (beaconboard.link_id, "status %d %d %d %d %d %d %d %d",
						robot_x, robot_y, robot_a_abs, 
						opp1_x, opp1_y, 
						opp2_x, opp2_y,
						checksum);
#else
	size = sprintf((char*)buff, "\nstatus %d %d %d %d %d %d %d %d\n",
						robot_x, robot_y, robot_a_abs, 
						opp1_x, opp1_y, 
						opp2_x, opp2_y,
						checksum);
	wt11_send_mux (robot_2nd.link_id, buff, size);
#endif

}

/* parse robot 2nd status */
void bt_robot_2nd_status_parser (int16_t c)
{
//#define debug_robot_2nd_parser

	static uint8_t state = 0;
   static uint16_t i = 0;
	static struct bt_robot_2nd_status_ans ans;
	static uint8_t *data = (uint8_t *) (&ans);

	uint8_t sync_header[] = BT_ROBOT_2ND_SYNC_HEADER;
   uint8_t flags = 0;
    
	c &= 0x00FF;

	switch(state) 
	{
		case 0:
			/* sync header */
			if ((uint8_t)c == sync_header[i]) { i++; }
			else { i = 0; }
			
			if(i == sizeof(sync_header)) {
				i = 0;
				state ++;
				//DEBUG(E_USER_BT_PROTO,"robot_2nd sync header detected");
			}
			break;

		case 1:
			data[i++] = (uint8_t)c;
			if (i < sizeof(ans))
				break;

			//DEBUG(E_USER_BT_PROTO,"robot_2nd size %d", i);
			state = 0;

		case 2:
		{
		 	double x, y, a_abs, a, d;

			if (ans.checksum != bt_checksum(data, sizeof(ans)-sizeof(ans.checksum)))
				goto error_checksum;

			//DEBUG (E_USER_BEACON, "RX cmd id %d, args %d, ret %d", 
			//		ans.cmd_id, ans.cmd_args_checksum, get_err(ans.cmd_ret));

			/* running command info */
			if (ans.cmd_id == robot_2nd.cmd_id) {
				robot_2nd.cmd_ret = ans.cmd_ret;
				robot_2nd.cmd_args_checksum_recv = ans.cmd_args_checksum;
			}

			/* strat infos */
			robot_2nd.color = ans.color;
			robot_2nd.done_flags = ans.done_flags;

			/* robot pos */
			x = ans.x;
			y = ans.y;
			a_abs = ans.a_abs;
#ifdef debug_robot_2nd_parser
			a = ans.a_abs;
			d = ans.a_abs;
#else
			abs_xy_to_rel_da(x, y, &d, &a);
#endif
			IRQ_LOCK(flags);
			robot_2nd.x = (int16_t)x;
			robot_2nd.y = (int16_t)y;
			robot_2nd.a = (int16_t)DEG(a);
			robot_2nd.a_abs = (int16_t)a_abs;
			robot_2nd.d = (int16_t)d;       
			IRQ_UNLOCK(flags);

			/* opponents pos with beacon correction */
			x = ans.opponent1_x;
			y = ans.opponent1_y;
#ifdef debug_robot_2nd_parser
			a = ans.opponent1_x;
			d = ans.opponent1_y;
#else
			abs_xy_to_rel_da(x, y, &d, &a);
#endif
			IRQ_LOCK(flags);
			robot_2nd.opponent1_x = (int16_t)x;
			robot_2nd.opponent1_y = (int16_t)y;
			robot_2nd.opponent1_a = (int16_t)DEG(a);
			robot_2nd.opponent1_d = (int16_t)d;       
			IRQ_UNLOCK(flags);


#ifdef TWO_OPPONENTS
			x = ans.opponent2_x;
			y = ans.opponent2_y;
#ifdef debug_robot_2nd_parser
			a = ans.opponent2_x;
			d = ans.opponent2_y;
#else
			abs_xy_to_rel_da(x, y, &d, &a);
#endif
			IRQ_LOCK(flags);
			robot_2nd.opponent2_x = (int16_t)x;
			robot_2nd.opponent2_y = (int16_t)y;
			robot_2nd.opponent2_a = (int16_t)DEG(a);
			robot_2nd.opponent2_d = (int16_t)d;       
			IRQ_UNLOCK(flags);
#endif
		}

		default:
			state = 0;
			break;
	}

	return;

 /* received errors */	
error_checksum:
	state = 0;
	bt_errors_checksum ++;
	NOTICE(E_USER_BT_PROTO, "robot 2nd CHECKSUM error (%d)", bt_errors_checksum);
  	return;
}



/* send and receive commands to/from bt devices, periodic dev status pulling */
void bt_protocol (void * dummy)
{
   int16_t c;
	uint8_t link_id;
	static microseconds pull_time_us = 0;
   uint8_t flags;
   uint8_t i;
	uint8_t cmd_sent = 0;

	static uint8_t toggle = 1;

	if ((mainboard.flags & DO_BT_PROTO)==0)
		return;

	/**
	 * NOTE: in HOST mode, there is only comunication with robot 2nd 
	 * 	   beacon status (opponent possition) is captured from 
    *       python simulator.
    */

  	/* receive commands */
#ifndef HOST_VERSION
  	c = wt11_recv_mux_char (&link_id);

  	while (c != -1) {

		//uart_send (CMDLINE_UART, c);

		//if ( (link_id == beaconboard.link_id) && (mainboard.flags & DO_BEACON) )
			bt_beacon_status_parser (c);
		//else if ( (link_id == robot_2nd.link_id) && (mainboard.flags & DO_ROBOT_2ND))
			bt_robot_2nd_status_parser (c);
	  	//else if ( (link_id != beaconboard.link_id) 
		//			&& (link_id != robot_2nd.link_id) ) {
		//	NOTICE (E_USER_BT_PROTO, "Unexpected link id (%d)", link_id);
			//uart_send (CMDLINE_UART, c);
		//}

		c = wt11_recv_mux_char (&link_id);	
	}
#else
  	c = robotsim_uart_recv_BT ();
  	while (c != -1) {
		if ((mainboard.flags & DO_ROBOT_2ND))
			bt_robot_2nd_status_parser (c);

	  	c = robotsim_uart_recv_BT ();
	}

#endif

  	/* send commands */
	for (i=0; i<BT_PROTO_NUM_DEVICES; i++)
	{
	  	if (cmd_size[i]) {
#ifndef HOST_VERSION
			wt11_send_mux (i, cmd_data[i], cmd_size[i]);
#else
			wt11_send (cmd_data[i], cmd_size[i]);
#endif
			IRQ_LOCK(flags);
			cmd_size[i] = 0;
			IRQ_UNLOCK(flags);

			cmd_sent = 1;
		}
	}

	/* avoid send a command and pulling in the same cycle) */
	if (cmd_sent) {
		toggle = 1; /* force robot 2nd pulling next cycle */
		pull_time_us = time_get_us2();
		return;
	}

  	/* pulling of status */
	if((time_get_us2() - pull_time_us > 60000UL)) {

		toggle ^= 1;

#ifndef HOST_VERSION
		if ((mainboard.flags & DO_BEACON) && toggle)
			bt_beacon_req_status ();	
#endif
		if ((mainboard.flags & DO_ROBOT_2ND) && !toggle)
			bt_robot_2nd_req_status ();

		pull_time_us = time_get_us2();
	}
}





