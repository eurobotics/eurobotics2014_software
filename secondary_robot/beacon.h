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

#ifndef __BEACON_H__
#define __BEACON_H__

/* IR sensor management */
#define IR_SENSOR_180_DEG	0
#define IR_SENSOR_0_DEG		1
#define IR_SENSOR_MAX		2

struct beacon {
	int32_t beacon_speed;
	uint8_t angle_offset;

	int32_t robot_x;
	int32_t robot_y;
	int32_t robot_a;
	
	int32_t opponent1_angle;
	int32_t opponent1_dist;
	int32_t opponent1_x;
	int32_t opponent1_y;

#ifdef TWO_OPPONENTS
	int32_t opponent2_angle;
	int32_t opponent2_dist;
	int32_t opponent2_x;
	int32_t opponent2_y;

	uint8_t tracking_opp1_counts;
	uint8_t tracking_opp2_counts;
	int32_t tracking_opp1_x;
	int32_t tracking_opp1_y;
	int32_t tracking_opp2_x;
	int32_t tracking_opp2_y;
#endif
#ifdef ROBOT_2ND
	int32_t robot_2nd_angle;
	int32_t robot_2nd_dist;
	int32_t robot_2nd_x;
	int32_t robot_2nd_y;
#endif
};

extern struct beacon beacon;

void beacon_init(void);
void beacon_start(void);
void beacon_stop(void);
void beacon_calc(void *dummy);

void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y);

void beacon_encoder_manage(void);
int32_t beacon_encoder_get_value(void);
void beacon_encoder_set_value(int32_t val);

#endif /* __BEACON_H__ */

