/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  $Id$
 */

#ifndef __BT_COMMANDS_H__
#define __BT_COMMANDS_H__


struct bt_cmd_hdr {
	uint16_t cmd;
} __attribute__ ((aligned (2)));



#define BT_BEACON_SYNC_HEADER	"beacon_header"
#define BT_BEACON_STATUS_ANS	0x01
struct bt_beacon_status_ans 
{
	struct bt_cmd_hdr hdr;

  /* opp pos */
	int16_t opponent_x;
	int16_t opponent_y;
	int16_t opponent_a;
	int16_t opponent_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;
	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif

  	uint16_t checksum;
} __attribute__ ((aligned (2)));

#if 0
#define BT_ROBOT_2ND_STATUS_ANS 0x02
struct bt_robot_2nd_status_ans{
	struct bt_cmd_hdr hdr;

  /* status */
  uint8_t status;

  /* END_TRAJ flags, see strat_base.h */
	uint8_t end_traj;

  /* robot position */
	int8_t x[2];
	int8_t y[2];
  int8_t a_abs[2];

  /* opponent pos */
	uint8_t opponent_x[2];
	uint8_t opponent_y[2];

#ifdef TWO_OPPONENTS
	uint8_t opponent2_x[2];
	uint8_t opponent2_y[2];
#endif

  uint8_t checksum;

};
#endif



/* return the sum of length datum */
inline uint16_t checksum(uint8_t *data, uint16_t length) {
  uint16_t sum=0, i=0;

  for (i=0; i<length; i++)
    sum += data[i];

  return sum;
}


#endif /* __BT_COMMANDS__ */

