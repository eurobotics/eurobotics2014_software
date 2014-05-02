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


/****/
/* request status */
/****/
#if 0
#define BT_BEACON_STATUS_REQ	0x01
struct bt_beacon_status_req
{
	struct bt_cmd_hdr hdr;

  /* robot pos */
	int16_t x;
	int16_t y;
	int16_t a;

  	uint16_t checksum;

} __attribute__ ((aligned (2)));
#endif

#define BT_BEACON_STATUS_ANS	0x02
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



/****/
/* commands to boards (write data) */
/****/

#define I2C_CMD_GENERIC				0x00

#define I2C_CMD_LED_CONTROL		0x01
struct i2c_cmd_led_control{
	struct i2c_cmd_hdr hdr;
	uint8_t led_num:7;
	uint8_t state:1;	
};

#define I2C_CMD_SLAVEDSPIC_SET_MODE 0x02
struct i2c_cmd_slavedspic_set_mode {
	struct i2c_cmd_hdr hdr;
	
#define I2C_SLAVEDSPIC_MODE_INIT		            0x01
#define I2C_SLAVEDSPIC_MODE_POWER_OFF		        0x02

#define I2C_SLAVEDSPIC_SET_ARM      		        0x02
#define I2C_SLAVEDSPIC_SET_STICK    		        0x02
#define I2C_SLAVEDSPIC_SET_COMB      		        0x02
#define I2C_SLAVEDSPIC_SET_COMB_TRAY 		        0x02
#define I2C_SLAVEDSPIC_SET_BOOT_DOOR 		        0x02
#define I2C_SLAVEDSPIC_SET_VACUUM_MOTOR	        0x02
#define I2C_SLAVEDSPIC_SET_VACUUM_PRESURE      	0x02


#define I2C_SLAVEDSPIC_MODE_PREP_PICKUP_FIRE	  0x03
#define I2C_SLAVEDSPIC_MODE_PICKUP_FIRE         0x04
#define I2C_SLAVEDSPIC_MODE_STORE_FIRE          0x04
#define I2C_SLAVEDSPIC_MODE_LOAD_FIRE           0x04
#define I2C_SLAVEDSPIC_MODE_PREP_DRAG_FIRE      0x04
#define I2C_SLAVEDSPIC_MODE_DRAG_FIRE           0x04
#define I2C_SLAVEDSPIC_MODE_PREP_TURN_FIRE      0x04
#define I2C_SLAVEDSPIC_MODE_TURN_FIRE           0x04
#define I2C_SLAVEDSPIC_MODE_PREP_TURN_FIRE      0x04

#define I2C_SLAVEDSPIC_MODE_HIDE_ARMS           0x04

#define I2C_SLAVEDSPIC_MODE_PREP_HARVEST_TREE   0x04
#define I2C_SLAVEDSPIC_MODE_HARVEST_TREE        0x04


#define I2C_SLAVEDSPIC_MODE_PREP_TOXIC_FRUIT    0x04
#define I2C_SLAVEDSPIC_MODE_PICKUP_TOXIC_FRUIT  0x04
#define I2C_SLAVEDSPIC_MODE_DRAG_TOXIC_FRUIT    0x04

	uint8_t mode;
	union{

    /* TODO */

		struct {

		} fire;

		/* add more here */
	};
};

#endif

/* return the sum of length datum */
inline uint16_t checksum(uint8_t *data, uint16_t length) {
  uint16_t sum=0, i=0;

  for (i=0; i<length; i++)
    sum += data[i];

  return sum;
}

inline void bt_struct_write_int (int16_t x, uint8_t *y) {
  y[0] = x & 0x00FF; 
  y[1] = (x >> 8) & 0x00FF;
}

inline int16_t bt_struct_read_int (uint8_t *x) {
  return ( (((uint16_t)x[1] << 8) & 0xFF00) | ((uint16_t)x[0] & 0x00FF) );
}

#endif /* __BT_COMMANDS__ */

