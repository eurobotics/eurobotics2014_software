/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
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
 *  Javier Balias Santos <javier@arc-robots.org> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

/* compilation flavours */
//#define HOMOLOGATION
//#define DEMO_MODE

/* area */
#define AREA_X 3000
#define AREA_Y 2000

/* position of the tokens XXX */
#define TREE_1_X			0
#define TREE_1_Y			1300
#define TREE_2_X			700
#define TREE_2_Y			2000
#define TREE_3_X			2300
#define TREE_3_Y			2000
#define TREE_4_X			3000
#define TREE_4_Y			1300
#define HEART_FIRE_1_X		140
#define HEART_FIRE_1_Y		1860
#define HEART_FIRE_2_X		1500
#define HEART_FIRE_2_Y		1050
#define HEART_FIRE_3_X		2860
#define HEART_FIRE_3_Y		1860
#define FIRE_1_X			400
#define FIRE_1_Y			1100
#define FIRE_2_X			900
#define FIRE_2_Y			600
#define FIRE_3_X			900
#define FIRE_3_Y			1600
#define FIRE_4_X			2100
#define FIRE_4_Y			600
#define FIRE_5_X			2100
#define FIRE_5_Y			1600
#define FIRE_6_X			2600
#define FIRE_6_Y			1100
#define TORCH_1_X			0
#define TORCH_1_Y			800
#define TORCH_2_X			1300
#define TORCH_2_Y			2000
#define TORCH_3_X			1700
#define TORCH_3_Y			2000
#define TORCH_4_X			3000
#define TORCH_4_Y			800
#define MOBILE_TORCH_1_X		900
#define MOBILE_TORCH_1_Y		1100
#define MOBILE_TORCH_2_X		2100
#define MOBILE_TORCH_2_Y		1100
#define BASKET_1_X		700
#define BASKET_1_Y		150
#define BASKET_2_X		2300
#define BASKET_2_Y		150
#define MAMOOTH_1_X		700
#define MAMOOTH_1_Y		0
#define MAMOOTH_2_X		2300
#define MAMOOTH_2_Y		0
#define FRESCO_X			1500
#define FRESCO_Y			0
#define HOME_RED_X		2800
#define HOME_RED_Y		300
#define HOME_YELLOW_X		200
#define HOME_YELLOW_Y		300

/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (!x))

#define COLOR_I(x)	  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) :  ((NB_SLOT_X-1)-x))

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1000

#define SIDE_REAR		I2C_SIDE_REAR
#define SIDE_FRONT 	I2C_SIDE_FRONT 
#define SIDE_MAX		I2C_SIDE_MAX

#define OPPOSITE_SIDE(side) ((side==I2C_SIDE_FRONT)? (I2C_SIDE_REAR) : (I2C_SIDE_FRONT))	

#define GO_FORWARD	0
#define GO_BACKWARD	1

/* useful traj flags */
#define TRAJ_SUCCESS(f) 				(f & (END_TRAJ|END_NEAR))
#define TRAJ_BLOCKING(f) 				(f & (END_BLOCKING))

#define TRAJ_FLAGS_STD 					(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 			(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

#define LAST_SECONDS_TIME	80


//#define CALIBRATION
#ifdef CALIBRATION

/* default acc */
#define ACC_DIST  5.
#define ACC_ANGLE 5.

/* default speeds */
#define SPEED_DIST_FAST 			1000.
#define SPEED_ANGLE_FAST 			1000.
#define SPEED_DIST_SLOW 		  	1000.
#define SPEED_ANGLE_SLOW 		  	1000.
#define SPEED_DIST_VERY_SLOW 		1000.
#define SPEED_ANGLE_VERY_SLOW 	1000.

#else

/* default acc */
#define ACC_DIST  30 //40.
#define ACC_ANGLE 30 //90.

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 			2000.
#define SPEED_ANGLE_FAST 			2000.
#else

#define SPEED_DIST_FAST 			5000.
#define SPEED_ANGLE_FAST 			5000.
#endif

//Do not change
#define SPEED_DIST_SLOW 		  	2500.
#define SPEED_ANGLE_SLOW 		  	2500.
#define SPEED_DIST_VERY_SLOW 		1000.
#define SPEED_ANGLE_VERY_SLOW 	1000.

#endif

/* zones */
#define ZONE_TREE_1			    0
#define ZONE_TREE_2       	1
#define ZONE_TREE_3			    2
#define ZONE_TREE_4			    3
#define ZONE_HEART_FIRE_1		4
#define ZONE_HEART_FIRE_2  	5
#define ZONE_HEART_FIRE_3 	6
#define ZONE_FIRE_1   	    7
#define ZONE_FIRE_2	        8
#define ZONE_FIRE_3	        9
#define ZONE_FIRE_4	        10
#define ZONE_FIRE_5			    11
#define ZONE_FIRE_6			    12
#define ZONE_TORCH_1   			13
#define ZONE_TORCH_2			  14
#define ZONE_TORCH_3			  15
#define ZONE_TORCH_4        16
#define ZONE_MOBILE_TORCH_1 17
#define ZONE_MOBILE_TORCH_2 18
#define ZONE_MOBILE_TORCH_3 19
#define ZONE_BASKET_1       20
#define ZONE_BASKET_2       21
#define ZONE_MAMOOTH_1      22
#define ZONE_MAMOOTH_2      23
#define ZONE_FRESCO      	  24
#define ZONE_HOME_RED       25
#define ZONE_HOME_YELLOW    26
#define ZONES_MAX		        27


/************************************************************* 
 * Strat data structures 
 ************************************************************/

/* boulding box */
struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

/* configuration */
struct conf {

/* depends on flags the robot
 * do one things or anothers */
	uint8_t flags;
  #define ENABLE_R2ND_POS				  1 /* TODO: set by command */
	#define ENABLE_DOWN_SIDE_ZONES	2
};



typedef struct {
	/* type */
	uint16_t type;
	#define ZONE_TYPE_TREE					0
	#define ZONE_TYPE_FIRE					1
	#define ZONE_TYPE_HEART_FIRE				2
	#define ZONE_TYPE_TORCH					3
	#define ZONE_TYPE_MOBILE_TORCH				4
	#define ZONE_TYPE_FRESCO					5
	#define ZONE_TYPE_MAMOOTH					6
	#define ZONE_TYPE_BASKET					7
	#define ZONE_TYPE_HOME					8
	#define ZONE_TYPE_MAX					9

	/* target point */
	int16_t x;
	int16_t y;	

	/* boundinbox */
	int16_t x_down;
	int16_t x_up;
	int16_t y_down;
	int16_t y_up;

   	/* init point */
	int16_t init_x;
	int16_t init_y;

	/* priority */
	uint8_t prio;
   #define ZONE_PRIO_0		0
   #define ZONE_PRIO_10		10
   #define ZONE_PRIO_20		20
   #define ZONE_PRIO_30		30
   #define ZONE_PRIO_40		40
   #define ZONE_PRIO_50		50
   #define ZONE_PRIO_60		60
   #define ZONE_PRIO_70		70
   #define ZONE_PRIO_80		80
   #define ZONE_PRIO_90		90
   #define ZONE_PRIO_100	100
   #define ZONE_PRIO_MAX	100

	uint16_t flags;
   #define ZONE_CHECKED		    	1
   #define ZONE_CHECKED_OPP		2
   #define ZONE_SEC_ROBOT	   	4
   #define ZONE_AVOID		    	8
} strat_zones;


struct strat_infos {
	uint8_t dump_enabled;
	uint8_t debug_step;
	struct conf conf;
	struct bbox area_bbox;

	/* points areas */
	strat_zones zones[ZONES_MAX];

	/* our zone position */
	uint8_t current_zone;
	uint8_t goto_zone;
	uint8_t last_zone;

	/* opponent zone position */
	uint8_t opp_current_zone;
	uint8_t opp2_current_zone;

	/* opponent statistics */
	uint32_t opp_time_zone_ms;

	/* state of the robot */
	uint8_t fruits_inside;
	uint8_t spears_inside;
	uint8_t net_inside;
	uint8_t fires_inside;
};

extern struct strat_infos strat_infos;

extern char numzone2name[ZONES_MAX + 1][30];

#ifndef HOST_VERSION_OA_TEST

/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c 
 *******************************************/
#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_beginning(void);
void strat_event(void *dummy);
void strat_event_enable(void);
void strat_event_disable(void);
void strat_start_match(uint8_t debug);


/********************************************
 * in strat_main.c 
 *******************************************/

/* auto possition depending on color */
void strat_auto_position (void);

/* match tasks */
uint8_t strat_patrol_fresco_mamooth(uint8_t balls_mamooth_1, uint8_t balls_mamooth_2);
uint8_t strat_paint_fresco(void);
uint8_t strat_patrol_between(int16_t x1, int16_t y1,int16_t x2, int16_t y2);
uint8_t strat_shoot_mamooth(uint8_t balls_mamooth_1, uint8_t balls_mamooth_2);
void strat_initial_move(void);

/********************************************
 * in strat_event.c 
 *******************************************/

/*************** STRAT EVENT SCHEDULE FUNCTIONS *******************************/

/* schedule a single strat tevent */
void strat_event_schedule_single (void (*f)(void *), void * data);

/* schedule a periodical strat tevent */
void strat_event_schedule_periodical(void (*f)(void *), void * data);


/*************** STRAT EVENT WRAPPER FUNCTIONS  *******************************/

/* auto position event */
void strat_auto_position_event (void *data);

/* trajectory functions */
void strat_goto_xy_abs_event (void *data);
void strat_goto_forward_xy_abs_event (void *data);
void strat_goto_backward_xy_abs_event (void *data);

void strat_goto_avoid_event (void *data);
void strat_goto_avoid_forward_event (void *data);
void strat_goto_avoid_backward_event (void *data);
/* match tasks */
void strat_patrol_and_paint_fresco_event (void *data);
void strat_fresco_event (void *data);
void strat_patrol_event (void *data);
void strat_mamooth_event (void *data);
void strat_protect_h_event(void *data);
void strat_patrol_fresco_mamooth_event(void *data);
void strat_net_event (void *data);

#else /* HOST_VERSION_OA_TEST */

#define AREA_BBOX_6X5	0
#define AREA_BBOX_4X4	1
void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION_OA_TEST */


#endif
