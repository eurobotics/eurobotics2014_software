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
 *  Javier Baliñas Santos <javier@arc-robots.org> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

#include <clock_time.h>
/* compilation flavours */
//#define HOMOLOGATION

/* area */
#define AREA_X 3000
#define AREA_Y 2000

/* INITIAL PRIORITIES */
#define PRIO_BASKET_AFTER_ONE_TREE 40
#define PRIO_HEART_AFTER_PUZZLE 90
#define PRIO_TREE_1 							ZONE_PRIO_50
#define PRIO_TREE_2 							ZONE_PRIO_50
#define PRIO_TREE_3 							ZONE_PRIO_50
#define PRIO_TREE_4 							ZONE_PRIO_50
#define PRIO_FIRE_1 							ZONE_PRIO_30
#define PRIO_FIRE_2 							ZONE_PRIO_10
#define PRIO_FIRE_3 							ZONE_PRIO_30
#define PRIO_FIRE_4 							ZONE_PRIO_20
#define PRIO_FIRE_5							ZONE_PRIO_20
#define PRIO_FIRE_6							ZONE_PRIO_20
#define PRIO_TORCH_1 							ZONE_PRIO_30
#define PRIO_TORCH_2 							ZONE_PRIO_30
#define PRIO_TORCH_3						ZONE_PRIO_30
#define PRIO_TORCH_4 							ZONE_PRIO_20
#define PRIO_M_TORCH_1 							ZONE_PRIO_60
#define PRIO_M_TORCH_2 							ZONE_PRIO_20
#define PRIO_BASKET_1 							ZONE_PRIO_0
#define PRIO_BASKET_2						ZONE_PRIO_0
#define PRIO_MAMOOTH_1 							ZONE_PRIO_80
#define PRIO_MAMOOTH_2						ZONE_PRIO_80
#define PRIO_FRESCO						ZONE_PRIO_80
#define PRIO_HEART_1						ZONE_PRIO_40
#define PRIO_HEART_2_UP						ZONE_PRIO_0
#define PRIO_HEART_2_LEFT						ZONE_PRIO_0
#define PRIO_HEART_2_DOWN						ZONE_PRIO_0
#define PRIO_HEART_2_RIGHT						ZONE_PRIO_0
#define PRIO_HEART_3						ZONE_PRIO_0

/* position of the elements */
#define TREE_1_X			0
#define TREE_1_Y			1300
#define TREE_2_X			700
#define TREE_2_Y			2000
#define TREE_3_X			2300
#define TREE_3_Y			2000
#define TREE_4_X			3000
#define TREE_4_Y			1300

#define HEART_1_X		    140
#define HEART_1_Y		    1860
#define HEART_2_X		    1500
#define HEART_2_Y   		1050
#define HEART_2_UP_X		    1500
#define HEART_2_UP_Y		    1350
#define HEART_2_LEFT_X		    1200
#define HEART_2_LEFT_Y		    1050
#define HEART_2_DOWN_X		    1500
#define HEART_2_DOWN_Y		    750
#define HEART_2_RIGHT_X		    1800
#define HEART_2_RIGHT_Y		    1050
#define HEART_3_X		    2860
#define HEART_3_Y		    1860

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

#define M_TORCH_1_X	        900
#define M_TORCH_1_Y	        1100
#define M_TORCH_2_X	        2100
#define M_TORCH_2_Y	        1100

#define BASKET_1_X		    750
#define BASKET_1_Y		    150
#define BASKET_2_X		    2250
#define BASKET_2_Y		    150

#define MAMOOTH_1_X		    700
#define MAMOOTH_1_Y		    0
#define MAMOOTH_2_X		    2300
#define MAMOOTH_2_Y		    0  

#define FRESCO_X			1500
#define FRESCO_Y			0

//#define HOME_RED_X		    2800
//#define HOME_RED_Y		    300

//#define HOME_YELLOW_X		200
//#define HOME_YELLOW_Y		300

#define HEART_2_RAD	150

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
#define SIDE_FRONT 	    I2C_SIDE_FRONT 
#define SIDE_MAX		I2C_SIDE_MAX

#define OPPOSITE_SIDE(side) ((side==I2C_SIDE_FRONT)? (I2C_SIDE_REAR) : (I2C_SIDE_FRONT))	

#define GO_FORWARD	    0
#define GO_BACKWARD	    1

/* useful traj flags */
#define TRAJ_SUCCESS(f) 			(f & (END_TRAJ|END_NEAR))
#define TRAJ_BLOCKING(f) 			(f & (END_BLOCKING))

#define TRAJ_FLAGS_STD 				(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 		(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

#define LAST_SECONDS_TIME	80

//#define CALIBRATION
#ifdef CALIBRATION

/* default acc */
#define ACC_DIST  1.
#define ACC_ANGLE 1.

/* default speeds */
#define SPEED_DIST_FAST 		1000.
#define SPEED_ANGLE_FAST 		1000.
#define SPEED_DIST_SLOW 		1000.
#define SPEED_ANGLE_SLOW 		1000.
#define SPEED_DIST_VERY_SLOW 	1000.
#define SPEED_ANGLE_VERY_SLOW	1000.

#else

/* default acc */
#define ACC_DIST  20. //35.
#define ACC_ANGLE 20.

/* default speeds */
#ifdef HOMOLOGATION
#define SPEED_DIST_FAST 		2000.
#define SPEED_ANGLE_FAST 		2000.
#else

#define SPEED_DIST_FAST 		3000.
#define SPEED_ANGLE_FAST 		3000.
#endif

//Do not change
#define SPEED_DIST_SLOW 		2000.
#define SPEED_ANGLE_SLOW 		2000.
#define SPEED_DIST_VERY_SLOW 	500.
#define SPEED_ANGLE_VERY_SLOW   500.

#endif

/* zones */
#define ZONE_TREE_1				0
#define ZONE_TREE_2       		1
#define ZONE_TREE_3				2
#define ZONE_TREE_4				3
#define ZONE_HEART_1			4
#define ZONE_HEART_2_LEFT 		5
#define ZONE_HEART_3  			6
#define ZONE_HEART_2_UP			7
#define ZONE_HEART_2_DOWN		8
#define ZONE_HEART_2_RIGHT		9
#define ZONE_FIRE_1   	    	10
#define ZONE_FIRE_2	        	11
#define ZONE_FIRE_3	        	12
#define ZONE_FIRE_4	        	13
#define ZONE_FIRE_5				14
#define ZONE_FIRE_6				15
#define ZONE_TORCH_1   			16
#define ZONE_TORCH_2			17
#define ZONE_TORCH_3			18
#define ZONE_TORCH_4        	19
#define ZONE_M_TORCH_1 			20
#define ZONE_M_TORCH_2 			21
#define ZONE_BASKET_1    		22
#define ZONE_BASKET_2       	23
#define ZONE_MAMOOTH_1      	24
#define ZONE_MAMOOTH_2      	25
#define ZONE_FRESCO      		26
//#define ZONE_HOME_RED       	27
//#define ZONE_HOME_YELLOW    	28
#define ZONES_MAX		    	27

/* max number of each elements */
#define TREE_NB_MAX     4
#define FIRE_NB_MAX     6
#define HEART_NB_MAX    3
#define TORCH_NB_MAX    4
#define MTORCH_NB_MAX   2
#define MAMOOTH_NB_MAX  2
#define BASKET_NB_MAX   2


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
 * will do different things */
	uint8_t flags;
#define CONF_FLAG_XXX   1
};


/* strat structure */
typedef struct {
	/* type */
	uint16_t type;
	#define ZONE_TYPE_TREE			0
	#define ZONE_TYPE_FIRE			1
	#define ZONE_TYPE_HEART			2
	#define ZONE_TYPE_TORCH			3
	#define ZONE_TYPE_M_TORCH		4
	#define ZONE_TYPE_FRESCO		5
	#define ZONE_TYPE_MAMOOTH		6
	#define ZONE_TYPE_BASKET		7
	#define ZONE_TYPE_HOME			8
	#define ZONE_TYPE_MAX			9

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
	#define ZONE_PRIO_0			 0
	#define ZONE_PRIO_10		10
	#define ZONE_PRIO_20		20
	#define ZONE_PRIO_30		30
	#define ZONE_PRIO_40		40
	#define ZONE_PRIO_50		50
	#define ZONE_PRIO_60		60
	#define ZONE_PRIO_70		70
	#define ZONE_PRIO_80		80
	#define ZONE_PRIO_90		90
	#define ZONE_PRIO_100	   100
	#define ZONE_PRIO_MAX	   100

	uint16_t flags;
	#define ZONE_CHECKED	 	1
	#define ZONE_CHECKED_OPP	2
	#define ZONE_SEC_ROBOT	   	4
	#define ZONE_AVOID		   	8
  
  
	/* opponent statistics */
	microseconds opp_time_zone_us;
	microseconds last_time_opp_here; 	/*in us, since beginning of the match*/
	
	/* which robots can perform this action */
	uint8_t robot;
	#define MAIN_ROBOT  0
	#define SEC_ROBOT   1
	#define BOTH_ROBOTS 2
	
} strat_zones_t;

/* information about strat stuff */
struct strat_infos {
	uint8_t dump_enabled;
	uint8_t debug_step;
	struct bbox area_bbox;

    /* strat configuration */
	struct conf conf;

	/* points areas */
	strat_zones_t zones[ZONES_MAX];
	
	/* our zone position */
	uint8_t current_zone;
	uint8_t goto_zone;
	uint8_t last_zone;

	/* state of the robot */
	uint8_t harvested_trees;     /* One unity per harvested tree */
	uint8_t fires_inside; 		 /* One unity per fire inside */

	/* opponent zone position */
	uint8_t opp_current_zone;
	uint8_t opp2_current_zone;

	/* opponent statistics */
	uint8_t opp_score;
	uint8_t opp_harvested_trees;

    uint8_t tree_harvesting_interrumped;
};

extern struct strat_infos strat_infos;

/* get zone struct index */
extern char numzone2name[ZONES_MAX + 1][3];

/* points we get from each zone */
extern uint8_t strat_zones_points[ZONES_MAX];

#ifndef HOST_VERSION_OA_TEST

/************************************************************* 
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c 
 *******************************************/
void strat_set_bounding_box(uint8_t type);

void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

uint8_t strat_main(void);
uint8_t strat_begin(void);
uint8_t strat_begin_alcabot (void);

void strat_event(void *dummy);
void strat_event_enable(void);
void strat_event_disable(void);

/********************************************
 * in strat_fruits.c 
 *******************************************/

/* harvest fruits from trees */
uint8_t strat_harvest_fruits(int16_t x, int16_t y, uint8_t clean_before);
uint8_t strat_leave_fruits(void);
uint8_t strat_leave_fruits_clean(void);

/********************************************
 * in strat_fire.c 
 *******************************************/
/* goto orphan fire */
uint8_t strat_goto_orphan_fire (uint8_t zone_num) ;

/* harvest orphan fires  */
uint8_t strat_harvest_orphan_fire (uint8_t zone_num);

/* goto torch */
uint8_t strat_goto_torch (uint8_t zone_num);

/* harvest torch  */
uint8_t strat_harvest_torch (uint8_t zone_num);

/* goto mobile torch */
uint8_t strat_goto_mobile_torch (uint8_t zone_num);

/* pickup mobile torch, top fire */
uint8_t strat_pickup_mobile_torch_top (uint8_t zone_num);

/* pickup mobile torch, middle fire */
uint8_t strat_pickup_mobile_torch_mid (uint8_t zone_num);

/* pickup mobile torch, bottom fire */
inline uint8_t strat_pickup_mobile_torch_bot (uint8_t zone_num);

/* goto heart of fire */
uint8_t strat_goto_heart_fire (uint8_t zone_num);

/* dump stored fires on heart of fire making a puzzle */
uint8_t strat_make_puzzle_on_heart (uint8_t zone_num);


/********************************************
 * in strat_main.c 
 *******************************************/

uint8_t strat_main_loop(void);

/* return new work zone, -1 if any zone is found */
int8_t strat_get_new_zone(void);

/* return END_TRAJ if zone is reached */
uint8_t strat_goto_zone(uint8_t zone_num);

/* return END_TRAJ if the work is done */
uint8_t strat_work_on_zone(uint8_t zone_num);

/* debug state machines step to step */
void state_debug_wait_key_pressed(void);

/* smart play */
//#define DEBUG_STRAT_SMART
uint8_t strat_smart(void);
void recalculate_priorities(void);

/* tracking of zones where opp has been working */
void strat_opp_tracking (void);

/* homologation */
void strat_homologation(void);


uint8_t goto_basket_path_down(void);
uint8_t goto_basket_path_up(void);
uint8_t goto_basket_best_path(uint8_t protect_zone_num);
uint8_t strat_wipe_out(void);
void strat_initial_move(void);
uint8_t strat_leave_fruits_from_fresco(void);
uint8_t strat_leave_fruits_from_home_red(void);

#else /* HOST_VERSION_OA_TEST */

void strat_set_bounding_box(uint8_t type);

#endif /* HOST_VERSION_OA_TEST */


#endif
