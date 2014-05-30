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


#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <aversive.h>
#include <aversive/error.h>
#include <clock_time.h>


#define old_version

#define LIFT_SPEED						100
#define LIFT_ACCEL						1
#define LIFT_K_IMP_mm					(-51474.0/250.0)
#define LIFT_CALIB_IMP_MAX				0
#define LIFT_HEIGHT_MAX_mm				250
#define LIFT_HEIGHT_MIN_mm				1



#define POS_COMB_R_OPEN				600
#define POS_COMB_R_HARVEST_OPEN		556
#define POS_COMB_R_HARVEST_CLOSE 	503
#define POS_COMB_R_HIDE				224

#define POS_COMB_L_OPEN				420
#define POS_COMB_L_HARVEST_OPEN		469
#define POS_COMB_L_HARVEST_CLOSE 	524
#define POS_COMB_L_HIDE				792

#define POS_STICK_R_OFFSET				(770-542) /* after stick repair */
#define POS_STICK_R_HIDE				(542 + POS_STICK_R_OFFSET)
#define POS_STICK_R_PUSH_TORCH_FIRE		(350 + POS_STICK_R_OFFSET)				
#define POS_STICK_R_PUSH_FIRE			(335 + POS_STICK_R_OFFSET)
#define POS_STICK_R_CLEAN_HEART			(311 + POS_STICK_R_OFFSET)
#define POS_STICK_R_CLEAN_FLOOR			(260 + POS_STICK_R_OFFSET)

#define POS_STICK_L_OFFSET				(246-241) /* after stick repair */
#define POS_STICK_L_HIDE				(241 + POS_STICK_L_OFFSET)
#define POS_STICK_L_PUSH_TORCH_FIRE		(439 + POS_STICK_L_OFFSET)
#define POS_STICK_L_PUSH_FIRE			(455 + POS_STICK_L_OFFSET)
#define POS_STICK_L_CLEAN_HEART			(480 + POS_STICK_L_OFFSET)
#define POS_STICK_L_CLEAN_FLOOR			(525 + POS_STICK_L_OFFSET)


#define POS_BOOT_DOOR_OPEN		700			
#define POS_BOOT_DOOR_CLOSE		960

#define BOOT_TRAY_VIBRATE_PWM	(2000)

#define POS_TREE_TRAY_OPEN		800
#define POS_TREE_TRAY_HARVEST	530 //506
#define POS_TREE_TRAY_CLOSE	358

#define END_TRAJ		1
#define END_BLOCKING	2


/* init actuators */
void actuator_init(void);

/**** lift functions ********************************************************/

/* stop without rampe */
void lift_hard_stop(void);

/* calibrate initial position */
void lift_calibrate(void);

/* set height in mm */
void lift_set_height(int32_t height_mm);

/* return heigh in mm */
int32_t lift_get_height(void);

/* return END_TRAJ or END_BLOCKING */
int8_t lift_check_height_reached(void);

/* return END_TRAJ or END_BLOCKING */
uint8_t lift_wait_end();



/**** combs funcions *********************************************************/

typedef struct {

	uint8_t mode;
	uint8_t mode_old;
#define COMBS_MODE_HIDE				0
#define COMBS_MODE_OPEN				1
#define COMBS_MODE_HARVEST_CLOSE	2
#define COMBS_MODE_HARVEST_OPEN	    3
#define COMBS_MODE_MAX				4

#define COMBS_MODE_R_POS_MAX		1	
#define COMBS_MODE_R_POS_MIN		0
#define COMBS_MODE_L_POS_MAX		0
#define COMBS_MODE_L_POS_MIN		1

	uint16_t ax12_pos_l;
	uint16_t ax12_pos_r;

} combs_t;

/* set combs position depends on mode */
int8_t combs_set_mode(combs_t *combs, uint8_t mode, int16_t pos_offset);

/* return END_TRAJ or END_TIMER */
uint8_t combs_wait_end(combs_t *combs);



/****sticks funcions *********************************************************/
typedef struct {
	uint8_t type;
#define STICK_TYPE_RIGHT	0
#define STICK_TYPE_LEFT		1
#define STICK_TYPE_MAX		2	

	uint8_t mode;
#define STICK_MODE_HIDE					0
#define STICK_MODE_PUSH_FIRE			1
#define STICK_MODE_PUSH_TORCH_FIRE		2
#define STICK_MODE_CLEAN_FLOOR			3
#define STICK_MODE_CLEAN_HEART			4
#define STICK_MODE_MAX					5

#define STICK_MODE_L_POS_MAX			3	
#define STICK_MODE_L_POS_MIN			0
#define STICK_MODE_R_POS_MAX			0
#define STICK_MODE_R_POS_MIN			3

	uint16_t ax12_pos;

} stick_t;

/* set stick position depends on mode */
uint8_t stick_set_mode(stick_t *stick, uint8_t mode, int16_t pos_offset);


/* return END_TRAJ or END_TIME */
uint8_t stick_wait_end(stick_t *stick);



/**** boot funcions *********************************************************/
typedef struct {
	uint8_t door_mode;
#define BOOT_DOOR_MODE_OPEN			0
#define BOOT_DOOR_MODE_CLOSE		1
#define BOOT_DOOR_MODE_MAX			2

	uint8_t tray_mode;
#define BOOT_TRAY_MODE_DOWN			0
#define BOOT_TRAY_MODE_VIBRATE		1
#define BOOT_TRAY_MODE_MAX			2

	uint16_t door_servo_pos;
	int16_t tray_motor_pwm;

} boot_t;

/* open/close boot door */
void boot_door_set_mode(boot_t *boot, uint8_t door_mode);

/* enable/disable boot vibration */
void boot_tray_set_mode(boot_t *boot, uint8_t tray_mode);

/**** trays funcions *********************************************************/

typedef struct {
	uint8_t mode;

#define TREE_TRAY_MODE_OPEN 		0	
#define TREE_TRAY_MODE_CLOSE 		1		
#define TREE_TRAY_MODE_HARVEST	2	
#define TREE_TRAY_MODE_MAX			3

#define TREE_TRAY_MODE_POS_MAX	0
#define TREE_TRAY_MODE_POS_MIN	1

#if 0
	microseconds time_us;
	uint8_t blocking;
#endif

	uint16_t ax12_pos;
} tree_tray_t;

/* set stick position depends on mode */
uint8_t tree_tray_set_mode(tree_tray_t *tree_tray, uint8_t mode, int16_t pos_offset);

#if 0
/* return END_TRAJ or END_BLOCKING */
int8_t tree_tray_check_mode_done(tree_tray_t *tree_tray);
#endif

/* return END_TRAJ or END_BLOCKING */
uint8_t tree_tray_wait_end(tree_tray_t *tree_tray);


/**** vacuum funcions *********************************************************/

void vacuum_motor_set (uint8_t num, uint8_t on);
void vacuum_ev_set (uint8_t num, uint8_t on);

void vacuum_system_enable (uint8_t num);
void vacuum_system_disable (uint8_t num);




#endif /* _ACTUATOR_H_ */


