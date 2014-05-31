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

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>
#include <rdline.h>

#include "../common/i2c_commands.h"
#include "actuator.h"
#include "ax12_user.h"
#include "state.h"
#include "main.h"

#define ACTUATORS_DEBUG(args...) 	DEBUG(E_USER_ACTUATORS, args)
#define ACTUATORS_NOTICE(args...) 	NOTICE(E_USER_ACTUATORS, args)
#define ACTUATORS_ERROR(args...) 	ERROR(E_USER_ACTUATORS, args)

/* AX12 stuff */
#define AX12_PULLING_TIME_us		5000L
#define AX12_WINDOW_POSITION		15
#define AX12_BLOCKING_TIMEOUT_us	600000L


/**************************  AX12 MANAGE FUNCTIONS ****************************/
struct ax12_traj 
{
#define AX12_K_IMP_DEG		(3.413) //(1024.0/300.0)  // 3.413 imp/deg
#define AX12_K_MS_DEG		(3.333) //(200.0/60.0)	// 3.333 ms/deg	
#define AX12_WINDOW_NO_NEAR	(5.0*AX12_K_IMP_DEG)  // 5 deg
#define AX12_WINDOW_NEAR	(10.0*AX12_K_IMP_DEG) // 20 deg

	uint8_t id;
	int16_t zero_offset_pos;

	int16_t goal_angle_deg;
	uint16_t goal_pos;

	microseconds goal_time_us;
    microseconds time_us;
    
    uint16_t pos;
	int16_t angle_deg;
};


/* set position */
void ax12_set_pos (struct ax12_traj *ax12, int16_t pos)
{
     /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (int16_t)(ax12->pos - ax12->zero_offset_pos);
 	ax12->angle_deg = (int16_t)(ax12->angle_deg / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_pos = pos;
    ax12->goal_angle_deg = (int16_t)((pos - ax12->zero_offset_pos) / AX12_K_IMP_DEG);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
	ax12->goal_time_us = (microseconds)(ABS(ax12->angle_deg - ax12->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->time_us = time_get_us2();
}

/* set angle */
void ax12_set_a (struct ax12_traj *ax12, int16_t a)
{
	printf ("%s, a = %d\n\r", __FUNCTION__, a);

    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
 	ax12->angle_deg = (int16_t)(ax12->pos - ax12->zero_offset_pos);
 	ax12->angle_deg = (int16_t)(ax12->angle_deg / AX12_K_IMP_DEG);

    /* set goal angle */
    ax12->goal_angle_deg = a;
    ax12->goal_pos = ax12->zero_offset_pos + (int16_t)(ax12->goal_angle_deg * AX12_K_IMP_DEG);

	//printf ("%s, goal pos = %d\n\r", __FUNCTION__, ax12->goal_pos);
	ax12_user_write_int(&gen.ax12, ax12->id , AA_GOAL_POSITION_L, ax12->goal_pos);

    /* update goal time */
	ax12->goal_time_us = (microseconds)(ABS(ax12->angle_deg - ax12->goal_angle_deg) * AX12_K_MS_DEG * 1000);
	ax12->time_us = time_get_us2();

	//printf ("da %d, goal_us %d\n\r", (int16_t)ABS(ax12->angle_deg - ax12->goal_angle_deg), ax12->goal_time_us);
	//printf ("a=%d, goal_a =%d\n\r", ax12->angle_deg, ax12->goal_angle_deg);
}

/* get angle */
int16_t ax12_get_a (struct ax12_traj *ax12)
{
    /* update current position/angle */
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
	ax12->angle_deg = ((ax12->pos - ax12->zero_offset_pos)); 
	ax12->angle_deg = (int16_t)(ax12->angle_deg  * (1.0/AX12_K_IMP_DEG));

	printf ("%s, a = %d\n\r", __FUNCTION__, ax12->angle_deg);
    return ax12->angle_deg;
}


const char *err_tab []= {
	"END_TRAJ",
	"END_NEAR",
	"END_TIME",
	"END_BLOCKING",
	"END_RESERVED",
};

/* return string from end traj type num */
const char *get_err (uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i=0 ; i<4; i++) {
		if (err & (1 <<i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}


uint8_t ax12_test_traj_end (struct ax12_traj *ax12, uint8_t flags)
{
	ax12_user_read_int(&gen.ax12, ax12->id, AA_PRESENT_POSITION_L, &ax12->pos);
    uint8_t ret = 0;

	if (flags & END_TRAJ)
		if (ABS(ax12->goal_pos - ax12->pos) < AX12_WINDOW_NO_NEAR)
			ret |= END_TRAJ;

	if (flags & END_NEAR)
		if (ABS(ax12->goal_pos - ax12->pos) < AX12_WINDOW_NEAR)
			ret |=  END_NEAR;

	if (flags & END_TIME)
		if (time_get_us2() - ax12->time_us > ax12->goal_time_us)
			ret |=  END_TIME;

    return ret;
}

/* wait traj end */
uint8_t ax12_wait_traj_end (struct ax12_traj *ax12, uint8_t flags) 
{
    microseconds us = time_get_us2();
    uint8_t ret = 0;

    while (ret == 0) {
        /* check end traj periodicaly (T = 5ms) */
        if (time_get_us2() - us >= 5000L) {
            ret = ax12_test_traj_end (ax12, flags);
            us = time_get_us2();
        }
    }
	DEBUG(E_USER_ACTUATORS, "Got %s",  get_err (ret));
    return ret;
}



/**** lift functions ********************************************************/

/* stop without rampe */
void lift_hard_stop(void)
{
	cs_set_consign(&slavedspic.lift.cs, encoders_dspic_get_value(LIFT_ENCODER));
	slavedspic.lift.qr.previous_var = 0;
	slavedspic.lift.qr.previous_out = encoders_dspic_get_value(LIFT_ENCODER);
}

/* calibrate initial position */
void lift_calibrate(void)
{
#define AUTOPOS_SPEED			50
#define AUTOPOS_ACCEL			1
#define AUTOPOS_BIG_DIST_mm		60000
#define AUTOPOS_BD_TIMEOUT_ms	5000

	int8_t ret;
	int16_t kp, ki, kd;

	/* save and set PID constants */
	kp = pid_get_gain_P(&slavedspic.lift.pid);
	ki = pid_get_gain_I(&slavedspic.lift.pid);
	kd = pid_get_gain_D(&slavedspic.lift.pid);
	pid_set_gains(&slavedspic.lift.pid,  5000, 0, 0);

	/* set low speed, and hi acceleration for fast response */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, AUTOPOS_SPEED, AUTOPOS_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, AUTOPOS_ACCEL, AUTOPOS_ACCEL);
	ACTUATORS_DEBUG("Down speed and acceleration");

	/* goto zero with cs */
	slavedspic.flags |= DO_CS;
	//cs_set_consign(&slavedspic.lift.cs, (int32_t)(AUTOPOS_BIG_DIST_mm * LIFT_K_IMP_mm));
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(AUTOPOS_BIG_DIST_mm));
	ACTUATORS_DEBUG("Goto zero");	

	/* wait end blocking */
	ret = WAIT_COND_OR_TIMEOUT(bd_get(&slavedspic.lift.bd), AUTOPOS_BD_TIMEOUT_ms);
	if(!ret) {
		ACTUATORS_DEBUG("Blocking not reached");
		return;
	}

	/* reset encoder */
	encoders_dspic_set_value(LIFT_ENCODER, LIFT_CALIB_IMP_MAX);
	lift_hard_stop();
	pid_reset(&slavedspic.lift.pid);
	bd_reset(&slavedspic.lift.bd);
	ACTUATORS_DEBUG("End blocking, encoder is reset to zero");	

	/* restore speed, acceleration and PID */
	quadramp_set_1st_order_vars(&slavedspic.lift.qr, LIFT_SPEED, LIFT_SPEED);
	quadramp_set_2nd_order_vars(&slavedspic.lift.qr, LIFT_ACCEL, LIFT_ACCEL);
	pid_set_gains(&slavedspic.lift.pid, kp, ki, kd);

	ACTUATORS_DEBUG("Restored speed, acceleration and PID");
	ACTUATORS_DEBUG("%s ends", __FUNCTION__);

	/* set calibration flag */
	slavedspic.lift.calibrated = 1;
	return;
}

/* set height in mm */
void lift_set_height(int32_t height_mm)
{
	/* check calibration flag */
	if(!slavedspic.lift.calibrated) {
		ACTUATORS_ERROR("Can't set height, lift is NOT CALIBRATED");
		return;
	}

	/* saturate heigh */
	if(height_mm > LIFT_HEIGHT_MAX_mm)
		height_mm = LIFT_HEIGHT_MAX_mm;
	if(height_mm < LIFT_HEIGHT_MIN_mm)
		height_mm = LIFT_HEIGHT_MIN_mm;

	/* apply consign */
	cs_set_consign(&slavedspic.lift.cs, (int32_t)(((int32_t)LIFT_HEIGHT_MAX_mm-height_mm) * LIFT_K_IMP_mm));
	slavedspic.lift.blocking = 0;
}

/* return heigh in mm */
int32_t lift_get_height(void)
{
	return (int32_t)((int32_t)LIFT_HEIGHT_MAX_mm - (encoders_dspic_get_value(LIFT_ENCODER)/LIFT_K_IMP_mm));
}

/* return 1 if height reached, -1 if blocking and zero if no ends yet */
int8_t lift_check_height_reached(void)
{
	/* test consign end */
	if(cs_get_consign(&slavedspic.lift.cs) == cs_get_filtered_consign(&slavedspic.lift.cs)){
		return END_TRAJ;
	}

	/* test blocking */
	if(bd_get(&slavedspic.lift.bd)) {
		lift_hard_stop();
		pid_reset(&slavedspic.lift.pid);
		bd_reset(&slavedspic.lift.bd);
		slavedspic.lift.blocking = 1;
		ACTUATORS_ERROR("Lift ENDS BLOCKING");
		return END_BLOCKING;
	}

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t lift_wait_end()
{
	uint8_t ret = 0;

	/* wait end */
	while(!ret)
		ret = lift_check_height_reached();

	return ret;
}



/**** combs funcions *********************************************************/

uint16_t combs_ax12_pos_l [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_L_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_L_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE]  = POS_COMB_L_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_L_HARVEST_OPEN, 
};

uint16_t combs_ax12_pos_r [COMBS_MODE_MAX] = {
	[COMBS_MODE_HIDE] 			= POS_COMB_R_HIDE, 
	[COMBS_MODE_OPEN] 			= POS_COMB_R_OPEN, 
	[COMBS_MODE_HARVEST_CLOSE]  = POS_COMB_R_HARVEST_CLOSE, 
	[COMBS_MODE_HARVEST_OPEN] 	= POS_COMB_R_HARVEST_OPEN, 
};

struct ax12_traj ax12_comb_l = { .id = AX12_ID_COMB_L, .zero_offset_pos = 0 };
struct ax12_traj ax12_comb_r = { .id = AX12_ID_COMB_R, .zero_offset_pos = 0 };

/* set finger position depends on mode */
int8_t combs_set_mode(combs_t *combs, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 possitions depends on mode and type */
	if(mode >= COMBS_MODE_MAX) {
		ACTUATORS_ERROR("Unknow COMBS MODE");
		return -1;
	}

   /* ax12 possitions */
	combs->mode_old = combs->mode;
	combs->mode = mode;
	combs->ax12_pos_l = combs_ax12_pos_l[combs->mode] + pos_offset;
	combs->ax12_pos_r = combs_ax12_pos_r[combs->mode] - pos_offset;

	/* set speed */
#if 0
   if(combs->mode == COMBS_MODE_CLOSE || combs->mode == COMBS_MODE_HOLD)) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 300);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 300);
	}
	else if(combs->type == COMBS_TYPE_TOTEM) {
		ax12_user_write_int(&gen.ax12, ax12_left_id, AA_MOVING_SPEED_L, 0x3ff);
		ax12_user_write_int(&gen.ax12, ax12_right_id, AA_MOVING_SPEED_L, 0x3ff);
	}
#endif

	/* saturate to position range */
	if(combs->ax12_pos_l > combs_ax12_pos_l[COMBS_MODE_L_POS_MAX])
		combs->ax12_pos_l = combs_ax12_pos_l[COMBS_MODE_L_POS_MAX];
	if(combs->ax12_pos_l < combs_ax12_pos_l[COMBS_MODE_L_POS_MIN])
		combs->ax12_pos_l = combs_ax12_pos_l[COMBS_MODE_L_POS_MIN];

	if(combs->ax12_pos_r > combs_ax12_pos_r[COMBS_MODE_R_POS_MAX])
		combs->ax12_pos_r = combs_ax12_pos_r[COMBS_MODE_R_POS_MAX];
	if(combs->ax12_pos_r < combs_ax12_pos_r[COMBS_MODE_R_POS_MIN])
		combs->ax12_pos_r = combs_ax12_pos_r[COMBS_MODE_R_POS_MIN];
 
	/* apply to ax12 */
    ax12_set_pos (&ax12_comb_l, combs->ax12_pos_l);
    ax12_set_pos (&ax12_comb_r, combs->ax12_pos_r);

	return 0;
}

/* return END_TRAJ or END_BLOCKING */
uint8_t combs_wait_end(combs_t *combs)
{
    uint8_t ret_l, ret_r;
   
    ret_l = ax12_wait_traj_end (&ax12_comb_l, END_TRAJ|END_TIME);
    ret_r = ax12_wait_traj_end (&ax12_comb_r, END_TRAJ|END_TIME);

    return (ret_l | ret_r);
}

/**** sticks funcions *********************************************************/
uint16_t stick_ax12_pos[STICK_TYPE_MAX][STICK_MODE_MAX] = {
	[STICK_TYPE_RIGHT][STICK_MODE_HIDE] 				= POS_STICK_R_HIDE,
	[STICK_TYPE_RIGHT][STICK_MODE_PUSH_FIRE] 			= POS_STICK_R_PUSH_FIRE,
	[STICK_TYPE_RIGHT][STICK_MODE_PUSH_TORCH_FIRE]		= POS_STICK_R_PUSH_TORCH_FIRE,
	[STICK_TYPE_RIGHT][STICK_MODE_CLEAN_FLOOR] 			= POS_STICK_R_CLEAN_FLOOR,
	[STICK_TYPE_RIGHT][STICK_MODE_CLEAN_HEART] 			= POS_STICK_R_CLEAN_HEART,

	[STICK_TYPE_LEFT][STICK_MODE_HIDE] 					= POS_STICK_L_HIDE,
	[STICK_TYPE_LEFT][STICK_MODE_PUSH_FIRE] 			= POS_STICK_L_PUSH_FIRE,
	[STICK_TYPE_LEFT][STICK_MODE_PUSH_TORCH_FIRE]		= POS_STICK_L_PUSH_TORCH_FIRE,
	[STICK_TYPE_LEFT][STICK_MODE_CLEAN_FLOOR] 			= POS_STICK_L_CLEAN_FLOOR,
	[STICK_TYPE_LEFT][STICK_MODE_CLEAN_HEART] 			= POS_STICK_L_CLEAN_HEART,
};

struct ax12_traj ax12_stick_l = { .id = AX12_ID_STICK_L, .zero_offset_pos = 0 };
struct ax12_traj ax12_stick_r = { .id = AX12_ID_STICK_R, .zero_offset_pos = 0 };

/* set finger position depends on mode */
uint8_t stick_set_mode(stick_t *stick, uint8_t mode, int16_t pos_offset)
{	
	/* set ax12 possitions depends on mode and type */
	if(mode >= STICK_MODE_MAX) {
		ACTUATORS_ERROR("Unknow %s STICK MODE", stick->type == STICK_TYPE_RIGHT? "RIGHT":"LEFT");
		return -1;
	}

	stick->mode = mode;
	if(stick->type == STICK_TYPE_RIGHT)
		stick->ax12_pos = stick_ax12_pos[stick->type][stick->mode] + pos_offset;
	else
		stick->ax12_pos = stick_ax12_pos[stick->type][stick->mode] + pos_offset;
	
	/* saturate to position range */
	if(stick->type == STICK_TYPE_LEFT) {
		if(stick->ax12_pos > stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MAX])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MAX];
		if(stick->ax12_pos < stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MIN])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_LEFT][STICK_MODE_L_POS_MIN];
	} 
	else {
		if(stick->ax12_pos > stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MAX])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MAX];
		if(stick->ax12_pos < stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MIN])
			stick->ax12_pos = stick_ax12_pos[STICK_TYPE_RIGHT][STICK_MODE_R_POS_MIN];
	}

    if(stick->type == STICK_TYPE_LEFT)
        ax12_set_pos (&ax12_stick_l, stick->ax12_pos);
    else
        ax12_set_pos (&ax12_stick_r, stick->ax12_pos);

	return 0;
}


/* return END_TRAJ or END_BLOCKING */
uint8_t stick_wait_end(stick_t *stick)
{
    if(stick->type == STICK_TYPE_LEFT)
        return ax12_wait_traj_end (&ax12_stick_l, END_TRAJ|END_TIME);
    else
        return ax12_wait_traj_end (&ax12_stick_r, END_TRAJ|END_TIME);
}


/**** boot funcions *********************************************************/

/* set boot position depends on mode */
void boot_door_set_mode(boot_t *boot, uint8_t door_mode)
{
	uint8_t pos_saturated = 0;

	switch(door_mode) {
		case BOOT_DOOR_MODE_OPEN:
			boot->door_servo_pos = pwm_servo_set(PWM_SERVO_BOOT_DOOR, POS_BOOT_DOOR_OPEN);
			boot->door_mode = door_mode;
			pos_saturated = boot->door_servo_pos != POS_BOOT_DOOR_OPEN? 1:0;
			break;
		case BOOT_DOOR_MODE_CLOSE:
			boot->door_servo_pos = pwm_servo_set(PWM_SERVO_BOOT_DOOR, POS_BOOT_DOOR_CLOSE);
			boot->door_mode = door_mode;
			pos_saturated = boot->door_servo_pos != POS_BOOT_DOOR_CLOSE? 1:0;
			break;
		default:
			ACTUATORS_ERROR("Unknown BOOT MODE");
			break;
	}
	if(pos_saturated)
		ACTUATORS_ERROR("Boot position saturated");
}

void boot_tray_set_mode(boot_t *boot, uint8_t tray_mode)
{
	uint8_t pos_saturated = 0;

	switch(tray_mode) {
		case BOOT_TRAY_MODE_DOWN:
			pwm_mc_set(PWM_MC_BOOT_TRAY, 0);
			break;
		case BOOT_TRAY_MODE_VIBRATE:
			pwm_mc_set(PWM_MC_BOOT_TRAY, BOOT_TRAY_VIBRATE_PWM);
			break;
		default:
			ACTUATORS_ERROR("Unknown BOOT TRAY MODE");
			break;
	}
	if(pos_saturated)
		ACTUATORS_ERROR("Boot tray position saturated");

}


/**** tree tray funcions *********************************************************/
uint16_t tree_tray_ax12_pos [TREE_TRAY_MODE_MAX] = {
	[TREE_TRAY_MODE_OPEN] 		= POS_TREE_TRAY_OPEN,
	[TREE_TRAY_MODE_CLOSE] 		= POS_TREE_TRAY_CLOSE,
	[TREE_TRAY_MODE_HARVEST]	= POS_TREE_TRAY_HARVEST,
};


struct ax12_traj ax12_tree_tray = { .id = AX12_ID_TREE_TRAY, .zero_offset_pos = 0 };


/* set tree_tray position depends on mode */
uint8_t tree_tray_set_mode(tree_tray_t *tree_tray, uint8_t mode, int16_t pos_offset)
{
	/* set ax12 possitions depends on mode and type */
	if(mode >= TREE_TRAY_MODE_MAX) {
		ACTUATORS_ERROR("Unknow TREE TRAY MODE");
		return -1;
	}

	tree_tray->mode = mode;
	tree_tray->ax12_pos = tree_tray_ax12_pos[tree_tray->mode] + pos_offset;

	
	/* saturate to position range */
	if(tree_tray->ax12_pos > tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MAX])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MAX];
	if(tree_tray->ax12_pos < tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MIN])
		tree_tray->ax12_pos = tree_tray_ax12_pos[TREE_TRAY_MODE_POS_MIN];

	/* apply to ax12 */
    ax12_set_pos (&ax12_tree_tray, tree_tray->ax12_pos);

	return 0;
}

/* return END_TRAJ or END_TIME */
uint8_t tree_tray_wait_end(tree_tray_t *tree_tray)
{
    return ax12_wait_traj_end (&ax12_tree_tray, END_TRAJ|END_TIME);
}

/**** vacuum funcions *********************************************************/

void vacuum_motor_set (uint8_t num, uint8_t on) {

	if (num == 1)
		_LATC1 = !on;
	else if (num == 2)
		_LATB3 = !on;
}

void vacuum_ev_set (uint8_t num, uint8_t on) 
{
	if (num == 1)
		_LATB10 = on;
	else if (num == 2)
		_LATB11 = on;
}

void vacuum_system_enable (uint8_t num) {
	vacuum_ev_set (num, 1);
	time_wait_ms (500);
	vacuum_motor_set (num, 1);
}

void vacuum_system_disable (uint8_t num) {
	vacuum_ev_set (num, 0);
	vacuum_motor_set (num, 0);
}



/****************************  ARM  *******************************************/

#define SHOULDER_JOIN_X   (165-21) // 144
#define SHOULDER_JOIN_Y   (130)
#define ARM_LENGTH        (219.0)
#define SUCKER_LENGTH_0	  (46.5)
#define SUCKER_LENGTH_180 (55.5)

#define ARM_X_MAX   (SHOULDER_JOIN_X + ARM_LENGTH)
#define ARM_X_MIN   (SHOULDER_JOIN_X - ARM_LENGTH)

#define ARM_Y_MAX   (SHOULDER_JOIN_Y + ARM_LENGTH) //363
#define ARM_Y_MIN   (SHOULDER_JOIN_Y)			   //144

#define ARM_H_MAX   (LIFT_HEIGHT_MAX_mm)
#define ARM_H_MIN   (LIFT_HEIGHT_MAX_mm)

#define ARM_SHOULDER_A_MAX   (180)
#define ARM_SHOULDER_A_MIN   (0)

#define ARM_ELBOW_A_MAX (180)
#define ARM_ELBOW_A_MIN (0)

#define ARM_WRIST_A_MAX (+90)
#define ARM_WRIST_A_MIN (-90)

/* some conversions and constants */
#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

struct ax12_traj ax12_shoulder = { .id = AX12_ID_SHOULDER, .zero_offset_pos = 805 };
struct ax12_traj ax12_elbow    = { .id = AX12_ID_ELBOW,    .zero_offset_pos = 822 };
struct ax12_traj ax12_wrist    = { .id = AX12_ID_WRIST,    .zero_offset_pos = 650 };

struct arm 
{
    int16_t h;
    int16_t x;
    int16_t y;
    int16_t a;

    int16_t elbow_a;
    int16_t wrist_a;
};

/*** Helper functions ********************************************************/
void arm_a_to_xy (int16_t a, int16_t *x, int16_t *y)
{
    double x1, y1;

    x1 = ARM_LENGTH * cos(RAD(a));
    y1 = ARM_LENGTH * sin(RAD(a));

    *x = (int16_t)x1 + SHOULDER_JOIN_X;
    *y = (int16_t)y1 + SHOULDER_JOIN_Y;
}

void arm_x_to_ay (int16_t x, int16_t *a, int16_t *y)
{
    double a1, x1, y1;

	x1 = (double) (x - SHOULDER_JOIN_X);
    a1 = acos (x1 / ARM_LENGTH);
    y1 = ARM_LENGTH * sin(a1);

	printf ("a1 = %f, x1 = %f, y1 = %f\n\r", DEG(a1), x1, y1);

    *a = (int16_t)DEG(a1);
    *y = (int16_t)y1 + SHOULDER_JOIN_Y; 

	printf ("a = %d, y = %d\n\r", *a, *y);  
}

void arm_y_to_ax (int16_t y, int16_t *a, int16_t *x)
{
    double a1, x1, y1;

	y1 = (double) (y - SHOULDER_JOIN_Y);
    a1 = M_PI - asin (y1 / ARM_LENGTH);
    x1 = ARM_LENGTH * cos(a1);

	printf ("a1 = %f, x1 = %f, y1 = %f\n\r", DEG(a1), x1, y1);

    *a = (int16_t)DEG(a1);
    *x = (int16_t)x1 + SHOULDER_JOIN_X;   

	printf ("a = %d, x = %d\n\r", *a, *x);  
}

/*** Joins functions *********************************************************/

/* shoulder angle */
void arm_shoulder_goto_a_abs (int16_t a)
{
    /* check limints */
    if (a > ARM_SHOULDER_A_MAX)   a = ARM_SHOULDER_A_MAX;
    if (a < ARM_SHOULDER_A_MIN)   a = ARM_SHOULDER_A_MIN;

    /* set pos, XXX set angle inverted */
    ax12_set_a (&ax12_shoulder, -a);
}

uint8_t arm_shoulder_wait_traj_end (uint8_t flags) {
    return ax12_wait_traj_end(&ax12_shoulder, flags);
}

int16_t arm_shoulder_get_a (void) {
	/* XXX get invert angle */
	return (-ax12_get_a(&ax12_shoulder));
}

/* elbow angle */
void arm_elbow_goto_a_abs (int16_t a)
{
    /* check limints */
    if (a > ARM_ELBOW_A_MAX)   a = ARM_ELBOW_A_MAX;
    if (a < ARM_ELBOW_A_MIN)   a = ARM_ELBOW_A_MIN;

    /* set pos, XXX set angle inverted */
    ax12_set_a (&ax12_elbow, -a);
}

void arm_elbow_goto_a_rel (int16_t a)
{
    /* calculate a abs */
	/* XXX get invert angle */
    a = -ax12_get_a(&ax12_elbow) + a;

    /* check limints */
    if (a > ARM_ELBOW_A_MAX)   a = ARM_ELBOW_A_MAX;
    if (a < ARM_ELBOW_A_MIN)   a = ARM_ELBOW_A_MIN;
  
    /* set pos, XXX set angle inverted */
    ax12_set_a (&ax12_elbow, -a);
}

int16_t arm_elbow_get_a (void)
{
    /* calculate a abs */
	/* XXX get invert angle */
    return (-ax12_get_a(&ax12_elbow));
}


uint8_t arm_elbow_wait_traj_end (uint8_t flags) {
    return ax12_wait_traj_end(&ax12_elbow, flags);
}

/* wrist angle */
void arm_wrist_goto_a_abs (int16_t a)
{
    /* check limints */
    if (a > ARM_WRIST_A_MAX)   a = ARM_WRIST_A_MAX;
    if (a < ARM_WRIST_A_MIN)   a = ARM_WRIST_A_MIN;

    /* set pos */
    ax12_set_a (&ax12_wrist, a);
}

void arm_wrist_goto_a_rel (int16_t a)
{
    /* calculate a abs */
    a = ax12_get_a (&ax12_wrist) + a;

    /* check limints */
    if (a > ARM_WRIST_A_MAX)   a = ARM_WRIST_A_MAX;
    if (a < ARM_WRIST_A_MIN)   a = ARM_WRIST_A_MIN;

    /* set pos */
    ax12_set_a (&ax12_wrist, a);
}

uint8_t arm_wrist_wait_traj_end (uint8_t flags) {
    return ax12_wait_traj_end(&ax12_wrist, flags);
}

int16_t arm_wrist_get_a (void) {
    return ax12_get_a (&ax12_wrist);
}

/* set height, relative to current sucker angle 
 * XXX elbow angle is taken in account */
void arm_goto_h (int16_t h)
{
    int16_t elbow_a;
    float sucker_offset = 0.0;
    int16_t h_sucker=0;
   
    /* calculate sucker height */
    elbow_a = arm_elbow_get_a();

	printf ("h_sucker (%d) = %d - %d = %d\n\r", elbow_a, h, (int16_t)sucker_offset, (int16_t)h_sucker);
 
	if (elbow_a <= 90)
    	sucker_offset = SUCKER_LENGTH_180 - (SUCKER_LENGTH_0 * cos(RAD(elbow_a)));
	else 
    	sucker_offset = SUCKER_LENGTH_180 + (SUCKER_LENGTH_180 * cos(RAD(elbow_a)));

    h_sucker = h;
	h_sucker -= (int16_t)sucker_offset;

	//printf ("h_sucker (%d) = %d - %d = %d\n\r", elbow_a, h, (int16_t)sucker_offset, (int16_t)h_sucker);
 
    lift_set_height (h_sucker);
}

/* set height, relative to elbow goal angle given as parameter 
 * XXX elbow angle is taken in account */
void arm_goto_h_elbow_a (int16_t h, int16_t elbow_a)
{
    float sucker_offset = 0.0;
    int16_t h_sucker;
   
    /* calculate sucker height */
	if (elbow_a <= 90)
    	sucker_offset = SUCKER_LENGTH_180 - (SUCKER_LENGTH_0 * cos(RAD(elbow_a)));
	else 
    	sucker_offset = SUCKER_LENGTH_180 + (SUCKER_LENGTH_180 * cos(RAD(elbow_a)));

    h_sucker = h;
	h_sucker -= (int16_t)sucker_offset;

	printf ("h_sucker (%d) = %d - %d = %d\n\r", elbow_a, h, (int16_t)sucker_offset, (int16_t)h_sucker);
 
    lift_set_height (h_sucker);
}

int16_t arm_get_h (void) 
{
	int16_t elbow_a = arm_elbow_get_a();

	if (elbow_a <= 90)
		return lift_get_height();
	else
		return (lift_get_height() + (SUCKER_LENGTH_180-SUCKER_LENGTH_0));
}


uint8_t arm_h_wait_traj_end (void)
{
    return lift_wait_end();
}


/* goto x coordinate, relative to robot zero coordinates.
 * XXX suposes elbow angle of 0 deg (sucker in parallel with ground) */
void arm_goto_x (int16_t x)
{
    int16_t a,y;

	/* check range */
	if (x > ARM_X_MAX) x = ARM_X_MAX;
	if (x < ARM_X_MIN) x = ARM_X_MIN;

    /* calculate angle pos */
    arm_x_to_ay (x, &a, &y); 

    /* set pos */
	arm_shoulder_goto_a_abs (a);
}

/* goto y coordinate, relative to robot zero coordinates.
 * XXX suposes elbow angle of 0 deg (sucker in parallel with ground) */
void arm_goto_y (int16_t y)
{
    int16_t a,x;

	/* check range */
	if (x > ARM_Y_MAX) x = ARM_Y_MAX;
	if (x < ARM_Y_MIN) x = ARM_Y_MIN;

    /* calculate angle pos */
    arm_y_to_ax (y, &a, &x); 

    /* set pos */
	arm_shoulder_goto_a_abs (a);
}

uint8_t arm_xy_wait_traj_end (uint8_t flags) {
    return arm_shoulder_wait_traj_end (flags);
}


void arm_goto_hx (int16_t h, int16_t x, int16_t elbow_a, int16_t wrist_a)
{
#define SHOULDER_A_SAFE (145)

	int16_t shoulder_h, shoulder_a, shoulder_a_final, y_final;

	shoulder_a = arm_shoulder_get_a ();
	shoulder_h = arm_get_h ();
	arm_x_to_ay (x, &shoulder_a_final, &y_final);

	/* goto safe position */
	/*if ((shoulder_a > SHOULDER_A_SAFE) && 
		(shoulder_a_final > SHOULDER_A_SAFE)) {
		arm_shoulder_goto_a_abs (SHOULDER_A_SAFE);
		arm_shoulder_wait_traj_end (END_TRAJ);
	}*/


	/* depending on direction of movement */
	if (shoulder_a_final <= shoulder_a)
	{
		/* form inside to outside */
		//ax12_user_write_int(&gen.ax12, AX12_ID_SHOULDER, AA_MOVING_SPEED_L, 0x3FF);

		/* goto final position */
		arm_goto_x (x);
		/* TODO detect safe shoulder angle for next movement */
		arm_xy_wait_traj_end (END_TRAJ|END_NEAR|END_TIME);

		arm_goto_h_elbow_a (h, elbow_a);	
		arm_elbow_goto_a_abs (elbow_a);
		arm_wrist_goto_a_abs (wrist_a);

		/* wait end positions reached */
		arm_xy_wait_traj_end (END_TRAJ|END_TIME);
		arm_h_wait_traj_end ();
		arm_elbow_wait_traj_end (END_TRAJ|END_TIME);
		arm_wrist_wait_traj_end (END_TRAJ|END_TIME);
	}
	else {
		/* form outside to inside */
		//ax12_user_write_int(&gen.ax12, AX12_ID_SHOULDER, AA_MOVING_SPEED_L, 125);

		/* set all except shoulder angle */
		arm_goto_h_elbow_a (h, elbow_a);
		arm_elbow_goto_a_abs (elbow_a);
		arm_wrist_goto_a_abs (wrist_a);

		/* wait for h reached */
		arm_h_wait_traj_end();
		arm_elbow_wait_traj_end (END_TRAJ|END_TIME);
		arm_wrist_wait_traj_end (END_TRAJ|END_TIME);
	
		/* goto final position */
		arm_goto_x (x);
		
		/* wait final positino reached */
		arm_xy_wait_traj_end(END_TRAJ|END_TIME);
	}
}




/* init all actuators */
void actuator_init(void)
{
//#define PROGRAM_AX12
#ifdef PROGRAM_AX12
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_SHUTDOWN, 0x24);
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_ALARM_LED, 0x24);

	/* specific config for mirror ax12, angle is limited */ 
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CW_ANGLE_LIMIT_L, 0x00);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_CCW_ANGLE_LIMIT_L, 0x3FF);
	ax12_user_write_int(&gen.ax12, AX12_BROADCAST_ID, AA_MOVING_SPEED_L, 0x3FF);
#endif

	/* init structures */
	slavedspic.stick_l.type = STICK_TYPE_LEFT;
	slavedspic.stick_r.type = STICK_TYPE_RIGHT;
}

