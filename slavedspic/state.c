/*  
 *  Copyright Droids Corporation (2009)
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
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  state.c,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */


#include <math.h>
#include <string.h>

#include <aversive.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <ax12.h>
#include <uart.h>
#include <timer.h>
#include <scheduler.h>
#include <clock_time.h>

#include <rdline.h>
#include <vt100.h>

#include "../common/i2c_commands.h"

#include "ax12_user.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "state.h"
#include "main.h"

#define STMCH_DEBUG(args...) DEBUG(E_USER_ST_MACH, args)
#define STMCH_NOTICE(args...) NOTICE(E_USER_ST_MACH, args)
#define STMCH_ERROR(args...) ERROR(E_USER_ST_MACH, args)


/* shorter aliases for this file */
#define INIT				I2C_SLAVEDSPIC_MODE_INIT
#define POWER_OFF			I2C_SLAVEDSPIC_MODE_POWER_OFF
#define BOOT_TRAY			I2C_SLAVEDSPIC_MODE_BOOT_TRAY
#define BOOT_DOOR			I2C_SLAVEDSPIC_MODE_BOOT_DOOR
#define COMBS				I2C_SLAVEDSPIC_MODE_COMBS
#define TREE_TRAY			I2C_SLAVEDSPIC_MODE_TREE_TRAY
#define STICK				I2C_SLAVEDSPIC_MODE_STICK

#define HARVEST_FRUITS 		I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS
#define DUMP_FRUITS			I2C_SLAVEDSPIC_MODE_DUMP_FRUITS
#define ARM_GOTO			I2C_SLAVEDSPIC_MODE_ARM_GOTO
#define ARM					I2C_SLAVEDSPIC_MODE_ARM



static struct i2c_cmd_slavedspic_set_mode mainboard_command;
static volatile uint8_t prev_state;
static volatile uint8_t mode_changed = 0;
uint8_t state_debug = 0;

/* set status value */
void state_set_status(uint8_t val)
{
	uint8_t flags;

	IRQ_LOCK(flags);
   slavedspic.status = val;
	IRQ_UNLOCK(flags);

	STMCH_DEBUG("state = %s", slavedspic.status==I2C_SLAVEDSPIC_STATUS_BUSY? "BUSY":"READY");
}

/* set a new state, return 0 on success */
int8_t state_set_mode(struct i2c_cmd_slavedspic_set_mode *cmd)
{
	prev_state = mainboard_command.mode;
	memcpy(&mainboard_command, cmd, sizeof(mainboard_command));
	//STMCH_DEBUG("%s mode=%d", __FUNCTION__, mainboard_command.mode); FIXME: bloking???

	/* XXX power off mode */
	if (mainboard_command.mode == POWER_OFF) {

		/* lift */
		#ifdef notyet
		slavedspic.lift.on = 0;
		dac_mc_set(&gen.dac_mc_left, 0);
		BRAKE_ON();
		#endif

		/* ax12 */
		ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0x00);

		wait_ms(100);
		pwm_servo_disable();
      
      state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
	}
	else {
		mode_changed = 1;
	}
	return 0;
}

/* get last mode */
uint8_t state_get_mode(void)
{
	return mainboard_command.mode;
}


/* check that state is the one in parameter and that state changed */
uint8_t state_check_update(uint8_t mode)
{
	if ((mode == mainboard_command.mode) && (mode_changed == 1)){
		mode_changed = 0;
		return 1;
	}
	return 0;
}

/* debug state machines step to step */
void state_debug_wait_key_pressed(void)
{
	if (!state_debug)
		return;
	printf_P(PSTR("press a key\r\n"));
	while(!cmdline_keypressed());
}

/* init mode */
static void state_do_init(void)
{
	if (!state_check_update(INIT))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	state_init();
	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

  	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}



/**
 * *************** simple actuators modes ***********
 */

/* set boot tray mode */
void state_do_boot_tray_mode(void)
{
	/* return if no update */
	if (!state_check_update(BOOT_TRAY))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set boot tray mode */
	boot_tray_set_mode(&slavedspic.boot, mainboard_command.boot_tray.mode);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set boot door mode */
void state_do_boot_door_mode(void)
{
	/* return if no update */
	if (!state_check_update(BOOT_DOOR))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set boot door mode */
	boot_door_set_mode(&slavedspic.boot, mainboard_command.boot_door.mode);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* set combs mode */
void state_do_combs_mode(void)
{
	/* return if no update */
	if (!state_check_update(COMBS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set combs mode */
	if(combs_set_mode(&slavedspic.combs, mainboard_command.combs.mode, mainboard_command.combs.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	combs_wait_end(&slavedspic.combs);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set tree tray mode */
void state_do_tree_tray_mode(void)
{
	/* return if no update */
	if (!state_check_update(TREE_TRAY))
		return;

	/* set tree tray mode */
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	/* set combs mode */
	if(tree_tray_set_mode(&slavedspic.tree_tray, mainboard_command.tree_tray.mode, mainboard_command.tree_tray.offset))
		STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

	tree_tray_wait_end(&slavedspic.tree_tray);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

/* set stick mode */
void state_do_stick_mode(void)
{
//#define STICK_MODES_NB_TRIES 3
	uint8_t err = 0;
//	uint8_t nb_tries = 0;

	/* return if no update */
	if (!state_check_update(STICK))
		return;

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.stick_mode = mainboard_command.stick.mode;
	slavedspic.stick_offset = mainboard_command.stick.offset;

	/*** RIGHT STICK */
	if(mainboard_command.stick.type == I2C_STICK_TYPE_RIGHT) {
		/* hide the other */
//retry_hide_left:
		if(stick_set_mode(&slavedspic.stick_l, STICK_MODE_HIDE, 0))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stick_wait_end(&slavedspic.stick_l);
#if 0
		if((err & END_BLOCKING) && (nb_tries < STICK_MODES_NB_TRIES)) {
			nb_tries ++;
			goto retry_hide_left;
		}

		/* set right */
		nb_tries = 0;
#endif
//retry_right:
		if(stick_set_mode(&slavedspic.stick_r, slavedspic.stick_mode, 
								slavedspic.stick_offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stick_wait_end(&slavedspic.stick_r);
#if 0
		if((err & END_BLOCKING) && (nb_tries < STICK_MODES_NB_TRIES)) {
			nb_tries ++;
			goto retry_right;
		}
#endif

	}	

	/*** LEFT_STICK */
	else if(mainboard_command.stick.type == I2C_STICK_TYPE_LEFT) {

		/* hide the other */
//retry_hide_right:
		if(stick_set_mode(&slavedspic.stick_r, STICK_MODE_HIDE, 0))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stick_wait_end(&slavedspic.stick_r);
#if 0
		if((err & END_BLOCKING) && (nb_tries < STICK_MODES_NB_TRIES)) {
			nb_tries ++;
			goto retry_hide_right;
		}
		/* set right */
		nb_tries = 0;
#endif
//retry_left:
		if(stick_set_mode(&slavedspic.stick_l, slavedspic.stick_mode, 
								slavedspic.stick_offset))
			STMCH_ERROR("ERROR %s mode=%d", __FUNCTION__, state_get_mode());

		err = stick_wait_end(&slavedspic.stick_l);
#if 0
		if((err & END_BLOCKING) && (nb_tries < STICK_MODES_NB_TRIES)) {
			nb_tries ++;
			goto retry_left;
		}
#endif
	}	

//	if(err & END_BLOCKING)
//		STMCH_DEBUG("HARVEST TREE mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/**
 * *************** multiple actuators modes ***********
 */

/* do harvest tree mode */
void state_do_harvest_fruits_mode(void)
{
	uint8_t err = 0;

	if (!state_check_update(HARVEST_FRUITS))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.harvest_fruits_mode = mainboard_command.harvest_fruits.mode;

	switch(slavedspic.harvest_fruits_mode)
	{
		case I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_READY:

			tree_tray_set_mode(&slavedspic.tree_tray, TREE_TRAY_MODE_OPEN, 0);
			combs_set_mode(&slavedspic.combs, COMBS_MODE_OPEN, 0);
			
			err = combs_wait_end(&slavedspic.combs);
			if(err & END_BLOCKING)
				break;

			err = tree_tray_wait_end (&slavedspic.tree_tray);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_DO:

			/* new speed */
			combs_set_mode(&slavedspic.combs, COMBS_MODE_HARVEST_CLOSE, 0);

			ax12_user_write_int(&gen.ax12, AX12_ID_TREE_TRAY, AA_MOVING_SPEED_L, 300);
			tree_tray_set_mode(&slavedspic.tree_tray, TREE_TRAY_MODE_HARVEST, 0);

			err = tree_tray_wait_end (&slavedspic.tree_tray);
			ax12_user_write_int(&gen.ax12, AX12_ID_TREE_TRAY, AA_MOVING_SPEED_L, 0x3FF);
			if(err & END_BLOCKING)
				break;

			err = combs_wait_end(&slavedspic.combs);
			if(err & END_BLOCKING)
				break;

			break;

		case I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS_END:

			tree_tray_set_mode(&slavedspic.tree_tray, TREE_TRAY_MODE_OPEN, -126);
			combs_set_mode(&slavedspic.combs, COMBS_MODE_HIDE, 0);

			err = tree_tray_wait_end(&slavedspic.tree_tray);

			tree_tray_set_mode(&slavedspic.tree_tray, TREE_TRAY_MODE_CLOSE, 0);

			err = tree_tray_wait_end(&slavedspic.tree_tray);
			if(err & END_BLOCKING)
				break;

			err = combs_wait_end(&slavedspic.combs);
			if(err & END_BLOCKING)
				break;

			break;

		default:
			break;
	}

	if(err & END_BLOCKING)
		STMCH_DEBUG("HARVEST TREE mode ends with BLOCKING");

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


/* do dump fruits mode */
void state_do_dump_fruits_mode(void)
{
	if (!state_check_update(DUMP_FRUITS))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.dump_fruits_mode = mainboard_command.dump_fruits.mode;

	switch(slavedspic.dump_fruits_mode)
	{
		case I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_DO:
			boot_door_set_mode (&slavedspic.boot, BOOT_DOOR_MODE_OPEN);
			boot_tray_set_mode (&slavedspic.boot, BOOT_TRAY_MODE_VIBRATE);
			break;

		case I2C_SLAVEDSPIC_MODE_DUMP_FRUITS_END:
			boot_door_set_mode (&slavedspic.boot, BOOT_DOOR_MODE_CLOSE);
			boot_tray_set_mode (&slavedspic.boot, BOOT_TRAY_MODE_DOWN);
			break;

		default:
			break;
	}

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}


uint16_t get_shoulder_h [I2C_SLAVEDSPIC_LEVEL_MAX] = {
	[I2C_SLAVEDSPIC_LEVEL_FIRE_GROUND] = 30,
	[I2C_SLAVEDSPIC_LEVEL_FIRE_HEART] = 60,
	[I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_DOWN] = 66,
	[I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_MIDDLE] = 96,
	[I2C_SLAVEDSPIC_LEVEL_FIRE_TORCH_TOP] = 126,
	[I2C_SLAVEDSPIC_LEVEL_MOBILE_TORCH] = 36
};

int16_t get_elbow_a [I2C_SLAVEDSPIC_SUCKER_TYPE_MAX] = {
	[I2C_SLAVEDSPIC_SUCKER_TYPE_SHORT] = 0,
	[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG] = 180
};

uint8_t get_vacuum_system [2] = {
	[I2C_SLAVEDSPIC_SUCKER_TYPE_SHORT] = 1,
	[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG] = 2
};

uint8_t get_sucker_sensor [2] = {
	[I2C_SLAVEDSPIC_SUCKER_TYPE_SHORT] = 3,
	[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG] = 2
};

/* do arm mode */
void state_do_arm_mode(void)
{
	if (!state_check_update(ARM))
		return;

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);
	slavedspic.arm_mode = mainboard_command.arm.mode;
	slavedspic.arm_x =  ((uint16_t)mainboard_command.arm.x_msb << 8) & 0xFF00;
	slavedspic.arm_x |= ((uint16_t)mainboard_command.arm.x_msb) & 0x00FF;
	slavedspic.arm_level = mainboard_command.arm.level;
	slavedspic.arm_sucker_type = mainboard_command.arm.sucker_type;
	slavedspic.arm_sucker_angle = mainboard_command.arm.sucker_angle;


	switch(slavedspic.arm_mode)
	{
		case I2C_SLAVEDSPIC_MODE_ARM_PICKUP_TORCH_READY:
			
			/* goto position and turn on the vaccum */
			slavedspic.arm_h = 95;
			slavedspic.arm_x = 5;
			slavedspic.arm_elbow_a = get_elbow_a[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG];
			slavedspic.arm_wrist_a = 0;

			arm_goto_hxaa (slavedspic.arm_h, slavedspic.arm_x, 
						   slavedspic.arm_elbow_a, slavedspic.arm_wrist_a);
			vacuum_system_enable (get_vacuum_system[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG]);
			break;

		case I2C_SLAVEDSPIC_MODE_ARM_PICKUP_TORCH_DO:

			/* go up a bit */
			slavedspic.arm_h = 125;
			slavedspic.arm_x = 0;
			slavedspic.arm_elbow_a = get_elbow_a[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG];
			slavedspic.arm_wrist_a = 0;

			arm_goto_hxaa (slavedspic.arm_h, slavedspic.arm_x, 
						   slavedspic.arm_elbow_a, slavedspic.arm_wrist_a);
			vacuum_system_enable (get_vacuum_system[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG]);
			break;

		case I2C_SLAVEDSPIC_MODE_ARM_STORE_TORCH:

			/* goto above storage */
			ax12_user_write_int(&gen.ax12, AX12_ID_SHOULDER, AA_MOVING_SPEED_L, 125);

			slavedspic.arm_h = 225;
			slavedspic.arm_x = -75;
			slavedspic.arm_elbow_a = get_elbow_a[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG];
			slavedspic.arm_wrist_a = -30;

			arm_goto_hxaa (slavedspic.arm_h, slavedspic.arm_x, 
						   slavedspic.arm_elbow_a, slavedspic.arm_wrist_a);
			vacuum_system_enable (get_vacuum_system[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG]);

			ax12_user_write_int(&gen.ax12, AX12_ID_SHOULDER, AA_MOVING_SPEED_L, 0x3ff);



			/* update statistis */
			if (sensor_get (get_sucker_sensor[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG])) {
				slavedspic.nb_stored_fires ++;
				STMCH_DEBUG("%d fires stored", slavedspic.nb_stored_fires);
			}			
			else {
				/* find lost fire */
				STMCH_DEBUG("finding fire 1");
				arm_wrist_goto_a_rel (15);
				arm_wrist_wait_traj_end (END_TRAJ|END_TIME);
				time_wait_ms(200);

				if (sensor_get (get_sucker_sensor[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG])) {
					slavedspic.nb_stored_fires ++;
					STMCH_DEBUG("%d fires stored", slavedspic.nb_stored_fires);
				}
				else {
					STMCH_DEBUG("finding fire 2");
					arm_wrist_goto_a_rel (15);
					arm_wrist_wait_traj_end (END_TRAJ|END_TIME);
					time_wait_ms(200);
					
					if (sensor_get (get_sucker_sensor[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG])) {
						slavedspic.nb_stored_fires ++;
						STMCH_DEBUG("%d fires stored", slavedspic.nb_stored_fires);
					}
					else 
						STMCH_DEBUG("lost fire");
				}
			}

			/* goto inside storage */
			slavedspic.arm_h = 200;
			slavedspic.arm_x = -75;
			slavedspic.arm_elbow_a = get_elbow_a[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG];
			slavedspic.arm_wrist_a = -30;

			arm_goto_hxaa (slavedspic.arm_h, slavedspic.arm_x, 
						   slavedspic.arm_elbow_a, slavedspic.arm_wrist_a);

			/* turn off vaccum */
			vacuum_system_disable (get_vacuum_system[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG]);
			

			/* goto avove storage */
			slavedspic.arm_h = 225;
			slavedspic.arm_x = -75;
			slavedspic.arm_elbow_a = get_elbow_a[I2C_SLAVEDSPIC_SUCKER_TYPE_LONG];
			slavedspic.arm_wrist_a = -30;

			arm_goto_hxaa (slavedspic.arm_h, slavedspic.arm_x, 
						   slavedspic.arm_elbow_a, slavedspic.arm_wrist_a);
			break;

		case I2C_SLAVEDSPIC_MODE_ARM_PICKUP_FIRE_READY:

			break;


		case I2C_SLAVEDSPIC_MODE_ARM_PICKUP_FIRE_DO:

			break;

		case I2C_SLAVEDSPIC_MODE_ARM_STORE_FIRE:

			break;


		case I2C_SLAVEDSPIC_MODE_ARM_LOAD_FIRE:

			break;

		case I2C_SLAVEDSPIC_MODE_ARM_FLIP_FIRE:

			break;

		case I2C_SLAVEDSPIC_MODE_ARM_PUTDOWN_FIRE:

			break;

		default:
			break;
	}

	/* notice status and update mode*/
	state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}



#if 0

/* set infos */
void state_do_set_infos(void)
{
	if (!state_check_update(SET_INFOS))
		return;

	state_set_status(I2C_SLAVEDSPIC_STATUS_BUSY);

	STMCH_DEBUG("%s mode=%d", __FUNCTION__, state_get_mode());

	if(mainboard_command.set_infos.nb_goldbars_in_boot >= 0)
		slavedspic.nb_goldbars_in_boot = mainboard_command.set_infos.nb_goldbars_in_boot;
	if(mainboard_command.set_infos.nb_goldbars_in_mouth >= 0)
		slavedspic.nb_goldbars_in_mouth = mainboard_command.set_infos.nb_goldbars_in_mouth;
	if(mainboard_command.set_infos.nb_coins_in_boot >= 0)
		slavedspic.nb_coins_in_boot = mainboard_command.set_infos.nb_coins_in_boot;
	if(mainboard_command.set_infos.nb_coins_in_mouth >= 0)
		slavedspic.nb_coins_in_mouth = mainboard_command.set_infos.nb_coins_in_mouth;

	STMCH_DEBUG("nb_goldbars_in_boot = %d", slavedspic.nb_goldbars_in_boot);
	STMCH_DEBUG("nb_goldbars_in_mouth = %d", slavedspic.nb_goldbars_in_mouth);
	STMCH_DEBUG("nb_coins_in_boot = %d", slavedspic.nb_coins_in_boot);
	STMCH_DEBUG("nb_coins_in_mouth = %d", slavedspic.nb_coins_in_mouth);

   state_set_status(I2C_SLAVEDSPIC_STATUS_READY);
}

#endif

/* state machines */
void state_machines(void)
{
	state_do_init();

	/* simple actuator modes */
	state_do_boot_tray_mode();
	state_do_boot_door_mode();
	state_do_stick_mode();
	state_do_combs_mode();
	state_do_tree_tray_mode();

	/* multiple actuators modes */
	state_do_harvest_fruits_mode();
	state_do_dump_fruits_mode();
	state_do_arm_mode();


#if 0
	state_do_set_infos();
#endif
}

void state_init(void)
{
	mainboard_command.mode = INIT;

	/* enable pwm servos */
	pwm_servo_enable();

	/* enable ax12 torque */
	ax12_user_write_byte(&gen.ax12, AX12_BROADCAST_ID, AA_TORQUE_ENABLE, 0xFF);

	/* start positions */

	/* close gadgets */
	boot_door_set_mode (&slavedspic.boot, BOOT_DOOR_MODE_CLOSE);

	combs_set_mode(&slavedspic.combs, COMBS_MODE_HIDE, 0);
	combs_wait_end(&slavedspic.combs);

	tree_tray_set_mode(&slavedspic.tree_tray, TREE_TRAY_MODE_CLOSE, 0);
	tree_tray_wait_end(&slavedspic.tree_tray);

	stick_set_mode(&slavedspic.stick_l, STICK_MODE_HIDE, 0);
	stick_wait_end(&slavedspic.stick_l);

	stick_set_mode(&slavedspic.stick_r, STICK_MODE_HIDE, 0);
	stick_wait_end(&slavedspic.stick_r);

#if 0
	BRAKE_OFF();
	slavedspic.lift.on = 1;

	/* lift calibration */
	lift_calibrate();
	lift_set_height(LIFT_HEIGHT_MIN_mm);
#endif
}

