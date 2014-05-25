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
      	

/*Elements handler file*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>


#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"
#include "i2c_protocol.h"
#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "../maindspic/strat_avoid.h"
#include "../maindspic/strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"
#include "cmdline.h"


#define MIN_D_TO_POINT 			(ROBOT_CENTER_TO_FRONT+10)

#define ERROUT(e) do {\
		err = e;			 \
		goto end;		 \
	} while(0)


/* Add here play tasks like "harvest fruits", 
 * "harvest ground fire", "harvest wall fire", "dump fruits on basket", etc. */

#ifdef notyet /* TODO 2014*/

/*it's neccessary to start from a point in the y_line of the goldbar */
uint8_t strat_empty_totem_side(int16_t x, int16_t y, uint8_t store_goldbar, uint8_t step)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   #define D_TOTEM_TO_FIRST_LINE 460 //450
   #define D_TO_COINS_TOTEM 35
   #define D_GOLDBAR 10

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
   strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);
   
   /*Turn until we face the totem*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);


   switch(step)
   {
      case 0:
      case 1:
      default:
         goto coins_isle;
         break;

      case 2:
         d=distance_from_robot(x,y);
      	trajectory_d_rel(&mainboard.traj, d-D_TOTEM_TO_FIRST_LINE);
      	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
      	if (!TRAJ_SUCCESS(err))
      			ERROUT(err);
         goto goldbar;
         break;

      case 3:
      	DEBUG(E_USER_STRAT, "Prepare for coins totem and go forward.");
         i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_COINS_TOTEM);
         d=distance_from_robot(x,y);
      	trajectory_d_rel(&mainboard.traj, d-TOTEM_WIDE/2-ROBOT_CENTER_TO_FRONT-D_TO_COINS_TOTEM);
      	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
      	if (!TRAJ_SUCCESS(err))
      			ERROUT(err);
         i2c_slavedspic_wait_ready();

         goto  coins_totem;
         break;
   }

coins_isle:

   /*Open fingers*/
	DEBUG(E_USER_STRAT, "Open fingers floor and fingers totem.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_TOTEM);
   i2c_slavedspic_wait_ready();

   /*Go forward until first line of coins*/
   d=distance_from_robot(x,y);
	trajectory_d_rel(&mainboard.traj, d-D_TOTEM_TO_FIRST_LINE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   

   /*Close fingers floor*/
	DEBUG(E_USER_STRAT, "Pickup first line of coins.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_ISLE);
   i2c_slavedspic_wait_ready();

   if(slavedspic.fingers_floor_blocked)
   { 
     	DEBUG(E_USER_STRAT, "Fingers floor blocked.");
   	trajectory_d_rel(&mainboard.traj, -20);
   	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if (!TRAJ_SUCCESS(err))
   			ERROUT(err);
      i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_TOTEM);
      i2c_slavedspic_wait_ready();
      i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_ISLE);
      i2c_slavedspic_wait_ready();
   }

   if(step)
      goto end;
goldbar:
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_TOTEM);
   i2c_slavedspic_wait_ready();

   /*Go forward until blocking with totem*/
	DEBUG(E_USER_STRAT, "Going forward until blocking with totem");
	strat_set_speed(1000,SPEED_ANGLE_VERY_SLOW);
	trajectory_d_rel(&mainboard.traj,600);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_BLOCKING(err))
			ERROUT(err);
	strat_set_speed(old_spdd, old_spdd);


	DEBUG(E_USER_STRAT, "Pickup goldbar.");
	trajectory_d_rel(&mainboard.traj, -D_GOLDBAR);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   DEBUG(E_USER_STRAT, "I'm going to take the goldbar");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_GOLDBAR_TOTEM);
   i2c_slavedspic_wait_ready();
   DEBUG(E_USER_STRAT, "Done! goldbar taken");

   /*Go a little backward*/
	trajectory_d_rel(&mainboard.traj, -100);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   i2c_slavedspic_mode_store(1,I2C_STORE_MODE_GOLDBAR_IN_BOOT);
   time_wait_ms(500);


   if(step)
      goto end;

coins_totem:
   if(!step){
   	DEBUG(E_USER_STRAT, "Prepare for coins totem and go forward.");
      i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_COINS_TOTEM);
   	trajectory_d_rel(&mainboard.traj, 100+D_GOLDBAR-D_TO_COINS_TOTEM);
   	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if (!TRAJ_SUCCESS(err))
   			ERROUT(err);
      //i2c_slavedspic_wait_ready();
   }

	DEBUG(E_USER_STRAT, "Close fingers and catch coins totem.");
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_HOLD,30);
   i2c_slavedspic_wait_ready();

   if(slavedspic.fingers_totem_blocked)
   { 
     	DEBUG(E_USER_STRAT, "Fingers totem blocked.");
   	trajectory_d_rel(&mainboard.traj, -20);
   	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if (!TRAJ_SUCCESS(err))
   			ERROUT(err);
      i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_HOLD,30);
      i2c_slavedspic_wait_ready();
   }

	strat_infos.treasure_in_mouth =1;
	strat_infos.treasure_in_boot =1;

	/* go backward to safe position*/
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);

end:
	strat_set_speed(old_spdd, old_spda);	
   strat_limit_speed_enable();
   return err;
}


uint8_t strat_pickup_coins_floor(int16_t x, int16_t y, uint8_t group)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   #define D_TO_COIN 300
   #define D_TO_COIN_GROUP 400

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

     
   /*Turn until we face the coin*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
   /*Go forward to near the coin*/
   if(!group) d=distance_from_robot(x,y)-D_TO_COIN;
   else       d=distance_from_robot(x,y)-D_TO_COIN_GROUP;

	trajectory_d_rel(&mainboard.traj, d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Return if opponent is in front*/
	if(robots_infront())  {
		ERROUT(END_OBSTACLE); }

	DEBUG(E_USER_STRAT, "Open fingers floor.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_COINS_FLOOR);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_OPEN,-50);
   i2c_slavedspic_wait_ready();

   /*XXX If we are taking the coin near to the ship, there is no space and we turn*/
   /*if(x==RED_FLOOR_COIN_3_X || x==PURPLE_FLOOR_COIN_3_X) {
   	trajectory_a_abs(&mainboard.traj, -90);
	   err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if (!TRAJ_SUCCESS(err))
   			ERROUT(err);
   }*/

   /*Go forward until we have the coins inside the robot*/
   if(!group){
   	d=distance_from_robot(x,y)-ROBOT_CENTER_TO_FRONT;
   	trajectory_d_rel(&mainboard.traj, d);
	   err = WAIT_COND_OR_TRAJ_END((distance_from_robot(x,y)-ROBOT_CENTER_TO_FRONT)<80, TRAJ_FLAGS_SMALL_DIST);
      /*returns 0 if cond, and err if traj ended before */
   	if (err!=0)
   			if(!TRAJ_SUCCESS(err)) ERROUT(err);
   }   

   else{
   	d=distance_from_robot(x,y)-ROBOT_CENTER_TO_FRONT+120;
   	trajectory_d_rel(&mainboard.traj, d);
	   err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   	if(!TRAJ_SUCCESS(err)) 
            ERROUT(err);
      i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_OPEN,40);
      i2c_slavedspic_wait_ready();
   }

	DEBUG(E_USER_STRAT, "Close fingers floor.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_COINS_FLOOR);
   i2c_slavedspic_wait_ready();

	strat_infos.treasure_in_mouth=1;

	/* go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);
	
end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


/*it's neccessary to start from a point in the y_line of the goldbar */
uint8_t strat_pickup_goldbar_floor(int16_t x, int16_t y, uint8_t store)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   #define D_TO_GOLDBAR 300
   #define D_PICKUP_GOLDBAR 180

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);   

   /*Turn until we face the goldbar*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*Go forward to near the goldbar*/
   d=distance_from_robot(x,y);
	trajectory_d_rel(&mainboard.traj, d-D_TO_GOLDBAR);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Open fingers.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_PREPARE_GOLDBAR_FLOOR);
   i2c_slavedspic_wait_ready();


   /*Go forward until we have the gold bar inside the robot*/
	trajectory_d_rel(&mainboard.traj, D_PICKUP_GOLDBAR);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
   if (!TRAJ_SUCCESS(err))
	   ERROUT(err);
	
	DEBUG(E_USER_STRAT, "Switch on turbine and take goldbar.");
   i2c_slavedspic_mode_harvest(I2C_HARVEST_MODE_GOLDBAR_FLOOR);
   i2c_slavedspic_wait_ready();
	
	DEBUG(E_USER_STRAT, "Store goldbar.");
   i2c_slavedspic_mode_store(1,I2C_STORE_MODE_GOLDBAR_IN_BOOT);
   i2c_slavedspic_wait_ready();
	
	strat_infos.treasure_in_boot =1;

	/*go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_OBSTACLE);

end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_send_message_bottle(int16_t x, int16_t y)
{
   uint8_t err;
	//uint8_t bottle_is_at_right;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);


#if 0
   /*Turn until position in front of the bottle (depending on which point we use to to send message
   we turn to abs 180 or 0 deg*/
   bottle_is_at_right = (position_get_x_s16(&mainboard.pos) < x);
   bottle_is_at_right ? trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(180)) : trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   
   /*go to the bottle*/
	trajectory_d_rel(&mainboard.traj, abs(x-position_get_x_s16(&mainboard.pos)));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
#endif

   /*Turn until we have bottle behind*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(-90));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();

   /*change to slow speed*/
   strat_set_speed(1000, 1000);

	/* go to blocking*/
	trajectory_d_rel(&mainboard.traj, -500);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (!TRAJ_BLOCKING(err))
			ERROUT(err);
   
	err = END_TRAJ;

	/* go forward to safe distance from token */
	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
	trajectory_d_rel(&mainboard.traj, 200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);

end:
	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
   return err;
}


uint8_t strat_save_treasure_generic(int16_t x, int16_t y)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   /* Minimum distance to point to be able to open fingers*/
   #define MIN_D_TO_SAFE_FINGER 	400
	//#define MIN_D_TO_POINT 			ROBOT_CENTER_TO_FRONT

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda); 
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

   /*Turn to point*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

   /*go to the point*/
   d= distance_from_robot(x,y);
   if(d < MIN_D_TO_POINT)
		trajectory_d_rel(&mainboard.traj, -(MIN_D_TO_POINT-d));
	else
		trajectory_d_rel(&mainboard.traj, d-MIN_D_TO_POINT);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
  

	DEBUG(E_USER_STRAT, "Open floor fingers.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_MOUTH);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_OPEN,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_OPEN,0);
   i2c_slavedspic_wait_ready();

   
	/* go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -MIN_D_TO_POINT);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);

   /*Close fingers*/
	DEBUG(E_USER_STRAT, "Close floor fingers.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_END_MOUTH);
   i2c_slavedspic_wait_ready();


	strat_infos.treasure_in_mouth=0;

	#if notyet
   d=distance_from_robot(COLOR_X(strat_infos.area_bbox.x1),position_get_y_s16(&mainboard.pos))+5;
	trajectory_d_rel(&mainboard.traj, -d);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	#endif

end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}

uint8_t strat_save_treasure_arms(int16_t x, int16_t y, uint8_t arm_side)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   /* Minimum distance to point to be able to open fingers*/
   //#define MIN_D_TO_SAFE_FINGER 400

	#define D_PUSH						(ROBOT_CENTER_TO_FRONT+75)

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

 

   /*Turn to point*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

#if 0
	DEBUG(E_USER_STRAT, "Show arm");
   i2c_slavedspic_mode_arm(arm_side, I2C_ARM_MODE_PUSH_FLOOR, 0);
   i2c_slavedspic_wait_ready();
#endif	   

	DEBUG(E_USER_STRAT, "Open floor fingers.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_MOUTH);
   i2c_slavedspic_wait_ready();

   /*go to the point*/
	DEBUG(E_USER_STRAT, "Go to the point.");
   d= distance_from_robot(x,y);
   if(d < MIN_D_TO_POINT)
		trajectory_d_rel(&mainboard.traj, -(MIN_D_TO_POINT-d));
	else
		trajectory_d_rel(&mainboard.traj, d-MIN_D_TO_POINT);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);

  // i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_OPEN,0);
  // i2c_slavedspic_wait_ready();
  //i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_OPEN,0);
  //i2c_slavedspic_wait_ready();

	/* XXX */
	if(time_get_s() > LAST_SECONDS_TIME-1
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_1].flags & ZONE_CHECKED)
		&& (strat_infos.zones[ZONE_OUR_BOTTLE_2].flags & ZONE_CHECKED)) {
		err = END_TIMER;
		goto end;
	}
   
	/* go backward to safe distance from token */
	strat_limit_speed_enable();
	trajectory_d_rel(&mainboard.traj, -(d-MIN_D_TO_POINT));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(END_TRAJ);

   /*Close fingers*/
	DEBUG(E_USER_STRAT, "Close floor fingers.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_END_MOUTH);
   i2c_slavedspic_wait_ready();

	//trajectory_d_rel(&mainboard.traj, D_PUSH);
	//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST|END_OBSTACLE);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);

#if 0
	if(arm_side == I2C_ARM_TYPE_RIGHT)
		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180+30));
	else
		trajectory_a_abs(&mainboard.traj, COLOR_A_ABS(180-30));
	
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
#endif

	//trajectory_d_rel(&mainboard.traj, -D_PUSH);
	//err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST|END_OBSTACLE);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);

	strat_infos.treasure_in_mouth=0;


end:
	#if 0
	DEBUG(E_USER_STRAT, "Hide arm");
   i2c_slavedspic_mode_arm(arm_side, I2C_ARM_MODE_HIDE, 0);
   i2c_slavedspic_wait_ready();
	#endif

	strat_set_speed(old_spdd, old_spda);
   return err;
}

/* Start position must be in line with the ship and the robot looking backwards*/
uint8_t strat_save_treasure_in_deck_back(int16_t x, int16_t y)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;
   #define D_SAVE 100

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
 
   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

   /*go to the ship*/
   d=distance_from_robot(x,y)+D_SAVE;
	trajectory_d_rel(&mainboard.traj, -d);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Prepare for dump.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_BOOT);
   i2c_slavedspic_wait_ready();

	DEBUG(E_USER_STRAT, "Dump.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_BOOT);


   /*Turn a little to each side to let tokens fall down*/
   trajectory_a_rel(&mainboard.traj,40);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   trajectory_a_rel(&mainboard.traj,-80);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
   trajectory_a_rel(&mainboard.traj,40);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);
	time_wait_ms(1000);

	/* go to safe distance*/
	strat_limit_speed_enable();
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);

   i2c_slavedspic_wait_ready();

   /*End of mode empty*/
	DEBUG(E_USER_STRAT, "End of mode empty.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_END_BOOT);
   i2c_slavedspic_wait_ready();


	strat_infos.treasure_in_boot =0;

end:
	strat_set_speed(old_spdd, old_spda);
   return err;
}

/* Start position must be in line with the ship and the robot looking backwards*/
uint8_t strat_save_treasure_in_hold_back(int16_t x, int16_t y)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
 
   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

   /*Spread hook out*/
	DEBUG(E_USER_STRAT, "Spread hook out.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_HOLD);
   i2c_slavedspic_wait_ready();

   /* it needs more time to get hook down*/
	time_wait_ms(1000);

   /*Deactivate cs angle*/
   /*TODO*/

   /*Go backward until blocking*/
   strat_set_speed(SPEED_DIST_SLOW, SPEED_ANGLE_SLOW);
	trajectory_d_rel(&mainboard.traj, -500);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (!TRAJ_BLOCKING(err))
			ERROUT(err);

   /*Activate cs angle*/
   /*TODO*/

   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(15);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(90,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(16);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(120,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_blow(125);
   i2c_slavedspic_wait_ready();

   /*Slave in mode empty*/
	DEBUG(E_USER_STRAT, "Mode empty.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_BOOT);
	time_wait_ms(3000);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_blow(0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(90,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(15);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(0,250);
   i2c_slavedspic_wait_ready();

	/* go to safe distance*/
	strat_limit_speed_enable();
	trajectory_d_rel(&mainboard.traj, PLACE_D_SAFE);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(END_TRAJ);


   /*Put down hook*/
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_END_BOOT);
   i2c_slavedspic_wait_ready();


	strat_infos.treasure_in_boot =0;

end:
   /*Put down hook*/
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_END_BOOT);
   i2c_slavedspic_wait_ready();

	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
   return err;
}

/* Start position must be in line with the ship and the robot looking backwards*/
uint8_t strat_save_treasure_in_deck_back_blowing(int16_t x, int16_t y)
{   
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
 
   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);

   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_BOOT);
	i2c_slavedspic_wait_ready();

   trajectory_d_rel(&mainboard.traj,-200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

#if 0
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_OPEN,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(15);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(90,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(16);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(120,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_blow(90);
   i2c_slavedspic_wait_ready();
#endif

   /*Slave in mode empty*/
	DEBUG(E_USER_STRAT, "Mode empty.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_BOOT);
   i2c_slavedspic_wait_ready();

#if 0
 	time_wait_ms(3000);
  	i2c_slavedspic_mode_turbine_blow(0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(90,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_lift_height(15);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_turbine_angle(0,250);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();
#endif


	strat_infos.treasure_in_boot =0;

	/* go to safe distance*/
	strat_limit_speed_enable();
	trajectory_d_rel(&mainboard.traj, 200);
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);



end:
#if 0
   i2c_slavedspic_mode_turbine_blow(0);
   i2c_slavedspic_wait_ready();
#endif

	/* close boot */
	i2c_slavedspic_mode_boot(I2C_BOOT_MODE_CLOSE);
	i2c_slavedspic_wait_ready();

	strat_set_speed(old_spdd, old_spda);
	strat_limit_speed_enable();
   return err;
}

uint8_t strat_raise_window(uint8_t window)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);

   /*Turn to abs 180 or 0 deg*/
   trajectory_a_abs(&mainboard.traj,COLOR_A_ABS(0));
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_FAST,SPEED_ANGLE_FAST);
 
   /*Spread hook out*/
	DEBUG(E_USER_STRAT, "Spread hook out.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_PREPARE_HOLD);
   i2c_slavedspic_wait_ready();

   /*Deactivate cs angle*/
   /*TODO*/

   /*Go backward until blocking*/
	trajectory_d_rel(&mainboard.traj, -300);
	err = wait_traj_end(END_INTR|END_TRAJ|END_BLOCKING);
	if (!TRAJ_BLOCKING(err))
			ERROUT(err);

   /*Activate cs angle*/
   /*TODO*/

   /*Slave in mode empty*/
	DEBUG(E_USER_STRAT, "Mode empty.");
   i2c_slavedspic_mode_dump(I2C_DUMP_MODE_BOOT);
   i2c_slavedspic_wait_ready();

end:
	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
   return err;
}

uint8_t strat_steal_treasure_hold(void)
{
   return 1;
}



uint8_t strat_stole_opp_treasure(int16_t x, int16_t y)
{
   uint8_t err;
	uint16_t old_spdd, old_spda;
   int16_t d;

	#define D_PUSH						(ROBOT_CENTER_TO_FRONT+75)

	/* save speed */
	strat_get_speed(&old_spdd, &old_spda);
	strat_limit_speed_disable();
	strat_set_speed(SPEED_DIST_VERY_SLOW,SPEED_ANGLE_VERY_SLOW);

   /*Turn to point*/
   trajectory_turnto_xy(&mainboard.traj,x,y);
	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	if (!TRAJ_SUCCESS(err))
			ERROUT(err);

	DEBUG(E_USER_STRAT, "Open floor fingers.");
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_OPEN,0);
   i2c_slavedspic_wait_ready();
  	i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_OPEN,0);
  	i2c_slavedspic_wait_ready();

   /*go to the point*/
	DEBUG(E_USER_STRAT, "Go to the point.");
   d= distance_from_robot(x,y);
   if(d < MIN_D_TO_POINT)
		trajectory_d_rel(&mainboard.traj, -(MIN_D_TO_POINT-d));
	else
		trajectory_d_rel(&mainboard.traj, d-MIN_D_TO_POINT);

	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
	//if (!TRAJ_SUCCESS(err))
	//		ERROUT(err);
  
  /*Close fingers*/
	DEBUG(E_USER_STRAT, "Close floor fingers.");
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_FLOOR, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();
   i2c_slavedspic_mode_fingers(I2C_FINGERS_TYPE_TOTEM, I2C_FINGERS_MODE_HOLD,0);
   i2c_slavedspic_wait_ready();

	/* go backward to safe distance from token */
	trajectory_d_rel(&mainboard.traj, -(d-MIN_D_TO_POINT));
	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
	if (!TRAJ_SUCCESS(err))
		ERROUT(END_TRAJ);

	strat_infos.treasure_in_mouth=1;

end:
	strat_limit_speed_enable();
	strat_set_speed(old_spdd, old_spda);
   return err;
}







/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////


#if 0
uint8_t strat_game(void)
{
   #define D_INI 295
   uint8_t err=0;
   static uint8_t _case;

   while(1)
   {
      switch(_case)
      {
         case 0:  //init
            DEBUG(E_USER_STRAT, "Game started!");
            i2c_slavedspic_mode_init();
            i2c_slavedspic_wait_ready();
            trajectory_d_rel(&mainboard.traj, D_INI);
         	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=1;
            break;
      
         case 1:  //coins group
            DEBUG(E_USER_STRAT, "Going to group of coins");
            err=goto_and_avoid(strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].init_x, strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].init_y,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_pickup_coins_floor(strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].x,strat_infos.zones[ZONE_MIDDLE_COINS_GROUP].y, GROUP);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=2;
            break;
      
         case 2:  //leave coins
            err=strat_save_treasure_generic(2362,1650);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=13;
            break;
      
         case 13: //Going to middle floor goldbar
            DEBUG(E_USER_STRAT, "Going to middle floor goldbar");
            err=goto_and_avoid(strat_infos.zones[ZONE_MIDDLE_FLOOR_GOLDBAR].init_x, strat_infos.zones[ZONE_MIDDLE_FLOOR_GOLDBAR].init_y,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_pickup_goldbar_floor(strat_infos.zones[ZONE_MIDDLE_FLOOR_GOLDBAR].x,strat_infos.zones[ZONE_MIDDLE_FLOOR_GOLDBAR].init_y, STORE_BOOT);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=3;
            break;

         case 3:  //totem 2 side 2
            DEBUG(E_USER_STRAT, "Going to totem 2 side 2");
            err=goto_and_avoid(strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_x, strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].init_y,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_empty_totem_side(strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].x,strat_infos.zones[ZONE_TOTEM_OPP_SIDE_2].y, STORE_BOOT, 0);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=4;
            break;
      
         case 4:  //coins apart
            DEBUG(E_USER_STRAT, "Going to pick up the group of coins apart");
            err=goto_and_avoid(1900, 1700, TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_pickup_coins_floor(2362,1650,GROUP);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=5;
            break;
      
         case 5:  //save treasure
            DEBUG(E_USER_STRAT, "Going to save treasure in deck");
            err=goto_and_avoid(2360, 800, TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err = strat_save_treasure_generic(2750,800);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=6;
            break;
      
         case 6:  //coin 2
            /*DEBUG(E_USER_STRAT, "Going to coin 2");
            trajectory_goto_xy_abs(&mainboard.traj,strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_2].init_x, strat_infos.zones[ZONE_PURPLE_FLOOR_COIN_2].init_y);
         	err = wait_traj_end(TRAJ_FLAGS_SMALL_DIST);
            if(!TRAJ_SUCCESS(err))
               break;
//            else
            //	if (!TRAJ_SUCCESS(err))
           // 			ERROUT(err);
            err=strat_pickup_coins_floor(PURPLE_FLOOR_COIN_2_X,PURPLE_FLOOR_COIN_2_Y,ONE);
            if(!TRAJ_SUCCESS(err))
               break;
//            else
            //	if (!TRAJ_SUCCESS(err))
           // 			ERROUT(err);*/
            _case=7;
            break;
      
         case 7:  //coin 1
            DEBUG(E_USER_STRAT, "Going to coin 1");
            err=goto_and_avoid(COLOR_X(strat_infos.area_bbox.x1), 250, TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_pickup_coins_floor(OUR_FLOOR_COIN_1_X,OUR_FLOOR_COIN_1_Y,ONE);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=8;
            break;
   
         
         case 8:  //totem 2 side 1
            DEBUG(E_USER_STRAT, "Going to totem 2 side 1");
            err=goto_and_avoid(strat_infos.zones[ZONE_TOTEM_OUR_SIDE_1].init_x, strat_infos.zones[ZONE_TOTEM_OUR_SIDE_1].init_y, 
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_empty_totem_side(strat_infos.zones[ZONE_TOTEM_OUR_SIDE_1].x,strat_infos.zones[ZONE_TOTEM_OUR_SIDE_1].y, STORE_BOOT,0);
            if(!TRAJ_SUCCESS(err))
               break;

            /* XXX restore if not work the game
            trajectory_d_rel(&mainboard.traj, -100);
         	err = wait_traj_end(TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            */
            _case=9;
            break;
   
         
         case 9:  //save treasure
            DEBUG(E_USER_STRAT, "Going to save treasure in deck");
            err=goto_and_avoid(2365, 1055,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err = strat_save_treasure_generic(2750,1055);
            if(!TRAJ_SUCCESS(err))
               break;
         
            _case=14;
            break;

         case 14:
            DEBUG(E_USER_STRAT, "Going to save goldbars in hold");
            err=goto_and_avoid(2360, 1700, TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err = strat_save_treasure_in_hold_back(2750,1700);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=10;
            break;
   
         
         case 10: //message bottle 1
            DEBUG(E_USER_STRAT, "Going to send message bottle 1");
            err=goto_and_avoid(strat_infos.zones[ZONE_PURPLE_BOTTLE_1].init_x, strat_infos.zones[ZONE_PURPLE_BOTTLE_1].init_y,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_send_message_bottle(strat_infos.zones[ZONE_PURPLE_BOTTLE_1].x, strat_infos.zones[ZONE_PURPLE_BOTTLE_1].y);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=11;
            break;
   
         
         case 11: //message bottle 2
            DEBUG(E_USER_STRAT, "Going to send message bottle 2");
            err=goto_and_avoid(strat_infos.zones[ZONE_PURPLE_BOTTLE_2].init_x, strat_infos.zones[ZONE_PURPLE_BOTTLE_2].init_y,
            TRAJ_FLAGS_NO_NEAR,TRAJ_FLAGS_NO_NEAR);
            if(!TRAJ_SUCCESS(err))
               break;
            err=strat_send_message_bottle(strat_infos.zones[ZONE_PURPLE_BOTTLE_2].x, strat_infos.zones[ZONE_PURPLE_BOTTLE_2].y);
            if(!TRAJ_SUCCESS(err))
               break;
            _case=12;
            break;
   
         
         case 12: //end
            DEBUG(E_USER_STRAT, "Mission completed!!");
            goto end;
            break;

         default:
            break;
      }
   }

end:
   return err;
}

#endif
#endif
