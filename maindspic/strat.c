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
 *  Javier Balinas Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>
#include <clock_time.h>

#ifndef HOST_VERSION_OA_TEST
#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>

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
#include "strat_avoid.h"
#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"
#include "beacon.h"

#else

/** 
   XXX 
   strat.c is used for OA test for both robots.
   We need define some robot dimensions here, depending on robot 
 */

#ifndef IM_SECONDARY_ROBOT

/* XXX synchronized with maindspic/main.h */

/* robot dimensions */
#define ROBOT_LENGTH            281.5
#define ROBOT_WIDTH             330.0
#define ROBOT_CENTER_TO_FRONT   162.5
#define ROBOT_CENTER_TO_BACK    119.0
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE            (232.+10.)

#else

/* XXX synchronized with secondary_robot/main.h */

/* robot dimensions */
#define ROBOT_LENGTH      	    150.
#define ROBOT_WIDTH 	    	230.
#define ROBOT_CENTER_TO_FRONT   75.
#define ROBOT_CENTER_TO_BACK    75.
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clearance */
#define OBS_CLERANCE            (137.+10.)

#endif /* ! IM_SECONDARY_ROBOT */


#endif


static volatile uint8_t strat_running = 0;


struct strat_infos strat_infos = { 
    /* conf */
    .conf = {
        .flags = 0,
    },

#define TREE_D_INIT             440
#define BASKET_D_INIT         (440 + 300)


   /*zones[W] =                 {type,            x,            y,            x_down,   x_up,  y_down,    y_up,      init_x,                 init_y,                       prio,             flags,        opp_time_zone_us,	last_time_opp_here,	robot };                            */
    .zones[ZONE_TREE_1]=        {ZONE_TYPE_TREE,  TREE_1_X,     TREE_1_Y,     0,         400,    1100,    1500,      TREE_D_INIT,            TREE_1_Y,                     PRIO_TREE_1,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_TREE_2]=        {ZONE_TYPE_TREE,  TREE_2_X,     TREE_2_Y,     500,       900,    1600,    2000,      TREE_2_X,               AREA_Y-TREE_D_INIT,           PRIO_TREE_2,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_TREE_3]=        {ZONE_TYPE_TREE,  TREE_3_X,     TREE_3_Y,     2100,     2500,    1600,    2000,      TREE_3_X,               AREA_Y-TREE_D_INIT,           PRIO_TREE_3,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_TREE_4]=        {ZONE_TYPE_TREE,  TREE_4_X,     TREE_4_Y,     2600,     3000,    1100,    1500,      AREA_X-TREE_D_INIT,     		TREE_4_Y,                     PRIO_TREE_4,     0,            0,					(9000*1000L),					MAIN_ROBOT},

   /*zones[W] =                 {type,             x,         y,         					x_down,    x_up,   y_down, y_up,  init_x,       init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
    .zones[ZONE_HEART_1]=       {ZONE_TYPE_HEART,  HEART_1_X, HEART_1_Y, 					0,         500,    1500,   2000   ,400 ,          1800,   PRIO_HEART_1,     0,            0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_HEART_2_UP]=       {ZONE_TYPE_HEART,  HEART_2_UP_X, HEART_2_UP_Y, 			1350,      1650,   1050,   1500   ,HEART_2_UP_X,  1600,   PRIO_HEART_2_UP,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_HEART_2_LEFT]=       {ZONE_TYPE_HEART,  HEART_2_LEFT_X, HEART_2_LEFT_Y, 	1050,      1500,   900,    1200   ,950,    HEART_2_LEFT_Y,   PRIO_HEART_2_LEFT,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_HEART_2_DOWN]=       {ZONE_TYPE_HEART,  HEART_2_DOWN_X, HEART_2_DOWN_Y, 	1350,      1650,   600,    1050   ,HEART_2_DOWN_X, 500,   PRIO_HEART_2_DOWN,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_HEART_2_RIGHT]=       {ZONE_TYPE_HEART,  HEART_2_RIGHT_X, HEART_2_RIGHT_Y, 	1500,      1950,   900,    1200   ,2050,   HEART_2_RIGHT_Y,   PRIO_HEART_2_RIGHT,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_HEART_3]=       {ZONE_TYPE_HEART,  HEART_3_X, HEART_3_Y, 					2500,      3000 ,  1500,   2000   ,2600 ,  1800,   PRIO_HEART_3,     0,            0,					(9000*1000L),				MAIN_ROBOT},

   /*zones[W] =                 {type,           x,        y,        x_down,    x_up,   y_down, y_up,   init_x, init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
    .zones[ZONE_FIRE_1]=        {ZONE_TYPE_FIRE, FIRE_1_X, FIRE_1_Y, 100,       700,    800,    1400,   630,    910,    PRIO_FIRE_1,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_FIRE_2]=        {ZONE_TYPE_FIRE, FIRE_2_X, FIRE_2_Y, 600,       1200,   300,    900,    630,    910,    PRIO_FIRE_2,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_FIRE_3]=        {ZONE_TYPE_FIRE, FIRE_3_X, FIRE_3_Y, 600,       1200,   1300,   1900,   1060,   1160,   PRIO_FIRE_3,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_FIRE_4]=        {ZONE_TYPE_FIRE, FIRE_4_X, FIRE_4_Y, 1800,      2400,   300,    900,    2370,   910,    PRIO_FIRE_4,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_FIRE_5]=        {ZONE_TYPE_FIRE, FIRE_5_X, FIRE_5_Y, 1800,      2400,   1300,   1900,   1940,   1160,   PRIO_FIRE_5,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_FIRE_6]=        {ZONE_TYPE_FIRE, FIRE_6_X, FIRE_6_Y, 2300,      2900,   800,    1400,   2370,   910,    PRIO_FIRE_6,     0,            0,					0,					MAIN_ROBOT},  

   /*zones[W] =                 {type,            x,         y,         x_down, x_up,   y_down, y_up,   init_x, init_y, prio,         flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */
    .zones[ZONE_TORCH_1]=       {ZONE_TYPE_TORCH, TORCH_1_X, TORCH_1_Y, 0,      400,    600,    1000,   340,    560,    PRIO_TORCH_1,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_TORCH_2]=       {ZONE_TYPE_TORCH, TORCH_2_X, TORCH_2_Y, 1100,   1500,   1600,   2000,   980,    1710,   PRIO_TORCH_2,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_TORCH_3]=       {ZONE_TYPE_TORCH, TORCH_3_X, TORCH_3_Y, 1500,   1900,   1600,   2000,   2020,   1710,   PRIO_TORCH_3,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_TORCH_4]=       {ZONE_TYPE_TORCH, TORCH_4_X, TORCH_4_Y, 2600,   3000,   600,    1000,   2660,   560,    PRIO_TORCH_4,     0,            0,					0,					MAIN_ROBOT},

   /*zones[W] =                 {type,              x,           y,           x_down,   x_up,   y_down, y_up,   init_x, init_y, prio,        flags,        opp_time_zone_us,	last_time_opp_here,	robot };  */ 
    .zones[ZONE_M_TORCH_1]=     {ZONE_TYPE_M_TORCH, M_TORCH_1_X, M_TORCH_1_Y, 600,      1200,   800,    1400,   630,    910,    PRIO_M_TORCH_1,     0,            0,					0,					MAIN_ROBOT},
    .zones[ZONE_M_TORCH_2]=     {ZONE_TYPE_M_TORCH, M_TORCH_2_X, M_TORCH_2_Y, 1800,     2400,   800,    1400,   2370,   910,    PRIO_M_TORCH_2,     0,            0,					0,					MAIN_ROBOT},
 
    .zones[ZONE_BASKET_1]=      {ZONE_TYPE_BASKET,  BASKET_1_X,  BASKET_1_Y,  400,      1100,   300,     600,    750,    BASKET_D_INIT,   PRIO_BASKET_1,     0,             0,					(9000*1000L),					MAIN_ROBOT},
    .zones[ZONE_BASKET_2]=      {ZONE_TYPE_BASKET,  BASKET_2_X,  BASKET_2_Y,  1900,     2600,   300,     600,    2250,   BASKET_D_INIT,   PRIO_BASKET_2,     0,             0,					(9000*1000L),					MAIN_ROBOT},

    .zones[ZONE_MAMOOTH_1]=     {ZONE_TYPE_MAMOOTH, MAMOOTH_1_X, MAMOOTH_1_Y, 400,  	1100,  300,     600,    750,    600,    PRIO_MAMOOTH_1,     0,            0,					0,					SEC_ROBOT},
    .zones[ZONE_MAMOOTH_2]=     {ZONE_TYPE_MAMOOTH, MAMOOTH_2_X, MAMOOTH_2_Y, 1900, 	2600,  300,     600,    2250,   600,    PRIO_MAMOOTH_2,     0,            0,					0,					SEC_ROBOT},

    .zones[ZONE_FRESCO]=        {ZONE_TYPE_FRESCO,  FRESCO_X,    FRESCO_Y,    1100, 	1900,  	0,   	500,    1500,   300,    PRIO_FRESCO,     0,            0,					0,					SEC_ROBOT},

    //.zones[ZONE_HOME_YELLOW]=        {ZONE_TYPE_HOME, HOME_RED_X, HOME_RED_Y, 0, 400,  0,    690   , 2800,  600,   -90,  ZONE_PRIO_0,     0,            0,					0,					BOTH_ROBOTS},
    //.zones[ZONE_HOME_RED]=     {ZONE_TYPE_HOME, HOME_YELLOW_X, HOME_YELLOW_Y, 2600, 3000,  0,   690   , 200,  600,   -90,  ZONE_PRIO_0,     0,            0,					0,					BOTH_ROBOTS},
};

/* point we get from each zone */
uint8_t strat_zones_points[ZONES_MAX]= {3,3,3,3,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,0,0,6,6,6};

/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(uint8_t type)
{

    strat_infos.area_bbox.x1 = OBS_CLERANCE;
    strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;

    strat_infos.area_bbox.y1 = OBS_CLERANCE + 300;
    strat_infos.area_bbox.y2 = 2000 - OBS_CLERANCE;

    polygon_set_boundingbox(strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);

#ifdef HOST_VERSION_OA_TEST

  printf("boundingbox at: %d %d %d %d\n", 
        strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);
#endif
}

#ifndef HOST_VERSION_OA_TEST

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
    time_reset();
    interrupt_traj_reset();
    mainboard.flags =  DO_ENCODERS | DO_CS | DO_RS |
                             DO_POS | DO_BD | DO_POWER | DO_BEACON | DO_ROBOT_2ND;

    /* XXX default conf */
    //strat_infos.conf.flags |= ENABLE_R2ND_POS;


    strat_dump_conf();
    strat_dump_infos(__FUNCTION__);
}

/* display curret strat configuration */
void strat_dump_conf(void)
{
    if (!strat_infos.dump_enabled)
        return;

    printf_P(PSTR("-- conf --\r\n"));

    //printf(" ENABLE_R2ND_POS is %s\n\r", strat_infos.conf.flags & ENABLE_R2ND_POS? "ON":"OFF");
    //printf(" ENABLE_DOWN_SIDE_ZONES is %s\n\r", strat_infos.conf.flags & ENABLE_DOWN_SIDE_ZONES? "ON":"OFF");

    /* add here configuration dump */
}


char numzone2name[ZONES_MAX + 1][3] = {
[ZONE_TREE_1]="t1",
[ZONE_TREE_2]="t2",
[ZONE_TREE_3]="t3",
[ZONE_TREE_4]="t4",
[ZONE_HEART_1]="h1",
[ZONE_HEART_2_UP]="h2u",
[ZONE_HEART_2_LEFT]="h2l",
[ZONE_HEART_2_DOWN]="h2d",
[ZONE_HEART_2_RIGHT]="h2r",
[ZONE_HEART_3]="h3",
[ZONE_FIRE_1]="f1",
[ZONE_FIRE_2]="f2",
[ZONE_FIRE_3]="f3",
[ZONE_FIRE_4]="f3",
[ZONE_FIRE_5]="f5",
[ZONE_FIRE_6]="f6",
[ZONE_TORCH_1]="tr1",
[ZONE_TORCH_2]="tr2",
[ZONE_TORCH_3]="tr3",
[ZONE_TORCH_4]="tr4",
[ZONE_M_TORCH_1]="mt1",
[ZONE_M_TORCH_2]="mt2",
[ZONE_BASKET_1]="b1",
[ZONE_BASKET_2]="b2",
[ZONE_MAMOOTH_1]="m1",
[ZONE_MAMOOTH_2]="m2",
[ZONE_FRESCO]="fco",
//[ZONE_HOME_RED]="rd",
//[ZONE_HOME_YELLOW]="yll",
[ZONES_MAX] = "nll",
};

/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
	int8_t zone_opp;
    if (!strat_infos.dump_enabled)
        return;

    printf(PSTR("%s() dump strat infos:\r\n"), caller);


    /* add here print infos */
	printf_P("%d %d\n", opponent1_is_infront(),opponent2_is_infront());
	
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
    /* bounding box */
    strat_set_bounding_box(mainboard.our_color);

    strat_infos.current_zone = ZONES_MAX;
    strat_infos.goto_zone = ZONES_MAX;
    strat_infos.last_zone = ZONES_MAX;

    /* add here other infos resets */
}

void strat_event_enable(void)
{
    strat_running = 1;
}

void strat_event_disable(void)
{
    strat_running = 0;
}

/* call it just before launching the strat */
void strat_init(void)
{
    strat_reset_infos();

    /* we consider that the color is correctly set */

    strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
    time_reset();
    interrupt_traj_reset();

    /* init other devices (lasers...) */
    i2c_slavedspic_mode_init();

    /* used in strat_base for END_TIMER */
    mainboard.flags = DO_ENCODERS | DO_CS | DO_RS |
        DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_BEACON | DO_ROBOT_2ND;
}


/* call it after each strat */
void strat_exit(void)
{
    uint8_t flags;

    /* stop robot, disable timer */
    mainboard.flags &= ~(DO_TIMER);
    strat_hardstop();
    time_reset();

    /* disable CS, and motors */
#ifndef HOST_VERSION
    IRQ_LOCK(flags);
    mainboard.flags &= ~(DO_CS);
    IRQ_UNLOCK(flags);
    dac_mc_set(LEFT_MOTOR, 0);
    dac_mc_set(RIGHT_MOTOR, 0);
#endif

    /* stop beacon */
    /* TODO beacon_cmd_beacon_off(); */

    /* slavespic exit TODO 2014 */
    //i2c_slavedspic_mode_turbine_blow(0);
    //i2c_slavedspic_wait_ready();

    /* turn off other devices (lasers...) */

    /* TODO stop beacon
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    beacon_cmd_beacon_off();
    */
}

/* called periodically */
void strat_event(void *dummy)
{
    /* XXX in parallel with main strat,
     *    disable/enable events depends on case or protect with IRQ_LOCK.
     */

    /* ignore when strat is not running */
    if (strat_running == 0)
        return;

    /* limit speed when opponent are close */
    strat_limit_speed();

    /* tracking of zones where opp has been working */
    strat_opp_tracking();

}

/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)        \
    do {                        \
        if (time_get_s() - last_print > 5) {    \
            dump();                \
            last_print = time_get_s();    \
        }                    \
    } while (0)


#define ERROUT(e) do {\
        err = e;             \
        goto end;         \
    } while(0)



/* Strat main loop */
uint8_t strat_main(void)
{
    uint8_t err, i;

    // AVOID ZONES IN CURRENT STRATEGY
    strat_infos.zones[ZONE_TORCH_4].flags |= ZONE_AVOID;
    strat_infos.zones[ZONE_M_TORCH_2].flags |= ZONE_AVOID;
    strat_infos.zones[ZONE_BASKET_1].flags |= ZONE_AVOID;
    strat_infos.zones[ZONE_FIRE_4].flags |= ZONE_AVOID;
    strat_infos.zones[ZONE_HEART_3].flags |= ZONE_AVOID;

	strat_homologation();
	while(1);
	
	/*
    do{
        err = strat_begin();
    }while((err & END_RESERVED) == 0);

    strat_limit_speed_enable ();*/
    strat_limit_speed_enable ();
	
    /* auto-play  */
    printf_P(PSTR("\r\n\r\nStrat smart\r\n"));
    do{
        err = strat_smart();
    }while((err & END_TIMER) == 0);
	

   strat_exit();
   return 0;
   
}



#endif /* HOST_VERSION_OA_TEST */


