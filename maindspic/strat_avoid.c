/*  
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  Revision : $Id: strat_avoid.c,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_avoid.c,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION
#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>

#include "main.h"
#include "strat.h"
#include "strat_base.h"
#include "strat_utils.h"
#include "sensor.h"

#else

#define E_USER_STRAT 200

#define END_TRAJ       1 /* traj successful */
#define END_BLOCKING   2 /* blocking during traj */
#define END_NEAR       4 /* we are near destination */
#define END_OBSTACLE   8 /* There is an obstacle in front of us */
#define END_ERROR     16 /* Cannot do the command */
#define END_INTR      32 /* interrupted by user */
#define END_TIMER     64 /* we don't a lot of time */
#define END_RESERVED 128 /* reserved */

#endif

#ifdef HOMOLOGATION
/* /!\ half size */
#define O_WIDTH  400
#define O_LENGTH 550
#else
/* /!\ half size */
#define O_WIDTH  245
#define O_LENGTH 340
#endif

#define CENTER_X 1500
#define CENTER_Y 1000

/* don't care about polygons further than this distance for escape */
#define ESCAPE_POLY_THRES 1000

/* don't reduce opp if opp is too far */
#define REDUCE_POLY_THRES 600

/* has to be longer than any poly */
#define ESCAPE_VECT_LEN 3000

#ifdef POLYS_IN_PATH
/* related with slot in path */
uint8_t num_slots_in_path;
uint8_t slot_in_path_flag[SLOT_NUMBER];
#endif

#ifdef HOST_VERSION
int16_t g_robot_x;
int16_t g_robot_y;
double  g_robot_a;

int16_t g_opp1_x;
int16_t g_opp1_y;
int16_t g_opp2_x;
int16_t g_opp2_y;
#endif

#ifdef HOST_VERSION
/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}
#endif


/* normalize vector from origin */
#if HOST_VERSION
double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}
#endif

/* rotate point */
#ifdef HOST_VERSION
void rotate(double *x, double *y, double rot)
{
	double l, a;
	
	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}
#endif

/* set rotated poly relative to robot coordinates */
void set_rotated_poly(poly_t *pol, const point_t *robot_pt, 
		      int16_t w, int16_t l, int16_t x, int16_t y)

{
	double tmp_x, tmp_y;
	double a_rad;

	/* calcule relative angle to robot */
	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);

	DEBUG(E_USER_STRAT, "%s() x,y=%d,%d a_rad=%2.2f", 
	      __FUNCTION__, x, y, a_rad);

	/* point 1 */
	tmp_x = w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 0);
	
	/* point 2 */
	tmp_x = -w;
	tmp_y = l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 1);
	
	/* point 3 */
	tmp_x = -w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 2);
	
	/* point 4 */
	tmp_x = w;
	tmp_y = -l;
	rotate(&tmp_x, &tmp_y, a_rad);
	tmp_x += x;
	tmp_y += y;
	oa_poly_set_point(pol, tmp_x, tmp_y, 3);
}

/* set poly that represent the opponent */
void set_opponent_1_poly(poly_t *pol, const point_t *robot_pt, int16_t w, int16_t l)
{
	int16_t x, y;
	
#ifndef HOST_VERSION
	get_opponent_xy(&x, &y);
#else
	x = g_opp1_x;
	y = g_opp1_y;
#endif	

	DEBUG(E_USER_STRAT, "oponent 1 at: %d %d", x, y);
	
	/* place poly even if invalid, because it's -1000 */
	set_rotated_poly(pol, robot_pt, w, l, x, y); 
}

/* set poly that represent the opponent */
void set_opponent_2_poly(poly_t *pol, const point_t *robot_pt, int16_t w, int16_t l)
{
	int16_t x, y;
	
#ifndef HOST_VERSION
   /* TODO: get second oponent xy */
	//get_opponent_xy(&x, &y);
	x = I2C_OPPONENT_NOT_THERE;
	y = I2C_OPPONENT_NOT_THERE;
#else
	x = g_opp2_x;
	y = g_opp2_y;
#endif	

	DEBUG(E_USER_STRAT, "oponent 2 at: %d %d", x, y);
	
	/* place poly even if invalid, because it's -1000 */
	set_rotated_poly(pol, robot_pt, w, l, x, y); 
}


/* set point of a rhombus, used for slot polys */
uint8_t set_rhombus_pts(point_t *pt,
               		int16_t w, int16_t l,
			      		int16_t x, int16_t y)
{
#ifdef SET_RHOMBUS_PTS_OLD_VERSION
	uint8_t i, j;

	/* loop for rhrombus points */
	for(i=0, j=0; i<4; i++) {
	
		/* add point of rhombus */
		if(i==0) {				
			pt[j].x = x + w;
			pt[j].y = y;
		}
		else if(i==1) {				
			pt[j].x = x;
			pt[j].y = y + l;
		}
		else if(i==2) {				
			pt[j].x = x - w;
			pt[j].y = y;
		}
		else if(i==3) {				
			pt[j].x = x;
			pt[j].y = y - l;
		}
				
		/* next point */
		j++;
	}
		
	/* return number of points */
	return j;	
	
	
#else
	
	/* points of rhombus */
		
	pt[0].x = x + w;
	pt[0].y = y;
			
	pt[1].x = x;
	pt[1].y = y + l;
		
	pt[2].x = x - w;
	pt[2].y = y;
				
	pt[3].x = x;
	pt[3].y = y - l;
	  
	return 4;

#endif	

}

/* set point of a square */
uint8_t set_square_pts(point_t *pt,
               		int16_t w, int16_t l,
			      		int16_t x, int16_t y)
{
	
	/* points of rhombus */
		
	pt[0].x = x + w;
	pt[0].y = y + l;
			
	pt[1].x = x - w;
	pt[1].y = y + l;
		
	pt[2].x = x - w;
	pt[2].y = y - l;
				
	pt[3].x = x + w;
	pt[3].y = y - l;
	  
	return 4;
}

/* set oa poly point */
void set_poly_pts(poly_t *pol_dest, poly_t *pol_org)
{
	uint8_t i;
	
	/* loop for all point */
	for(i=0; i < pol_org->l; i++) {
	   oa_poly_set_point(pol_dest, pol_org->pts[i].x, pol_org->pts[i].y, i);
   }
}

#ifdef POLYS_IN_PATH
/* set slot polys that are in direct path between two point */
void set_slots_poly_in_path(poly_t **pol,
									 int32_t x0, int32_t y0, int32_t x1, int32_t y1,
									 int16_t org_x, int16_t org_y,
									 int16_t dst_x, int16_t dst_y)
{
	uint8_t i, j, k;
	int16_t ret;
	poly_t poly_slot;
	point_t poly_slot_pts[SLOT_EDGE_NUMBER];
	point_t init_pt, dst_pt, intersect_slot_pt;

	/* set init and destination points */
	init_pt.x = x0;
	init_pt.y = y0;
	dst_pt.x = x1;
	dst_pt.y = y1;

	//NOTICE(E_USER_STRAT,"set_slots_poly_in_path");
   //NOTICE(E_USER_STRAT,"init_pt (%"PRId32"  %"PRId32")", init_pt.x, init_pt.y);
 	//NOTICE(E_USER_STRAT,"dst_pt (%"PRId32", %"PRId32")", dst_pt.x, dst_pt.y);

	/* init poly structure */
	poly_slot.l = SLOT_EDGE_NUMBER;
	poly_slot.pts = poly_slot_pts;
   
	/* loop all slots less green areas and safe zones */  
	k = 0;
	for(i=2; i<(NB_SLOT_X-2); i++)
	{
		for(j=1; j<(NB_SLOT_Y-1); j++)
		{			
	
#ifndef HOST_VERSION		
			if((strat_infos.slot[i][j].flags & SLOT_AVOID) 
				& (strat_infos.slot[i][j].color == mainboard.our_color))
//			if((strat_infos.slot[i][j].color == mainboard.our_color))
#else
			if((strat_infos.slot[i][j].color == SLOT_BLUE)) /* XXX: test one color */
//			if(0)                                          /* XXX: test without slots poly */
#endif
			{  
				/* set points */				
				if(strat_infos.slot[i][j].flags_poly & SLOT_POLY_SQUARE) {
               poly_slot.l = set_square_pts(poly_slot_pts,
														  SLOT_MARGIN, SLOT_MARGIN,
														  strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);									
				}
				else {					
				   poly_slot.l = set_rhombus_pts(poly_slot_pts,
														   SLOT_RADIUS, SLOT_RADIUS,
														   strat_infos.slot[i][j].x, strat_infos.slot[i][j].y);
	         }
#ifdef SKIP_DST_POLY
				/* skip slot if destination point is included */
				if (is_point_in_poly(&poly_slot, dst_x, dst_y)) {
		      	NOTICE(E_USER_STRAT, " dst is in our color slot[%d][%d] %d", i, j, k);
		      	NOTICE(E_USER_STRAT, " skip slot");
		      	continue;
	      	}
#endif	      	
	      	/* skip slot if origin point is included */
				if (is_point_in_poly(&poly_slot, org_x, org_y)) {
		      	NOTICE(E_USER_STRAT, " we are in our color slot[%d][%d] %d", i, j, k);
		      	NOTICE(E_USER_STRAT, " skip slot");
		      	continue;
	      	}


				/* check if poly is in direct path */
		    	ret = is_crossing_poly(init_pt, dst_pt, &intersect_slot_pt, &poly_slot);
		                 
#ifdef TEST_ALL_POLYS		                 
		      /* test all poly generation */
		      ret = 1;               
#endif		                           
				/* if poly is in path set it */
				if(ret==1 && slot_in_path_flag[k]==0)
				{
					slot_in_path_flag[k] = 1;
					num_slots_in_path++;

					NOTICE(E_USER_STRAT,"slot[%d][%d] (%d) is in path", i, j, k);	         
					NOTICE(E_USER_STRAT,"num_slots_in_path %d", num_slots_in_path);
					
					*(pol+k) = oa_new_poly(poly_slot.l);
					set_poly_pts(*(pol+k), &poly_slot);           

				}
			}
			
			/* slot linear index*/
			k ++;
		}
	} 
}
#endif /* POLYS_IN_PATH */

/*
 * Go in playground, loop until out of poly. The argument robot_pt is 
 * the pointer to the current position of the robot.
 * Return 0 if there was nothing to do.
 * Return 1 if we had to move. In this case, update the theorical 
 * position of the robot in robot_pt.
 */
static int8_t go_in_area(point_t *robot_pt)
{
	point_t poly_pts_area[4];
	poly_t poly_area;
	point_t center_pt, dst_pt;

	center_pt.x = CENTER_X;
	center_pt.y = CENTER_Y;

	/* Go in playground */
	if (!is_in_boundingbox(robot_pt)){
		NOTICE(E_USER_STRAT, "not in playground %"PRId32", %"PRId32"",
		       robot_pt->x, robot_pt->y);

		poly_area.l = 4;
		poly_area.pts = poly_pts_area;
		poly_pts_area[0].x = strat_infos.area_bbox.x1;
		poly_pts_area[0].y = strat_infos.area_bbox.y1;

		poly_pts_area[1].x = strat_infos.area_bbox.x2;
		poly_pts_area[1].y = strat_infos.area_bbox.y1;

		poly_pts_area[2].x = strat_infos.area_bbox.x2;
		poly_pts_area[2].y = strat_infos.area_bbox.y2;

		poly_pts_area[3].x = strat_infos.area_bbox.x1;
		poly_pts_area[3].y = strat_infos.area_bbox.y2;

		is_crossing_poly(*robot_pt, center_pt, &dst_pt, &poly_area);
		NOTICE(E_USER_STRAT, "pt dst %"PRId32", %"PRId32"", dst_pt.x, dst_pt.y);

		NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
		       dst_pt.x, dst_pt.y);

		/* scape from poly */
#ifndef HOST_VERSION		
		strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
		robot_pt->x = dst_pt.x;
		robot_pt->y = dst_pt.y;


		return 1;
	}

	return 0;
}


/*
 * Escape from polygons if needed.
 * robot_pt is the current position of the robot, it will be
 * updated.
 */
static int8_t escape_from_poly(point_t *robot_pt,
										int16_t opp1_x, int16_t opp1_y, 
										int16_t opp2_x, int16_t opp2_y, 
										poly_t *pol_opp1, poly_t *pol_opp2, poly_t *pol_totems)
{
#define TOTEM_1_LIMIT_X    (TOTEM_1_X + 300)
#define TOTEM_2_LIMIT_X    (TOTEM_2_X - 300)

   int16_t totems_x, totems_y; 
	uint8_t in_opp1 = 0, in_opp2 = 0, in_totems = 0;
	double escape_dx = 0, escape_dy = 0;
	double opp1_dx = 0, opp1_dy = 0;
	double opp2_dx = 0, opp2_dy = 0;
	double totems_dx = 0, totems_dy = 0;
	double len;

	point_t dst_pt;
	point_t intersect_opp1_pt, intersect_opp2_pt, intersect_totems_pt;


	/* check if we are in any poly */
	if (is_in_poly(robot_pt, pol_opp1) == 1)
		in_opp1 = 1;

	if (is_in_poly(robot_pt, pol_opp2) == 1)
		in_opp2 = 1;

	if (is_in_poly(robot_pt, pol_totems) == 1)
		in_totems = 1;
   
	if (in_opp1 == 0 && in_opp2 == 0 && in_totems == 0) {
		NOTICE(E_USER_STRAT, "no need to escape");
		return 0;
	}
	
	NOTICE(E_USER_STRAT, "in_opp1=%d", in_opp1);
	NOTICE(E_USER_STRAT, "in_opp2=%d", in_opp2);
	NOTICE(E_USER_STRAT, "in_totems=%d", in_totems);
	
	/* determine totems coordinates */
	if(robot_pt->x < TOTEM_1_LIMIT_X) {
	   totems_x = TOTEM_1_X;
	   totems_y = TOTEM_1_Y;
	}
	else if(robot_pt->x > TOTEM_2_LIMIT_X){
		totems_x = TOTEM_2_X;
	   totems_y = TOTEM_2_Y;
	}
	else {
		totems_x = AREA_X/2;
	   totems_y = AREA_Y/2;	
	}
	
	/* process escape vectors */
	if (distance_between(robot_pt->x, robot_pt->y, opp1_x, opp1_y) < ESCAPE_POLY_THRES) {
		opp1_dx = robot_pt->x - opp1_x;
		opp1_dy = robot_pt->y - opp1_y;
		NOTICE(E_USER_STRAT, " robot is near opp1: vect=%2.2f,%2.2f",
		       opp1_dx, opp1_dy);
		len = norm(opp1_dx, opp1_dy);
		if (len != 0) {
			opp1_dx /= len;
			opp1_dy /= len;
		}
		else {
			opp1_dx = 1.0; /* XXX why?, is posible? */
			opp1_dy = 0.0;
		}
		escape_dx += opp1_dx;
		escape_dy += opp1_dy;
	}
	
	if (distance_between(robot_pt->x, robot_pt->y, opp2_x, opp2_y) < ESCAPE_POLY_THRES) {
		opp2_dx = robot_pt->x - opp2_x;
		opp2_dy = robot_pt->y - opp2_y;
		NOTICE(E_USER_STRAT, " robot is near opp2: vect=%2.2f,%2.2f",
		       opp2_dx, opp2_dy);
		len = norm(opp2_dx, opp2_dy);
		if (len != 0) {
			opp2_dx /= len;
			opp2_dy /= len;
		}
		else {
			opp2_dx = 1.0;
			opp2_dy = 0.0;
		}
		escape_dx += opp2_dx;
		escape_dy += opp2_dy;
	}

	if (distance_between(robot_pt->x, robot_pt->y, totems_x, totems_y) < ESCAPE_POLY_THRES) {
		totems_dx = robot_pt->x - totems_x;
		totems_dy = robot_pt->y - totems_y;
		NOTICE(E_USER_STRAT, " robot is near totems: vect=%2.2f,%2.2f",
		       totems_dx, totems_dy);
		len = norm(totems_dx, totems_dy);
		if (len != 0) {
			totems_dx /= len;
			totems_dy /= len;
		}
		else {
			totems_dx = 1.0;
			totems_dy = 0.0;
		}
		escape_dx += totems_dx;
		escape_dy += totems_dy;
	}
	
	/* normalize escape vector */
	len = norm(escape_dx, escape_dy);
	if (len != 0) {
		escape_dx /= len;
		escape_dy /= len;
	}
	else {
      if (pol_opp1 != NULL) {
			/* rotate 90� */
			escape_dx = opp1_dy;
			escape_dy = opp1_dx;
		}
      else if (pol_opp2 != NULL) {
			/* rotate 90� */
			escape_dx = opp2_dy;
			escape_dy = opp2_dx;
		}
      else if (pol_totems != NULL) {
			/* rotate 90� */
			escape_dx = totems_dy;
			escape_dy = totems_dx;
		}
		else { /* should not happen */
			escape_dx = 1.0;
			escape_dy = 0.0;
		}
	}

	NOTICE(E_USER_STRAT, " escape vect = %2.2f,%2.2f",
	       escape_dx, escape_dy);

	/* process the correct len of escape vector */
	dst_pt.x = robot_pt->x + escape_dx * ESCAPE_VECT_LEN;
	dst_pt.y = robot_pt->y + escape_dy * ESCAPE_VECT_LEN;

	NOTICE(E_USER_STRAT, "robot pt %"PRId32" %"PRId32,
	       robot_pt->x, robot_pt->y);
	NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
	       dst_pt.x, dst_pt.y);

	if (in_opp1) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_opp1_pt,
				     pol_opp1) == 1) {
				     
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp1_pt.x + escape_dx * 2;
			dst_pt.y = intersect_opp1_pt.y + escape_dy * 2;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       dst_pt.x, dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_totems, dst_pt.x, dst_pt.y) != 1 ){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       dst_pt.x, dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;
				
				return 0;
			}
		}
	}

	if (in_opp2) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_opp2_pt,
				     pol_opp2) == 1) {
				     
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_opp2_pt.x + escape_dx * 2;
			dst_pt.y = intersect_opp2_pt.y + escape_dy * 2;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       dst_pt.x, dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_totems, dst_pt.x, dst_pt.y) != 1 ){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       dst_pt.x, dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;
				
				return 0;
			}
		}
	}
	
	if (in_totems) {
		if (is_crossing_poly(*robot_pt, dst_pt, &intersect_totems_pt,
				     pol_totems) == 1) {
				     
			/* we add 2 cm to be sure we are out of th polygon */
			dst_pt.x = intersect_totems_pt.x + escape_dx * 2;
			dst_pt.y = intersect_totems_pt.y + escape_dy * 2;

			NOTICE(E_USER_STRAT, "dst point %"PRId32",%"PRId32,
			       dst_pt.x, dst_pt.y);

         /* XXX check that destination point is not in an other poly */
			if (is_point_in_poly(pol_opp1, dst_pt.x, dst_pt.y) != 1 &&
			    is_point_in_poly(pol_opp2, dst_pt.x, dst_pt.y) != 1 ){

            /* check if destination point is in playground */
				if (!is_in_boundingbox(&dst_pt))
					return -1;
				
				NOTICE(E_USER_STRAT, "GOTO %"PRId32",%"PRId32"",
				       dst_pt.x, dst_pt.y);

				/* XXX comment for virtual scape from poly */
#ifndef HOST_VERSION
				strat_goto_xy_force(dst_pt.x, dst_pt.y);
#endif
				robot_pt->x = dst_pt.x;
				robot_pt->y = dst_pt.y;
				
				return 0;
			}
		}
	}

	/* should not happen */
	return -1;
}

/* set totem islands polygon */
void set_totems_poly(poly_t *pol)
{
#define TOTEMS_W   250
#define TOTEMS_L   800 + TOTEMS_W
#define TOTEMS_X   1100 - (TOTEMS_W/2)
#define TOTEMS_Y   1000 - (TOTEMS_W/2)

#define POLY_CORNER        108

   oa_poly_set_point(pol, TOTEMS_X - POLY_CORNER, TOTEMS_Y - OBS_CLERANCE, 0);
   oa_poly_set_point(pol, TOTEMS_X + TOTEMS_L + POLY_CORNER, TOTEMS_Y - OBS_CLERANCE, 1);

   oa_poly_set_point(pol, TOTEMS_X + TOTEMS_L + OBS_CLERANCE, TOTEMS_Y - POLY_CORNER, 2);
   oa_poly_set_point(pol, TOTEMS_X + TOTEMS_L + OBS_CLERANCE, TOTEMS_Y + TOTEMS_W + POLY_CORNER, 3);

   oa_poly_set_point(pol, TOTEMS_X + TOTEMS_L + POLY_CORNER, TOTEMS_Y + TOTEMS_W + OBS_CLERANCE, 4);
   oa_poly_set_point(pol, TOTEMS_X - POLY_CORNER, TOTEMS_Y + TOTEMS_W + OBS_CLERANCE, 5);

   oa_poly_set_point(pol, TOTEMS_X - OBS_CLERANCE, TOTEMS_Y + TOTEMS_W + POLY_CORNER, 6);
   oa_poly_set_point(pol, TOTEMS_X - OBS_CLERANCE, TOTEMS_Y - POLY_CORNER, 7);
}
 

#define GO_AVOID_AUTO		0
#define GO_AVOID_FORWARD	1
#define GO_AVOID_BACKWARD	2

#ifndef HOST_VERSION
static int8_t __goto_and_avoid(int16_t x, int16_t y,
			       uint8_t flags_intermediate,
			       uint8_t flags_final, uint8_t direction)
#else
int8_t goto_and_avoid(int16_t x, int16_t y,
					   	int16_t robot_x, int16_t robot_y, double robot_a,
					   	int16_t opp1_x, int16_t opp1_y, 
					   	int16_t opp2_x, int16_t opp2_y)

#endif
{
	int8_t len = -1;
	int8_t i;

#ifdef POLYS_IN_PATH
	int8_t num_slots_in_path_save;
	poly_t *pol_slots_in_path[SLOT_NUMBER];
#endif

	point_t *p;
	poly_t *pol_opp1, *pol_opp2;
	poly_t *pol_totems;
	int8_t ret;

	int16_t opp1_w, opp1_l;
	int16_t opp2_w, opp2_l;
#ifndef HOST_VERSION
	int16_t opp1_x, opp1_y;
	int16_t opp2_x, opp2_y;
#endif	

	point_t p_dst, robot_pt;
	
	void * p_retry;
	p_retry = &&retry;

	
#ifndef HOST_VERSION	
	DEBUG(E_USER_STRAT, "%s(%d,%d) flags_i=%x flags_f=%x direct=%d",
	      __FUNCTION__, x, y, flags_intermediate, flags_final, direction);
#else
	g_robot_x = robot_x;
	g_robot_y = robot_y;
	g_robot_a = robot_a;
	g_opp1_x = opp1_x;
	g_opp1_y = opp1_y;
	g_opp2_x = opp2_x;
	g_opp2_y = opp2_y;
#endif

 retry:

#ifdef POLYS_IN_PATH
	/* reset slots in path */
  	num_slots_in_path = 0;
  	for(i=0; i<SLOT_NUMBER; i++)
		slot_in_path_flag[i] = 0;
#endif
	
	/* opponent info */
#ifndef HOST_VERSION	
	get_opponent_xy(&opp1_x, &opp1_y);
	/* TODO: get second opponent */
	opp2_x = I2C_OPPONENT_NOT_THERE;
	opp2_y = I2C_OPPONENT_NOT_THERE;
#endif

	opp1_w = O_WIDTH;
	opp1_l = O_LENGTH;
	opp2_w = O_WIDTH;
	opp2_l = O_LENGTH;

	/* robot info */
#ifndef HOST_VERSION
	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);
#else
	robot_pt.x = robot_x;
	robot_pt.y = robot_y;
#endif

	/* init oa */
	oa_init();
  
   /* add totems islands poly */
   pol_totems = oa_new_poly(8);
   set_totems_poly(pol_totems);
  
	/* add opponent poly */
	pol_opp1 = oa_new_poly(4);
	set_opponent_1_poly(pol_opp1, &robot_pt, O_WIDTH, O_LENGTH);
	pol_opp2 = oa_new_poly(4);
	set_opponent_2_poly(pol_opp2, &robot_pt, O_WIDTH, O_LENGTH);
																				

	/* if we are not in the limited area, try to go in it. */
	ret = go_in_area(&robot_pt);

#ifdef POLYS_IN_PATH
	/* set slots in path beetween robot and destination point */ 
	set_slots_poly_in_path(&pol_slots_in_path[0],
	                       robot_pt.x, robot_pt.y, x, y, 
	                       robot_pt.x, robot_pt.y, x, y);
#endif

	/* check that destination is in playground */
	p_dst.x = x;
	p_dst.y = y;
	if (!is_in_boundingbox(&p_dst)) {
		NOTICE(E_USER_STRAT, " dst is not in playground");
		return END_ERROR;
	}
  
	/* check if destination is in opponent */
  	if (is_point_in_poly(pol_opp1, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp 1");
		return END_ERROR;
	}
  	if (is_point_in_poly(pol_opp2, x, y)) {
		NOTICE(E_USER_STRAT, " dst is in opp 2");
		return END_ERROR;
	}

	/* now start to avoid */
	while (opp1_w && opp1_l && opp2_w && opp2_l) {

      /* escape from polys */
		/* XXX robot_pt is not updated if it fails */		
		ret = escape_from_poly(&robot_pt,
				                opp1_x, opp1_y, opp2_x, opp2_y, 
				                pol_opp1, pol_opp2, pol_totems);
		

      /* XXX uncomment in order to skip escape from poly */
		//ret = 0;	

		if (ret == 0) {

 			/* reset and set start and end points */
			oa_reset();
			oa_start_end_points(robot_pt.x, robot_pt.y, x, y);
			//oa_dump();
	
			/* proccesing path */
			len = oa_process();
	
			if (len > 0) {

#ifdef POLYS_IN_PATH
				/* check if there are any more slots in path */
				num_slots_in_path_save = num_slots_in_path;
            p = oa_get_path();
            
            /* loop for all pair of path points */
            set_slots_poly_in_path(&pol_slots_in_path[0], 
                                   robot_pt.x, robot_pt.y, 
                                   p->x, p->y,
                                   robot_pt.x, robot_pt.y, 
                                   x, y);
                                   
         	for (i=0 ; i<(len-1) ; i++) {
	            set_slots_poly_in_path(&pol_slots_in_path[0],
	                                   p->x, p->y, 
	                                   (p+1)->x, (p+1)->y, 
	                                   robot_pt.x, robot_pt.y,
	                                   x, y);
	            p++;
				}
				
				/* if there are new slots in path repeat oa */
				if(num_slots_in_path_save != num_slots_in_path){
   				NOTICE(E_USER_STRAT,"new slots in path");
					continue;   				
				}
				else
#endif				
				   break;
			}
			else if(len == 0)
			   break;
		}

		/* len < 0, try reduce opponent to get a valid path */
		if (distance_between(robot_pt.x, robot_pt.y, opp1_x, opp1_y) < REDUCE_POLY_THRES ) {
			if (opp1_w == 0) {
				opp1_l /= 2;
			}

			opp1_w /= 2;

			NOTICE(E_USER_STRAT, "reducing opponent 1 %d %d", opp1_w, opp1_l);
			set_opponent_1_poly(pol_opp1, &robot_pt, opp1_w, opp1_l);
		}
		else if (distance_between(robot_pt.x, robot_pt.y, opp2_x, opp2_y) < REDUCE_POLY_THRES ) {
			if (opp2_w == 0) {
				opp2_l /= 2;
			}

			opp2_w /= 2;

			NOTICE(E_USER_STRAT, "reducing opponent 2 %d %d", opp2_w, opp2_l);
			set_opponent_2_poly(pol_opp2, &robot_pt, opp2_w, opp2_l);
		}
		else {
			NOTICE(E_USER_STRAT, "oa_process() returned %d", len);
			return END_ERROR;
		}
	}
	
	/* execute path */
	p = oa_get_path();
	for (i=0 ; i<len ; i++) {

#ifndef HOST_VERSION
		
//		if (d<20){
//			p++;
//			continue;
//		}

		if (direction == GO_AVOID_FORWARD){
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" forward", i, p->x, p->y);
			trajectory_goto_forward_xy_abs(&mainboard.traj, p->x, p->y);
		}
		else if(direction == GO_AVOID_BACKWARD){
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" backward", i, p->x, p->y);
			trajectory_goto_backward_xy_abs(&mainboard.traj, p->x, p->y);
		}
		else {
			DEBUG(E_USER_STRAT, "With avoidance %d: x=%"PRId32" y=%"PRId32" forward", i, p->x, p->y);
			trajectory_goto_xy_abs(&mainboard.traj, p->x, p->y);
		}
		
		/* no END_NEAR for the last point */
		if (i == len - 1)
			ret = wait_traj_end(flags_final);
		else
			ret = wait_traj_end(flags_intermediate);

		if (ret == END_BLOCKING) {
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto *p_retry;
		}
		else if (ret == END_OBSTACLE) {
			/* brake and wait the speed to be slow */
			DEBUG(E_USER_STRAT, "Retry avoidance %s(%d,%d)",
			      __FUNCTION__, x, y);
			goto *p_retry;
		}
		/* else if it is not END_TRAJ or END_NEAR, return */
		else if (!TRAJ_SUCCESS(ret)) {
			return ret;
		}

#endif /* HOST_VERSION */

		DEBUG(E_USER_STRAT, "With avoidance %d: x=%d y=%d", i, p->x, p->y);		

		/* next point */
		p++;
	}
	
	return END_TRAJ;
}

#ifndef HOST_VERSION
/* go to a x,y point */
uint8_t goto_and_avoid(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_AUTO);
}

/* go forward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_forward(int16_t x, int16_t y, uint8_t flags_intermediate,
			       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_FORWARD);
}

/* go backward to a x,y point. use current speed for that */
uint8_t goto_and_avoid_backward(int16_t x, int16_t y, uint8_t flags_intermediate,
		       uint8_t flags_final)
{
	return __goto_and_avoid(x, y, flags_intermediate, flags_final, GO_AVOID_BACKWARD);
}

#endif




#if 0
void set_rotated_pentagon(poly_t *pol, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){
		
		if(special == 1 && (i==0||i==1||i==2))
		   oa_poly_set_point(pol, x + px1, PLAYGROUND_Y_MAX+2, i);
		else if(special == 2 && (i==3))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else if(special == 3 && (i==5))
		   oa_poly_set_point(pol, x + px1, y + py1 - OFFSET_AVOID_RAMPE, i);
		else
		   oa_poly_set_point(pol, x + px1, y + py1, i);
	
		
		px2 = px1*c_a + py1*s_a;
		py2 = -px1*s_a + py1*c_a;

		px1 = px2;
		py1 = py2;
	}

}

void set_rotated_pentagon_pts(point_t *pt, int16_t radius,
			      int16_t x, int16_t y, uint8_t special)
{

	double c_a, s_a;
	uint8_t i;
	double px1, py1, px2, py2;
	double a_rad;

	//a_rad = atan2(y - robot_y, x - robot_x);
   a_rad = 0;

	/* generate pentagon  */
	c_a = cos(-2*M_PI/CORN_EDGE_NUMBER);
	s_a = sin(-2*M_PI/CORN_EDGE_NUMBER);

	/*
	px1 = radius;
	py1 = 0;
	*/
	px1 = radius * cos(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));
	py1 = radius * sin(a_rad + 2*M_PI/(2*CORN_EDGE_NUMBER));


	for (i = 0; i < CORN_EDGE_NUMBER; i++){
	
	  if(special == 1 && (i==0||i==1||i==2)){
			pt[i].x = x + px1;
		  pt[i].y = PLAYGROUND_Y_MAX+2;
		}
		else if(special == 2 && (i==3)){
		  pt[i].x = x + px1;
		  pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
    }
		else if(special == 3 && (i==5)){
		  pt[i].x = x + px1;
		  pt[i].y = y + py1 - OFFSET_AVOID_RAMPE;
    }
		else{	
		  pt[i].x = x + px1;
		  pt[i].y = y + py1;
		}   		
    
	  px2 = px1*c_a + py1*s_a;
	  py2 = -px1*s_a + py1*c_a;
      
		px1 = px2;
		py1 = py2;
	}

}

//#define EDGE_NUMBER 5
//void set_rotated_pentagon(poly_t *pol, const point_t *robot_pt,
//			  int16_t radius, int16_t x, int16_t y)
//{
//
//	double c_a, s_a;
//	uint8_t i;
//	double px1, py1, px2, py2;
//	double a_rad;
//
//	a_rad = atan2(y - robot_pt->y, x - robot_pt->x);
//
//	/* generate pentagon  */
//	c_a = cos(-2*M_PI/EDGE_NUMBER);
//	s_a = sin(-2*M_PI/EDGE_NUMBER);
//
//	/*
//	px1 = radius;
//	py1 = 0;
//	*/
//	px1 = radius * cos(a_rad + 2*M_PI/(2*EDGE_NUMBER));
//	py1 = radius * sin(a_rad + 2*M_PI/(2*EDGE_NUMBER));
//
//
//	for (i = 0; i < EDGE_NUMBER; i++){
//		oa_poly_set_point(pol, x + px1, y + py1, i);
//		
//		px2 = px1*c_a + py1*s_a;
//		py2 = -px1*s_a + py1*c_a;
//
//		px1 = px2;
//		py1 = py2;
//	}
//}

#endif
