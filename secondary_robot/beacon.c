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

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <i2c.h>
#include <parse.h>
#include <rdline.h>
#include <pwm_mc.h>
#include <encoders_dspic.h>
#include <timer.h>
#include <scheduler.h>
#include <pid.h>
#include <clock_time.h>
#include <quadramp.h>
#include <control_system_manager.h>

#include <blocking_detection_manager.h>

#include "sensor.h"

#include "../common/i2c_commands.h"
#include "main.h"
#include "beacon.h"
#include "beacon_calib.h"
#include "../maindspic/strat_utils.h"

/* functional modes of beacon */
#undef BEACON_MODE_EXTERNAL


/* some conversions and constants */
#define DEG(x) (((double)(x)) * (180.0 / M_PI))
#define RAD(x) (((double)(x)) * (M_PI / 180.0))
#define M_2PI (2*M_PI)

/* field area */
#define AREA_X 3000
#define AREA_Y 2100

/* convert coords according to our color */
#define COLOR_X(x)     ((beaconboard.our_color==I2C_COLOR_RED)? (x) : (AREA_X-(x)))
#define COLOR_Y(y)     ((beaconboard.our_color==I2C_COLOR_RED)? (y) : (AREA_Y-(y)))

/* fixed beacon coordenates */
#ifdef BEACON_MODE_EXTERNAL
#define BEACON_X_OFFSET    (-62)                // beacon calibration includes the offset of 6.2cm
#define BEACON_Y_OFFSET    (AREA_Y/2)
#define BEACON_A_OFFSET    (0)
#endif /* BEACON_MODE_EXTERNAL */

/* IR sensors pin read value */
#define IR_SENSOR_180_DEG_PIN()     (!(_RB2))

#define EDGE_RISING        0
#define EDGE_FALLING     1
#define EDGE_MAX             2

/* modulo of timer base */
#define MODULO_TIMER (65535L)

/* to work in units of mili degrees */
#define MDEG (1000L)

/* debug macros */
#define BEACON_DEBUG(args...) DEBUG(E_USER_BEACON, args)
#define BEACON_NOTICE(args...) NOTICE(E_USER_BEACON, args)
#define BEACON_ERROR(args...) ERROR(E_USER_BEACON, args)

/* beacon calculations funcions */
//static int32_t get_dist(int32_t size, int32_t period);
static int32_t get_angle(int32_t middle, int32_t period, int32_t offset);

/* data structure to store beacon results */
struct beacon beacon;

/* turn period measure */
static volatile int32_t count_period = 0;        /* counts of timer asociated */
static volatile int32_t count_period_ov = 0;    /* overflow flag of timer    */
static volatile int8_t valid_period = 0;        /* new valid measure is available */

/* IR sensors pulse measure */
static volatile int32_t 
count_edge[IR_SENSOR_MAX][EDGE_MAX] = {{0, 0}, {0 ,0}};         /* counts of timer */
static volatile int32_t 
count_edge_ov[IR_SENSOR_MAX][EDGE_MAX] = {{0, 0}, {0 ,0}};    /* overflow flag   */
static volatile int8_t 
valid_pulse[IR_SENSOR_MAX] = {0, 0};    /* new valid measures available */
static int32_t invalid_count[IR_SENSOR_MAX] = {0, 0};        /* timeout of pulse measure */


/* initialize beacon */
void beacon_init(void)
{
    /* clear data structures */
    memset(&beacon, 0, sizeof(struct beacon));
    beacon.opponent1_x = I2C_OPPONENT_NOT_THERE;
#ifdef TWO_OPPONENTS
    beacon.opponent2_x = I2C_OPPONENT_NOT_THERE;
    beacon.tracking_opp1_x = beacon.tracking_opp1_y = I2C_OPPONENT_NOT_THERE;
    beacon.tracking_opp2_x = beacon.tracking_opp2_y = I2C_OPPONENT_NOT_THERE;
#endif
#ifdef ROBOT_2ND
    beacon.robot_2nd_x = I2C_OPPONENT_NOT_THERE;
#endif

    /* HARDWARE INIT */

    /* XXX: all measures are syncronized with then Timer 2 */

    /* initialize input capture (IC) 2 for IR sensors events */
    IC2CONbits.ICM =0b00;         // disable Input Capture module
    IC2CONbits.ICTMR = 1;         // select Timer2 as the IC time base
    IC2CONbits.ICI = 0b00;         // interrupt on every capture event
    IC2CONbits.ICM = 0b001;     // generate capture event on every edge

    /* initialize input capture 7 */
    IC7CONbits.ICM =0b000;        // disable Input Capture module
    IC7CONbits.ICTMR = 1;         // select Timer2 as the IC time base
    IC7CONbits.ICI = 0b00;         // interrupt on every capture event
    IC7CONbits.ICM = 0b011;     // generate capture event on every rising edge


    /* enable all inputs capture interrupts and Timer 2 */
    IPC1bits.IC2IP = 5;     // setup IC2 interrupt priority level XXX, higher than scheduler!
    IFS0bits.IC2IF = 0;     // clear IC2 Interrupt Status Flag
    IEC0bits.IC2IE = 1;     // enable IC2 interrupt

    IPC5bits.IC7IP = 6;     // setup IC7 interrupt priority level XXX, higher than scheduler!
    IFS1bits.IC7IF = 0;     // clear IC7 Interrupt Status Flag
    IEC1bits.IC7IE = 1;     // enable IC7 interrupt


	/* config encoder input */
	T4CONbits.TCS = 1;      // TXCK pin is the clock */
    T4CONbits.TON = 1;      // enable Timer
	 


    /* config and enable Timer 2 */
    PR2 = 65535;
    IFS0bits.T2IF      = 0;     // clear T2 Interrupt Status Flag

    T2CONbits.TCKPS = 0b11;    // Timer 2 prescaler, T_TIMER2 = 6.4 us
    T2CONbits.TON     = 1;        // enable Timer 2


    /* CS EVENT */
    scheduler_add_periodical_event_priority(beacon_calc, NULL,
                        EVENT_PERIOD_BEACON / SCHEDULER_UNIT, EVENT_PRIO_BEACON);

}


/* input compare 2 interrupt connected to IR_SENSOR */
void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{
    uint8_t flags;

    /* reset flag */
    _IC2IF=0;

    /* Capture of Timer 2 counts on falling and risign edge of IR sensor.
    * After falling edge set valid_pulse.
     */

    /* NOTE: Timer 2 count is hardware buffered by Input capture so,
     *       we don't lose counts.
     */

    /* rising edge */
    if ( IR_SENSOR_180_DEG_PIN()) {
        IRQ_LOCK(flags);
        count_edge[IR_SENSOR_180_DEG][EDGE_RISING] = (int32_t)IC2BUF;
        count_edge_ov[IR_SENSOR_180_DEG][EDGE_RISING] = _T2IF;
        valid_pulse[IR_SENSOR_180_DEG] = 0;
        IRQ_UNLOCK(flags);
    }
    /* falling edge */
    else {
        IRQ_LOCK(flags);
        count_edge[IR_SENSOR_180_DEG][EDGE_FALLING] = (int32_t)IC2BUF;
        count_edge_ov[IR_SENSOR_180_DEG][EDGE_FALLING] = _T2IF;
        valid_pulse[IR_SENSOR_180_DEG] = 1;
        IRQ_UNLOCK(flags);
    }

}

/* input compare 7 interrupt connected to turn sensor aligned with IR_SENSOR_0_DEG */
void __attribute__((__interrupt__, no_auto_psv)) _IC7Interrupt(void)
{
    uint8_t flags;

    /* reset flag */
    _IC7IF=0;


    /* Capture of Timer 2 counts on risign edge of turn sensor.
     */

    /* block interrupt */
    IRQ_LOCK(flags);

    /* reset timer */
    TMR2 = 0;

    /* XXX: there is a delay between the instant of capture
    * and timer reset, this involve an offset error on angle measure.
    */

    /* save bufferd counts and overflow flag */
    count_period = (int32_t)IC7BUF;
    count_period_ov = _T2IF;

    /* reset overflow flag */
    _T2IF = 0;

    /* set valid_period */
    valid_period = 1;

    /* unblock interrupt */
    IRQ_UNLOCK(flags);
}

/* beacon enconder */
static int32_t encoder_val = 0, encoder_val_previous = 0;

void beacon_encoder_manage(void) 
{
	uint8_t flags;
	uint16_t val;
	int32_t temp;
	

	IRQ_LOCK (flags);
	val = TMR4;      
	IRQ_UNLOCK (flags);               		
                    							
	temp = (val - encoder_val_previous); 
	encoder_val_previous = val;        
                                             	
	encoder_val += temp ;
}


int32_t beacon_encoder_get_value(void) 
{
  int32_t value;
  uint8_t flags;
  
  IRQ_LOCK(flags);
  value = encoder_val;
  IRQ_UNLOCK(flags);

  return value;
}

void beacon_encoder_set_value(int32_t val) 
{
	uint8_t flags;
	IRQ_LOCK (flags);
	encoder_val = val;
	IRQ_UNLOCK (flags);  
}

/* reset of position of beacon */
void beacon_reset_pos(void)
{
   beacon_encoder_set_value(0);
}

/* start turn and measures */
void beacon_start(void)
{
    /* enable beacon_calc event flag */
    mainboard.flags |= DO_BEACON;

    /* enable cs */
    beacon_reset_pos();
    pid_reset(&mainboard.beacon_speed.pid);
    mainboard.beacon_speed.on = 1;
    cs_set_consign(&mainboard.beacon_speed.cs, 80/4);
}

/* stop turn and measures */
void beacon_stop(void)
{
	 uint8_t flags;

    /* disable beacon_calc event flag */
    mainboard.flags &= ~(DO_BEACON);

    /* disable cs */
    cs_set_consign(&mainboard.beacon_speed.cs, 0);
    wait_ms(2000);

	IRQ_LOCK (flags);
    mainboard.beacon_speed.on = 0;
    pwm_mc_set(BEACON_MOTOR, 0);
    pid_reset(&mainboard.beacon_speed.pid);
    pid_reset(&mainboard.beacon_speed.pid);
	IRQ_UNLOCK (flags);

    /* no opponent there */
    beacon.opponent1_x = I2C_OPPONENT_NOT_THERE;
#ifdef TWO_OPPONENTS
    beacon.opponent2_x = I2C_OPPONENT_NOT_THERE;
#endif
#ifdef ROBOT_2ND
    beacon.robot_2nd_x = I2C_OPPONENT_NOT_THERE;
#endif
}

/**********************************************************************
 * HELPERS FOR BEACON CALCULUS
 *********************************************************************/

#if 0
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

/* return the normal of a vector with origin (0,0) */
static double norm(double x, double y)
{
    return sqrt(x*x + y*y);
}

/* calculate the distance and angle (between -180 and 180) of a beacon
 * opponent coordenates relative to robot coordinates                          */
void abs_xy_to_rel_da(double x_robot, double y_robot, double a_robot, 
                             double x_abs, double y_abs,
                           int32_t *d_rel, int32_t *a_rel_deg)
{
    double a_rel_rad;

    a_rel_rad = atan2(y_abs - y_robot, x_abs - x_robot) - RAD(a_robot);

    if (a_rel_rad < -M_PI) {
        a_rel_rad += M_2PI;
    }
    else if (a_rel_rad > M_PI) {
        a_rel_rad -= M_2PI;
    }

    *a_rel_deg = (int32_t)(DEG(a_rel_rad));
    *d_rel = (int32_t)(norm(x_abs-x_robot, y_abs-y_robot));
}

#endif

/* return true if the point (x,y) is in area defined by margin */
uint8_t is_in_margin(int16_t dx, int16_t dy, int16_t margin)
{
    if ((ABS(dx) < margin) && (ABS(dy) < margin))
        return 1;
    return 0;
}


/* calculate distance from size of a pulse width and turn period */
//static int32_t get_dist(int32_t size, int32_t period)
//{
//    int32_t dist=0;
//    double size_rel;
//
//    /* calcule relative angle */
//    size_rel = size*1.0/period;
//
//    /* dist = offset + (a0 + a1*x + a2*x² + a3x³) */
//    dist =  (int32_t)((0.0062 + (-0.1546*size_rel) +
//                            (1.1832*size_rel*size_rel) +
//                            (-2.4025*size_rel*size_rel*size_rel))*100000);
//
//    /* practical offset */
//    dist += 16;
// 
//    return dist;
//}

/* calculate angle from middle of pulse width, turn period and angle offset */
static int32_t get_angle(int32_t middle, int32_t period, int32_t offset)
{
    int32_t ret_angle;

    ret_angle = (int32_t)(middle * 360.0 * MDEG / period);
    ret_angle = (ret_angle + offset*MDEG)%(360*MDEG);

    return (int32_t)(360-(ret_angle/MDEG)); /* XXX angle is -ret_angle because beacon turns clockwise */
}

/* calculate absolute (x,y) coordinates from angle and distance measures */
void beacon_angle_dist_to_x_y(int32_t angle, int32_t dist, int32_t *x, int32_t *y)
{
#ifndef BEACON_MODE_EXTERNAL
    uint8_t flags;
#endif

    int32_t local_x = 0;
    int32_t local_y = 0;
    int32_t x_opponent;
    int32_t y_opponent;
    int32_t local_robot_angle = 0;

#ifndef BEACON_MODE_EXTERNAL
    IRQ_LOCK(flags);
    local_x           = beacon.robot_x;
    local_y           = beacon.robot_y;
    local_robot_angle = beacon.robot_a;
    IRQ_UNLOCK(flags);
#endif

    if (local_robot_angle < 0)
        local_robot_angle += 360;

    x_opponent = cos((local_robot_angle + angle)* 2 * M_PI / 360)* dist;
    y_opponent = sin((local_robot_angle + angle)* 2 * M_PI / 360)* dist;

//    BEACON_DEBUG("x_op= %ld\t",x_opponent);
//    BEACON_DEBUG("y_op= %ld\r\n",y_opponent);
//    BEACON_NOTICE("robot_x= %ld\t",local_x);
//    BEACON_NOTICE("robot_y= %ld\t",local_y);
//    BEACON_NOTICE("robot_angle= %ld\r\n",local_robot_angle);

    *x = local_x + x_opponent;
    *y = local_y + y_opponent;

}


/**********************************************************************
 * BEACON CALCULUS
 *********************************************************************/

/* calculate distance (d) and angle (a) and (x,y) possition of the 
 * opponent beacon relative to the robot coordinates.
 */
void sensor_calc(uint8_t sensor)
{
    /* XXX: filtered versions of variable are only for calib and debug */

    static int32_t local_count_period;
    static int32_t local_count_period_filtered = 0;
    int32_t local_count_period_ov;

    int32_t local_count_edge[EDGE_MAX];
    int32_t local_count_edge_ov[EDGE_MAX];

    int32_t count_size;
    static int32_t count_size_filtered = 0;

    int32_t count_middle;
    static int32_t count_middle_filtered = 0;

    int32_t local_angle;
    int32_t local_dist;

#ifdef BEACON_MODE_EXTERNAL
    int32_t local_robot_x = 0;
    int32_t local_robot_y = 0;
    int32_t local_robot_a = 0;
#endif

#ifdef TWO_OPPONENTS
    int16_t d_opp1, d_opp2;
    uint8_t tracking_update = 0;
    int16_t angle_dif = 0;
#define OPPONENT_1    1
#define OPPONENT_2    2
#endif

    int32_t result_x = 0;
    int32_t result_y = 0;
    uint8_t flags;

    /* calculate/update turn period if valid*/
    if(valid_period){

        IRQ_LOCK(flags);

        /* read turn period measures */
        local_count_period     = count_period;
        local_count_period_ov = count_period_ov;

        /* reset flag */
        valid_period = 0;

        IRQ_UNLOCK(flags);

        /* calculate period in counts */
        local_count_period += ((MODULO_TIMER + 1)*local_count_period_ov);

        /* low pass filtered version of period */
        local_count_period_filtered = (int32_t)(local_count_period_filtered*0.5 + local_count_period*0.5);
    }

    /* if not valid pulse return */
    if(!valid_pulse[sensor]){
        //BEACON_NOTICE("non valid pulse\r\n\n");
        goto error;
    }

    /* continue with the calculus ... */

    /* read measurements */
    IRQ_LOCK(flags);

    /* rising edge measure */
    local_count_edge[EDGE_RISING]      = count_edge[sensor][EDGE_RISING];
    local_count_edge_ov[EDGE_RISING] = count_edge_ov[sensor][EDGE_RISING];

    /* falling edge measure */
    local_count_edge[EDGE_FALLING] = count_edge[sensor][EDGE_FALLING];
    local_count_edge_ov[EDGE_FALLING] = count_edge_ov[sensor][EDGE_FALLING];

    /* reset flag */
    valid_pulse[sensor]=0;

    IRQ_UNLOCK(flags);

    /* calculate total edges counts */
    local_count_edge[EDGE_RISING] = local_count_edge[EDGE_RISING] + (MODULO_TIMER + 1)*local_count_edge_ov[EDGE_RISING];
    local_count_edge[EDGE_FALLING] = local_count_edge[EDGE_FALLING] + (MODULO_TIMER + 1)*local_count_edge_ov[EDGE_FALLING];

    /* calcule pulse size and the middle */
    if(local_count_edge[EDGE_RISING]> local_count_edge[EDGE_FALLING])
    {
        count_size = local_count_period - local_count_edge[EDGE_RISING] + local_count_edge[EDGE_FALLING];
        count_middle = (local_count_edge[EDGE_RISING] + (int32_t)(count_size/2) + local_count_period) % local_count_period;
    }
    else
    {
        count_size = local_count_edge[EDGE_FALLING] - local_count_edge[EDGE_RISING];
        count_middle = local_count_edge[EDGE_RISING] + (int32_t)(count_size/2);
    }

    /* filter version of size and middle counts */
    count_size_filtered   = (int32_t)(count_size_filtered*0.5 + count_size*0.5);
    count_middle_filtered = (int32_t)(count_middle_filtered*0.5 + count_middle*0.5);


    /* debug counts, uncomment for calibrate */
    /*
    BEACON_DEBUG("period = %.5ld / size = %.5ld / middle = %.5ld (x0.1)",
                     local_count_period_filtered/10, count_size_filtered/10, count_middle_filtered/10);
                     //local_count_period/10, count_size/10, count_middle/10);
    return;
    */

    /* if pulse width is out of range return */
/*        if(count_size > 5000){
            BEACON_DEBUG("count_size_discarted = %ld", count_size);
            goto error;
        }
*/

    /* calculate angle in Mega degrees */
    if(sensor == IR_SENSOR_180_DEG)
        local_angle = get_angle(count_middle, local_count_period, 180);
    else
        local_angle = get_angle(count_middle, local_count_period, 0);

    /* calculate distance in mm */
    local_dist = get_dist_array(sensor, count_size, local_count_period);
    //local_dist = get_dist_array(sensor, count_size_filtered, local_count_period_filtered);

    if(local_dist == DIST_ERROR)
        goto error;


    /* debug angle and distance */
    //BEACON_DEBUG("a = %.4ld deg / d = %.4ld mm", local_angle, local_dist);

#ifdef BEACON_MODE_EXTERNAL
    /* check angle range */
    if(local_angle > 85){
        if(local_angle < 275){
            BEACON_DEBUG("angle discarted = %ld", local_angle);
            goto error;
        }
    }
#endif

    /* calculate (x,y) coordenates relative to (0,0) */
    beacon_angle_dist_to_x_y(local_angle, local_dist, &result_x, &result_y);

    /* error if point is out off playground */
#define PLAYGROUND_MARGIN 120
    if(result_x < PLAYGROUND_MARGIN || result_x > (3000-PLAYGROUND_MARGIN)) {
        BEACON_NOTICE("discard xy (%ld %ld), x is out of playground", result_x, result_y);
        goto error;
    }
    if(result_y < PLAYGROUND_MARGIN || result_y > (2000-PLAYGROUND_MARGIN)) {
        BEACON_NOTICE("discard xy (%ld %ld), y is out of playground", result_x, result_y);
        goto error;
    }


#ifdef BEACON_MODE_EXTERNAL

    /* translate (x,y) to beacon coordinates */
    result_x = COLOR_X(result_x + BEACON_X_OFFSET);
    result_y = COLOR_Y((-result_y) + BEACON_Y_OFFSET);

    /* translate (x,y) coodinates to (d,a) coordinates relative to robot */
    abs_xy_to_rel_da(local_robot_x, local_robot_y, local_robot_a,
                          result_x, result_y, &local_dist, &local_angle);

    //BEACON_NOTICE("opponent rel beacon angle= %.3d\t",local_angle);
    //BEACON_NOTICE("opponent rel beacon dist= %.4ld\r\n",local_dist);

    /* if opponent is in our robot area return */
    if(is_in_margin((result_x-local_robot_x), (result_y-local_robot_y), 255)){ // 255 robot_length/2 + error_baliza_max(100mm)
        BEACON_NOTICE("discard xy (%ld %ld) in robot (%ld %ld) margin\r\n",
                           result_x, result_y, local_robot_x, local_robot_y);
        goto error;
    }

#endif /* BEACON_MODE_EXTERNAL */

    /* reset timeout */
    invalid_count[sensor] = 0;

    if(sensor == IR_SENSOR_180_DEG)
    {
    #ifndef TWO_OPPONENTS
        /* update results */
        IRQ_LOCK(flags);
        beacon.opponent1_x = result_x;
        beacon.opponent1_y = result_y;
        beacon.opponent1_angle = local_angle;
        beacon.opponent1_dist = local_dist;
        IRQ_UNLOCK(flags);

        /* final results */
        BEACON_NOTICE("opp1: a = %.3ld / d = %.4ld / x = %.4ld / y = %.4ld",
                         beacon.opponent1_angle, beacon.opponent1_dist,
                         beacon.opponent1_x, beacon.opponent1_y);
        return;

    #else /* TWO OPPONENTS */

#define TRACKING_WINDOW_mm    200

        /* XXX
            HIGH LIKELY CASE: case of reflexive beacon on in our secondary robot.
            Solution: discard if the angle is very similar to secondary robot angle
         */

#ifdef ROBOT_2ND
#define ANGLE_MARGIN    4
#define DIST_MARGIN    150
        if(beacon.robot_2nd_x != I2C_OPPONENT_NOT_THERE) {

            angle_dif = beacon.robot_2nd_angle - local_angle;

            if( ABS(angle_dif) < ANGLE_MARGIN) {
                //&& (ABS(beacon.robot_2nd_dist) - ABS(local_dist)) < DIST_MARGIN) {
                BEACON_NOTICE("discarting opponent because is robot 2nd (a %ld, diff %d)", local_angle, angle_dif);
                goto error;
            }
        }
#endif


        /* distance from new xy to opponents xy */
        d_opp1 = distance_between(result_x, result_y, beacon.tracking_opp1_x, beacon.tracking_opp1_y);
        d_opp2 = distance_between(result_x, result_y, beacon.tracking_opp2_x, beacon.tracking_opp2_y);

        /* mimimun distance inside a window */
        if(d_opp1 <= d_opp2) {
             if(d_opp1 < TRACKING_WINDOW_mm)
                tracking_update = OPPONENT_1;
        }
        else {
            if(d_opp2 < TRACKING_WINDOW_mm)
                tracking_update = OPPONENT_2;
        }

        /* initialization or not in window */
        if(tracking_update == 0) {

            /* initial state or lost both opponents */
            if(beacon.opponent1_x == I2C_OPPONENT_NOT_THERE) {
                tracking_update = OPPONENT_1;
                BEACON_DEBUG("Opponent 1 intialized");
            }
            else if(beacon.opponent2_x == I2C_OPPONENT_NOT_THERE) {
                tracking_update = OPPONENT_2;
                BEACON_DEBUG("Opponent 2 intialized");
            }
            else {
                BEACON_DEBUG("Point out of windows");

                if(d_opp1 < d_opp2) tracking_update = OPPONENT_1;
                else tracking_update = OPPONENT_2;
            }
        }

        /* update results */
        if(tracking_update == OPPONENT_1) {
            IRQ_LOCK(flags);
            beacon.opponent1_x = beacon.tracking_opp1_x = result_x;
            beacon.opponent1_y = beacon.tracking_opp1_y = result_y;
            beacon.opponent1_angle = local_angle;
            beacon.opponent1_dist = local_dist;
            IRQ_UNLOCK(flags);

            /* reset tracking watchdog counter */
            beacon.tracking_opp1_counts = 0;
        }
        else if(tracking_update == OPPONENT_2) {
            IRQ_LOCK(flags);
            beacon.opponent2_x = beacon.tracking_opp2_x = result_x;
            beacon.opponent2_y = beacon.tracking_opp2_y = result_y;
            beacon.opponent2_angle = local_angle;
            beacon.opponent2_dist = local_dist;
            IRQ_UNLOCK(flags);

            /* reset tracking watchdog counter */
            beacon.tracking_opp2_counts = 0;
        }

        /* traking watchdog */
        if(beacon.tracking_opp1_counts < 10)
            beacon.tracking_opp1_counts++;
        else {
            //BEACON_DEBUG("opponent 1 not there");
            beacon.opponent1_x = I2C_OPPONENT_NOT_THERE;
        }

        if(beacon.tracking_opp2_counts < 10)
            beacon.tracking_opp2_counts++;
        else {
            //BEACON_DEBUG("opponent 2 not there");
            beacon.opponent2_x = I2C_OPPONENT_NOT_THERE;
        }

        BEACON_NOTICE("opp1 (%.3ld, %.4ld, %.4ld, %.4ld) "
    #ifdef TWO_OPPONENTS
                          "opp2 (%.3ld, %.4ld, %.4ld, %.4ld) "
    #endif
    #ifdef ROBOT_2ND
                          "r2nd (%.3ld, %.4ld, %.4ld, %.4ld) "
    #endif
                          , beacon.opponent1_angle, beacon.opponent1_dist, beacon.opponent1_x, beacon.opponent1_y
    #ifdef TWO_OPPONENTS
                          , beacon.opponent2_angle, beacon.opponent2_dist, beacon.opponent2_x, beacon.opponent2_y
    #endif
    #ifdef ROBOT_2ND
                          , beacon.robot_2nd_angle, beacon.robot_2nd_dist, beacon.robot_2nd_x, beacon.robot_2nd_y
    #endif
                          );
    #endif

    }
#ifdef ROBOT_2ND
    else /* IR_SENSOR_0_DEG */
    {
        /* XXX
            UNLIKELY CASE: case opponent robots have a reflected tape on beacon sensor place, like our secondary robot.
            Solution: discard if angle is very similar to opponents angle
         */

        /* update results */
        IRQ_LOCK(flags);
        beacon.robot_2nd_x = result_x;
        beacon.robot_2nd_y = result_y;
        beacon.robot_2nd_angle = local_angle;
        beacon.robot_2nd_dist = local_dist;
        IRQ_UNLOCK(flags);

        /* final results */
        BEACON_NOTICE("opp1 (%.3ld, %.4ld, %.4ld, %.4ld) "
    #ifdef TWO_OPPONENTS
                          "opp2 (%.3ld, %.4ld, %.4ld, %.4ld) "
    #endif
    #ifdef ROBOT_2ND
                          "r2nd (%.3ld, %.4ld, %.4ld, %.4ld) "
    #endif
                          , beacon.opponent1_angle, beacon.opponent1_dist, beacon.opponent1_x, beacon.opponent1_y
    #ifdef TWO_OPPONENTS
                          , beacon.opponent2_angle, beacon.opponent2_dist, beacon.opponent2_x, beacon.opponent2_y
    #endif
    #ifdef ROBOT_2ND
                          , beacon.robot_2nd_angle, beacon.robot_2nd_dist, beacon.robot_2nd_x, beacon.robot_2nd_y
    #endif
                          );

    }
#endif

    return;

error:

    /* 0.5 second timeout */
    if (invalid_count[sensor] < 25)
        invalid_count[sensor]++;
    else {
        invalid_count[sensor] = 0;

        if(sensor == IR_SENSOR_180_DEG) {
            IRQ_LOCK(flags);
            beacon.opponent1_x = I2C_OPPONENT_NOT_THERE;
#ifdef TWO_OPPONENTS
            beacon.opponent2_x = I2C_OPPONENT_NOT_THERE;
#endif
            IRQ_UNLOCK(flags);

            //BEACON_NOTICE("opponent/s not there");
        }
#ifdef ROBOT_2ND
        else {
            IRQ_LOCK(flags);
            beacon.robot_2nd_x = I2C_OPPONENT_NOT_THERE;
            IRQ_UNLOCK(flags);

            //BEACON_NOTICE("robot_2nd not there");
        }
#endif
    }

}

/* beacon calculus event */
void beacon_calc(void *dummy)
{
    if (mainboard.flags & DO_BEACON){
        sensor_calc(IR_SENSOR_180_DEG);
    }
}






