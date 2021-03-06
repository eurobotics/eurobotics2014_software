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
 *  Olivier MATZ <zer0@droids-corp.org> 
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands_traj.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <pwm_servo.h>
#include <clock_time.h>
#include <encoders_dspic.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
#include <f64.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>

#include "main.h"
#include "cs.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat_avoid.h"
#include "strat.h"
#include "../common/i2c_commands.h"
#include "bt_protocol.h"

#ifdef HOST_VERSION
#define COMPILE_COMMANDS_TRAJ
#define COMPILE_COMMANDS_TRAJ_OPTIONALS
#endif

/**********************************************************/
/* Traj_Speeds for trajectory_manager */

/* this structure is filled when cmd_traj_speed is parsed successfully */
struct cmd_traj_speed_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float s;
};

/* function called when cmd_traj_speed is parsed successfully */
static void cmd_traj_speed_parsed(void *parsed_result, void *data)
{
    struct cmd_traj_speed_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("angle")))
    {
        trajectory_set_speed(&mainboard.traj, mainboard.traj.d_speed, res->s);
    }
    else if (!strcmp_P(res->arg1, PSTR("distance")))
    {
        trajectory_set_speed(&mainboard.traj, res->s, mainboard.traj.a_speed);
    }
    /* else it is a "show" */

    printf_P(PSTR("angle %2.2f, distance %2.2f\r\n"),
             mainboard.traj.a_speed,
             mainboard.traj.d_speed);
}

prog_char str_traj_speed_arg0[] = "traj_speed";
parse_pgm_token_string_t cmd_traj_speed_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg0, str_traj_speed_arg0);
prog_char str_traj_speed_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_speed_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_arg1);
parse_pgm_token_num_t cmd_traj_speed_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_speed_result, s, FLOAT);

prog_char help_traj_speed[] = "Set traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed = {
    .f = cmd_traj_speed_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_traj_speed,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_speed_arg0,
        (prog_void *) & cmd_traj_speed_arg1,
        (prog_void *) & cmd_traj_speed_s,
        NULL,
    },
};

/* show */

prog_char str_traj_speed_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_speed_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_speed_result, arg1, str_traj_speed_show_arg);

prog_char help_traj_speed_show[] = "Show traj_speed values for trajectory manager";
parse_pgm_inst_t cmd_traj_speed_show = {
    .f = cmd_traj_speed_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_traj_speed_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_speed_arg0,
        (prog_void *) & cmd_traj_speed_show_arg,
        NULL,
    },
};

/**********************************************************/
/* Traj_Accs for trajectory_manager */

/* this structure is filled when cmd_traj_acc is parsed successfully */
struct cmd_traj_acc_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float s;
};

/* function called when cmd_traj_acc is parsed successfully */
static void cmd_traj_acc_parsed(void *parsed_result, void *data)
{
    struct cmd_traj_acc_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("angle")))
    {
        trajectory_set_acc(&mainboard.traj, mainboard.traj.d_acc, res->s);
    }
    else if (!strcmp_P(res->arg1, PSTR("distance")))
    {
        trajectory_set_acc(&mainboard.traj, res->s, mainboard.traj.a_acc);
    }
    /* else it is a "show" */

    printf_P(PSTR("angle %2.2f, distance %2.2f\r\n"),
             mainboard.traj.a_acc,
             mainboard.traj.d_acc);
}

prog_char str_traj_acc_arg0[] = "traj_acc";
parse_pgm_token_string_t cmd_traj_acc_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg0, str_traj_acc_arg0);
prog_char str_traj_acc_arg1[] = "angle#distance";
parse_pgm_token_string_t cmd_traj_acc_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_arg1);
parse_pgm_token_num_t cmd_traj_acc_s = TOKEN_NUM_INITIALIZER(struct cmd_traj_acc_result, s, FLOAT);

prog_char help_traj_acc[] = "Set traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc = {
    .f = cmd_traj_acc_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_traj_acc,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_acc_arg0,
        (prog_void *) & cmd_traj_acc_arg1,
        (prog_void *) & cmd_traj_acc_s,
        NULL,
    },
};

/* show */

prog_char str_traj_acc_show_arg[] = "show";
parse_pgm_token_string_t cmd_traj_acc_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_traj_acc_result, arg1, str_traj_acc_show_arg);

prog_char help_traj_acc_show[] = "Show traj_acc values for trajectory manager";
parse_pgm_inst_t cmd_traj_acc_show = {
    .f = cmd_traj_acc_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_traj_acc_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_traj_acc_arg0,
        (prog_void *) & cmd_traj_acc_show_arg,
        NULL,
    },
};


#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS /*-------------------------------------*/

/**********************************************************/
/* circle coef configuration */

/* this structure is filled when cmd_circle_coef is parsed successfully */
struct cmd_circle_coef_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float circle_coef;
};

/* function called when cmd_circle_coef is parsed successfully */
static void cmd_circle_coef_parsed(void *parsed_result, void *data)
{
    struct cmd_circle_coef_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        trajectory_set_circle_coef(&mainboard.traj, res->circle_coef);
    }

    printf_P(PSTR("circle_coef set %2.2f\r\n"), mainboard.traj.circle_coef);
}

prog_char str_circle_coef_arg0[] = "circle_coef";
parse_pgm_token_string_t cmd_circle_coef_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg0, str_circle_coef_arg0);
prog_char str_circle_coef_arg1[] = "set";
parse_pgm_token_string_t cmd_circle_coef_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_arg1);
parse_pgm_token_num_t cmd_circle_coef_val = TOKEN_NUM_INITIALIZER(struct cmd_circle_coef_result, circle_coef, FLOAT);

prog_char help_circle_coef[] = "Set circle coef";
parse_pgm_inst_t cmd_circle_coef = {
    .f = cmd_circle_coef_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_circle_coef,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_circle_coef_arg0,
        (prog_void *) & cmd_circle_coef_arg1,
        (prog_void *) & cmd_circle_coef_val,
        NULL,
    },
};

/* show */

prog_char str_circle_coef_show_arg[] = "show";
parse_pgm_token_string_t cmd_circle_coef_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_circle_coef_result, arg1, str_circle_coef_show_arg);

prog_char help_circle_coef_show[] = "Show circle coef";
parse_pgm_inst_t cmd_circle_coef_show = {
    .f = cmd_circle_coef_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_circle_coef_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_circle_coef_arg0,
        (prog_void *) & cmd_circle_coef_show_arg,
        NULL,
    },
};

/**********************************************************/
/* trajectory window configuration */

/* this structure is filled when cmd_trajectory is parsed successfully */
struct cmd_trajectory_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float d_win;
    float a_win;
    float a_start;
};

/* function called when cmd_trajectory is parsed successfully */
static void cmd_trajectory_parsed(void * parsed_result, void * data)
{
    struct cmd_trajectory_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        trajectory_set_windows(&mainboard.traj, res->d_win,
                               res->a_win, res->a_start);
    }

    printf_P(PSTR("trajectory %2.2f %2.2f %2.2f\r\n"), mainboard.traj.d_win,
             DEG(mainboard.traj.a_win_rad), DEG(mainboard.traj.a_start_rad));
}

prog_char str_trajectory_arg0[] = "trajectory";
parse_pgm_token_string_t cmd_trajectory_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg0, str_trajectory_arg0);
prog_char str_trajectory_arg1[] = "set";
parse_pgm_token_string_t cmd_trajectory_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_arg1);
parse_pgm_token_num_t cmd_trajectory_d = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, d_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_a = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_win, FLOAT);
parse_pgm_token_num_t cmd_trajectory_as = TOKEN_NUM_INITIALIZER(struct cmd_trajectory_result, a_start, FLOAT);

prog_char help_trajectory[] = "Set trajectory windows (distance, angle, angle_start)";
parse_pgm_inst_t cmd_trajectory = {
    .f = cmd_trajectory_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_trajectory,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_trajectory_arg0,
        (prog_void *) & cmd_trajectory_arg1,
        (prog_void *) & cmd_trajectory_d,
        (prog_void *) & cmd_trajectory_a,
        (prog_void *) & cmd_trajectory_as,
        NULL,
    },
};

/* show */

prog_char str_trajectory_show_arg[] = "show";
parse_pgm_token_string_t cmd_trajectory_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_trajectory_result, arg1, str_trajectory_show_arg);

prog_char help_trajectory_show[] = "Show trajectory window configuration";
parse_pgm_inst_t cmd_trajectory_show = {
    .f = cmd_trajectory_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_trajectory_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_trajectory_arg0,
        (prog_void *) & cmd_trajectory_show_arg,
        NULL,
    },
};



/**********************************************************/
/* rs_gains configuration */

/* this structure is filled when cmd_rs_gains is parsed successfully */
struct cmd_rs_gains_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float ed;
};

/* function called when cmd_rs_gains is parsed successfully */
static void cmd_rs_gains_parsed(void * parsed_result, void * data)
{
#ifdef HOST_VERSION
    printf("not implemented\n");
#else
    struct cmd_rs_gains_result * res = parsed_result;
    double cl = 0.0, cr = 0.0, ed = 0.0;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        ed = res->ed;
        cl = (2.0 / (ed + 1.0));
        cr = (2.0 / ((1.0 / ed) + 1.0));

        /* increase gain to decrease dist, increase left and it will turn more left */
        rs_set_left_ext_encoder(&mainboard.rs, encoders_dspic_get_value,
                                LEFT_ENCODER, IMP_COEF * cl);
        rs_set_right_ext_encoder(&mainboard.rs, encoders_dspic_get_value,
                                 RIGHT_ENCODER, IMP_COEF * -cr);
    }

    printf_P(PSTR("rs_gains set ed = %f, cr = %f, cl = %f\r\n"), ed, cr, cl);
#endif
}

prog_char str_rs_gains_arg0[] = "rs_gains";
parse_pgm_token_string_t cmd_rs_gains_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg0, str_rs_gains_arg0);
prog_char str_rs_gains_arg1[] = "set";
parse_pgm_token_string_t cmd_rs_gains_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_arg1);
parse_pgm_token_num_t cmd_rs_gains_ed = TOKEN_NUM_INITIALIZER(struct cmd_rs_gains_result, ed, FLOAT);

prog_char help_rs_gains[] = "Set rs_gains (left, right)";
parse_pgm_inst_t cmd_rs_gains = {
    .f = cmd_rs_gains_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_rs_gains,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_gains_arg0,
        (prog_void *) & cmd_rs_gains_arg1,
        (prog_void *) & cmd_rs_gains_ed,
        NULL,
    },
};

/* show */
prog_char str_rs_gains_show_arg[] = "show";
parse_pgm_token_string_t cmd_rs_gains_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_rs_gains_result, arg1, str_rs_gains_show_arg);

prog_char help_rs_gains_show[] = "Show rs_gains";
parse_pgm_inst_t cmd_rs_gains_show = {
    .f = cmd_rs_gains_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_rs_gains_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_gains_arg0,
        (prog_void *) & cmd_rs_gains_show_arg,
        NULL,
    },
};


/**********************************************************/
/* track configuration */

/* this structure is filled when cmd_track is parsed successfully */
struct cmd_track_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float val;
};

/* function called when cmd_track is parsed successfully */
static void cmd_track_parsed(void * parsed_result, void * data)
{
    struct cmd_track_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set_physical_params(&mainboard.pos, res->val, DIST_IMP_MM);
    }
    printf_P(PSTR("track set %f\r\n"), mainboard.pos.phys.track_mm);
}

prog_char str_track_arg0[] = "track";
parse_pgm_token_string_t cmd_track_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg0, str_track_arg0);
prog_char str_track_arg1[] = "set";
parse_pgm_token_string_t cmd_track_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_arg1);
parse_pgm_token_num_t cmd_track_val = TOKEN_NUM_INITIALIZER(struct cmd_track_result, val, FLOAT);

prog_char help_track[] = "Set track in mm";
parse_pgm_inst_t cmd_track = {
    .f = cmd_track_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_track,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_track_arg0,
        (prog_void *) & cmd_track_arg1,
        (prog_void *) & cmd_track_val,
        NULL,
    },
};

/* show */

prog_char str_track_show_arg[] = "show";
parse_pgm_token_string_t cmd_track_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_track_result, arg1, str_track_show_arg);

prog_char help_track_show[] = "Show track";
parse_pgm_inst_t cmd_track_show = {
    .f = cmd_track_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_track_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_track_arg0,
        (prog_void *) & cmd_track_show_arg,
        NULL,
    },
};



/**********************************************************/
/* centrifugal configuration */

/* this structure is filled when cmd_centrifugal is parsed successfully */
struct cmd_centrifugal_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    float val;
};

/* function called when cmd_centrifugal is parsed successfully */
static void cmd_centrifugal_parsed(void * parsed_result, void * data)
{
    struct cmd_centrifugal_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set_centrifugal_coef(&mainboard.pos, res->val);
    }
    printf_P(PSTR("centrifugal set %f\r\n"), mainboard.pos.centrifugal_coef);
}

prog_char str_centrifugal_arg0[] = "centrifugal";
parse_pgm_token_string_t cmd_centrifugal_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg0, str_centrifugal_arg0);
prog_char str_centrifugal_arg1[] = "set";
parse_pgm_token_string_t cmd_centrifugal_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_arg1);
parse_pgm_token_num_t cmd_centrifugal_val = TOKEN_NUM_INITIALIZER(struct cmd_centrifugal_result, val, FLOAT);

prog_char help_centrifugal[] = "Set centrifugal coef";
parse_pgm_inst_t cmd_centrifugal = {
    .f = cmd_centrifugal_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_centrifugal,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_centrifugal_arg0,
        (prog_void *) & cmd_centrifugal_arg1,
        (prog_void *) & cmd_centrifugal_val,
        NULL,
    },
};

/* show */

prog_char str_centrifugal_show_arg[] = "show";
parse_pgm_token_string_t cmd_centrifugal_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_centrifugal_result, arg1, str_centrifugal_show_arg);

prog_char help_centrifugal_show[] = "Show centrifugal";
parse_pgm_inst_t cmd_centrifugal_show = {
    .f = cmd_centrifugal_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_centrifugal_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_centrifugal_arg0,
        (prog_void *) & cmd_centrifugal_show_arg,
        NULL,
    },
};


/**********************************************************/
/* Pt_Lists for testing traj */

#define PT_LIST_SIZE 10
static struct xy_point pt_list[PT_LIST_SIZE];
static uint16_t pt_list_len = 0;

/* this structure is filled when cmd_pt_list is parsed successfully */
struct cmd_pt_list_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint16_t arg2;
    int16_t arg3;
    int16_t arg4;
};

/* function called when cmd_pt_list is parsed successfully */
static void cmd_pt_list_parsed(void * parsed_result, void * data)
{
    struct cmd_pt_list_result * res = parsed_result;
    uint8_t i, why = 0;

    if (!strcmp_P(res->arg1, PSTR("append")))
    {
        res->arg2 = pt_list_len;
    }

    if (!strcmp_P(res->arg1, PSTR("insert")) ||
            !strcmp_P(res->arg1, PSTR("append")))
    {
        if (res->arg2 > pt_list_len)
        {
            printf_P(PSTR("Index too large\r\n"));
            return;
        }
        if (pt_list_len >= PT_LIST_SIZE)
        {
            printf_P(PSTR("List is too large\r\n"));
            return;
        }
        memmove(&pt_list[res->arg2 + 1], &pt_list[res->arg2],
                PT_LIST_SIZE - 1 - res->arg2);
        pt_list[res->arg2].x = res->arg3;
        pt_list[res->arg2].y = res->arg4;
        pt_list_len++;
    }
    else if (!strcmp_P(res->arg1, PSTR("del")))
    {
        if (pt_list_len <= 0)
        {
            printf_P(PSTR("Error: list empty\r\n"));
            return;
        }
        if (res->arg2 > pt_list_len)
        {
            printf_P(PSTR("Index too large\r\n"));
            return;
        }
        memmove(&pt_list[res->arg2], &pt_list[res->arg2 + 1],
                (PT_LIST_SIZE - 1 - res->arg2) * sizeof (struct xy_point));
        pt_list_len--;
    }
    else if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        pt_list_len = 0;
    }

    /* else it is a "show" or a "start" */
    if (pt_list_len == 0)
    {
        printf_P(PSTR("List empty\r\n"));
        return;
    }
    for (i = 0; i < pt_list_len; i++)
    {
        printf_P(PSTR("%d: x=%d y=%d\r\n"), i, pt_list[i].x, pt_list[i].y);
        if (!strcmp_P(res->arg1, PSTR("start")))
        {
            trajectory_goto_xy_abs(&mainboard.traj, pt_list[i].x, pt_list[i].y);
            why = wait_traj_end(TRAJ_FLAGS_NO_NEAR_NO_TIMER); /* all */
        }
        else if (!strcmp_P(res->arg1, PSTR("avoid_start")))
        {
            while (1)
            {
                why = goto_and_avoid(pt_list[i].x, pt_list[i].y, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR_NO_TIMER);
                printf("next point\r\n");
                if (why != END_OBSTACLE)
                    break;
            }
        }
        if (why & (~(END_TRAJ | END_NEAR)))
            trajectory_stop(&mainboard.traj);
    }
}

prog_char str_pt_list_arg0[] = "pt_list";
parse_pgm_token_string_t cmd_pt_list_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg0, str_pt_list_arg0);
prog_char str_pt_list_arg1[] = "insert";
parse_pgm_token_string_t cmd_pt_list_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1);
parse_pgm_token_num_t cmd_pt_list_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg2, UINT16);
parse_pgm_token_num_t cmd_pt_list_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg3, INT16);
parse_pgm_token_num_t cmd_pt_list_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_pt_list_result, arg4, INT16);

prog_char help_pt_list[] = "Insert point in pt_list (idx,x,y)";
parse_pgm_inst_t cmd_pt_list = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_pt_list,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_arg1,
        (prog_void *) & cmd_pt_list_arg2,
        (prog_void *) & cmd_pt_list_arg3,
        (prog_void *) & cmd_pt_list_arg4,
        NULL,
    },
};

/* append */

prog_char str_pt_list_arg1_append[] = "append";
parse_pgm_token_string_t cmd_pt_list_arg1_append = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_arg1_append);

prog_char help_pt_list_append[] = "Append point in pt_list (x,y)";
parse_pgm_inst_t cmd_pt_list_append = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_pt_list_append,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_arg1_append,
        (prog_void *) & cmd_pt_list_arg3,
        (prog_void *) & cmd_pt_list_arg4,
        NULL,
    },
};

/* del */

prog_char str_pt_list_del_arg[] = "del";
parse_pgm_token_string_t cmd_pt_list_del_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_del_arg);

prog_char help_pt_list_del[] = "Del or insert point in pt_list (num)";
parse_pgm_inst_t cmd_pt_list_del = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_pt_list_del,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_del_arg,
        (prog_void *) & cmd_pt_list_arg2,
        NULL,
    },
};
/* show */

prog_char str_pt_list_show_arg[] = "show#reset#start#avoid_start";
parse_pgm_token_string_t cmd_pt_list_show_arg = TOKEN_STRING_INITIALIZER(struct cmd_pt_list_result, arg1, str_pt_list_show_arg);

prog_char help_pt_list_show[] = "Show, start or reset pt_list";
parse_pgm_inst_t cmd_pt_list_show = {
    .f = cmd_pt_list_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_pt_list_show,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_pt_list_arg0,
        (prog_void *) & cmd_pt_list_show_arg,
        NULL,
    },
};

#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS ------------------------------------*/


/**********************************************************/
/* Goto function */

/* this structure is filled when cmd_goto is parsed successfully */
struct cmd_goto_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
    int32_t arg4;
};

/* function called when cmd_goto is parsed successfully */
static void cmd_goto_parsed(void * parsed_result, void * data)
{
    struct cmd_goto_result * res = parsed_result;
    uint8_t err;
    microseconds t1, t2;

    interrupt_traj_reset();

    if (!strcmp_P(res->arg1, PSTR("a_rel")))
    {
        trajectory_a_rel(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("d_rel")))
    {
        trajectory_d_rel(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_abs")))
    {
        trajectory_a_abs(&mainboard.traj, res->arg2);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_to_xy")))
    {
        trajectory_turnto_xy(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("a_behind_xy")))
    {
        trajectory_turnto_xy_behind(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_rel")))
    {
        trajectory_goto_xy_rel(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs")))
    {
        trajectory_goto_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid")))
    {
        err = goto_and_avoid(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_fw")))
    {
        err = goto_and_avoid_forward(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();
    }
    else if (!strcmp_P(res->arg1, PSTR("avoid_bw")))
    {
        err = goto_and_avoid_backward(res->arg2, res->arg3, TRAJ_FLAGS_STD, TRAJ_FLAGS_NO_NEAR);
        if (err != END_TRAJ && err != END_NEAR)
            strat_hardstop();
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_fow")))
    {
        trajectory_goto_forward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("xy_abs_back")))
    {
        trajectory_goto_backward_xy_abs(&mainboard.traj, res->arg2, res->arg3);
    }
    else if (!strcmp_P(res->arg1, PSTR("da_rel")))
    {
        trajectory_d_a_rel(&mainboard.traj, res->arg2, res->arg3);
    }
    t1 = time_get_us2();
    while ((err = test_traj_end(TRAJ_FLAGS_NO_NEAR)) == 0)
    {
        t2 = time_get_us2();
        if (t2 - t1 > 20000)
        {
            dump_cs_debug("angle", &mainboard.angle.cs);
            dump_cs_debug("distance", &mainboard.distance.cs);
            t1 = t2;
        }
    }
    if (err != END_TRAJ && err != END_NEAR)
        strat_hardstop();
    printf_P(PSTR("returned %s\r\n"), get_err(err));
}

prog_char str_goto_arg0[] = "goto";
parse_pgm_token_string_t cmd_goto_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg0, str_goto_arg0);
prog_char str_goto_arg1_a[] = "d_rel#a_rel#a_abs";
parse_pgm_token_string_t cmd_goto_arg1_a = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_a);
parse_pgm_token_num_t cmd_goto_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg2, INT32);

/* 1 params */
prog_char help_goto1[] = "Change orientation of the mainboard";
parse_pgm_inst_t cmd_goto1 = {
    .f = cmd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_goto1,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_goto_arg0,
        (prog_void *) & cmd_goto_arg1_a,
        (prog_void *) & cmd_goto_arg2,
        NULL,
    },
};

prog_char str_goto_arg1_b[] = "xy_rel#xy_abs#xy_abs_fow#xy_abs_back#da_rel#a_to_xy#avoid#avoid_fw#avoid_bw#a_behind_xy";
parse_pgm_token_string_t cmd_goto_arg1_b = TOKEN_STRING_INITIALIZER(struct cmd_goto_result, arg1, str_goto_arg1_b);
parse_pgm_token_num_t cmd_goto_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_goto_result, arg3, INT32);

/* 2 params */
prog_char help_goto2[] = "Go to a (x,y) or (d,a) position";
parse_pgm_inst_t cmd_goto2 = {
    .f = cmd_goto_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_goto2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_goto_arg0,
        (prog_void *) & cmd_goto_arg1_b,
        (prog_void *) & cmd_goto_arg2,
        (prog_void *) & cmd_goto_arg3,
        NULL,
    },
};

/**********************************************************/
/* Position tests */

/* this structure is filled when cmd_position is parsed successfully */
struct cmd_position_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    int32_t arg2;
    int32_t arg3;
    int32_t arg4;
};

#define AUTOPOS_SPEED_FAST 	500
#define ROBOT_DIS2_WALL         (int16_t)(119)

void auto_position(void)
{
    uint8_t err;
    uint16_t old_spdd, old_spda;

    /* save & set speeds */
    interrupt_traj_reset();
    strat_get_speed(&old_spdd, &old_spda);
    strat_set_speed(AUTOPOS_SPEED_FAST, AUTOPOS_SPEED_FAST);

    /* goto blocking to x axis */
    trajectory_d_rel(&mainboard.traj, -300);
    err = wait_traj_end(END_INTR | END_TRAJ | END_BLOCKING);
    if (err == END_INTR)
        goto intr;
    wait_ms(100);

    /* set x and angle */
    strat_reset_pos(COLOR_X(ROBOT_CENTER_TO_BACK), DO_NOT_SET_POS, COLOR_A_ABS(0));

    /* prepare to y axis */
    trajectory_d_rel(&mainboard.traj, 90);
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;

    trajectory_a_rel(&mainboard.traj, COLOR_A_REL(90));
    err = wait_traj_end(END_INTR | END_TRAJ);
    if (err == END_INTR)
        goto intr;

    /* goto blocking to y axis */
    trajectory_d_rel(&mainboard.traj, -800);
    err = wait_traj_end(END_INTR | END_TRAJ | END_BLOCKING);
    if (err == END_INTR)
        goto intr;
    wait_ms(100);

    /* set y */
    strat_reset_pos(DO_NOT_SET_POS, COLOR_Y(ROBOT_CENTER_TO_BACK), 90);

    /* restore speeds */
    strat_set_speed(old_spdd, old_spda);
    return;

intr:
    strat_hardstop();
    strat_set_speed(old_spdd, old_spda);
}

/* function called when cmd_position is parsed successfully */
static void cmd_position_parsed(void * parsed_result, void * data)
{
    struct cmd_position_result * res = parsed_result;

    /* display raw position values */
    if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        position_set(&mainboard.pos, 0, 0, 0);
    }
    else if (!strcmp_P(res->arg1, PSTR("set")))
    {
        position_set(&mainboard.pos, res->arg2, res->arg3, res->arg4);
    }
    else if (!strcmp_P(res->arg1, PSTR("autoset_yellow")))
    {
        mainboard.our_color = I2C_COLOR_YELLOW;
        auto_position();
    }
    else if (!strcmp_P(res->arg1, PSTR("autoset_red")))
    {
        mainboard.our_color = I2C_COLOR_RED;
        auto_position();
    }

    /* else it's just a "show" */
    printf_P(PSTR("x=%.2f y=%.2f a=%.2f\r\n"),
             position_get_x_double(&mainboard.pos),
             position_get_y_double(&mainboard.pos),
             DEG(position_get_a_rad_double(&mainboard.pos)));
}

prog_char str_position_arg0[] = "position";
parse_pgm_token_string_t cmd_position_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg0, str_position_arg0);
prog_char str_position_arg1[] = "show#reset#autoset_yellow#autoset_red";
parse_pgm_token_string_t cmd_position_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1);

prog_char help_position[] = "Show/reset (x,y,a) position";
parse_pgm_inst_t cmd_position = {
    .f = cmd_position_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_position,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_position_arg0,
        (prog_void *) & cmd_position_arg1,
        NULL,
    },
};


prog_char str_position_arg1_set[] = "set";
parse_pgm_token_string_t cmd_position_arg1_set = TOKEN_STRING_INITIALIZER(struct cmd_position_result, arg1, str_position_arg1_set);
parse_pgm_token_num_t cmd_position_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg2, INT32);
parse_pgm_token_num_t cmd_position_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg3, INT32);
parse_pgm_token_num_t cmd_position_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_position_result, arg4, INT32);

prog_char help_position_set[] = "Set (x,y,a) position";
parse_pgm_inst_t cmd_position_set = {
    .f = cmd_position_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_position_set,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_position_arg0,
        (prog_void *) & cmd_position_arg1_set,
        (prog_void *) & cmd_position_arg2,
        (prog_void *) & cmd_position_arg3,
        (prog_void *) & cmd_position_arg4,
        NULL,
    },
};


/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_infos is parsed successfully */
struct cmd_strat_infos_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_strat_infos is parsed successfully */
static void cmd_strat_infos_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_infos_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("reset")))
    {
        strat_reset_infos();
    }

    strat_infos.dump_enabled = 1;
    strat_dump_infos(__FUNCTION__);
}

prog_char str_strat_infos_arg0[] = "strat_infos";
parse_pgm_token_string_t cmd_strat_infos_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg0, str_strat_infos_arg0);
prog_char str_strat_infos_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_strat_infos_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_infos_result, arg1, str_strat_infos_arg1);

prog_char help_strat_infos[] = "reset/show strat infos";
parse_pgm_inst_t cmd_strat_infos = {
    .f = cmd_strat_infos_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_strat_infos,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_infos_arg0,
        (prog_void *) & cmd_strat_infos_arg1,
        NULL,
    },
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf is parsed successfully */
struct cmd_strat_conf_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_strat_conf is parsed successfully */
static void cmd_strat_conf_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("base")))
    {

        /* flags */
        strat_infos.conf.flags = 0;
#ifndef HOMOLOGATION

#endif
    }

    strat_infos.dump_enabled = 1;
    strat_dump_conf();
}

prog_char str_strat_conf_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg0, str_strat_conf_arg0);
prog_char str_strat_conf_arg1[] = "show#base";
parse_pgm_token_string_t cmd_strat_conf_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf_result, arg1, str_strat_conf_arg1);

prog_char help_strat_conf[] = "configure specific strat for a match";
parse_pgm_inst_t cmd_strat_conf = {
    .f = cmd_strat_conf_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_strat_conf,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf_arg0,
        (prog_void *) & cmd_strat_conf_arg1,
        NULL,
    },
};

/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf2 is parsed successfully */
struct cmd_strat_conf2_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    fixed_string_t arg2;
};

/* function called when cmd_strat_conf2 is parsed successfully */
static void cmd_strat_conf2_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf2_result *res = parsed_result;
    uint8_t on, bit = 0;

    if (!strcmp_P(res->arg2, PSTR("on")))
        on = 1;
    else
        on = 0;

    /* contruct here the bit mask */

    if (on)
        strat_infos.conf.flags |= bit;
    else
        strat_infos.conf.flags &= (~bit);

    strat_infos.dump_enabled = 1;
    strat_dump_conf();
}

prog_char str_strat_conf2_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg0, str_strat_conf2_arg0);
prog_char str_strat_conf2_arg1[] = "opp_tracking";
parse_pgm_token_string_t cmd_strat_conf2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg1, str_strat_conf2_arg1);
prog_char str_strat_conf2_arg2[] = "on#off";
parse_pgm_token_string_t cmd_strat_conf2_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf2_result, arg2, str_strat_conf2_arg2);


prog_char help_strat_conf2[] = "enable/disable strat option";
parse_pgm_inst_t cmd_strat_conf2 = {
    .f = cmd_strat_conf2_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_strat_conf2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf2_arg0,
        (prog_void *) & cmd_strat_conf2_arg1,
        (prog_void *) & cmd_strat_conf2_arg2,
        NULL,
    },
};


/**********************************************************/
/* strat configuration */

/* this structure is filled when cmd_strat_conf3 is parsed successfully */
struct cmd_strat_conf3_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint8_t arg2;
};

/* function called when cmd_strat_conf3 is parsed successfully */
static void cmd_strat_conf3_parsed(void *parsed_result, void *data)
{
    struct cmd_strat_conf3_result *res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("something")))
    {
        /* set value of a strat parameter */
			printf ("not implemented");
    }

    strat_infos.dump_enabled = 1;
    strat_dump_conf();
}

prog_char str_strat_conf3_arg0[] = "strat_conf";
parse_pgm_token_string_t cmd_strat_conf3_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg0, str_strat_conf3_arg0);
prog_char str_strat_conf3_arg1[] = "something";
parse_pgm_token_string_t cmd_strat_conf3_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_strat_conf3_result, arg1, str_strat_conf3_arg1);
parse_pgm_token_num_t cmd_strat_conf3_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_strat_conf3_result, arg2, UINT16);

prog_char help_strat_conf3[] = "set strat param value";
parse_pgm_inst_t cmd_strat_conf3 = {
    .f = cmd_strat_conf3_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_strat_conf3,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_strat_conf3_arg0,
        (prog_void *) & cmd_strat_conf3_arg1,
        (prog_void *) & cmd_strat_conf3_arg2,
        NULL,
    },
};


/**********************************************************/
/* Subtraj 1 */

/* this structure is filled when cmd_subtraj1 is parsed successfully */
struct cmd_subtraj1_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    //int32_t arg2;
    //int32_t arg3;
};

/* function called when cmd_subtraj1 is parsed successfully */
static void cmd_subtraj1_parsed(void *parsed_result, void *data)
{
    struct cmd_subtraj1_result *res = parsed_result;
    uint8_t err = 0;


    if (strcmp_P(res->arg1, PSTR("homolog")) == 0) {
        strat_homologation();
    }
    else if (strcmp_P(res->arg1, PSTR("begin")) == 0) {
        //err = strat_begin();
        err = strat_begin_alcabot();
    }
    else if (strcmp_P(res->arg1, PSTR("strat_smart")) == 0) {
        strat_smart();
    }

    printf_P(PSTR("subtraj returned %s\r\n"), get_err(err));
    trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj1_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj1_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj1_result, arg0, str_subtraj1_arg0);
prog_char str_subtraj1_arg1[] = "homolog#begin#strat_smart";
parse_pgm_token_string_t cmd_subtraj1_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj1_result, arg1, str_subtraj1_arg1);
//parse_pgm_token_num_t cmd_subtraj1_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj1_result, arg2, INT32);
//parse_pgm_token_num_t cmd_subtraj1_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj1_result, arg3, INT32);
prog_char help_subtraj1[] = "Test sub-trajectories"; //" (a,b,c: specific params)";
parse_pgm_inst_t cmd_subtraj1 = {
    .f = cmd_subtraj1_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_subtraj1,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_subtraj1_arg0,
        (prog_void *) & cmd_subtraj1_arg1,
        //(prog_void *)&cmd_subtraj1_arg2,
        //(prog_void *)&cmd_subtraj1_arg3,
        NULL,
    },
};


/**********************************************************/
/* Subtraj 2 */

/* this structure is filled when cmd_subtraj2 is parsed successfully */
struct cmd_subtraj2_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    uint8_t arg2;
    //	int32_t arg2;
    //	int32_t arg3;
    //	int32_t arg4;
    //	int32_t arg5;
};

/* function called when cmd_subtraj2 is parsed successfully */
static void cmd_subtraj2_parsed(void *parsed_result, void *data)
{
    struct cmd_subtraj2_result *res = parsed_result;
    uint8_t err = 0;
    uint8_t zone_num = ZONES_MAX;

    if (!strcmp_P(res->arg1, PSTR("tree")))
        zone_num = ZONE_TREE_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("fire")))
        zone_num = ZONE_FIRE_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("heart"))) {
        zone_num = ZONE_HEART_1 + res->arg2 - 1;
	}

    else if (!strcmp_P(res->arg1, PSTR("torch")))
        zone_num = ZONE_TORCH_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("mtorch")))
        zone_num = ZONE_M_TORCH_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("mamooth")))
            zone_num = ZONE_MAMOOTH_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("basket")))
        zone_num = ZONE_BASKET_1 + res->arg2 - 1;

    else if (!strcmp_P(res->arg1, PSTR("fresco")))
        zone_num = ZONE_FRESCO;


	printf_P(PSTR("%d %s\r\n"),zone_num,numzone2name[zone_num]);
    if (zone_num < ZONES_MAX) {
        err = strat_goto_zone(zone_num);
        printf_P(PSTR("goto returned %s\r\n"), get_err(err));    

        err = strat_work_on_zone(zone_num);
        printf_P(PSTR("work returned %s\r\n"), get_err(err));
    }

    trajectory_hardstop(&mainboard.traj);
}

prog_char str_subtraj2_arg0[] = "subtraj";
parse_pgm_token_string_t cmd_subtraj2_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj2_result, arg0, str_subtraj2_arg0);
prog_char str_subtraj2_arg1[] = "tree#fire#heart#torch#mtorch#mamooth#basket#fresco";
parse_pgm_token_string_t cmd_subtraj2_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_subtraj2_result, arg1, str_subtraj2_arg1);
parse_pgm_token_num_t cmd_subtraj2_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg2, UINT8);
//parse_pgm_token_num_t cmd_subtraj2_arg3 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg3, INT32);
//parse_pgm_token_num_t cmd_subtraj2_arg4 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg4, INT32);
//parse_pgm_token_num_t cmd_subtraj2_arg5 = TOKEN_NUM_INITIALIZER(struct cmd_subtraj2_result, arg5, INT32);

prog_char help_subtraj2[] = "Test sub-trajectories (a,b,c,d: specific params)";
parse_pgm_inst_t cmd_subtraj2 = {
    .f = cmd_subtraj2_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    .help_str = help_subtraj2,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_subtraj2_arg0,
        (prog_void *) & cmd_subtraj2_arg1,
		(prog_void *)&cmd_subtraj2_arg2,
        //		(prog_void *)&cmd_subtraj2_arg3,
        //		(prog_void *)&cmd_subtraj2_arg4,
        //		(prog_void *)&cmd_subtraj2_arg5,
        NULL,
    },
};



#if 0
/**********************************************************/
/* Goto zone */

// this structure is filled when cmd_gotozone is parsed successfully

struct cmd_gotozone_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

// function called when cmd_gotozone is parsed successfully

static void cmd_gotozone_parsed(void *parsed_result, void *data)
{
    struct cmd_gotozone_result *res = parsed_result;
    int8_t num = -1;
    uint8_t i = 0;
    uint8_t err = 0;

    for (i = 0; i < ZONES_MAX; i++)
    {
        if (strcmp(numzone2name[i], res->arg1) == 0)
        {
            num = i;
            break;
        }
    }
    if (num != -1)
    {
        err = strat_goto_zone(num);
        printf_P(PSTR("goto_zone %s\r\n"), get_err(err));
    }
    else
        printf("Error: can't find zone!\n\r");
}

prog_char str_gotozone_arg0[] = "goto_zone";
parse_pgm_token_string_t cmd_gotozone_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_gotozone_result, arg0, str_gotozone_arg0);
prog_char str_gotozone_arg1[] = "t1#t2#t3#t4#h1#h2#h3#f1#f1#f3#f4#f5#f6#tr1#tr2#tr3#tr4#mt1#mt2#b1#b2#m1#m2#fco#rd#yll";
parse_pgm_token_string_t cmd_gotozone_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_gotozone_result, arg1, str_gotozone_arg1);


prog_char help_gotozone[] = "Go to a zone of the field";
parse_pgm_inst_t cmd_gotozone = {
    .f = cmd_gotozone_parsed, // function to call
    .data = NULL, // 2nd arg of func
    .help_str = help_gotozone,
    .tokens =
    { // token list, NULL terminated
        (prog_void *) & cmd_gotozone_arg0,
        (prog_void *) & cmd_gotozone_arg1,
        NULL,
    },
};

/**********************************************************/
/* Work on a zone */

// this structure is filled when cmd_workonzone is parsed successfully

struct cmd_workonzone_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

// function called when cmd_workonzone is parsed successfully

static void cmd_workonzone_parsed(void *parsed_result, void *data)
{
    struct cmd_workonzone_result *res = parsed_result;
    int8_t num = -1;
    uint8_t i = 0;
    uint8_t err = 0;

    for (i = 0; i < ZONES_MAX; i++)
    {
        if (strcmp(numzone2name[i], res->arg1) == 0)
        {
            num = i;
            break;
        }
    }
    if (num != -1)
    {
        err = strat_work_on_zone(num);
        printf_P(PSTR("work_on_zone %s\r\n"), get_err(err));
    }
    else
        printf("Error: can't find zone!\n\r");
}

prog_char str_workonzone_arg0[] = "workon_zone";
parse_pgm_token_string_t cmd_workonzone_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_workonzone_result, arg0, str_workonzone_arg0);
//prog_char str_workonzone_arg1[] = "t1#t2#t3#t4#h1#h2#h3#f1#f1#f3#f4#f5#f6#to1#to2#to3#to4#mt1#mt2#b1#b2#m1#m2#fco#red#yellow";
parse_pgm_token_string_t cmd_workonzone_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_workonzone_result, arg1, str_gotozone_arg1);


prog_char help_workonzone[] = "Work on a zone of the field";
parse_pgm_inst_t cmd_workonzone = {
    .f = cmd_workonzone_parsed, // function to call
    .data = NULL, // 2nd arg of func
    .help_str = help_workonzone,
    .tokens =
    { // token list, NULL terminated
        (prog_void *) & cmd_workonzone_arg0,
        (prog_void *) & cmd_workonzone_arg1,
        NULL,
    },
};
#endif

