// Morgan SCARA kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
// SCARA Kinematics: 2024-2026 Quentin Harley <quentin.harley@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // move_get_coord
#include <math.h>      // sqrt
#include <stddef.h>    // offsetof
#include <stdlib.h>    // malloc
#include <string.h>    // memset

struct morgan_stepper
{
    struct stepper_kinematics sk;
    double l1, l2;
    double l1_squared, l2_squared;
    double column_x, column_y;
    double d_limit;
};

static inline struct morgan_stepper*
morgan_stepper_from_sk(struct stepper_kinematics* sk)
{
    return (struct morgan_stepper*) ((char*) sk -
                                     offsetof(struct morgan_stepper, sk));
}

static inline double morgan_stepper_calc_psi(struct morgan_stepper* ms,
                                             struct coord c)
{
    double x = c.x - ms->column_x;
    double y = c.y - ms->column_y;
    double r_squared = x * x + y * y;
    double d =
        (r_squared - ms->l1_squared - ms->l2_squared) / (2.0 * ms->l1 * ms->l2);
    d = fmin(fmax(d, -ms->d_limit), ms->d_limit);
    return copysign(atan2(sqrt(1.0 - d * d), d), -1.0);
}

static double
morgan_scara_stepper_a_calc_position(struct stepper_kinematics* sk,
                                     struct move* m, double move_time)
{
    struct morgan_stepper* ms = morgan_stepper_from_sk(sk);
    struct coord c = move_get_coord(m, move_time);
    double x = c.x - ms->column_x;
    double y = c.y - ms->column_y;
    double psi = morgan_stepper_calc_psi(ms, c);
    return atan2(y, x) - atan2(ms->l2 * sin(psi), ms->l1 + ms->l2 * cos(psi));
}

static double
morgan_scara_stepper_b_calc_position(struct stepper_kinematics* sk,
                                     struct move* m, double move_time)
{
    struct morgan_stepper* ms = morgan_stepper_from_sk(sk);
    struct coord c = move_get_coord(m, move_time);
    double theta = morgan_scara_stepper_a_calc_position(sk, m, move_time);
    double psi = morgan_stepper_calc_psi(ms, c);
    return theta + psi;
}

struct stepper_kinematics*
morgan_scara_stepper_alloc(char type, double l1, double l2, double column_x,
                           double column_y, double d_limit)
{
    struct morgan_stepper* ms = malloc(sizeof(*ms));
    memset(ms, 0, sizeof(*ms));
    ms->l1 = l1;
    ms->l2 = l2;
    ms->l1_squared = l1 * l1;
    ms->l2_squared = l2 * l2;
    ms->column_x = column_x;
    ms->column_y = column_y;
    ms->d_limit = d_limit;

    if (type == 'a')
        ms->sk.calc_position_cb = morgan_scara_stepper_a_calc_position;
    else if (type == 'b')
        ms->sk.calc_position_cb = morgan_scara_stepper_b_calc_position;

    ms->sk.active_flags = AF_X | AF_Y;
    return &ms->sk;
}
