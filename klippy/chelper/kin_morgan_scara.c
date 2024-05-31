// Morgan SCARA kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
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
    double L1, L2;
    double L1_squared, L2_squared;
    double column_x, column_y;
    double D_limit;
};

// This function calculates the position of
static double
morgan_scara_stepper_a_calc_position(struct stepper_kinematics* sk,
                                     struct move* m, double move_time)
{
    struct morgan_stepper* ms = container_of(sk, struct morgan_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    // SCARA zero point is at the center of the column
    // Morgan zero point is inferred by the column offset position as defined by
    // the user
    double x = c.x - ms->column_x;
    double y = c.y - ms->column_y;

    // Calculate the distance to the point
    double r_squared = x * x + y * y;

    // Calculate psi
    double D =
        (r_squared - ms->L1_squared - ms->L2_squared) / (2.0 * ms->L1 * ms->L2);
    D = fmin(fmax(D, -ms->D_limit),
             ms->D_limit); // Clamp D to prevent sqrt from returning NaN and
                           // collisions

    // Psi in Morgan kinematics: Distal arm is always on the right side of the
    // proximal arm
    double psi = atan2(sqrt(1 - D * D), D);
    psi = copysign(psi, -1.0); // Negate psi if positive

    // Calculate theta
    double theta =
        atan2(y, x) - atan2(ms->L2 * sin(psi), ms->L1 + ms->L2 * cos(psi));

    return theta;
}

static double
morgan_scara_stepper_b_calc_position(struct stepper_kinematics* sk,
                                     struct move* m, double move_time)
{
    struct morgan_stepper* ms = container_of(sk, struct morgan_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    // SCARA zero point is at the center of the column
    // Morgan zero point is inferred by the column offset position as defined by
    // the user
    double x = c.x - ms->column_x;
    double y = c.y - ms->column_y;

    // Calculate the distance to the point
    double r_squared = x * x + y * y;

    // Calculate psi
    double D =
        (r_squared - ms->L1_squared - ms->L2_squared) / (2.0 * ms->L1 * ms->L2);
    D = fmin(fmax(D, -ms->D_limit),
             ms->D_limit); // Clamp D to prevent sqrt from returning NaN and
                           // collisions

    // Psi in Morgan kinematics: Distal arm is always on the right side of the
    // proximal arm
    double psi = atan2(sqrt(1 - D * D), D);
    psi = copysign(psi, -1.0); // Negate psi if positive

    // Calculate theta
    double theta =
        atan2(y, x) - atan2(ms->L2 * sin(psi), ms->L1 + ms->L2 * cos(psi));

    // Return psi, as a sum with theta
    return theta + psi; // Morgan kinematics: Distal arm is driven from the base
}

struct stepper_kinematics* __visible
morgan_scara_stepper_alloc(char type, double L1, double L2, double column_x,
                           double column_y, double D_limit)
{
    struct morgan_stepper* ms = malloc(sizeof(*ms));
    memset(ms, 0, sizeof(*ms));
    ms->L1 = L1;
    ms->L2 = L2;
    ms->L1_squared = L1 * L1;
    ms->L2_squared = L2 * L2;
    ms->column_x = column_x;
    ms->column_y = column_y;
    ms->D_limit = D_limit;

    if (type == 'a')
    {
        ms->sk.calc_position_cb = morgan_scara_stepper_a_calc_position;
    }
    else if (type == 'b')
    {
        ms->sk.calc_position_cb = morgan_scara_stepper_b_calc_position;
    }

    // Set the active flags for X and Y moves
    ms->sk.active_flags = AF_X | AF_Y;

    return &ms->sk;
}