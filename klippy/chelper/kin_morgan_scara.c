// Morgan SCARA kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h> // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h" // move_get_coord

struct morgan_stepper {
    struct stepper_kinematics sk;
    double L1, L2;
    double L1_squared, L2_squared;
};

// This function calculates the position of 
static double
morgan_scara_stepper_a_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct morgan_stepper *ms = container_of(sk, struct morgan_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    // Calculate the distance to the point
    float r_squared = c.x * c.x + c.y * c.y;

    // Calculate theta2
    float D = (r_squared - ms->L1_squared - ms->L2_squared) / (2 * ms->L1 * ms->L2);
    float theta2 = atan2(sqrt(1 - D * D), D);

    // Calculate theta1
    return atan2(y, x) - atan2(ms->L2 * sin(theta2), ms->L1 + ms->L2 * cos(theta2));
}

static double
morgan_scara_stepper_b_calc_position(struct stepper_kinematics *sk, struct move *m
                            , double move_time)
{
    struct morgan_stepper *ms = container_of(sk, struct morgan_stepper, sk);
    struct coord c = move_get_coord(m, move_time);

    // Calculate the distance to the point
    float r_squared = c.x * c.x + c.y * c.y;

   // Calculate theta2
    float D = (r_squared - ms->L1_squared - ms->L2_squared) / (2 * ms->L1 * ms->L2);
    
    return atan2(sqrt(1 - D * D), D);

}


struct stepper_kinematics * __visible
morgan_scara_stepper_alloc(char type, double L1, double L2)
{
    struct morgan_stepper *ms = malloc(sizeof(*ms));
    memset(ms, 0, sizeof(*ds));
    ms->L1 = L1;
    ms->L2 = L2;
    ms->L1_squared = L1 * L1;
    ms->L2_squared = L2 * L2;
    if (type == 'a') {
        ms->sk.calc_position_cb = morgan_scara_stepper_a_calc_position;
    } else if (type == 'b') {
        ms->sk.calc_position_cb = morgan_scara_stepper_b_calc_position;
    }    

    // Set the active flags for X and Y moves
    ms->sk.active_flags = AF_X | AF_Y;

    return &ms->sk;
}
