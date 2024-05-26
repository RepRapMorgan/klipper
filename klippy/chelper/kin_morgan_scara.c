// Morgan SCARA kinematics stepper pulse time generation
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2024  Quentin Harley <quentin.harley@gmail.com> (RepRapMorgan)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>   // sqrt, acos
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <stdio.h>
#include <string.h>    // memset
#include "compiler.h"  // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "trapq.h"     // move_get_coord
//#define _USE_MATH_DEFINES
//#include <cmath>

#define SQ(x) ((x) * (x))

struct morgan_stepper
{
    struct stepper_kinematics sk;
    double link_a, link_b;
    double link_a2, link_b2;
    double alpha, beta;
};

// returns the alpha angle in radians
static double
morgan_calc_alpha(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct morgan_stepper *ms = container_of(sk, struct morgan_stepper, sk);

    struct coord c = move_get_coord(m, move_time);
    ms->beta = acos((SQ(c.x) + SQ(c.y) - ms->link_a2 - ms->link_b2)
        / (2 * ms->link_a2 * ms->link_b2));
    ms->alpha = atan((c.y / c.x) - atan((ms->link_b * sin(ms->beta))
        / (ms->link_a + ms->link_b * cos(ms->beta))));

    return ms->alpha;
}

// returns the beta angle in radians
static double
morgan_calc_beta(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    struct morgan_stepper *ms = container_of(sk, struct morgan_stepper, sk);

    // *** For Faultfinding: Normally the next line is not needed ***
    ms->beta = acos((SQ(c.x) + SQ(c.y) - ms->link_a2 - ms->link_b2)
        / (2 * ms->link_a2 * ms->link_b2));
    //ms->alpha = atan((c.y / c.x) - atan((ms->link_b * sin(ms->beta))
    //    / (ms->link_a + ms->link_b * cos(ms->beta))));

    // return the beta value already calculated in morgan_calc_alpha()
    return ms->beta;
}

struct stepper_kinematics *__visible morgan_scara_stepper_alloc(
    char stepper, double link_a, double link_b)
{
    struct morgan_stepper *ms = malloc(sizeof(*ms));
    memset(ms, 0, sizeof(*ms));

    ms->link_a = link_a;
    ms->link_b = link_b;
    ms->link_a2 = SQ(link_a);
    ms->link_b2 = SQ(link_b);

    /* Ensure itersolve is called for all X and Y moves */
    ms->sk.active_flags = AF_X | AF_Y;
    
    /* Setup itersolve callback */
    switch (stepper)
    {
    case 'a':
        ms->sk.calc_position_cb = morgan_calc_alpha;
        break;
    case 'b':
        ms->sk.calc_position_cb = morgan_calc_beta;
        break;
    default:
        break;
    }
    
    return &ms->sk;
}
