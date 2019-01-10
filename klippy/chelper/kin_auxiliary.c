// Auxiliary axis kinematics stepper pulse time generation
//
// Copyright (C) 2019  Trevor Jones <trevorjones141@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // move_get_coord
#include "pyhelper.h" // errorf

static double
auxiliary_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

struct stepper_kinematics * __visible
auxiliary_stepper_alloc(void)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    sk->calc_position = auxiliary_stepper_calc_position;
    return sk;
}
