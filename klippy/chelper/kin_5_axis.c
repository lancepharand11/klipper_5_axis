// Author: Lance Pharand
// Modified from original version of kin_cartesian.c by Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.


#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

static double
rot_stepper_u_calc_position(struct stepper_kinematics *sk, struct move *m,
                            double move_time)
{
    return move_get_coord(m, move_time).u;
}

static double
rot_stepper_w_calc_position(struct stepper_kinematics *sk, struct move *m,
                            double move_time)
{
    return move_get_coord(m, move_time).w;
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x') {
        sk->calc_position_cb = cart_stepper_x_calc_position;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = cart_stepper_y_calc_position;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = cart_stepper_z_calc_position;
        sk->active_flags = AF_Z;
    }
    return sk;
}

struct stepper_kinematics * __visible
rot_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));

    if (axis == 'u') 
    {
        sk->calc_position_cb = rot_stepper_u_calc_position;
        sk->active_flags = AF_U;
    } 
    else if (axis == 'w') 
    {
        sk->calc_position_cb = rot_stepper_w_calc_position;
        sk->active_flags = AF_W;
    } 

    return sk;
}
