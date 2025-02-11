#ifndef ITERSOLVE_H
#define ITERSOLVE_H

#include <stdint.h> // int32_t

// Active flags for five axes: X, Y, Z, U, W
enum {
    AF_X = 1 << 0,
    AF_Y = 1 << 1,
    AF_Z = 1 << 2,
    AF_U = 1 << 3,
    AF_W = 1 << 4,
};

struct stepper_kinematics;
struct move;

// Callback signature for calculating position of a single stepper
typedef double (*sk_calc_callback)(struct stepper_kinematics *sk,
                                   struct move *m,
                                   double move_time);

// Optional post-processing callback
typedef void (*sk_post_callback)(struct stepper_kinematics *sk);

// Main structure describing one stepper's kinematics
struct stepper_kinematics {
    double step_dist;          // Distance per step
    double commanded_pos;      // Last commanded position for this stepper
    struct stepcompress *sc;   // Step-compress context

    double last_flush_time;    // Last time we flushed steps
    double last_move_time;     // End time of last move
    struct trapq *tq;          // The trapezoid queue (list of moves)
    int active_flags;          // Which axis this stepper is controlling
    double gen_steps_pre_active, gen_steps_post_active;

    sk_calc_callback calc_position_cb;
    sk_post_callback post_cb;
};

// Public interface
int32_t itersolve_generate_steps(struct stepper_kinematics *sk,
                                 double flush_time);
double itersolve_check_active(struct stepper_kinematics *sk,
                              double flush_time);
int32_t itersolve_is_active_axis(struct stepper_kinematics *sk, char axis);

void itersolve_set_trapq(struct stepper_kinematics *sk, struct trapq *tq);
void itersolve_set_stepcompress(struct stepper_kinematics *sk,
                                struct stepcompress *sc, double step_dist);

/*
 * Calculates this stepper's position at the midpoint of a dummy move
 * that starts at (x, y, z, u, w). The actual stepper might only use
 * one coordinate (e.g. x or u), but we allow all five for consistency
 * with 5-axis data structures.
 */
double itersolve_calc_position_from_coord(struct stepper_kinematics *sk,
                                          double x, double y, double z,
                                          double u, double w);

/*
 * Update the commanded position of this stepper based on
 * a 5D position (x, y, z, u, w). Typically you only need
 * one of these coordinates for a specific stepper. 
 */
void itersolve_set_position(struct stepper_kinematics *sk,
                            double x, double y, double z,
                            double u, double w);

/*
 * Retrieve the last commanded position of this stepper (in user units,
 * such as mm or radians).
 */
double itersolve_get_commanded_pos(struct stepper_kinematics *sk);

#endif // itersolve.h
