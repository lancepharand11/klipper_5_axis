// Trapezoidal velocity movement queue
// Author: Lance Pharand
// Modified from original version of trapq.c by Kevin O'Connor <kevin@koconnor.net>
#include <math.h>   // sqrt
#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h"
#include "trapq.h"

// Allocate a new 'move' object
struct move *
move_alloc(void)
{
    struct move *m = malloc(sizeof(*m));
    memset(m, 0, sizeof(*m));
    return m;
}

// Linear Axes: Return the distance moved given a time in a move
inline double
move_get_distance(struct move *m, double move_time)
{
    return (m->start_v + m->half_accel * move_time) * move_time;
}

// Rotational Axes: Return the distance moved given a time in a move
inline double
move_rot_get_distance(struct move *m, double move_time)
{
    return (m->start_rot_v + m->half_rot_accel * move_time) * move_time;
}

// Return the XYZUW coordinates given a time in a move
inline struct coord
move_get_coord(struct move *m, double move_time)
{
    double move_dist = move_get_distance(m, move_time);
    double rot_move_dist = move_rot_get_distance(m, move_time);

    return (struct coord) {
        // Linear components:
        .x = m->start_pos.x + m->axes_r.x * move_dist,
        .y = m->start_pos.y + m->axes_r.y * move_dist,
        .z = m->start_pos.z + m->axes_r.z * move_dist,

        // Rotational components:
        .u = m->start_pos.u + m->axes_r.u * rot_move_dist,
        .w = m->start_pos.w + m->axes_r.w * rot_move_dist,
    };
}

#define NEVER_TIME 9999999999999999.9

// Allocate a new 'trapq' object
struct trapq * __visible
trapq_alloc(void)
{
    struct trapq *tq = malloc(sizeof(*tq));
    memset(tq, 0, sizeof(*tq));
    list_init(&tq->moves);
    list_init(&tq->history);
    struct move *head_sentinel = move_alloc();
    struct move *tail_sentinel = move_alloc();
    tail_sentinel->print_time = tail_sentinel->move_t = NEVER_TIME;
    list_add_head(&head_sentinel->node, &tq->moves);
    list_add_tail(&tail_sentinel->node, &tq->moves);
    return tq;
}

// Free memory associated with a 'trapq' object
void __visible
trapq_free(struct trapq *tq)
{
    while (!list_empty(&tq->moves)) {
        struct move *m = list_first_entry(&tq->moves, struct move, node);
        list_del(&m->node);
        free(m);
    }
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        list_del(&m->node);
        free(m);
    }
    free(tq);
}

// Update the list sentinels
void
trapq_check_sentinels(struct trapq *tq)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    if (tail_sentinel->print_time)
        // Already up to date
        return;
    struct move *m = list_prev_entry(tail_sentinel, node);
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    if (m == head_sentinel) {
        // No moves at all on this list
        tail_sentinel->print_time = NEVER_TIME;
        return;
    }
    tail_sentinel->print_time = m->print_time + m->move_t;
    tail_sentinel->start_pos = move_get_coord(m, m->move_t);
}

#define MAX_NULL_MOVE 1.0

// Add a move to the trapezoid velocity queue
void
trapq_add_move(struct trapq *tq, struct move *m)
{
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    struct move *prev = list_prev_entry(tail_sentinel, node);
    if (prev->print_time + prev->move_t < m->print_time) {
        // Add a null move to fill time gap
        struct move *null_move = move_alloc();
        null_move->start_pos = m->start_pos;
        if (!prev->print_time && m->print_time > MAX_NULL_MOVE)
            // Limit the first null move to improve numerical stability
            null_move->print_time = m->print_time - MAX_NULL_MOVE;
        else
            null_move->print_time = prev->print_time + prev->move_t;
        null_move->move_t = m->print_time - null_move->print_time;
        list_add_before(&null_move->node, &tail_sentinel->node);
    }
    list_add_before(&m->node, &tail_sentinel->node);
    tail_sentinel->print_time = 0.;
}

// Fill and add a move to the trapezoid velocity queue
static const double lin_to_rot_velo_scaling = 1.0;
static const double lin_to_rot_accel_scaling = 1.0;
void __visible
trapq_append(struct trapq *tq, double print_time,
             double accel_t, double cruise_t, double decel_t,
             double start_pos_x, double start_pos_y, double start_pos_z,
             double start_pos_u, double start_pos_w,
             double axes_r_x, double axes_r_y, double axes_r_z,
             double axes_r_u, double axes_r_w,
             double start_v, double cruise_v, double accel)
{
    // Add separate rotational parameters like start_rot_v, rot_cruise, rot_accel. 
    // For now, set them to zero

    struct coord start_pos = {
        .x = start_pos_x, .y = start_pos_y, .z = start_pos_z,
        .u = start_pos_u, .w = start_pos_w
    };
    struct coord axes_r = {
        .x = axes_r_x, .y = axes_r_y, .z = axes_r_z,
        .u = axes_r_u, .w = axes_r_w
    };

    // Accelerate segment
    if (accel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = accel_t;
        // Linear
        m->start_v = start_v;
        m->half_accel = 0.5 * accel;
        // Rotational
        m->start_rot_v = start_v * lin_to_rot_velo_scaling;  // apply scaling factor to linear velo and convert to rot
        m->half_rot_accel = 0.5 * accel * lin_to_rot_accel_scaling;  // apply scaling factor to linear velo and convert to rot

        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        // Update for next segment
        print_time += accel_t;
        start_pos = move_get_coord(m, accel_t);
    }

    // Cruise segment
    if (cruise_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = cruise_t;
        // Linear
        m->start_v = cruise_v;
        m->half_accel = 0.;
        // Rotational
        m->start_rot_v = cruise_v * lin_to_rot_velo_scaling;  // apply scaling factor to linear velo and convert to rot
        m->half_rot_accel = 0.;

        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);

        print_time += cruise_t;
        start_pos = move_get_coord(m, cruise_t);
    }

    // Decelerate segment
    if (decel_t) {
        struct move *m = move_alloc();
        m->print_time = print_time;
        m->move_t = decel_t;
        // Linear
        m->start_v = cruise_v;
        m->half_accel = -0.5 * accel;
        // Rotational
        m->start_rot_v = cruise_v * lin_to_rot_velo_scaling;  // apply scaling factor to linear velo and convert to rot
        m->half_rot_accel = -0.5 * accel * lin_to_rot_accel_scaling;  // apply scaling factor to linear velo and convert to rot

        m->start_pos = start_pos;
        m->axes_r = axes_r;
        trapq_add_move(tq, m);
    }
}

// Expire any moves older than `print_time` from the trapezoid velocity queue
void __visible
trapq_finalize_moves(struct trapq *tq, double print_time,
                     double clear_history_time)
{
    struct move *head_sentinel = list_first_entry(&tq->moves, struct move,node);
    struct move *tail_sentinel = list_last_entry(&tq->moves, struct move, node);
    // Move expired moves from main "moves" list to "history" list
    for (;;) {
        struct move *m = list_next_entry(head_sentinel, node);
        if (m == tail_sentinel) {
            tail_sentinel->print_time = NEVER_TIME;
            break;
        }
        if (m->print_time + m->move_t > print_time)
            break;
        list_del(&m->node);
        if (m->start_v || m->half_accel)
            list_add_head(&m->node, &tq->history);
        else
            free(m);
    }
    // Free old moves from history list
    if (list_empty(&tq->history))
        return;
    struct move *latest = list_first_entry(&tq->history, struct move, node);
    for (;;) {
        struct move *m = list_last_entry(&tq->history, struct move, node);
        if (m == latest || m->print_time + m->move_t > clear_history_time)
            break;
        list_del(&m->node);
        free(m);
    }
}

// Note a position change in the trapq history (including U, W)
void __visible
trapq_set_position(struct trapq *tq, double print_time,
                   double pos_x, double pos_y, double pos_z,
                   double pos_u, double pos_w)
{
    // Flush all moves from trapq
    trapq_finalize_moves(tq, NEVER_TIME, 0);

    // Prune any moves in the trapq history that were interrupted
    while (!list_empty(&tq->history)) {
        struct move *m = list_first_entry(&tq->history, struct move, node);
        if (m->print_time < print_time) {
            if (m->print_time + m->move_t > print_time)
                m->move_t = print_time - m->print_time;
            break;
        }
        list_del(&m->node);
        free(m);
    }

    // Add a marker to the trapq history
    struct move *m = move_alloc();
    m->print_time = print_time;
    m->start_pos.x = pos_x;
    m->start_pos.y = pos_y;
    m->start_pos.z = pos_z;
    m->start_pos.u = pos_u;
    m->start_pos.w = pos_w;
    // No velocity or acceleration for a marker move
    list_add_head(&m->node, &tq->history);
}

// Return history of movement queue
int __visible
trapq_extract_old(struct trapq *tq, struct pull_move *p, int max,
                  double start_time, double end_time)
{
    int res = 0;
    struct move *m;
    list_for_each_entry(m, &tq->history, node) {
        if (start_time >= m->print_time + m->move_t || res >= max)
            break;
        if (end_time <= m->print_time)
            continue;

        p->print_time = m->print_time;
        p->move_t = m->move_t;
        p->start_v = m->start_v;
        p->accel = 2. * m->half_accel;
        p->start_rot_v = m->start_rot_v;
        p->rot_accel = 2. * m->half_rot_accel;

        p->start_x = m->start_pos.x;
        p->start_y = m->start_pos.y;
        p->start_z = m->start_pos.z;
        p->start_u = m->start_pos.u;
        p->start_w = m->start_pos.w;

        p->x_r = m->axes_r.x;
        p->y_r = m->axes_r.y;
        p->z_r = m->axes_r.z;
        p->u_r = m->axes_r.u;
        p->w_r = m->axes_r.w;

        p++;
        res++;
    }
    return res;
}
