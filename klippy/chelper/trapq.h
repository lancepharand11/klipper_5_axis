#ifndef TRAPQ_H
#define TRAPQ_H

#include "list.h" // list_node

// Modified coord to handle 5 axes: x, y, z, u, w
struct coord {
    union {
        struct {
            double x, y, z, u, w;
        };
        double axis[5];
    };
};

struct move {
    double print_time, move_t;
    // Linear velocity/acceleration
    double start_v, half_accel;
    // Rotational velocity/acceleration
    double start_rot_v, half_rot_accel;

    // Starting position for this move (x,y,z,u,w)
    struct coord start_pos;
    // Direction vector (x,y,z,u,w), scaled by 1/total_distance
    struct coord axes_r;

    // Linked-list node in the queue
    struct list_node node;
};

struct trapq {
    struct list_head moves;  // Linked list of scheduled moves
    struct list_head history;  // Linked list of completed/expired moves
};

// Modified pull_move to include rotational velocity/accel
struct pull_move {
    double print_time, move_t;
    double start_v, accel;
    double start_rot_v, rot_accel;

    // Linear starts
    double start_x, start_y, start_z;
    // Rotational starts
    double start_u, start_w;

    // Linear direction vectors
    double x_r, y_r, z_r;
    // Rotational direction vectors
    // TODO: Figure out how these are set
    double u_r, w_r;
};


struct move *move_alloc(void);
double move_get_distance(struct move *m, double move_time);
double move_rot_get_distance(struct move *m, double move_time);
struct coord move_get_coord(struct move *m, double move_time);

struct trapq *trapq_alloc(void);
void trapq_free(struct trapq *tq);
void trapq_check_sentinels(struct trapq *tq);
void trapq_add_move(struct trapq *tq, struct move *m);
void trapq_append(struct trapq *tq, double print_time,
                  double accel_t, double cruise_t, double decel_t,
                  double start_pos_x, double start_pos_y, double start_pos_z,
                  double start_pos_u, double start_pos_w,  // For U/W
                  double axes_r_x, double axes_r_y, double axes_r_z,
                  double axes_r_u, double axes_r_w,  // For U/W
                  double start_v, double cruise_v, double accel);  // For U/W

void trapq_finalize_moves(struct trapq *tq, double print_time,
                          double clear_history_time);

void trapq_set_position(struct trapq *tq, double print_time,
                        double pos_x, double pos_y, double pos_z,
                        double pos_u, double pos_w);

int trapq_extract_old(struct trapq *tq, struct pull_move *p, int max,
                      double start_time, double end_time);

#endif // TRAPQ_H
