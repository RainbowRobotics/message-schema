#ifndef MOVE_SPEED_J_H
#define MOVE_SPEED_J_H

#include "rmath.h"
struct move_speed_j_control_ret{
    VectorJd        J_output;
    unsigned char   is_thereErr;
    bool            is_finished;
};

class move_speed_j
{
public:
    move_speed_j();
    ~move_speed_j();

    int Init(VectorJd j_sta, VectorJd j_limit_up, VectorJd j_limit_dw, VectorJd j_vel_limit, VectorJd j_acc_limit);
    int Set_Taget_Velocity(VectorJd deg_per_sec);

    move_speed_j_control_ret Control(double _delta_time);

private:
    VectorJd        speed_j_ori;
    VectorJd        speed_j_accumed_delta;
    VectorJd        speed_j_vel;

    VectorJd        speed_j_vel_target;

    VectorJd        speed_j_accum_limi_to_up;
    VectorJd        speed_j_accum_limi_to_dw;

    VectorJd        speed_j_vel_limit;
    VectorJd        speed_j_acc_limit;
    
};
#endif // MOVE_SPEED_J_H
