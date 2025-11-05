#ifndef MOVE_J_H
#define MOVE_J_H

#include "rmath.h"
struct move_j_control_ret{
    VectorJd        J_output;
    unsigned char   is_thereErr;
    bool            is_finished;
};

class move_j
{
public:
    move_j();
    ~move_j();

    int Init(VectorJd j_sta, VectorJd j_tar, VectorJd j_vel, VectorJd j_acc);

    void Update_Timer(double dt);
    double Get_Timer();

    move_j_control_ret Control(double _time);

private:
    double          timer;
    double          time_zone[3];
    double          time_total;

    VectorJd        move_j_vel;
    VectorJd        move_j_acc;
    VectorJd        move_j_tar;
    VectorJd        move_j_ori;
};
#endif // MOVE_J_H
