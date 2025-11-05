#ifndef MOVE_L_H
#define MOVE_L_H

#include "rmath.h"
struct move_l_control_ret{
    VectorCd        L_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;
};

class move_l
{
public:
    move_l();
    ~move_l();

    int Init(VectorCd c_sta, VectorCd c_tar, double vel_pos, double acc_pos, double vel_ori, double acc_ori);

    void Update_Timer(double dt);
    double Get_Timer();

    move_l_control_ret Control(double _time);

private:
    double          timer;
    double          time_zone[3];
    double          time_total;

    bool            is_FirstLoop;

    Eigen::Vector3d        move_l_pos_ori;
    Eigen::Vector3d        move_l_pos_tar;
    Eigen::Vector3d        move_l_pos_vel;
    Eigen::Vector3d        move_l_pos_acc;

    Eigen::Matrix3d         move_l_rot_ori;
    Eigen::Matrix3d         move_l_rot_tar;
    Eigen::Vector3d         move_l_rot_vec;
    double                  move_l_rot_dist;
    double                  move_l_rot_vel;
    double                  move_l_rot_acc;

    double                  move_l_redun_ori;
    double                  move_l_redun_tar;
    double                  move_l_redun_vel;
    double                  move_l_redun_acc;
};
#endif // MOVE_L_H
