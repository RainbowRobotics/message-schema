#ifndef MOVE_SPEED_L_H
#define MOVE_SPEED_L_H

#include "rmath.h"
struct move_speed_l_control_ret{
    VectorCd        L_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;
};

class move_speed_l
{
public:
    move_speed_l();
    ~move_speed_l();

    int Init(VectorCd c_sta, double acc_pos, double acc_rot);
    int Set_Taget_Velocity(VectorCd carte_speed_per_sec);

    move_speed_l_control_ret Control(bool is_break, double break_alpha);
    void Update_Internal_Scaler(double alpha);

private:
    bool            is_FirstLoop;
    bool            is_BreakCall;

    double          internal_scaler;

    Eigen::Vector3d prev_Pos;
    Eigen::Matrix3d prev_Rmat;
    double          prev_Aa;

    double          speed_l_acc_pos;//mm/s^2
    double          speed_l_acc_rcv;//rad/s^2
    double          speed_l_acc_aaa;//rad/s^2
    
    Eigen::Vector3d speed_l_pos_vel_tar;// mm/s
    Eigen::Vector3d speed_l_rcv_vel_tar;// rad/s
    double          speed_l_aaa_vel_tar;// rad/s

    Eigen::Vector3d speed_l_pos_vel;// mm/s
    Eigen::Vector3d speed_l_rcv_vel;// rad/s
    double          speed_l_aaa_vel;
};
#endif // MOVE_SPEED_L_H
