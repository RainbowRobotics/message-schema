#ifndef MOVE_CIR_H
#define MOVE_CIR_H

#include "rmath.h"
struct move_cir_control_ret{
    VectorCd        L_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;
};

class move_cir
{
public:
    move_cir();
    ~move_cir();

    int Init(VectorCd c_sta, Eigen::Vector3d cir_cen, Eigen::Vector3d cir_axis, double cir_angle
    , double target_redun_ang
    , double vel_pos, double acc_pos, double vel_ori, double acc_ori
    , int rot_mode
    , int radi_mode, double raid_para);

    void Update_Timer(double dt);
    double Get_Timer();

    move_cir_control_ret Control(double _time);

private:
    double          timer;
    double          time_zone[3];
    double          time_total;

    bool            is_FirstLoop;

    VectorCd                move_c_spin_Ori;
    Eigen::Vector3d         move_c_spin_center;
    Eigen::Vector3d         move_c_spin_axis;    


    Eigen::Vector3d         move_c_spin_r_vec;
    double                  move_c_spin_dist;
    double                  move_c_spin_vel;
    double                  move_c_spin_acc;

    

    int                     move_c_spin_rot_mode;//0 = radial, 1 = constant;
    int                     move_c_spin_r_mode;
    double                  move_c_spin_r_para;

    double                  move_c_redun_ori;
    double                  move_c_redun_tar;
    double                  move_c_redun_vel;
    double                  move_c_redun_acc;
};
#endif // MOVE_CIR_H
