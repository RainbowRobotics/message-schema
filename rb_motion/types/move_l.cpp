#define P_NAME  "MOVE_L"

#include "move_l.h"
#include "message.h"

move_l::move_l()
{
    ;
}

move_l::~move_l(){
    ;
}

int move_l::Init(VectorCd c_sta, VectorCd c_tar, double vel_pos, double acc_pos, double vel_ori, double acc_ori){
    double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;

    move_l_redun_ori = rb_math::get_REDUN_1x1(c_sta);
    move_l_redun_tar = rb_math::get_REDUN_1x1(c_tar);
    {
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(vel_ori, acc_ori, fabs(move_l_redun_tar - move_l_redun_ori));
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }

    move_l_pos_ori = rb_math::get_P_3x1(c_sta);
    move_l_pos_tar = rb_math::get_P_3x1(c_tar);
    {
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(vel_pos, acc_pos, (move_l_pos_tar - move_l_pos_ori).norm());
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }
    
    move_l_rot_ori = rb_math::get_R_3x3(c_sta);
    move_l_rot_tar = rb_math::get_R_3x3(c_tar);
    Eigen::Vector3d rotation_vect = rb_math::R_to_RCV(move_l_rot_ori.transpose() * move_l_rot_tar);
    double rotation_dist = rotation_vect.norm();
    if(rotation_dist < 1e-6)    rotation_dist = 1e-6;
    {
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(vel_ori, acc_ori, rotation_dist * MATH_R2D);
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }
    time_zone[0] = t1_sum;
    time_zone[1] = t2_sum;
    time_zone[2] = t3_sum;
    time_total = time_zone[0] + time_zone[1] + time_zone[2];

    move_l_redun_vel = (move_l_redun_tar - move_l_redun_ori) / (time_zone[0] + time_zone[1]);
    move_l_redun_acc = move_l_redun_vel / time_zone[0];

    move_l_pos_vel = (move_l_pos_tar - move_l_pos_ori) / (time_zone[0] + time_zone[1]);
    move_l_pos_acc = move_l_pos_vel / time_zone[0];

    move_l_rot_dist = rotation_dist * MATH_R2D;
    move_l_rot_vec = rotation_vect / rotation_dist;
    move_l_rot_vel = move_l_rot_dist / (time_zone[0] + time_zone[1]);
    move_l_rot_acc = move_l_rot_vel / time_zone[0];

    is_FirstLoop = true;
    timer = 0;
    return MSG_OK;
}

void move_l::Update_Timer(double dt){
    timer += dt;
}

double move_l::Get_Timer(){
    return timer;
}

move_l_control_ret move_l::Control(double _time){
    if(_time < 0.){
        _time = timer;
    }
    
    move_l_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;

    Eigen::Vector3d pos_ret = move_l_pos_ori;
    double rot_angle = 0.;
    double redun_val = 0.;

    if(_time < time_zone[0]){
        pos_ret = move_l_pos_ori + 0.5 * move_l_pos_acc * _time * _time;
        rot_angle = 0 + 0.5 * move_l_rot_acc * _time * _time;
        redun_val = move_l_redun_ori + 0.5 * move_l_redun_acc * _time * _time;
    }else if(_time < (time_zone[0] + time_zone[1])){
        pos_ret = move_l_pos_ori + 0.5 * move_l_pos_acc * time_zone[0] * time_zone[0]
                    + move_l_pos_vel * (_time - time_zone[0]);
        rot_angle = 0 + 0.5 * move_l_rot_acc * time_zone[0] * time_zone[0]
                    + move_l_rot_vel * (_time - time_zone[0]);
        redun_val = move_l_redun_ori + 0.5 * move_l_redun_acc * time_zone[0] * time_zone[0]
                    + move_l_redun_vel * (_time - time_zone[0]);
    }else if(_time < (time_zone[0] + time_zone[1] + time_zone[2])){
        pos_ret = move_l_pos_tar - 0.5 * move_l_pos_acc * (time_total - _time) * (time_total - _time);
        rot_angle = move_l_rot_dist - 0.5 * move_l_rot_acc * (time_total - _time) * (time_total - _time);
        redun_val = move_l_redun_tar - 0.5 * move_l_redun_acc * (time_total - _time) * (time_total - _time);
    }else{
        ret.is_finished = true;
        pos_ret = move_l_pos_tar;
        rot_angle = move_l_rot_dist;
        redun_val = move_l_redun_tar;
    }

    Eigen::Vector3d rot_Euler = rb_math::R_to_RPY(move_l_rot_ori * rb_math::RCV_to_R(move_l_rot_vec * rot_angle * MATH_D2R));

    ret.L_output.block(0, 0, 3, 1) = pos_ret;
    ret.L_output.block(3, 0, 3, 1) = rot_Euler;
    #if NO_OF_JOINT == 7//7DOF
        ret.L_output(6) = redun_val;
    #endif
    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;

    return ret;
}