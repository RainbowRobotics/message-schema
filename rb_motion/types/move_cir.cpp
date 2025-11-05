#define P_NAME  "MOVE_L"

#include "move_cir.h"
#include "message.h"
move_cir::move_cir()
{
    ;
}

move_cir::~move_cir(){
    ;
}

int move_cir::Init(VectorCd c_sta, Eigen::Vector3d cir_cen, Eigen::Vector3d cir_axis, double cir_angle
    , double target_redun_ang
    , double vel_pos, double acc_pos, double vel_ori, double acc_ori
    , int rot_mode
    , int radi_mode, double raid_para){

    double r_vec_size = (rb_math::get_P_3x1(c_sta) - cir_cen).norm();
    if(r_vec_size < 1e-3) return 0;
    if(cir_axis.norm() < 1e-3) return 0;

    double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;
    {
        double temp_spin_vel = vel_pos / r_vec_size * MATH_R2D;
        double temp_spin_acc = acc_pos / r_vec_size * MATH_R2D;
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(temp_spin_vel, temp_spin_acc, fabs(cir_angle));
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }
    if(rot_mode == 0){
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(vel_ori, acc_ori, fabs(cir_angle));
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }
    {
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(vel_ori, acc_ori, fabs(move_c_redun_tar - move_c_redun_ori));
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }

    time_zone[0] = t1_sum;
    time_zone[1] = t2_sum;
    time_zone[2] = t3_sum;
    time_total = time_zone[0] + time_zone[1] + time_zone[2];

    move_c_spin_rot_mode = rot_mode;
    move_c_spin_r_mode = radi_mode;
    move_c_spin_r_para = raid_para;
    
    move_c_spin_Ori = c_sta;
    move_c_spin_center = cir_cen;
    move_c_spin_axis = cir_axis;
    move_c_spin_axis /= move_c_spin_axis.norm();
    move_c_spin_r_vec = rb_math::get_P_3x1(c_sta) - move_c_spin_center;

    move_c_spin_dist = cir_angle;
    move_c_spin_vel  = move_c_spin_dist / (time_zone[0] + time_zone[1]);
    move_c_spin_acc  = move_c_spin_vel / time_zone[0];

    move_c_redun_ori = rb_math::get_REDUN_1x1(c_sta);
    move_c_redun_tar = target_redun_ang;
    move_c_redun_vel = (move_c_redun_tar - move_c_redun_ori) / (time_zone[0] + time_zone[1]);
    move_c_redun_acc = move_c_redun_vel / time_zone[0];

    is_FirstLoop = true;
    timer = 0;
    return MSG_OK;
}

void move_cir::Update_Timer(double dt){
    timer += dt;
}

double move_cir::Get_Timer(){
    return timer;
}

move_cir_control_ret move_cir::Control(double _time){
    if(_time < 0.){
        _time = timer;
    }
    
    move_cir_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;

    double rot_angle = 0.;
    double redun_val = 0.;

    if(_time < time_zone[0]){
        rot_angle = 0 + 0.5 * move_c_spin_acc * _time * _time;
        redun_val = move_c_redun_ori + 0.5 * move_c_redun_acc * _time * _time;
    }else if(_time < (time_zone[0] + time_zone[1])){
        rot_angle = 0 + 0.5 * move_c_spin_acc * time_zone[0] * time_zone[0]
                    + move_c_spin_vel * (_time - time_zone[0]);
        redun_val = move_c_redun_ori + 0.5 * move_c_redun_acc * time_zone[0] * time_zone[0]
                    + move_c_redun_vel * (_time - time_zone[0]);
    }else if(_time < (time_zone[0] + time_zone[1] + time_zone[2])){
        rot_angle = move_c_spin_dist - 0.5 * move_c_spin_acc * (time_total - _time) * (time_total - _time);
        redun_val = move_c_redun_tar - 0.5 * move_c_redun_acc * (time_total - _time) * (time_total - _time);
    }else{
        ret.is_finished = true;
        rot_angle = move_c_spin_dist;
        redun_val = move_c_redun_tar;
    }

    double temp_r_muliplier = 1.;
    if(move_c_spin_r_mode == 1){
        temp_r_muliplier = pow(move_c_spin_r_para, (rot_angle / 360.));
    }else if(move_c_spin_r_mode == 2){
        double new_r_size = move_c_spin_r_vec.size() + move_c_spin_r_para * (rot_angle / 360.);
        temp_r_muliplier = new_r_size / move_c_spin_r_vec.size();
    }
    if(temp_r_muliplier < 1e-3){
        ret.is_finished = true;
    }

    Eigen::Matrix3d roational_matrix = rb_math::RCV_to_R( (move_c_spin_axis * (rot_angle * MATH_D2R)) );
    Eigen::Vector3d pos_ret = move_c_spin_center + (roational_matrix * move_c_spin_r_vec * temp_r_muliplier);
    Eigen::Matrix3d rot_Mat = roational_matrix * rb_math::get_R_3x3(move_c_spin_Ori);
    if(move_c_spin_rot_mode == 1){
        rot_Mat = rb_math::get_R_3x3(move_c_spin_Ori);
    }else if(move_c_spin_r_para == 2){
        ;
    }

    ret.L_output.block(0, 0, 3, 1) = pos_ret;
    ret.L_output.block(3, 0, 3, 1) = rb_math::R_to_RPY(rot_Mat);
    #if NO_OF_JOINT == 7//7DOF
        ret.L_output(6) = redun_val;
    #endif
    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;

    return ret;
}