#define P_NAME  "MOVE_SPEED_J"

#include "move_speed_j.h"
#include "message.h"
#include "iostream"

move_speed_j::move_speed_j()
{
    ;
}

move_speed_j::~move_speed_j(){
    ;
}

int move_speed_j::Init(VectorJd j_sta, VectorJd j_limit_up, VectorJd j_limit_dw, VectorJd j_vel_limit, VectorJd j_acc_limit){
    speed_j_ori = j_sta;
    speed_j_vel = VectorJd::Zero(NO_OF_JOINT, 1);

    speed_j_vel_target = VectorJd::Zero(NO_OF_JOINT, 1);
    speed_j_accumed_delta = VectorJd::Zero(NO_OF_JOINT, 1);

    speed_j_accum_limi_to_up = j_limit_up - speed_j_ori;
    speed_j_accum_limi_to_dw = j_limit_dw - speed_j_ori;

    speed_j_vel_limit = j_vel_limit;
    speed_j_acc_limit = j_acc_limit;

    return MSG_OK;
}

int move_speed_j::Set_Taget_Velocity(VectorJd deg_per_sec){
    for(int i = 0; i < NO_OF_JOINT; ++i){
        speed_j_vel_target(i) = rb_math::saturation_L_and_U(deg_per_sec(i), -speed_j_vel_limit(i), +speed_j_vel_limit(i));
    }
    return MSG_OK;
}

move_speed_j_control_ret move_speed_j::Control(double _delta_time){
    move_speed_j_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;

    for(int i = 0; i < NO_OF_JOINT; ++i){
        double last_vel = speed_j_vel(i);
        double last_accum = speed_j_accumed_delta(i);
        double targ_vel = speed_j_vel_target(i) * _delta_time / RT_PERIOD_SEC;
        double acc_gap = speed_j_acc_limit(i) * RT_PERIOD_SEC;

        double new_vel = rb_math::saturation_L_and_U(targ_vel, (last_vel - acc_gap), (last_vel + acc_gap));

        double tar_accum_j = last_accum + (new_vel * RT_PERIOD_SEC);
        double new_accum_j = rb_math::saturation_L_and_U(tar_accum_j, speed_j_accum_limi_to_dw(i), speed_j_accum_limi_to_up(i));

        speed_j_accumed_delta(i) = new_accum_j;
        speed_j_vel(i) = (new_accum_j - last_accum) / RT_PERIOD_SEC;
    }

    ret.J_output = speed_j_ori + speed_j_accumed_delta;

    return ret;
}