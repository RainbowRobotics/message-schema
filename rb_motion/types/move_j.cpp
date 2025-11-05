#define P_NAME  "MOVE_J"

#include "move_j.h"
#include "message.h"

move_j::move_j()
{
    ;
}

move_j::~move_j(){
    ;
}

int move_j::Init(VectorJd j_sta, VectorJd j_tar, VectorJd j_vel, VectorJd j_acc){
    VectorJd j_del = j_tar - j_sta;
    double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;
    for(int k = 0; k < NO_OF_JOINT; ++k){
        auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(j_vel(k), j_acc(k), fabs(j_del(k)));
        if(t1 > t1_sum) t1_sum = t1;
        if(t2 > t2_sum) t2_sum = t2;
        if(t3 > t3_sum) t3_sum = t3;
    }

    time_zone[0] = t1_sum;
    time_zone[1] = t2_sum;
    time_zone[2] = t3_sum;
    time_total = time_zone[0] + time_zone[1] + time_zone[2];

    for(int k = 0; k < NO_OF_JOINT; ++k){
        move_j_vel(k) = j_del(k) / (time_zone[0] + time_zone[1]);
        move_j_acc(k) = move_j_vel(k) / time_zone[0];
    }

    move_j_ori = j_sta;
    move_j_tar = j_tar;

    timer = 0;
    return MSG_OK;
}

void move_j::Update_Timer(double dt){
    timer += dt;
}

double move_j::Get_Timer(){
    return timer;
}

move_j_control_ret move_j::Control(double _time){
    if(_time < 0.){
        _time = timer;
    }
    
    move_j_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;

    if(_time < time_zone[0]){
        ret.J_output = move_j_ori + 0.5 * move_j_acc * _time * _time;
    }else if(_time < (time_zone[0] + time_zone[1])){
        ret.J_output = move_j_ori + 0.5 * move_j_acc * time_zone[0] * time_zone[0]
                    + move_j_vel * (_time - time_zone[0]);
    }else if(_time < (time_zone[0] + time_zone[1] + time_zone[2])){
        ret.J_output = move_j_tar - 0.5 * move_j_acc * (time_total - _time) * (time_total - _time);
    }else{
        ret.is_finished = true;
        ret.J_output = move_j_tar;
    }

    return ret;
}