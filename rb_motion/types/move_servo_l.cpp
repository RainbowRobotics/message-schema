#define P_NAME  "MOVE_SERVO_L"

#include "move_servo_l.h"
#include "message.h"
#include "iostream"

move_servo_l::move_servo_l()
{
    ;
}

move_servo_l::~move_servo_l(){
    ;
}

int move_servo_l::Init(VectorCd c_sta){
    servo_l_queue_st dummy;
    while(command_queue.try_dequeue(dummy));

    is_FirstLoop = true;

    c_input_target_old = c_sta;
    c_input_target_new = c_sta;
    c_target_final = c_sta;

    return MSG_OK;
}

int move_servo_l::Queue_Target(VectorCd carte, double t1, double t2, double gain, double filter_lpf){
    if(t1 < 0.0 || t2 < 0.0001 || gain < 0.0001){
        std::cout<<"Parameter Fail"<<t1<<"/ "<<t2<<"/ "<<gain<<std::endl;
        return MSG_DESIRED_VALUE_IS_OVER_BOUND;
    }

    servo_l_queue_st temp;
    temp.target_carte = carte;
    temp.t1 = t1;
    temp.t2 = t2;
    temp.gain = gain;
    temp.filter = filter_lpf;

    command_queue.enqueue(temp);

    return MSG_OK;
}

move_servo_l_control_ret move_servo_l::Control(double speed_scaler){
    move_servo_l_control_ret ret;
    ret.is_finished = false;
    ret.is_thereErr = 0;
    ret.C_output = c_target_final;

    servo_l_queue_st line;
    if(command_queue.try_dequeue(line)){
        c_input_target_old = c_input_target_new;
        c_input_target_new = line.target_carte;
        
        //c_target_speed = (c_input_target_new - c_input_target_old) / line.t1;
        
        timer_control = line.t1;
        timer_goal    = line.t1 + line.t2;
        timer = 0.0;
        user_gain = line.gain;
        filter_value = line.filter;
    }

    timer += RT_PERIOD_SEC;
    if(timer <= timer_goal){
        double progress_alpha = timer / timer_control;
        c_target_final = rb_math::Blend_Carte(c_input_target_old, c_input_target_new, progress_alpha);
    }else{
        ret.is_finished = true;
        timer = 0;
    }

    ret.ratio = speed_scaler * user_gain;
    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;


    return ret;
}

double move_servo_l::Get_Filter_Value(){
    return filter_value;
}