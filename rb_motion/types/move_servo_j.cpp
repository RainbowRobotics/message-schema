#define P_NAME  "MOVE_SERVO_J"

#include "move_servo_j.h"
#include "message.h"
#include "iostream"

move_servo_j::move_servo_j()
{
    ;
}

move_servo_j::~move_servo_j(){
    ;
}

int move_servo_j::Init(VectorJd j_sta, VectorJd limit_up, VectorJd limit_dw, VectorJd limit_vel){
    servo_j_queue_st dummy;
    while(command_queue.try_dequeue(dummy));

    j_input_target_old = j_sta;
    j_input_target_new = j_sta;
    j_target_final = j_sta;
    j_target_speed = VectorJd::Zero(NO_OF_JOINT, 1);
    j_output = j_sta;

    j_limit_up = limit_up;
    j_limit_dw = limit_dw;
    j_limit_vel = limit_vel;

    return MSG_OK;
}

int move_servo_j::Queue_Target(VectorJd deg, double t1, double t2, double gain, double filter_lpf){
    if(t1 < 0.0 || t2 < 0.0001 || gain < 0.0001){
        std::cout<<"Parameter Fail"<<t1<<"/ "<<t2<<"/ "<<gain<<std::endl;
        return MSG_DESIRED_VALUE_IS_OVER_BOUND;
    }
    for(int i = 0; i < NO_OF_JOINT; ++i){
        if(deg(i) > j_limit_up(i) || deg(i) < j_limit_dw(i)){
            std::cout<<"Target Fail"<<deg(i)<<" / "<<j_limit_up(i)<<" / "<<j_limit_dw(i)<<std::endl;
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }
    }

    servo_j_queue_st temp;
    temp.target_deg = deg;
    temp.t1 = t1;
    temp.t2 = t2;
    temp.gain = gain;
    temp.filter = filter_lpf;

    command_queue.enqueue(temp);

    return MSG_OK;
}

move_servo_j_control_ret move_servo_j::Control(double speed_scaler){
    move_servo_j_control_ret ret;
    ret.is_finished = false;
    ret.is_thereErr = 0;
    ret.J_output = j_output;

    servo_j_queue_st line;
    if(command_queue.try_dequeue(line)){
        j_input_target_old = j_input_target_new;
        j_input_target_new = line.target_deg;
        
        j_target_speed = (j_input_target_new - j_input_target_old) / line.t1;
        
        timer_control = line.t1;
        timer_goal    = line.t1 + line.t2;
        timer = 0.0;
        user_gain = line.gain;
        filter_value = line.filter;
    }

    timer += RT_PERIOD_SEC;
    if(timer <= timer_goal){
        double progress_alpha = timer / timer_control;
        //std::cout<<"progress_alpha: "<<progress_alpha<<std::endl;
        j_target_final = (1. - progress_alpha) * j_input_target_old + progress_alpha * j_input_target_new;
        //j_target_final += j_target_speed * RT_PERIOD_SEC;
        
        double total_gain = speed_scaler * user_gain;
        if(total_gain < 0.0001){
            j_output = j_output;
        }else{
            VectorJd desired_delta = j_target_final - j_output;
            double max_over_rate = 1.;
            for(int i = 0; i < NO_OF_JOINT; ++i){
                double joint_max_delta = j_limit_vel(i) * 0.99 * total_gain * RT_PERIOD_SEC;
                if(fabs(desired_delta(i)) > joint_max_delta){
                    double temp_over_rate = fabs(desired_delta(i)) / joint_max_delta;
                    max_over_rate = rb_math::return_big(temp_over_rate, max_over_rate);
                }
            }
            j_output += desired_delta / max_over_rate;
        }
        ret.J_output = j_output;
    }else{
        ret.is_finished = true;
        timer = 0;
    }
    return ret;
}

double move_servo_j::Get_Filter_Value(){
    return filter_value;
}
