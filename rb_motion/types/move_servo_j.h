#ifndef MOVE_SERVO_J_H
#define MOVE_SERVO_J_H

#include "rmath.h"
#include "concurrentqueue.h"
struct move_servo_j_control_ret{
    VectorJd        J_output;
    unsigned char   is_thereErr;
    bool            is_finished;
};



class move_servo_j
{
public:
    move_servo_j();
    ~move_servo_j();

    int Init(VectorJd j_sta, VectorJd limit_up, VectorJd limit_dw, VectorJd limit_vel);
    int Queue_Target(VectorJd deg, double t1, double t2, double gain, double filter_lpf);

    move_servo_j_control_ret Control(double speed_scaler);

    double Get_Filter_Value();

private:
    struct servo_j_queue_st{
        VectorJd        target_deg;
        double          t1;
        double          t2;
        double          gain;
        double          filter;
    };

    VectorJd        j_limit_up;
    VectorJd        j_limit_dw;
    VectorJd        j_limit_vel;

    moodycamel::ConcurrentQueue<servo_j_queue_st> command_queue;
    double          user_gain;
    double          timer;
    double          timer_control;
    double          timer_goal;

    double          filter_value;

    VectorJd        j_input_target_new;
    VectorJd        j_input_target_old;

    VectorJd        j_target_speed;//deg/sec
    VectorJd        j_target_final;

    VectorJd        j_output;
};
#endif // MOVE_SERVO_J_H
