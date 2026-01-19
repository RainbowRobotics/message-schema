#ifndef MOVE_SERVO_L_H
#define MOVE_SERVO_L_H

#include "rmath.h"
#include "concurrentqueue.h"
struct move_servo_l_control_ret{
    VectorCd        C_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;

    double          ratio;
};



class move_servo_l
{
public:
    move_servo_l();
    ~move_servo_l();

    int Init(VectorCd c_sta);
    int Queue_Target(VectorCd carte, double t1, double t2, double gain, double filter_lpf);

    move_servo_l_control_ret Control(double speed_scaler);

    double Get_Filter_Value();

private:
    struct servo_l_queue_st{
        VectorCd        target_carte;
        double          t1;
        double          t2;
        double          gain;
        double          filter;
    };

    bool            is_FirstLoop = true;

    moodycamel::ConcurrentQueue<servo_l_queue_st> command_queue;
    double          user_gain;
    double          timer;
    double          timer_control;
    double          timer_goal;

    double          filter_value;

    VectorCd        c_input_target_new;
    VectorCd        c_input_target_old;
    //VectorCd        c_target_speed;
    VectorCd        c_target_final;

};
#endif // MOVE_SERVO_L_H
