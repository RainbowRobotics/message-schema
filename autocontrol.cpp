#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent)
    : QObject{parent}
{

}

AUTOCONTROL::~AUTOCONTROL()
{

}

void AUTOCONTROL::move_pp(Eigen::Vector3d goal)
{

}

void AUTOCONTROL::move_turn_and_go(Eigen::Vector3d goal)
{

}

void AUTOCONTROL::move_holonomic_pp(Eigen::Vector3d goal)
{

}

void AUTOCONTROL::fsm_loop_pp()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    while(fsm_flag)
    {





        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

void AUTOCONTROL::fsm_loop_turn_and_go()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    while(fsm_flag)
    {



        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

void AUTOCONTROL::fsm_loop_holonomic_pp()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    while(fsm_flag)
    {



        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

