#define P_NAME  "WRAPPER_TCP_SYM"

#include "wrapper_tcp_weave_sym.h"
#include "message.h"

wrapper_tcp_weave_sym::wrapper_tcp_weave_sym()
{
    ;
}

wrapper_tcp_weave_sym::~wrapper_tcp_weave_sym(){
    ;
}

int wrapper_tcp_weave_sym::Start(double magnitude, double time){
    L = magnitude;
    T[0] = time;
    onoff = true;
    return MSG_OK;
}

int wrapper_tcp_weave_sym::Finish(){
    onoff = false;
    return MSG_OK;
}

bool wrapper_tcp_weave_sym::IsWorking(){
    return onoff;
}

tcp_weave_shm_ret wrapper_tcp_weave_sym::Control(VectorCd original_path){
    tcp_weave_shm_ret ret;

    ret.L_output = original_path;

    double magnitude = L * sin(2. * MATH_PI * timer / T[0]);
    ret.L_output.block(0, 0, 3, 1) = ret.L_output.block(0, 0, 3, 1)
                                    + rb_math::get_R_3x3(original_path) * Eigen::Vector3d(0, 0, 1) * magnitude;

    return ret;
}

void wrapper_tcp_weave_sym::Update_Timer(double dt){
    if(onoff == false) return;
    timer += dt;
}

double wrapper_tcp_weave_sym::Get_Timer(){
    return timer;
}

void wrapper_tcp_weave_sym::Set_Timer(double _time){
    timer = _time;
}