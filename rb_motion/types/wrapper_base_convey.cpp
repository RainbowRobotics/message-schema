#define P_NAME  "WRAPPER_TCP_SYM"

#include "wrapper_base_convey.h"
#include "message.h"

#include "iostream"

wrapper_base_conveyor::wrapper_base_conveyor()
{
    ;
}

wrapper_base_conveyor::~wrapper_base_conveyor(){
    ;
}

int wrapper_base_conveyor::Start(Eigen::Vector3d tar_pos_vel){
    pos_vel = tar_pos_vel;
    onoff = true;
    return MSG_OK;
}

int wrapper_base_conveyor::Finish(){
    onoff = false;
    return MSG_OK;
}

bool wrapper_base_conveyor::IsWorking(){
    return onoff;
}

base_conveyor_ret wrapper_base_conveyor::Control(VectorCd original_path){
    std::cout<<"pos sum : "<<pos_sum.transpose()<<std::endl;
    base_conveyor_ret ret;
    ret.L_output = original_path;
    ret.L_output.block(0, 0, 3, 1) = pos_sum + ret.L_output.block(0, 0, 3, 1);
    return ret;
}

void wrapper_base_conveyor::Update_Sum(double dt){
    if(onoff == false) return;
    pos_sum += (pos_vel * dt);
}

void wrapper_base_conveyor::Set_Sum(Eigen::Vector3d _sum){
    pos_sum = _sum;
}