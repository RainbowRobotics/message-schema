#ifndef WRAPPER_BASE_CONVEY_H
#define WRAPPER_BASE_CONVEY_H

#include "rmath.h"
struct base_conveyor_ret {
    VectorCd L_output;
};

class wrapper_base_conveyor
{
public:
    wrapper_base_conveyor();
    ~wrapper_base_conveyor();

    int Start(Eigen::Vector3d tar_pos_vel);
    int Finish();
    bool IsWorking();

    base_conveyor_ret Control(VectorCd original_path);

    void Update_Sum(double dt);
    void Set_Sum(Eigen::Vector3d _sum);

private:
    bool            onoff;
    Eigen::Vector3d pos_vel;
    Eigen::Vector3d pos_sum;
};

#endif // WRAPPER_BASE_CONVEY_H
