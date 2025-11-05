#ifndef MOVE_LB_H
#define MOVE_LB_H

#include "rmath.h"
#include "deque"

struct move_lb_control_ret{
    VectorCd        L_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;
};

struct lb_data_struct{
    VectorCd        _x_tar;
    double          _x_vel[2];
    double          _x_acc[2];
    int             _m_type;
    float           _b_par;

    float           _m_indexing;
};

class move_lb
{
public:
    move_lb();
    ~move_lb();

    void Clear(VectorCd x_sta);
    int Add(VectorCd x_tar, double x_pos_vel, double x_pos_acc, double x_rot_vel, double x_rot_acc, int p_type, float blend_para);
    int Init(int rot_mode, int filter_window);

    void Update_Timer(double dt);
    double Get_Timer();

    move_lb_control_ret Control(double _time);

private:
    double          timer;
    double          time_total;

    bool            is_FirstLoop;

    double                      sampling_time = 0.001;
    std::deque<lb_data_struct>  time_datas;
    double                      time_datas_mid_vel;
    int                         time_rot_mode;
    double                      time_speed_alpha;

    lb_data_struct  Blend_LB_Data(lb_data_struct d1, lb_data_struct d2, double alpha);
    lb_data_struct  get_LB_Average(const std::deque<lb_data_struct>& buffer, lb_data_struct center_value);
    

    std::deque<lb_data_struct> ref_datas;

};
#endif // MOVE_LB_H
