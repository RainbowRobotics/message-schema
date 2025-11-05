#ifndef MOVE_JB_H
#define MOVE_JB_H

#include "rmath.h"
#include "deque"

struct move_jb_control_ret{
    VectorJd        J_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    int             passing_index;
};

struct jb_data_struct{
    // User Input Area
    VectorJd _j_tar;
    VectorJd _j_vel;
    VectorJd _j_acc;
    int      _b_opt;
    float    _b_par;

    // Processed Area;
    double   _t_pseg[3];
    VectorJd _j_pvel;
    VectorJd _j_pacc;
};

struct jb_runtime_struct{
    double  t_sta;
    double  t_end;
    double  f_A;
    double  f_V;
    double  f_P;
    int     seg_index;
};

class move_jb
{
public:
    move_jb();
    ~move_jb();

    void Clear(VectorJd j_sta);
    int Add(VectorJd j_tar, VectorJd j_vel, VectorJd j_acc, int blend_option, float blend_para);
    int Init(VectorJd Limit_Vel);

    void Update_Timer(double dt);
    double Get_Timer();

    move_jb_control_ret Control(double _time);

private:
    double          timer;
    double          time_total;

    double                          JLast[NO_OF_JOINT];
    std::deque<jb_data_struct>      ref_datas;
    std::deque<jb_runtime_struct>   runtime_datas[NO_OF_JOINT];
};
#endif // MOVE_JB_H
