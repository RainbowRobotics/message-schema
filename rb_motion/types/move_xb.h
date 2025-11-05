#ifndef MOVE_XB_H
#define MOVE_XB_H

#include "rmath.h"
#include "deque"
#include "rrbdl.h"

struct move_xb_control_ret{
    VectorJd        J_output;
    unsigned char   is_thereErr;
    bool            is_finished;
    bool            is_First;
    int             passing_index;
    double          timing_belt;
};

struct xb_data_struct{
    // User Input Area
    VectorJd _j_tar;
    VectorJd _j_vel;
    VectorJd _j_acc;
    VectorCd _x_tar;
    double   _x_vel[2];
    double   _x_acc[2];
    int      _m_type;
    int      _b_opt;
    float    _b_par;

    // Processed Area;
    double   _p_seg_dist_mm;
    double   _p_seg_time[3];
    double   _p_seg_mixtime;
    double   _p_seg_T_sta;
    double   _p_seg_T_end;

    // Processed RunTime;
    VectorJd _r_accum_j;
};

struct xb_subret_struct{
    VectorJd    values;
    float       find_index;
    bool        is_success;
    double      timing_belt;
};

class move_xb
{
public:
    move_xb();
    ~move_xb();

    void Clear(VectorJd j_sta, VectorCd x_sta);
    int Add_J(VectorJd j_tar, VectorCd x_tar, VectorJd j_vel, VectorJd j_acc, int blend_option, float blend_para);
    int Add_X(VectorCd x_tar, VectorJd j_tar, double x_pos_vel, double x_pos_acc, double x_rot_vel, double x_rot_acc, int blend_option, float blend_para);
    int Init(int mode, VectorJd internal_Q_ref, VectorJd t_q_min, VectorJd t_q_max, VectorJd t_dq_max, VectorJd t_ddq_max);

    void Update_Timer(double dt);
    double Get_Timer();

    VectorJd Get_LastAdd_J();
    VectorCd Get_LastAdd_X();

    move_xb_control_ret Control(double _time, VectorJd global_Q_ref, VectorJd global_dQ_ref);
    xb_subret_struct Control_VelocityBlend(double _time);
    xb_subret_struct Control_PositionBlend(double _time, VectorJd last_q_vel_deg);

private:
    double          timer;
    double          time_total;

    bool            is_FirstLoop;

    VectorJd        _last_J;
    VectorCd        _last_X;

    int             xb_mode;

    std::deque<xb_data_struct> ref_datas;

    VectorJd        internal_Qref;
    RBDLrobot       xb_robot;
    VectorJd        xb_q_min;
    VectorJd        xb_q_max;
    VectorJd        xb_dq_max;
    VectorJd        xb_ddq_max;
};
#endif // MOVE_XB_H
