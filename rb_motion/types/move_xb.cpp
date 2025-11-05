#define P_NAME  "MOVE_JB"

#include "move_xb.h"

#include "iostream"

move_xb::move_xb()
{
    ;
}

move_xb::~move_xb(){
    ;
}

void move_xb::Clear(VectorJd j_sta, VectorCd x_sta){
    _last_J = j_sta;
    _last_X = x_sta;

    ref_datas.clear();
    xb_data_struct temp_line;
    temp_line._j_tar = j_sta;
    temp_line._j_vel = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._j_acc = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._x_tar = x_sta;
    temp_line._x_vel[0] = temp_line._x_vel[1] = 0;
    temp_line._x_acc[0] = temp_line._x_acc[1] = 0;
    temp_line._b_opt = 0;
    temp_line._b_par = 0;
    temp_line._m_type = 0;

    temp_line._p_seg_dist_mm = 0;
    temp_line._p_seg_mixtime = 0;
    temp_line._p_seg_T_end = 0;
    temp_line._p_seg_T_sta = 0;
    temp_line._p_seg_time[0] = temp_line._p_seg_time[1] = temp_line._p_seg_time[2]  =0;
    temp_line._r_accum_j = VectorJd::Zero(NO_OF_JOINT, 1);
    ref_datas.push_back(temp_line);
}

int move_xb::Add_J(VectorJd j_tar, VectorCd x_tar, VectorJd j_vel, VectorJd j_acc, int blend_option, float blend_para){
    std::cout<<"XB Add J"<<std::endl;
    if(blend_option == 0){
        blend_para = rb_math::saturation_L_and_U(blend_para, 0, 1);
    }else if(blend_option == 1){
        blend_para = rb_math::saturation_Low(blend_para, 0);
    }else{
        blend_option = 0;
        blend_para = 1.0;
    }

    _last_J = j_tar;
    _last_X = x_tar;

    xb_data_struct temp_line;
    temp_line._j_tar = j_tar;
    temp_line._j_vel = j_vel;
    temp_line._j_acc = j_acc;

    temp_line._x_tar = x_tar;
    temp_line._x_vel[0] = temp_line._x_vel[1] = 0;
    temp_line._x_acc[0] = temp_line._x_acc[1] = 0;
    
    temp_line._b_opt = blend_option;
    temp_line._b_par = blend_para;
    temp_line._m_type = 0;
    // DUMMY ADD
    temp_line._p_seg_dist_mm = 0;
    temp_line._p_seg_mixtime = 0;
    temp_line._p_seg_T_end = 0;
    temp_line._p_seg_T_sta = 0;
    temp_line._p_seg_time[0] = temp_line._p_seg_time[1] = temp_line._p_seg_time[2]  =0;
    temp_line._r_accum_j = VectorJd::Zero(NO_OF_JOINT, 1);

    // std::cout<<"templine: "<<temp_line._j_vel.transpose()<<" / "<<temp_line._j_acc.transpose()<<std::endl;
    ref_datas.push_back(temp_line);
    return MSG_OK;
}

int move_xb::Add_X(VectorCd x_tar, VectorJd j_tar, double x_pos_vel, double x_pos_acc, double x_rot_vel, double x_rot_acc, int blend_option, float blend_para){
    std::cout<<"XB Add X"<<std::endl;
    if(blend_option == 0){
        blend_para = rb_math::saturation_L_and_U(blend_para, 0, 1);
    }else if(blend_option == 1){
        blend_para = rb_math::saturation_Low(blend_para, 0);
    }else{
        blend_option = 0;
        blend_para = 1.0;
    }

    _last_J = j_tar;
    _last_X = x_tar;

    xb_data_struct temp_line;
    temp_line._j_tar = j_tar;
    temp_line._j_vel = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._j_acc = VectorJd::Zero(NO_OF_JOINT, 1);

    temp_line._x_tar = x_tar;
    temp_line._x_vel[0] = x_pos_vel;
    temp_line._x_vel[1] = x_rot_vel;
    temp_line._x_acc[0] = x_pos_acc;
    temp_line._x_acc[1] = x_rot_acc;
    
    temp_line._b_opt = blend_option;
    temp_line._b_par = blend_para;
    temp_line._m_type = 1;

    // DUMMY ADD
    temp_line._p_seg_dist_mm = 0;
    temp_line._p_seg_mixtime = 0;
    temp_line._p_seg_T_end = 0;
    temp_line._p_seg_T_sta = 0;
    temp_line._p_seg_time[0] = temp_line._p_seg_time[1] = temp_line._p_seg_time[2]  =0;
    temp_line._r_accum_j = VectorJd::Zero(NO_OF_JOINT, 1);
    // std::cout<<"templine: "<<temp_line._x_vel[0]<<" / "<<temp_line._x_acc[0]<<std::endl;
    ref_datas.push_back(temp_line);
    return MSG_OK;
}

int move_xb::Init(int mode, VectorJd internal_Q_ref, VectorJd t_q_min, VectorJd t_q_max, VectorJd t_dq_max, VectorJd t_ddq_max){
    xb_mode = mode;
    xb_q_min = t_q_min;
    xb_q_max = t_q_max;
    xb_dq_max = t_dq_max;
    xb_ddq_max = t_ddq_max;
    internal_Qref = internal_Q_ref;

    xb_robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());

    // Calc segment Timing
    for(int i = 1; i < (int)ref_datas.size(); ++i){
        xb_data_struct t_Seg_prev = ref_datas.at(i - 1);
        xb_data_struct t_Seg = ref_datas.at(i);

        // std::cout<<"t_Seg: "<<i<<" = "<<t_Seg._j_tar.transpose()<<std::endl;

        if(t_Seg._m_type == 0){
            double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;
            for(int k = 0; k < NO_OF_JOINT; ++k){
                auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(t_Seg._j_vel(k), t_Seg._j_acc(k), fabs(t_Seg._j_tar(k) - t_Seg_prev._j_tar(k)));
                if(t1 > t1_sum) t1_sum = t1;
                if(t2 > t2_sum) t2_sum = t2;
                if(t3 > t3_sum) t3_sum = t3;
            }
            ref_datas.at(i)._p_seg_dist_mm = (rb_math::get_P_3x1(t_Seg._x_tar) - rb_math::get_P_3x1(t_Seg_prev._x_tar)).norm();
            ref_datas.at(i)._p_seg_time[0] = t1_sum;
            ref_datas.at(i)._p_seg_time[1] = t2_sum;
            ref_datas.at(i)._p_seg_time[2] = t3_sum;
        }else{
            double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;
            {
                auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(t_Seg._x_vel[0], t_Seg._x_acc[0], (rb_math::get_P_3x1(t_Seg._x_tar) - rb_math::get_P_3x1(t_Seg_prev._x_tar)).norm());
                if(t1 > t1_sum) t1_sum = t1;
                if(t2 > t2_sum) t2_sum = t2;
                if(t3 > t3_sum) t3_sum = t3;
            }
            {
                auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(t_Seg._x_vel[1], t_Seg._x_acc[1], (rb_math::R_to_RCV(rb_math::get_R_3x3(t_Seg_prev._x_tar).transpose() * rb_math::get_R_3x3(t_Seg._x_tar)).norm() * MATH_R2D));
                if(t1 > t1_sum) t1_sum = t1;
                if(t2 > t2_sum) t2_sum = t2;
                if(t3 > t3_sum) t3_sum = t3;
            }
            {
                auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(t_Seg._x_vel[1], t_Seg._x_acc[1], fabs(rb_math::get_REDUN_1x1(t_Seg._x_tar) - rb_math::get_REDUN_1x1(t_Seg_prev._x_tar)));
                if(t1 > t1_sum) t1_sum = t1;
                if(t2 > t2_sum) t2_sum = t2;
                if(t3 > t3_sum) t3_sum = t3;
            }
            
            ref_datas.at(i)._p_seg_dist_mm = (rb_math::get_P_3x1(t_Seg._x_tar) - rb_math::get_P_3x1(t_Seg_prev._x_tar)).norm();
            ref_datas.at(i)._p_seg_time[0] = t1_sum;
            ref_datas.at(i)._p_seg_time[1] = t2_sum;
            ref_datas.at(i)._p_seg_time[2] = t3_sum;
        }
    }
    // Match T3<->T1
    for(int i = 1; i < (int)(ref_datas.size() - 1); ++i){
        double pre_t3 = ref_datas.at(i)._p_seg_time[2];
        double nex_t1 = ref_datas.at(i + 1)._p_seg_time[0];
        double sync_t = rb_math::return_big(pre_t3, nex_t1);
        ref_datas.at(i)._p_seg_time[2] = ref_datas.at(i + 1)._p_seg_time[0] = sync_t;
    }
    // Calc Mix Up time
    for(int i = 1; i < (int)(ref_datas.size() - 1); ++i){
        xb_data_struct t_Seg_pre = ref_datas.at(i);
        xb_data_struct t_Seg_nex = ref_datas.at(i + 1);

        double pre_ht = t_Seg_pre._p_seg_time[1] * 0.5 + t_Seg_pre._p_seg_time[2];
        double nex_ht = t_Seg_nex._p_seg_time[0] + t_Seg_nex._p_seg_time[1] * 0.5;
        double sync_t = rb_math::return_small(pre_ht, nex_ht);

        //std::cout<<"pre_ht: "<<pre_ht<<"/ "<<nex_ht<<"/ "<<sync_t<<std::endl;

        if(t_Seg_pre._b_opt == 0){
            ref_datas.at(i)._p_seg_mixtime = sync_t * 0.99 * t_Seg_pre._b_par;
            //std::cout<<"ref_datas.at(i)._p_seg_mixtime: "<<ref_datas.at(i)._p_seg_mixtime<<std::endl;
        }else{
            double pre_nett = ref_datas.at(i)._p_seg_time[0] * 0.5 + ref_datas.at(i)._p_seg_time[1] + ref_datas.at(i)._p_seg_time[2] *0.5;
            double nex_nett = ref_datas.at(i + 1)._p_seg_time[0] * 0.5 + ref_datas.at(i + 1)._p_seg_time[1] + ref_datas.at(i + 1)._p_seg_time[2] *0.5;
            if(pre_nett < 1e-4 || nex_nett <1e-4){
                sync_t = 0;
            }else{
                double bD = fabs(ref_datas.at(i)._b_par);
                double pre_vv = ref_datas.at(i)._p_seg_dist_mm / pre_nett;
                double nex_vv = ref_datas.at(i + 1)._p_seg_dist_mm/ nex_nett;

                double pre_D_step1 = pre_vv * ref_datas.at(i)._p_seg_time[2] * 0.5;
                double pre_D_step2 = pre_D_step1 + pre_vv * ref_datas.at(i)._p_seg_time[1] * 0.5;
                double nex_D_step1 = nex_vv * ref_datas.at(i + 1)._p_seg_time[0] * 0.5;
                double nex_D_step2 = nex_D_step1 + nex_vv * ref_datas.at(i + 1)._p_seg_time[1] * 0.5;

                double pre_eq_t = pre_ht;
                if(pre_vv > 1e-3){
                    if(bD < pre_D_step1){
                        pre_eq_t = sqrt(2. * bD * ref_datas.at(i)._p_seg_time[2] / pre_vv);
                    }else if(bD < pre_D_step2){
                        pre_eq_t = (bD - pre_D_step1) / pre_vv + ref_datas.at(i)._p_seg_time[2];
                    }else{
                        pre_eq_t = pre_ht;
                    }
                }
                double nex_eq_t = nex_ht;
                if(nex_vv > 1e-3){
                    if(bD < nex_D_step1){
                        nex_eq_t = sqrt(2. * bD * ref_datas.at(i + 1)._p_seg_time[0] / nex_vv);
                    }else if(bD < nex_D_step2){
                        nex_eq_t = (bD - nex_D_step1) / nex_vv + ref_datas.at(i + 1)._p_seg_time[0];
                    }else{
                        nex_eq_t = nex_ht;
                    }
                }
                sync_t = rb_math::return_small(rb_math::return_small(pre_eq_t, pre_ht), rb_math::return_small(nex_eq_t, nex_ht));
            }
            //std::cout<<"sync_t: "<<sync_t<<std::endl;
            ref_datas.at(i)._p_seg_mixtime = sync_t;
        }

        if(i == ((int)ref_datas.size() - 2)){
            ref_datas.at(i + 1)._p_seg_mixtime = 0.;
        }
    }
    // Calc Timing Point
    double total_motion_time = 0.; double temp_time_sum = 0.;
    for(int i = 1; i < (int)ref_datas.size(); ++i){
        std::cout<<"ref_datas.at(i)._p_seg_mixtime: "<<ref_datas.at(i)._p_seg_mixtime<<std::endl;
        total_motion_time += (ref_datas.at(i)._p_seg_time[0] + ref_datas.at(i)._p_seg_time[1] + ref_datas.at(i)._p_seg_time[2] - ref_datas.at(i)._p_seg_mixtime);
        
        ref_datas.at(i)._p_seg_T_sta = temp_time_sum;
        ref_datas.at(i)._p_seg_T_end = temp_time_sum + (ref_datas.at(i)._p_seg_time[0] + ref_datas.at(i)._p_seg_time[1] + ref_datas.at(i)._p_seg_time[2]);
        temp_time_sum = ref_datas.at(i)._p_seg_T_end - ref_datas.at(i)._p_seg_mixtime;
    }

    // Init Accum Joint Value
    for(int i = 0; i < (int)ref_datas.size(); ++i){
        ref_datas.at(i)._r_accum_j = VectorJd::Zero(NO_OF_JOINT, 1);
    }


    std::cout<<"total_motion_time: "<<total_motion_time<<std::endl;
    // return
    // 0 : no Err

    is_FirstLoop = true;
    time_total = total_motion_time;
    timer = 0;
    return MSG_OK;
}

void move_xb::Update_Timer(double dt){
    timer += dt;
}

double move_xb::Get_Timer(){
    return timer;
}

VectorJd move_xb::Get_LastAdd_J(){
    return _last_J;
}

VectorCd move_xb::Get_LastAdd_X(){
    return _last_X;
}

move_xb_control_ret move_xb::Control(double _time, VectorJd global_Q_ref, VectorJd global_dQ_ref){
    if(_time < 0.){
        _time = timer;
    }

    move_xb_control_ret ret;
    ret.is_thereErr = 0;
    ret.is_finished = false;
    ret.timing_belt = 1.;

    float find_index = -1;
    bool is_time_end = false;

    if(_time > ref_datas.at(ref_datas.size() - 1)._p_seg_T_end){
        find_index = ref_datas.size();
        is_time_end = true;
    }else{
        if(xb_mode == 1){
            ;//position blending
            xb_subret_struct ret_q = Control_PositionBlend(_time, global_dQ_ref);
            ret.timing_belt = ret_q.timing_belt;
            if(ret_q.is_success){
                find_index = ret_q.find_index;
                internal_Qref = ret_q.values;
            }else{
                ret.is_thereErr = 1;
            }
        }else{
            ;//speed blending
            xb_subret_struct ret_dq = Control_VelocityBlend(_time);
            ret.timing_belt = ret_dq.timing_belt;
            if(ret_dq.is_success){
                find_index = ret_dq.find_index;
                internal_Qref += (ret_dq.values * RT_PERIOD_SEC);
            }else{
                ret.is_thereErr = 1;
            }
        }
    }

    (void)find_index;

    bool is_converge = true;
    for(int k = 0; k < NO_OF_JOINT; k++){
        double delt_jnt = internal_Qref(k) - global_Q_ref(k);
        if(fabs(delt_jnt) > 0.0005){
            is_converge &= false;
        }
        delt_jnt = rb_math::saturation_L_and_U(delt_jnt, (-xb_dq_max(k) * RT_PERIOD_SEC), (xb_dq_max(k) * RT_PERIOD_SEC));
        ret.J_output(k) = global_Q_ref(k) + delt_jnt;
    }

    if(is_time_end && is_converge){
        ret.is_finished = true;
    }
    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;
    return ret;
}

xb_subret_struct move_xb::Control_VelocityBlend(double _time){
    xb_subret_struct ret_dq;
    ret_dq.values = VectorJd::Zero(NO_OF_JOINT, 1);
    ret_dq.find_index = -1.;
    ret_dq.is_success = true;
    ret_dq.timing_belt = 1.;

    for(int i = 1; i < (int)ref_datas.size(); ++i){
        xb_data_struct t_Seg = ref_datas.at(i);
        xb_data_struct t_Seg_Old = ref_datas.at(i - 1);
        if(_time >= t_Seg._p_seg_T_sta && _time <= t_Seg._p_seg_T_end){
            double st = _time - t_Seg._p_seg_T_sta;
            double t1 = t_Seg._p_seg_time[0];
            double t2 = t_Seg._p_seg_time[1];
            double t3 = t_Seg._p_seg_time[2];

            if(ret_dq.find_index < 0){
                ret_dq.find_index = ((float)i) + st / (t_Seg._p_seg_T_end - t_Seg._p_seg_T_sta);
            }

            if(t_Seg._m_type == 0){
                ;// Joint Move
                VectorJd ori_j = t_Seg_Old._j_tar;
                VectorJd tar_j = t_Seg._j_tar;
                VectorJd spd_j = (tar_j - ori_j) / (0.5 * t1 + t2 + 0.5 * t3);
                VectorJd acc_j_1 = spd_j / t1;
                VectorJd acc_j_3 = -spd_j / t3;
                VectorJd out_vel = VectorJd::Zero(NO_OF_JOINT, 1);
                if(st <= t1){
                    out_vel = st * acc_j_1;
                }else if(st <= (t1 + t2)){
                    out_vel = spd_j;
                }else if(st <= (t1 + t2 + t3)){
                    out_vel = spd_j + (st - t1 - t2) * acc_j_3;
                }
                ret_dq.values += out_vel;
            }else{
                ;// Position Move
                Eigen::Vector3d pos_start = rb_math::get_P_3x1(t_Seg_Old._x_tar);
                Eigen::Matrix3d mat_start = rb_math::get_R_3x3(t_Seg_Old._x_tar);
                double        redun_start = rb_math::get_REDUN_1x1(t_Seg_Old._x_tar);


                Eigen::Vector3d pos_target = rb_math::get_P_3x1(t_Seg._x_tar);
                Eigen::Matrix3d mat_target = rb_math::get_R_3x3(t_Seg._x_tar);
                double        redun_target = rb_math::get_REDUN_1x1(t_Seg._x_tar);

                Eigen::Vector3d spd_pos = (pos_target - pos_start) / (0.5 * t1 + t2 + 0.5 * t3);
                Eigen::Vector3d acc_1_pos = spd_pos / t1;
                Eigen::Vector3d acc_3_pos = -spd_pos / t3;

                Eigen::Vector3d delta_rcv = rb_math::R_to_RCV(mat_start.transpose() * mat_target);
                double delta_rcv_size = delta_rcv.norm();
                if(delta_rcv_size < 1e-6)
                    delta_rcv_size = 1e-6;
                Eigen::Vector3d delta_rcv_vec = delta_rcv / delta_rcv_size;
                double spd_rcv = delta_rcv_size / (0.5 * t1 + t2 + 0.5 * t3);//rad
                double acc_1_rcv = spd_rcv / t1;//rad
                double acc_3_rcv = -spd_rcv / t3;//rad

                double spd_redun = (redun_target - redun_start) / (0.5 * t1 + t2 + 0.5 * t3);
                double acc_1_redun = spd_redun / t1;
                double acc_3_redun = -spd_redun / t3;

                Eigen::Vector3d pos_ik = pos_target;
                Eigen::Vector3d rcv_delta_ik = delta_rcv;
                Eigen::Matrix3d mat_ik = mat_target;
                double          redun_ik = redun_target;

                if(st <= t1){
                    pos_ik = pos_start + 0.5 * acc_1_pos * st * st;
                    rcv_delta_ik = delta_rcv_vec * (0.5 * acc_1_rcv * st * st);
                    redun_ik = redun_start + 0.5 * acc_1_redun * st * st;
                }else if(st <= (t1 + t2)){
                    pos_ik = pos_start + 0.5 * acc_1_pos * t1 * t1 + spd_pos * (st - t1);
                    rcv_delta_ik = delta_rcv_vec * (0.5 * acc_1_rcv * t1 * t1 + spd_rcv * (st - t1));
                    redun_ik = redun_start + 0.5 * acc_1_redun * t1 * t1 + spd_redun * (st - t1);
                }else if(st <= (t1 + t2 + t3)){
                    pos_ik = pos_target + 0.5 * acc_3_pos * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st);
                    rcv_delta_ik = delta_rcv_vec * (delta_rcv_size + 0.5 * acc_3_rcv * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st));
                    redun_ik = redun_target + 0.5 * acc_3_redun * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st);
                }
                mat_ik = mat_start * rb_math::RCV_to_R(rcv_delta_ik);

                VectorJd ik_start_q_deg = t_Seg_Old._j_tar + t_Seg._r_accum_j;
                VectorCd ik_start_x_pos = xb_robot.Calc_ForwardKinematics(ik_start_q_deg);
                #if NO_OF_JOINT == 7
                    VectorCd ik_target_x_pos = rb_math::Make_C_from_PandR(pos_ik, mat_ik, redun_ik);
                #else
                    VectorCd ik_target_x_pos = rb_math::Make_C_from_PandR(pos_ik, mat_ik);
                #endif

                IkResult ik_re = xb_robot.Calc_InverseKinematics_Static(ik_target_x_pos, 2, 15, ik_start_x_pos, ik_start_q_deg);

                if(ik_re.result != IkRet::IK_OK){
                    std::cout<<"IK PROB IK PROB"<<std::endl;
                    ret_dq.is_success = false;
                }else{
                    VectorJd temp_new_accum = ik_re.q_out_deg - t_Seg_Old._j_tar;//deg
                    ret_dq.values += ((temp_new_accum - t_Seg._r_accum_j) / RT_PERIOD_SEC);
                    ret_dq.timing_belt = rb_math::return_small(ret_dq.timing_belt, ik_re.time_scaler);
                    ref_datas.at(i)._r_accum_j = temp_new_accum;
                }
            }
        }
    }

    if(ret_dq.find_index < 0){
        std::cout<<"ret_dq.find_index -1"<<std::endl;
    }
    
    return ret_dq;
}

xb_subret_struct move_xb::Control_PositionBlend(double _time, VectorJd last_q_vel_deg){
    xb_subret_struct ret_q;
    ret_q.values = VectorJd::Zero(NO_OF_JOINT, 1);
    ret_q.find_index = -1;
    ret_q.is_success = true;
    ret_q.timing_belt = 1.;

    int related_segment_num = 0;
    for(int i = 1; i < (int)ref_datas.size(); ++i){
        xb_data_struct t_Seg = ref_datas.at(i);
        xb_data_struct t_Seg_Old = ref_datas.at(i - 1);
        if(_time >= t_Seg._p_seg_T_sta && _time <= t_Seg._p_seg_T_end){
            related_segment_num++;
            if(related_segment_num > 2){
                std::cout<<"!!!!!!!!!!!!!!!!!!! something goes wrong in segment counting"<<std::endl;
            }

            double st = _time - t_Seg._p_seg_T_sta;
            double t1 = t_Seg._p_seg_time[0];
            double t2 = t_Seg._p_seg_time[1];
            double t3 = t_Seg._p_seg_time[2];
            double tm = t_Seg._p_seg_mixtime;
            double mixing_alpha = 1.;
            if(related_segment_num == 1){
                double mixing_time = t1 + t2 + t3 - tm;
                if(st > mixing_time){
                    mixing_alpha = 1. - (st - mixing_time) / tm;
                }
            }else if(related_segment_num == 2){
                if(st < t_Seg_Old._p_seg_mixtime){
                    mixing_alpha = st / t_Seg_Old._p_seg_mixtime;
                }
            }
            mixing_alpha = rb_math::filt_Curve(mixing_alpha, 0, 1, 0, 1);
            if(ret_q.find_index < 0){
                double time_zone_alpha = st / (t_Seg._p_seg_T_end - t_Seg._p_seg_T_sta);
                ret_q.find_index = ((float)i) + time_zone_alpha;
            }

            if(t_Seg._m_type == 0){
                ;// Joint Move
                VectorJd ori_j = t_Seg_Old._j_tar;
                VectorJd tar_j = t_Seg._j_tar;
                VectorJd spd_j = (tar_j - ori_j) / (0.5 * t1 + t2 + 0.5 * t3);
                VectorJd acc_j_1 = spd_j / t1;
                VectorJd acc_j_3 = -spd_j / t3;
                VectorJd out_joint = tar_j;
                if(st <= t1){
                    out_joint = ori_j + 0.5 * acc_j_1 * st * st;
                }else if(st <= (t1 + t2)){
                    out_joint = ori_j + 0.5 * acc_j_1 * t1 * t1 + spd_j * (st - t1);
                }else if(st <= (t1 + t2 + t3)){
                    out_joint = ori_j + 0.5 * acc_j_1 * t1 * t1 + spd_j * t2 + spd_j * (st - t1 - t2) + 0.5 * acc_j_3 * (st - t1 - t2) * (st - t1 - t2);
                }
                ret_q.values += (out_joint * mixing_alpha);
            }else{
                ;// Position Move
                Eigen::Vector3d pos_start = rb_math::get_P_3x1(t_Seg_Old._x_tar);
                Eigen::Matrix3d mat_start = rb_math::get_R_3x3(t_Seg_Old._x_tar);
                double        redun_start = rb_math::get_REDUN_1x1(t_Seg_Old._x_tar);

                Eigen::Vector3d pos_target = rb_math::get_P_3x1(t_Seg._x_tar);
                Eigen::Matrix3d mat_target = rb_math::get_R_3x3(t_Seg._x_tar);
                double        redun_target = rb_math::get_REDUN_1x1(t_Seg._x_tar);

                Eigen::Vector3d spd_pos = (pos_target - pos_start) / (0.5 * t1 + t2 + 0.5 * t3);
                Eigen::Vector3d acc_1_pos = spd_pos / t1;
                Eigen::Vector3d acc_3_pos = -spd_pos / t3;

                Eigen::Vector3d delta_rcv = rb_math::R_to_RCV(mat_start.transpose() * mat_target);
                double delta_rcv_size = delta_rcv.norm();
                if(delta_rcv_size < 1e-6)
                    delta_rcv_size = 1e-6;
                Eigen::Vector3d delta_rcv_vec = delta_rcv / delta_rcv_size;
                double spd_rcv = delta_rcv_size / (0.5 * t1 + t2 + 0.5 * t3);//rad
                double acc_1_rcv = spd_rcv / t1;//rad
                double acc_3_rcv = -spd_rcv / t3;//rad

                double spd_redun = (redun_target - redun_start) / (0.5 * t1 + t2 + 0.5 * t3);
                double acc_1_redun = spd_redun / t1;
                double acc_3_redun = -spd_redun / t3;

                Eigen::Vector3d pos_ik = pos_target;
                Eigen::Vector3d rcv_delta_ik = delta_rcv;
                Eigen::Matrix3d mat_ik = mat_target;
                double          redun_ik = redun_target;

                if(st <= t1){
                    pos_ik = pos_start + 0.5 * acc_1_pos * st * st;
                    rcv_delta_ik = delta_rcv_vec * (0.5 * acc_1_rcv * st * st);
                    redun_ik = redun_start + 0.5 * acc_1_redun * st * st;
                }else if(st <= (t1 + t2)){
                    pos_ik = pos_start + 0.5 * acc_1_pos * t1 * t1 + spd_pos * (st - t1);
                    rcv_delta_ik = delta_rcv_vec * (0.5 * acc_1_rcv * t1 * t1 + spd_rcv * (st - t1));
                    redun_ik = redun_start + 0.5 * acc_1_redun * t1 * t1 + spd_redun * (st - t1);
                }else if(st <= (t1 + t2 + t3)){
                    pos_ik = pos_target + 0.5 * acc_3_pos * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st);
                    rcv_delta_ik = delta_rcv_vec * (delta_rcv_size + 0.5 * acc_3_rcv * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st));
                    redun_ik = redun_target + 0.5 * acc_3_redun * (t1 + t2 + t3 - st) * (t1 + t2 + t3 - st);
                }
                mat_ik = mat_start * rb_math::RCV_to_R(rcv_delta_ik);

                #if NO_OF_JOINT == 7
                    VectorCd temp_x_target_ik = rb_math::Make_C_from_PandR(pos_ik, mat_ik, redun_ik);
                #else
                    VectorCd temp_x_target_ik = rb_math::Make_C_from_PandR(pos_ik, mat_ik);
                #endif
                VectorJd temp_q_ang_ik = t_Seg_Old._j_tar + t_Seg._r_accum_j;
                VectorCd temp_x_pos_ik = xb_robot.Calc_ForwardKinematics(temp_q_ang_ik);
                double   temp_jacob_ik = xb_robot.Calc_Jacobian_Det(temp_q_ang_ik);
                (void)temp_jacob_ik;

                IkResult temp_ret_ik = xb_robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, RT_PERIOD_SEC, temp_x_target_ik, temp_x_pos_ik
                , temp_q_ang_ik, xb_q_min, xb_q_max, xb_dq_max, xb_ddq_max, last_q_vel_deg, 5.0, 1.0, true);

                if(temp_ret_ik.result == IkRet::IK_OK){
                    
                    ref_datas.at(i)._r_accum_j = temp_ret_ik.q_out_deg - t_Seg_Old._j_tar;
                    ret_q.values += (temp_ret_ik.q_out_deg * mixing_alpha);
                    ret_q.timing_belt = rb_math::return_small(ret_q.timing_belt, temp_ret_ik.time_scaler);
                }else{
                    std::cout<<"XB POS ik fail"<<std::endl;
                    ret_q.is_success = false;
                }
            }
        }
    }

    if(ret_q.find_index < 0){
        std::cout<<"ret_dq.find_index -1"<<std::endl;
    }
    
    return ret_q;
}