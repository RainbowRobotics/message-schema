#define P_NAME  "MOVE_LB"

#include "move_lb.h"

#include "iostream"
#include "message.h"

move_lb::move_lb()
{
    ;
}

move_lb::~move_lb(){
    ;
}

void move_lb::Clear(VectorCd x_sta){
    ref_datas.clear();

    lb_data_struct temp_line;
    temp_line._x_tar = x_sta;
    temp_line._x_vel[0] = temp_line._x_vel[1] = 0;
    temp_line._x_acc[0] = temp_line._x_acc[1] = 0;
    temp_line._b_par = 0;
    temp_line._m_type = 0;
    temp_line._m_indexing = 0.;

    ref_datas.push_back(temp_line);
}

int move_lb::Add(VectorCd x_tar, double x_pos_vel, double x_pos_acc, double x_rot_vel, double x_rot_acc, int p_type, float blend_para){
    lb_data_struct temp_line;
    temp_line._x_tar = x_tar;
    temp_line._x_vel[0] = x_pos_vel;
    temp_line._x_vel[1] = x_rot_vel;
    temp_line._x_acc[0] = x_pos_acc;
    temp_line._x_acc[1] = x_rot_acc;
    temp_line._b_par = blend_para;
    temp_line._m_type = p_type;
    temp_line._m_indexing = (float)(ref_datas.size());

    ref_datas.push_back(temp_line);

    return MSG_OK;
}

int move_lb::Init(int rot_mode, int filter_window){
    // rot mode
    // 0 : intended
    // 1 : constant

    float pos_dev_ = 10;
    float ang_dev_ = 10;

    time_rot_mode = rot_mode;
    // -----------------------------------------------
    // Set Last Point to Normal Point
    // ref_datas
    // -----------------------------------------------
    ref_datas.at(ref_datas.size() - 1)._m_type = 0;
    ref_datas.at(0)._x_vel[0] = ref_datas.at(1)._x_vel[0];
    ref_datas.at(0)._x_vel[1] = ref_datas.at(1)._x_vel[1];
    ref_datas.at(0)._x_acc[0] = ref_datas.at(1)._x_acc[0];
    ref_datas.at(0)._x_acc[1] = ref_datas.at(1)._x_acc[1];

    // -----------------------------------------------
    // Make Touch data
    // ref_datas -> tch_datas
    // -----------------------------------------------
    std::deque<lb_data_struct> tch_datas; tch_datas.clear();
    tch_datas.push_back(ref_datas.at(0));
    for(int i = 1; i < ref_datas.size(); ++i){
        if(ref_datas.at(i)._m_type >= 2){
            if(ref_datas.at(i - 1)._m_type == 1){
                tch_datas.push_back(Blend_LB_Data(ref_datas.at(i - 1), ref_datas.at(i + 0), 0.5));
            }
            tch_datas.push_back(ref_datas.at(i + 0));
            if(ref_datas.at(i + 1)._m_type != 0){
                tch_datas.push_back(Blend_LB_Data(ref_datas.at(i + 0), ref_datas.at(i + 1), 0.5));
            }
        }else{
            tch_datas.push_back(ref_datas.at(i + 0));
        }
    }
    // -----------------------------------------------
    // Make Touch data
    // tch_datas -> pth_datas
    // -----------------------------------------------
    std::deque<lb_data_struct> pth_datas; pth_datas.clear();
    if(tch_datas.at(1)._m_type <= 0){
        lb_data_struct t_cur = tch_datas.at(0);
        lb_data_struct t_nex = tch_datas.at(1);

        int line_number = (rb_math::get_P_3x1(t_nex._x_tar) -  rb_math::get_P_3x1(t_cur._x_tar)).norm() * pos_dev_;
        if(rot_mode != 1){
            int num_of_iter_deg = rb_math::R_to_RCV(rb_math::get_R_3x3(t_nex._x_tar) * rb_math::get_R_3x3(t_cur._x_tar).transpose()).norm() * MATH_R2D * ang_dev_;
            line_number = rb_math::return_big(line_number, num_of_iter_deg);
        }
        line_number = rb_math::saturation_Low(line_number, 10);
        for(int b = 1; b <= line_number; ++b){
            double blend_rate = ((double)(b-1))/((double)(line_number-1));
            pth_datas.push_back(Blend_LB_Data(t_cur, t_nex, blend_rate));
        }
    }else{
        pth_datas.push_back(tch_datas.at(0));
    }

    for(int i = 1; i < (tch_datas.size() - 1); ++i){
        lb_data_struct t_old = tch_datas.at(i - 1);
        lb_data_struct t_cur = tch_datas.at(i + 0);
        lb_data_struct t_nex = tch_datas.at(i + 1);

        if(t_cur._m_type <= 0){
            if(t_nex._m_type <= 0){
                int line_number = (rb_math::get_P_3x1(t_nex._x_tar) -  rb_math::get_P_3x1(t_cur._x_tar)).norm() * pos_dev_;
                if(rot_mode != 1){
                    int num_of_iter_deg = rb_math::R_to_RCV(rb_math::get_R_3x3(t_nex._x_tar) * rb_math::get_R_3x3(t_cur._x_tar).transpose()).norm() * MATH_R2D * ang_dev_;
                    line_number = rb_math::return_big(line_number, num_of_iter_deg);
                }
                line_number = rb_math::saturation_Low(line_number, 10);
                for(int b = 1; b <= line_number; ++b){
                    double blend_rate = ((double)(b-1))/((double)(line_number-1));
                    pth_datas.push_back(Blend_LB_Data(t_cur, t_nex, blend_rate));
                }
            }
        }else if(t_cur._m_type == 1){
            // t_old -> t_cur -> t_nex
            Circle_3P_RET cir_fit = rb_math::fit_CircleFrom3Points(rb_math::get_P_3x1(t_old._x_tar), rb_math::get_P_3x1(t_cur._x_tar), rb_math::get_P_3x1(t_nex._x_tar));
            if(cir_fit.result != true){
                std::cout<<"Circle Generation Err"<<std::endl;
                return MSG_MOVE_PATH_CIRCLE_FIT_ERR;
            }
            int cir_draw_sector_num = 2;
            if(t_nex._m_type == 1){
                cir_draw_sector_num = 1;
            }
            for(int cir = 0; cir < cir_draw_sector_num; ++cir){
                double t_draw_angle = 0;
                lb_data_struct draw_sta, draw_eta;
                if(cir == 0){
                    t_draw_angle = cir_fit.angle12;
                    draw_sta = t_old;
                    draw_eta = t_cur;
                }else{
                    t_draw_angle = cir_fit.angle23;
                    draw_sta = t_cur;
                    draw_eta = t_nex;
                }

                int circle_number = (int)(2. * MATH_PI * cir_fit.radius / 360. * t_draw_angle * pos_dev_);
                if(rot_mode != 1){
                    int num_of_iter_deg = rb_math::R_to_RCV(rb_math::get_R_3x3(draw_eta._x_tar) * rb_math::get_R_3x3(draw_sta._x_tar).transpose()).norm() * MATH_R2D * ang_dev_;
                    circle_number = rb_math::return_big(circle_number, num_of_iter_deg);
                }
                circle_number = rb_math::saturation_Low(circle_number, 10);
                for(int c = 1; c <= circle_number; ++c){
                    double t_circle_rate = ((double)(c-1))/((double)(circle_number-1));
                    lb_data_struct t_circle_data = Blend_LB_Data(draw_sta, draw_eta, t_circle_rate);
                    double t_circle_angle = t_circle_rate * t_draw_angle;
                    Eigen::Vector3d t_P = cir_fit.center + rb_math::RCV_to_R(cir_fit.axis * (t_circle_angle * MATH_D2R)) * (rb_math::get_P_3x1(draw_sta._x_tar) - cir_fit.center);
                    t_circle_data._x_tar.block(0, 0, 3, 1) = t_P;
                    pth_datas.push_back(t_circle_data);
                }
            }
        }else{
            double rate_old_to_cur = 1. - t_cur._b_par;
            double rate_cur_to_nex = t_cur._b_par;

            lb_data_struct blend_sta = Blend_LB_Data(t_old, t_cur, rate_old_to_cur);
            lb_data_struct blend_mid = t_cur;
            lb_data_struct blend_end = Blend_LB_Data(t_cur, t_nex, rate_cur_to_nex);

            // t_old -> blend_sta
            int line_number_1 = (rb_math::get_P_3x1(blend_sta._x_tar) -  rb_math::get_P_3x1(t_old._x_tar)).norm() * pos_dev_;
            if(rot_mode != 1){
                int num_of_iter_deg = rb_math::R_to_RCV(rb_math::get_R_3x3(blend_sta._x_tar) * rb_math::get_R_3x3(t_old._x_tar).transpose()).norm() * MATH_R2D * ang_dev_;
                line_number_1 = rb_math::return_big(line_number_1, num_of_iter_deg);
            }
            line_number_1 = rb_math::saturation_Low(line_number_1, 10);
            // blend_sta -> blend_mid(t_cur) -> blend_end
            int blend_number = ((rb_math::get_P_3x1(blend_mid._x_tar) -  rb_math::get_P_3x1(blend_sta._x_tar)).norm() + (rb_math::get_P_3x1(blend_end._x_tar) -  rb_math::get_P_3x1(blend_mid._x_tar)).norm()) * pos_dev_;
            if(rot_mode != 1){
                int num_of_iter_deg = (rb_math::R_to_RCV(rb_math::get_R_3x3(blend_mid._x_tar) * rb_math::get_R_3x3(blend_sta._x_tar).transpose()).norm() * MATH_R2D
                                        + rb_math::R_to_RCV(rb_math::get_R_3x3(blend_end._x_tar) * rb_math::get_R_3x3(blend_mid._x_tar).transpose()).norm() * MATH_R2D) * ang_dev_;
                blend_number = rb_math::return_big(blend_number, num_of_iter_deg);
            }
            blend_number = rb_math::saturation_Low(blend_number, 10);
            // blend_end -> t_nex
            int line_number_2 = (rb_math::get_P_3x1(t_nex._x_tar) -  rb_math::get_P_3x1(blend_end._x_tar)).norm() * pos_dev_;
            if(rot_mode != 1){
                int num_of_iter_deg = rb_math::R_to_RCV(rb_math::get_R_3x3(t_nex._x_tar) * rb_math::get_R_3x3(blend_end._x_tar).transpose()).norm() * MATH_R2D * ang_dev_;
                line_number_2 = rb_math::return_big(line_number_2, num_of_iter_deg);
            }
            line_number_2 = rb_math::saturation_Low(line_number_2, 10);

            for(int b = 1; b <= line_number_1; ++b){
                double blend_rate = ((double)(b-1))/((double)(line_number_1-1));
                pth_datas.push_back(Blend_LB_Data(t_old, blend_sta, blend_rate));
            }
            for(int b = 1; b <= blend_number; ++b){
                double blend_rate = ((double)(b-1))/((double)(blend_number-1));
                lb_data_struct temp_sm = Blend_LB_Data(blend_sta, blend_mid, blend_rate);
                lb_data_struct temp_me = Blend_LB_Data(blend_mid, blend_end, blend_rate);
                pth_datas.push_back(Blend_LB_Data(temp_sm, temp_me, blend_rate));
            }
            for(int b = 1; b <= line_number_2; ++b){
                double blend_rate = ((double)(b-1))/((double)(line_number_2-1));
                pth_datas.push_back(Blend_LB_Data(blend_end, t_nex, blend_rate));
            }
        }
    }

    // -----------------------------------------------
    // Make Filtered data
    // pth_datas -> fil_datas
    // -----------------------------------------------
    int max_horizon = filter_window;
    int window_size = pth_datas.size();
    std::deque<lb_data_struct> fil_datas; fil_datas.clear();
    for(int i = 0; i < window_size; ++i){
        int target_horizon = 0;
        for(int h = 0; h <= max_horizon; ++h){
            int temp_horizon = max_horizon - h;
            if((i - temp_horizon) >= 0 && (i + temp_horizon) < window_size){
                target_horizon = temp_horizon;
                break;
            }
        }

        std::deque<lb_data_struct> avg_buf; avg_buf.clear();
        for(int h = (i - target_horizon); h <= (i + target_horizon); ++ h){
            avg_buf.push_back(pth_datas.at(h));
        }
        fil_datas.push_back(get_LB_Average(avg_buf, pth_datas.at(i)));
    }

    // -----------------------------------------------
    // Calc Travel Distance
    // fil_datas -> travel_distance
    // -----------------------------------------------
    double travel_distance = 0.;
    double travel_vel = 0.;
    double travel_acc_1 = ref_datas.at(0)._x_acc[0];
    double travel_acc_3 = ref_datas.at(ref_datas.size() - 1)._x_acc[0];
    for(int i = 0; i < fil_datas.size(); ++i){
        if(i != 0){
            travel_distance += (rb_math::get_P_3x1(fil_datas.at(i)._x_tar) - rb_math::get_P_3x1(fil_datas.at(i - 1)._x_tar)).norm();
        }
        travel_vel += fabs(fil_datas.at(i)._x_vel[0]);
    }
    travel_vel /= static_cast<double>(fil_datas.size());
    auto [travel_t1, travel_t2, travel_t3] = rb_math::Calc_Trapizoidal_Non_Symetric(travel_distance, travel_vel, travel_acc_1, travel_acc_3);
    travel_vel = travel_t1 * travel_acc_1;
    double travel_Time = travel_t1 + travel_t2 + travel_t3;

    // -----------------------------------------------
    // Make TimeLine
    // fil_datas -> time_datas
    // -----------------------------------------------
    time_datas.clear();
    time_datas.push_back(fil_datas.at(0));

    for(double t = RT_PERIOD_SEC; t <= (travel_Time + 1e-9); t += sampling_time){
        // 3-1. 누적 이동거리 traveled
        double traveled = 0;
        if(t < travel_t1){
            traveled = 0.5 * travel_acc_1 * t * t;
        }else if(t < (travel_t1 + travel_t2)){
            traveled = 0.5 * travel_acc_1 * travel_t1 * travel_t1 + travel_vel * (t - travel_t1);
        }else{
            double td = travel_Time - t;
            traveled = travel_distance - 0.5 * travel_acc_3 * td * td;
        }

        // 3-2. traveled 에 해당하는 점 찾기
        double accum = 0.0;
        size_t segIdx = 0;
        for(size_t i = 0; i < fil_datas.size()-1; i++){
            double segLen = (rb_math::get_P_3x1(fil_datas.at(i + 1)._x_tar) - rb_math::get_P_3x1(fil_datas.at(i + 0)._x_tar)).norm();
            if(accum + segLen >= traveled){
                segIdx = i;
                break;
            }
            accum += segLen;
        }

        // 3-3. 해당 세그먼트 내 보간
        double segLen = (rb_math::get_P_3x1(fil_datas.at(segIdx + 1)._x_tar) - rb_math::get_P_3x1(fil_datas.at(segIdx + 0)._x_tar)).norm();
        double ratio = (segLen > 1e-9) ? (traveled - accum) / segLen : 0.0;
        if(ratio < 0.)  std::cout<<"ratio under 0"<<std::endl;
        if(ratio > 1.)  std::cout<<"ratio over  1"<<std::endl;
        time_datas.push_back(Blend_LB_Data(fil_datas.at(segIdx), fil_datas.at(segIdx + 1), ratio));
    }
    time_datas.push_back(fil_datas.at(fil_datas.size() - 1));

    time_datas_mid_vel = travel_vel;
    time_total = travel_Time;
    is_FirstLoop = true;
    timer = 0;
    time_speed_alpha = 1.;

    return MSG_OK;
}

void move_lb::Update_Timer(double dt){
    timer += (dt * time_speed_alpha);
}

double move_lb::Get_Timer(){
    return timer;
}

move_lb_control_ret move_lb::Control(double _time){
    if(_time < 0.){
        _time = timer;
    }
    
    move_lb_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;

    if(_time < time_total){
        double _idx_d = _time / sampling_time;
        int    _idx_i = (int)_idx_d;
        if(_idx_i == (time_datas.size() - 1)){
            ret.is_finished = true;
            ret.L_output = time_datas.at(_idx_i)._x_tar;
        }else{
            double p_rate = _idx_d - ((double)_idx_i);
            lb_data_struct temp_out = Blend_LB_Data(time_datas.at(_idx_i), time_datas.at(_idx_i + 1), p_rate);
            ret.L_output = temp_out._x_tar;
            time_speed_alpha = temp_out._x_vel[0] / time_datas_mid_vel;
        }
    }else{
        ret.is_finished = true;
        ret.L_output = time_datas.at(time_datas.size() - 1)._x_tar;
    }

    if(time_rot_mode == 1){
        ret.L_output.block(3, 0, 3, 1) = time_datas.at(0)._x_tar.block(3, 0, 3, 1);
    }

    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;

    return ret;
}

lb_data_struct move_lb::Blend_LB_Data(lb_data_struct d1, lb_data_struct d2, double alpha){
    lb_data_struct ret = d1;

    ret._x_tar      = rb_math::Blend_Carte(d1._x_tar, d2._x_tar, alpha);
    ret._x_vel[0]   = rb_math::Blend_NUM(d1._x_vel[0], d2._x_vel[0], alpha);
    ret._x_vel[1]   = rb_math::Blend_NUM(d1._x_vel[1], d2._x_vel[1], alpha); 
    ret._x_acc[0]   = rb_math::Blend_NUM(d1._x_acc[0], d2._x_acc[0], alpha);
    ret._x_acc[1]   = rb_math::Blend_NUM(d1._x_acc[1], d2._x_acc[1], alpha); 
    ret._m_indexing = rb_math::Blend_NUM(d1._m_indexing, d2._m_indexing, alpha);
    ret._b_par = 1;
    ret._m_type = -1;

    return ret;
}

lb_data_struct move_lb::get_LB_Average(const std::deque<lb_data_struct>& buffer, lb_data_struct center_value){
    lb_data_struct ret = center_value;

    std::deque<VectorCd> dq_x_tar;
    std::deque<double> dq_x_vel_0;
    std::deque<double> dq_x_vel_1;
    std::deque<double> dq_x_acc_0;
    std::deque<double> dq_x_acc_1;
    for (const auto& t : buffer) {
        dq_x_tar.push_back(t._x_tar);
        dq_x_vel_0.push_back(t._x_vel[0]);
        dq_x_vel_1.push_back(t._x_vel[1]);
        dq_x_acc_0.push_back(t._x_acc[0]);
        dq_x_acc_1.push_back(t._x_acc[1]);
    }

    ret._x_tar = rb_math::get_Average_VectorCd(dq_x_tar);
    ret._x_vel[0] = rb_math::get_Average_Number(dq_x_vel_0);
    ret._x_vel[1] = rb_math::get_Average_Number(dq_x_vel_1);
    ret._x_acc[0] = rb_math::get_Average_Number(dq_x_acc_0);
    ret._x_acc[1] = rb_math::get_Average_Number(dq_x_acc_1);

    return ret;
}