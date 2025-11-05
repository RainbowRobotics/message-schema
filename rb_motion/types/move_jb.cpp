#define P_NAME  "MOVE_JB"

#include "move_jb.h"

#include "iostream"
#include "message.h"

move_jb::move_jb()
{
    ;
}

move_jb::~move_jb(){
    ;
}

void move_jb::Clear(VectorJd j_sta){
    ref_datas.clear();

    jb_data_struct temp_line;
    temp_line._j_tar = j_sta;
    temp_line._j_vel = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._j_acc = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._b_opt = 0;
    temp_line._b_par = 0;
    // dummy for not init warning
    temp_line._j_pvel = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._j_pacc = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._t_pseg[0] = temp_line._t_pseg[1] = temp_line._t_pseg[2] = 0.;
    ref_datas.push_back(temp_line);
}

int move_jb::Add(VectorJd j_tar, VectorJd j_vel, VectorJd j_acc, int blend_option, float blend_para){
    if(blend_option == 0){
        blend_para = rb_math::saturation_L_and_U(blend_para, 0, 1);
    }else if(blend_option == 1){
        blend_para = rb_math::saturation_Low(blend_para, 0);
    }else{
        blend_option = 0;
        blend_para = 1.0;
    }

    jb_data_struct temp_line;
    temp_line._j_tar = j_tar;
    temp_line._j_vel = j_vel;
    temp_line._j_acc = j_acc;
    temp_line._b_opt = blend_option;
    temp_line._b_par = blend_para;
    // dummy for not init warning
    temp_line._j_pvel = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._j_pacc = VectorJd::Zero(NO_OF_JOINT, 1);
    temp_line._t_pseg[0] = temp_line._t_pseg[1] = temp_line._t_pseg[2] = 0.;
    
    ref_datas.push_back(temp_line);
    return 0;
}

int move_jb::Init(VectorJd Limit_Vel){

    if(ref_datas.size() < 2){
        return MSG_MOVE_POINT_NUM_ERR;
    }
    for(int i = 1; i < (int)ref_datas.size(); ++i){
        jb_data_struct prev_line = ref_datas.at(i - 1);
        jb_data_struct temp_line = ref_datas.at(i);

        double t1_sum = 1e-6; double t2_sum = 1e-6; double t3_sum = 1e-6;
        for(int k = 0; k < NO_OF_JOINT; ++k){
            auto [a1, a2, a3, t1, t2, t3, ev, mv] = rb_math::Calc_Trapizoidal(temp_line._j_vel(k), temp_line._j_acc(k), fabs(temp_line._j_tar(k) - prev_line._j_tar(k)));
            if(t1 > t1_sum) t1_sum = t1;
            if(t2 > t2_sum) t2_sum = t2;
            if(t3 > t3_sum) t3_sum = t3;
        }

        temp_line._t_pseg[0] = t1_sum;
        temp_line._t_pseg[1] = t2_sum;
        temp_line._t_pseg[2] = t3_sum;
        for(int k = 0; k < NO_OF_JOINT; ++k){
            temp_line._j_pvel(k) = (temp_line._j_tar(k) - prev_line._j_tar(k)) / (t1_sum + t2_sum);
            temp_line._j_pacc(k) = temp_line._j_pvel(k) / t1_sum;
        }
        ref_datas.at(i) = temp_line;
    }

    
    double TLast[NO_OF_JOINT];
    for(int k = 0; k < NO_OF_JOINT; ++k){
        runtime_datas[k].clear();
        JLast[k] = ref_datas.at(0)._j_tar(k);
        TLast[k] = 0.;
    }

    for(int i = 1; i < (int)ref_datas.size(); ++i){
        jb_data_struct c_line = ref_datas.at(i);
        jb_data_struct n_line = ref_datas.at(i);
        if(i != ((int)ref_datas.size() - 1)){
            n_line = ref_datas.at(i + 1);
        }

        double c_t1 = c_line._t_pseg[0];
        double c_t2 = c_line._t_pseg[1];
        double c_t3 = c_line._t_pseg[2];
        double n_t1 = n_line._t_pseg[0];
        double n_t2 = n_line._t_pseg[1];

        bool is_there_overlap_max_vel = false;
        for(int k = 0; k < NO_OF_JOINT; ++k){
            if(fabs(c_line._j_pvel(k) + n_line._j_pvel(k)) > fabs(Limit_Vel(k))){
                is_there_overlap_max_vel = true;
            }
        }

        double c_max_blend_T = c_t2 / 2. + c_t3;
        double n_max_blend_T = n_t1 + n_t2 / 2.;
        if(is_there_overlap_max_vel){
            c_max_blend_T = c_t3;
            n_max_blend_T = n_t1;
        }
        double blend_Time = rb_math::return_small(c_max_blend_T, n_max_blend_T);
        if(c_line._b_opt == 0){
            blend_Time *= (0.95 * c_line._b_par);
        }else{
            blend_Time *= (0.95 * 0.5);
        }
        
        for(int k = 0; k < NO_OF_JOINT; ++k){
            double c_aa = c_line._j_pacc(k);
            double c_vv = c_line._j_pvel(k);
            double n_aa = n_line._j_pacc(k);
            double n_vv = n_line._j_pvel(k);

            if(i == 1){
                jb_runtime_struct temp;
                // push first
                temp.t_sta = 0;
                temp.t_end = c_t1;
                temp.f_A = c_aa;
                temp.f_V = 0;
                temp.f_P = JLast[k];
                temp.seg_index = i;
                runtime_datas[k].push_back(temp);
                JLast[k] += (0.5 * c_aa * c_t1 * c_t1);
                // push midde
                temp.t_sta = c_t1;
                temp.t_end = c_t1 + c_t2 / 2.;
                temp.f_A = 0;
                temp.f_V = c_vv;
                temp.f_P = JLast[k];
                temp.seg_index = i;
                runtime_datas[k].push_back(temp);
                JLast[k] += (c_vv * c_t2 / 2.);
                //
                double st = c_t2 / 2. + c_t3 - blend_Time;
                std::deque<double> timing;   //SWAP//modi230729
                timing.push_back(c_t2 / 2.);  timing.push_back(c_t2 / 2.+ c_t3); timing.push_back(st);
                timing.push_back(st + n_t1);  timing.push_back(st + n_t1 + n_t2 / 2.);
                sort(timing.begin(), timing.end());
                for(unsigned int ss=0; ss<timing.size(); ++ss){
                    double ts = 0;
                    if(ss != 0)
                        ts = timing.at(ss - 1);
                    double te = timing.at(ss);
                    double tm = (ts + te) / 2.;

                    double A1, B1, A2, B2 = 0.;
                    if(tm < (c_t2 / 2.)){
                        A1 = 0;
                        B1 = c_vv - A1 * 0;
                    }else if(tm < (c_t2 / 2.+c_t3)){
                        A1 = -c_aa;
                        B1 = c_vv - A1 * (c_t2 / 2.);
                    }else{
                        A1 = B1 = 0;
                    }

                    if(tm < st){
                        A2 = B2 = 0;
                    }else if(tm < (st + n_t1)){
                        A2 = n_aa;
                        B2 = 0 - A2 * (st);
                    }else{
                        A2 = 0;
                        B2 = n_vv - A2 * (st + n_t1);
                    }
                    double AT = A1 + A2;
                    double BT = B1 + B2;

                    double aaa = AT;
                    double vvv = AT * ts + BT;
                    double ppp = JLast[k];
                    double toff = c_t1 + c_t2 / 2.;
                    temp.t_sta = ts + toff;
                    temp.t_end = te + toff;
                    temp.f_A = aaa;
                    temp.f_V = vvv;
                    temp.f_P = ppp;
                    if(ss < 2){
                        temp.seg_index = i;
                    }else{
                        temp.seg_index = i + 1;
                    }
                    runtime_datas[k].push_back(temp);
                    JLast[k] = 0.5 * aaa * (te - ts) * (te - ts) + vvv * (te - ts) + ppp;
                    TLast[k] = te + toff;
                }

                timing.clear();     std::deque<double>().swap(timing);
            }else if(i == ((int)ref_datas.size() - 1)){
                jb_runtime_struct temp;
                temp.t_sta = TLast[k];
                temp.t_end = TLast[k] + c_t2 / 2.;
                temp.f_A = 0.;
                temp.f_V = c_vv;
                temp.f_P = JLast[k];
                temp.seg_index = i;
                runtime_datas[k].push_back(temp);
                JLast[k] += (c_vv * c_t2 / 2.);

                temp.t_sta = TLast[k] + c_t2 / 2.;
                temp.t_end = TLast[k] + c_t2 / 2. + c_t3;
                temp.f_A = -c_aa;
                temp.f_V = c_vv;
                temp.f_P = JLast[k];
                temp.seg_index = i;
                runtime_datas[k].push_back(temp);
                JLast[k] += (c_vv * c_t3 + 0.5 * (-c_aa) * c_t3 * c_t3);
                TLast[k] += (c_t2 / 2. + c_t3);
            }else{
                double st = c_t2 / 2. + c_t3 - blend_Time;
                std::deque<double> timing;       //SWAP//modi230729
                timing.push_back(c_t2 / 2.);  timing.push_back(c_t2 / 2. + c_t3); timing.push_back(st);
                timing.push_back(st + n_t1);  timing.push_back(st + n_t1 + n_t2 / 2.);
                sort(timing.begin(), timing.end());
                unsigned int how_much_seg = timing.size();
                for(unsigned int ss = 0; ss < how_much_seg; ++ss){
                    double ts = 0;
                    if(ss != 0)
                        ts = timing.at(ss-1);
                    double te = timing.at(ss);
                    double tm = (ts + te) / 2.;
                    double A1, B1, A2, B2 = 0.;
                    if(tm < (c_t2/2.)){
                        A1 = 0;
                        B1 = c_vv - A1 * 0;
                    }else if(tm < (c_t2/2.+c_t3)){
                        A1 = -c_aa;
                        B1 = c_vv - A1 * (c_t2 / 2.);
                    }else{
                        A1 = B1 = 0;
                    }

                    if(tm < st){
                        A2 = B2 = 0;
                    }else if(tm < (st + n_t1)){
                        A2 = n_aa;
                        B2 = 0 - A2 * (st);
                    }else{
                        A2 = 0;
                        B2 = n_vv - A2 * (st + n_t1);
                    }
                    double AT = A1 + A2;
                    double BT = B1 + B2;

                    double aaa = AT;
                    double vvv = AT * ts + BT;
                    double ppp = JLast[k];

                    jb_runtime_struct temp;
                    temp.t_sta = ts + TLast[k];
                    temp.t_end = te + TLast[k];
                    temp.f_A = aaa;
                    temp.f_V = vvv;
                    temp.f_P = ppp;
                    if(ss < 2){
                        temp.seg_index = i;
                    }else{
                        temp.seg_index = i + 1;
                    }
                    runtime_datas[k].push_back(temp);
                    JLast[k] = 0.5 * aaa * (te - ts) * (te - ts) + vvv * (te - ts) + ppp;

                    if(ss == (how_much_seg-1)){
                        TLast[k] += te;
                    }
                }
                timing.clear();     std::deque<double>().swap(timing);
            }
        }
    }

    bool is_size_matching = true;
    bool is_max_vel_ok = true;
    bool is_end_point_matching = true;
    bool is_tlast_matching = true;
    for(int k = 0; k < NO_OF_JOINT; k++){
        if(runtime_datas[0].size() != runtime_datas[k].size()){
            is_size_matching = false;
        }

        for(unsigned int li = 0; li < runtime_datas[k].size(); li++){
            if(fabs(runtime_datas[k].at(li).f_V) > fabs(Limit_Vel[k])){
                is_max_vel_ok = false;
                std::cout<<"over vel at : "<<li<<" and "<<k<<"because "<<runtime_datas[k].at(li).f_V<<std::endl;
            }
        }

        if(fabs(JLast[k] - ref_datas.at(ref_datas.size() - 1)._j_tar(k)) > 0.03){
            is_end_point_matching = false;
        }
        if(fabs(TLast[0] - TLast[k]) > 0.001){
            is_tlast_matching = false;
        }
    }
    double total_motion_time = (TLast[0] + TLast[1] + TLast[2] + TLast[3] + TLast[4] + TLast[5]) / 6.;

    ref_datas.clear();     std::deque<jb_data_struct>().swap(ref_datas);

    if(!is_size_matching)       return MSG_MOVE_PATH_GEN_ERR_SIZE_MATCH;
    if(!is_max_vel_ok)          return MSG_MOVE_PATH_GEN_ERR_VEL_CHECK;
    if(!is_end_point_matching)  return MSG_MOVE_PATH_GEN_ERR_LAST_POINT_MATCH;
    if(!is_tlast_matching)      return MSG_MOVE_PATH_GEN_ERR_LAST_TIME_MATCH;


    time_total = total_motion_time;
    timer = 0;
    return MSG_OK;
}

void move_jb::Update_Timer(double dt){
    timer += dt;
}

double move_jb::Get_Timer(){
    return timer;
}

move_jb_control_ret move_jb::Control(double _time){
    if(_time < 0.){
        _time = timer;
    }
    
    move_jb_control_ret ret;
    ret.is_thereErr = 0;
    ret.is_finished = false;

    bool is_found = false;
    unsigned int targ_idx = 0;
    for (unsigned int li = 0; li < runtime_datas[0].size(); li++){
        if(_time <= runtime_datas[0].at(li).t_end){
            targ_idx = li;
            is_found = true;
            break;
        }
    }
    if(!is_found){
        ret.is_finished = true;
    }

    if(ret.is_finished == true){
        for(int j = 0; j < NO_OF_JOINT; ++j){
            ret.J_output(j) = JLast[j];
        }
        ret.passing_index = runtime_datas[0].at(runtime_datas[0].size() - 1).seg_index + 1;
    }else{
        double seg_t = _time - runtime_datas[0].at(targ_idx).t_sta;
        for(int j = 0; j < NO_OF_JOINT; ++j){
            double my_a = runtime_datas[j].at(targ_idx).f_A;
            double my_v = runtime_datas[j].at(targ_idx).f_V;
            double my_p = runtime_datas[j].at(targ_idx).f_P;
            ret.J_output(j) = 0.5 * my_a * seg_t * seg_t + my_v * seg_t + my_p;
        }
        ret.passing_index = runtime_datas[0].at(targ_idx).seg_index;
    }
    return ret;
}