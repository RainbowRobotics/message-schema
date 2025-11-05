#define P_NAME  "MOVE_SPEED_L"

#include "move_speed_l.h"
#include "message.h"
#include "iostream"

move_speed_l::move_speed_l()
{
    ;
}

move_speed_l::~move_speed_l(){
    ;
}

int move_speed_l::Init(VectorCd c_sta, double acc_pos, double acc_rot){
    prev_Pos = rb_math::get_P_3x1(c_sta);
    prev_Rmat = rb_math::get_R_3x3(c_sta);
    prev_Aa = rb_math::get_REDUN_1x1(c_sta);

    speed_l_acc_pos = acc_pos;
    speed_l_acc_rcv = acc_rot * MATH_D2R;
    speed_l_acc_aaa = acc_rot * MATH_D2R;

    speed_l_pos_vel = Eigen::Vector3d::Zero(3, 1);
    speed_l_rcv_vel = Eigen::Vector3d::Zero(3, 1);
    speed_l_aaa_vel = 0;

    is_FirstLoop = true;
    is_BreakCall = false;
    internal_scaler = 1.;
    return MSG_OK;
}

int move_speed_l::Set_Taget_Velocity(VectorCd carte_speed_per_sec){
    if(is_BreakCall){
        return MSG_OK;
    }
    speed_l_pos_vel_tar = rb_math::get_P_3x1(carte_speed_per_sec);
    speed_l_rcv_vel_tar = rb_math::get_E_3x1(carte_speed_per_sec) * MATH_D2R;
    speed_l_aaa_vel_tar = rb_math::get_REDUN_1x1(carte_speed_per_sec) * MATH_D2R;
    return MSG_OK;
}

void move_speed_l::Update_Internal_Scaler(double alpha){
    internal_scaler = alpha;
}

move_speed_l_control_ret move_speed_l::Control(bool is_break, double break_alpha){
    move_speed_l_control_ret ret;

    ret.is_finished = false;
    ret.is_thereErr = 0;
    
    if(is_break){
        if(!is_BreakCall){
            std::cout<<"Reset Speed "<<speed_l_pos_vel.norm()<<std::endl;
            is_BreakCall = true;
            speed_l_pos_vel_tar = speed_l_pos_vel;
            speed_l_rcv_vel_tar = speed_l_rcv_vel;
            speed_l_aaa_vel_tar = speed_l_aaa_vel;
        }
        speed_l_pos_vel = speed_l_pos_vel_tar * break_alpha;
        speed_l_rcv_vel = speed_l_rcv_vel_tar * break_alpha;
        speed_l_aaa_vel = speed_l_aaa_vel_tar * break_alpha;
    }else{
        {
            Eigen::Vector3d del_tar_cur = (speed_l_pos_vel_tar * internal_scaler) - speed_l_pos_vel;
            double del_tar_cur_size = del_tar_cur.size();
            Eigen::Vector3d vec_tar_cur = Eigen::Vector3d::Zero(3, 1);
            if(del_tar_cur_size > 1e-7){
                vec_tar_cur = del_tar_cur / del_tar_cur_size;
            }

            Eigen::Vector3d acc_tar_cur = vec_tar_cur * rb_math::saturation_Up(del_tar_cur_size, (speed_l_acc_pos * RT_PERIOD_SEC));
            speed_l_pos_vel += acc_tar_cur;
        }
        {
            Eigen::Vector3d del_tar_cur = (speed_l_rcv_vel_tar * internal_scaler) - speed_l_rcv_vel;
            double del_tar_cur_size = del_tar_cur.size();
            Eigen::Vector3d vec_tar_cur = Eigen::Vector3d::Zero(3, 1);
            if(del_tar_cur_size > 1e-7){
                vec_tar_cur = del_tar_cur / del_tar_cur_size;
            }

            Eigen::Vector3d acc_tar_cur = vec_tar_cur * rb_math::saturation_Up(del_tar_cur_size, (speed_l_acc_rcv * RT_PERIOD_SEC));
            speed_l_rcv_vel += acc_tar_cur;
        }
        {
            double del_tar_cur = (speed_l_aaa_vel_tar * internal_scaler) - speed_l_aaa_vel;
            double del_tar_cur_size = fabs(del_tar_cur);
            double vec_tar_cur = rb_math::sign(del_tar_cur);
            double acc_tar_cur = vec_tar_cur * rb_math::saturation_Up(fabs(del_tar_cur), (speed_l_acc_aaa * RT_PERIOD_SEC));
            speed_l_aaa_vel += acc_tar_cur;
        }
    }

    // std::cout<<"speed_l_pos_vel: "<<speed_l_pos_vel.norm()<<" / "<<internal_scaler<<std::endl;

    prev_Pos += (speed_l_pos_vel * RT_PERIOD_SEC);
    Eigen::Matrix3d temp_R = rb_math::RCV_to_R(speed_l_rcv_vel * RT_PERIOD_SEC) * prev_Rmat;
    Eigen::JacobiSVD<Eigen::Matrix3d> t_svd(temp_R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    prev_Rmat = t_svd.matrixU() * t_svd.matrixV().transpose();
    prev_Aa  += (speed_l_aaa_vel * MATH_R2D * RT_PERIOD_SEC);

    #if NO_OF_JOINT == 7
        ret.L_output = rb_math::Make_C_from_PandR(prev_Pos, prev_Rmat, prev_Aa);
    #else
        ret.L_output = rb_math::Make_C_from_PandR(prev_Pos, prev_Rmat);
    #endif

    ret.is_First = is_FirstLoop;
    is_FirstLoop = false;

    return ret;
}