#define P_NAME  "MATH"

#include <iostream>

#include "rmath.h"
#include <deque>


namespace rb_math {
    namespace {
        std::deque<TEST_SAVE_STRUCT> saving_datas;
    }
    Eigen::Matrix3d RPY_to_R(Eigen::Vector3d rpy_deg){
        Eigen::Matrix3d ret = Rz(rpy_deg(2) * MATH_D2R) * Ry(rpy_deg(1) * MATH_D2R) * Rx(rpy_deg(0) * MATH_D2R);
        return ret;
    }

    Eigen::Matrix3d RPY_to_R(double rx_deg, double ry_deg, double rz_deg){
        Eigen::Matrix3d ret = Rz(rz_deg * MATH_D2R) * Ry(ry_deg * MATH_D2R) * Rx(rx_deg * MATH_D2R);
        return ret;
    }

    Eigen::Matrix3d Rx(double rad){
        Eigen::Matrix3d _Rx;

        _Rx(0,0) = 1.0;
        _Rx(0,1) = 0.0;
        _Rx(0,2) = 0.0;

        _Rx(1,0) = 0.0;
        _Rx(1,1) = cos(rad);
        _Rx(1,2) = -sin(rad);

        _Rx(2,0) = 0.0;
        _Rx(2,1) = sin(rad);
        _Rx(2,2) = cos(rad);

        return _Rx;
    }

    Eigen::Matrix3d Ry(double rad){
        Eigen::Matrix3d _Ry;

        _Ry(0,0) = cos(rad);
        _Ry(0,1) = 0.0;
        _Ry(0,2) = sin(rad);

        _Ry(1,0) = 0.0;
        _Ry(1,1) = 1.0;
        _Ry(1,2) = 0.0;

        _Ry(2,0) = -sin(rad);
        _Ry(2,1) = 0.0;
        _Ry(2,2) = cos(rad);

        return _Ry;
    }

    Eigen::Matrix3d Rz(double rad){
        Eigen::Matrix3d _Rz;

        _Rz(0,0) = cos(rad);
        _Rz(0,1) = -sin(rad);
        _Rz(0,2) = 0.0;

        _Rz(1,0) = sin(rad);
        _Rz(1,1) = cos(rad);
        _Rz(1,2) = 0.0;

        _Rz(2,0) = 0.0;
        _Rz(2,1) = 0.0;
        _Rz(2,2) = 1.0;

        return _Rz;
    }

    Eigen::Matrix3d RCV_to_R(Eigen::Vector3d trcv){
        Eigen::Matrix3d tR = Eigen::Matrix3d::Identity(3, 3);
        double angle = trcv.norm();
        if(angle <1e-7)
            angle = 1e-7;
        Eigen::Vector3d axis = Eigen::Vector3d(0., 0., 0.);
        axis = trcv / angle;

        tR(0, 0) = axis(0,0)*axis(0,0)*(1.0-cos(angle)) + cos(angle);
        tR(0, 1) = axis(0,0)*axis(1,0)*(1.0-cos(angle)) - axis(2,0)*sin(angle);
        tR(0, 2) = axis(0,0)*axis(2,0)*(1.0-cos(angle)) + axis(1,0)*sin(angle);

        tR(1, 0) = axis(0,0)*axis(1,0)*(1.0-cos(angle)) + axis(2,0)*sin(angle);
        tR(1, 1) = axis(1,0)*axis(1,0)*(1.0-cos(angle)) + cos(angle);
        tR(1, 2) = axis(1,0)*axis(2,0)*(1.0-cos(angle)) - axis(0,0)*sin(angle);

        tR(2, 0) = axis(0,0)*axis(2,0)*(1.0-cos(angle)) - axis(1,0)*sin(angle);
        tR(2, 1) = axis(1,0)*axis(2,0)*(1.0-cos(angle)) + axis(0,0)*sin(angle);
        tR(2, 2) = axis(2,0)*axis(2,0)*(1.0-cos(angle)) + cos(angle);

        return tR;
    }

    Eigen::Vector3d R_to_RPY(Eigen::Matrix3d inR){
        Eigen::Vector3d _RPY = Eigen::Vector3d(0.,0.,0.);

        double singul_flag = sqrt(inR(0,0)*inR(0,0) + inR(1,0)*inR(1,0));
        if(singul_flag < 1e-6){
            _RPY(2) = 0.;
            _RPY(1) = atan2(-inR(2,0), singul_flag) * MATH_R2D;
            _RPY(0) = atan2(-inR(1,2), inR(1,1)) * MATH_R2D;
        }else{
            _RPY(2) = atan2(inR(1,0), inR(0,0)) * MATH_R2D;
            _RPY(1) = atan2(-inR(2,0), singul_flag) * MATH_R2D;
            _RPY(0) = atan2(inR(2,1), inR(2,2)) * MATH_R2D;
        }
        return _RPY;
    }

    Eigen::Vector3d R_to_RCV(Eigen::Matrix3d tR){
        Eigen::Vector3d tRCV = Eigen::Vector3d(0., 0., 0.);
        Eigen::AngleAxisd vvv = Eigen::AngleAxisd(tR);

        double c_ang = vvv.angle() * MATH_R2D;
        double c_aX = vvv.axis()(0);
        double c_aY = vvv.axis()(1);
        double c_aZ = vvv.axis()(2);

        if(c_ang < 0. || c_ang > 360.){
            std::cout<<"UNEXPECTED SITUATION in RtoRCV !!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        }

        if(c_ang > 180.){
            c_ang = c_ang - 360.;
            tRCV(0) = c_aX * c_ang * MATH_D2R;
            tRCV(1) = c_aY * c_ang * MATH_D2R;
            tRCV(2) = c_aZ * c_ang * MATH_D2R;
        }else{
            tRCV(0) = c_aX * c_ang * MATH_D2R;
            tRCV(1) = c_aY * c_ang * MATH_D2R;
            tRCV(2) = c_aZ * c_ang * MATH_D2R;

        }

        return tRCV;
    }

    Eigen::Vector3d Blend_Pos(Eigen::Vector3d p1, Eigen::Vector3d p2, double alpha){
        return ((1. - alpha) * p1 + alpha * p2);
    }

    Eigen::Vector3d Blend_Rcv(Eigen::Vector3d r1, Eigen::Vector3d r2, double alpha){
        Eigen::Vector3d delvec = R_to_RCV(RCV_to_R(r2) * RCV_to_R(r1).transpose());
        return R_to_RCV(RCV_to_R(delvec * alpha) * RCV_to_R(r1));
    }

    Eigen::Matrix3d Blend_Rmat(Eigen::Matrix3d R1, Eigen::Matrix3d R2, double alpha){
        return RCV_to_R(Blend_Rcv(R_to_RCV(R1), R_to_RCV(R2), alpha));
    }

    Eigen::Vector3d Blend_Euler(Eigen::Vector3d E1, Eigen::Vector3d E2, double alpha){
        return R_to_RPY(Blend_Rmat(RPY_to_R(E1), RPY_to_R(E2), alpha));
    }

    double Blend_NUM(double a1, double a2, double alpha){
        return ((1.- alpha) * a1 + alpha * a2);
    }

    VectorJd Blend_Joint(VectorJd j1, VectorJd j2, double alpha){
        return ((1. - alpha) * j1 + alpha * j2);
    }

    VectorCd Blend_Carte(VectorCd c1, VectorCd c2, double alpha){//7DOF
        #if NO_OF_JOINT == 7
            Eigen::Vector3d tP = Blend_Pos(get_P_3x1(c1), get_P_3x1(c2), alpha);
            Eigen::Vector3d tE = Blend_Euler(get_E_3x1(c1), get_E_3x1(c2), alpha);
            double tA = Blend_NUM(get_REDUN_1x1(c1), get_REDUN_1x1(c2), alpha);
            VectorCd ret = c1;
            ret.block(0, 0, 3, 1) = tP;
            ret.block(3, 0, 3, 1) = tE;
            ret(6) = tA;
            return ret;
        #else
            Eigen::Vector3d tP = Blend_Pos(get_P_3x1(c1), get_P_3x1(c2), alpha);
            Eigen::Vector3d tE = Blend_Euler(get_E_3x1(c1), get_E_3x1(c2), alpha);
            VectorCd ret = c1;
            ret.block(0, 0, 3, 1) = tP;
            ret.block(3, 0, 3, 1) = tE;
            return ret;
        #endif
    }

    VectorCd Saturation_Carte(VectorCd from, VectorCd to, double max_pos_change, double max_rot_change, double max_redun_change){
        VectorCd ret = from;
        double target[3] = {0., 0., 0.};
        target[0] = (get_P_3x1(to) - get_P_3x1(from)).norm();
        target[1] = R_to_RCV(get_R_3x3(to) * get_R_3x3(from).transpose()).norm() * MATH_R2D;
        target[2] = fabs(get_REDUN_1x1(to) - get_REDUN_1x1(from));
        double limit[3] = {max_pos_change, max_rot_change, max_redun_change};

        double progress_rate = 1.;
        for(int i = 0; i < 3; ++i){
            if(target[i] > limit[i] && target[i] > 1e-6){
                double temp_rate = limit[i] / target[i];
                progress_rate = return_small(temp_rate, progress_rate);
            }
        }
        ret = Blend_Carte(from, to, progress_rate);
        return ret;
    }

    Eigen::Vector3d get_P_3x1(VectorCd posture){
        return Eigen::Vector3d(posture(0), posture(1), posture(2));
    }

    Eigen::Matrix3d get_R_3x3(VectorCd posture){
        return RPY_to_R(posture(3), posture(4), posture(5));
    }

    Eigen::Vector3d get_E_3x1(VectorCd posture){
        return Eigen::Vector3d(posture(3), posture(4), posture(5));
    }

    double get_REDUN_1x1(VectorCd posture){//7DOF
        #if NO_OF_JOINT == 7
            return posture(6);
        #else
            return 0;
        #endif
    }

    #if NO_OF_JOINT == 7
    VectorCd Make_C_from_PandR(Eigen::Vector3d p, Eigen::Matrix3d R, double A){//7DOF
        VectorCd ret;
        ret.block(0, 0, 3, 1) = p;
        ret.block(3, 0, 3, 1) = R_to_RPY(R);
        ret(6) = A;
        return ret;
    }
    #else
    VectorCd Make_C_from_PandR(Eigen::Vector3d p, Eigen::Matrix3d R){
        VectorCd ret;
        ret.block(0, 0, 3, 1) = p;
        ret.block(3, 0, 3, 1) = R_to_RPY(R);
        return ret;
    }
    #endif

    TARGET_INPUT Make_Input_from_Vec3(Eigen::Vector3d t3x1){
        TARGET_INPUT ret;
        ret.target_frame = FRAME_GLOBAL;
        ret.target_unit = 0;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret.target_value[i] = 0;
        }
        ret.target_value[0] = t3x1(0);
        ret.target_value[1] = t3x1(1);
        ret.target_value[2] = t3x1(2);
        return ret;
    }
    TARGET_INPUT Make_Input_Zero(int frame){
        TARGET_INPUT ret;
        ret.target_frame = frame;
        ret.target_unit = 0;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret.target_value[i] = 0;
        }
        return ret;
    }

    std::array<float, NO_OF_CARTE> Convert_VectorCd_to_Array(VectorCd inpt_c){
        std::array<float, NO_OF_CARTE> ret;
        for(int i = 0; i < NO_OF_CARTE; ++i){
            ret[i] = inpt_c(i);
        }
        return ret;
    }

    std::array<float, NO_OF_JOINT> Convert_VectorJd_to_Array(VectorJd inpt_j){
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret[i] = inpt_j(i);
        }
        return ret;
    }

    double sign(double in){
        if(in > 0.)
            return 1.;
        else
            return -1.;
    }

    double saturation_L_and_U(double in, double low, double up){
        if(in <= low){
            return low;
        }else if(in >= up){
            return up;
        }else{
            return in;
        }
    }

    double saturation_Low(double in, double low){
        if(in <= low){
            return low;
        }else{
            return in;
        }
    }

    double saturation_Up(double in, double up){
        if(in >= up){
            return up;
        }else{
            return in;
        }
    }

    double return_small(double a, double b){
        if(a > b){
            return b;
        }else{
            return a;
        }
    }

    double return_big(double a, double b){
        if(a > b){
            return a;
        }else{
            return b;
        }
    }

    double filt_Zero(double val, double x1, double x2, double y1, double y2){
        if(fabs(x1 - x2) < 1e-6){
            if(val <= x1){
                return y1;
            }else{
                return y2;
            }
        }

        if(x1 > x2){
            double ori_x1 = x1;
            x1 = x2;
            x2 = ori_x1;
        }

        double n_val = val;
        if(val <= x1){
            n_val = y1;
        }else if(val >= x2){
            n_val = y2;
        }else{
            n_val = 0.0;
        }
        return n_val;
    }

    double filt_Line(double val, double x1, double x2, double y1, double y2){
        if(fabs(x1 - x2) < 1e-6){
            if(val <= x1){
                return y1;
            }else{
                return y2;
            }
        }

        if(x1 > x2){
            double ori_x1 = x1;
            x1 = x2;
            x2 = ori_x1;
        }

        double n_val = val;
        if(val <= x1){
            n_val = y1;
        }else if(val >= x2){
            n_val = y2;
        }else{
            double filt_a = (y2 - y1) / (x2 - x1);
            double filt_b = y2 - filt_a * x2;
            n_val = filt_a * val + filt_b;
        }
        return n_val;
    }

    double filt_Curve(double val, double x1, double x2, double y1, double y2){
        if(fabs(x1 - x2) < 1e-6){
            if(val <= x1){
                return y1;
            }else{
                return y2;
            }
        }

        if(fabs(y2 - y1) < 1e-6){
            return y1;
        }

        if(x1 > x2){
            double ori_x1 = x1;
            x1 = x2;
            x2 = ori_x1;
        }

        double n_val = val;
        if(val <= x1){
            n_val = y1;
        }else if(val >= x2){
            n_val = y2;
        }else{
            double A = fabs(y2 - y1) / 2.;
            double w = 2. * MATH_PI / (fabs(x2 - x1) * 2.);
            double C = (y2 + y1) / 2.;
            double t = (y2 - C) / A;
            if(t >= +1.)    t = +1.;
            if(t <= -1.)    t = -1.;
            double b = acos(t) - w * x2;

            n_val = A * cos(w * val + b) + C;
        }
        return n_val;
    }

    std::tuple<double, double, double> Calc_Trapizoidal_Non_Symetric(double tD, double max_V, double a1, double a3){
        double time_1 = 0;  double time_2 = 0;  double time_3 = 0;

        double temp_leng = 0.5 * max_V * max_V / a1 + 0.5 * max_V * max_V / a3;
        if(temp_leng > tD){
            double tt1 = sqrt((2.0 * tD) / a1 / (1.0 + a1/a3));
            double tt2 = 0.;
            double tt3 = a1 / a3 * tt1;

            time_1 = saturation_Low(tt1, 1e-7);
            time_2 = saturation_Low(tt2, 1e-7);
            time_3 = saturation_Low(tt3, 1e-7);
        }else{
            double tt1 = max_V / a1;
            double tt2 = fabs(tD - temp_leng) / max_V;
            double tt3 = max_V / a3;

            time_1 = saturation_Low(tt1, 1e-7);
            time_2 = saturation_Low(tt2, 1e-7);
            time_3 = saturation_Low(tt3, 1e-7);
        }
        
        return {time_1, time_2, time_3};
    }

    std::tuple<double, double, double, double, double, double, double, double> Calc_Trapizoidal(double max_V, double max_A, double tD, double start_vel, double end_vel){
        double a_1, a_2, a_3;
        double time_1, time_2, time_3;
        double end_vel_new;

        double del_v = end_vel - start_vel;
        double min_v = (end_vel + start_vel)/2.;

        double temp_leng = min_v*fabs(del_v) / max_A;

        if(temp_leng > tD){
            if(sign(del_v) > 0){
                a_1 = max_A;
                a_2 = 0;
                a_3 = 0;

                time_1 = (sqrt(start_vel*start_vel + 2.*a_1*tD) - start_vel)/a_1;
                time_2 = 0.;
                time_3 = 0.;

                end_vel_new = start_vel + a_1*time_1;
            }else{
                a_1 = 0.;
                a_2 = 0.;
                a_3 = -max_A;

                time_1 = 0.;
                time_2 = 0.;
                time_3 = (sqrt(start_vel*start_vel + 2.*a_3*tD) - start_vel)/a_3;

                end_vel_new = start_vel + a_3*time_3;
            }
        }else{
            double tt1 = (max_V - start_vel)/max_A;
            double tt3 = (max_V - end_vel)/max_A;
            double tSS = tt1*(max_V + start_vel)/2. + tt3*(max_V + end_vel)/2.;

            a_1 = max_A;
            a_2 = 0.;
            a_3 = -max_A;
            if(tD > tSS){
                time_1 = tt1;
                time_2 = (tD - tSS)/max_V;
                time_3 = tt3;
            }else{
                double temp_v = sqrt((2*max_A*tD + start_vel*start_vel + end_vel*end_vel)/2.);

                time_1 = (temp_v - start_vel)/max_A;
                time_2 = 0.;
                time_3 = (temp_v - end_vel)/max_A;
            }

            end_vel_new = end_vel;
        }

        return {a_1, a_2, a_3, time_1, time_2, time_3, end_vel_new, (start_vel + a_1 * time_1)};
    }
    
    std::tuple<double, double, double, double, double, double, double, double> Calc_Trapizoidal(double max_V, double max_A, double tD){
        return Calc_Trapizoidal(max_V, max_A, tD, 0, 0);
    }


    Circle_3P_RET fit_CircleFrom3Points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3){
        double eps = 1e-4;

        Circle_3P_RET circle;
        circle.result = false; // 기본 실패
        // 거리 체크 (너무 가까운 점)
        if ((p1 - p2).norm() < eps || (p2 - p3).norm() < eps || (p1 - p3).norm() < eps) {
            return circle; // 실패
        }
        // 법선 (회전축)
        if (((p2 - p1).cross(p3 - p1)).norm() < eps) {
            return circle; // 세 점이 거의 일직선
        }

        Eigen::Vector3d ct_p1 = p1;
        Eigen::Vector3d ct_p2 = p2;
        Eigen::Vector3d ct_p3 = p3;
        Eigen::Vector3d ct_vec12 = ct_p2 - ct_p1;
        Eigen::Vector3d ct_vec23 = ct_p3 - ct_p2;
        Eigen::Vector3d ct_vec31 = ct_p1 - ct_p3;

        Eigen::Vector3d ct_axis_1 = ct_vec12.cross(ct_vec23);
        Eigen::Vector3d ct_axis_2 = ct_vec23.cross(ct_vec31);
        Eigen::Vector3d ct_axis_3 = ct_vec31.cross(ct_vec12);

        double norm_ct_axis_1 = ct_axis_1.norm();
        double norm_ct_axis_2 = ct_axis_2.norm();
        double norm_ct_axis_3 = ct_axis_3.norm();

        if(norm_ct_axis_1 < eps || norm_ct_axis_2 < eps || norm_ct_axis_3 < eps){
            return circle;
        }

        ct_axis_1 /= norm_ct_axis_1;
        ct_axis_2 /= norm_ct_axis_2;
        ct_axis_3 /= norm_ct_axis_3;
        Eigen::Vector3d ct_axis = ct_axis_1 + ct_axis_2 + ct_axis_3;
        ct_axis = ct_axis/ct_axis.norm();

        Eigen::Vector3d ct_vec12_n = ct_axis.cross(ct_vec12);
        Eigen::Vector3d ct_vec23_n = ct_axis.cross(ct_vec23);
        Eigen::Vector3d ct_vec31_n = ct_axis.cross(ct_vec31);
        ct_vec12_n = ct_vec12_n / ct_vec12_n.norm();
        ct_vec23_n = ct_vec23_n / ct_vec23_n.norm();
        ct_vec31_n = ct_vec31_n / ct_vec31_n.norm();

        Eigen::Vector3d m1 = (ct_p1 + ct_p2)/2.;
        Eigen::Vector3d m2 = (ct_p2 + ct_p3)/2.;
        Eigen::Vector3d m3 = (ct_p3 + ct_p1)/2.;
        Eigen::Vector3d v1 = ct_vec12_n;
        Eigen::Vector3d v2 = ct_vec23_n;
        Eigen::Vector3d v3 = ct_vec31_n;

        Eigen::MatrixXd TT = Eigen::MatrixXd::Zero(9, 3);
        Eigen::MatrixXd BB = Eigen::MatrixXd::Zero(9, 1);
        Eigen::MatrixXd Q_eps = Eigen::MatrixXd::Identity(3,3);
        Q_eps *= 1e-9;
        TT.block(0, 0, 3, 1) = -v1;
        TT.block(0, 1, 3, 1) = v2;
        TT.block(3, 0, 3, 1) = -v1;
        TT.block(3, 2, 3, 1) = v3;
        TT.block(6, 1, 3, 1) = -v2;
        TT.block(6, 2, 3, 1) = v3;

        BB.block(0, 0, 3, 1) = m1-m2;
        BB.block(3, 0, 3, 1) = m1-m3;
        BB.block(6, 0, 3, 1) = m2-m3;

        Eigen::Vector3d ct_LSQ_x = (TT.transpose()*TT + Q_eps).inverse()*TT.transpose()*BB;
        Eigen::Vector3d cent_1 = m1 + ct_LSQ_x(0)*v1;
        Eigen::Vector3d cent_2 = m2 + ct_LSQ_x(1)*v2;
        Eigen::Vector3d cent_3 = m3 + ct_LSQ_x(2)*v3;
        Eigen::Vector3d ct_center = (cent_1 + cent_2 + cent_3)/3.;

        // calc radius
        double ct_radius_1 = (ct_p1 - ct_center).norm();
        double ct_radius_2 = (ct_p2 - ct_center).norm();
        double ct_radius_3 = (ct_p3 - ct_center).norm();
        double ct_radius = (ct_radius_1 + ct_radius_2 + ct_radius_3)/3.;


        if((fabs(ct_radius_1 - ct_radius_2) > 0.2) || (fabs(ct_radius_2 - ct_radius_3) > 0.2) || (fabs(ct_radius_3 - ct_radius_1) > 0.2)){
            return circle;
        }
        if(ct_radius < 0.){
            return circle;
        }

        auto GET_angle_between_vector = [&](Eigen::Vector3d center, Eigen::Vector3d axis, Eigen::Vector3d p1, Eigen::Vector3d p2){
            Eigen::Vector3d tv_1x = p1 - center;
            Eigen::Vector3d tv_2x = p2 - center;

            tv_1x /= tv_1x.norm();
            tv_2x /= tv_2x.norm();
            axis /= axis.norm();

            Eigen::Vector3d tv_1y = axis.cross(tv_1x);
            Eigen::Vector3d tv_2y = axis.cross(tv_2x);
            tv_1y /= tv_1y.norm();
            tv_2y /= tv_2y.norm();


            Eigen::Matrix3d tR_1 = Eigen::Matrix3d::Identity(3,3);
            Eigen::Matrix3d tR_2 = Eigen::Matrix3d::Identity(3,3);
            tR_1(0,0) = tv_1x(0);
            tR_1(1,0) = tv_1x(1);
            tR_1(2,0) = tv_1x(2);
            tR_1(0,1) = tv_1y(0);
            tR_1(1,1) = tv_1y(1);
            tR_1(2,1) = tv_1y(2);
            tR_1(0,2) = axis(0);
            tR_1(1,2) = axis(1);
            tR_1(2,2) = axis(2);

            tR_2(0,0) = tv_2x(0);
            tR_2(1,0) = tv_2x(1);
            tR_2(2,0) = tv_2x(2);
            tR_2(0,1) = tv_2y(0);
            tR_2(1,1) = tv_2y(1);
            tR_2(2,1) = tv_2y(2);
            tR_2(0,2) = axis(0);
            tR_2(1,2) = axis(1);
            tR_2(2,2) = axis(2);

            Eigen::Matrix3d tR_del = tR_1.transpose()*tR_2;

            double angle = atan2(-tR_del(0,1), tR_del(0,0)) * MATH_R2D;
            if(angle < 0.)
                angle += 360;

            return angle;
        };

        double ct_angle_p12 = GET_angle_between_vector(ct_center, ct_axis, ct_p1, ct_p2);
        double ct_angle_p23 = GET_angle_between_vector(ct_center, ct_axis, ct_p2, ct_p3);
        double ct_angle_tot = GET_angle_between_vector(ct_center, ct_axis, ct_p1, ct_p3);
        if(fabs(ct_angle_p12 + ct_angle_p23 - ct_angle_tot) > 1){
            return circle;
        }

        circle.angle12 = ct_angle_p12;
        circle.angle23 = ct_angle_p23;
        circle.angle13 = ct_angle_tot;
        circle.axis = ct_axis;
        circle.center = ct_center;
        circle.radius = ct_radius;
        circle.result = true;

        return circle;
    }

    double get_Average_Number(const std::deque<double>& buffer){
        double avg = 0;
        for (const auto& p : buffer) {
            avg += p;
        }
        avg /= static_cast<double>(buffer.size());
        return avg;
    }

    Eigen::Vector3d get_Average_Pvec(const std::deque<Eigen::Vector3d>& buffer) {
        Eigen::Vector3d avg = Eigen::Vector3d::Zero(3, 1);
        for (const auto& p : buffer) {
            avg += p;
        }
        avg /= static_cast<double>(buffer.size());
        return avg;
    }

    Eigen::Matrix3d get_Average_Rmat(const std::deque<Eigen::Matrix3d>& buffer) {
        // (1) 행렬 평균
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (const auto& R : buffer) {
            M += R;
        }
        M /= static_cast<double>(buffer.size());

        // (2) SVD 분해
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // (3) 직교화
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();

        // (4) det 보정 (SO(3) 보장)
        if (R.determinant() < 0) {
            Eigen::Matrix3d U = svd.matrixU();
            U.col(2) *= -1;
            R = U * svd.matrixV().transpose();
        }
        return R;
    }

    VectorCd get_Average_VectorCd(const std::deque<VectorCd>& buffer){
        std::deque<Eigen::Vector3d> dq_pos;
        std::deque<Eigen::Matrix3d> dq_rmat;
        std::deque<double> dq_aa;
        for (const auto& t : buffer) {
            dq_pos.push_back(get_P_3x1(t));
            dq_rmat.push_back(get_R_3x3(t));
            dq_aa.push_back(get_REDUN_1x1(t));
        }

        #if NO_OF_JOINT == 7
            return Make_C_from_PandR(get_Average_Pvec(dq_pos), get_Average_Rmat(dq_rmat), get_Average_Number(dq_aa));
        #else
            return Make_C_from_PandR(get_Average_Pvec(dq_pos), get_Average_Rmat(dq_rmat));
        #endif
    }

    double Calc_LPF_Alpha(double Sampling_Hz, double CutOff_Hz)
    {
        double tau = 1.0 / (2.0 * M_PI * CutOff_Hz);   // 시정수 τ
        return std::exp(-1.0 / (Sampling_Hz * tau));     // α 계산
    }

    double Calc_CutOff_Hz(double Sampleing_Hz, double Alpha_v){
        return -Sampleing_Hz / (2.0 * M_PI * std::log(Alpha_v));
    }

    double ShortestDistance_Box_Box(BOX_STRUCT box_A, BOX_STRUCT box_B){
        // 상대 위치 및 회전
        Eigen::Matrix3d R = box_A.box_rot.transpose() * box_B.box_rot;
        Eigen::Vector3d t = box_A.box_rot.transpose() * (box_B.box_center - box_A.box_center);

        // 수치 안정화 처리
        Eigen::Matrix3d absR = R.cwiseAbs();
        const double EPS = 1e-8;
        absR = absR.array() + EPS;

        // 박스 A, B의 절대 좌표축
        Eigen::Vector3d axesA[3] = { box_A.box_rot.col(0), box_A.box_rot.col(1), box_A.box_rot.col(2) };
        Eigen::Vector3d axesB[3] = { box_B.box_rot.col(0), box_B.box_rot.col(1), box_B.box_rot.col(2) };

        // 15개의 분리축 검사
        double minOverlap = std::numeric_limits<double>::infinity();
        Eigen::Vector3d sepAxis;
        bool separated = false;

        // A의 3축
        for (int i = 0; i < 3; ++i) {
            double ra = box_A.box_half[i];
            double rb = box_B.box_half[0] * absR(i,0) + box_B.box_half[1] * absR(i,1) + box_B.box_half[2] * absR(i,2);
            double dist = std::abs(t[i]) - (ra + rb);
            if (dist > 0.0) return dist; // 분리됨
            minOverlap = std::min(minOverlap, -dist);
        }

        // B의 3축
        for (int i = 0; i < 3; ++i) {
            double ra = box_A.box_half[0] * absR(0,i) + box_A.box_half[1] * absR(1,i) + box_A.box_half[2] * absR(2,i);
            double rb = box_B.box_half[i];
            double dist = std::abs(t.dot(R.col(i))) - (ra + rb);
            if (dist > 0.0) return dist; // 분리됨
            minOverlap = std::min(minOverlap, -dist);
        }

        // 9개의 교차 축 (A_i × B_j)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                Eigen::Vector3d axis = axesA[i].cross(axesB[j]);
                double len = axis.norm();
                if (len < EPS) continue; // 평행 축은 skip
                axis /= len;

                // 투영 길이 계산
                double ra = box_A.box_half[0] * std::abs(axis.dot(axesA[0])) +
                            box_A.box_half[1] * std::abs(axis.dot(axesA[1])) +
                            box_A.box_half[2] * std::abs(axis.dot(axesA[2]));
                double rb = box_B.box_half[0] * std::abs(axis.dot(axesB[0])) +
                            box_B.box_half[1] * std::abs(axis.dot(axesB[1])) +
                            box_B.box_half[2] * std::abs(axis.dot(axesB[2]));
                double dist = std::abs((box_B.box_center - box_A.box_center).dot(axis)) - (ra + rb);
                if (dist > 0.0) return dist; // 분리됨
                minOverlap = std::min(minOverlap, -dist);
            }
        }

        // 교차 상태 → 최소 간격 0
        return -0.01;
    }

    double ShortestDistance_Cap_Box(const CAPSULE_STRUCT cap, const BOX_STRUCT box){
        // 1. 선분을 박스 local 좌표로 변환
        Eigen::Vector3d lp0 = box.box_rot.transpose() * (cap.p0 - box.box_center);
        Eigen::Vector3d lp1 = box.box_rot.transpose() * (cap.p1 - box.box_center);
        Eigen::Vector3d d = lp1 - lp0; // direction

        // 2️⃣ 선분 중심/방향
        Eigen::Vector3d segCenter = 0.5 * (lp0 + lp1);
        double segExtent    = 0.5 * d.norm();

        // 선분이 "점"에 가까운 경우(길이 0)는 예외 처리
        if (segExtent < 1e-9)
        {
            Eigen::Vector3d q = segCenter.cwiseMax(-box.box_half).cwiseMin(box.box_half);
            return (segCenter - q).norm();
        }

        Eigen::Vector3d segDir = d.normalized();

        // 3️⃣ 박스 중심과 선분 중심 간의 벡터
        Eigen::Vector3d diff = segCenter;

        // 4️⃣ 절대값 기반 거리 계산 (Ericson 공식)
        double fAWdU[3], fADdU[3];
        for (int i = 0; i < 3; i++) {
            fAWdU[i] = std::abs(segDir[i]);
            fADdU[i] = std::abs(diff[i]);
        }

        // 5️⃣ 최소 거리 제곱 계산
        double fDistanceSq = 0.0;
        for (int i = 0; i < 3; i++) {
            double fDelta = fADdU[i] - (box.box_half[i] + segExtent * fAWdU[i]);
            if (fDelta > 0.0)
                fDistanceSq += fDelta * fDelta;
        }

        return std::sqrt(fDistanceSq);
    }

    double ShortestDistance_Cap_Cap(const CAPSULE_STRUCT s1, const CAPSULE_STRUCT s2){
    //    MatrixNd u(3, 1);//save
    //    MatrixNd v(3, 1);
    //    MatrixNd w(3, 1);

        double u[3];
        double v[3];
        double w[3];

        double a, b, c, d, e;
        double D;
        double sc, sN, sD;      // sc = sN/sD
        double tc, tN, tD;      // tc = tN/tD
        double dP[3];

    //    u = s1.p1 - s1.p0;
    //    v = s2.p1 - s2.p0;
    //    w = s1.p0 - s2.p0;
        for(int k=0; k<3; k++){
            u[k] = s1.p1(k,0) - s1.p0(k,0);
            v[k] = s2.p1(k,0) - s2.p0(k,0);
            w[k] = s1.p0(k,0) - s2.p0(k,0);
        }

    //    a = u(0,0)*u(0,0) + u(1,0)*u(1,0) + u(2,0)*u(2,0);
    //    b = u(0,0)*v(0,0) + u(1,0)*v(1,0) + u(2,0)*v(2,0);
    //    c = v(0,0)*v(0,0) + v(1,0)*v(1,0) + v(2,0)*v(2,0);
    //    d = u(0,0)*w(0,0) + u(1,0)*w(1,0) + u(2,0)*w(2,0);
    //    e = v(0,0)*w(0,0) + v(1,0)*w(1,0) + v(2,0)*w(2,0);
        a = u[0]*u[0] + u[1]*u[1] + u[2]*u[2];
        b = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
        c = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
        d = u[0]*w[0] + u[1]*w[1] + u[2]*w[2];
        e = v[0]*w[0] + v[1]*w[1] + v[2]*w[2];


        D = a*c - b*b;
        sD = D;
        tD = D;

        // a*sc - b*tc = -d
        // b*sc - c*tc = -e

        // sc = (be-cd)/(ac-b^2)
        // tc = (ae-bd)/(ac-b^2)

        if(D < 0.000001){
            // almost parallel
            sN = 0.0;
            sD = 1.0;
            tN = e;
            tD = c;
        }else{
            sN = (b*e - c*d);
            tN = (a*e - b*d);
            if(sN < 0.0){
                sN = 0.0;
                tN = e;
                tD = c;
            }else if(sN > sD){
                sN = sD;
                tN = e+b;
                tD = c;
            }
        }

        if(tN < 0.0){
            tN = 0.0;
            if(-d < 0.0){
                sN = 0.0;
            }else if(-d > a){
                sN = sD;
            }else{
                sN = -d;
                sD = a;
            }
        }else if(tN > tD){
            tN = tD;
            if((-d+b) < 0.0){
                sN = 0.0;
            }else if((-d+b) > a){
                sN = sD;
            }else{
                sN = (-d+b);
                sD = a;
            }
        }

        sc = (fabs(sN) < 0.000001 ? 0.0 : sN/sD);
        tc = (fabs(tN) < 0.000001 ? 0.0 : tN/tD);

    //    dP[0] = w(0,0) + sc*u(0,0) - tc*v(0,0);
    //    dP[1] = w(1,0) + sc*u(1,0) - tc*v(1,0);
    //    dP[2] = w(2,0) + sc*u(2,0) - tc*v(2,0);
        dP[0] = w[0] + sc*u[0] - tc*v[0];
        dP[1] = w[1] + sc*u[1] - tc*v[1];
        dP[2] = w[2] + sc*u[2] - tc*v[2];

        double ret = sqrt(dP[0]*dP[0] + dP[1]*dP[1] + dP[2]*dP[2]);
        if(std::isinf(ret) || std::isnan(ret)){
            return -1;
        }
        return ret;
    }

    void TestSaveStart(){
        saving_datas.clear();
        std::cout<<"Saving Data Clear"<<std::endl;
    }

    void TestSavePush(TEST_SAVE_STRUCT line){
        saving_datas.push_back(line);
    }

    void TestSaveDone(std::string f_name){
        std::cout<<"Saving Data Size: "<<(int)saving_datas.size()<<std::endl;
        FILE* fp_save;
        fp_save = fopen(f_name.data(), "w");
        for(int i = 0; i < saving_datas.size(); ++i){
            for(int it = 0; it< 30; ++it){
                fprintf(fp_save, "%f\t", saving_datas.at(i).data[it]);
            }
            fprintf(fp_save, "\n");
        }
        fflush(fp_save);
        fclose(fp_save);

        std::cout<<"Saving Data Done"<<std::endl;
    }

    sloper_f::sloper_f(float init_value, float t_gap_up, float t_gap_dw) {
        output_value = init_value;
        gap_up = t_gap_up;
        gap_dw = t_gap_dw;
    }
    float sloper_f::Update(float new_input){
        if(new_input >= output_value){
            float new_output = output_value + gap_up;
            if(new_output >= new_input){
                new_output = new_input;
            }
            output_value = new_output;
        }else{
            float new_output = output_value - gap_dw;
            if(new_output <= new_input){
                new_output = new_input;
            }
            output_value = new_output;
        }
        return output_value;
    }
}
