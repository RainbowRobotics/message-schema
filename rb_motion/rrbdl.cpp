#include "rrbdl.h"
#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

RBDLrobot::RBDLrobot() {
    // model_ = Model();
    model_ = new RigidBodyDynamics::Model();

    jacob_matrix = MatrixNd::Zero(6, NO_OF_JOINT);
    jacob_determinant = 0.;

    ik_speed_scaler_jacob_input = 1.;
    ik_speed_scaler_jacob = 1.;
    ik_speed_scaler_local = 1.;    

    ik_Aineq_ub = MatrixNd::Identity(NO_OF_JOINT, NO_OF_JOINT);
    ik_Aineq_lb = -1.0*MatrixNd::Identity(NO_OF_JOINT, NO_OF_JOINT);
}

RBDLrobot::~RBDLrobot() {
    if(model_ != NULL){
        delete model_;
    }
}

int RBDLrobot::Init_Robot(ROBOT_CONFIG rConfig, TCP_CONFIG tConfig, Vector3d tGravity){
    // model_ = Model();

    model_->gravity = tGravity * 9810.;

    redundancy_mode = rConfig.redundancy_type;

    for(int k = 0; k < NO_OF_JOINT; ++k){
        
        r_inertias[k] = rConfig.link_inerti[k];
        if(k == (NO_OF_JOINT - 1)){
            double defalut_mass = rConfig.link_mass_m[k];
            Eigen::Vector3d default_com = rConfig.link_mass_C[k];

            double user_mass = tConfig.com_mass;
            Eigen::Vector3d user_com = tConfig.com_offset + rConfig.link_ee_offset;

            double total_mass = defalut_mass + user_mass;
            Eigen::Vector3d total_com = (defalut_mass * default_com + user_mass * user_com) / total_mass;

            r_masss[k] = total_mass;
            r_coms[k] = total_com;
        }else{
            r_masss[k] = rConfig.link_mass_m[k];
            r_coms[k] = rConfig.link_mass_C[k];
        }
        r_bodys[k] = Body(r_masss[k], r_coms[k], r_inertias[k]);

        if(rConfig.modules_axis[k] == 1){
            r_joints[k] = Joint(JointTypeRevoluteY);
        }else if(rConfig.modules_axis[k] == 2){
            r_joints[k] = Joint(JointTypeRevoluteZ);
        }else{
            r_joints[k] = Joint(JointTypeRevoluteX);
        }

        default_joint_lim_low(k) = rConfig.modules_range_low[k];
        default_joint_lim_up(k) = rConfig.modules_range_up[k];
    }

    for(int k = 0; k < NO_OF_JOINT; ++k){
        int prev_body = 0;
        if(k != 0){
            prev_body = r_nums[k - 1];
        }

        Matrix3d temp_Rotation = rConfig.link_rotation[k];
        SpatialTransform tempTransform(temp_Rotation, rConfig.link_offset[k]);
        std::string tempName = "myBody_" + std::to_string(k);
        r_nums[k] = model_->AddBody(prev_body, tempTransform, r_joints[k], r_bodys[k], tempName);
    }

    m_ee = 0;          c_ee = Vector3d(0,0,0);
    I_ee = Matrix3d::Zero(3, 3);
    b_ee = Body(m_ee, c_ee, I_ee);
    j_ee = Joint(JointTypeFixed);

    Matrix3d eeR = tConfig.tcp_rotation;
    SpatialTransform st_dummy_ee(
                eeR, (tConfig.tcp_offset + rConfig.link_ee_offset)
                );
    ee_body = model_->AddBody(r_nums[NO_OF_JOINT - 1], st_dummy_ee, j_ee, b_ee, "EEFT");

    if(model_->q_size != NO_OF_JOINT){
        std::cout<<("NO_OF_JOINT and Model size is different ...!!!");
    }
    return ((int)model_->q_size);
}

int RBDLrobot::Update_Robot(ROBOT_CONFIG rConfig, TCP_CONFIG tConfig, Vector3d tGravity){

    RigidBodyDynamics::Model *temp_model_ = new RigidBodyDynamics::Model();

    temp_model_->gravity = tGravity * 9810.;

    redundancy_mode = rConfig.redundancy_type;

    for(int k = 0; k < NO_OF_JOINT; ++k){
        
        r_inertias[k] = rConfig.link_inerti[k];
        if(k == (NO_OF_JOINT - 1)){
            double defalut_mass = rConfig.link_mass_m[k];
            Eigen::Vector3d default_com = rConfig.link_mass_C[k];

            double user_mass = tConfig.com_mass;
            Eigen::Vector3d user_com = tConfig.com_offset + rConfig.link_ee_offset;

            double total_mass = defalut_mass + user_mass;
            Eigen::Vector3d total_com = (defalut_mass * default_com + user_mass * user_com) / total_mass;

            r_masss[k] = total_mass;
            r_coms[k] = total_com;
        }else{
            r_masss[k] = rConfig.link_mass_m[k];
            r_coms[k] = rConfig.link_mass_C[k];
        }
        r_bodys[k] = Body(r_masss[k], r_coms[k], r_inertias[k]);

        if(rConfig.modules_axis[k] == 1){
            r_joints[k] = Joint(JointTypeRevoluteY);
        }else if(rConfig.modules_axis[k] == 2){
            r_joints[k] = Joint(JointTypeRevoluteZ);
        }else{
            r_joints[k] = Joint(JointTypeRevoluteX);
        }

        default_joint_lim_low(k) = rConfig.modules_range_low[k];
        default_joint_lim_up(k) = rConfig.modules_range_up[k];
    }

    for(int k = 0; k < NO_OF_JOINT; ++k){
        int prev_body = 0;
        if(k != 0){
            prev_body = r_nums[k - 1];
        }

        Matrix3d temp_Rotation = rConfig.link_rotation[k];
        SpatialTransform tempTransform(temp_Rotation, rConfig.link_offset[k]);
        std::string tempName = "myBody_" + std::to_string(k);
        r_nums[k] = temp_model_->AddBody(prev_body, tempTransform, r_joints[k], r_bodys[k], tempName);
    }

    m_ee = 0;          c_ee = Vector3d(0,0,0);
    I_ee = Matrix3d::Zero(3, 3);
    b_ee = Body(m_ee, c_ee, I_ee);
    j_ee = Joint(JointTypeFixed);

    Matrix3d eeR = tConfig.tcp_rotation;
    SpatialTransform st_dummy_ee(
                eeR, (tConfig.tcp_offset + rConfig.link_ee_offset)
                );
    ee_body = temp_model_->AddBody(r_nums[NO_OF_JOINT - 1], st_dummy_ee, j_ee, b_ee, "EEFT");

    if(temp_model_->q_size != NO_OF_JOINT){
        std::cout<<("NO_OF_JOINT and Model size is different ...!!!");
    }


    RigidBodyDynamics::Model *temp_bowl = model_;
    model_ = temp_model_;
    if(temp_bowl != NULL){
        delete temp_bowl;
    }

    return ((int)model_->q_size);
}

VectorCd RBDLrobot::Calc_ForwardKinematics(const Eigen::VectorXd& q) {
    Eigen::VectorXd q_rad = q * MATH_D2R;
    UpdateKinematicsCustom(*model_, &q_rad, nullptr, nullptr);
    Vector3d ee_pos = CalcBodyToBaseCoordinates(*model_, q_rad, ee_body, Vector3d::Zero(), false);
    Matrix3d ee_rotT = CalcBodyWorldOrientation(*model_, q_rad, ee_body, false);
    Vector3d ee_euler = rb_math::R_to_RPY(ee_rotT.transpose());

    VectorCd ret;
    ret.block(0, 0, 3, 1) = ee_pos;
    ret.block(3, 0, 3, 1) = ee_euler;
    #if NO_OF_JOINT == 7//7DOF
        ret(6) = Calc_Redundancy(q);
    #endif
    return ret;
}

VectorJd RBDLrobot::Calc_InverseDynamics(const Eigen::VectorXd& q,
                                           const Eigen::VectorXd& dq,
                                           const Eigen::VectorXd& ddq) {
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(NO_OF_JOINT);
    RigidBodyDynamics::InverseDynamics(*model_, q * MATH_D2R, dq * MATH_D2R, ddq * MATH_D2R, tau);
    tau *= 0.000001;//mN*mm -> Nm
    return tau;
}

Eigen::MatrixXd RBDLrobot::Calc_Jacobian(const Eigen::VectorXd& q) {
    Eigen::VectorXd q_rad = q * MATH_D2R;
    UpdateKinematicsCustom(*model_, &q_rad, nullptr, nullptr);
    CalcPointJacobian6D(*model_, q_rad, ee_body, Vector3d::Zero(), jacob_matrix, false);
    return jacob_matrix;
}

double RBDLrobot::Calc_Jacobian_Det(const Eigen::VectorXd& q) {
    MatrixNd temp_NN = MatrixNd::Zero(NO_OF_CARTE, NO_OF_JOINT);
    temp_NN.block(0, 0, 6, NO_OF_JOINT) = Calc_Jacobian(q);
    #if NO_OF_JOINT == 7//7DOF
        // for(int i = 0; i < 6; ++i){
        //     temp_NN(i, redundancy_mode) = 0;
        // }
        temp_NN(6, redundancy_mode) = 1;
    #endif

    jacob_determinant = fabs(temp_NN.determinant()) / 100000000.;
    return jacob_determinant;
}

double RBDLrobot::Calc_Redundancy(const Eigen::VectorXd& q_ang){
    if(redundancy_mode >= 0 && redundancy_mode < NO_OF_JOINT){
        return q_ang(redundancy_mode);
    }else{
        return 0;
    }
}

Eigen::MatrixXd RBDLrobot::Calc_MassMatrix(const Eigen::VectorXd& q) {
    Eigen::VectorXd q_rad = q * MATH_D2R;
    MatrixNd H = MatrixNd::Zero(NO_OF_JOINT, NO_OF_JOINT);
    CompositeRigidBodyAlgorithm(*model_, q_rad, H, false);
    return H;
}

double RBDLrobot::Calc_MaxVel(double t_jacob_det, double original_MaxVel_deg){
    // return deg/s

    if(0){
        return original_MaxVel_deg;
    }

    double jx_1 = 0.05;
    double jx_2 = 0.2;
    double jy_1 = original_MaxVel_deg * 0.61;//modi211215
    double jy_2 = original_MaxVel_deg;

    double jw = 0.5/(jx_2 - jx_1)*2.* MATH_PI;
    double jA = (jy_2 - jy_1)/2.;
    double jO = (jy_2 + jy_1)/2.;

    double tx = t_jacob_det;
    double ty = 70.;
    if(tx < jx_1)
        ty = jy_1;
    else if(tx > jx_2)
        ty = jy_2;
    else
        ty = -jA*cos(jw*(tx)-jw*jx_1)+jO;

    return ty;
}

IkResult RBDLrobot::Calc_InverseKinematics_Loop(IkMode mode, double dt_sec, VectorCd target, VectorCd current
                                    , VectorJd q_begin, VectorJd q_min, VectorJd q_max, VectorJd dq_max, VectorJd ddq_max, VectorJd prev_dq_deg
                                    , double err_bound_pos, double err_bound_ori, int is_firstLoop){
    
    if(redundancy_mode >= 0 && redundancy_mode < NO_OF_JOINT){
        q_begin(redundancy_mode) = rb_math::get_REDUN_1x1(target);
        Calc_Jacobian_Det(q_begin);
        current = Calc_ForwardKinematics(q_begin);
    }

    double temp_jacob_det = jacob_determinant;
    if(is_firstLoop){
        if(temp_jacob_det < 0.4)
            ik_speed_scaler_jacob_input = 0.4;
        else
            ik_speed_scaler_jacob_input = 1.;

        ik_speed_scaler_local = 1.;
        ik_speed_scaler_jacob = ik_speed_scaler_jacob_input;
    }

    int WRIST_SINGUL_IDX = 4;
    #if NO_OF_JOINT == 7
        WRIST_SINGUL_IDX = 5;
    #endif

    double wrist_check_1 = fabs(q_begin(WRIST_SINGUL_IDX)  - 0.);
    double wrist_check_2 = fabs(q_begin(WRIST_SINGUL_IDX) -180.);
    double wrist_check_3 = fabs(q_begin(WRIST_SINGUL_IDX) +180.);
    double wrist_check_4 = fabs(q_begin(WRIST_SINGUL_IDX) -360.);
    double wrist_check_5 = fabs(q_begin(WRIST_SINGUL_IDX) +360.);

    if((temp_jacob_det < jacob_determinant_old) && (temp_jacob_det < 0.4) && (wrist_check_1 < 10. || wrist_check_2 < 10. || wrist_check_3 < 10. || wrist_check_4 < 10. || wrist_check_5 < 10.))
        ik_speed_scaler_jacob_input -= 0.01;
    else if((temp_jacob_det > jacob_determinant_old) && (temp_jacob_det > 0.05) && (ik_speed_scaler_local > 0.5))
        ik_speed_scaler_jacob_input += 0.004;

    ik_speed_scaler_jacob_input = rb_math::saturation_L_and_U(ik_speed_scaler_jacob_input, 0.4, 1.0);
    ik_speed_scaler_jacob = 0.97 * ik_speed_scaler_jacob + 0.03 * ik_speed_scaler_jacob_input;
    jacob_determinant_old = temp_jacob_det;

    IkResult ret;
    ret.error_size[0] = ret.error_size[1] = 0.;
    ret.result = IkRet::IK_OK;
    ret.q_out_deg = q_begin;
    ret.time_scaler = 1.;

    
    
    
    Vector3d tar_Pos  = rb_math::get_P_3x1(target);
    Matrix3d tar_Rmat = rb_math::get_R_3x3(target);
    
    Vector3d cur_Pos  = rb_math::get_P_3x1(current);
    Matrix3d cur_Rmat = rb_math::get_R_3x3(current);

    Vector3d err_Pos  = tar_Pos - cur_Pos;
    Matrix3d err_Rmat = tar_Rmat * cur_Rmat.transpose();
    Vector3d err_RCV  = rb_math::R_to_RCV(err_Rmat);

    q_begin *= MATH_D2R;
    q_min *= MATH_D2R;
    q_max *= MATH_D2R;

    for(int i = 0; i < NO_OF_JOINT; ++i){
        double gap_max = q_max(i, 0) - q_begin(i);
        double gap_min = q_min(i, 0) - q_begin(i);
        if(gap_max <= 0){
            gap_max = 0.;
        }
        if(gap_min >= 0){
            gap_min = 0.;
        }
        ik_Bineq_ub(i, 0) = gap_max / dt_sec;
        ik_Bineq_lb(i, 0) = -1.0 * gap_min / dt_sec;
    }

    ik_Aineq.block(0, 0, NO_OF_JOINT, NO_OF_JOINT) = ik_Aineq_ub;
    ik_Aineq.block(NO_OF_JOINT, 0, NO_OF_JOINT, NO_OF_JOINT) = ik_Aineq_lb;

    ik_Bineq.block(0, 0, NO_OF_JOINT, 1) = ik_Bineq_ub;
    ik_Bineq.block(NO_OF_JOINT, 0, NO_OF_JOINT, 1) = ik_Bineq_lb;

    double gaingain = 800.;
    double gain_ori = 300.;
    double gain_pos = 10.;

    ik_Abehav.block(0, 0, 6, NO_OF_JOINT) = jacob_matrix;
    ik_Abehav.block(0, 0, 3, NO_OF_JOINT) *= gain_ori;     // ori gainik_
    ik_Abehav.block(3, 0, 3, NO_OF_JOINT) *= gain_pos;     // pos gainik_

    ik_Bbehav.block(0, 0, 3, 1) = err_RCV * gain_ori *gaingain;
    ik_Bbehav.block(3, 0, 3, 1) = err_Pos * gain_pos *gaingain;

    if(redundancy_mode >= 0 && redundancy_mode < NO_OF_JOINT){
        for(int c = 0; c < 6; ++c){
            ik_Abehav(c, redundancy_mode) = 0;
        }
        ik_Aeq = MatrixNd::Zero(1, NO_OF_JOINT);
        ik_Aeq(0, redundancy_mode) = 1;
        ik_Beq(0, 0) = 0;

        ik_quad_prog.setNums(NO_OF_JOINT, 1, NO_OF_JOINT * 2);//q#, eq#, ineq#
        ik_quad_prog.make_EQ(ik_Aeq, ik_Beq);
    }else{
        ik_quad_prog.setNums(NO_OF_JOINT, 0, NO_OF_JOINT * 2);//q#, eq#, ineq#
    }
    ik_quad_prog.make_IEQ(ik_Aineq, ik_Bineq);
    ik_quad_prog.make_HF(ik_Abehav,ik_Bbehav);

    

    VectorNd    tar_dq_cobot = VectorNd::Zero(NO_OF_JOINT);
    tar_dq_cobot = ik_quad_prog.solve_QP();

    if(mode == IkMode::IK_STRICT){
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(std::isnan(tar_dq_cobot(i)) || std::isinf(tar_dq_cobot(i))){
                ret.time_scaler = 1.;
                ret.q_out_deg = q_begin * MATH_R2D;
                ret.result = IkRet::IK_FAIL_DQ_NAN;
                return ret;
            }
        }

        ret.time_scaler = 1.;
        ret.q_out_deg = (q_begin + tar_dq_cobot * dt_sec) * MATH_R2D;
        VectorCd test_fk = Calc_ForwardKinematics(ret.q_out_deg);

        ret.error_size[0] = (rb_math::get_P_3x1(test_fk) - tar_Pos).norm();
        ret.error_size[1] = rb_math::R_to_RCV(rb_math::get_R_3x3(test_fk).transpose() * tar_Rmat).norm() * MATH_R2D;
        ret.result = IkRet::IK_OK;
        return ret;
    }

    double t_new_vel[NO_OF_JOINT];
    for(int i = 0; i < NO_OF_JOINT; ++i){
        t_new_vel[i] = tar_dq_cobot(i);
        double macc = ddq_max(i) * MATH_D2R;//PARA_JOINT_MAX_ACC_SW[k] * D2R * _hik_velocity_alpha_global * _hik_velocity_alpha_user;
        double mvel = Calc_MaxVel(temp_jacob_det, dq_max(i)) * MATH_D2R;//lets_calc_mvel(k);

        double t_vel_plus = prev_dq_deg(i) * MATH_D2R + macc * dt_sec;
        double t_vel_minus = prev_dq_deg(i) * MATH_D2R - macc * dt_sec;

        if(t_vel_plus > mvel)
            t_vel_plus = mvel;
        if(t_vel_plus < -mvel)
            t_vel_plus = -mvel;
        if(t_vel_minus > mvel)
            t_vel_minus = mvel;
        if(t_vel_minus < -mvel)
            t_vel_minus = -mvel;

        if(t_new_vel[i] > t_vel_plus){
            t_new_vel[i] = t_vel_plus;
        }else if(t_new_vel[i] < t_vel_minus){
            t_new_vel[i] = t_vel_minus;
        }
    }

    double my_max = 1.;
    for(int i = 0; i < NO_OF_JOINT; ++i){
        double mvel = Calc_MaxVel(temp_jacob_det, dq_max(i)) * MATH_D2R;
        double rr = 1.;
        if(fabs(tar_dq_cobot(i)) <= mvel){
            rr = 1.;
        }else{
            rr = tar_dq_cobot(i,0) / t_new_vel[i];
        }

        if(rr > my_max)
            my_max = rr;
    }

    if(std::isnan(my_max) || std::isinf(my_max)){
        ret.time_scaler = 1.;
        ret.q_out_deg = q_begin * MATH_R2D;
        ret.result = IkRet::IK_FAIL_NAN;
        return ret;
    }else if(my_max < 1e-8){
        ret.time_scaler = 1.;
        ret.q_out_deg = q_begin * MATH_R2D;
        ret.result = IkRet::IK_FAIL_TOSMALL;
        return ret;
    }else if((1./my_max) < 1e-8){
        ret.time_scaler = 1.;
        ret.q_out_deg = q_begin * MATH_R2D;
        ret.result = IkRet::IK_FAIL_ZERO;
        return ret;
    }

    ik_speed_scaler_local = ik_speed_scaler_local * 0.8 + 0.2 * 1./fabs(my_max);
    tar_dq_cobot /= my_max;

    for(int i = 0; i < NO_OF_JOINT; ++i){
        if(std::isnan(tar_dq_cobot(i)) || std::isinf(tar_dq_cobot(i))){
            ret.time_scaler = 1.;
            ret.q_out_deg = q_begin * MATH_R2D;
            ret.result = IkRet::IK_FAIL_DQ_NAN;
            return ret;
        }
    }


    ret.q_out_deg = (q_begin + tar_dq_cobot * dt_sec) * MATH_R2D;

    VectorCd test_fk = Calc_ForwardKinematics(ret.q_out_deg);

    ret.error_size[0] = (rb_math::get_P_3x1(test_fk) - tar_Pos).norm();
    ret.error_size[1] = rb_math::R_to_RCV(rb_math::get_R_3x3(test_fk).transpose() * tar_Rmat).norm() * MATH_R2D;

    if(ret.error_size[0] >= err_bound_pos){
        ret.time_scaler = 1.;
        ret.q_out_deg = q_begin * MATH_R2D;
        ret.result = IkRet::IK_FAIL_POS_ERR;

        std::cout<<"ret.error_size[0] "<<ret.error_size[0]<<std::endl;
        return ret;
    }else if(ret.error_size[1] >= err_bound_ori){
        ret.time_scaler = 1.;
        ret.q_out_deg = q_begin * MATH_R2D;
        ret.result = IkRet::IK_FAIL_ROT_ERR;
        return ret;
    }

    ret.time_scaler = ik_speed_scaler_local * ik_speed_scaler_jacob;
    ret.result = IkRet::IK_OK;

    return ret;
}

IkResult RBDLrobot::Calc_InverseKinematics_Static(VectorCd target, int iter_traj, int iter_loop, VectorCd start_X, VectorJd start_J, VectorJd q_min, VectorJd q_max){

    IkResult ret;
    ret.error_size[0] = ret.error_size[1] = 0.;
    ret.q_out_deg = start_J;
    ret.time_scaler = 1.;
    ret.result = IkRet::IK_UNSOLV;

    if(iter_traj < 1) iter_traj = 100;
    if(iter_loop < 1) iter_loop = 100;

    double   ik_loop_err[2] = {0, 0};
    VectorJd ik_loop_J_ref = start_J;
    for(int i = 0; i < iter_traj; ++i){
        double traj_rate = ((double)i+1) / ((double)iter_traj);
        VectorCd traj_target = rb_math::Blend_Carte(start_X, target, traj_rate);
        
        for(int k = 0; k < iter_loop; ++k){
            VectorCd current_loop_pos = Calc_ForwardKinematics(ik_loop_J_ref);
            Calc_Jacobian_Det(ik_loop_J_ref);
            IkResult loop_ret = Calc_InverseKinematics_Loop(IkMode::IK_STRICT, RT_PERIOD_SEC, traj_target, current_loop_pos, ik_loop_J_ref
                                , q_min, q_max, VectorJd::Zero(NO_OF_JOINT, 1), VectorJd::Zero(NO_OF_JOINT, 1), VectorJd::Zero(NO_OF_JOINT, 1), 1., 1., true);

            if(loop_ret.result == IkRet::IK_OK){
                ik_loop_J_ref = loop_ret.q_out_deg;
                ik_loop_err[0] = loop_ret.error_size[0];
                ik_loop_err[1] = loop_ret.error_size[1];
                if(loop_ret.error_size[0] < 0.08 && loop_ret.error_size[1] < 0.08){
                    break;
                }
            }else{
                std::cout<<"iter break"<<std::endl;
                return ret;
            }
        }
    }

    ret.q_out_deg = ik_loop_J_ref;
    ret.error_size[0] = ik_loop_err[0];
    ret.error_size[1] = ik_loop_err[1];
    if(ret.error_size[0] < 0.1 && ret.error_size[1] < 0.1){
        ret.result = IkRet::IK_OK;
    }else{
        std::cout<<":ret.error_size[0]: "<<ret.error_size[0]<<" / "<<ret.error_size[1]<<std::endl;
    }

    return ret;
}

IkResult RBDLrobot::Calc_InverseKinematics_Static(VectorCd target, int iter_traj, int iter_loop, VectorCd start_X, VectorJd start_J){
    return Calc_InverseKinematics_Static(target, iter_traj, iter_loop, start_X, start_J, default_joint_lim_low, default_joint_lim_up);
}

// IK_RESULT_STRUCT RCR_CALC_IK(double px, double py, double pz, double ox, double oy, double oz, unsigned char q_flag, unsigned int iter_num, ANGLE_INFO_STRUCT ik_ref_angs){
//     IK_RESULT_STRUCT ret;

//     Vector3d pos_start = X_cobot;
//     Vector3d rcv_start = RPY_to_RCV(YPR_cobot_arr[2], YPR_cobot_arr[1], YPR_cobot_arr[0]);
//     Vector3d pos_goal = Vector3d(px, py, pz);
//     Vector3d rcv_goal = RPY_to_RCV(ox, oy, oz);

//     VectorNd jang = VectorNd::Zero(MAX_MC);
//     if(q_flag == 1){//default
//         jang = q_inner_loop*R2D;
//     }else if(q_flag == 2){
//         POSTURE_INFO_STRUCT temp_fk = RCR_CALC_FK(ik_ref_angs, 0);
//         pos_start = Vector3d(temp_fk.posture[0], temp_fk.posture[1], temp_fk.posture[2]);
//         rcv_start = RPY_to_RCV(temp_fk.posture[3], temp_fk.posture[4], temp_fk.posture[5]);

//         for(int ski = 0; ski < MAX_MC; ski++){
//             jang(ski) = ik_ref_angs.ang[ski];
//         }
//     }else if(q_flag == 3){
//         POSTURE_INFO_STRUCT temp_fk = RCR_CALC_FK(ik_ref_angs, 0);
//         pos_start = Vector3d(temp_fk.posture[0], temp_fk.posture[1], temp_fk.posture[2]);
//         rcv_start = RPY_to_RCV(temp_fk.posture[3], temp_fk.posture[4], temp_fk.posture[5]);

//         jang = q_inner_loop*R2D;
//     }else{
//         for(int ski = 0; ski < MAX_MC; ski++){
//             jang(ski) = ik_ref_angs.ang[ski];
//         }
//     }


//     ret.is_success = true;
//     for(unsigned int k = 0; k < iter_num; k++){
//         double p_rate = ((double)k+1)/((double)iter_num);
//         Vector3d temp_pos = (1.-p_rate)*pos_start + p_rate*pos_goal;
//         Vector3d temp_rcv = RCV_BLEND(rcv_start, rcv_goal, p_rate);
//         Vector3d temp_rpy = R_to_RPY(RCVtoR(temp_rcv));

//         ANGLE_INFO_STRUCT ik_ref_ang;
//         for(int iki = 0; iki < MAX_MC; iki++){
//             ik_ref_ang.ang[iki] = jang(iki);
//         }
//         jang = calc_IK(temp_pos(0), temp_pos(1), temp_pos(2), temp_rpy(0), temp_rpy(1), temp_rpy(2), ik_ref_ang, 0);

//         if(jang(0) > 499 && jang(0)<501){
//             ret.is_success = false;
//             break;
//         }
//     }


//     for(unsigned char k = 0; k < MAX_MC; k++){
//         if(ret.is_success){
//             ret.ang[k] = jang(k);
//         }else{
//             ret.ang[k] = q_inner_loop_LPF(k) * R2D;
//         }
//     }

//     return ret;
// }