#define P_NAME  "MOTION"

#include "../rb_common/common.h"

#include "motion.h"
#include "types/move_j.h"
#include "types/move_l.h"
#include "types/move_jb.h"
#include "types/move_lb.h"
#include "types/move_xb.h"
#include "types/move_cir.h"
#include "types/move_speed_j.h"
#include "types/move_speed_l.h"
#include "rrbdl.h"


namespace rb_motion {
    namespace {
        //--------------------------------------------------------------
        // Sector Move
        //--------------------------------------------------------------
        struct PreMotionRet{
            VectorJd j_output;
            VectorCd c_output;
            int ret;
        };
        enum class MoveTerminate        {DONE, IK_ERR, TRAJ_ERR};

        
        //--------------------------------------------------------------
        // Sector Motion
        //--------------------------------------------------------------
        RBDLrobot       _motion_Robot;
        MotionMode      _motion_mode = MotionMode::MOVE_NONE;
        
        VectorJd        _motion_q_ang;
        VectorJd        _motion_q_vel;
        VectorJd        _motion_q_acc;

        VectorCd        _motion_x_pos;
        VectorCd        _motion_x_pos_old;
        Vector3d        _motion_x_vel_3 = Vector3d(0, 0, 0);
        Vector3d        _motion_x_vel_rcv = Vector3d(0, 0, 0);
        double          _motion_x_jacobdet;

        move_j          _motion_move_j;
        move_l          _motion_move_l;
        move_jb         _motion_move_jb;
        move_lb         _motion_move_lb;
        move_xb         _motion_move_xb;
        move_cir        _motion_move_cir;
        move_speed_j    _motion_move_speed_j;
        move_speed_l    _motion_move_speed_l;

        SHIFT_INPUT     _motion_shift[NO_OF_SHIFT];
        //--------------------------------------------------------------
        // Sector Wrapper
        //--------------------------------------------------------------
        RBDLrobot       _wrapper_Robot;
        WrapperMode     _wrapper_mode = WrapperMode::WRAPPER_NONE;

        VectorJd        _wrapper_q_ang;
        VectorJd        _wrapper_q_vel;
        VectorJd        _wrapper_q_acc;

        VectorCd        _wrapper_x_pos;
        double          _wrapper_x_jacobdet;

        bool            _wrapper_is_First;
        Vector3d        _wrapper_sum;
        Vector3d        _wrapper_vel;

        //--------------------------------------------------------------

        void Terminate_Motion(MoveTerminate type){
            if(type == MoveTerminate::IK_ERR){
                MESSAGE(MSG_LEVEL_ERRR, MSG_MOVE_RUNTIME_TERMINATE_IK_ERR);
                LOG_ERROR("Motion-Ik Problem");
            }else if(type == MoveTerminate::TRAJ_ERR){
                MESSAGE(MSG_LEVEL_ERRR, MSG_MOVE_RUNTIME_TERMINATE_TRAJ_ERR);
                LOG_ERROR("Motion-Traj Problem");
            }else{
                LOG_INFO("Motion-Time Done");
            }

            rb_system::Reset_MoveBreak();

            Set_Motion_Mode(MotionMode::MOVE_NONE);
        }

        PreMotionRet PreMotion_to_J(TARGET_INPUT input, int calc_option){
            // calc_option
            // 0 : General
            // 1 : no poisional reflection

            PreMotionRet ret;
            if(input.target_frame == FRAME_JOINT){
                ;//input is Joint Space
                for(int i = 0; i < NO_OF_JOINT; i++){
                    ret.j_output(i) = input.target_value[i];
                }
                ret.ret = MSG_OK;
            }else{
                ;//input is Cartesian
                VectorCd input_cartesian;
                for(int i = 0; i < NO_OF_CARTE; i++){
                    input_cartesian(i) = input.target_value[i];
                }
                if(input.target_unit == UNIT_INCH_DEG){
                    input_cartesian(0) *= MATH_INCH2MM;
                    input_cartesian(1) *= MATH_INCH2MM;
                    input_cartesian(2) *= MATH_INCH2MM;
                }

                Vector3d input_Pos = rb_math::get_P_3x1(input_cartesian);
                Matrix3d input_Rmat = rb_math::get_R_3x3(input_cartesian);
                double   input_Aa = rb_math::get_REDUN_1x1(input_cartesian);

                Vector3d ref_frame_Pos = Vector3d::Zero(3, 1);
                Matrix3d ref_frame_Rmat = Matrix3d::Identity(3, 3);
                if(input.target_frame == FRAME_LOCAL){
                    ref_frame_Pos = rb_math::get_P_3x1(_wrapper_x_pos);
                    ref_frame_Rmat = rb_math::get_R_3x3(_wrapper_x_pos);
                }else if(input.target_frame == FRAME_USER){
                    ref_frame_Pos = rb_system::Get_CurrentUserFrameParameter().userf_offset;
                    ref_frame_Rmat = rb_system::Get_CurrentUserFrameParameter().userf_rotation;
                }

                if(calc_option == 1){
                    ref_frame_Pos = Vector3d(0, 0, 0);
                }

                Vector3d new_Pos = ref_frame_Pos + ref_frame_Rmat * input_Pos;
                Matrix3d new_Rmat = ref_frame_Rmat * input_Rmat;

                #if NO_OF_JOINT == 7
                    VectorCd new_cartesian = rb_math::Make_C_from_PandR(new_Pos, new_Rmat, input_Aa);
                #else
                    VectorCd new_cartesian = rb_math::Make_C_from_PandR(new_Pos, new_Rmat);
                #endif

                {//for shiftting
                    Vector3d ori_P = rb_math::get_P_3x1(new_cartesian);
                    Matrix3d ori_R = rb_math::get_R_3x3(new_cartesian);
                    double   ori_A = rb_math::get_REDUN_1x1(new_cartesian);

                    Vector3d del_P = Vector3d(0, 0, 0);
                    Matrix3d del_R = Matrix3d::Identity(3, 3);
                    double   del_A = 0.;
                    for (auto& temp_shift : _motion_shift) {
                        if(temp_shift.mode != 0){
                            Matrix3d shift_frame_R = Matrix3d::Identity(3, 3);
                            if(temp_shift.value.target_frame == FRAME_LOCAL){
                                shift_frame_R = rb_math::get_R_3x3(_wrapper_x_pos);
                            }else if(temp_shift.value.target_frame == FRAME_USER){
                                shift_frame_R = rb_system::Get_CurrentUserFrameParameter().userf_rotation;
                            }else if(temp_shift.value.target_frame == FRAME_TARGET){
                                shift_frame_R = ori_R;
                            }

                            del_P = (shift_frame_R * Vector3d(temp_shift.value.target_value[0], temp_shift.value.target_value[1], temp_shift.value.target_value[2])) + del_P;
                            del_R = (shift_frame_R * rb_math::RPY_to_R(temp_shift.value.target_value[3], temp_shift.value.target_value[4], temp_shift.value.target_value[5])) * del_R;
                            #if NO_OF_JOINT == 7
                                del_A = del_A + temp_shift.value.target_value[6];
                            #endif
                        }
                    }

                    #if NO_OF_JOINT == 7
                        new_cartesian = rb_math::Make_C_from_PandR((del_P + ori_P), (del_R * ori_R), (del_A + ori_A));
                    #else
                        new_cartesian = rb_math::Make_C_from_PandR((del_P + ori_P), (del_R * ori_R));
                    #endif
                }

                RBDLrobot temp_robot;
                temp_robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());

                IkResult ik_ = temp_robot.Calc_InverseKinematics_Static(new_cartesian, 100, 100, _motion_x_pos, _motion_q_ang);

                if(ik_.result == IkRet::IK_OK){
                    ret.j_output = ik_.q_out_deg;
                    ret.ret = MSG_OK;
                }else{
                    ret.j_output = _motion_q_ang;
                    ret.ret = MSG_MOVE_PRE_IK_ERR;
                }
            }
            return ret;
        }

        PreMotionRet PreMotion_to_L(TARGET_INPUT input, int calc_option){
            // calc_option
            // 0 : General

            PreMotionRet ret;
            if(input.target_frame == FRAME_JOINT){
                ;//input is Joint Space
                VectorJd input_joints;
                for(int i = 0; i < NO_OF_JOINT; i++){
                    input_joints(i) = input.target_value[i];
                }

                RBDLrobot temp_robot;
                temp_robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
                ret.c_output = temp_robot.Calc_ForwardKinematics(input_joints);
                ret.ret = MSG_OK;
            }else{
                VectorCd input_cartesian;
                for(int i = 0; i < NO_OF_CARTE; i++){
                    input_cartesian(i) = input.target_value[i];
                }
                if(input.target_unit == UNIT_INCH_DEG){
                    input_cartesian(0) *= MATH_INCH2MM;
                    input_cartesian(1) *= MATH_INCH2MM;
                    input_cartesian(2) *= MATH_INCH2MM;
                }

                Vector3d input_Pos = rb_math::get_P_3x1(input_cartesian);
                Matrix3d input_Rmat = rb_math::get_R_3x3(input_cartesian);
                double   input_Aa = rb_math::get_REDUN_1x1(input_cartesian);

                Vector3d ref_frame_Pos = Vector3d::Zero(3, 1);
                Matrix3d ref_frame_Rmat = Matrix3d::Identity(3, 3);
                if(input.target_frame == FRAME_LOCAL){
                    ref_frame_Pos = rb_math::get_P_3x1(_wrapper_x_pos);
                    ref_frame_Rmat = rb_math::get_R_3x3(_wrapper_x_pos);
                }else if(input.target_frame == FRAME_USER){
                    ref_frame_Pos = rb_system::Get_CurrentUserFrameParameter().userf_offset;
                    ref_frame_Rmat = rb_system::Get_CurrentUserFrameParameter().userf_rotation;
                }

                Vector3d new_Pos = ref_frame_Pos + ref_frame_Rmat * input_Pos;
                Matrix3d new_Rmat = ref_frame_Rmat * input_Rmat;

                #if NO_OF_JOINT == 7
                    VectorCd new_cartesian = rb_math::Make_C_from_PandR(new_Pos, new_Rmat, input_Aa);
                #else
                    VectorCd new_cartesian = rb_math::Make_C_from_PandR(new_Pos, new_Rmat);
                #endif

                ret.c_output = new_cartesian;
                ret.ret = MSG_OK;
            }

            {//for shiftting
                Vector3d ori_P = rb_math::get_P_3x1(ret.c_output);
                Matrix3d ori_R = rb_math::get_R_3x3(ret.c_output);
                double   ori_A = rb_math::get_REDUN_1x1(ret.c_output);

                Vector3d del_P = Vector3d(0, 0, 0);
                Matrix3d del_R = Matrix3d::Identity(3, 3);
                double   del_A = 0.;
                for (auto& temp_shift : _motion_shift) {
                    if(temp_shift.mode != 0){
                        Matrix3d shift_frame_R = Matrix3d::Identity(3, 3);
                        if(temp_shift.value.target_frame == FRAME_LOCAL){
                            shift_frame_R = rb_math::get_R_3x3(_wrapper_x_pos);
                        }else if(temp_shift.value.target_frame == FRAME_USER){
                            shift_frame_R = rb_system::Get_CurrentUserFrameParameter().userf_rotation;
                        }else if(temp_shift.value.target_frame == FRAME_TARGET){
                            shift_frame_R = ori_R;
                        }

                        del_P = (shift_frame_R * Vector3d(temp_shift.value.target_value[0], temp_shift.value.target_value[1], temp_shift.value.target_value[2])) + del_P;
                        del_R = (shift_frame_R * rb_math::RPY_to_R(temp_shift.value.target_value[3], temp_shift.value.target_value[4], temp_shift.value.target_value[5])) * del_R;
                        #if NO_OF_JOINT == 7
                            del_A = del_A + temp_shift.value.target_value[6];
                        #endif
                    }
                }

                #if NO_OF_JOINT == 7
                    ret.c_output = rb_math::Make_C_from_PandR((del_P + ori_P), (del_R * ori_R), (del_A + ori_A));
                #else
                    ret.c_output = rb_math::Make_C_from_PandR((del_P + ori_P), (del_R * ori_R));
                #endif
            }
            return ret;
        }

        VectorCd PreMotion_to_L_for_Speed(TARGET_INPUT input){
            VectorCd ret;
            if(input.target_frame == FRAME_JOINT){
                ret = VectorCd::Zero(NO_OF_CARTE, 1);
            }else{
                Matrix3d frame_R = Matrix3d::Identity(3, 3);
                if(input.target_frame == FRAME_LOCAL){
                    frame_R = rb_math::get_R_3x3(_wrapper_x_pos);
                }else if(input.target_frame == FRAME_USER){
                    frame_R = rb_system::Get_CurrentUserFrameParameter().userf_rotation;
                }

                for(int i = 0; i < NO_OF_CARTE; i++){
                    ret(i) = input.target_value[i];
                }
                Vector3d new_v_P = frame_R * Vector3d(input.target_value[0], input.target_value[1], input.target_value[2]);
                Vector3d new_v_O = frame_R * Vector3d(input.target_value[3], input.target_value[4], input.target_value[5]);
                ret.block(0, 0, 3, 1) = new_v_P;
                ret.block(3, 0, 3, 1) = new_v_O;
            }
            return ret;
        }

        void Set_Wrapper_Mode(WrapperMode t_mode){
            if(_wrapper_mode != t_mode){
                LOG_INFO("Wrapper Mode Changed: " + std::to_string((int)_motion_mode) + "->" + std::to_string((int)t_mode));
            }

            if(t_mode == WrapperMode::WRAPPER_NONE){
                Set_Motion_Mode(MotionMode::MOVE_NONE);
                Sync_Wrapper_to_Motion();
            }

            _wrapper_mode = t_mode;
        }

        void Terminate_Wrapper(MoveTerminate type){
            if(type == MoveTerminate::IK_ERR){
                LOG_ERROR("Wrapper-Ik Problem");
            }else if(type == MoveTerminate::TRAJ_ERR){
                LOG_ERROR("Wrapper-Traj Problem");
            }else{
                LOG_INFO("Wrapper-Time Done");
            }

            Set_Wrapper_Mode(WrapperMode::WRAPPER_NONE);
        }

        ;
    }

    //--------------------------------------------------------------------------------------
    // Sector Move
    //--------------------------------------------------------------------------------------
    bool initialize(){
        _motion_Robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        _wrapper_Robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        return true;
    }

    bool Model_Update(){
        _motion_Robot.Update_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        _wrapper_Robot.Update_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        return true;
    }

    bool Get_Is_Idle(){
        return (_motion_mode == MotionMode::MOVE_NONE
                && _wrapper_mode == WrapperMode::WRAPPER_NONE);
    }

    bool Get_Is_Motion_Idle(){
        return (_motion_mode == MotionMode::MOVE_NONE);
    }

    bool Get_Is_Wrapper_Idle(){
        return (_wrapper_mode == WrapperMode::WRAPPER_NONE);
    }

    void Sync_Wrapper_to_Motion(){
        _motion_q_ang = _wrapper_q_ang;
        _motion_x_pos = _wrapper_x_pos;
    }

    std::tuple<int, TARGET_INPUT> Calc_J_Relative(TARGET_INPUT delta_){
        TARGET_INPUT pin_ = rb_math::Make_Input_Zero(FRAME_JOINT);
        for(int i = 0; i < NO_OF_JOINT; ++i){
            pin_.target_value[i] = _wrapper_q_ang(i);
        }
        return Calc_J_Relative(delta_, pin_);
    }
    std::tuple<int, TARGET_INPUT> Calc_J_Relative(TARGET_INPUT delta_, TARGET_INPUT pin_){
        PreMotionRet pre_del = PreMotion_to_J(delta_, 0);
        PreMotionRet pre_pin = PreMotion_to_J(pin_, 0);

        TARGET_INPUT ret_v = rb_math::Make_Input_Zero(FRAME_JOINT);
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret_v.target_value[i] = _wrapper_q_ang(i);
        }

        if(pre_del.ret != MSG_OK){
            return {pre_del.ret, ret_v};
        }
        if(pre_pin.ret != MSG_OK){
            return {pre_pin.ret, ret_v};
        }        

        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret_v.target_value[i] = pre_pin.j_output(i) + pre_del.j_output(i);
        }
        return {MSG_OK, ret_v};
    }
    std::tuple<int, TARGET_INPUT> Calc_L_Relative(TARGET_INPUT delta_){
        TARGET_INPUT pin_ = rb_math::Make_Input_Zero(FRAME_GLOBAL);
        for(int i = 0; i < NO_OF_CARTE; ++i){
            pin_.target_value[i] = _wrapper_x_pos(i);
        }
        return Calc_L_Relative(delta_, pin_);
    }
    std::tuple<int, TARGET_INPUT> Calc_L_Relative(TARGET_INPUT delta_, TARGET_INPUT pin_){
        PreMotionRet pre_pin = PreMotion_to_L(pin_, 0);

        TARGET_INPUT ret_v = rb_math::Make_Input_Zero(FRAME_GLOBAL);
        for(int i = 0; i < NO_OF_CARTE; ++i){
            ret_v.target_value[i] = _wrapper_x_pos(i);
        }

        VectorCd delta_Cd = PreMotion_to_L_for_Speed(delta_);

        if(pre_pin.ret != MSG_OK){
            return {pre_pin.ret, ret_v};
        }  
        Vector3d tP = rb_math::get_P_3x1(delta_Cd) + rb_math::get_P_3x1(pre_pin.c_output);
        Matrix3d tR = rb_math::get_R_3x3(delta_Cd) * rb_math::get_R_3x3(pre_pin.c_output);
        double   tA = rb_math::get_REDUN_1x1(delta_Cd) + rb_math::get_REDUN_1x1(pre_pin.c_output);

        #if NO_OF_JOINT == 7
            VectorCd new_cartesian = rb_math::Make_C_from_PandR(tP, tR, tA);
        #else
            VectorCd new_cartesian = rb_math::Make_C_from_PandR(tP, tR);
        #endif

        for(int i = 0; i < NO_OF_CARTE; ++i){
            ret_v.target_value[i] = new_cartesian(i);
        }
        return {MSG_OK, ret_v};
    }
    //--------------------------------------------------------------------------------------
    // Sector Motion
    //--------------------------------------------------------------------------------------
    std::tuple<VectorJd, VectorCd> Task_Motion_Handler(double System_DT, double External_Alpha, bool is_motion_break, double break_alpha){
        // -------------------------------------------------------------------
        VectorJd temp_q_ang = _motion_q_ang;
        VectorJd temp_q_vel = _motion_q_vel;
        // -------------------------------------------------------------------
        double temp_time_scaler = External_Alpha;

        switch(_motion_mode){
            case MotionMode::MOVE_NONE:
            {
                ;
                break;
            }
            case MotionMode::MOVE_J:
            {
                move_j_control_ret ret = _motion_move_j.Control(-1);

                if(ret.is_thereErr != 0){
                    Terminate_Motion(MoveTerminate::TRAJ_ERR);
                }else{
                    temp_q_ang = ret.J_output;
                    _motion_move_j.Update_Timer(System_DT * temp_time_scaler);
                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }
                break;
            }
            case MotionMode::MOVE_L:
            {
                
                move_l_control_ret ret = _motion_move_l.Control(-1);

                VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
                VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
                VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
                VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

                IkResult ik_ret = _motion_Robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, System_DT, ret.L_output, _motion_x_pos
                , temp_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max, _motion_q_vel, 5.0, 1.0, ret.is_First);

                if(ik_ret.result == IkRet::IK_OK){
                    temp_q_ang = ik_ret.q_out_deg;
                    _motion_move_l.Update_Timer(System_DT * ik_ret.time_scaler * temp_time_scaler);

                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }else{
                    LOG_ERROR("IK ERR ... !!! " + std::to_string((int)ik_ret.result));
                    Terminate_Motion(MoveTerminate::IK_ERR);
                }
                break;
            }
            case MotionMode::MOVE_JB:
            {
                move_jb_control_ret ret = _motion_move_jb.Control(-1);

                if(ret.is_thereErr != 0){
                    Terminate_Motion(MoveTerminate::TRAJ_ERR);
                }else{
                    temp_q_ang = ret.J_output;
                    _motion_move_jb.Update_Timer(System_DT * temp_time_scaler);
                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }
                break;
            }
            case MotionMode::MOVE_LB:
            {
                move_lb_control_ret ret = _motion_move_lb.Control(-1);

                VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
                VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
                VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
                VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

                IkResult ik_ret = _motion_Robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, System_DT, ret.L_output, _motion_x_pos
                , temp_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max, _motion_q_vel, 5.0, 1.0, ret.is_First);              

                if(ik_ret.result == IkRet::IK_OK){
                    temp_q_ang = ik_ret.q_out_deg;
                    _motion_move_lb.Update_Timer(System_DT * ik_ret.time_scaler * temp_time_scaler);

                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }else{
                    LOG_ERROR("IK ERR ... !!! " + std::to_string((int)ik_ret.result));
                    Terminate_Motion(MoveTerminate::IK_ERR);
                }
                break;
            }
            case MotionMode::MOVE_XB:
            {
                move_xb_control_ret ret = _motion_move_xb.Control(-1, temp_q_ang, temp_q_vel);

                if(ret.is_thereErr != 0){
                    Terminate_Motion(MoveTerminate::IK_ERR);
                }else{
                    temp_q_ang = ret.J_output;
                    _motion_move_xb.Update_Timer(System_DT * ret.timing_belt * temp_time_scaler);
                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }
                break;
            }
            case MotionMode::MOVE_CIR:
            {   
                move_cir_control_ret ret = _motion_move_cir.Control(-1);

                VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
                VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
                VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
                VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

                IkResult ik_ret = _motion_Robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, System_DT, ret.L_output, _motion_x_pos
                , temp_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max, _motion_q_vel, 5.0, 1.0, ret.is_First);


                if(ik_ret.result == IkRet::IK_OK){
                    temp_q_ang = ik_ret.q_out_deg;
                    _motion_move_cir.Update_Timer(System_DT * ik_ret.time_scaler * temp_time_scaler);

                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }else{
                    LOG_ERROR("IK ERR ... !!! " + std::to_string((int)ik_ret.result));
                    Terminate_Motion(MoveTerminate::IK_ERR);
                }
                break;
            }
            case MotionMode::MOVE_SPD_J:
            {
                move_speed_j_control_ret ret = _motion_move_speed_j.Control(RT_PERIOD_SEC * temp_time_scaler);
                if(ret.is_thereErr != 0){
                    Terminate_Motion(MoveTerminate::TRAJ_ERR);
                }else{
                    temp_q_ang = ret.J_output;
                }
                break;
            }
            case MotionMode::MOVE_SPD_L:
            {
                move_speed_l_control_ret ret = _motion_move_speed_l.Control(is_motion_break, break_alpha);

                VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
                VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
                VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
                VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

                IkResult ik_ret = _motion_Robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, System_DT, ret.L_output, _motion_x_pos
                , temp_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max, _motion_q_vel, 5.0, 1.0, ret.is_First);

                // std::cout<<"SPD: "<<_motion_x_vel_3.norm()<<" / "<<(_motion_x_vel_rcv.norm() * MATH_R2D)<<" # "<<temp_time_scaler<<std::endl;

                if(ik_ret.result == IkRet::IK_OK){
                    temp_q_ang = ik_ret.q_out_deg;
                    _motion_move_speed_l.Update_Internal_Scaler(ik_ret.time_scaler * temp_time_scaler);

                    if(ret.is_finished){
                        Terminate_Motion(MoveTerminate::DONE);
                    }
                }else{
                    LOG_ERROR("IK ERR ... !!! " + std::to_string((int)ik_ret.result));
                    Terminate_Motion(MoveTerminate::IK_ERR);
                }

                break;
            }
        }

        // -------------------------------------------------------------------
        // DON'T TOUCH AREA
        // -------------------------------------------------------------------
        _motion_q_vel = (temp_q_ang - _motion_q_ang) / System_DT;
        _motion_q_acc = (_motion_q_vel - temp_q_vel) / System_DT;
        _motion_q_ang = temp_q_ang;
        
        _motion_x_pos_old = _motion_x_pos;
        _motion_x_pos = _motion_Robot.Calc_ForwardKinematics(temp_q_ang);
        _motion_x_jacobdet = _motion_Robot.Calc_Jacobian_Det(temp_q_ang);

        _motion_x_vel_3 = (rb_math::get_P_3x1(_motion_x_pos) - rb_math::get_P_3x1(_motion_x_pos_old)) / RT_PERIOD_SEC;
        _motion_x_vel_rcv = rb_math::R_to_RCV(rb_math::get_R_3x3(_motion_x_pos) * rb_math::get_R_3x3(_motion_x_pos_old).transpose()) / RT_PERIOD_SEC;

        return {_motion_q_ang, _motion_x_pos};
    }

    void Set_Motion_Mode(MotionMode t_mode){
        if(_motion_mode != t_mode){
            LOG_INFO("Motion Mode Changed: " + std::to_string((int)_motion_mode) + "->" + std::to_string((int)t_mode));
        }
        _motion_mode = t_mode;
    }

    MotionMode Get_Motion_Mode(){
        return _motion_mode;
    }

    void Set_Motion_q(unsigned int idx, double angle){
        if(idx >= NO_OF_JOINT){
            return;
        }
        _motion_q_ang(idx) = angle;
    }

    VectorCd Get_Motion_X(){
        return _motion_x_pos;
    }

    VectorJd Get_Motion_J(){
        return _motion_q_ang;
    }

    void Wait_Motion_Finish(){
        while(1){
            if(_motion_mode == MotionMode::MOVE_NONE){
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    bool Motion_Condition_Checker(){
        return (Get_Is_Motion_Idle() && rb_system::Get_Is_Idle());
    }

    int Start_Motion_J(TARGET_INPUT tar, double vel_para, double acc_para, int mode){
        // mode
        // 0 : %
        // 1 : deg/s

        VectorJd j_vel = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd j_acc = VectorJd::Zero(NO_OF_JOINT, 1);
        if(mode == 0){
            vel_para = rb_math::saturation_Up(vel_para, 1.0);
            acc_para = rb_math::saturation_Up(acc_para, 1.0);
            j_vel = rb_system::Get_Motor_HwMax_Vel() * vel_para;
            j_acc = rb_system::Get_Motor_HwMax_Acc() * acc_para;
        }else{
            for(int k = 0; k < NO_OF_JOINT; ++k){
                j_vel(k) = vel_para;
                j_acc(k) = acc_para;
            }
        }

        return Start_Motion_J(tar, j_vel, j_acc);
    }

    int Start_Motion_J(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_J(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        VectorJd j_tar = pre_t.j_output;
        VectorJd j_sta = _motion_q_ang;

        std::cout<<"j_vel: "<<j_vel.transpose()<<std::endl;

        for(int k = 0; k < NO_OF_JOINT; ++k){
            if(j_vel(k) < 0.){
                j_vel(k) = rb_system::Get_Motor_Reco_Vel()(k);
            }else{
                j_vel(k) = rb_math::saturation_Up(j_vel(k), rb_system::Get_Motor_Limit_Vel()(k) * 0.95);
            }

            if(j_acc(k) < 0.){
                j_acc(k) = rb_system::Get_Motor_Reco_Acc()(k);
            }else{
                j_acc(k) = rb_math::saturation_Up(j_acc(k), rb_system::Get_Motor_HwMax_Acc()(k) * 0.95);
            }
        }

        int run_ret = _motion_move_j.Init(j_sta, j_tar, j_vel, j_acc);
        if(run_ret != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }

        Set_Motion_Mode(MotionMode::MOVE_J);

        return MSG_OK;
    }

    int Start_Motion_L(TARGET_INPUT tar, double vel_para, double acc_para, int mode){
        // mode
        // 0 : %
        // 1 : mm/s

        double t_pos_vel = 0;
        double t_pos_acc = 0;
        double t_rot_vel = 0;
        double t_rot_acc = 0;

        if(mode == 0){
            vel_para = rb_math::saturation_Up(vel_para, 1.0);
            acc_para = rb_math::saturation_Up(acc_para, 1.0);

            t_pos_vel = vel_para * rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
            t_pos_acc = acc_para * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
            t_rot_vel = vel_para * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
            t_rot_acc = acc_para * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;
        }else{
            double l_vel_alpha = vel_para / rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
            double l_acc_alpha = acc_para / rb_system::Get_CurrentRobotParameter().arm_max_pos_acc; 
            l_vel_alpha = rb_math::saturation_Up(l_vel_alpha, 1.0);
            l_acc_alpha = rb_math::saturation_Up(l_acc_alpha, 1.0);
            t_pos_vel = vel_para;
            t_pos_acc = acc_para;
            t_rot_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
            t_rot_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;
        }
        return Start_Motion_L(tar, t_pos_vel, t_pos_acc, t_rot_vel, t_rot_acc);
    }

    int Start_Motion_L(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_L(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        int run_ret = _motion_move_l.Init(_motion_x_pos, pre_t.c_output, p_vel, p_acc, o_vel, o_acc);
        if(run_ret != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }

        Set_Motion_Mode(MotionMode::MOVE_L);
        return MSG_OK;
    }

    int Start_Motion_JB_Clear(){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        _motion_move_jb.Clear(_motion_q_ang);
        return MSG_OK;
    }

    int Start_Motion_JB_Add(TARGET_INPUT tar, double vel_para, double acc_para, int mode, int blend_option, double blend_parameter){
        // mode
        // 0 : %
        // 1 : deg/s

        VectorJd j_vel = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd j_acc = VectorJd::Zero(NO_OF_JOINT, 1);
        if(mode == 0){
            vel_para = rb_math::saturation_Up(vel_para, 1.0);
            acc_para = rb_math::saturation_Up(acc_para, 1.0);
            j_vel = rb_system::Get_Motor_HwMax_Vel() * vel_para;
            j_acc = rb_system::Get_Motor_HwMax_Acc() * acc_para;
        }else{
            for(int k = 0; k < NO_OF_JOINT; ++k){
                j_vel(k) = vel_para;
                j_acc(k) = acc_para;
            }
        }
        return Start_Motion_JB_Add(tar, j_vel, j_acc, blend_option, blend_parameter);
    }

    int Start_Motion_JB_Add(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc, int blend_option, double blend_parameter){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_J(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        //VectorJd j_tar = pre_t.j_output;

        for(int k = 0; k < NO_OF_JOINT; ++k){
            if(j_vel(k) < 0.){
                j_vel(k) = rb_system::Get_Motor_Reco_Vel()(k);
            }else{
                j_vel(k) = rb_math::saturation_Up(j_vel(k), rb_system::Get_Motor_Limit_Vel()(k));
            }
            
            if(j_acc(k) < 0.){
                j_acc(k) = rb_system::Get_Motor_Reco_Acc()(k);
            }else{
                j_acc(k) = rb_math::saturation_Up(j_acc(k), rb_system::Get_Motor_HwMax_Acc()(k));
            }
        }

        int add_ret = _motion_move_jb.Add(pre_t.j_output, j_vel, j_acc, blend_option, blend_parameter);
        if(add_ret != 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_PATH_ADDING_ERR);
        }
        return MSG_OK;
    }

    int Start_Motion_JB(){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        int run_ret = _motion_move_jb.Init(rb_system::Get_Motor_Limit_Vel());
        if(run_ret != 0){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }

        Set_Motion_Mode(MotionMode::MOVE_JB);
        return MSG_OK;
    }

    int Start_Motion_LB_Clear(){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        _motion_move_lb.Clear(_motion_x_pos);
        return MSG_OK;
    }

    int Start_Motion_LB_Add(TARGET_INPUT tar, double vel_para, double acc_para, int mode, int point_type, double blend_parameter){
        // mode
        // 0 : %
        // 1 : mm/s

        double t_pos_vel = 0;
        double t_pos_acc = 0;
        double t_rot_vel = 0;
        double t_rot_acc = 0;

        if(mode == 0){
            vel_para = rb_math::saturation_Up(vel_para, 1.0);
            acc_para = rb_math::saturation_Up(acc_para, 1.0);

            t_pos_vel = vel_para * rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
            t_pos_acc = acc_para * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
            t_rot_vel = vel_para * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
            t_rot_acc = acc_para * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;
        }else{
            double l_vel_alpha = vel_para / rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
            double l_acc_alpha = acc_para / rb_system::Get_CurrentRobotParameter().arm_max_pos_acc; 
            l_vel_alpha = rb_math::saturation_Up(l_vel_alpha, 1.0);
            l_acc_alpha = rb_math::saturation_Up(l_acc_alpha, 1.0);
            t_pos_vel = vel_para;
            t_pos_acc = acc_para;
            t_rot_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
            t_rot_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;
        }

        return Start_Motion_LB_Add(tar, t_pos_vel, t_pos_acc, t_rot_vel, t_rot_acc, point_type, blend_parameter);
    }

    int Start_Motion_LB_Add(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc, int point_type, double blend_parameter){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_L(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        int add_ret = _motion_move_lb.Add(pre_t.c_output, p_vel, p_acc, o_vel, o_acc, point_type, blend_parameter);
        if(add_ret != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, add_ret);
        }
        return MSG_OK;
    }

    int Start_Motion_LB(int rot_mode, int filter_window){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        filter_window = rb_math::saturation_L_and_U(filter_window, 0, 200);
        
        int run_ret = _motion_move_lb.Init(rot_mode, filter_window);
        if(run_ret != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }
        Set_Motion_Mode(MotionMode::MOVE_LB);
        return MSG_OK; 
    }

    int Start_Motion_XB_Clear(){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        _motion_move_xb.Clear(_motion_q_ang, _motion_x_pos);
        return MSG_OK;
    }

    int Start_Motion_XB_Add(TARGET_INPUT tar, double g_vel_alpha, double g_acc_alpha, int blend_option, double blend_parameter, int xb_move_type){
        g_vel_alpha = rb_math::saturation_Up(g_vel_alpha, 1.0);
        g_acc_alpha = rb_math::saturation_Up(g_acc_alpha, 1.0);

        if(xb_move_type == 1){
            ;// Move Type : Linear
            return Start_Motion_XB_Add_L(tar, g_vel_alpha, g_acc_alpha, blend_option, blend_parameter);
        }else{
            ;// Move Type : Joint
            return Start_Motion_XB_Add_J(tar, g_vel_alpha, g_acc_alpha, blend_option, blend_parameter);
        }
    }

    int Start_Motion_XB_Add_J(TARGET_INPUT tar, double j_vel_alpha, double j_acc_alpha, int blend_option, double blend_parameter){
        VectorJd j_vel = rb_system::Get_Motor_HwMax_Vel() * rb_math::saturation_Up(j_vel_alpha, 1.0);
        VectorJd j_acc = rb_system::Get_Motor_HwMax_Acc() * rb_math::saturation_Up(j_acc_alpha, 1.0);
        return Start_Motion_XB_Add_J(tar, j_vel, j_acc, blend_option, blend_parameter);
    }

    int Start_Motion_XB_Add_J(TARGET_INPUT tar, VectorJd j_vel, VectorJd j_acc, int blend_option, double blend_parameter){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_J(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        RBDLrobot temp_robot;
        temp_robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        VectorCd temp_calc_X = temp_robot.Calc_ForwardKinematics(pre_t.j_output);

        // std::cout<<"j_vel: "<<j_vel.transpose()<<std::endl;
        // std::cout<<"j_acc: "<<j_acc.transpose()<<std::endl;

        int ret_add = _motion_move_xb.Add_J(pre_t.j_output, temp_calc_X, j_vel, j_acc, blend_option, blend_parameter);
        if(ret_add != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, ret_add);
        }

        return MSG_OK;
    }

    int Start_Motion_XB_Add_L(TARGET_INPUT tar, double l_vel_alpha, double l_acc_alpha, int blend_option, double blend_parameter){
        l_vel_alpha = rb_math::saturation_Up(l_vel_alpha, 1.0);
        l_acc_alpha = rb_math::saturation_Up(l_acc_alpha, 1.0);
        double t_pos_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
        double t_pos_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
        double t_rot_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
        double t_rot_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;

        return Start_Motion_XB_Add_L(tar, t_pos_vel, t_pos_acc, t_rot_vel, t_rot_acc, blend_option, blend_parameter);
    }

    int Start_Motion_XB_Add_L(TARGET_INPUT tar, double p_vel, double p_acc, double o_vel, double o_acc, int blend_option, double blend_parameter){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_t = PreMotion_to_L(tar, 0);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        RBDLrobot temp_robot;
        temp_robot.Init_Robot(rb_system::Get_CurrentRobotParameter(), rb_system::Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        IkResult ik_ = temp_robot.Calc_InverseKinematics_Static(pre_t.c_output, 100, 100, _motion_move_xb.Get_LastAdd_X(), _motion_move_xb.Get_LastAdd_J());
        if(ik_.result == IkRet::IK_OK){
            int ret_add = _motion_move_xb.Add_X(pre_t.c_output, ik_.q_out_deg, p_vel, p_acc, o_vel, o_acc, blend_option, blend_parameter);
            if(ret_add != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, ret_add);
            }
            return MSG_OK;
        }else{
            std::cout<<"IK err: "<<(int)ik_.result<<std::endl;
            return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_PATH_IK_ERR);
        }
    }

    int Start_Motion_XB(int mode){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
        VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
        VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
        VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

        int run_ret = _motion_move_xb.Init(mode, _motion_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max);
        if(run_ret != 0){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }
        Set_Motion_Mode(MotionMode::MOVE_XB);
        return MSG_OK;
    }
    
    int Start_Motion_CIR_AXIS(TARGET_INPUT center, TARGET_INPUT axis, double angle, int rot_mode, double l_vel_alpha, double l_acc_alpha, int radi_mode, double radi_para){
        l_vel_alpha = rb_math::saturation_Up(l_vel_alpha, 1.0);
        l_acc_alpha = rb_math::saturation_Up(l_acc_alpha, 1.0);
        double t_pos_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
        double t_pos_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
        double t_rot_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
        double t_rot_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;

        return Start_Motion_CIR_AXIS(center, axis, rot_mode, angle, t_pos_vel, t_pos_acc, t_rot_vel, t_rot_acc, radi_mode, radi_para);
    }

    int Start_Motion_CIR_AXIS(TARGET_INPUT center, TARGET_INPUT axis, double angle, int rot_mode, double p_vel, double p_acc, double o_vel, double o_acc, int radi_mode, double radi_para){
        if(!Motion_Condition_Checker()) return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        PreMotionRet pre_cent = PreMotion_to_L(center, 0);
        if(pre_cent.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_cent.ret);

        PreMotionRet pre_axis = PreMotion_to_L(axis, 0);
        if(pre_axis.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_axis.ret);

        int run_ret = _motion_move_cir.Init(_motion_x_pos, rb_math::get_P_3x1(pre_cent.c_output), rb_math::get_P_3x1(pre_axis.c_output), angle
                            , rb_math::get_REDUN_1x1(_motion_x_pos)
                            , p_vel, p_acc, o_vel, o_acc
                            , rot_mode, radi_mode, radi_para);
        if(run_ret != MSG_OK){
            return MESSAGE(MSG_LEVEL_WARN, run_ret);
        }

        Set_Motion_Mode(MotionMode::MOVE_CIR);
        return MSG_OK;
    }

    int Start_Motion_CIR_3PNTS(TARGET_INPUT via, TARGET_INPUT tar, int rot_mode, double l_vel_alpha, double l_acc_alpha, int radi_mode, double radi_para){
        l_vel_alpha = rb_math::saturation_Up(l_vel_alpha, 1.0);
        l_acc_alpha = rb_math::saturation_Up(l_acc_alpha, 1.0);
        double t_pos_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_vel;
        double t_pos_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
        double t_rot_vel = l_vel_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_vel;
        double t_rot_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;

        return Start_Motion_CIR_3PNTS(via, tar, rot_mode, t_pos_vel, t_pos_acc, t_rot_vel, t_rot_acc, radi_mode, radi_para);
    }

    int Start_Motion_CIR_3PNTS(TARGET_INPUT via, TARGET_INPUT tar, int rot_mode, double p_vel, double p_acc, double o_vel, double o_acc, int radi_mode, double radi_para){
        PreMotionRet pre_via = PreMotion_to_L(via, 0);
        if(pre_via.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_via.ret);
        PreMotionRet pre_tar = PreMotion_to_L(tar, 0);
        if(pre_tar.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_tar.ret);

        Vector3d p1 = rb_math::get_P_3x1(_motion_x_pos);
        Vector3d p2 = rb_math::get_P_3x1(pre_via.c_output);
        Vector3d p3 = rb_math::get_P_3x1(pre_tar.c_output);

        Circle_3P_RET cir_fit = rb_math::fit_CircleFrom3Points(p1, p2, p3);
        if(cir_fit.result != true){
            return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_PATH_CIRCLE_FIT_ERR);
        }

        TARGET_INPUT temp_cent_input = rb_math::Make_Input_from_Vec3(cir_fit.center);
        TARGET_INPUT temp_axis_input = rb_math::Make_Input_from_Vec3(cir_fit.axis);
        return Start_Motion_CIR_AXIS(temp_cent_input, temp_axis_input, cir_fit.angle13, rot_mode, p_vel, p_acc, o_vel, o_acc, radi_mode, radi_para);
    }

    int Start_Motion_SPEED_J(TARGET_INPUT tar, double j_acc_alpha){
        if(!Get_Is_Motion_Idle()){
            if(_motion_mode != MotionMode::MOVE_SPD_J){
                return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);
            }
        }

        PreMotionRet pre_t = PreMotion_to_J(tar, 1);
        if(pre_t.ret != MSG_OK) return MESSAGE(MSG_LEVEL_WARN, pre_t.ret);

        if(_motion_mode == MotionMode::MOVE_SPD_J){
            int ret_update = _motion_move_speed_j.Set_Taget_Velocity(pre_t.j_output);
            if(ret_update != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, ret_update);
            }
        }else{
            VectorJd j_ang_lim_up = rb_system::Get_Motor_Limit_Ang_Up();
            VectorJd j_ang_lim_dw = rb_system::Get_Motor_Limit_Ang_Low();
            VectorJd j_vel_lim = rb_system::Get_Motor_HwMax_Vel();
            VectorJd j_acc_lim = rb_system::Get_Motor_HwMax_Acc() * j_acc_alpha;
            int run_ret = _motion_move_speed_j.Init(_motion_q_ang, j_ang_lim_up, j_ang_lim_dw, j_vel_lim, j_acc_lim);
            if(run_ret != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, run_ret);
            }
            int ret_update = _motion_move_speed_j.Set_Taget_Velocity(pre_t.j_output);
            if(ret_update != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, ret_update);
            }
            Set_Motion_Mode(MotionMode::MOVE_SPD_J);
        }
        return MSG_OK;
    }

    int Start_Motion_SPEED_L(TARGET_INPUT tar, double l_acc_alpha){
        if(!Get_Is_Motion_Idle()){
            if(_motion_mode != MotionMode::MOVE_SPD_L){
                return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);
            }
        }

        VectorCd interpreted_v = PreMotion_to_L_for_Speed(tar);

        if(_motion_mode == MotionMode::MOVE_SPD_L){
            int ret_update = _motion_move_speed_l.Set_Taget_Velocity(interpreted_v);
            if(ret_update != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, ret_update);
            }
        }else{
            double t_pos_acc = l_acc_alpha * rb_system::Get_CurrentRobotParameter().arm_max_pos_acc;
            double t_rot_acc = 0.2 * rb_system::Get_CurrentRobotParameter().arm_max_rot_acc;
            int run_ret = _motion_move_speed_l.Init(_motion_x_pos, t_pos_acc, t_rot_acc);
            if(run_ret != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, run_ret);
            }
            int ret_update = _motion_move_speed_l.Set_Taget_Velocity(interpreted_v);
            if(ret_update != MSG_OK){
                return MESSAGE(MSG_LEVEL_WARN, ret_update);
            }
            Set_Motion_Mode(MotionMode::MOVE_SPD_L);
        }
        return MSG_OK;
    }

    int Set_Motion_Shift(int shift_no, int shift_mode, TARGET_INPUT shift_target){
        if(shift_no < 0 || shift_no >= NO_OF_SHIFT || shift_mode < 0 || shift_mode > 1){
            return MSG_DESIRED_INDEX_IS_OVER_BOUND;
        }
        if(shift_target.target_frame < FRAME_GLOBAL || shift_target.target_frame >= FRAME_NUMBERS){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }

        SHIFT_INPUT temp_line;
        temp_line.mode = shift_mode;
        if(shift_mode == 0){
            temp_line.value = rb_math::Make_Input_Zero(FRAME_GLOBAL);
        }else{
            temp_line.value = shift_target;
        }

        _motion_shift[shift_no] = temp_line;
        return MSG_OK;
    }
    //--------------------------------------------------------------------------------------
    // Sector Wrapper
    //--------------------------------------------------------------------------------------
    std::tuple<VectorJd, VectorJd, VectorJd> Task_Wrapper_Handler(double System_DT, double External_Alpha, VectorJd pre_motion_q, VectorCd pre_motion_x){
        VectorJd temp_q_ang = _wrapper_q_ang;
        VectorJd temp_q_vel = _wrapper_q_vel;

        double temp_time_scaler = External_Alpha;

        switch(_wrapper_mode){
            case WrapperMode::WRAPPER_NONE:
            {
                temp_q_ang = pre_motion_q;
                break;
            }
            case WrapperMode::WRAPPER_CONV:
            {
                VectorCd current_target_x = pre_motion_x;
                current_target_x.block(0, 0, 3, 1) = current_target_x.block(0, 0, 3, 1) + _wrapper_sum;

                VectorJd temp_q_min = rb_system::Get_Motor_Limit_Ang_Low();
                VectorJd temp_q_max = rb_system::Get_Motor_Limit_Ang_Up();
                VectorJd temp_dq_max = rb_system::Get_Motor_Limit_Vel();
                VectorJd temp_ddq_max = rb_system::Get_Motor_HwMax_Acc();

                IkResult ik_ret = _wrapper_Robot.Calc_InverseKinematics_Loop(IkMode::IK_GENERAL, System_DT, current_target_x, _wrapper_x_pos
                , temp_q_ang, temp_q_min, temp_q_max, temp_dq_max, temp_ddq_max, _wrapper_q_vel, 5.0, 1.0, _wrapper_is_First);
                if(_wrapper_is_First){
                    _wrapper_is_First = false;
                }

                if(ik_ret.result == IkRet::IK_OK){
                    temp_q_ang = ik_ret.q_out_deg;

                    _wrapper_sum += (_wrapper_vel * System_DT * ik_ret.time_scaler * temp_time_scaler);

                    // std::cout<<"_wrapper_sum: "<<_wrapper_sum.transpose()<<std::endl;
                }else{
                    LOG_ERROR("IK ERR ... !!! " + std::to_string((int)ik_ret.result));
                    Terminate_Wrapper(MoveTerminate::IK_ERR);
                }
                break;
            }
        }

        // -------------------------------------------------------------------
        // DON'T TOUCH AREA
        // -------------------------------------------------------------------
        _wrapper_q_vel = (temp_q_ang - _wrapper_q_ang) / System_DT;
        _wrapper_q_acc = (_wrapper_q_vel - temp_q_vel) / System_DT;
        _wrapper_q_ang = temp_q_ang;

        _wrapper_x_pos = _wrapper_Robot.Calc_ForwardKinematics(temp_q_ang);
        _wrapper_x_jacobdet = _wrapper_Robot.Calc_Jacobian_Det(temp_q_ang);

        return {_wrapper_q_ang, _wrapper_q_vel, _wrapper_q_acc};
    }

    void Set_Wrapper_q(unsigned int idx, double angle){
        if(idx >= NO_OF_JOINT){
            return;
        }
        _wrapper_q_ang(idx) = angle;
    }

    std::array<float, NO_OF_CARTE> Get_Wrapper_X(){
        std::array<float, NO_OF_CARTE> ret;
        for(int i = 0; i < NO_OF_CARTE; ++i){
            ret[i] = _wrapper_x_pos(i);
        }
        return ret;
    }

    std::array<float, NO_OF_CARTE> Get_Wrapper_X_User(){
        VectorCd glob_pos = _wrapper_x_pos;
        VectorCd user_pos = glob_pos;

        USERF_CONFIG user_f = rb_system::Get_CurrentUserFrameParameter();

        Matrix3d u_R = user_f.userf_rotation.transpose() * rb_math::get_R_3x3(glob_pos);
        Vector3d u_E = rb_math::R_to_RPY(u_R);
        Vector3d u_P = user_f.userf_rotation.transpose() * (rb_math::get_P_3x1(glob_pos) - user_f.userf_offset);

        user_pos.block(0, 0, 3, 1) = u_P;
        user_pos.block(3, 0, 3, 1) = u_E;

        std::array<float, NO_OF_CARTE> ret;
        for(int i = 0; i < NO_OF_CARTE; ++i){
            ret[i] = user_pos(i);
        }
        return ret;
    }

    std::array<float, NO_OF_JOINT> Get_Wrapper_J(){
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret[i] = _wrapper_q_ang(i);
        }
        return ret;
    }

    void Wait_Wrapper_Finish(){
        while(1){
            if(_wrapper_mode == WrapperMode::WRAPPER_NONE){
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
    }

    int Start_Wrapper_Conv(Eigen::Vector3d target_vel_mmps){
        _wrapper_sum = Vector3d(0, 0, 0);
        _wrapper_vel = target_vel_mmps;
        _wrapper_is_First = true;

        Set_Wrapper_Mode(WrapperMode::WRAPPER_CONV);
        return MSG_OK;
    }

    int Stop_Wrapper_Conv(){
        Set_Wrapper_Mode(WrapperMode::WRAPPER_NONE);
        return MSG_OK;
    }
    
    ;
}