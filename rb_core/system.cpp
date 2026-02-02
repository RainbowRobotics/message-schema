#define P_NAME  "SYSTEM"

#include <mutex>
#include <condition_variable>
// #include <memory.h>
// #include <fcntl.h>
// #include <sys/mman.h>
// #include <sys/stat.h>        /* For mode constants */

#include "system.h"
#include "motion.h"
#include "common.h"
#include "configcall.h"

#include "shareddata.h"

#include "lan2can.h"
#include "scb_v1.h"
#include "side_io.h"
#include "motor.h"
#include "ledlight.h"
#include "toolflange.h"
#include "rrbdl.h"

#include "rb_ipc/ipc.h"
#include "rb_service/socket_server/command_server.h"

bool is_save = false;

lan2can         *_gv_Handler_Lan;
scb_v1          *_gv_Handler_SCB;
side_io         *_gv_Handler_Side;
motor           *_gv_Handler_Motor[NO_OF_JOINT];
ledlight        *_gv_Handler_Ledlight;
toolflange      *_gv_Handler_Toolflange;

namespace rb_system {
    namespace {
        // IO Def
        std::atomic<unsigned char>          request_PowerControl{0};       // 1=PowerOn, 2=PowerOff
        std::atomic<unsigned char>          request_ServoControl{0};       // 1=ServoOn
        std::atomic<unsigned char>          request_ReferenceControl{0};   // 1=toReal 2=toSimul
        std::atomic<unsigned char>          request_PyFMControl{0};        // 1=Start, 2=Stop, 3=Pause, 4=Resume
        enum IO_DEF_DIN{
            DIN_DEF_NONE = 0,
            DIN_DEF_R_PowerOn,
            DIN_DEF_R_PowerOff,
            DIN_DEF_R_ServoOn,
            DIN_DEF_R_ServoOn_F_PowerOff,
            DIN_DEF_R_RealMode,
            DIN_DEF_R_SimulMode,
            DIN_DEF_R_StopMove,
            DIN_DEF_R_PauseMove,
            DIN_DEF_R_ResumeMove,
            DIN_DEF_R_PauseMove_F_ResumeMove,
            DIN_DEF_R_ResumeColl,
            DIN_DEF_R_FreeDriveOn_F_FreeDriveOff,
            DIN_DEF_R_ProgramLoad,
            DIN_DEF_R_ProgramStop,
            DIN_DEF_R_ProgramStart,
            DIN_DEF_R_ProgramPause,
            DIN_DEF_R_ProgramResume,
            DIN_DEF_R_ProgramPause_F_Resume,
            DIN_DEF_NUM
        };
        enum IO_DEF_DOUT{
            DOUT_DEF_NONE = 0,
            DOUT_DEF_H_SystemReady,
            DOUT_DEF_H_HeartBeat1Sec,
            DOUT_DEF_H_ByPassDin,            
            DOUT_DEF_H_PowerOn,
            DOUT_DEF_H_ServoOn,
            DOUT_DEF_H_RealMode,
            DOUT_DEF_H_RealMode_and_Idle,
            DOUT_DEF_H_Idle,
            DOUT_DEF_H_Pause,
            DOUT_DEF_H_OutColl,
            DOUT_DEF_H_SelfColl,
            DOUT_DEF_H_OutColl_or_SelfColl,
            DOUT_DEF_H_FreeDrive,
            DOUT_DEF_H_PorgramLoaded,
            DOUT_DEF_H_ProgramRunning,
            DOUT_DEF_NUM
        };

        // RT
        volatile int64_t            g_max_jitter_ns = 0;
        pthread_mutex_t             mutex_shm = PTHREAD_MUTEX_INITIALIZER;

        // reuqest flag
        std::atomic<PowerOption>    request_powerControl{PowerOption::NONE};
        std::atomic<unsigned char>  request_ModelChange{0};

        // Heart Beat
        uint8_t                     flag_heart_beat = 0;
        int                         heart_beat_counter = 0;

        // system flag
        bool                        flag_soft_power_cutoff = false;
        bool                        flag_reference_onoff = false;
        bool                        flag_direct_teaching = false;
        bool                        flag_sw_switch_free_drive = false;

        // Multi Robot Working
        int                         parameter_id = 0;
        int                         parameter_id_friend = 1;
        int                         flag_master_mode = 0;
        std::string                 friend_robot_namespace = "";
        VectorCd                    m2s_starting_carte_slave;
        VectorCd                    m2s_starting_carte_master;
        
        // User Script
        std::string                 parameter_userscript[16];

        // Connection Detector
        bool                        flag_dev_configuration[NO_OF_JOINT + 3];//motor + tfb + side + emg
        bool                        flag_connection_components[NO_OF_JOINT + 2];//motor + tfb + side
        bool                        flag_connection_is_all = false;
        bool                        flag_connection_is_one = false;

        // Self Collision
        bool                        flag_collision_self_occur = false;
        int                         parameter_self_coll_mode = 0;
        float                       parameter_self_coll_dist_int = 0;
        float                       parameter_self_coll_dist_ext = 0;

        // Collision Detection
        bool                        flag_joint_impedance_mode = false;
        bool                        flag_collision_out_occur = false;
        VectorJd                    torque_esetimated = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    torque_delta_lpf = VectorJd::Zero(NO_OF_JOINT, 1);
        std::vector<rb_math::sloper_f> sloper_delta_torque(NO_OF_JOINT, rb_math::sloper_f(0, 0.04, 0.04));
        int                         torque_limit_A[NO_OF_JOINT];

        // Delta Watt
        double                      delta_Watt_lpf_dc_and_esti = 0.;
        rb_math::sloper_f           sloper_delta_Watt(0, 0.001, 0.002);
        double                      estimated_Temperature[NO_OF_JOINT];

        // Output Filtering
        double                      output_lpf_alpha = 0.1;
        VectorJd                    output_lpf_q_input = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    output_lpf_q_ang = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    output_lpf_q_ang_old = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    output_lpf_q_vel = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    output_lpf_q_vel_old = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd                    output_lpf_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);



        std::mutex                  power_mutex;
        std::condition_variable     power_cv;

        ServoState                  servo_stat;

        
        int                         parameter_code;
        ROBOT_CONFIG                parameter_robot;
        int                         parameter_out_coll_onoff;
        int                         parameter_out_coll_react;
        float                       parameter_out_coll_limit;
        int                         parameter_gravity_mode;
        Vector3d                    parameter_gravity_direction;
        VectorJd                    parameter_direct_teach_sensitivity;
        TCP_CONFIG                  parameter_tcp[NO_OF_TOOL];
        USERF_CONFIG                parameter_userf[NO_OF_USERF];
        AREA_CONFIG                 parameter_area[NO_OF_AREA];
        int                         parameter_special_dout_box[NO_OF_DOUT];
        int                         parameter_special_din_box[NO_OF_DIN];
        int                         parameter_filter_count_din_box[NO_OF_DIN];
        int                         userf_selection_num = 0;
        int                         tcp_selection_num = 0;

        rb_math::sloper_f           sloper_tfb_button(0, 0.002, 0.002);
        

        TimeScaler                  _sys_timescale[(int)SystemTimeScaler::NUM];
        bool                        flag_is_pause = false;
        bool                        flag_is_break = false;

        std::vector<int> Joint_Action_List_Generator(int bno){
            std::vector<int> shot_arr;
            if(bno < 0){
                for(int j = 0; j < NO_OF_JOINT; ++j){
                    shot_arr.push_back(j);
                }
            }else if(bno >= 0 && bno < NO_OF_JOINT){
                shot_arr.push_back(bno);
            }
            return shot_arr;
        }

        std::tuple<bool, bool, int, float> Calc_Self_Collision(ROBOT_CONFIG tRobot, VectorJd qDeg, float dist_int, float dist_ext){
            CAPSULE_STRUCT tCAP[NO_OF_JOINT + 1];

            Vector3d prev_P = Vector3d::Zero(3, 1);
            Matrix3d prev_R = Matrix3d::Identity(3, 3);
            for(int i = 0; i < NO_OF_JOINT; ++i){
                Vector3d motor_P = prev_P + prev_R * tRobot.link_offset[i];

                tCAP[i].p0 = prev_P  + prev_R * tRobot.link_capsule_dw[i];
                tCAP[i].p1 = motor_P + prev_R * tRobot.link_capsule_up[i];
                tCAP[i].radius = tRobot.link_capsule_radi[i];

                Matrix3d tR = Matrix3d::Identity(3, 3);
                if(tRobot.modules_axis[i] == 1){
                    tR = rb_math::Ry(qDeg(i) * MATH_D2R);
                }else if(tRobot.modules_axis[i] == 2){
                    tR = rb_math::Rz(qDeg(i) * MATH_D2R);
                }else{
                    tR = rb_math::Rx(qDeg(i) * MATH_D2R);
                }

                prev_R *= tR;
                prev_P = motor_P;
            }

            Vector3d ee_P = prev_P + prev_R * tRobot.link_ee_offset;
            tCAP[NO_OF_JOINT].p0 = prev_P + prev_R * tRobot.link_ee_capsule_dw;
            tCAP[NO_OF_JOINT].p1 = ee_P   + prev_R * tRobot.link_ee_capsule_up;
            tCAP[NO_OF_JOINT].radius = tRobot.link_ee_capsule_radi;

            bool is_self_coll_occur = false;
            bool is_elbow_related_self_coll = true;
            int  which_combination = -1;
            float shortest_dist = 999999999.;
            for(int i = 0; i < tRobot.self_coll_check_list.size(); ++i){
                int segA = tRobot.self_coll_check_list.at(i).ind_A;
                int segB = tRobot.self_coll_check_list.at(i).ind_B;

                float dist_cover = rb_math::ShortestDistance_Cap_Cap(tCAP[segA], tCAP[segB]) - tCAP[segA].radius - tCAP[segB].radius;
                if(dist_cover < dist_int){
                    is_self_coll_occur = true;
                    // --------------------------------------------------------------------------
                    // ARM vs ARM
                    // --------------------------------------------------------------------------
                    if(dist_cover < shortest_dist)  which_combination = i + 0;// 0   : arm <-> arm
                }
                shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
            }

            TCP_CONFIG c_tool = Get_CurrentTcpParameter();
            if(c_tool.box_type != 0){
                Vector3d tool_center_P = ee_P + prev_R * Vector3d(c_tool.box_parameter[0], c_tool.box_parameter[1], c_tool.box_parameter[2]);
                Matrix3d tool_center_R = prev_R * rb_math::RPY_to_R(c_tool.box_parameter[3], c_tool.box_parameter[4], c_tool.box_parameter[5]);
                if(c_tool.box_type == 1 || c_tool.box_type == 2){
                    float capsule_height = 0.5;
                    if(c_tool.box_type == 2)    capsule_height = c_tool.box_parameter[8] * 0.5;

                    CAPSULE_STRUCT temp_tool;
                    temp_tool.p0 = tool_center_P + tool_center_R * Vector3d(0, 0, -capsule_height);
                    temp_tool.p1 = tool_center_P + tool_center_R * Vector3d(0, 0, +capsule_height);
                    temp_tool.radius = 0.5 * c_tool.box_parameter[6];

                    for(int i = 0; i < (NO_OF_JOINT - 1); ++i){
                        float dist_cover = rb_math::ShortestDistance_Cap_Cap(tCAP[i], temp_tool) - tCAP[i].radius - temp_tool.radius;
                        if(dist_cover < dist_ext){
                            is_self_coll_occur = true;
                            // --------------------------------------------------------------------------
                            // ARM vs TOOL
                            // --------------------------------------------------------------------------
                            if(dist_cover < shortest_dist)  which_combination = i + 100;// 100 : arm <-> tool
                        }
                        shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                    }
                }else if(c_tool.box_type == 3){
                    BOX_STRUCT temp_box;
                    temp_box.box_center = tool_center_P;
                    temp_box.box_rot = tool_center_R;
                    temp_box.box_half = 0.5 * Vector3d(c_tool.box_parameter[6], c_tool.box_parameter[7], c_tool.box_parameter[8]);

                    for(int i = 0; i < (NO_OF_JOINT - 1); ++i){
                        float dist_cover = rb_math::ShortestDistance_Cap_Box(tCAP[i], temp_box) - tCAP[i].radius;
                        if(dist_cover < dist_ext){
                            is_self_coll_occur = true;
                            // --------------------------------------------------------------------------
                            // ARM vs TOOL
                            // --------------------------------------------------------------------------
                            if(dist_cover < shortest_dist)  which_combination = i + 100;// 100 : arm <-> tool
                        }
                        shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                    }
                }
            }

            for(int a = 0; a < NO_OF_AREA; ++a){
                AREA_CONFIG c_area = parameter_area[a];
                if(c_area.area_type != 0){
                    if(c_area.area_type == 1 || c_area.area_type == 2){
                        float area_height = 0.5;
                        if(c_area.area_type == 2)    area_height = c_area.area_parameter[2] * 0.5;

                        CAPSULE_STRUCT area_capsule;
                        area_capsule.p0 = c_area.area_offset + c_area.area_rotation * Vector3d(0, 0, -area_height);
                        area_capsule.p1 = c_area.area_offset + c_area.area_rotation * Vector3d(0, 0, +area_height);
                        area_capsule.radius = 0.5 * c_area.area_parameter[0];

                        // Area(CAP) <-> Robot Arm
                        for(int i = 0; i < (NO_OF_JOINT - 1); ++i){
                            float dist_cover = rb_math::ShortestDistance_Cap_Cap(tCAP[i], area_capsule) - tCAP[i].radius - area_capsule.radius;
                            if(dist_cover < dist_ext){
                                is_self_coll_occur = true;
                                // --------------------------------------------------------------------------
                                // AREA VS ARM
                                // --------------------------------------------------------------------------
                                if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + i + 0;// 1000 : arm <-> area
                            }
                            shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                        }
                        // Area(CAP) <-> Tool
                        if(c_tool.box_type != 0){
                            Vector3d tool_center_P = ee_P + prev_R * Vector3d(c_tool.box_parameter[0], c_tool.box_parameter[1], c_tool.box_parameter[2]);
                            Matrix3d tool_center_R = prev_R * rb_math::RPY_to_R(c_tool.box_parameter[3], c_tool.box_parameter[4], c_tool.box_parameter[5]);
                            if(c_tool.box_type == 1 || c_tool.box_type == 2){
                                float capsule_height = 0.5;
                                if(c_tool.box_type == 2)    capsule_height = c_tool.box_parameter[8] * 0.5;

                                CAPSULE_STRUCT tool_capsule;
                                tool_capsule.p0 = tool_center_P + tool_center_R * Vector3d(0, 0, -capsule_height);
                                tool_capsule.p1 = tool_center_P + tool_center_R * Vector3d(0, 0, +capsule_height);
                                tool_capsule.radius = 0.5 * c_tool.box_parameter[6];

                                float dist_cover = rb_math::ShortestDistance_Cap_Cap(area_capsule, tool_capsule) - area_capsule.radius - tool_capsule.radius;
                                if(dist_cover < dist_ext){
                                    is_self_coll_occur = true;
                                    // --------------------------------------------------------------------------
                                    // AREA VS TOOL
                                    // --------------------------------------------------------------------------
                                    if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + 0 + 100;// 1100 : tool <-> area
                                }
                                shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                            }else if(c_tool.box_type == 3){
                                BOX_STRUCT tool_box;
                                tool_box.box_center = tool_center_P;
                                tool_box.box_rot = tool_center_R;
                                tool_box.box_half = 0.5 * Vector3d(c_tool.box_parameter[6], c_tool.box_parameter[7], c_tool.box_parameter[8]);

                                float dist_cover = rb_math::ShortestDistance_Cap_Box(area_capsule, tool_box) - area_capsule.radius;
                                if(dist_cover < dist_ext){
                                    is_self_coll_occur = true;
                                    // --------------------------------------------------------------------------
                                    // AREA VS TOOL
                                    // --------------------------------------------------------------------------
                                    if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + 0 + 100;
                                }
                                shortest_dist = rb_math::return_small(shortest_dist, dist_cover);// 1100 : tool <-> area
                            }
                        }
                    }else if(c_area.area_type == 3){
                        BOX_STRUCT area_box;
                        area_box.box_center = c_area.area_offset;
                        area_box.box_rot = c_area.area_rotation;
                        area_box.box_half = 0.5 * Vector3d(c_area.area_parameter[0], c_area.area_parameter[1], c_area.area_parameter[2]);

                        // Area(BOX) <-> Robot Arm
                        for(int i = 0; i < (NO_OF_JOINT - 1); ++i){
                            float dist_cover = rb_math::ShortestDistance_Cap_Box(tCAP[i], area_box) - tCAP[i].radius;
                            if(dist_cover < dist_ext){
                                is_self_coll_occur = true;
                                // --------------------------------------------------------------------------
                                // AREA VS ARM
                                // --------------------------------------------------------------------------
                                if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + i + 0;// 1000 : arm <-> area
                            }
                            shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                        }

                        // Area(BOX) <-> Tool
                        if(c_tool.box_type != 0){
                            Vector3d tool_center_P = ee_P + prev_R * Vector3d(c_tool.box_parameter[0], c_tool.box_parameter[1], c_tool.box_parameter[2]);
                            Matrix3d tool_center_R = prev_R * rb_math::RPY_to_R(c_tool.box_parameter[3], c_tool.box_parameter[4], c_tool.box_parameter[5]);
                            if(c_tool.box_type == 1 || c_tool.box_type == 2){
                                float capsule_height = 0.5;
                                if(c_tool.box_type == 2)    capsule_height = c_tool.box_parameter[8] * 0.5;

                                CAPSULE_STRUCT tool_capsule;
                                tool_capsule.p0 = tool_center_P + tool_center_R * Vector3d(0, 0, -capsule_height);
                                tool_capsule.p1 = tool_center_P + tool_center_R * Vector3d(0, 0, +capsule_height);
                                tool_capsule.radius = 0.5 * c_tool.box_parameter[6];

                                float dist_cover = rb_math::ShortestDistance_Cap_Box(tool_capsule, area_box) - tool_capsule.radius;
                                if(dist_cover < dist_ext){
                                    is_self_coll_occur = true;
                                    // --------------------------------------------------------------------------
                                    // AREA VS TOOL
                                    // --------------------------------------------------------------------------
                                    if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + 0 + 100;
                                }
                                shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                            }else if(c_tool.box_type == 3){
                                BOX_STRUCT tool_box;
                                tool_box.box_center = tool_center_P;
                                tool_box.box_rot = tool_center_R;
                                tool_box.box_half = 0.5 * Vector3d(c_tool.box_parameter[6], c_tool.box_parameter[7], c_tool.box_parameter[8]);

                                float dist_cover = rb_math::ShortestDistance_Box_Box(tool_box, area_box);
                                if(dist_cover < dist_ext){
                                    is_self_coll_occur = true;
                                    // --------------------------------------------------------------------------
                                    // AREA VS TOOL
                                    // --------------------------------------------------------------------------
                                    if(dist_cover < shortest_dist)  which_combination = (a + 1) * 1000 + 0 + 100;// 1100 : tool <-> area
                                }
                                shortest_dist = rb_math::return_small(shortest_dist, dist_cover);
                            }
                        }
                    }
                }// if area is configed
            } // for loop AREA

            //which_combination
            // 0 ~ 99     : arm  <-> arm
            // 100 ~ 199  : arm  <-> tool
            // 1000 : arm <-> area // a * 1000 + i
            // 1100 : tool <-> area // a * 1000  + 100

            return {is_self_coll_occur, is_elbow_related_self_coll, which_combination, shortest_dist};
        }

        void Config_TimeScaler(SystemTimeScaler sts, double value, double alpha){
            value = rb_math::saturation_L_and_U(value, 0.0, 1.0);
            alpha = rb_math::saturation_L_and_U(alpha, 0.0, 1.0);

            TimeScaler temp_set;
            temp_set.input = temp_set.output = value;
            temp_set.lpf_alpha = alpha;

            _sys_timescale[(int)sts] = temp_set;
        }

        void Set_Servo_State(ServoState t_state){
            if(servo_stat != t_state){
                LOG_INFO("ServoState: " + std::to_string((int)t_state));
            }
            servo_stat = t_state;
        }

        void Set_Direct_Teaching_Flag(bool t_flag){
            if(t_flag){
                if(!flag_reference_onoff){
                    LOG_WARNING("Cannot Ex DT: Ref is Off");
                }else if(!rb_motion::Get_Is_Idle()){
                    LOG_WARNING("Cannot Ex DT: Not Idle");
                }else{
                    if(flag_direct_teaching != t_flag){
                        LOG_INFO("RUN DT Mode");
                    }
                    flag_direct_teaching = t_flag;
                }
            }else{
                if(flag_direct_teaching != t_flag){
                    for(int i = 0; i < NO_OF_JOINT; i++){
                        _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_MakeErrorSumZero(0));
                    }
                    LOG_INFO("STOP DT Mode");
                }
                flag_direct_teaching = t_flag;
            }
        }

        void RT_Update_My_Shared_Memory(){
            TCP_CONFIG cur_TCP = rb_system::Get_CurrentTcpParameter();
            Eigen::Vector3d cur_TCP_euler = rb_math::R_to_RPY(cur_TCP.tcp_rotation);

            ST_SHM_M2F_STATE_CORE state_coreT;

            state_coreT.heart_beat      = rb_system::Get_Flag_Heart_Beat();
            state_coreT.joint_q_ref     = rb_motion::Get_Wrapper_J();
            state_coreT.joint_q_enc     = rb_system::Get_Motor_Encoder();
            state_coreT.joint_t_esti    = rb_system::Get_Torque(1);
            state_coreT.joint_t_meas    = rb_system::Get_Torque(0);
            state_coreT.joint_temper    = rb_system::Get_Temperature(1);

            state_coreT.carte_x_ref     = rb_motion::Get_Wrapper_X();
            state_coreT.carte_x_enc     = rb_motion::Get_Wrapper_X();

            state_coreT.userf_selection_no  = rb_system::Get_CurrentUserFrameNumber();
            state_coreT.userf_x_ref         = rb_motion::Get_Wrapper_X_User();

            state_coreT.tool_selection_no   = rb_system::Get_CurrentTcpNumber();
            // state_coreT.tool_name = cur_TCP.tool_name;
            state_coreT.tool_tcp_x              = cur_TCP.tcp_offset(0);
            state_coreT.tool_tcp_y              = cur_TCP.tcp_offset(1);
            state_coreT.tool_tcp_z              = cur_TCP.tcp_offset(2);
            state_coreT.tool_tcp_rx             = cur_TCP_euler(0);
            state_coreT.tool_tcp_ry             = cur_TCP_euler(1);
            state_coreT.tool_tcp_rz             = cur_TCP_euler(2);
            state_coreT.tool_com_m              = cur_TCP.com_mass;
            state_coreT.tool_com_x              = cur_TCP.com_offset(0);
            state_coreT.tool_com_y              = cur_TCP.com_offset(1);
            state_coreT.tool_com_z              = cur_TCP.com_offset(2);

            state_coreT.cbox_digital_input      = rb_system::Get_Box_Din();
            state_coreT.cbox_digital_output     = rb_system::Get_Box_Dout();
            state_coreT.cbox_analog_input       = rb_system::Get_Box_Ain();
            state_coreT.cbox_analog_output      = rb_system::Get_Box_Aout();

            state_coreT.ex_digital_input        = rb_system::Get_EX_Din();
            state_coreT.ex_digital_output       = rb_system::Get_EX_Dout();
            state_coreT.ex_analog_input         = rb_system::Get_EX_Ain();
            state_coreT.ex_analog_output        = rb_system::Get_EX_Aout();

            state_coreT.tool_digital_input      = rb_system::Get_Tool_Din();
            state_coreT.tool_digital_output     = rb_system::Get_Tool_Dout();
            state_coreT.tool_analog_input       = rb_system::Get_Tool_Ain();
            state_coreT.tool_analog_output      = rb_system::Get_Tool_Aout();
            state_coreT.tool_voltage_output     = static_cast<float>(rb_system::Get_Tool_Voltage());

            state_coreT.motion_mode             = static_cast<uint8_t>(rb_motion::Get_Motion_Mode());
            state_coreT.motion_execution_result = static_cast<uint8_t>(rb_motion::Get_Motion_Ex_Result());
            state_coreT.motion_speed_bar        = static_cast<float>(rb_system::Get_MoveSpeedBar());
            state_coreT.motion_is_pause         = static_cast<uint8_t>(rb_system::Get_MovePauseState());

            state_coreT.status_lan2can          = static_cast<uint8_t>(rb_system::Get_Lan2Can_State());
            if(rb_system::Get_Lan2Can_State()){
                state_coreT.status_switch_emg   = static_cast<uint8_t>(rb_system::Get_Power_Switch());
                state_coreT.status_power_out    = static_cast<uint8_t>(rb_system::Get_Power());
            }else{
                state_coreT.status_switch_emg   = 0;
                state_coreT.status_power_out    = 0;
            }
            
            state_coreT.status_servo_num        = static_cast<uint8_t>(rb_system::Get_Servo());
            state_coreT.status_is_refon         = static_cast<uint8_t>(rb_system::Get_ReferenceOnOff());
            state_coreT.status_out_coll         = static_cast<uint8_t>(rb_system::Get_Flag_Out_Collision_Occur());
            state_coreT.status_self_coll        = static_cast<uint8_t>(rb_system::Get_Flag_Self_Collision_Occur());
            state_coreT.status_dt_mode          = static_cast<uint8_t>(rb_system::Get_Flag_Direct_Teaching());

            rb_shareddata::getGlobalShm()->robots[parameter_id].m2f_state_core = state_coreT;
        }

        void RT_Update_My_Intention_to_Friends(){
            if((parameter_id + parameter_id_friend) == 1){
                if(!rb_shareddata::getLocalShm()->robots[parameter_id_friend].m2f_state_core.status_is_refon){
                    if(flag_master_mode != 0){
                        LOG_WARNING("RESET MASTER FLAG");
                        flag_master_mode = 0;
                    }
                }
                if(flag_master_mode == 1){
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[0] = +rb_motion::Get_Wrapper_J().at(0);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[1] = -rb_motion::Get_Wrapper_J().at(1);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[2] = -rb_motion::Get_Wrapper_J().at(2);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[3] = +rb_motion::Get_Wrapper_J().at(3);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[4] = -rb_motion::Get_Wrapper_J().at(4);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[5] = +rb_motion::Get_Wrapper_J().at(5);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[6] = -rb_motion::Get_Wrapper_J().at(6);
                }else if(flag_master_mode == 2){
                    VectorCd m2s_current_carte_master;
                    for(int k = 0; k < NO_OF_CARTE; ++k){
                        m2s_current_carte_master(k) = rb_motion::Get_Wrapper_X().at(k);
                    }

                    Matrix3d m2s_delta_R = rb_math::get_R_3x3(m2s_starting_carte_master).transpose() * rb_math::get_R_3x3(m2s_starting_carte_slave);
                    Vector3d m2s_delta_P = rb_math::get_R_3x3(m2s_starting_carte_master).transpose() * (rb_math::get_P_3x1(m2s_starting_carte_slave) - rb_math::get_P_3x1(m2s_starting_carte_master));

                    Matrix3d new_slave_R = rb_math::get_R_3x3(m2s_current_carte_master) * m2s_delta_R;
                    Vector3d new_slave_E = rb_math::R_to_RPY(new_slave_R);
                    Vector3d new_slave_P = rb_math::get_P_3x1(m2s_current_carte_master) + rb_math::get_R_3x3(m2s_current_carte_master) * m2s_delta_P;
                    double   new_slave_A = rb_math::get_REDUN_1x1(m2s_starting_carte_slave);

                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[0] = new_slave_P(0);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[1] = new_slave_P(1);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[2] = new_slave_P(2);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[3] = new_slave_E(0);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[4] = new_slave_E(1);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[5] = new_slave_E(2);
                    rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_f[6] = new_slave_A;
                }

                rb_shareddata::getGlobalShm()->robots[parameter_id_friend].f2m_command.command_flag = flag_master_mode;
            }            
        }

        void RT_Copy_From_Shared_Memory(){
            rb_shareddata::copySharedDataToLocal();
        }

        double RT_SysSpeed_Handler(){
            _sys_timescale[(int)SystemTimeScaler::COLLISION_OUT].input = static_cast<double>(flag_collision_out_occur != true);
            if(parameter_self_coll_mode == 1){
                _sys_timescale[(int)SystemTimeScaler::COLLISION_SELF].input = static_cast<double>(flag_collision_self_occur != true);
            }else{
                _sys_timescale[(int)SystemTimeScaler::COLLISION_SELF].input = 1;
            }

            if(flag_is_pause){
                _sys_timescale[(int)SystemTimeScaler::PAUSE].input = 0.;
                _sys_timescale[(int)SystemTimeScaler::PAUSE].lpf_alpha = 0.980;
            }else{
                _sys_timescale[(int)SystemTimeScaler::PAUSE].input = 1.;
                _sys_timescale[(int)SystemTimeScaler::PAUSE].lpf_alpha = 0.995;
            }

            if(flag_is_break){
                if(rb_motion::Get_Is_Motion_Idle()){
                    _sys_timescale[(int)SystemTimeScaler::BREAK].input = 1.;
                    _sys_timescale[(int)SystemTimeScaler::BREAK].output = 1.;
                    _sys_timescale[(int)SystemTimeScaler::BREAK].lpf_alpha = 0.;
                    flag_is_break = false;
                }else{
                    double temp_out = _sys_timescale[(int)SystemTimeScaler::BREAK].output - _sys_timescale[(int)SystemTimeScaler::BREAK].lpf_alpha;
                    if(temp_out <= 0){
                        LOG_INFO("Break Finished");

                        _sys_timescale[(int)SystemTimeScaler::BREAK].input = 1.;
                        _sys_timescale[(int)SystemTimeScaler::BREAK].output = 1.;
                        _sys_timescale[(int)SystemTimeScaler::BREAK].lpf_alpha = 0.;
                        flag_is_break = false;

                        rb_motion::Set_Motion_Mode(rb_motion::MotionMode::MOVE_NONE);
                    }else{
                        _sys_timescale[(int)SystemTimeScaler::BREAK].output = temp_out;
                    }
                }
            }

            double ret_alpha = 1.;
            for(int i = 0 ; i < (int)SystemTimeScaler::NUM; ++i){
                if(i != ((int)SystemTimeScaler::BREAK)){
                    _sys_timescale[i].output = _sys_timescale[i].lpf_alpha * _sys_timescale[i].output + (1. - _sys_timescale[i].lpf_alpha) * _sys_timescale[i].input;
                }
                ret_alpha *= _sys_timescale[i].output;
            }
            ret_alpha = rb_math::saturation_L_and_U(ret_alpha, 0.0, 1.0);
            return ret_alpha;
        }

        bool RT_Collision_Checker(VectorJd &torque_esti, VectorJd &torque_meas, VectorJd &torque_limit){
            bool temp_is_delta_torque_big = false;
            for(int i = 0; i < NO_OF_JOINT; ++i){
                torque_delta_lpf(i) = 0.97 * torque_delta_lpf(i) + 0.03 * fabs(torque_esti(i) - torque_meas(i));
                double delta_thresh = torque_limit(i) * rb_math::filt_Line(parameter_out_coll_limit, 0, 1, 0.15, 0.5);

                double t_slopper_value = sloper_delta_torque.at(i).Update(static_cast<float>(torque_delta_lpf(i) > delta_thresh));
                if(t_slopper_value > 0.5){
                    temp_is_delta_torque_big = true;
                }
            }
            return temp_is_delta_torque_big;
        }

        void RT_StateChecker(){
            // Heart Beat
            heart_beat_counter++;
            if(heart_beat_counter >= RT_FREQUENCY){
                heart_beat_counter = 0;
                flag_heart_beat ^= 1;
            }
            
            // Arm Connection Check Update
            {
                bool temp_is_all_connected = true;
                bool temp_is_one_connected = false;
                for(int i = 0; i < NO_OF_JOINT; ++i){
                    if(flag_dev_configuration[i] == 0)  continue;

                    flag_connection_components[i] = _gv_Handler_Motor[i]->Set_ConnectionTimerUp(2);
                    temp_is_all_connected &= flag_connection_components[i];
                    temp_is_one_connected |= flag_connection_components[i];
                }
                if(flag_dev_configuration[NO_OF_JOINT] == 1){
                    flag_connection_components[NO_OF_JOINT] = _gv_Handler_Toolflange->Set_ConnectionTimerUp(2);
                    temp_is_all_connected &= flag_connection_components[NO_OF_JOINT];
                    temp_is_one_connected |= flag_connection_components[NO_OF_JOINT];
                }
                if(flag_dev_configuration[NO_OF_JOINT + 1] == 1){
                    flag_connection_components[NO_OF_JOINT + 1] = _gv_Handler_Side->Set_ConnectionTimerUp(2);
                }
                _gv_Handler_SCB->Set_ConnectionTimerUp(0, 1);
                _gv_Handler_SCB->Set_ConnectionTimerUp(1, 1);

                flag_connection_is_all = temp_is_all_connected;
                if(temp_is_one_connected != flag_connection_is_one){
                    if(flag_connection_is_one){
                        for(int i = 0; i < NO_OF_JOINT; ++i){
                            _gv_Handler_Motor[i]->Clear_Infos(1);
                        }
                        MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_POWER_OFF);
                        LOG_INFO("STATE POWER DIS-ENGAGED");
                    }else{
                        LOG_INFO("STATE POWER ENGAGED");
                    }
                }
                flag_connection_is_one = temp_is_one_connected;
            }

            // Power Component State Checker
            if(flag_connection_is_one){
                if(_gv_Handler_Lan->power_mc_sw_stat != 1){
                    Set_Power(PowerOption::Off, true);
                }
            }
            

            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i] == 0)  continue;

                bool is_there_critical_err_in_motor = false;
                if(flag_connection_components[i]){
                    auto m_param = _gv_Handler_Motor[i]->Get_Parameters();
                    auto m_infos = _gv_Handler_Motor[i]->Get_Infos();

                    float t_tq_limit = m_param.para_hwmax_torque_rept * 1.5;
                    if(fabs(m_infos.torque_Nm_movingFiltered) > t_tq_limit){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_GEAR_OVER_TORQUE_0 + i);
                        LOG_ERROR("Motor OverTorque: " + std::to_string(i) + "[" + std::to_string(fabs(m_infos.torque_Nm_movingFiltered)) + "/" + std::to_string(t_tq_limit) + "]");
                        is_there_critical_err_in_motor = true;
                    }

                    float delta_Temp = ((m_infos.torque_mA_movingFiltered * 0.0008165) * (m_infos.torque_mA_movingFiltered * 0.0008165) - m_param.para_temperature_esti[0] * (estimated_Temperature[i] - 30.))
                                        * RT_PERIOD_SEC / m_param.para_temperature_esti[1];
                    estimated_Temperature[i] += delta_Temp;
                    if(estimated_Temperature[i] <= 30.) estimated_Temperature[i] = 30.;
                    if(estimated_Temperature[i] > 128){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_ESTI_TMP_OVR_0 + i);
                        LOG_ERROR("Motor OverEstiTemp: " + std::to_string(i) + "[" + std::to_string(estimated_Temperature[i]) + "]");
                        is_there_critical_err_in_motor = true;
                    }
                }else{
                    estimated_Temperature[i] = 30;
                }

                if(is_there_critical_err_in_motor){
                    Set_Power(PowerOption::Off, true);
                    break;
                }
            }

            if(servo_stat == ServoState::DONE){
                for(int i = 0; i < (NO_OF_JOINT + 1); ++i){
                    if(flag_dev_configuration[i] == 0)  continue;

                    if(!flag_connection_components[i]){
                        if(i == NO_OF_JOINT){
                            MESSAGE(MSG_LEVEL_ERRR, MSG_TFB_DISCONNECTED_DURING_RUN);
                        }else{
                            MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_DISCONNECTED_DURING_RUN + i);
                        }
                        LOG_ERROR("COMM ERR " + std::to_string(i));
                        Set_Power(PowerOption::Off, true);
                        break;
                    }
                }
            }

            if(servo_stat == ServoState::DONE){
                bool is_there_critical_err_in_motor = false;
                for(int i = 0; i < NO_OF_JOINT; ++i){
                    if(flag_dev_configuration[i] == 0)  continue;

                    auto m_state = _gv_Handler_Motor[i]->Get_States();
                    auto m_infos = _gv_Handler_Motor[i]->Get_Infos();

                    if(m_state.b.BIG){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_BIG_0 + i);
                        LOG_ERROR("Motor Big Error: " + std::to_string(i));
                        is_there_critical_err_in_motor = true;
                    }else if(m_state.b.INP){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_INP_0 + i);
                        LOG_ERROR("Motor Input Error: " + std::to_string(i));
                        is_there_critical_err_in_motor = true;
                    }else if(m_state.b.CUR || m_state.b.CUR_BIG){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_CUR_0 + i);
                        LOG_ERROR("Motor Cur Error: " + std::to_string(i));
                        is_there_critical_err_in_motor = true;
                    }else if(m_state.b.JAM){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_JAM_0 + i);
                        LOG_ERROR("Motor Jam Error: " + std::to_string(i));
                        is_there_critical_err_in_motor = true;
                    }else if(m_state.b.EST_TMP){
                        MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_TMP_0 + i);
                        LOG_ERROR("Motor Tmp Error: " + std::to_string(i));
                        is_there_critical_err_in_motor = true;
                    }else if(m_infos.encoder_deg_error > 12){
                        if(flag_reference_onoff && flag_joint_impedance_mode == false){
                            MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_MAT_0 + i);
                            LOG_ERROR("Motor Mat Error: " + std::to_string(i));
                            is_there_critical_err_in_motor = true;
                        }
                    }
                }

                if(is_there_critical_err_in_motor){
                    Set_Power(PowerOption::Off, true);
                }
            }
        }

        void RT_Task_LED(){
            static unsigned short arm_led_tx_cnt = 0;
            arm_led_tx_cnt++;
            if(arm_led_tx_cnt > 400){
                arm_led_tx_cnt = 0;
                if(flag_connection_is_one){
                    ARMLED_CMD led_target;
                    if(servo_stat == ServoState::DONE){
                        led_target = ARMLED_CMD::ARM_LED_AFTER_ACTIVE;

                        if(flag_reference_onoff){
                            led_target = ARMLED_CMD::ARM_LED_AFTER_REAL_MODE;
                            if(!rb_motion::Get_Is_Idle()){
                                led_target = ARMLED_CMD::ARM_LED_ARM_MOVING;
                            }
                            if(flag_direct_teaching){
                                led_target = ARMLED_CMD::ARM_LED_ARM_GRAVITY_COM;
                            }
                        }
                        if(flag_collision_out_occur || flag_collision_self_occur){//event
                            led_target = ARMLED_CMD::ARM_LED_COLLISION;
                        }
                    }else{
                        led_target = ARMLED_CMD::ARM_LED_BEFORE_ACTIVE;
                    }

                    _gv_Handler_Lan->CAN_writeData(_gv_Handler_Ledlight->CmdLED(led_target));
                }
            }
        }

        void RT_IO_Handler(){
            _gv_Handler_Side->Update_Dout_Pulse(RT_PERIOD_SEC);
            
            //---------------------------------------------
            // Special Input
            //---------------------------------------------
            static sSTAT old_state = _gv_Handler_Side->Get_State();
            sSTAT cur_state = _gv_Handler_Side->Get_State();
            for(int i = 0; i < NO_OF_DIN; ++i){
                bool nor_edge = false;
                bool rev_edge = false;
                bool din = false;
                int sp_mode = parameter_special_din_box[i];
                if(sp_mode < 0){
                    sp_mode = abs(sp_mode);

                    nor_edge = static_cast<bool>((old_state.din_filt[i]== 1) && (cur_state.din_filt[i] == 0));
                    rev_edge = static_cast<bool>((old_state.din_filt[i]== 0) && (cur_state.din_filt[i] == 1));
                    din = !cur_state.din_filt[i];
                }else{
                    nor_edge = static_cast<bool>((old_state.din_filt[i]== 0) && (cur_state.din_filt[i] == 1));
                    rev_edge = static_cast<bool>((old_state.din_filt[i]== 1) && (cur_state.din_filt[i] == 0));
                    din = cur_state.din_filt[i];
                }
                switch(sp_mode){
                    case DIN_DEF_R_PowerOn:
                    {
                        if(nor_edge){
                            if(request_PowerControl.load(std::memory_order_relaxed) == 0){
                                request_PowerControl.store(1, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_PowerOff:
                    {
                        if(nor_edge){
                            if(request_PowerControl.load(std::memory_order_relaxed) == 0){
                                request_PowerControl.store(2, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ServoOn:
                    {
                        if(nor_edge){
                            if(request_ServoControl.load(std::memory_order_relaxed) == 0){
                                request_ServoControl.store(1, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ServoOn_F_PowerOff:
                    {
                        if(nor_edge){
                            if(request_ServoControl.load(std::memory_order_relaxed) == 0){
                                request_ServoControl.store(1, std::memory_order_relaxed);
                            }
                        }else if(rev_edge){
                            if(request_PowerControl.load(std::memory_order_relaxed) == 0){
                                request_PowerControl.store(2, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_RealMode:
                    {
                        if(nor_edge){
                            if(request_ReferenceControl.load(std::memory_order_relaxed) == 0){
                                request_ReferenceControl.store(1, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_SimulMode:
                    {
                        if(nor_edge){
                            if(request_ReferenceControl.load(std::memory_order_relaxed) == 0){
                                request_ReferenceControl.store(2, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_StopMove:
                    {
                        if(nor_edge){
                            rb_system::Call_Halt();
                        }
                        break;
                    }
                    case DIN_DEF_R_PauseMove:
                    {
                        if(nor_edge){
                            rb_system::Call_MovePause();
                        }
                        break;
                    }
                    case DIN_DEF_R_ResumeMove:
                    {
                        if(nor_edge){
                            rb_system::Call_MoveResume();
                        }
                        break;
                    }
                    case DIN_DEF_R_PauseMove_F_ResumeMove:
                    {
                        if(nor_edge){
                            rb_system::Call_MovePause();
                        }else if(rev_edge){
                            rb_system::Call_MoveResume();
                        }
                        break;
                    }
                    case DIN_DEF_R_ResumeColl:
                    {
                        if(nor_edge){
                            rb_system::Call_Reset_Out_Coll();
                        }
                        break;
                    }
                    case DIN_DEF_R_FreeDriveOn_F_FreeDriveOff:
                    {
                        if(nor_edge){
                            Set_Free_Drive_Mode(true, 1);
                        }else if(rev_edge){
                            Set_Free_Drive_Mode(true, 0);
                        }
                        break;
                    }
                    case DIN_DEF_R_ProgramLoad:
                    {
                        break;
                    }
                    case DIN_DEF_R_ProgramStop:
                    {
                        if(nor_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(1, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ProgramStart:
                    {
                        if(nor_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(2, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ProgramPause:
                    {
                        if(nor_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(3, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ProgramResume:
                    {
                        if(nor_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(4, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    case DIN_DEF_R_ProgramPause_F_Resume:
                    {
                        if(nor_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(3, std::memory_order_relaxed);
                            }
                        }else if(rev_edge){
                            if(request_PyFMControl.load(std::memory_order_relaxed) == 0){
                                request_PyFMControl.store(4, std::memory_order_relaxed);
                            }
                        }
                        break;
                    }
                    default:
                        break;
                }
            }
            old_state = cur_state;

            //---------------------------------------------
            // Special Output
            //---------------------------------------------
            for(int i = 0; i< NO_OF_DOUT; ++i){
                bool is_reverse = false;
                int temp_out = -1;
                int sp_mode = parameter_special_dout_box[i];
                if(sp_mode < 0){
                    is_reverse = true;
                    sp_mode = abs(sp_mode);
                }
                switch(sp_mode){
                    case DOUT_DEF_H_SystemReady:
                    {
                        temp_out = 1;
                        break;
                    }
                    case DOUT_DEF_H_HeartBeat1Sec:
                    {
                        temp_out = static_cast<int>(flag_heart_beat == 1);
                        break;
                    }
                    case DOUT_DEF_H_ByPassDin:
                    {
                        temp_out = static_cast<int>(cur_state.din_filt[i] == 1);
                        break;
                    }
                    case DOUT_DEF_H_PowerOn:
                    {
                        temp_out = static_cast<int>(flag_connection_is_one == true);
                        break;
                    }
                    case DOUT_DEF_H_ServoOn:
                    {
                        temp_out = static_cast<int>(Get_Servo() == ServoState::DONE);
                        break;
                    }
                    case DOUT_DEF_H_RealMode:
                    {
                        temp_out = static_cast<int>(flag_reference_onoff == true);
                        break;
                    }
                    case DOUT_DEF_H_RealMode_and_Idle:
                    {
                        temp_out = static_cast<int>(flag_reference_onoff == true);
                        temp_out &= static_cast<int>(rb_motion::Get_Is_Motion_Idle() == true);
                        break;
                    }
                    case DOUT_DEF_H_Idle:
                    {
                        temp_out = static_cast<int>(rb_motion::Get_Is_Motion_Idle() == true);
                        break;
                    }
                    case DOUT_DEF_H_Pause:
                    {
                        temp_out = static_cast<int>(flag_is_pause == true);
                        break;
                    }
                    case DOUT_DEF_H_OutColl:
                    {
                        temp_out = static_cast<int>(flag_collision_out_occur == true);
                        break;
                    }
                    case DOUT_DEF_H_SelfColl:
                    {
                        temp_out = static_cast<int>(flag_collision_self_occur == true);
                        break;
                    }
                    case DOUT_DEF_H_OutColl_or_SelfColl:
                    {
                        temp_out = static_cast<int>(flag_collision_out_occur == true);
                        temp_out |= static_cast<int>(flag_collision_self_occur == true);
                        break;
                    }
                    case DOUT_DEF_H_FreeDrive:
                    {
                        temp_out = static_cast<int>(flag_direct_teaching == true);
                        break;
                    }
                    case DOUT_DEF_H_PorgramLoaded:
                    {
                        break;
                    }
                    case DOUT_DEF_H_ProgramRunning:
                    {
                        break;
                    }
                    default:
                        break;
                }

                if(temp_out >= 0){
                    if(is_reverse){
                        temp_out = !temp_out;
                    }
                    Set_Box_Digital_Output(i, temp_out);
                }
            }

            //---------------------------------------------
            // Sending Handler
            //---------------------------------------------
            CAN_MSG side_msg_output_state = _gv_Handler_Side->GetCurrentOutputState();
            CAN_MSG side_msg_now = _gv_Handler_Side->CmdIOControl();
            static CAN_MSG side_msg_old = side_msg_now;
            if(_gv_Handler_Lan->Compare_CAN_MSG_Is_Different(side_msg_now, side_msg_old)
                || _gv_Handler_Lan->Compare_CAN_MSG_Is_Different(side_msg_now, side_msg_output_state)){
                _gv_Handler_Lan->CAN_writeData(side_msg_now);
            }
            side_msg_old = side_msg_now;
        }

        pthread_t hThread_sys_gen;
        void *thread_system_general(void *) {
            int non_rt_counter = 0;
            while(1){
                non_rt_counter++;
                if(non_rt_counter%50 == 0){
                    // origianl = 2msec = 2000us = 2000000ns
                    // 2% jitter = 40us = 40000ns
                    if(g_max_jitter_ns > 40000){
                        LOG_WARNING("System General Thread Jitter High: " + std::to_string(g_max_jitter_ns) + " ns");
                        g_max_jitter_ns = 0;
                    }
                }

                auto scb_info = _gv_Handler_SCB->Get_Infos();
                if(scb_info.connection_flag[0] && scb_info.connection_flag[1]){
                    if(scb_info.configure_done[0] != true || scb_info.configure_done[1] != true){
                        SAFETY_ONOFF_STRUCTURE temp_onoff;
                        temp_onoff.raw = 0;
                        temp_onoff.on_EM1 = 1;
                        int ret = Set_SafetyBoard_Para_Single(SFSET_CONFIG_SF_ONOFF, temp_onoff.raw);
                        if(ret == MSG_OK){
                            _gv_Handler_SCB->Set_ConfigDoneFlag(-1, true);
                            LOG_INFO("SCB CONFIG DONE");
                        }else{
                            std::cout<<"SCB Config fail"<<std::endl;
                        }
                    }
                }

                auto state_powerControl = request_powerControl.load(std::memory_order_relaxed);
                if (state_powerControl == PowerOption::Off) {
                    Set_Power(PowerOption::Off, false);
                    request_powerControl.store(PowerOption::NONE, std::memory_order_relaxed);
                    break;
                }

                unsigned char request_trigger = 0;
                if((request_trigger = request_PowerControl.load(std::memory_order_relaxed)) != 0){
                    if(request_trigger == 1){
                        rb_system::Set_Power(rb_system::PowerOption::On, false);
                    }else if(request_trigger == 2){
                        rb_system::Set_Power(rb_system::PowerOption::Off, false);
                    }
                    request_PowerControl.store(0, std::memory_order_relaxed);
                }
                if((request_trigger = request_ServoControl.load(std::memory_order_relaxed)) != 0){
                    if(request_trigger == 1){
                        rb_system::Set_Servo(1, 1);
                    }
                    request_ServoControl.store(0, std::memory_order_relaxed);
                }
                if((request_trigger = request_ReferenceControl.load(std::memory_order_relaxed)) != 0){
                    if(request_trigger == 1){
                        rb_system::Set_ReferenceOnOff(true);
                    }else if(request_trigger == 2){
                        rb_system::Set_ReferenceOnOff(false);
                    }
                    request_ReferenceControl.store(0, std::memory_order_relaxed);
                }
                if((request_trigger = request_PyFMControl.load(std::memory_order_relaxed)) != 0){
                    rb_ipc::toPyFM_FlowControl(((int)request_trigger) - 1);
                    request_PyFMControl.store(0, std::memory_order_relaxed);
                }
                
                std::this_thread::sleep_for(20ms);
            }
            return nullptr;
        }
    }

    bool initialize(std::string domain, int th_cpu_sysgen, int th_cpu_lan_connection){
        // ------------------------------------------------------
        // Call DB infomation
        // ------------------------------------------------------
        parameter_id    = rb_config::READ_System_Id_No();
        if(parameter_id == 0)   parameter_id_friend = 1;
        if(parameter_id == 1)   parameter_id_friend = 0;
        parameter_code  = rb_config::READ_Robot_Model();
        parameter_robot = rb_config::READ_Robot_Parameter(parameter_code);

        std::cout<<"ARM: "<<parameter_robot.arm_name<<std::endl;
        
        // ------------------------------------------------------
        // Get Dev Configuration
        // ------------------------------------------------------
        for(int i = 0; i < NO_OF_JOINT; ++i){
            flag_dev_configuration[i] = rb_config::READ_DEV_Motor(i);
        }
        flag_dev_configuration[NO_OF_JOINT + 0] = rb_config::READ_DEV_Tool_Flange();
        flag_dev_configuration[NO_OF_JOINT + 1] = rb_config::READ_DEV_Side_IO();
        flag_dev_configuration[NO_OF_JOINT + 2] = rb_config::READ_DEV_Emg_Switch();

        // ------------------------------------------------------
        // Get Dev Configuration
        // ------------------------------------------------------
        Recover_Out_Coll_Para();
        Recover_Self_Coll_Para();
        Recover_Gravity_Para();
        Recover_Direct_Teaching_Sensitivity();
        Recover_TcpParameter(-1);
        Recover_UserFrameParameter(-1);
        Recover_UserScript(-1);
        Recover_AreaParameter(-1);

        Recover_Box_Special_Dout(-1);
        Recover_Box_Special_Din(-1);
        Recover_Box_FilterCount_Din(-1);

        // ------------------------------------------------------
        // Make Instances
        // ------------------------------------------------------

        _gv_Handler_Lan = new lan2can(rb_config::READ_System_Gate_Port(), rb_config::READ_System_Gate_IP(0), rb_config::READ_System_Gate_IP(1), rb_config::READ_System_Gate_IP(2), rb_config::READ_System_Gate_IP(3), th_cpu_lan_connection, domain);
        _gv_Handler_SCB = new scb_v1();
        _gv_Handler_Side = new side_io(1);
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Motor[i] = new motor(i, parameter_robot.can_Ch[i], parameter_robot.mdr_target[i], rb_module::Get_Module_Info(parameter_robot.modules_type[i]).current_scaler);
        }
        _gv_Handler_Ledlight = new ledlight(parameter_robot.can_Ch[2]);
        _gv_Handler_Toolflange = new toolflange(parameter_robot.can_Ch[NO_OF_JOINT]);

        // ------------------------------------------------------
        for(int i = 0; i < NO_OF_DIN; ++i){
            _gv_Handler_Side->Set_Din_Filter_Count(i, parameter_filter_count_din_box[i]);
        }


        _gv_Handler_Lan->CAN_registerObserver(_gv_Handler_SCB);
        _gv_Handler_Lan->CAN_registerObserver(_gv_Handler_Side);
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Lan->CAN_registerObserver(_gv_Handler_Motor[i]);
            flag_connection_components[i] = false;
        }
        _gv_Handler_Lan->CAN_registerObserver(_gv_Handler_Ledlight);
        _gv_Handler_Lan->CAN_registerObserver(_gv_Handler_Toolflange);
        flag_connection_components[NO_OF_JOINT] = false;
        flag_connection_components[NO_OF_JOINT +1] = false;
        flag_connection_is_all = false;
        flag_connection_is_one = false;
        _gv_Handler_Lan->Power_register_state_callback([](){
            rb_system::Notify_PowerChanged();
        });

        for(int i = 0; i < NO_OF_JOINT; ++i){
            rb_module::Module_Info module_INFO = rb_module::Get_Module_Info(parameter_robot.modules_type[i]);

            mPARA setting_mot;
            setting_mot.para_enc_resol = module_INFO.encoder_resolution;
            setting_mot.para_reduction_rate = module_INFO.gear_ratio;
            setting_mot.para_pulse_to_deg = 360. / setting_mot.para_enc_resol / setting_mot.para_reduction_rate;
            setting_mot.para_torque_const = module_INFO.torque_Constant;
            setting_mot.para_r_q = module_INFO.resi_R_q;

            setting_mot.para_hwmax_torque_rept = module_INFO.torque_Max_Rept;
            setting_mot.para_hwmax_torque_momt = module_INFO.torque_Max_Moment;
            setting_mot.para_hwmax_vel = module_INFO.max_Vel;
            setting_mot.para_hwmax_acc = module_INFO.max_ACC;

            setting_mot.para_limit_angleDeg_Low = parameter_robot.modules_range_low[i];
            setting_mot.para_limit_angleDeg_Up = parameter_robot.modules_range_up[i];
            setting_mot.para_limit_speedDeg = setting_mot.para_hwmax_vel;
            setting_mot.para_limit_torqueNm = setting_mot.para_hwmax_torque_momt;
            setting_mot.para_limit_current_mA = module_INFO.max_Cur_mA;

            setting_mot.para_temperature_esti[0] = module_INFO.temperature_esti[0];
            setting_mot.para_temperature_esti[1] = module_INFO.temperature_esti[1];

            setting_mot.para_inertia = module_INFO.rotor_i;
            setting_mot.para_friction_a = module_INFO.rotor_a;
            setting_mot.para_friction_b = module_INFO.rotor_b;

            setting_mot.para_shake_pulse = module_INFO.shake_pulse;
            
            _gv_Handler_Motor[i]->Set_Parameters(setting_mot);
            torque_limit_A[i] = -1;
        }

        Config_TimeScaler(SystemTimeScaler::SPEEDBAR, 1.0, 0.99);//constant alpha
        Config_TimeScaler(SystemTimeScaler::USER, 1.0, 0.985);//constant alpha
        Config_TimeScaler(SystemTimeScaler::PAUSE, 1.0, 0.98);//alpha will change
        Config_TimeScaler(SystemTimeScaler::BREAK, 1.0, 0.0);//alpha will change
        Config_TimeScaler(SystemTimeScaler::COLLISION_OUT, 1.0, 0.96);
        Config_TimeScaler(SystemTimeScaler::COLLISION_SELF, 1.0, 0.0);

        if(rb_common::thread_create(thread_system_general, th_cpu_sysgen, std::string("RB_" + domain + "_SYSGEN"), hThread_sys_gen, NULL) != 0){
            return false;
        }

        return true;
    }

    void system_destroyer(){
        std::cout<<"Bye Bye"<<std::endl;
        pthread_mutex_destroy(&mutex_shm);  // 프로그램 종료 시 뮤텍스를 소멸
        std::cout << "Mutex destroyed!" << std::endl;
    }

    std::tuple<std::string, std::string, std::string, std::string> Get_System_Basic_Info(){
        std::string s_category = SYSTEM_CATEGORY;
        std::string s_model = "C" + std::to_string(parameter_code);

        std::string s_version = SYSTEM_VERSION;
        std::string s_alias = parameter_robot.arm_name;
        return {s_category, s_model, s_version, s_alias};
    }

    int Save_Robot_Code(int t_code, int option){
        // auto [temp_valid, temp_para] = rb_config::READ_Robot_Parameter(t_code);
        // if(temp_valid == false){
        //     return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_VALUE_IS_OVER_BOUND);
        // }
        if(!rb_config::WRITE_Robot_Model(t_code)){
            return MSG_SAVE_TO_DB_FAIL;
        }

        // ADJUST
        // NO ADJUST !!!!!!!!!!!!!!!!!!!!!!!!

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    ROBOT_CONFIG Get_CurrentRobotParameter(){
        return parameter_robot;
    }

    // ---------------------------------------------------------------
    // Multi Robot
    // ---------------------------------------------------------------
    int Get_System_ID_Index(){
        return parameter_id;
    }
    int Set_Master_Mode(int mode){
        // mode
        // 0 : Off
        // 1 : J / J
        // 2 : L / L
        LOG_INFO("MASTER MODE: " + std::to_string(mode));

        if(mode == 0){
            flag_master_mode = 0;
            return MSG_OK;
        }

        if(!Get_Is_Idle())  return MESSAGE(MSG_LEVEL_WARN, MSG_MOVE_COMMAND_ERR);

        if(mode == 1 || mode == 2){
            if(parameter_id == 0){
                friend_robot_namespace = "C501880";
            }else if(parameter_id == 1){
                friend_robot_namespace = "C500880";
            }else{
                return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_OPTION_IS_OVER_BOUND);
            }

            ST_SHM_M2F_STATE_CORE friends_core = rb_shareddata::getLocalShm()->robots[parameter_id_friend].m2f_state_core;

            for(int k = 0; k < NO_OF_CARTE; ++k){
                m2s_starting_carte_slave(k) = friends_core.carte_x_ref.at(k);
                m2s_starting_carte_master(k) = rb_motion::Get_Wrapper_X().at(k);
            }
        }else{
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_OPTION_IS_OVER_BOUND);
        }

        // {
        //     std::array<float, 7> target = rb_motion::Get_Wrapper_J();
        //     target.at(1) *= -1;
        //     target.at(2) *= -1;
        //     target.at(4) *= -1;
        //     target.at(6) *= -1;
        //     auto t0 = std::chrono::steady_clock::now();
        //     int ipc_ret = rb_ipc::toFriend_ServoJ(friend_robot_namespace, target, ((float)RT_PERIOD_SEC) * 2, 0.05, 1, 0.1);
        //     auto t1 = std::chrono::steady_clock::now();
        //     auto elapsed_us =
        //         std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        //     std::cout << "toFriend_ServoJ elapsed: "
        //             << elapsed_us << " us" << std::endl;
        //     std::cout<<"ipc_ret: "<<ipc_ret<<std::endl;
        //     return MSG_OK;
        // }
        flag_master_mode = mode;
        LOG_INFO("MASTER MODE: " + std::to_string(flag_master_mode) + "START");
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // JOINT
    // ---------------------------------------------------------------
    VectorJd Get_Motor_Limit_Ang_Low(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_limit_angleDeg_Low;
        }
        return ret;
    }
    VectorJd Get_Motor_Limit_Ang_Up(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_limit_angleDeg_Up;
        }
        return ret;
    }

    VectorJd Get_Motor_Limit_Vel(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_limit_speedDeg * 0.99;
        }
        return ret;
    }

    VectorJd Get_Motor_Reco_Vel(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_reco_vel;
        }
        return ret;
    }
    VectorJd Get_Motor_Reco_Acc(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_reco_acc;
        }
        return ret;
    }

    VectorJd Get_Motor_HwMax_Vel(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_hwmax_vel;
        }
        return ret;
    }
    VectorJd Get_Motor_HwMax_Acc(){
        VectorJd ret;
        for(int i = 0; i < NO_OF_JOINT; i++){
            ret(i) = _gv_Handler_Motor[i]->Get_Parameters().para_hwmax_acc;
        }
        return ret;
    }

    std::array<float, NO_OF_JOINT> Get_Motor_Encoder(){
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret[i] = _gv_Handler_Motor[i]->Get_Infos().encoder_deg;
        }
        return ret;
    }

    std::array<float, NO_OF_JOINT> Get_Torque(int option){
        // 1 : Estimated
        // 0 : Measured
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(option == 1){
                ret[i] = torque_esetimated(i);
            }else{
                ret[i] = _gv_Handler_Motor[i]->Get_Infos().torque_Nm;
            }
        }
        return ret;
    }

    std::array<float, NO_OF_JOINT> Get_Temperature(int option){
        // 1 : Estimated
        // 0 : Mesasured
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(option == 1){
                ret[i] = _gv_Handler_Motor[i]->Get_Infos().temperature_board;
            }else{
                ret[i] = _gv_Handler_Motor[i]->Get_Infos().temperature_board;
            }
        }
        return ret;
    }

    // ---------------------------------------------------------------
    // Other Systems
    // ---------------------------------------------------------------
    int Save_UserScript(unsigned int s_num, std::string s_txt){
        if(s_num >= 16)    return MSG_DESIRED_INDEX_IS_OVER_BOUND;

        if(!rb_config::WRITE_Script_Command(s_num, s_txt)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_UserScript(s_num);

        return MSG_OK;
    }
    int Recover_UserScript(int s_num){
        if(s_num >= 0 && s_num < 16){
            parameter_userscript[s_num] = rb_config::READ_Script_Command(s_num);
        }else{
            for(int i = 0; i < 16; ++i){
                parameter_userscript[i] = rb_config::READ_Script_Command(i);
            }
        }
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // USER FRAME
    // ---------------------------------------------------------------
    int Change_UserFrame_Number(int u_num){
        if(u_num < 0){
            u_num = 0;
        }else if(u_num >= NO_OF_USERF){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }

        userf_selection_num = u_num;
        return MSG_OK;
    }
    int Get_CurrentUserFrameNumber(){
        return userf_selection_num;
    }
    USERF_CONFIG Get_CurrentUserFrameParameter(){
        return parameter_userf[userf_selection_num];
    }
    USERF_CONFIG Get_DesiredUserFrameParameter(unsigned int u_num){
        if(u_num >= NO_OF_USERF)    u_num = 0;
        return parameter_userf[u_num];
    }
    int Save_UserFrameParameter(unsigned int u_num, USERF_CONFIG u_conf){
        if(u_num >= NO_OF_USERF)    return MSG_DESIRED_INDEX_IS_OVER_BOUND;

        if(!rb_config::WRITE_User_Frame(u_num, u_conf)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_UserFrameParameter(u_num);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_UserFrameParameter(int u_num){
        if(u_num >= 0 && u_num < NO_OF_USERF){
            parameter_userf[u_num] = rb_config::READ_User_Frame(u_num);
        }else{
            for(int i = 0; i < NO_OF_USERF; ++i){
                parameter_userf[i] = rb_config::READ_User_Frame(i);
            }
        }
        return MSG_OK;
    }
    int Set_UserFrame_6DOF(unsigned int u_num, unsigned int option, float x, float y, float z, float rx, float ry, float rz){
        // option
        // 0 : temporary
        // 1 : temporary + change 
        if(u_num >= NO_OF_USERF)    return MSG_DESIRED_INDEX_IS_OVER_BOUND;
        
        USERF_CONFIG current_config = parameter_userf[u_num];
        current_config.userf_offset = Eigen::Vector3d(x, y, z);
        current_config.userf_rotation = rb_math::RPY_to_R(rx, ry, rz);

        if(option == 1){
            if(!rb_config::WRITE_User_Frame(u_num, current_config)){
                return MSG_SAVE_TO_DB_FAIL;
            }
        }
        parameter_userf[u_num] = current_config;

        return MSG_OK;
    }
    int Set_UserFrame_TCP(unsigned int u_num, unsigned int option){
        return Set_UserFrame_6DOF(u_num, option
            , rb_motion::Get_Wrapper_X()[0], rb_motion::Get_Wrapper_X()[1], rb_motion::Get_Wrapper_X()[2]
            , rb_motion::Get_Wrapper_X()[3], rb_motion::Get_Wrapper_X()[4], rb_motion::Get_Wrapper_X()[5]);
    }
    int Set_UserFrame_3Points(unsigned int u_num, unsigned int option, unsigned int point_mode, float p1x, float p1y, float p1z, float p2x, float p2y, float p2z, float p3x, float p3y, float p3z){
        Eigen::Vector3d point_1 = Eigen::Vector3d(p1x, p1y, p1z);
        Eigen::Vector3d point_2 = Eigen::Vector3d(p2x, p2y, p2z);
        Eigen::Vector3d point_3 = Eigen::Vector3d(p3x, p3y, p3z);

        Eigen::Vector3d vec_p12 = point_2 - point_1;
        Eigen::Vector3d vec_p23 = point_3 - point_2;
        Eigen::Vector3d vec_p13 = point_3 - point_1;

        double del_p12 = vec_p12.norm();
        double del_p23 = vec_p23.norm();
        double del_p13 = vec_p13.norm();

        if(vec_p12.norm() < 0.7 || vec_p23.norm() < 0.7 || vec_p13.norm() < 0.7){
            return MSG_POINTS_TOO_CLOSE;
        }

        Eigen::Vector3d temp_vec1 = vec_p12/vec_p12.norm();
        Eigen::Vector3d temp_vec2 = vec_p13/vec_p13.norm();
        if(fabs(temp_vec1.dot(temp_vec2)) > 0.99){
            return MSG_POINTS_ARE_IN_LINE;
        }

        Eigen::Vector3d x_Axis = Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d y_Axis = Eigen::Vector3d(0, 1, 0);
        Eigen::Vector3d z_Axis = Eigen::Vector3d(0, 0, 1);

        if(point_mode == 0){
            x_Axis = point_2 - point_1;
            x_Axis = x_Axis/x_Axis.norm();

            Eigen::Vector3d temp_vec = point_3 - point_1;
            z_Axis = x_Axis.cross(temp_vec);
            z_Axis = z_Axis/z_Axis.norm();

            y_Axis = z_Axis.cross(x_Axis);
            y_Axis = y_Axis/y_Axis.norm();
        }else if(point_mode == 1){
            y_Axis = point_2 - point_1;
            y_Axis = y_Axis/y_Axis.norm();

            Eigen::Vector3d temp_vec = point_3 - point_1;
            z_Axis = temp_vec.cross(y_Axis);
            z_Axis = z_Axis/z_Axis.norm();

            x_Axis = y_Axis.cross(z_Axis);
            x_Axis = x_Axis/x_Axis.norm();
        }else if(point_mode == 2){
            z_Axis = point_2 - point_1;
            z_Axis = z_Axis/z_Axis.norm();

            Eigen::Vector3d temp_vec = point_3 - point_1;
            x_Axis = temp_vec.cross(z_Axis);
            x_Axis = x_Axis/x_Axis.norm();

            y_Axis = z_Axis.cross(x_Axis);
            y_Axis = y_Axis/y_Axis.norm();
        }

        Eigen::Matrix3d temp_R;
        temp_R.block(0, 0, 3, 1) = x_Axis;
        temp_R.block(0, 1, 3, 1) = y_Axis;
        temp_R.block(0, 2, 3, 1) = z_Axis;
        Eigen::Vector3d temp_E = rb_math::R_to_RPY(temp_R);
        return Set_UserFrame_6DOF(u_num, option
            , point_1(0), point_1(1), point_1(2)
            , temp_E(0), temp_E(1), temp_E(2));
    }

    // ---------------------------------------------------------------
    // AREA
    // ---------------------------------------------------------------
    AREA_CONFIG Get_DesiredAreaParameter(unsigned int a_num){
        if(a_num >= NO_OF_AREA)    a_num = 0;
        return parameter_area[a_num];
    }
    int Save_AreaParameter(unsigned int a_num, AREA_CONFIG a_conf){
        if(a_num >= NO_OF_AREA){
            return MSG_DESIRED_INDEX_IS_OVER_BOUND;
        }

        if(!rb_config::WRITE_Area_Parameter(a_num, a_conf)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_AreaParameter(a_num);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_AreaParameter(int a_num){
        if(a_num >= 0 && a_num < NO_OF_AREA){
            parameter_area[a_num] = rb_config::READ_Area_Parameter(a_num);
        }else{
            for(int i = 0; i < NO_OF_AREA; ++i){
                parameter_area[i] = rb_config::READ_Area_Parameter(i);
            }
        }
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // TCP 
    // ---------------------------------------------------------------
    int Change_Tool_Number(int t_num){
        if(t_num < 0){
            t_num = 0;
        }else if(t_num >= NO_OF_TOOL){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }

        if(tcp_selection_num != t_num){
            tcp_selection_num = t_num;
        }else{
            return MSG_OK;
        }

        request_ModelChange.store(1, std::memory_order_relaxed);
        while(1){
            auto state_ModelChange = request_ModelChange.load(std::memory_order_relaxed);
            if (state_ModelChange == 2) {
                break;
            }
            
            std::this_thread::sleep_for(1ms);
        }
        rb_motion::Model_Update();
        request_ModelChange.store(0, std::memory_order_relaxed);
        return MSG_OK;
    }
    int Get_CurrentTcpNumber(){
        return tcp_selection_num;
    }
    TCP_CONFIG Get_CurrentTcpParameter(){
        return parameter_tcp[tcp_selection_num];
    }
    TCP_CONFIG Get_DesiredTcpParameter(int t_num){
        return parameter_tcp[t_num];
    }
    int Save_TcpParameter(unsigned int t_num, TCP_CONFIG t_conf){
        if(t_num >= NO_OF_TOOL){
            return MSG_DESIRED_INDEX_IS_OVER_BOUND;
        }

        if(!rb_config::WRITE_Tcp_Parameter(t_num, t_conf)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_TcpParameter(t_num);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_TcpParameter(int t_num){
        if(t_num >= 0 && t_num < NO_OF_TOOL){
            parameter_tcp[t_num] = rb_config::READ_Tcp_Parameter(t_num);
        }else{
            for(int i = 0; i < NO_OF_TOOL; ++i){
                parameter_tcp[i] = rb_config::READ_Tcp_Parameter(i);
            }
        }
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // Collision 
    // ---------------------------------------------------------------
    int Call_Reset_Out_Coll(){//nonblock
        flag_collision_out_occur = false;
        return MSG_OK;
    }
    int Set_Out_Coll_Para(int onoff, int react, float th){
        parameter_out_coll_react = react;
        parameter_out_coll_limit = rb_math::saturation_L_and_U(th, 0, 1);
        parameter_out_coll_onoff = onoff;

        flag_collision_out_occur = false;
        return MSG_OK;
    }
    std::tuple<int, int, float> Get_Out_Coll_Para(){
        return {parameter_out_coll_onoff, parameter_out_coll_react, parameter_out_coll_limit};
    }
    int Save_Out_Coll_Para(int onoff, int react, float th){
        th = rb_math::saturation_L_and_U(th, 0.0, 1.0);

        if(!rb_config::WRITE_Out_Collision_Para(onoff, react, th)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_Out_Coll_Para();

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_Out_Coll_Para(){
        auto [t_out_coll_onoff, t_out_coll_react, t_out_coll_limit] = rb_config::READ_Out_Collision_Para();
        Set_Out_Coll_Para(t_out_coll_onoff, t_out_coll_react, t_out_coll_limit);
        return MSG_OK;
    }

    int Set_Self_Coll_Para(int mode, float dist_int, float dist_ext){
        parameter_self_coll_mode = mode;
        parameter_self_coll_dist_int = dist_int;
        parameter_self_coll_dist_ext = dist_ext;

        flag_collision_self_occur = false;
        return MSG_OK;
    }
    std::tuple<int, float, float> Get_Self_Coll_Para(){
        return {parameter_self_coll_mode, parameter_self_coll_dist_int, parameter_self_coll_dist_ext};
    }
    int Save_Self_Coll_Para(int mode, float dist_int, float dist_ext){
        dist_int = rb_math::saturation_Low(dist_int, 0);
        dist_ext = rb_math::saturation_Low(dist_ext, 0);

        if(!rb_config::WRITE_Self_Collision_Para(mode, dist_int, dist_ext)){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_Self_Coll_Para();

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
     int Recover_Self_Coll_Para(){
        auto [t_self_coll_mode, t_self_coll_dist_int, t_self_coll_dist_ext] = rb_config::READ_Self_Collision_Para();
        Set_Self_Coll_Para(t_self_coll_mode, t_self_coll_dist_int, t_self_coll_dist_ext);
        return MSG_OK;
     }
    
    std::tuple<int, float, float, float> Get_Gravity_Para(){
        return {parameter_gravity_mode, ((float)parameter_gravity_direction(0)), ((float)parameter_gravity_direction(1)), ((float)parameter_gravity_direction(2))};
    }
    int Save_Gravity_Para(int mode, float gx, float gy, float gz){
        if(mode < 0 || mode >= 1){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_VALUE_IS_OVER_BOUND);
        }

        Vector3d t_v = Vector3d(gx, gy, gz);
        if(t_v.norm() < 0.1){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_VALUE_IS_OVER_BOUND);
        }
        t_v /= t_v.norm();

        if(!rb_config::WRITE_Gravity_Vector(mode, t_v(0), t_v(1), t_v(2))){
            return MSG_SAVE_TO_DB_FAIL;
        }
        // ADJUST
        Recover_Gravity_Para();

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    Eigen::Vector3d Get_CurrentGravityParameter(){
        return parameter_gravity_direction;
    }
    int Recover_Gravity_Para(){
        auto [t_grav_mode, t_grav_x, t_grav_y, t_grav_z] = rb_config::READ_Gravity_Vector();
        parameter_gravity_mode = t_grav_mode;
        parameter_gravity_direction = Eigen::Vector3d(t_grav_x, t_grav_y, t_grav_z);
        return MSG_OK;
    }
    
    std::array<float, NO_OF_JOINT> Get_Direct_Teaching_Sensitivity(){
        std::array<float, NO_OF_JOINT> ret;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            ret[i] = parameter_direct_teach_sensitivity(i);
        }
        return ret;
    }
    int Save_Direct_Teaching_Sensitivity(std::array<float, NO_OF_JOINT> f_targets){
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(!rb_config::WRITE_Direct_Teach_Sensitivity(i, f_targets[i])){
                return MSG_SAVE_TO_DB_FAIL;
            }
        }
        // ADJUST
        Recover_Direct_Teaching_Sensitivity();

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_Direct_Teaching_Sensitivity(){
        for(int i = 0; i < NO_OF_JOINT; ++ i){
            parameter_direct_teach_sensitivity(i) = rb_config::READ_Direct_Teach_Sensitivity(i);
        }
        return MSG_OK;
    }

    int Set_Joint_Impedance_On(std::array<float, NO_OF_JOINT> rate_gain, std::array<float, NO_OF_JOINT> rate_torque){
        int target_P[NO_OF_JOINT];
        int target_I[NO_OF_JOINT];
        int target_D[NO_OF_JOINT];
        int target_A[NO_OF_JOINT];

        flag_joint_impedance_mode = true;

        for(int i = 0; i < NO_OF_JOINT; ++i){
            rate_gain[i] = rb_math::saturation_L_and_U(rate_gain[i], 0.0, 1.0);
            rate_torque[i] = rb_math::saturation_L_and_U(rate_torque[i], 0.0, 1.0);

            double gain_ori = rate_gain[i];
            double gain_double = gain_ori * gain_ori;

            mINFO temp_minfos = _gv_Handler_Motor[i]->Get_Infos();
            mPARA temp_mpara = _gv_Handler_Motor[i]->Get_Parameters();
            target_P[i] = ((float)temp_minfos.gain_position_P) * gain_double;
            target_I[i] = ((float)temp_minfos.gain_position_I) * gain_double;
            target_D[i] = ((float)temp_minfos.gain_position_D) * (0.1 + 0.9 * gain_ori);
            target_P[i] = rb_math::saturation_Low(target_P[i], 1);

            target_A[i] = temp_mpara.para_limit_current_mA * 0.001 * rate_torque[i];
            target_A[i] = rb_math::saturation_Low(target_A[i], 1);

            std::cout<<"Impedance Para: "<<i<<" : "<<target_P[i]<<", "<<target_I[i]<<", "<<target_D[i]<<", "<<target_A[i]<<std::endl;
        }

        // Step1: Blind Error
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_BlindError(true, true));
            std::this_thread::sleep_for(2ms);
        }
        // Step2: Set Temporary PID and torque limit
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Temporary_Gain_Position(1, target_P[i], target_I[i], target_D[i]));
            torque_limit_A[i] = target_A[i];
            std::this_thread::sleep_for(2ms);
        }
        return MSG_OK;
    }
    int Set_Joint_Impedance_Off(){
        if(flag_joint_impedance_mode == false){
            return MSG_OK;
        }
        for(int i = 0; i < NO_OF_JOINT; ++i){
            //----------------------------
            mINFO temp_motor_info = _gv_Handler_Motor[i]->Get_Infos();
            double enc_deg   = temp_motor_info.encoder_deg;
            
            rb_motion::Set_Motion_q(i, enc_deg);//impedance off
            rb_motion::Set_Wrapper_q(i, enc_deg);

            output_lpf_q_input(i) = enc_deg;
            output_lpf_q_ang(i) = enc_deg;
            output_lpf_q_ang_old(i) = enc_deg;

            //----------------------------

            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_MakeErrorSumZero(0));
            std::this_thread::sleep_for(2ms);
        }
        
        // Recover PID Gain and TQ bound
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Temporary_Gain_Position(0, 0, 0, 0));
            torque_limit_A[i] = -1;
            std::this_thread::sleep_for(2ms);
        }

        // Reocver Blind Mode
        for(int i = 0; i < NO_OF_JOINT; ++i){
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_BlindError(false, false));
            std::this_thread::sleep_for(2ms);
        }
        flag_joint_impedance_mode = false;
        return MSG_OK;
    }
    int Set_Free_Drive_Mode(int onoff, float sensitivity){
        if(onoff == 1){
            flag_sw_switch_free_drive = true;
        }else{
            flag_sw_switch_free_drive = false;
        }
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // MoveFlow
    // ---------------------------------------------------------------
    int Call_Halt(){//nonblock
        LOG_INFO("CALL HALT");
        rb_motion::Set_Motion_Mode(rb_motion::MotionMode::MOVE_NONE);
        flag_collision_out_occur = false;
        flag_collision_self_occur = false;
        flag_is_break = false;
        flag_is_pause = false;
        flag_direct_teaching = false;
        // flag_joint_impedance_mode = false;
        flag_sw_switch_free_drive = false;
        return MSG_OK;
    }
    int Set_MoveSpeedBar(double alpha){
        _sys_timescale[(int)SystemTimeScaler::SPEEDBAR].input = rb_math::saturation_L_and_U(alpha, 0.0, 1.0);
        return MSG_OK;
    }
    double Get_MoveSpeedBar(){
        return _sys_timescale[(int)SystemTimeScaler::SPEEDBAR].output;
    }

    void Set_MoveUserSpeedBar(double alpha){
        _sys_timescale[(int)SystemTimeScaler::USER].input = rb_math::saturation_L_and_U(alpha, 0.0, 1.0);
    }
    double Get_MoveUserSpeedBar(){
        return _sys_timescale[(int)SystemTimeScaler::USER].output;
    }

    int Call_MovePause(){//nonblock
        if(!flag_is_pause){
            LOG_INFO("PAUSE");
        }
        flag_is_pause = true;
        return MSG_OK;
    }
    int Call_MoveResume(){//nonblock
        if(flag_is_pause){
            LOG_INFO("RESUME from PAUSE");
        }
        flag_is_pause = false;
        return MSG_OK;
    }
    int Get_MovePauseState(){
        return flag_is_pause;
    }

    int Call_MoveBreak(double t_time){
        if(rb_motion::Get_Is_Motion_Idle()){
            return MSG_OK;
        }
        if(!flag_is_break){
            LOG_INFO("Break Called");
        }
        _sys_timescale[(int)SystemTimeScaler::BREAK].lpf_alpha = RT_PERIOD_SEC / rb_math::saturation_Low(t_time, 0.01);
        flag_is_break = true;
        return MSG_OK;
    }
    void Reset_MoveBreak(){
        _sys_timescale[(int)SystemTimeScaler::BREAK].input = 1.;
        _sys_timescale[(int)SystemTimeScaler::BREAK].output = 1.;
        _sys_timescale[(int)SystemTimeScaler::BREAK].lpf_alpha = 0.;
        flag_is_break;
    }

    int Call_Program_Before(int option){
        (void)option;

        std::cout<<"---------------------------"<<std::endl;
        std::cout<<"- Program Start"<<std::endl;
        std::cout<<"---------------------------"<<std::endl;
        return MSG_OK;
    }
    int Call_Program_After(int option){
        (void)option;

        rb_motion::Stop_Wrapper_All();
        rb_system::Recover_UserFrameParameter(-1);
        rb_system::Recover_AreaParameter(-1);
        rb_motion::Clear_Motion_Shift(-1);
        rb_system::Recover_Out_Coll_Para();
        rb_system::Recover_Self_Coll_Para();
        rb_system::Recover_TcpParameter(-1);

        std::cout<<"---------------------------"<<std::endl;
        std::cout<<"- Program End"<<std::endl;
        std::cout<<"---------------------------"<<std::endl;
        return MSG_OK;
    }

    // ---------------------------------------------------------------
    // IO
    // ---------------------------------------------------------------

    std::array<uint8_t, NO_OF_DOUT> Get_Box_Dout(){
        std::array<uint8_t, NO_OF_DOUT> ret;
        for(int i = 0; i < NO_OF_DOUT; ++i){
            ret[i] = _gv_Handler_Side->Get_State().dout_raw[i];
        }
        return ret;
    }
    std::array<uint8_t, NO_OF_DIN> Get_Box_Din(){
        std::array<uint8_t, NO_OF_DIN> ret;
        for(int i = 0; i < NO_OF_DIN; ++i){
            ret[i] = _gv_Handler_Side->Get_State().din_filt[i];
        }
        return ret;
    }
    std::array<float, NO_OF_AOUT> Get_Box_Aout(){
        std::array<float, NO_OF_AOUT> ret;
        for(int i = 0; i < NO_OF_AOUT; ++i){
            ret[i] = _gv_Handler_Side->Get_State().dac_raw[i];
        }
        return ret;
    }
    std::array<float, NO_OF_AIN> Get_Box_Ain(){
        std::array<float, NO_OF_AIN> ret;
        for(int i = 0; i < NO_OF_AIN; ++i){
            ret[i] = _gv_Handler_Side->Get_State().adc_raw[i];
        }
        return ret;
    }

    std::array<uint8_t, NO_OF_DOUT> Get_EX_Dout(){
        std::array<uint8_t, NO_OF_DOUT> ret;
        for(int i = 0; i < NO_OF_DOUT; ++i){
            ret[i] = 0;
        }
        return ret;
    }
    std::array<uint8_t, NO_OF_DIN> Get_EX_Din(){
        std::array<uint8_t, NO_OF_DIN> ret;
        for(int i = 0; i < NO_OF_DIN; ++i){
            ret[i] = 0;
        }
        return ret;
    }
    std::array<float, NO_OF_AOUT> Get_EX_Aout(){
        std::array<float, NO_OF_AOUT> ret;
        for(int i = 0; i < NO_OF_AOUT; ++i){
            ret[i] = 0;
        }
        return ret;
    }
    std::array<float, NO_OF_AIN> Get_EX_Ain(){
        std::array<float, NO_OF_AIN> ret;
        for(int i = 0; i < NO_OF_AIN; ++i){
            ret[i] = 0;
        }
        return ret;
    }

    std::array<uint8_t, NO_OF_DOUT> Get_Tool_Dout(){
        std::array<uint8_t, NO_OF_DOUT> ret;
        for(int i = 0; i < NO_OF_DOUT; ++i){
            ret[i] = 0;
        }
        for(int i = 0; i  < TFB_NUM_DOUT; ++i){
            ret[i] = _gv_Handler_Toolflange->Get_State().dout[i];
        }
        return ret;
    }
    std::array<uint8_t, NO_OF_DIN> Get_Tool_Din(){
        std::array<uint8_t, NO_OF_DIN> ret;
        for(int i = 0; i < NO_OF_DIN; ++i){
            ret[i] = 0;
        }
        for(int i = 0; i  < TFB_NUM_DIN; ++i){
            ret[i] = _gv_Handler_Toolflange->Get_State().din[i];
        }
        return ret;
    }
    std::array<float, NO_OF_AOUT> Get_Tool_Aout(){
        std::array<float, NO_OF_AOUT> ret;
        for(int i = 0; i < NO_OF_AOUT; ++i){
            ret[i] = 0;
        }
        return ret;
    }
    std::array<float, NO_OF_AIN> Get_Tool_Ain(){
        std::array<float, NO_OF_AIN> ret;
        for(int i = 0; i < NO_OF_AIN; ++i){
            ret[i] = 0;
        }
        return ret;
    }
    int Get_Tool_Voltage(){
        return _gv_Handler_Toolflange->Get_State().voltage;
    }

    std::array<int8_t, NO_OF_DOUT> Get_Box_Special_Dout(){
        std::array<int8_t, NO_OF_DOUT> ret;
        for(int i = 0; i < NO_OF_DOUT; ++i){
            ret[i] = parameter_special_dout_box[i];
            //std::cout<<"dout ret[i]"<<(int)ret[i]<<std::endl;
        }
        return ret;
    }
    int Save_Box_Special_Dout(unsigned int p_num, int func_no){
        if(p_num >= NO_OF_DOUT){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }
        if(func_no >= DOUT_DEF_NUM){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }
        if(!rb_config::WRITE_IO_Special_BOX(0, p_num, func_no)){
            return MSG_SAVE_TO_DB_FAIL;
        }

        // ADJUST
        Recover_Box_Special_Dout(p_num);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_Box_Special_Dout(int p_num){
        if(p_num >= 0 && p_num < NO_OF_DOUT){
            parameter_special_dout_box[p_num] = rb_config::READ_IO_Special_BOX(0, p_num);
        }else{
            for(int i = 0; i < NO_OF_DOUT; ++i){
                parameter_special_dout_box[i] = rb_config::READ_IO_Special_BOX(0, i);
            }
        }
        return MSG_OK;
    }

    std::array<int8_t, NO_OF_DIN> Get_Box_Special_Din(){
        std::array<int8_t, NO_OF_DIN> ret;
        for(int i = 0; i < NO_OF_DIN; ++i){
            ret[i] = parameter_special_din_box[i];
            //std::cout<<"din ret[i]"<<(int)ret[i]<<std::endl;
        }
        return ret;
    }
    int Save_Box_Special_Din(unsigned int p_num, int func_no){
        if(p_num >= NO_OF_DIN){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }
        if(func_no >= DIN_DEF_NUM){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }
        if(!rb_config::WRITE_IO_Special_BOX(1, p_num, func_no)){
            return MSG_SAVE_TO_DB_FAIL;
        }

        // ADJUST
        Recover_Box_Special_Din(p_num);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_Box_Special_Din(int p_num){
        if(p_num >= 0 && p_num < NO_OF_DIN){
            parameter_special_din_box[p_num] = rb_config::READ_IO_Special_BOX(1, p_num);
        }else{
            for(int i = 0; i < NO_OF_DIN; ++i){
                parameter_special_din_box[i] = rb_config::READ_IO_Special_BOX(1, i);
            }
        }
        return MSG_OK;
    }

    std::array<uint8_t, NO_OF_DIN> Get_Box_FilterCount_Din(){
        std::array<uint8_t, NO_OF_DIN> ret;
        for(int i = 0; i < NO_OF_DIN; ++i){
            ret[i] = parameter_filter_count_din_box[i];
        }
        return ret;
    }
    int Save_Box_FilterCount_Din(unsigned int p_num, unsigned int f_cnt){
        if(p_num >= NO_OF_DIN){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }
        if(f_cnt > 200){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }
        if(!rb_config::WRITE_DIN_Filter_Count(p_num, f_cnt)){
            return MSG_SAVE_TO_DB_FAIL;
        }

        // ADJUST
        Recover_Box_FilterCount_Din(p_num);
        _gv_Handler_Side->Set_Din_Filter_Count(p_num, parameter_filter_count_din_box[p_num]);

        return MESSAGE(MSG_LEVEL_INFO, MSG_OK);
    }
    int Recover_Box_FilterCount_Din(int p_num){
        if(p_num >= 0 && p_num < NO_OF_DIN){
            parameter_filter_count_din_box[p_num] = rb_config::READ_DIN_Filter_Count(p_num);
        }else{
            for(int i = 0; i < NO_OF_DIN; ++i){
                parameter_filter_count_din_box[i] = rb_config::READ_DIN_Filter_Count(i);
            }
        }
        return MSG_OK;
    }
    
    int Set_Box_Digital_Output(int p_num, unsigned int value){
        if(p_num >= NO_OF_DOUT){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }

        //std::cout<<"Set_Box_Digital_Output: "<<p_num<<", "<<value<<std::endl;

        if(p_num < 0){
            for(unsigned int i = 0; i < NO_OF_DOUT; ++i){
                _gv_Handler_Side->Set_Dout(i, value);
            }
        }else{
            _gv_Handler_Side->Set_Dout(p_num, value);
        }
        return MSG_OK;
    }
    int Set_Box_Digital_Output_Toggle(int p_num){
        if(p_num >= NO_OF_DOUT){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }
        if(p_num < 0){
            for(unsigned int i = 0; i < NO_OF_DOUT; ++i){
                int value = !_gv_Handler_Side->Get_Dout(i);
                _gv_Handler_Side->Set_Dout(i, value);
            }
        }else{
            int value = !_gv_Handler_Side->Get_Dout(p_num);
            std::cout<<"value: "<<value<<std::endl;
            _gv_Handler_Side->Set_Dout(p_num, value);
        }
        return MSG_OK;
    }
    int Set_Box_Digital_Output_Bit(unsigned int p_num_start, unsigned int p_num_end, int value, unsigned char option_dir){
        // option_dir
        // 0 : normal direction
        // 1 : inverse direction

        bool is_any_problem = false;
        if(p_num_start >= NO_OF_DOUT || p_num_end >= NO_OF_DOUT || p_num_end < p_num_start){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_PORT_IS_OVER_BOUND);
        }
        unsigned int bit_length = p_num_end - p_num_start + 1;
        int max_value = (int)(round(pow(2, bit_length)));
        if(value < 0 || value >= max_value){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_VALUE_IS_OVER_BOUND);
        }
        unsigned char is_every_port_is_normal = true;
        for(int k = p_num_start; k <= p_num_end; ++k){
            if(parameter_special_dout_box[k] != 0){
                return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_PORT_IS_NOT_AVAIL);
            }
        }


        for(int k = p_num_start; k <= p_num_end; ++k){
            int off = k - p_num_start;
            int index = k;
            if(option_dir == 1){
                // inverse direction
                index = (p_num_start + p_num_end) - k;
            }
            _gv_Handler_Side->Set_Dout(index, ((value >> off) & 0b01));
        }
        return MSG_OK;
    }
    int Set_Box_Digital_Output_Pulse(unsigned int p_num, unsigned int mode, unsigned int direction, float t1, float t2, float t3){
        // mode : 
        // 0 : block
        // 1 : nonblock
        // direction :
        // 0 : 010
        // 1 : 101

        if(mode != 1)       mode = 0;
        if(direction != 1)  direction = 0;

        if(p_num >= NO_OF_DOUT){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_PORT_IS_OVER_BOUND);
        }
        if(parameter_special_dout_box[p_num] != 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_PORT_IS_NOT_AVAIL);
        }
        if(t1 < 0. || t2 < 0. || t3 < 0. || t1 > 3. || t2 > 3. || t3 > 3.){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_VALUE_IS_OVER_BOUND);
        }
        t3 = rb_math::saturation_Low(t3, 0.003);

        _gv_Handler_Side->Set_Dout_Pulse(p_num, direction, t1, (t1 + t2), (t1 + t2 + t3));
        if(mode == 0){
            float timeout_sec = 0.;
            while(1){
                if(_gv_Handler_Side->Get_Dout_Pulse_State(p_num) == false){
                    break;
                }
                if(timeout_sec >= 10.){
                    return MESSAGE(MSG_LEVEL_WARN, MSG_COMMAND_TIME_OUT);
                }
                timeout_sec += 0.01;
                std::this_thread::sleep_for(10ms);
            }
        }
        return MSG_OK;
    }
    int Set_Box_Analog_Output(int p_num, float value){
        if(p_num >= NO_OF_AOUT){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }

        if(p_num < 0){
            for(unsigned int i = 0; i < NO_OF_AOUT; ++i){
                _gv_Handler_Side->Set_Aout(i, value);
            }
        }else{
            _gv_Handler_Side->Set_Aout(p_num, value);
        }
        return MSG_OK;
    }

    int Set_Flange_Power(int desired_voltage){
        std::cout<<"desired_voltage: "<<desired_voltage<<std::endl;
        if(desired_voltage > 0 && desired_voltage != 12 && desired_voltage != 24){
            return MSG_DESIRED_VALUE_IS_OVER_BOUND;
        }

        if(!flag_connection_components[NO_OF_JOINT]){
            return MSG_SYSTEM_THIS_FUNC_IS_AVAIL_WHEN_POWER_ENGAGED;
        }

        _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdPowerControl(desired_voltage));
        return MSG_OK;
    }
    int Set_Flange_Digital_Output(int p_num, int value){
        std::cout<<"tool out"<<p_num<<" / "<<value<<std::endl;
        if(p_num >= TFB_NUM_DOUT){
            return MSG_DESIRED_PORT_IS_OVER_BOUND;
        }

        if(!flag_connection_components[NO_OF_JOINT]){
            return MSG_SYSTEM_THIS_FUNC_IS_AVAIL_WHEN_POWER_ENGAGED;
        }

        if(value > 0 && value != 1) value = 0;

        if(p_num < 0){
            for(unsigned int i = 0; i < TFB_NUM_DOUT; ++i){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdDigitalOutput_Single(i, value));
            }
        }else{
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdDigitalOutput_Single(p_num, value));
        }
        return MSG_OK;
    }

    // ---------------------------------------------------------------
    // Power & ServoOn
    // ---------------------------------------------------------------
    int Set_Joint_Brake(int bno, int brk_action){
        // action
        // 0 : Close
        // 1 : Open (Hard)
        // 2 : Weak
        // 3 : Close only

        LOG_INFO("Joint Brake: " + std::to_string(bno) + "/" +std::to_string(brk_action));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }
        if(brk_action == 1 || brk_action == 2){
            if(!Get_Power()){
                if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
            }
        }

        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Brake(brk_action, false));
        }

        return MSG_OK;
    }

    int Set_Joint_Encoder_Zero(int bno){
        LOG_INFO("Joint Encoder: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }
        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }

        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_HomeOffsetZero());
        }

        return MSG_OK;
    }

    int Set_Joint_Sensor_Reset(int bno){
        LOG_INFO("Joint Sensor Reset: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }
        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }

        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);

            std::cout<<"Stage 1: Open and DQ Align"<<std::endl;
            for(int tr = 0; tr < 3; ++tr){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Brake(1, false));
                std::this_thread::sleep_for(0.2s);
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_DQ_Align());
                std::this_thread::sleep_for(1.5s);
            }
            std::cout<<"Stage 2: DQ Save"<<std::endl;
            for(int tr = 0; tr < 2; ++tr){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_DQ_Save());
                std::this_thread::sleep_for(1.5s);
            }
            std::cout<<"Stage 3: Sensor Nulling"<<std::endl;
            for(int tr = 0; tr < 2; ++tr){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Current_Nulling());
                std::this_thread::sleep_for(1.5s);
            }


            std::cout<<"Sensor Reset joint "<<target_bno<<" done"<<std::endl;
            // _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_HomeOffsetZero());
        }

        return MSG_OK;
    }

    int Pop_Joint_GainPos(int bno){
        LOG_INFO("Joint Pgain Pop: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }
        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }
        
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);

            std::this_thread::sleep_for(3ms);
            _gv_Handler_Motor[target_bno]->Clear_Gain_Position();
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Ask_Gain_Position());
        }

        std::this_thread::sleep_for(20ms);

        std::string total_str = "";
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            mINFO tMotor = _gv_Handler_Motor[target_bno]->Get_Infos();
            if(tMotor.gain_position_flag == true){
                total_str += ("Motor " + std::to_string(target_bno) + " PosGain = " + std::to_string(tMotor.gain_position_P) + "/" + std::to_string(tMotor.gain_position_I) + "/" + std::to_string(tMotor.gain_position_D));
            }else{
                total_str += ("Motor " + std::to_string(target_bno) + " PosGain not received");
            }
            if(s != ((int)shot_arr.size() - 1)){
                total_str += "\n";
            }
        }
        POPUP(MSG_LEVEL_INFO, MSG_OK, total_str);
        return MSG_OK;
    }
    int Pop_Joint_GainCur(int bno){
        LOG_INFO("Joint Cgain Pop: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }
        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }

        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);

            std::this_thread::sleep_for(3ms);
            _gv_Handler_Motor[target_bno]->Clear_Gain_Current();
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Ask_Gain_Current());
        }

        std::this_thread::sleep_for(20ms);

        std::string total_str = "";
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            mINFO tMotor = _gv_Handler_Motor[target_bno]->Get_Infos();
            if(tMotor.gain_current_flag == true){
                total_str += ("Motor " + std::to_string(target_bno) + " CurGain = " + std::to_string(tMotor.gain_current_P) + "/" + std::to_string(tMotor.gain_current_I));
            }else{
                total_str += ("Motor " + std::to_string(target_bno) + " CurGain not received");
            }
            if(s != ((int)shot_arr.size() - 1)){
                total_str += "\n";
            }
        }
        POPUP(MSG_LEVEL_INFO, MSG_OK, total_str);
        return MSG_OK;
    }
    int Set_Joint_GainPos(int bno, int pgain, int igain, int dgain){
        LOG_INFO("Joint Pgain Set: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }

        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Save_Gain_Posision(pgain, igain, dgain));
            std::this_thread::sleep_for(20ms);
        }
        return MESSAGE(MSG_LEVEL_INFO, MSG_SAVE_DONE);
    }
    int Set_Joint_GainCur(int bno, int pgain, int igain){
        LOG_INFO("Joint Cgain Set: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }

        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[target_bno]->Cmd_Save_Gain_Current(pgain, igain));
            std::this_thread::sleep_for(20ms);
        }
        return MESSAGE(MSG_LEVEL_INFO, MSG_SAVE_DONE);
    }

    int Pop_Joint_Infos(int bno){
        LOG_INFO("Joint Infos: " + std::to_string(bno));
        std::vector<int> shot_arr = Joint_Action_List_Generator(bno);
        if(shot_arr.size() == 0){
            return MESSAGE(MSG_LEVEL_WARN, MSG_DESIRED_INDEX_IS_OVER_BOUND);
        }

        if(!Get_Power()){
            if (int pw_ret = Set_Power(PowerOption::On, false); pw_ret != MSG_OK) return pw_ret;
        }
        
        std::string total_str = "";
        for(int s = 0; s < shot_arr.size(); ++s){
            int target_bno = shot_arr.at(s);
            auto m_info = _gv_Handler_Motor[target_bno]->Get_Infos();
            auto m_stat = _gv_Handler_Motor[target_bno]->Get_States();
            auto m_para = _gv_Handler_Motor[target_bno]->Get_Parameters();

            std::string single_str = "J" + std::to_string(target_bno) + ": ";
            if(flag_connection_components[target_bno]){
                single_str += ("V(" + std::to_string(m_info.firmware_version) +") ");
                single_str += ("T(" + std::to_string(m_info.type_num) +") ");
                single_str += ("D(" + std::to_string(m_info.type_mdr) +") ");
                single_str += ("E(" + std::to_string((int)m_info.encoder_deg) +") ");
                single_str += ("M(" + std::to_string(m_info.stat_mul) +") ");
            }else{
                single_str += "Not Connected";
            }
            total_str += ("\n" + single_str);
        }

        POPUP(MSG_LEVEL_INFO, MSG_OK, total_str);
        return MSG_OK;
    }

    int Set_Servo(int option, int who_issue){
        // option
        // 0 : Basic Servo-on
        // 1 : Basic + ReferenceOn

        //----------------------------------------------------------------------
        LOG_INFO("Servo Call: " + std::to_string(option) + "/" +std::to_string(who_issue));
        if(servo_stat == ServoState::DONE){
            return MSG_OK;
        }
        //----------------------------------------------------------------------
        Set_ReferenceOnOff(false);
        Set_Servo_State(ServoState::NONE);
        //----------------------------------------------------------------------
        // check power
        if(!Get_Power()){
            int pw_ret = Set_Power(PowerOption::On, false);
            if(pw_ret != MSG_OK){
                return pw_ret;
            }
        }
        Set_Servo_State(ServoState::PWCHECK);
        //----------------------------------------------------------------------
        if(!flag_connection_is_all){
            if(!flag_connection_components[NO_OF_JOINT + 0]){
                return MESSAGE(MSG_LEVEL_ERRR, MSG_TFB_NOT_DETECTED);
            }
            if(!flag_connection_components[NO_OF_JOINT + 1]){
                return MESSAGE(MSG_LEVEL_ERRR, MSG_SIDE_NOT_DETECTED);
            }
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(!flag_connection_components[i]){
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_NOT_DETECTED_0 + i);
                }
            }
        }

        {// Do Version Check
            for(int i = 0; i < NO_OF_JOINT; ++i){
                std::this_thread::sleep_for(3ms);
                _gv_Handler_Motor[i]->Set_Version(-1);
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_RequestVersion());
            }
            _gv_Handler_Side->Set_Version(-1);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Side->CmdRequestVersion());

            _gv_Handler_Toolflange->Set_Version(-1);
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdRequestVersion());
            std::this_thread::sleep_for(15ms);

            for(int i = 0; i < NO_OF_JOINT; ++i){
                std::this_thread::sleep_for(3ms);
                _gv_Handler_Motor[i]->Clear_Gain_Position();
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Ask_Gain_Position());
            }
            std::this_thread::sleep_for(15ms);
            for(int i = 0; i < NO_OF_JOINT; ++i){
                std::this_thread::sleep_for(3ms);
                _gv_Handler_Motor[i]->Clear_Gain_Current();
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Ask_Gain_Current());
            }
            std::this_thread::sleep_for(15ms);
            for(int i = 0; i < NO_OF_JOINT; ++i){
                std::this_thread::sleep_for(3ms);
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_AdminMode(true));
            }

            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i]){
                    int m_version = _gv_Handler_Motor[i]->Get_Infos().firmware_version;
                    if(m_version < 0){
                        return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_FIRMWARE_IS_NOT_DETECTED_0 + i);
                    }else{
                        LOG_INFO("Motor Firmware: " + std::to_string(i) + " : " + std::to_string(m_version));
                        if(m_version < 1240219){
                            return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_FIRMWARE_VER_IS_LOW_0 + i);
                        }
                    }
                }
            }

            if(flag_dev_configuration[NO_OF_JOINT + 0]){
                int t_version = _gv_Handler_Toolflange->Get_Info().firmware_version;
                if(t_version < 0){
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_TFB_ERR_FIRMWARE_IS_NOT_DETECTED);
                }else{
                    LOG_INFO("ToolFlange Firmware : " + std::to_string(t_version));
                }
            }
            if(flag_dev_configuration[NO_OF_JOINT + 1]){
                int s_version = _gv_Handler_Side->Get_Info().firmware_version;
                if(s_version < 0){
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_SIDE_ERR_FIRMWARE_IS_NOT_DETECTED);
                }else{
                    LOG_INFO("Side Firmware : " + std::to_string(s_version));
                }
            }
        }

        {// Do Type and Bound Check
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i] == 0)  continue;

                mINFO tMotor = _gv_Handler_Motor[i]->Get_Infos();
                if((int)parameter_robot.modules_type[i] != tMotor.type_num){
                    LOG_ERROR("Motor Type Not Math: " + std::to_string(i) + "[" + std::to_string(tMotor.type_num) + "/" + std::to_string((int)parameter_robot.modules_type[i]) + "]");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_TYPE_NOT_MATCH_0 + i);
                }
                if(tMotor.stat_mul == 1){
                    LOG_ERROR("Motor Mul Error: " + std::to_string(i) + "[" + std::to_string(tMotor.encoder_deg)  + "]");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_MUL_0 + i);
                }
                if(tMotor.encoder_deg > (parameter_robot.modules_range_up[i] + 10)){
                    LOG_ERROR("Motor Bound Up Over: " + std::to_string(i) + "[" + std::to_string(tMotor.encoder_deg) + "/" + std::to_string(parameter_robot.modules_range_up[i]) + "]");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_ANG_LIMIT_UP_0 + i);
                }
                if(tMotor.encoder_deg < (parameter_robot.modules_range_low[i] - 10)){
                    LOG_ERROR("Motor Bound Dw Over: " + std::to_string(i) + "[" + std::to_string(tMotor.encoder_deg) + "/" + std::to_string(parameter_robot.modules_range_low[i]) + "]");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_ANG_LIMIT_DW_0 + i);
                }
                if(tMotor.gain_position_flag == true){
                    LOG_INFO("Motor PGain: " + std::to_string(i) + " : " + std::to_string(tMotor.gain_position_P) + "/" + std::to_string(tMotor.gain_position_I) + "/" + std::to_string(tMotor.gain_position_D));
                }else{
                    LOG_ERROR("Motor PGain: " + std::to_string(i) + " : Not Received");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_CORE_PARAMETER_NOT_RECEIVED_0 + i);
                }
                if(tMotor.gain_current_flag == true){
                    LOG_INFO("Motor CGain: " + std::to_string(i) + " : " + std::to_string(tMotor.gain_current_P) + "/" + std::to_string(tMotor.gain_current_I));
                }else{
                    LOG_ERROR("Motor CGain: " + std::to_string(i) + " : Not Received");
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_CORE_PARAMETER_NOT_RECEIVED_0 + i);
                }
            }
        }

        Set_Servo_State(ServoState::DEVCHECK);
        //----------------------------------------------------------------------
        // do or check somthing
        {
            ;
        }
        Set_Servo_State(ServoState::PARACHECK);
        //----------------------------------------------------------------------
        // turn on motor driver
        VectorJd t_enc_deg;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            t_enc_deg(i) = _gv_Handler_Motor[i]->Get_Infos().encoder_deg;
        }
        RBDLrobot temp_robot;
        temp_robot.Init_Robot(Get_CurrentRobotParameter(), Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
        VectorJd t_torque_ID = temp_robot.Calc_InverseDynamics(t_enc_deg, VectorJd::Zero(NO_OF_JOINT, 1), VectorJd::Zero(NO_OF_JOINT, 1));
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(flag_dev_configuration[i] == 0)  continue;
            _gv_Handler_Motor[i]->Activation_Process_Start();
            _gv_Handler_Motor[i]->Clear_States();
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_ServoOn(t_torque_ID(i)));            ;
            std::this_thread::sleep_for(80ms);
        }
        bool is_success_to_servoOn = false;
        float init_check_time = 0.;
        while(1){
            bool is_all_motor_on = true;
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i] == 0)  continue;

                mSTAT t_info = _gv_Handler_Motor[i]->Get_States();
                if(!t_info.b.INIT || !t_info.b.RUN){
                    is_all_motor_on = false;
                }
            }
            if(is_all_motor_on){
                is_success_to_servoOn = true;
                break;
            }
            std::this_thread::sleep_for(50ms);
            init_check_time += 0.05;
            if(init_check_time > 5.0){
                std::cout<<"Servo Flag Check time out"<<std::endl;
                for(int i = 0; i < NO_OF_JOINT; ++i){
                    mSTAT t_info = _gv_Handler_Motor[i]->Get_States();
                    std::cout<<"Joint : "<<i<<"= "<<(int)t_info.b.INIT<<" / "<<(int)t_info.b.RUN<<std::endl;
                }
                break;
            }
        }

        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(flag_dev_configuration[i] == 0)  continue;

            double desired_shake_ang = _gv_Handler_Motor[i]->Get_Parameters().para_shake_pulse * _gv_Handler_Motor[i]->Get_Parameters().para_pulse_to_deg;
            auto [act_min, act_max] = _gv_Handler_Motor[i]->Activation_Process_Stop();
            LOG_INFO("Motor Acts: " + std::to_string(i) + "[" + std::to_string(act_min) + "," + std::to_string(act_max) + "/" + std::to_string(desired_shake_ang) + "]");
            
            bool is_there_minus_err = false;
            bool is_there_plus_err = false;
            double shake_check_limit = 0.3;
            if(fabs(act_min) < (desired_shake_ang * shake_check_limit)){
                is_there_minus_err = true;
            }
            if(fabs(act_max) < (desired_shake_ang * shake_check_limit)){
                is_there_plus_err = true;
            }

            // if(is_there_minus_err && is_there_plus_err){
            //     LOG_ERROR("Motor Shake Both Fail: " + std::to_string(i));
            //     Set_Power(PowerOption::Off, false);
            //     return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_NO_MOVE_DURING_SERVOON_0 + i);
            // }else if(is_there_minus_err){
            //     LOG_ERROR("Motor Shake Minus Fail: " + std::to_string(i));
            //     Set_Power(PowerOption::Off, false);
            //     return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_NO_MOVE_DURING_SERVOON_MINUS_0 + i);
            // }else if(is_there_plus_err){
            //     LOG_ERROR("Motor Shake Plus Fail: " + std::to_string(i));
            //     Set_Power(PowerOption::Off, false);
            //     return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_NO_MOVE_DURING_SERVOON_PLUS_0 + i);
            // }
        }

        if(!is_success_to_servoOn){
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i] == 0)  continue;

                mSTAT t_info = _gv_Handler_Motor[i]->Get_States();
                if(!t_info.b.INIT || !t_info.b.RUN){
                    rb_common::log_push(LogLevel::Error, "Motor Servon Fail: " + std::to_string(i), P_NAME);
                    Set_Power(PowerOption::Off, false);
                    return MESSAGE(MSG_LEVEL_ERRR, MSG_JOINT_ERR_SERVO_ON_FAIL_0 + i);
                }
            }
        }
        Set_Servo_State(ServoState::MOTORCHECK);
        //----------------------------------------------------------------------
        // do or check somthing
        Set_Servo_State(ServoState::TORQUECHECK);
        //----------------------------------------------------------------------
        // finalize process
        Set_Servo_State(ServoState::DONE);
        //----------------------------------------------------------------------
        if(option == 1){
            int refon_ret = Set_ReferenceOnOff(true);
            if(refon_ret != MSG_OK) return refon_ret;
        }

        return MSG_OK;
    }

    ServoState Get_Servo(){
        return servo_stat;
    }

    int Set_Power(PowerOption opt, bool is_call_from_RT){        
        if(opt == PowerOption::Off){
            Call_Halt();
            Set_Servo_State(ServoState::NONE);
            if(is_call_from_RT){
                request_powerControl.store(PowerOption::Off, std::memory_order_relaxed);
            }else{
                LOG_WARNING("CMD Power Off");
                for(int i = 0; i < NO_OF_JOINT; ++i){
                    _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Brake(3, 0));
                }
                if(flag_reference_onoff){
                    std::cout<<"Soft Off Mode"<<std::endl;
                    flag_soft_power_cutoff = true;
                    std::this_thread::sleep_for(0.5s);
                    flag_soft_power_cutoff = false;
                }
                Set_ReferenceOnOff(false);                
                _gv_Handler_Lan->Power_Command(0);//Off anyway
            }
        }else if(opt == PowerOption::On){
            LOG_INFO("CMD Power On");

            if(!_gv_Handler_Lan->LAN_connectionStatus){
                return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_NOT_CONNECT_INTERFACE);
            }
            auto scb_state = _gv_Handler_SCB->Get_Infos();
            for(int scb = 0; scb < 2; ++scb){
                if(!scb_state.connection_flag[scb]){
                    return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_NOT_CONNECT_SAFETY_1 + scb);
                }
                if(!scb_state.configure_done[scb]){
                    return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_NOT_CONFIGURED_SAFETY_1 + scb);
                }
            }

            if(_gv_Handler_Lan->power_48V_in_stat != true){
                return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_INPUT_LOW_48V);
            }
            if(_gv_Handler_Lan->power_mc_sw_stat != true){
                return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SWITCH_EMG_MAIN);
            }

            if(_gv_Handler_Lan->power_pc_sw_stat != true){
                _gv_Handler_Lan->Power_Command(1);//On PC Sw

                std::unique_lock<std::mutex> lock(power_mutex);
                bool switched = power_cv.wait_for(lock, std::chrono::seconds(1), []{
                    return _gv_Handler_Lan->power_pc_sw_stat == true;
                });

                if (!switched) {
                    return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SCB_FET_CMD_IS_NOT_CHANGED);
                }
            }

            if(_gv_Handler_Lan->power_48V_out_stat != true){
                _gv_Handler_Lan->Power_Command(2);//Reset SCB
                
                std::unique_lock<std::mutex> lock(power_mutex);
                bool switched = power_cv.wait_for(lock, std::chrono::seconds(2), []{
                    return _gv_Handler_Lan->power_48V_out_stat == true;
                });

                if (!switched) {
                    return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_OUTPUT_LOW_48V);
                }
            }

            {
                bool is_comm_time_out = true;
                float is_comm_checking_time = 0.;
                while(1){
                    if(flag_connection_is_one){
                        is_comm_time_out = false;
                        break;
                    }
                    std::this_thread::sleep_for(50ms);
                    is_comm_checking_time += 0.05;
                    if(is_comm_checking_time > 2.0){
                        break;
                    }
                }

                if(is_comm_time_out){
                    Set_Power(PowerOption::Off, is_call_from_RT);
                    return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_NO_CAN_MESSAGE_CHECK_CABLE);
                }

                std::this_thread::sleep_for(1s);
            }
            
        }
        return MSG_OK;
    }

    bool Get_Power(){
        std::lock_guard<std::mutex> lock(power_mutex);
        return _gv_Handler_Lan->power_48V_out_stat;
    }

    int Get_Power_Switch(){
        return !_gv_Handler_Lan->power_mc_sw_stat;
    }

    int Get_Lan2Can_State(){
        return _gv_Handler_Lan->LAN_connectionStatus;
    }

    void Notify_PowerChanged() {
        std::lock_guard<std::mutex> lock(power_mutex);
        power_cv.notify_all();  // 상태가 바뀌었음을 알림
    }

    int Set_SafetyBoard_Para_Single(unsigned int reg, unsigned int dat){
        if(reg >= SFSET_NUMBER){
            return MSG_DESIRED_INDEX_IS_OVER_BOUND;
        }
        for(int k = 0; k < 2; ++k){
            _gv_Handler_SCB->Clear_Infos();
            _gv_Handler_Lan->CAN_writeData(_gv_Handler_SCB->Cmd_Setting(k, reg, dat));

            float time_out = 0.;
            while(1){
                std::this_thread::sleep_for(0.002s);
                time_out += 0.002;
                auto temp = _gv_Handler_SCB->Get_Infos();
                if(temp.config_flag[k] && (temp.config_addr[k] == reg) && (temp.config_data[k] == dat)){
                    break;
                }
                if(time_out > 0.05){
                    return MSG_EXECUTION_TIMEOUT;
                }
            }
        }
        return MSG_OK;
    }
    // ---------------------------------------------------------------
    // Reference
    // ---------------------------------------------------------------
    int Set_ReferenceOnOff(bool opt){
        if(opt == flag_reference_onoff){
            return MSG_OK;
        }
        if(opt == false){
            LOG_INFO("Reference Off");
        }else{
            if(servo_stat != ServoState::DONE){
                return MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_CANNOT_REF_ON_BEFORE_SERVO);
            }

            for(int i = 0; i < NO_OF_JOINT; i++){
                mINFO temp_motor_info = _gv_Handler_Motor[i]->Get_Infos();
                double enc_deg   = temp_motor_info.encoder_deg;
                double torque_Nm = temp_motor_info.torque_Nm;

                _gv_Handler_Motor[i]->Set_Last_Reference(enc_deg, torque_Nm, 1.0, 1.0, -1);
                
                rb_motion::Set_Motion_q(i, enc_deg);//ref on/off
                rb_motion::Set_Wrapper_q(i, enc_deg);

                output_lpf_q_input(i) = enc_deg;
                output_lpf_q_ang(i) = enc_deg;
                output_lpf_q_ang_old(i) = enc_deg;
            }
            output_lpf_q_vel = output_lpf_q_vel_old = output_lpf_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);

            LOG_INFO("Reference On");

            rb_socket_command_server::ascii_script_execution(parameter_userscript[15]);
        }
        flag_reference_onoff = opt;
        return MSG_OK;
    }

    bool Get_ReferenceOnOff(){
        return flag_reference_onoff;
    }

    double Set_Output_LPF(double t_alpha){
        output_lpf_alpha = rb_math::saturation_L_and_U(t_alpha, 0.1, 1.0);
        return output_lpf_alpha;
    }

    // ---------------------------------------------------------------
    // State / Flags
    // ---------------------------------------------------------------

    GET_SYSTEM_DATA_RET Get_System_Data(int option, std::string data_name){
        GET_SYSTEM_DATA_RET ret;
        ret.type = GET_SYS_DATA_NO_EXIST;
        ret.payload_num = 0;
        ret.payload_arr_length = 0;
        for(int i = 0; i < 32; ++i){
            ret.payload_arr[i] = 0.0;
        }
        ret.payload_str = "";

        (void)option;
        if(data_name == "RB_JOINT_REF"){
            for(int i = 0; i < NO_OF_JOINT; i++){
                ret.payload_arr[i] = rb_motion::Get_Wrapper_J()[i];
            }
            ret.payload_arr_length = NO_OF_JOINT;
            ret.type = GET_SYS_DATA_ARRAY;
        }else if(data_name == "RB_JOINT_ENC"){
            for(int i = 0; i < NO_OF_JOINT; i++){
                ret.payload_arr[i] = rb_system::Get_Motor_Encoder()[i];
            }
            ret.payload_arr_length = NO_OF_JOINT;
            ret.type = GET_SYS_DATA_ARRAY;
        }else if(data_name == "RB_POINT_REF"){
            for(int i = 0; i< NO_OF_CARTE; i++){
                ret.payload_arr[i] = rb_motion::Get_Wrapper_X()[i];
            }
            ret.payload_arr_length = NO_OF_CARTE;
            ret.type = GET_SYS_DATA_ARRAY;
        }else if(data_name == "RB_POINT_ENC"){
            for(int i = 0; i< NO_OF_CARTE; i++){
                ret.payload_arr[i] = rb_motion::Get_Wrapper_X()[i];
            }
            ret.payload_arr_length = NO_OF_CARTE;
            ret.type = GET_SYS_DATA_ARRAY;
        }else{
            if (data_name.rfind("RB_DIN_", 0) == 0){
                std::string numberPart = data_name.substr(7);
                try {
                    int num = std::stoi(numberPart);
                    if(num >= 0 && num < NO_OF_DIN){
                        ret.payload_num = _gv_Handler_Side->Get_State().din_filt[num];
                        ret.type = GET_SYS_DATA_NUMBER;
                    }
                }
                catch (...) {
                    std::cout<<"Wrong DIN Number Format"<<std::endl;
                }
            }else if(data_name.rfind("RB_DOUT_", 0) == 0){
                std::string numberPart = data_name.substr(8);
                try {
                    int num = std::stoi(numberPart);
                    if(num >= 0 && num < NO_OF_DOUT){
                        ret.payload_num = _gv_Handler_Side->Get_State().dout_raw[num];
                        ret.type = GET_SYS_DATA_NUMBER;
                    }
                }
                catch (...) {
                    std::cout<<"Wrong DIN Number Format"<<std::endl;
                }
            }
        }

        return ret;
    }

    bool Get_Is_Idle(){
        return (flag_direct_teaching == false && rb_motion::Get_Is_Idle());
    }

    int Get_Flag_Direct_Teaching(){
        return flag_direct_teaching;
    }

    int Get_Flag_Out_Collision_Occur(){
        return flag_collision_out_occur;
    }

    int Get_Flag_Self_Collision_Occur(){
        return flag_collision_self_occur;
    }

    uint8_t Get_Flag_Heart_Beat(){
        return flag_heart_beat;
    }

    // ---------------------------------------------------------------
    // RT
    // ---------------------------------------------------------------
    void Update_Zitter_Measurement(int64_t max_ns){
        g_max_jitter_ns = max_ns;
    }

    void Task_RealTime(){
        // -------------------------------------------------------------------------
        // SHM 
        // -------------------------------------------------------------------------
        pthread_mutex_lock(&mutex_shm);
        RT_Update_My_Shared_Memory();
        RT_Update_My_Intention_to_Friends();
        RT_Copy_From_Shared_Memory();
        pthread_mutex_unlock(&mutex_shm);

        // // TX command through Zenoh
        // if(flag_master_mode != 0){
        //     ST_SHM_M2F_STATE_CORE friends_core = rb_shareddata::getLocalShm()->robots[parameter_id_friend].m2f_state_core;
        //     static int servo_cnt = 0;
        //     servo_cnt++;
        //     if(!friends_core.status_is_refon && false){
        //         LOG_ERROR("FRIEND IS NOT REF ON");
        //         flag_master_mode = 0;
        //     }else{
        //         int ipc_ret = MSG_OK;

        //         if(flag_master_mode == 1){
        //             std::array<float, 7> target = rb_motion::Get_Wrapper_J();
        //             target.at(1) *= -1;
        //             target.at(2) *= -1;
        //             target.at(4) *= -1;
        //             target.at(6) *= -1;
        //             ipc_ret = rb_ipc::toFriend_ServoJ(friend_robot_namespace, target, ((float)RT_PERIOD_SEC) * 2, 0.05, 1, 0.1);
        //         }else if(flag_master_mode == 2){
        //             VectorCd m2s_current_carte_master;
        //             for(int k = 0; k < NO_OF_CARTE; ++k){
        //                 m2s_current_carte_master(k) = rb_motion::Get_Wrapper_X().at(k);
        //             }

        //             Matrix3d m2s_delta_R = rb_math::get_R_3x3(m2s_starting_carte_master).transpose() * rb_math::get_R_3x3(m2s_starting_carte_slave);
        //             Vector3d m2s_delta_P = rb_math::get_R_3x3(m2s_starting_carte_master).transpose() * (rb_math::get_P_3x1(m2s_starting_carte_slave) - rb_math::get_P_3x1(m2s_starting_carte_master));

        //             Matrix3d new_slave_R = rb_math::get_R_3x3(m2s_current_carte_master) * m2s_delta_R;
        //             Vector3d new_slave_E = rb_math::R_to_RPY(new_slave_R);
        //             Vector3d new_slave_P = rb_math::get_P_3x1(m2s_current_carte_master) + rb_math::get_R_3x3(m2s_current_carte_master) * m2s_delta_P;
        //             double   new_slave_A = rb_math::get_REDUN_1x1(m2s_starting_carte_slave);
        //             std::array<float, 7> target;
        //             target.at(0) = new_slave_P(0);
        //             target.at(1) = new_slave_P(1);
        //             target.at(2) = new_slave_P(2);
        //             target.at(3) = new_slave_E(0);
        //             target.at(4) = new_slave_E(1);
        //             target.at(5) = new_slave_E(2);
        //             target.at(6) = new_slave_A;
        //             ipc_ret = rb_ipc::toFriend_ServoL(friend_robot_namespace, target, ((float)RT_PERIOD_SEC) * 2, 0.05, 1, 0.1);
        //         }

        //         if(ipc_ret != MSG_OK){
        //             LOG_ERROR("FRIEND IPC RETURN FAIL" + std::to_string(ipc_ret));
        //             flag_master_mode = 0;
        //         }
        //     }
        // }

        {//SHM Receiver
            ST_SHM_F2M_COMMAND temp_income = rb_shareddata::getLocalShm()->robots[parameter_id].f2m_command;
            if(temp_income.command_flag == 1){
                TARGET_INPUT temp_input = rb_math::Make_Input_Zero(FRAME_JOINT);
                for(int j = 0; j < NO_OF_JOINT; ++j){
                    temp_input.target_value[j] = temp_income.command_f[j];
                }
                rb_motion::Start_Motion_SERVO_J(temp_input, RT_PERIOD_SEC, 0.05, 1.0, 0.1);
            }else if(temp_income.command_flag == 2){
                TARGET_INPUT temp_input = rb_math::Make_Input_Zero(FRAME_GLOBAL);
                for(int j = 0; j < NO_OF_CARTE; ++j){
                    temp_input.target_value[j] = temp_income.command_f[j];
                }
                rb_motion::Start_Motion_SERVO_L(temp_input, RT_PERIOD_SEC, 0.05, 1.0, 0.1);
            }
        }

        // -------------------------------------------------------------------------
        // Update 
        // -------------------------------------------------------------------------
        if(_gv_Handler_Lan != NULL) _gv_Handler_Lan->LAN_Flush();

        auto state_ModelChange = request_ModelChange.load(std::memory_order_relaxed);
        if(state_ModelChange != 0){
            if(state_ModelChange == 1){
                request_ModelChange.store(2, std::memory_order_relaxed);
            }
            std::cout<<"skip while TOOL changing"<<std::endl;
            return;
        }

        
        // -------------------------------------------------------------------------
        // Do motion
        // -------------------------------------------------------------------------
        double timer_scaler                         = RT_SysSpeed_Handler();
        auto [mot_q_ang,  mot_x_pos, mot_lpf_alpha] = rb_motion::Task_Motion_Handler(RT_PERIOD_SEC, timer_scaler, flag_is_break, _sys_timescale[(int)SystemTimeScaler::BREAK].output);
        auto [wrap_q_ang, wrap_q_vel, wrap_q_acc, wrap_lpf_alpha]   = rb_motion::Task_Wrapper_Handler(RT_PERIOD_SEC, timer_scaler, mot_q_ang, mot_x_pos);

        Set_Output_LPF(rb_math::return_small(mot_lpf_alpha, wrap_lpf_alpha));

        float fb_gain = 1.0;
        float ff_gain = 1.0;//1.0;//1.0;
        
        // -------------------------------------------------------------------------
        // Self Collsion
        // -------------------------------------------------------------------------
        auto [sc_is_coll, sc_is_elbow, sc_idx, sc_dist] = Calc_Self_Collision(parameter_robot, wrap_q_ang, parameter_self_coll_dist_int, parameter_self_coll_dist_ext);
        if(sc_is_coll == true){
            if(parameter_self_coll_mode == 0){
                if(!flag_collision_self_occur){
                    std::cout<<"self coll will occur at "<<sc_idx<<" = "<<sc_dist<<std::endl;
                }
                rb_motion::Set_Motion_Mode(rb_motion::MotionMode::MOVE_NONE);
                rb_motion::Stop_Wrapper_All();

                for(int i = 0; i < NO_OF_JOINT; i++){
                    double forced_angle = output_lpf_q_input(i);
                    rb_motion::Set_Motion_q(i, forced_angle);//self coll
                    rb_motion::Set_Wrapper_q(i, forced_angle);

                    mot_q_ang(i) = forced_angle;
                    wrap_q_ang(i) = forced_angle;
                }

                wrap_q_vel = wrap_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);
            }
            if(flag_collision_self_occur != true){
                int sc_th_up = sc_idx / 1000;
                int sc_th_dw = sc_idx % 1000;
                if(sc_th_up >= 1){
                    // AREA vs *
                    if(sc_th_dw >= 100){
                        ;//AREA vs TOOL
                        MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SELF_COLL_AREA_TOOL_OCCUR);
                    }else{
                        ;//AREA vs ARM
                        MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SELF_COLL_AREA_ARM_OCCUR);
                    }
                }else{
                    if(sc_th_dw >= 100){
                        ;//ARM vs TOOL
                        MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SELF_COLL_ARM_TOOL_OCCUR);
                    }else{
                        ;//ARM vs ARM
                        MESSAGE(MSG_LEVEL_WARN, MSG_SYSTEM_SELF_COLL_ARM_ARM_OCCUR);
                    }
                }
            }
        }
        flag_collision_self_occur = sc_is_coll;

        
        // -------------------------------------------------------------------------
        // Validation
        // -------------------------------------------------------------------------
        bool is_there_q_speed_err = false;
        for(int i = 0; i < NO_OF_JOINT; ++i){
            if(fabs(wrap_q_vel(i)) > Get_Motor_HwMax_Vel()(i)){
                is_there_q_speed_err = true;
                LOG_ERROR("JNT OVR SPD REF: " + std::to_string(i) + " / " + std::to_string(fabs(wrap_q_vel(i))));
            }
        }
        if(is_there_q_speed_err){
            rb_motion::Set_Motion_Mode(rb_motion::MotionMode::MOVE_NONE);
            rb_motion::Stop_Wrapper_All();

            for(int i = 0; i < NO_OF_JOINT; i++){
                double forced_angle = output_lpf_q_input(i);
                rb_motion::Set_Motion_q(i, forced_angle);//speed error
                rb_motion::Set_Wrapper_q(i, forced_angle);

                mot_q_ang(i) = forced_angle;
                wrap_q_ang(i) = forced_angle;
            }

            wrap_q_vel = wrap_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);

            MESSAGE(MSG_LEVEL_ERRR, MSG_MOVE_RUNTIME_TERMINATE_OVER_SPD);
            std::cout<<"skip OVRSPD"<<std::endl;
            return;
        }
        // -------------------------------------------------------------------------
        // Output Filter
        // -------------------------------------------------------------------------
        output_lpf_q_input = wrap_q_ang;
        output_lpf_q_ang = (1.0 - output_lpf_alpha) * output_lpf_q_ang + output_lpf_alpha * output_lpf_q_input;
        output_lpf_q_vel = (output_lpf_q_ang - output_lpf_q_ang_old) / RT_PERIOD_SEC;
        output_lpf_q_acc = (output_lpf_q_vel - output_lpf_q_vel_old) / RT_PERIOD_SEC;
        output_lpf_q_ang_old = output_lpf_q_ang;
        output_lpf_q_vel_old = output_lpf_q_vel;
        
        // -------------------------------------------------------------------------
        // Direct Teaching
        // -------------------------------------------------------------------------
        double tfb_slope = sloper_tfb_button.Update(((float)_gv_Handler_Toolflange->Get_State().button));
        if(flag_reference_onoff && rb_motion::Get_Is_Idle()){
            if(flag_sw_switch_free_drive){
                Set_Direct_Teaching_Flag(true);
            }else{
                if(tfb_slope > 0.001){
                    Set_Direct_Teaching_Flag(true);
                }else{
                    Set_Direct_Teaching_Flag(false);
                }
            }
        }else{
            flag_sw_switch_free_drive =  false;
            Set_Direct_Teaching_Flag(false);
        }

        if(flag_direct_teaching || flag_soft_power_cutoff){
            fb_gain = 0.0;
            ff_gain = 1.0;

            for(int i = 0; i < NO_OF_JOINT; i++){
                mINFO temp_motor_info = _gv_Handler_Motor[i]->Get_Infos();
                double enc_deg   = temp_motor_info.encoder_deg;
                
                rb_motion::Set_Motion_q(i, enc_deg);//direc teaching
                rb_motion::Set_Wrapper_q(i, enc_deg);

                mot_q_ang(i) = enc_deg;
                wrap_q_ang(i) = enc_deg;
                output_lpf_q_input(i) = enc_deg;
                output_lpf_q_ang(i) = enc_deg;
                output_lpf_q_ang_old(i) = enc_deg;
            }
            wrap_q_vel = wrap_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);
            output_lpf_q_vel = output_lpf_q_vel_old = output_lpf_q_acc = VectorJd::Zero(NO_OF_JOINT, 1);
        }


        // -------------------------------------------------------------------------
        // Calc Dynamics
        // -------------------------------------------------------------------------
        RBDLrobot robot_for_dynamics;    
        robot_for_dynamics.Init_Robot(Get_CurrentRobotParameter(), Get_CurrentTcpParameter(), rb_system::Get_CurrentGravityParameter());
    
        VectorJd t_torque_ID;
        if(flag_joint_impedance_mode && flag_direct_teaching == false){
            VectorJd                    temp_q_ang = VectorJd::Zero(NO_OF_JOINT, 1);
            VectorJd                    temp_q_vel = VectorJd::Zero(NO_OF_JOINT, 1);
            for(int i = 0; i < NO_OF_JOINT; i++){
                temp_q_ang(i) = _gv_Handler_Motor[i]->Get_Infos().encoder_deg;
            }
            t_torque_ID = robot_for_dynamics.Calc_InverseDynamics(temp_q_ang, temp_q_vel, output_lpf_q_acc);
        }else{
            t_torque_ID = robot_for_dynamics.Calc_InverseDynamics(output_lpf_q_ang, output_lpf_q_vel, output_lpf_q_acc);        
        }

        VectorJd t_torque_FF = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd t_torque_Meas = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd t_torque_Limit = VectorJd::Zero(NO_OF_JOINT, 1);
        VectorJd t_currentA_Meas = VectorJd::Zero(NO_OF_JOINT, 1);
        for(int i = 0; i < NO_OF_JOINT; ++i){
            mPARA joint_para = _gv_Handler_Motor[i]->Get_Parameters();
            mINFO joint_info = _gv_Handler_Motor[i]->Get_Infos();

            t_torque_Meas(i) = joint_info.torque_Nm;
            t_torque_Limit(i) = joint_para.para_hwmax_torque_rept;
            t_currentA_Meas(i) = joint_info.torque_mA * 0.001;

            double m_i = joint_para.para_inertia;
            double m_a = joint_para.para_friction_a;
            double m_b = joint_para.para_friction_b;

            double ff_mA = 0.;
            if(flag_direct_teaching || flag_soft_power_cutoff){
                ff_mA  = m_i * joint_info.encoder_deg_acc_LPF * MATH_D2R 
                        + m_a * joint_info.encoder_deg_vel_LPF * MATH_D2R 
                        + m_b * rb_math::filt_Zero(joint_info.encoder_deg_vel_LPF * MATH_D2R, -0.1, 0.1, -1.0, 1.0);

                ff_mA *= parameter_direct_teach_sensitivity(i);

                ff_mA *= (0.7);
                ff_mA *= tfb_slope;
                if(flag_soft_power_cutoff){
                    ff_mA = 0.;
                }
                // Velocity Limiter
                ff_mA -= rb_math::sign(joint_info.encoder_deg_vel_LPF) * rb_math::filt_Line(fabs(joint_info.encoder_deg_vel_LPF), (0.2 * joint_para.para_hwmax_vel), (0.6 * joint_para.para_hwmax_vel), 0., m_b * 5.);
                // Brake
                double limit_up = joint_para.para_limit_angleDeg_Up - 10.;
                double limit_dw = joint_para.para_limit_angleDeg_Low + 10.;
                if(joint_info.encoder_deg >= limit_up){
                    ff_mA += rb_math::filt_Line(joint_info.encoder_deg, limit_up, joint_para.para_limit_angleDeg_Up, 0, -m_b * 5.);
                }else if(joint_info.encoder_deg <= limit_dw){
                    ff_mA += rb_math::filt_Line(joint_info.encoder_deg, joint_para.para_limit_angleDeg_Low, limit_dw, +m_b * 5., 0);
                }
                // Damper                
                ff_mA += ((tfb_slope - 1.) * 10. * m_a * joint_info.encoder_deg_vel_LPF * MATH_D2R);
                // Saturation
                ff_mA = rb_math::saturation_L_and_U(ff_mA, -5000, +5000);
            }else{
                ff_mA  = m_i * wrap_q_acc(i) * MATH_D2R 
                        + m_a * wrap_q_vel(i) * MATH_D2R 
                        + m_b * rb_math::filt_Line(wrap_q_vel(i) * MATH_D2R, -0.1, 0.1, -1.0, 1.0);
            }
            t_torque_FF(i) = ff_mA * 0.001 * joint_para.para_torque_const * joint_para.para_reduction_rate;
        }
        VectorJd t_torque_Esti = t_torque_ID + t_torque_FF;

        if(flag_direct_teaching || flag_soft_power_cutoff){//<----------------------------
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(std::isnan(t_torque_Esti(i)) || std::isinf(t_torque_Esti(i))){
                    t_torque_Esti(i) = 0.;
                    std::cout<<"t_torque_ID: "<<t_torque_ID.transpose()<<std::endl;
                    std::cout<<"t_torque_FF: "<<t_torque_FF.transpose()<<std::endl;
                    std::cout<<"t_torque_Esti: "<<t_torque_Esti.transpose()<<std::endl;
                }
            }
        }
        torque_esetimated = t_torque_Esti;

        // -------------------------------------------------------------------------
        // Collision Detection
        // -------------------------------------------------------------------------
        bool is_over_torque = RT_Collision_Checker(t_torque_Esti, t_torque_Meas, t_torque_Limit);
        if(flag_reference_onoff 
            && flag_direct_teaching == false
            && flag_soft_power_cutoff == false 
            && parameter_out_coll_onoff 
            && flag_joint_impedance_mode == false){

            flag_collision_out_occur |= is_over_torque;
        }else{
            torque_delta_lpf = VectorJd::Zero(NO_OF_JOINT, 1);
            flag_collision_out_occur = false;
        }
        // -------------------------------------------------------------------------
        // Power Estimation
        // -------------------------------------------------------------------------
        double measured_Watt = (_gv_Handler_Lan->power_48V_amp + 0.2) * 48.;
        double estimated_Watt = 0.;    
        for(int i = 0; i < NO_OF_JOINT; ++i){
            mINFO joint_info = _gv_Handler_Motor[i]->Get_Infos();
            mPARA joint_para = _gv_Handler_Motor[i]->Get_Parameters();
            double t_esti_current_A = t_torque_Esti(i) / joint_para.para_reduction_rate / joint_para.para_torque_const;
            double kinematic_Watt = t_torque_Esti(i) * (output_lpf_q_vel(i) * MATH_D2R);
            double static_Watt = t_esti_current_A * t_esti_current_A * joint_para.para_r_q;
            estimated_Watt += (rb_math::saturation_Low(kinematic_Watt, 0) + static_Watt);
        }
        if(!flag_reference_onoff) estimated_Watt = 0;
        delta_Watt_lpf_dc_and_esti =    0.97 * delta_Watt_lpf_dc_and_esti + 0.03 * fabs(measured_Watt - estimated_Watt);
        double delta_Watt_Slope = sloper_delta_Watt.Update(static_cast<float>(delta_Watt_lpf_dc_and_esti > 333));
        if(delta_Watt_Slope > 0.5){
            LOG_ERROR("DC TOO MUCH POWER");
            Set_Power(PowerOption::Off, true);
        }

        // -------------------------------------------------------------------------
        // Send Message
        // -------------------------------------------------------------------------
        if(flag_reference_onoff){
            for(int i = 0; i < NO_OF_JOINT; ++i){
                if(flag_dev_configuration[i] == 0)  continue;
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Motor[i]->Cmd_Control(output_lpf_q_ang(i), t_torque_Esti(i), fb_gain, ff_gain, torque_limit_A[i]));
            }
        }
        // -------------------------------------------------------------------------
        // Check
        // -------------------------------------------------------------------------
        RT_StateChecker();
        RT_Task_LED();
        RT_IO_Handler();
        // -------------------------------------------------------------------------
        
    }

    int TestTestTest(){
        _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdPowerControl(24));

        std::this_thread::sleep_for(1000ms);
        _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdGMbus_Init(0, 115200, 0));
        std::this_thread::sleep_for(1000ms);
        for(int k = 0; k < 3; k++){
            for(int f = 0; f < 6; f++){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdGMbus_Write_Single(1, 6, 1486 + 2 *f, 1000));
                std::this_thread::sleep_for(10ms);
            }
            std::this_thread::sleep_for(1000ms);
            for(int f = 0; f < 6; f++){
                _gv_Handler_Lan->CAN_writeData(_gv_Handler_Toolflange->CmdGMbus_Write_Single(1, 6, 1486 + 2 *f, 0));
                std::this_thread::sleep_for(10ms);
            }
            std::this_thread::sleep_for(1000ms);
        }
        return MSG_OK;
    }

}

