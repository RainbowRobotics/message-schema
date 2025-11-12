#include "ipc.h"
#include <iostream>

#include "dof.h"
#include "common.h"

#include "session.h"
// #include "session_options.h"
#include "concurrentqueue.h"
//---------------------------------------------------
#include "comm_generated/state_core_generated.h"
#include "comm_generated/state_message_generated.h"
#include "comm_generated/state_log_generated.h"

#include "comm_generated/func_servo_generated.h"
#include "comm_generated/func_flow_generated.h"
#include "comm_generated/func_move_generated.h"
#include "comm_generated/func_config_generated.h"
#include "comm_generated/func_arm_generated.h"
#include "comm_generated/func_box_generated.h"
#include "comm_generated/func_set_generated.h"
#include "comm_generated/func_get_generated.h"

//---------------------------------------------------
#include "rb_core/system.h"
#include "rb_common/rmath.h"
#include "rb_motion/motion.h"
//---------------------------------------------------
// #include <grpcpp/grpcpp.h>
// #include "comm_proto/helloworld.grpc.pb.h"
// using grpc::Server;
// using grpc::ServerBuilder;
// using grpc::ServerContext;
// using grpc::Status;
// using helloworld::Greeter;
// using helloworld::HelloRequest;
// using helloworld::HelloReply;
//---------------------------------------------------

// #define ADD_SERVE_SIMPLE(CMD_NAME, func_req, func_resp, func_body) \
//     session.Serve<func_req, func_resp>( \
//         CMD_NAME, [](flatbuffers::FlatBufferBuilder& fbb, const func_req* req) { \
//             std::cout << "[IPC] Function call: " << CMD_NAME << std::endl; \
//             int return_int = MSG_OK; \
//             func_body \
//             return IPC::CreateResponse_Functions(fbb, return_int); \
//         } \
//     )

#define ADD_SERVE_SIMPLE(CMD_NAME, func_req, func_resp, ...) \
    session.Serve<func_req, func_resp>( \
        CMD_NAME, [](flatbuffers::FlatBufferBuilder& fbb, const func_req* req) { \
            std::cout << "[IPC] Function call: " << CMD_NAME << std::endl; \
            int return_int = MSG_OK; \
            __VA_ARGS__; \
            return IPC::CreateResponse_Functions(fbb, return_int); \
        } \
    );

#define ADD_SERVE_BLANK(CMD_NAME, func_req, func_resp, ...) \
    session.Serve<func_req, func_resp>( \
        CMD_NAME, [](flatbuffers::FlatBufferBuilder& fbb, const func_req* req) { \
            std::cout << "[IPC] Function call: " << CMD_NAME << std::endl; \
            __VA_ARGS__; \
        } \
    );

namespace rb_ipc {
    namespace {
        pthread_t hThread_ipccomm_rx;
        pthread_t hThread_ipccomm_tx;
        pthread_t hThread_grpccomm_server;

        moodycamel::ConcurrentQueue<PUB_MESSAGE_ST>     que_publish_message;
        moodycamel::ConcurrentQueue<PUB_LOG_ST>         que_publish_log;
        //----------------------------------------------------------------------------------------------
        // RX
        //----------------------------------------------------------------------------------------------
        void *thread_ipccomm_rx(void *) {

            auto [s_category, s_model, s_version, s_alias] = rb_system::Get_System_Basic_Info();

            rb::io::SessionOptions options;
            options.backend = rb::io::SessionBackendType::kZenoh;
            options.ns = s_model;
            options.zenoh_config = rb::io::ZenohConfig::FromFile("config.json5");
            auto session = rb::io::Session::Open(std::move(options));

            // session.Subscribe<IPC::State_Core>("state_core", [](IPC::State_CoreT robot_core_state) {
            //     if (robot_core_state.joint_q_ref) {
            //         const auto *arr = robot_core_state.joint_q_ref->f();
            //         if (arr) {
            //             float q_ref[7];
            //             for (int i = 0; i < 7; ++i) q_ref[i] = arr->Get(i);
            //             std::cout<<"SUB REF: "<<q_ref[0]<<" / "<<q_ref[1]<<" / "<<q_ref[2]<<" / "<<q_ref[3]<<" / "<<q_ref[4]<<" / "<<q_ref[5]<<" / "<<q_ref[6]<<std::endl;
            //         }
            //     }
            //     if (robot_core_state.joint_q_enc) {
            //         const auto *arr = robot_core_state.joint_q_enc->f();
            //         if (arr) {
            //             float q_ref[7];
            //             for (int i = 0; i < 7; ++i) q_ref[i] = arr->Get(i);
            //             std::cout<<"SUB ENC: "<<q_ref[0]<<" / "<<q_ref[1]<<" / "<<q_ref[2]<<" / "<<q_ref[3]<<" / "<<q_ref[4]<<" / "<<q_ref[5]<<" / "<<q_ref[6]<<std::endl;
            //         }
            //     }

            //     std::cout<<"SUB STATE: "<<(int)robot_core_state.status_servo_num<<" / "<<(int)robot_core_state.status_is_refon<<" / "<<(int)robot_core_state.status_dt_mode<<std::endl;
            // });

            // -----------------------------------------------------------------------
            // Arm
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("save_collision_parameter", IPC::Request_Save_Collision_Parameter, IPC::Response_Functions, {
                int p_onoff = req->onoff();
                int p_react = req->react();
                float p_th = req->threshold();
                return_int = rb_system::Save_Out_Coll_Para(p_onoff, p_react, p_th);
            });
            ADD_SERVE_SIMPLE("save_selfcoll_parameter", IPC::Request_Save_SelfColl_Parameter, IPC::Response_Functions, {
                int p_mode = req->mode();
                float p_int = req->dist_internal();
                float p_ext = req->dist_external();
                return_int = rb_system::Save_Self_Coll_Para(p_mode, p_int, p_ext);
            });
            ADD_SERVE_SIMPLE("save_gravity_parameter", IPC::Request_Save_Gravity_Parameter, IPC::Response_Functions, {
                int p_mode = req->mode();
                float p_gx = req->gx();
                float p_gy = req->gy();
                float p_gz = req->gz();
                return_int = rb_system::Save_Gravity_Para(p_mode, p_gx, p_gy, p_gz);
            });
            ADD_SERVE_SIMPLE("save_direct_teach_sensitivity", IPC::Request_Save_Direct_Teach_Sensitivity, IPC::Response_Functions, {
                std::array<float, NO_OF_JOINT> temp_input;
                auto arr = req->sensitivity()->f();
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    temp_input[i] = arr->Get(i);
                }
                return_int = rb_system::Save_Direct_Teaching_Sensitivity(temp_input);
            });
            ADD_SERVE_SIMPLE("save_tool_parameter", IPC::Request_Save_Tool_List_Para, IPC::Response_Functions, {
                int tar_no = req->tool_no();
                TCP_CONFIG tar_conf;
                tar_conf.tool_name = req->tool_name()->c_str();
                tar_conf.tcp_offset = Eigen::Vector3d(req->tcp_x(), req->tcp_y(), req->tcp_z());
                tar_conf.tcp_rotation = rb_math::RPY_to_R(req->tcp_rx(), req->tcp_ry(), req->tcp_rz());
                tar_conf.com_mass = req->mass_m();
                tar_conf.com_offset = Eigen::Vector3d(req->mass_x(), req->mass_y(), req->mass_z());
                tar_conf.box_type = req->box_type();
                tar_conf.box_parameter[0] = req->box_para_0();
                tar_conf.box_parameter[1] = req->box_para_1();
                tar_conf.box_parameter[2] = req->box_para_2();
                tar_conf.box_parameter[3] = req->box_para_3();
                tar_conf.box_parameter[4] = req->box_para_4();
                tar_conf.box_parameter[5] = req->box_para_5();              
                tar_conf.box_parameter[6] = req->box_para_6();
                tar_conf.box_parameter[7] = req->box_para_7();
                tar_conf.box_parameter[8] = req->box_para_8();
                return_int = rb_system::Save_TcpParameter(tar_no, tar_conf);
            });
            // -----------------------------------------------------------------------
            // BOX
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("save_user_frame_parameter", IPC::Request_Save_User_Frame, IPC::Response_Functions, {
                USERF_CONFIG tar_conf;
                tar_conf.userf_name = req->userf_name()->c_str();
                tar_conf.userf_offset = Eigen::Vector3d(req->userf_x(), req->userf_y(), req->userf_z());
                tar_conf.userf_rotation = rb_math::RPY_to_R(req->userf_rx(), req->userf_ry(), req->userf_rz());
                return_int = rb_system::Save_UserFrameParameter(req->userf_no(), tar_conf);
            });
            ADD_SERVE_SIMPLE("save_area_parameter", IPC::Request_Save_Area_Para, IPC::Response_Functions, {
                AREA_CONFIG tar_conf;
                tar_conf.area_name = req->area_name()->c_str();
                tar_conf.area_type = req->area_type();          
                tar_conf.area_offset = Eigen::Vector3d(req->area_x(), req->area_y(), req->area_z());
                tar_conf.area_rotation = rb_math::RPY_to_R(req->area_rx(), req->area_ry(), req->area_rz());
                tar_conf.area_parameter[0] = req->area_para_0();
                tar_conf.area_parameter[1] = req->area_para_1();
                tar_conf.area_parameter[2] = req->area_para_2();
                return_int = rb_system::Save_AreaParameter(req->area_no(), tar_conf);
            });
            ADD_SERVE_SIMPLE("save_side_dout_function", IPC::Request_Save_SideDout_SpecialFunc, IPC::Response_Functions, {
                return_int = rb_system::Save_Box_Special_Dout(req->port_num(), req->desired_function());
            });
            ADD_SERVE_SIMPLE("save_side_din_function", IPC::Request_Save_SideDin_SpecialFunc, IPC::Response_Functions, {
                return_int = rb_system::Save_Box_Special_Din(req->port_num(), req->desired_function());
            });
            ADD_SERVE_SIMPLE("save_side_din_filter", IPC::Request_Save_SideDin_FilterCount, IPC::Response_Functions, {
                return_int = rb_system::Save_Box_FilterCount_Din(req->port_num(), req->desired_count());
            });
            ADD_SERVE_SIMPLE("call_side_dout", IPC::Request_SideDout_General, IPC::Response_Functions, {
                return_int = rb_system::Set_Digital_Output(req->port_num(), req->desired_out());
            });
            ADD_SERVE_SIMPLE("call_side_aout", IPC::Request_SideAout_General, IPC::Response_Functions, {
                return_int = rb_system::Set_Analog_Output(req->port_num(), req->desired_voltage());
            });
            // -----------------------------------------------------------------------
            // Servo
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("call_powercontrol", IPC::Request_PowerControl, IPC::Response_Functions, {
                if(req->power_option() == 1){
                    return_int = rb_system::Set_Power(rb_system::PowerOption::On, false);
                }else{
                    return_int = rb_system::Set_Power(rb_system::PowerOption::Off, false);
                }
            });
            ADD_SERVE_SIMPLE("call_servocontrol", IPC::Request_ServoControl, IPC::Response_Functions, {
                return_int = rb_system::Set_Servo(req->servo_option(), 1);
            });
            ADD_SERVE_SIMPLE("call_referencecontrol", IPC::Request_ReferenceControl, IPC::Response_Functions, {
                return_int = rb_system::Set_ReferenceOnOff(req->refcontrol_option());
            });
            // -----------------------------------------------------------------------
            // Flow
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("call_speedbar", IPC::Request_MotionSpeedBar, IPC::Response_Functions, {
                return_int = rb_system::Set_MoveSpeedBar(req->alpha());
            });
            ADD_SERVE_SIMPLE("call_pause", IPC::Request_MotionPause, IPC::Response_Functions, {
                return_int = rb_system::Call_MovePause();
            });
            ADD_SERVE_SIMPLE("call_resume", IPC::Request_MotionResume, IPC::Response_Functions, {
                return_int = rb_system::Call_MoveResume();
            });
            ADD_SERVE_SIMPLE("call_reset_outcoll", IPC::Request_MotionResetOutColl, IPC::Response_Functions, {
                return_int = rb_system::Call_Reset_Out_Coll();
            });
            
            // -----------------------------------------------------------------------
            // Config call
            // -----------------------------------------------------------------------
            ADD_SERVE_BLANK("call_whoami", IPC::Request_CallWhoAmI, IPC::Response_CallWhoamI, {
                auto [ts_category, ts_model, ts_version, ts_alias] = rb_system::Get_System_Basic_Info();

                IPC::Response_CallWhoamIT who_am_i_T;
                who_am_i_T.category = ts_category;
                who_am_i_T.model = ts_model;
                who_am_i_T.version = ts_version;
                who_am_i_T.alias = ts_alias;
                return IPC::Response_CallWhoamI::Pack(fbb, &who_am_i_T);
            });
            ADD_SERVE_BLANK("call_config_controlbox", IPC::Request_CallConfigControlBox, IPC::Response_CallConfigControlBox, {
                IPC::Response_CallConfigControlBoxT cbox_T; 
                cbox_T.dout_special_func = std::make_unique<IPC::N_DOUT_u>(rb_system::Get_Box_Special_Dout());
                cbox_T.din_special_func = std::make_unique<IPC::N_DIN_u>(rb_system::Get_Box_Special_Din());
                cbox_T.din_filter_count = std::make_unique<IPC::N_DIN_u>(rb_system::Get_Box_FilterCount_Din());         

                for (int i = 0; i < NO_OF_AREA; ++i) {
                    AREA_CONFIG src = rb_system::Get_DesiredAreaParameter(i);

                    auto areaT = std::make_unique<IPC::ST_Config_AreaT>();
                    areaT->area_name = src.area_name;
                    areaT->area_type = static_cast<int>(src.area_type);
                    {
                        std::array<float, 3> arr = {
                            static_cast<float>(src.area_offset.x()),
                            static_cast<float>(src.area_offset.y()),
                            static_cast<float>(src.area_offset.z())
                        };
                        areaT->area_offset = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    {
                        Eigen::Vector3d temp_euler = rb_math::R_to_RPY(src.area_rotation);
                        std::array<float, 3> arr = {
                            static_cast<float>(temp_euler.x()),
                            static_cast<float>(temp_euler.y()),
                            static_cast<float>(temp_euler.z())
                        };
                        areaT->area_euler = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    {
                        std::array<float, 3> arr = {
                            static_cast<float>(src.area_parameter[0]),
                            static_cast<float>(src.area_parameter[1]),
                            static_cast<float>(src.area_parameter[2])
                        };
                        areaT->area_para = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    switch (i) {
                        case 0: cbox_T.area_configs_0 = std::move(areaT); break;
                        case 1: cbox_T.area_configs_1 = std::move(areaT); break;
                        case 2: cbox_T.area_configs_2 = std::move(areaT); break;
                        case 3: cbox_T.area_configs_3 = std::move(areaT); break;
                        case 4: cbox_T.area_configs_4 = std::move(areaT); break;
                        case 5: cbox_T.area_configs_5 = std::move(areaT); break;
                        case 6: cbox_T.area_configs_6 = std::move(areaT); break;
                        case 7: cbox_T.area_configs_7 = std::move(areaT); break;
                    }
                }

                for (int i = 0; i < NO_OF_USERF; ++i) {
                    USERF_CONFIG src = rb_system::Get_DesiredUserFrameParameter(i);

                    auto userfT = std::make_unique<IPC::ST_Config_UserFrameT>();
                    userfT->userf_name = src.userf_name;
                    {
                        std::array<float, 3> arr = {
                            static_cast<float>(src.userf_offset.x()),
                            static_cast<float>(src.userf_offset.y()),
                            static_cast<float>(src.userf_offset.z())
                        };
                        userfT->userf_offset = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    {
                        Eigen::Vector3d temp_euler = rb_math::R_to_RPY(src.userf_rotation);
                        std::array<float, 3> arr = {
                            static_cast<float>(temp_euler.x()),
                            static_cast<float>(temp_euler.y()),
                            static_cast<float>(temp_euler.z())
                        };
                        userfT->userf_euler = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    switch (i) {
                        case 0: cbox_T.user_frame_0 = std::move(userfT); break;
                        case 1: cbox_T.user_frame_1 = std::move(userfT); break;
                        case 2: cbox_T.user_frame_2 = std::move(userfT); break;
                        case 3: cbox_T.user_frame_3 = std::move(userfT); break;
                        case 4: cbox_T.user_frame_4 = std::move(userfT); break;
                        case 5: cbox_T.user_frame_5 = std::move(userfT); break;
                        case 6: cbox_T.user_frame_6 = std::move(userfT); break;
                        case 7: cbox_T.user_frame_7 = std::move(userfT); break;
                    }
                }

                return IPC::Response_CallConfigControlBox::Pack(fbb, &cbox_T);
            });
            ADD_SERVE_BLANK("call_config_robotarm", IPC::Request_CallConfigRobotArm, IPC::Response_CallConfigRobotArm, {
                IPC::Response_CallConfigRobotArmT robot_T;

                auto [t_oc_onoff, t_oc_react, t_oc_limit] = rb_system::Get_Out_Coll_Para();
                robot_T.out_coll_onoff = static_cast<uint8_t>(t_oc_onoff);
                robot_T.out_coll_react = static_cast<uint8_t>(t_oc_react);
                robot_T.out_coll_limit = t_oc_limit;

                auto [t_sc_mode, t_sc_dist_int, t_sc_dist_ext] = rb_system::Get_Self_Coll_Para();
                robot_T.self_coll_mode = t_sc_mode;
                robot_T.self_coll_distance_inter = t_sc_dist_int;
                robot_T.self_coll_distance_exter = t_sc_dist_ext;

                auto [t_gv_mode, t_gv_gx, t_gv_gy, t_gv_gz] = rb_system::Get_Gravity_Para();
                robot_T.gravity_mode = t_gv_mode;
                robot_T.gravity_gx = t_gv_gx;
                robot_T.gravity_gy = t_gv_gy;
                robot_T.gravity_gz = t_gv_gz;

                robot_T.direct_teaching_sensitivity = std::make_unique<IPC::N_JOINT_f>(rb_system::Get_Direct_Teaching_Sensitivity());

                return IPC::Response_CallConfigRobotArm::Pack(fbb, &robot_T);
            });
            ADD_SERVE_BLANK("call_config_toollist", IPC::Request_CallConfigToolList, IPC::Response_CallConfigToolList, {
                IPC::Response_CallConfigToolListT tool_list_T;
                for (int i = 0; i < 8; i++) {
                    TCP_CONFIG src = rb_system::Get_DesiredTcpParameter(i);

                    auto toolT = std::make_unique<IPC::ST_Tool_ParaT>();
                    toolT->tool_name = src.tool_name;
                    toolT->com_mass  = static_cast<float>(src.com_mass);

                    {
                        std::array<float, 3> arr = {
                            static_cast<float>(src.com_offset.x()),
                            static_cast<float>(src.com_offset.y()),
                            static_cast<float>(src.com_offset.z())
                        };
                        toolT->com_offset = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    {
                        std::array<float, 3> arr = {
                            static_cast<float>(src.tcp_offset.x()),
                            static_cast<float>(src.tcp_offset.y()),
                            static_cast<float>(src.tcp_offset.z())
                        };
                        toolT->tcp_offset = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }
                    {
                        Eigen::Vector3d temp_euler = rb_math::R_to_RPY(src.tcp_rotation);
                        std::array<float, 3> arr = {
                            static_cast<float>(temp_euler.x()),
                            static_cast<float>(temp_euler.y()),
                            static_cast<float>(temp_euler.z())
                        };
                        toolT->tcp_euler = std::make_unique<IPC::Vec3f>(::flatbuffers::span<const float, 3>(arr.data(), 3));
                    }


                    toolT->box_type = src.box_type;
                    {
                        std::array<float, 9> barr;
                        for (int j = 0; j < 9; ++j) barr[j] = src.box_parameter[j];
                        toolT->box_parameter = std::make_unique<IPC::ST_Box_Para>(::flatbuffers::span<const float, 9>(barr.data(), 9));
                    }

                    // configs_0 ~ configs_7에 넣기
                    switch (i) {
                        case 0: tool_list_T.t_configs_0 = std::move(toolT); break;
                        case 1: tool_list_T.t_configs_1 = std::move(toolT); break;
                        case 2: tool_list_T.t_configs_2 = std::move(toolT); break;
                        case 3: tool_list_T.t_configs_3 = std::move(toolT); break;
                        case 4: tool_list_T.t_configs_4 = std::move(toolT); break;
                        case 5: tool_list_T.t_configs_5 = std::move(toolT); break;
                        case 6: tool_list_T.t_configs_6 = std::move(toolT); break;
                        case 7: tool_list_T.t_configs_7 = std::move(toolT); break;
                    }
                }
                return IPC::Response_CallConfigToolList::Pack(fbb, &tool_list_T);
            });
            // -----------------------------------------------------------------------
            // Set Call
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("set_toollist_num", IPC::Request_Set_Tool_List, IPC::Response_Functions, {
                return_int = rb_system::Change_Tool_Number(req->target_tool_num());
            });
            ADD_SERVE_SIMPLE("set_userframe_num", IPC::Request_Set_User_Frame, IPC::Response_Functions, {
                return_int = rb_system::Change_UserFrame_Number(req->user_frame_num());
            });
            ADD_SERVE_SIMPLE("set_shift", IPC::Request_Set_Shift, IPC::Response_Functions, {
                TARGET_INPUT shift_input;
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    shift_input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                shift_input.target_frame  = req->target()->tar_frame();
                shift_input.target_unit   = req->target()->tar_unit();
                return_int = rb_motion::Set_Motion_Shift(req->shift_no(), req->shift_mode(), shift_input);
            });
            ADD_SERVE_SIMPLE("set_out_collision_para", IPC::Request_Set_Out_Collision_Para, IPC::Response_Functions, {
                return_int = rb_system::Set_Out_Coll_Para(req->onoff(), req->react_mode(), req->threshold());
            });
            ADD_SERVE_SIMPLE("set_self_collision_para", IPC::Request_Set_Self_Collision_Para, IPC::Response_Functions, {
                return_int = rb_system::Set_Self_Coll_Para(req->mode(), req->dist_int(), req->dist_ext());
            });
            ADD_SERVE_SIMPLE("set_joint_impedance", IPC::Request_Set_Joint_Impedance, IPC::Response_Functions, {
                int onoff = req->onoff();
                std::array<float, NO_OF_JOINT> stiffness;
                std::array<float, NO_OF_JOINT> torqelimit;
                auto arr_stiffness = req->stiffness()->f();
                auto arr_torqelimit = req->torquelimit()->f();
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    stiffness[i] = arr_stiffness->Get(i);
                    torqelimit[i] = arr_torqelimit->Get(i);
                }
                if(req->onoff() == 1){
                    return_int = rb_system::Set_Joint_Impedance_On(stiffness, torqelimit);
                }else{
                    return_int = rb_system::Set_Joint_Impedance_Off();
                }
            });
            ADD_SERVE_SIMPLE("set_freedrive", IPC::Request_Set_Free_Drive, IPC::Response_Functions, {
                return_int = rb_system::Set_Free_Drive_Mode(req->onoff(), req->sensitivity());
            });
            // -----------------------------------------------------------------------
            // Get Call
            // -----------------------------------------------------------------------
            ADD_SERVE_BLANK("get_core_data", IPC::Request_Get_Core_Data, IPC::Response_Get_Core_Data, {

                int req_option = req->option();
                std::string req_name = req->name()->c_str();

                int return_valid = 0;
                std::array<float, 32> return_arr;
                for(int i=0; i<32; i++){
                    return_arr[i] = 0.0f;
                }

                if(req_name == "RB_JOINT_REF"){
                    for(int i = 0; i < NO_OF_JOINT; i++){
                        return_arr[i] = rb_motion::Get_Wrapper_J()[i];
                    }
                    return_valid = 1;
                }else if(req_name == "RB_JOINT_ENC"){
                    for(int i = 0; i < NO_OF_JOINT; i++){
                        return_arr[i] = rb_system::Get_Motor_Encoder()[i];
                    }
                    return_valid = 1;
                }else if(req_name == "RB_POINT_REF"){
                    for(int i = 0; i< NO_OF_CARTE; i++){
                        return_arr[i] = rb_motion::Get_Wrapper_X()[i];
                    }
                    return_valid = 1;
                }else if(req_name == "RB_POINT_ENC"){
                    for(int i = 0; i< NO_OF_CARTE; i++){
                        return_arr[i] = rb_motion::Get_Wrapper_X()[i];
                    }
                    return_valid = 1;
                }

                IPC::Response_Get_Core_DataT core_data_T;
                core_data_T.valid = return_valid;
                core_data_T.payload = std::make_unique<IPC::FloatArray32>(::flatbuffers::span<const float, 32>(return_arr.data(), 32));
                return IPC::Response_Get_Core_Data::Pack(fbb, &core_data_T);
            });

            // -----------------------------------------------------------------------
            // Move Call
            // -----------------------------------------------------------------------
            ADD_SERVE_SIMPLE("call_smoothjog_j", IPC::Request_Move_SmoothJogJ, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();
                return_int = rb_motion::Start_Motion_SPEED_J(input, 0.5);
            });
            ADD_SERVE_SIMPLE("call_smoothjog_l", IPC::Request_Move_SmoothJogL, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_CARTE; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();
                return_int = rb_motion::Start_Motion_SPEED_L(input, 0.001);
            });
            ADD_SERVE_SIMPLE("call_smoothjog_stop", IPC::Request_Move_SmoothJogStop, IPC::Response_Functions, {
                return_int = rb_system::Call_MoveBreak(req->stoptime());
            });
            ADD_SERVE_SIMPLE("call_tickjog_j", IPC::Request_Move_TickJogJ, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();

                int spd_mode        = req->speed()->spd_mode();
                float vel_para      = req->speed()->spd_vel_para();
                float acc_para      = req->speed()->spd_acc_para();

                auto [return_val, tTar] = rb_motion::Calc_J_Relative(input);
                if(return_val == MSG_OK){
                    return_int = rb_motion::Start_Motion_J(tTar, vel_para, acc_para, spd_mode);
                }else{
                    return_int = return_val;
                }
            });
            ADD_SERVE_SIMPLE("call_tickjog_l", IPC::Request_Move_TickJogL, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_CARTE; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();

                int spd_mode        = req->speed()->spd_mode();
                float vel_para      = req->speed()->spd_vel_para();
                float acc_para      = req->speed()->spd_acc_para();
                auto [return_val, tTar] = rb_motion::Calc_L_Relative(input);
                if(return_val == MSG_OK){
                    return_int = rb_motion::Start_Motion_L(tTar, vel_para, acc_para, spd_mode);
                }else{
                    return_int = return_val;
                }
            });
            ADD_SERVE_SIMPLE("call_move_j", IPC::Request_Move_J, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();

                int spd_mode        = req->speed()->spd_mode();
                float vel_para      = req->speed()->spd_vel_para();
                float acc_para      = req->speed()->spd_acc_para();

                return_int = rb_motion::Start_Motion_J(input, vel_para, acc_para, spd_mode);
            });
            ADD_SERVE_SIMPLE("call_move_l", IPC::Request_Move_L, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_CARTE; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();

                int spd_mode        = req->speed()->spd_mode();
                float vel_para      = req->speed()->spd_vel_para();
                float acc_para      = req->speed()->spd_acc_para();
                return_int = rb_motion::Start_Motion_L(input, vel_para, acc_para, spd_mode);
            });

            ADD_SERVE_SIMPLE("call_move_jb_clr", IPC::Request_Move_JB_CLR, IPC::Response_Functions, {
                return_int = rb_motion::Start_Motion_JB_Clear();
            });
            ADD_SERVE_SIMPLE("call_move_jb_add", IPC::Request_Move_JB_ADD, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_JOINT; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();
                int     spd_mode    = req->speed()->spd_mode();
                float   vel_para    = req->speed()->spd_vel_para();
                float   acc_para    = req->speed()->spd_acc_para();
                int     blend_type  = req->type()->pnt_type();
                float   blend_para  = req->type()->pnt_para();
                return_int = rb_motion::Start_Motion_JB_Add(input, vel_para, acc_para, spd_mode, blend_type, blend_para);
            });
            ADD_SERVE_SIMPLE("call_move_jb_run", IPC::Request_Move_JB_RUN, IPC::Response_Functions, {
                return_int = rb_motion::Start_Motion_JB();
            });
            ADD_SERVE_SIMPLE("call_move_lb_clr", IPC::Request_Move_LB_CLR, IPC::Response_Functions, {
                return_int = rb_motion::Start_Motion_LB_Clear();
            });
            ADD_SERVE_SIMPLE("call_move_lb_add", IPC::Request_Move_LB_ADD, IPC::Response_Functions, {
                TARGET_INPUT input;
                for (int i = 0; i < NO_OF_CARTE; i++) {
                    input.target_value[i] = req->target()->tar_values()->f()->Get(i);
                }
                input.target_frame  = req->target()->tar_frame();
                input.target_unit   = req->target()->tar_unit();
                int spd_mode        = req->speed()->spd_mode();
                float vel_para      = req->speed()->spd_vel_para();
                float acc_para      = req->speed()->spd_acc_para();
                int pt_type         = req->type()->pnt_type();
                float blend_para    = req->type()->pnt_para();
                return_int = rb_motion::Start_Motion_LB_Add(input, vel_para, acc_para, spd_mode, pt_type, blend_para);
            });
            ADD_SERVE_SIMPLE("call_move_lb_run", IPC::Request_Move_LB_RUN, IPC::Response_Functions, {
                return_int = rb_motion::Start_Motion_LB(req->orientation(), 10);
            });
            
            while (true) {
                std::this_thread::sleep_for(1s);
            }
            return nullptr;
        }

        //----------------------------------------------------------------------------------------------
        // TX
        //----------------------------------------------------------------------------------------------
        // bool Function_PowerControl(rb::io::Session& session, const IPC::Request_PowerControlT& req, IPC::Response_FunctionsT* res) {
        //     bool rv = session.CallWith<IPC::Request_PowerControl, IPC::Response_Functions>(
        //         "function_powercontrol",
        //         [&req](flatbuffers::FlatBufferBuilder& fbb) {
        //             return IPC::CreateRequest_PowerControl(fbb);
        //         },
        //         [res](const IPC::Response_Functions* r) { r->UnPackTo(res); }, 100);
        //     return rv;
        // }

        void *thread_ipccomm_tx(void *) {

            auto [s_category, s_model, s_version, s_alias] = rb_system::Get_System_Basic_Info();

            rb::io::SessionOptions options;
            options.backend = rb::io::SessionBackendType::kZenoh;
            options.ns = s_model;
            options.zenoh_config = rb::io::ZenohConfig::FromFile("config.json5");
            auto session = rb::io::Session::Open(std::move(options));

            while(true){
                {// PUB MESSAGE
                    PUB_MESSAGE_ST extract_msg;
                    if(que_publish_message.try_dequeue(extract_msg)){
                        IPC::State_MessageT state_msg;
                        state_msg.type = extract_msg.type;
                        state_msg.code = extract_msg.message_code;
                        state_msg.sub_str = extract_msg.sub_message;
                        session.Publish<IPC::State_Message>("state_message", state_msg);

                        // std::cout<<"Message Extracted ... !! : "<<extract_msg.message_code<<std::endl;
                    }
                }

                {// PUB LOG
                    PUB_LOG_ST extract_log;
                    if(que_publish_log.try_dequeue(extract_log)){
                        IPC::State_LogT log_msg;
                        log_msg.level = extract_log.level;
                        log_msg.timestamp = extract_log.timestamp;
                        log_msg.contents = extract_log.logcontents;
                        session.Publish<IPC::State_Log>("state_log", log_msg);

                        // std::cout<<"Log Extracted ... !! : "<<extract_log.logcontents<<std::endl;
                    }
                }
                
                {// PUB CORE

                    TCP_CONFIG cur_TCP = rb_system::Get_CurrentTcpParameter();
                    Eigen::Vector3d cur_TCP_euler = rb_math::R_to_RPY(cur_TCP.tcp_rotation);

                    IPC::State_CoreT state_coreT;
                    state_coreT.heart_beat  = static_cast<uint8_t>(rb_system::Get_Flag_Heart_Beat());

                    state_coreT.joint_q_ref = std::make_unique<IPC::N_JOINT_f>(rb_motion::Get_Wrapper_J());
                    state_coreT.joint_q_enc = std::make_unique<IPC::N_JOINT_f>(rb_system::Get_Motor_Encoder());
                    state_coreT.joint_t_esti = std::make_unique<IPC::N_JOINT_f>(rb_system::Get_Torque(1));
                    state_coreT.joint_t_meas = std::make_unique<IPC::N_JOINT_f>(rb_system::Get_Torque(0));
                    state_coreT.joint_temper = std::make_unique<IPC::N_JOINT_f>(rb_system::Get_Temperature(1));

                    state_coreT.carte_x_ref = std::make_unique<IPC::N_CARTE_f>(rb_motion::Get_Wrapper_X());
                    state_coreT.carte_x_enc = std::make_unique<IPC::N_CARTE_f>(rb_motion::Get_Wrapper_X());

                    state_coreT.userf_selection_no = rb_system::Get_CurrentUserFrameNumber();
                    state_coreT.userf_x_ref = std::make_unique<IPC::N_CARTE_f>(rb_motion::Get_Wrapper_X_User());

                    state_coreT.tool_selection_no = rb_system::Get_CurrentTcpNumber();
                    state_coreT.tool_name = cur_TCP.tool_name;
                    state_coreT.tool_tcp_x = cur_TCP.tcp_offset(0);
                    state_coreT.tool_tcp_y = cur_TCP.tcp_offset(1);
                    state_coreT.tool_tcp_z = cur_TCP.tcp_offset(2);
                    state_coreT.tool_tcp_rx = cur_TCP_euler(0);
                    state_coreT.tool_tcp_ry = cur_TCP_euler(1);
                    state_coreT.tool_tcp_rz = cur_TCP_euler(2);
                    state_coreT.tool_com_m = cur_TCP.com_mass;
                    state_coreT.tool_com_x = cur_TCP.com_offset(0);
                    state_coreT.tool_com_y = cur_TCP.com_offset(1);
                    state_coreT.tool_com_z = cur_TCP.com_offset(2);

                    state_coreT.cbox_digital_input = std::make_unique<IPC::N_DIN_u>(rb_system::Get_Box_Din());
                    state_coreT.cbox_digital_output = std::make_unique<IPC::N_DOUT_u>(rb_system::Get_Box_Dout());
                    state_coreT.cbox_analog_input = std::make_unique<IPC::N_AIN_f>(rb_system::Get_Box_Ain());
                    state_coreT.cbox_analog_output = std::make_unique<IPC::N_AOUT_f>(rb_system::Get_Box_Aout());

                    state_coreT.ex_digital_input = std::make_unique<IPC::N_DIN_u>(rb_system::Get_EX_Din());
                    state_coreT.ex_digital_output = std::make_unique<IPC::N_DOUT_u>(rb_system::Get_EX_Dout());
                    state_coreT.ex_analog_input = std::make_unique<IPC::N_AIN_f>(rb_system::Get_EX_Ain());
                    state_coreT.ex_analog_output = std::make_unique<IPC::N_AOUT_f>(rb_system::Get_EX_Aout());

                    // state_coreT.tool_digital_input = std::make_unique<IPC::N_DIN_u>(rb_system::Get_Tool_Din());
                    // state_coreT.tool_digital_output = std::make_unique<IPC::N_DOUT_u>(rb_system::Get_Tool_Dout());
                    // state_coreT.tool_analog_input = std::make_unique<IPC::N_DIN_u>(rb_system::Get_Tool_Din());
                    // state_coreT.tool_analog_output = std::make_unique<IPC::N_DOUT_u>(rb_system::Get_Tool_Dout());

                    state_coreT.motion_mode = static_cast<uint8_t>(rb_motion::Get_Motion_Mode());
                    state_coreT.motion_speed_bar = static_cast<float>(rb_system::Get_MoveSpeedBar());
                    state_coreT.motion_is_pause = static_cast<uint8_t>(rb_system::Get_MovePauseState());
                    
                    state_coreT.status_lan2can       = static_cast<uint8_t>(rb_system::Get_Lan2Can_State());
                    state_coreT.status_switch_emg    = static_cast<uint8_t>(rb_system::Get_Power_Switch());
                    state_coreT.status_power_out     = static_cast<uint8_t>(rb_system::Get_Power());
                    state_coreT.status_servo_num     = static_cast<uint8_t>(rb_system::Get_Servo());
                    state_coreT.status_is_refon      = static_cast<uint8_t>(rb_system::Get_ReferenceOnOff());
                    state_coreT.status_out_coll      = static_cast<uint8_t>(rb_system::Get_Flag_Out_Collision_Occur());
                    state_coreT.status_self_coll     = static_cast<uint8_t>(rb_system::Get_Flag_Self_Collision_Occur());
                    state_coreT.status_dt_mode       = static_cast<uint8_t>(rb_system::Get_Flag_Direct_Teaching());

                    session.Publish<IPC::State_Core>("state_core", state_coreT);
                    // std::cout<<"?"<<std::endl;
                }
                std::this_thread::sleep_for(0.05s);

                // {
                //     IPC::Request_PowerControlT req;
                //     req.power_option = 1;
                //     IPC::Response_PowerControlT res;

                //     if(!Function_PowerControl(session, req, &res)){
                //         std::cout<<"[TX] PowerControl RPC Failed"<<std::endl;
                //     }else{
                //         if(res.return_value == 0){
                //             std::cout<<"[TX] Response OK"<<std::endl;
                //         }else{
                //             std::cout<<"[TX] Response has some error ... !!!"<<std::endl;
                //         }
                //     }
                // }
                // std::this_thread::sleep_for(1s);
            }
            return nullptr;
        }

        // class GreeterServiceImpl final : public Greeter::Service {
        //     // .proto 파일에 정의된 SayHello 함수를 오버라이드합니다.
        //     Status SayHello(ServerContext* context, const HelloRequest* request,
        //                     HelloReply* reply) override {
        //         std::string prefix("안녕하세요, ");
        //         // 응답 메시지의 'message' 필드를 설정합니다.
        //         reply->set_message(prefix + request->name() + "님!");
        //         return Status::OK;
        //     }
        // };

        // void *thread_grpccomm_server(void *) {
        //     std::cout<<"Hello gRPC Server"<<std::endl;
        //     std::string server_address("0.0.0.0:50051");
        //     GreeterServiceImpl service;

        //     ServerBuilder builder;
        //     // 서버 주소와 포트를 지정합니다.
        //     builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
        //     // Greeter 서비스를 서버에 등록합니다.
        //     builder.RegisterService(&service);

        //     // 서버를 빌드하고 시작합니다.
        //     std::unique_ptr<Server> server(builder.BuildAndStart());
        //     std::cout << "gRPC 서버가 " << server_address << " 포트에서 실행 중입니다..." << std::endl;

        //     // 서버가 종료될 때까지 대기합니다.
        //     server->Wait();

        //     while(1){
        //         std::this_thread::sleep_for(1s);
        //     }
        //     return nullptr;
        // }
    }

    bool initialize(std::string domain, int th_cpu){
        if(rb_common::thread_create(thread_ipccomm_rx, th_cpu, ("RB_" + domain + "_IPCRX"), hThread_ipccomm_rx, NULL) == 0
            && rb_common::thread_create(thread_ipccomm_tx, th_cpu, ("RB_" + domain + "_IPCTX"), hThread_ipccomm_tx, NULL) == 0
            // && rb_common::thread_create(thread_grpccomm_server, th_cpu, ("RB_" + domain + "_GRPCS"), hThread_grpccomm_server, NULL) == 0
            ){
            return true;
        }else{
            return false;
        }
    }

    void Publish_Message(PUB_MESSAGE_ST t_msg){
        que_publish_message.enqueue(t_msg);
        return;
    }

    void Publish_Log(PUB_LOG_ST t_log){
        que_publish_log.enqueue(t_log);
        return;
    }
}