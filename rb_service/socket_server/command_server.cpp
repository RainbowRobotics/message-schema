#define P_NAME  "COMMAND_SERVER"

#include <arpa/inet.h>
#include <netinet/tcp.h> // TCP_KEEPALIVE 옵션용

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <regex>
#include <atomic>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <charconv>

#include "common.h"
#include "command_server.h"

#include "rb_core/system.h"
#include "rb_motion/motion.h"

struct Command_Return{
    int return_code;
    std::string return_str;
};

#define ADD_CMD_HANDLER(name, ...) \
    { name, [](const std::vector<std::string>& a) -> Command_Return { \
        std::cout << "[SCS] Executing command: " << name << std::endl; \
        __VA_ARGS__; \
    }}

#define MAX_CLIENTS 5
#define MAX_EVENTS 64
#define EPOLL_TIMEOUT_MS 100 // epoll_wait timeout (ms)

namespace rb_socket_command_server {
    namespace {

        template<typename IntT>
        bool parse_int(const std::string& s, IntT& out){
            static_assert(std::is_integral_v<IntT>, "parse_int requires integral type");

            const char* begin = s.data();
            const char* end   = s.data() + s.size();

            auto [ptr, ec] = std::from_chars(begin, end, out);

            return ec == std::errc() && ptr == end;
        }
        bool parse_float(const std::string& s, float& out)
        {
            const char* begin = s.data();
            const char* end   = s.data() + s.size();

            auto [ptr, ec] = std::from_chars(begin, end, out);

            return ec == std::errc() && ptr == end;
        }
        

        std::atomic<bool> running(false);
        int listen_port = 0;
        int event_fd = -1;
        pthread_t server_thread;

        struct ClientInfo {
            int fd;
            std::string ip;
            std::string buffer; // partial recv buffer
        };
        std::unordered_map<int, ClientInfo> clients;

        std::mutex clients_mutex;

        // =============================
        // 🔹 Non-blocking 설정
        // =============================
        static int set_nonblocking(int fd) {
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags == -1) return -1;
            return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        }

        // =============================
        // 🔹 TCP KeepAlive 설정 추가
        // =============================
        void enable_tcp_keepalive(int sockfd) {
            int optval;
            socklen_t optlen = sizeof(optval);

            // 기본 keepalive 활성화
            optval = 1;
            if (setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) == -1) {
                perror("setsockopt(SO_KEEPALIVE)");
            }

#ifdef TCP_KEEPIDLE
            // 10초 동안 트래픽 없으면 keepalive 시작
            optval = 10;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPIDLE)");
            }
#endif

#ifdef TCP_KEEPINTVL
            // 실패한 keepalive 재시도 간격 5초
            optval = 5;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPINTVL)");
            }
#endif

#ifdef TCP_KEEPCNT
            // 최대 3번 시도 후 연결 끊음
            optval = 3;
            if (setsockopt(sockfd, IPPROTO_TCP, TCP_KEEPCNT, &optval, optlen) == -1) {
                perror("setsockopt(TCP_KEEPCNT)");
            }
#endif
        }

        // =============================
        // 🔹 클라이언트 종료 처리
        // =============================
        void close_client(int epfd, int fd) {
            std::cout << "[SCS] Closing client fd: " << fd << std::endl;
            epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
            close(fd);
            std::lock_guard<std::mutex> lk(clients_mutex);
            clients.erase(fd);
        }

        // =============================
        // 🔹 명령 처리
        // =============================
        static void parse_command(const std::string &input, std::string &func_name, std::vector<std::string> &args) {
            func_name.clear();
            args.clear();

            std::string cmd = input;
            cmd.erase(std::remove_if(cmd.begin(), cmd.end(), ::isspace), cmd.end());

            auto open = cmd.find('(');
            auto close = cmd.find(')');
            if (open == std::string::npos || close == std::string::npos || close <= open) {
                func_name = cmd;
                return;
            }

            func_name = cmd.substr(0, open);
            std::string arg_str = cmd.substr(open + 1, close - open - 1);

            std::stringstream ss(arg_str);
            std::string arg;
            while (std::getline(ss, arg, ',')) {
                args.push_back(arg);
            }
        }

        // 🔹 디스패처: 함수명 → 실제 동작
        static Command_Return execute_command(const std::string &func, const std::vector<std::string> &args) {
            static const std::unordered_map<std::string, std::function<Command_Return(const std::vector<std::string>&)>> dispatch = {
                // -----------------------------------------------------------------------
                // BOX
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("save_robot_code", {
                    if (a.size() == 2){
                        return {rb_system::Save_Robot_Code(std::stoi(a[0]), std::stoi(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_side_dout", {
                    if (a.size() == 2){
                        return {rb_system::Set_Box_Digital_Output(std::stoi(a[0]), std::stoi(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_side_aout", {
                    if (a.size() == 2){
                        return {rb_system::Set_Box_Analog_Output(std::stoi(a[0]), std::stof(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_side_dout_toggle", {
                    if (a.size() == 1){
                        return {rb_system::Set_Box_Digital_Output_Toggle(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_side_dout_bitcombination", {
                    if (a.size() == 4){
                        return {rb_system::Set_Box_Digital_Output_Bit(std::stoi(a[0]), std::stoi(a[1]), std::stoi(a[2]), std::stoi(a[3])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_side_dout_pulse", {
                    if (a.size() == 6){
                        return {rb_system::Set_Box_Digital_Output_Pulse(std::stoi(a[0]), std::stoi(a[1]), std::stoi(a[2]), std::stof(a[3]), std::stof(a[4]), std::stof(a[5])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Flage
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("call_flange_power", {
                    if (a.size() == 1){
                        return {rb_system::Set_Flange_Power(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_flange_dout", {
                    if (a.size() == 2){
                        return {rb_system::Set_Flange_Digital_Output(std::stoi(a[0]), std::stoi(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Servo
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("call_joint_brake", {
                    if (a.size() == 2){
                        int bno, option;
                        if(!parse_int(a[0], bno) ||
                            !parse_int(a[1], option)) {
                            return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                        }
                        return {rb_system::Set_Joint_Brake(bno, option), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_joint_encoder_zero", {
                    if (a.size() == 1){
                        int bno;
                        if(!parse_int(a[0], bno)) {
                            return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                        }
                        return {rb_system::Set_Joint_Encoder_Zero(bno), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_powercontrol", {
                    if (a.size() == 1){
                        int input_val = std::stoi(a[0]);
                        int return_val = 0;
                        if(input_val == 1){
                            return_val = rb_system::Set_Power(rb_system::PowerOption::On, false);
                        }else{
                            return_val = rb_system::Set_Power(rb_system::PowerOption::Off, false);
                        }
                        return {return_val, ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_servocontrol", {
                    if (a.size() == 1){
                        return {rb_system::Set_Servo(std::stoi(a[0]), 1), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_referencecontrol", {
                    if (a.size() == 1){
                        return {rb_system::Set_ReferenceOnOff(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),

                // -----------------------------------------------------------------------
                // Flow
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("call_speedbar", {
                    if (a.size() == 1){
                        return {rb_system::Set_MoveSpeedBar(std::stof(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_pause", {
                    if (a.size() == 0){
                        return {rb_system::Call_MovePause(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_resume", {
                    if (a.size() == 0){
                        return {rb_system::Call_MoveResume(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_reset_outcoll", {
                    if (a.size() == 0){
                        return {rb_system::Call_Reset_Out_Coll(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_halt", {
                    if (a.size() == 0){
                        return {rb_system::Call_Halt(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_program_before", {
                    if (a.size() == 1){
                        return {rb_system::Call_Program_Before(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_program_after", {
                    if (a.size() == 1){
                        return {rb_system::Call_Program_After(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Set Call
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("set_toollist_num", {
                    if (a.size() == 1){
                        return {rb_system::Change_Tool_Number(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_userframe_num", {
                    if (a.size() == 1){
                        return {rb_system::Change_UserFrame_Number(std::stoi(a[0])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_shift", {
                    if (a.size() == (NO_OF_JOINT + 4)){
                        int shift_no = std::stoi(a[0]);
                        int shift_mode = std::stoi(a[1]);
                        TARGET_INPUT shift_input;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            shift_input.target_value[i] = std::stof(a[i + 2]);
                        }
                        shift_input.target_frame  = std::stoi(a[a.size() - 2]);
                        shift_input.target_unit   = std::stoi(a[a.size() - 1]);
                        return {rb_motion::Set_Motion_Shift(shift_no, shift_mode, shift_input), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_out_collision_para", {
                    if (a.size() == 3){
                        return {rb_system::Set_Out_Coll_Para(std::stoi(a[0]), std::stoi(a[1]), std::stof(a[2])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_self_collision_para", {
                    if (a.size() == 3){
                        return {rb_system::Set_Self_Coll_Para(std::stoi(a[0]), std::stof(a[1]), std::stof(a[2])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_joint_impedance", {
                    if (a.size() == (1 + NO_OF_JOINT * 2)){
                        int onoff = std::stoi(a[0]);
                        std::array<float, NO_OF_JOINT> stiffness;
                        std::array<float, NO_OF_JOINT> torqelimit;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            stiffness[i] = std::stof(a[i + 1]);
                            torqelimit[i] = std::stof(a[i + 1 + NO_OF_JOINT]);
                        }
                        if(onoff == 1){
                            return {rb_system::Set_Joint_Impedance_On(stiffness, torqelimit), ""};
                        }else{
                            return {rb_system::Set_Joint_Impedance_Off(), ""};
                        }
                    }else if(a.size() == 3){
                        int onoff = std::stoi(a[0]);
                        std::array<float, NO_OF_JOINT> arr_stiffness;
                        std::array<float, NO_OF_JOINT> arr_torqelimit;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            arr_stiffness[i] = std::stof(a[1]);
                            arr_torqelimit[i] = std::stof(a[2]);
                        }
                        if(onoff == 1){
                            return {rb_system::Set_Joint_Impedance_On(arr_stiffness, arr_torqelimit), ""};
                        }else{
                            return {rb_system::Set_Joint_Impedance_Off(), ""};
                        }
                    }else{
                        return {rb_system::Set_Joint_Impedance_Off(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_freedrive", {
                    if (a.size() == 1 || a.size() == 2){
                        int input_onoff = std::stoi(a[0]);
                        float input_sensitivity = 1.0f;
                        if(a.size() == 2){
                            input_sensitivity = std::stof(a[1]);
                        }
                        return {rb_system::Set_Free_Drive_Mode(input_onoff, input_sensitivity), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_userframe_6dof", {
                    if(a.size() == 8){
                        return {rb_system::Set_UserFrame_6DOF(std::stoi(a[0]), std::stoi(a[1])
                            , std::stof(a[2]), std::stof(a[3]), std::stof(a[4]), std::stof(a[5]), std::stof(a[6]), std::stof(a[7])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_userframe_tcp", {
                    if(a.size() == 2){
                        return {rb_system::Set_UserFrame_TCP(std::stoi(a[0]), std::stoi(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("set_userframe_3points", {
                    if(a.size() == 12){
                        return {rb_system::Set_UserFrame_3Points(std::stoi(a[0]), std::stoi(a[1]), std::stoi(a[2])
                            , std::stof(a[3]), std::stof(a[4]), std::stof(a[5])
                            , std::stof(a[6]), std::stof(a[7]), std::stof(a[8])
                            , std::stof(a[9]), std::stof(a[10]), std::stof(a[11])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Get Call
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("get_core_data", {
                    if (a.size() == 1 || a.size() == 2){
                        std::string input_name = "";
                        int input_option = 0;
                        if(a.size() == 1){
                            input_name = a[0];
                        }else{
                            input_option = std::stoi(a[0]);
                            input_name = a[1];
                        }
                        GET_SYSTEM_DATA_RET sys_ret = rb_system::Get_System_Data(input_option, input_name);
                        if(sys_ret.type != GET_SYS_DATA_NO_EXIST){
                            // 성공
                            std::string data_str = "";
                            if(sys_ret.type == GET_SYS_DATA_NUMBER){
                                data_str = std::to_string(sys_ret.payload_num);
                            }else if(sys_ret.type == GET_SYS_DATA_ARRAY){
                                for(int i = 0; i < sys_ret.payload_arr_length; ++i){
                                    data_str += std::to_string(sys_ret.payload_arr[i]);;
                                    if(i != (sys_ret.payload_arr_length -1)){
                                        data_str += ",";
                                    }
                                }
                            }else if(sys_ret.type == GET_SYS_DATA_STRING){
                                data_str = sys_ret.payload_str;
                            }
                            
                            return {-1, data_str}; // 성공 코드
                        }else{
                            // 실패
                            return {-1, "UNKNOWN"}; // 실패 코드
                        }
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Move Call
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("call_move_j", {
                    if (a.size() == (NO_OF_JOINT + 4)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            input.target_value[i] = std::stof(a[i]);
                        }
                        input.target_frame  = std::stoi(a[NO_OF_JOINT + 0]);
                        input.target_unit   = 0;

                        int spd_mode        = std::stoi(a[NO_OF_JOINT + 1]);
                        float vel_para      = std::stof(a[NO_OF_JOINT + 2]);
                        float acc_para      = std::stof(a[NO_OF_JOINT + 3]);

                        return {rb_motion::Start_Motion_J(input, vel_para, acc_para, spd_mode), ""};
                    }else if(a.size() == (NO_OF_JOINT + 2)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            input.target_value[i] = std::stof(a[i]);
                        }
                        input.target_frame  = FRAME_JOINT;
                        input.target_unit   = 0;

                        int spd_mode = 0;
                        float vel_para = std::stof(a[NO_OF_JOINT]);
                        float acc_para = std::stof(a[NO_OF_JOINT + 1]);
                        return {rb_motion::Start_Motion_J(input, vel_para, acc_para, spd_mode), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_move_l", {
                    if (a.size() == (NO_OF_CARTE + 4)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_CARTE; i++) {
                            input.target_value[i] = std::stof(a[i]);
                        }
                        input.target_frame  = std::stoi(a[NO_OF_CARTE + 0]);
                        input.target_unit   = 0;

                        int spd_mode        = std::stoi(a[NO_OF_CARTE + 1]);
                        float vel_para      = std::stof(a[NO_OF_CARTE + 2]);
                        float acc_para      = std::stof(a[NO_OF_CARTE + 3]);

                        return {rb_motion::Start_Motion_L(input, vel_para, acc_para, spd_mode), ""};
                    }else if(a.size() == (NO_OF_CARTE + 2)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_CARTE; i++) {
                            input.target_value[i] = std::stof(a[i]);
                        }
                        input.target_frame  = FRAME_GLOBAL;
                        input.target_unit   = 0;

                        int spd_mode = 0;
                        float vel_para = std::stof(a[NO_OF_CARTE]);
                        float acc_para = std::stof(a[NO_OF_CARTE + 1]);
                        return {rb_motion::Start_Motion_L(input, vel_para, acc_para, spd_mode), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_servo_j", {
                    if (a.size() == (NO_OF_JOINT + 4)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_JOINT; i++) {
                            if (!parse_float(a[i], input.target_value[i])) {
                                // std:::cout<<"Invalid joint value at index " + std::to_string(i)<<std:::endl;
                                return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                            }
                        }
                        input.target_frame  = FRAME_JOINT;
                        input.target_unit   = 0;
                        float t1, t2, gain, filter;
                        if (!parse_float(a[NO_OF_JOINT + 0], t1) ||
                            !parse_float(a[NO_OF_JOINT + 1], t2) ||
                            !parse_float(a[NO_OF_JOINT + 2], gain) ||
                            !parse_float(a[NO_OF_JOINT + 3], filter)) {
                            return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                        }
                        return {rb_motion::Start_Motion_SERVO_J(input, t1, t2, gain, filter), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_servo_l", {
                    if (a.size() == (NO_OF_CARTE + 4)){
                        TARGET_INPUT input;
                        for (size_t i = 0; i < NO_OF_CARTE; i++) {
                            if (!parse_float(a[i], input.target_value[i])) {
                                // std:::cout<<"Invalid joint value at index " + std::to_string(i)<<std:::endl;
                                return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                            }
                        }
                        input.target_frame  = FRAME_GLOBAL;
                        input.target_unit   = 0;
                        float t1, t2, gain, filter;
                        if (!parse_float(a[NO_OF_JOINT + 0], t1) ||
                            !parse_float(a[NO_OF_JOINT + 1], t2) ||
                            !parse_float(a[NO_OF_JOINT + 2], gain) ||
                            !parse_float(a[NO_OF_JOINT + 3], filter)) {
                            return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                        }
                        return {rb_motion::Start_Motion_SERVO_L(input, t1, t2, gain, filter), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_tcp_weaving_on", {
                    if(a.size() == 2){
                        return {rb_motion::Start_Wrapper_Tcp_Weaving(std::stof(a[0]), std::stof(a[1])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_tcp_weaving_off", {
                    if(a.size() == 0){
                        return {rb_motion::Stop_Wrapper_Tcp_Weaving(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_base_conveyor_on", {
                    if(a.size() == 3){
                        return {rb_motion::Start_Wrapper_Base_Conv(std::stof(a[0]), std::stof(a[1]), std::stof(a[2])), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                ADD_CMD_HANDLER("call_base_conveyor_off", {
                    if(a.size() == 0){
                        return {rb_motion::Stop_Wrapper_Base_Conv(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }),
                // -----------------------------------------------------------------------
                // Just for Testing
                // -----------------------------------------------------------------------
                ADD_CMD_HANDLER("testtesttest", {
                    if (a.size() == 0){
                        return {rb_system::TestTestTest(), ""};
                    }
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                })
            };

            auto it = dispatch.find(func);
            if (it != dispatch.end()) {
                // std::cout<<"[SCS] Found command: " << func << "\n";
                try {
                    return it->second(args);
                } catch (const std::exception &e) {
                    std::cerr << "[SCS] Exception while executing '" << func << "': " << e.what() << "\n";
                    return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
                }
            } else {
                std::cerr << "[SCS] Unknown command: " << func << "\n";
                return {MSG_NOT_VALID_COMMAND_FORMAT, ""};
            }
        }
        void parse_and_execute(const std::string &cmd, int client_fd) {
            std::string func;
            std::vector<std::string> args;
            parse_command(cmd, func, args);

            Command_Return ret = execute_command(func, args);

            std::string response = "";
            if(ret.return_code >= 0){
                response = "return[SCS][" + std::to_string(ret.return_code) + "]\n";
            }else{
                response = "return[SCS][" + ret.return_str + "]\n";
            }

            send(client_fd, response.c_str(), response.size(), 0);
        }

        // =============================
        // 🔹 클라이언트 데이터 처리
        // =============================
        void handle_client_data(int epfd, int fd) {
            char buf[512];
            int n = read(fd, buf, sizeof(buf));
            if (n <= 0) {
                if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK)) {
                    std::cout << "[SCS] Client disconnected (FD=" << fd << ")" << std::endl;
                    close_client(epfd, fd);
                }
                return;
            }

            std::lock_guard<std::mutex> lk(clients_mutex);
            auto &info = clients[fd];
            info.buffer.append(buf, n);

            size_t start = 0;
            int paren_level = 0;

            for (size_t i = 0; i < info.buffer.size(); ++i) {
                char c = info.buffer[i];

                // 공백, 탭 무시
                if (c == ' ' || c == '\t') continue;

                // 괄호 레벨 계산
                if (c == '(') paren_level++;
                else if (c == ')') paren_level--;

                // 명령 종료 조건: 괄호가 닫히거나 종료 문자 발견
                bool end_of_command = false;
                if (paren_level == 0) {
                    if (c == ')' || c == '\n' || c == '\r' || c == '\0') {
                        end_of_command = true;
                    }
                }

                if (end_of_command) {
                    // 명령어 추출
                    std::string cmd = info.buffer.substr(start, i - start + 1);

                    // 앞뒤 공백, 개행, 널 제거
                    size_t first = cmd.find_first_not_of(" \r\n\t\0");
                    size_t last = cmd.find_last_not_of(" \r\n\t\0");
                    if (first != std::string::npos && last != std::string::npos)
                        cmd = cmd.substr(first, last - first + 1);
                    else
                        cmd.clear(); // 전부 공백이면 빈 문자열

                    if (!cmd.empty()) {
                        std::cout << "[SCS] Executing raw: " << cmd << std::endl;
                        parse_and_execute(cmd, fd);
                    }

                    start = i + 1; // 다음 명령 시작 위치
                }
            }

            // 처리된 부분은 버퍼에서 제거
            if (start > 0) {
                info.buffer.erase(0, start);
            }
        }





        // =============================
        // 🔹 서버 스레드 본체
        // =============================
        void *thread_asciiserver(void *) {
            int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
            if (listen_fd < 0) {
                perror("socket");
                return nullptr;
            }

            int opt = 1;
            setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

            sockaddr_in addr{};
            addr.sin_family = AF_INET;
            addr.sin_addr.s_addr = INADDR_ANY;
            addr.sin_port = htons(listen_port);

            if (bind(listen_fd, (sockaddr *)&addr, sizeof(addr)) < 0) {
                perror("bind");
                close(listen_fd);
                return nullptr;
            }

            if (listen(listen_fd, MAX_CLIENTS) < 0) {
                perror("listen");
                close(listen_fd);
                return nullptr;
            }

            set_nonblocking(listen_fd);
            event_fd = eventfd(0, EFD_NONBLOCK);

            int epfd = epoll_create1(0);
            if (epfd == -1) {
                perror("epoll_create1");
                close(listen_fd);
                return nullptr;
            }

            // epoll 등록
            {
                epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = listen_fd;
                epoll_ctl(epfd, EPOLL_CTL_ADD, listen_fd, &ev);
            }
            {
                epoll_event ev{};
                ev.events = EPOLLIN;
                ev.data.fd = event_fd;
                epoll_ctl(epfd, EPOLL_CTL_ADD, event_fd, &ev);
            }

            std::vector<epoll_event> events(MAX_EVENTS);

            std::cout << "[SCS] TCP Server started on port " << listen_port << std::endl;

            // ----------------------------
            // 메인 루프
            // ----------------------------
            while (running.load()) {
                int nfds = epoll_wait(epfd, events.data(), MAX_EVENTS, EPOLL_TIMEOUT_MS);
                if (nfds == -1) {
                    if (errno == EINTR) continue;
                    perror("epoll_wait");
                    break;
                }

                for (int i = 0; i < nfds; ++i) {
                    int fd = events[i].data.fd;
                    uint32_t evt = events[i].events;

                    // eventfd → 종료 이벤트
                    if (fd == event_fd) {
                        uint64_t v;
                        read(event_fd, &v, sizeof(v));
                        running.store(false);
                        break;
                    }

                    // 새 클라이언트 연결
                    if (fd == listen_fd) {
                        while (true) {
                            sockaddr_in caddr{};
                            socklen_t clen = sizeof(caddr);
                            int cfd = accept(listen_fd, (sockaddr *)&caddr, &clen);
                            if (cfd == -1) {
                                if (errno == EAGAIN || errno == EWOULDBLOCK)
                                    break;
                                perror("accept");
                                break;
                            }

                            set_nonblocking(cfd);
                            enable_tcp_keepalive(cfd); // ✅ KeepAlive 추가

                            char ipbuf[INET_ADDRSTRLEN];
                            inet_ntop(AF_INET, &caddr.sin_addr, ipbuf, sizeof(ipbuf));

                            epoll_event cev{};
                            cev.events = EPOLLIN | EPOLLRDHUP;
                            cev.data.fd = cfd;
                            epoll_ctl(epfd, EPOLL_CTL_ADD, cfd, &cev);

                            {
                                std::lock_guard<std::mutex> lk(clients_mutex);
                                clients.emplace(cfd, ClientInfo{cfd, ipbuf, ""});
                            }

                            std::cout << "[SCS] Client connected: " << ipbuf << " FD=" << cfd << std::endl;
                        }
                        continue;
                    }

                    // 클라이언트 종료 이벤트
                    if (evt & (EPOLLERR | EPOLLRDHUP | EPOLLHUP)) {
                        close_client(epfd, fd);
                        continue;
                    }

                    // 데이터 수신
                    if (evt & EPOLLIN) {
                        // std::cout<<"handle_client_data called"<<std::endl;
                        handle_client_data(epfd, fd);
                    }
                }
            }

            // ----------------------------
            // 종료 처리
            // ----------------------------
            std::cout << "[SCS] Shutting down..." << std::endl;
            {
                std::lock_guard<std::mutex> lk(clients_mutex);
                for (auto &p : clients) {
                    epoll_ctl(epfd, EPOLL_CTL_DEL, p.first, nullptr);
                    close(p.first);
                }
                clients.clear();
            }

            close(event_fd);
            close(listen_fd);
            close(epfd);

            std::cout << "[SCS] Server stopped." << std::endl;
            return nullptr;
        } // thread_asciiserver
    } // namespace

    // =============================
    // 🔹 서버 초기화/종료 API
    // =============================
    bool initialize(std::string domain, int th_cpu, int port_no) {
        running.store(true);
        listen_port = port_no;
        if (rb_common::thread_create(thread_asciiserver, th_cpu, ("RB_" + domain + "_SCS"), server_thread, NULL) != 0) {
            running.store(false);
            return false;
        }
        return true;
    }

    void shutdown() {
        running.store(false);
        if (event_fd != -1) {
            uint64_t v = 1;
            write(event_fd, &v, sizeof(v));
        }
        pthread_join(server_thread, nullptr);
    }

    void broadcast(const std::string& message) {
        std::lock_guard<std::mutex> lk(clients_mutex);

        if (clients.empty()) {
            std::cout << "[SCS] Broadcast skipped (no clients)" << std::endl;
            return;
        }

        std::string msg = message;
        if (msg.back() != '\n') msg += "\n";  // 클라이언트에서 읽기 편하게 개행 추가

        for (auto it = clients.begin(); it != clients.end();) {
            int fd = it->first;
            ssize_t sent = send(fd, msg.c_str(), msg.size(), MSG_NOSIGNAL);
            if (sent == -1) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    std::cerr << "[SCS] Removing dead client FD=" << fd << std::endl;
                    close(fd);
                    it = clients.erase(it);
                    continue;
                }
            }
            ++it;
        }

        std::cout << "[SCS] Broadcasted to " << clients.size() << " clients: " << message << std::endl;
    }
} // namespace rb_socket_command_server
