
#define P_NAME  "MAIN"

#include <signal.h>

#include "rb_common/dof.h"
#include "rb_common/common.h"

#include "rb_core/configcall.h"
#include "rb_core/daemon.h"
#include "rb_ipc/ipc.h"
#include "rb_motion/motion.h"

#include "rb_service/shareddata.h"
#include "rb_service/joystick/joystick.h"
#include "rb_service/mbus_server/mbustcp_server.h"
#include "rb_service/socket_server/command_server.h"
#include "rb_service/socket_server/data_server.h"
#include "rb_service/socket_server/udp_command_server.h"
#include "rb_service/socket_server/udp_data_server.h"


extern bool is_save;
void signal_handler(int signum) {
    std::cout << "SIGINT received, cleaning up before exit:: "<< signum << std::endl;
    rb_system::system_destroyer();  // 자원 정리 함수 호출
    rb_shareddata::finalize();
    rb_mbus_server::shutdown();
    exit(0);    // 프로그램 정상 종료
}

int main() {
    
    signal(SIGHUP,  signal_handler);//1
    signal(SIGINT,  signal_handler);//2
    signal(SIGQUIT, signal_handler);//3
    signal(SIGILL,  signal_handler);//4
    signal(SIGABRT, signal_handler);//6
    signal(SIGFPE,  signal_handler);//8
    signal(SIGKILL, signal_handler);//9
    signal(SIGSEGV, signal_handler);//11
    signal(SIGPIPE, signal_handler);//13
    signal(SIGTERM, signal_handler);//15
    
    
    

    std::cout << "PROG PROCESSPID: " << getpid() << std::endl;
    std::cout << "PROG BUILD DATE: " << __DATE__ << std::endl;
    std::cout << "PROG BUILD TIME: " << __TIME__ << std::endl;

    if(!rb_config::initialize("RB_CORE")){
        std::cout<<"Fail to initialize rb_config"<<std::endl;
        return 0;
    }std::this_thread::sleep_for(0.5s);
    std::string SYSTEM_NAME = rb_config::READ_System_Cpu_Domain_Name();

    rb_common::log_initialize(SYSTEM_NAME, "logs");
    LOG_INFO(std::string("System Version: ") + SYSTEM_VERSION);

    int CPU_Class_A = rb_config::READ_System_Cpu_Number(0);
    int CPU_Class_B = rb_config::READ_System_Cpu_Number(1);
    int CPU_Class_C = rb_config::READ_System_Cpu_Number(2);
    int CPU_Class_D = rb_config::READ_System_Cpu_Number(3);

    std::cout << "CPU: "<<CPU_Class_A<<"/"<<CPU_Class_B<<"/"<<CPU_Class_C<<"/"<<CPU_Class_D<<std::endl;

    if(!rb_shareddata::initialize("RB_COBOT_SHM")){
        LOG_ERROR("FAIL to initialize rb_shareddata");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_system::initialize(SYSTEM_NAME, CPU_Class_B, CPU_Class_C)){
        LOG_ERROR("FAIL to initialize rb_system");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_daemon::initialize(SYSTEM_NAME, CPU_Class_A, CPU_Class_A)){
        LOG_ERROR("FAIL to initialize rb_daemon");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_motion::initialize()){
        LOG_ERROR("FAIL to initialize rb_motion");
        return 0;
    }std::this_thread::sleep_for(0.5s);
    
    if(!rb_ipc::initialize(SYSTEM_NAME, CPU_Class_C)){
        LOG_ERROR("FAIL to initialize rb_ipc");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_joystick::initialize(SYSTEM_NAME, CPU_Class_D)){
        LOG_ERROR("FAIL to initialize rb_joystick");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_mbus_server::initialize(SYSTEM_NAME, CPU_Class_D, rb_config::READ_System_Port_Number(0))){
        LOG_ERROR("FAIL to initialize rb_mbus_server");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_socket_command_server::initialize(SYSTEM_NAME, CPU_Class_C, rb_config::READ_System_Port_Number(1))){
        LOG_ERROR("FAIL to initialize rb_scs_server");
        return 0;
    }std::this_thread::sleep_for(0.5s);
    if(!rb_socket_data_server::initialize(SYSTEM_NAME, CPU_Class_C, rb_config::READ_System_Port_Number(2))){
        LOG_ERROR("FAIL to initialize rb_sds_server");
        return 0;
    }std::this_thread::sleep_for(0.5s);
    // if(!rb_udp_command_server::initialize(SYSTEM_NAME, CPU_Class_D, rb_config::READ_System_Port_Number(3))){
    //     LOG_ERROR("FAIL to initialize rb_ucs_server");
    //     return 0;
    // }std::this_thread::sleep_for(0.5s);
    // if(!rb_udp_data_server::initialize(SYSTEM_NAME, CPU_Class_D, rb_config::READ_System_Port_Number(4))){
    //     LOG_ERROR("FAIL to initialize rb_uds_server");
    //     return 0;
    // }std::this_thread::sleep_for(0.5s);

    LOG_INFO("SYSTEM STARTING");

    while(1){
        std::this_thread::sleep_for(3s);
    }

    rb_shareddata::finalize();
    rb_mbus_server::shutdown();
    
    return 0;
}