
#define P_NAME  "MAIN"

#include "rb_common/dof.h"
#include "rb_common/common.h"

#include "rb_core/configcall.h"
#include "rb_core/daemon.h"
#include "rb_ipc/ipc.h"
#include "rb_motion/motion.h"

#include "rb_service/shareddata.h"
#include "rb_service/joystick/joystick.h"
#include "rb_service/mbus_server/mbustcp_server.h"

extern bool is_save;
int main() {
    std::cout << "PID: " << getpid() << std::endl;
    rb_common::log_initialize(SYSTEM_NAME, "logs");
    LOG_INFO(std::string("System Version: ") + SYSTEM_VERSION);
   
    if(!rb_config::initialize("RB_CORE")){
        LOG_ERROR("FAIL to initialize rb_config");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_shareddata::initialize(SYSTEM_NAME)){
        LOG_ERROR("FAIL to initialize rb_shareddata");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_system::initialize(SYSTEM_NAME, 2)){
        LOG_ERROR("FAIL to initialize rb_system");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_daemon::initialize(SYSTEM_NAME)){
        LOG_ERROR("FAIL to initialize rb_daemon");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_motion::initialize()){
        LOG_ERROR("FAIL to initialize rb_motion");
        return 0;
    }std::this_thread::sleep_for(0.5s);
    
    if(!rb_ipc::initialize(SYSTEM_NAME, 3)){
        LOG_ERROR("FAIL to initialize rb_ipc");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_joystick::initialize(SYSTEM_NAME, 3)){
        LOG_ERROR("FAIL to initialize rb_joystick");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    if(!rb_mbus_server::initialize(SYSTEM_NAME, 3, 502)){
        LOG_ERROR("FAIL to initialize rb_mbus_server");
        return 0;
    }std::this_thread::sleep_for(0.5s);

    LOG_INFO("SYSTEM STARTING");

    while(1){
        std::this_thread::sleep_for(3s);
    }

    rb_shareddata::finalize();
    rb_mbus_server::shutdown();
    
    return 0;
}