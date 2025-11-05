#define P_NAME  "DAEMON"

#include <alchemy/task.h>


#include "../rb_common/common.h"
#include "zenoh.h"
#include "dof.h"
#include "daemon.h"
#include "lan2can.h"
#include "system.h"


extern lan2can *_gv_Handler_Lan;

namespace rb_daemon {
    namespace {
        static RT_TASK  rtTaskCon;

        void *thread_readLan(void *){
            rb_common::log_push(LogLevel::Info, "Succeed to start read thread", P_NAME);
            while(1){
                if(_gv_Handler_Lan != NULL){
                    _gv_Handler_Lan->LAN_readData();
                }
                std::this_thread::sleep_for(50us);
            }
            return NULL;
        }

        void thread_realtime(void *arg){
            (void)arg;
            rt_task_set_periodic(NULL, TM_NOW, RT_PERIOD_MS * 1000000);

            pthread_t hThread;
            rb_common::thread_create(thread_readLan, 1, "RB_READ", hThread, NULL);

            while(1){
                rt_task_wait_period(NULL);

                rb_system::Task_RealTime();
            }
        }
    }
    bool initialize(std::string domain){
        std::string th_name = "RB_" + domain + "_DAEMON";
        if(rt_task_create(&rtTaskCon, th_name.c_str(), 0, 99, 0) == 0){
            cpu_set_t aCPU;
            CPU_ZERO(&aCPU);
            CPU_SET(0, &aCPU);
            if(rt_task_set_affinity(&rtTaskCon, &aCPU) != 0){
                rb_common::log_push(LogLevel::Error, "Fail to set CPU affinity real-time thread", P_NAME);
                return false;
            }
            if(rt_task_start(&rtTaskCon, &thread_realtime, NULL) == 0){
                rb_common::log_push(LogLevel::Info, "Succeed to start real-time thread", P_NAME);
            }else{
                rb_common::log_push(LogLevel::Error, "Fail to start real-time thread", P_NAME);
                return false;
            }
        }
        return true;
    }   
}

