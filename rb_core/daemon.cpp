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
            RTIME prev_time = 0;
            RTIME max_abs_jitter = 0;
            RTIME period_ns = RT_PERIOD_MS * 1000000LL;
            bool first = true;
            static int cnt = 0;

            rt_task_set_periodic(NULL, TM_NOW, period_ns);

            pthread_t hThread;
            rb_common::thread_create(thread_readLan, 1, "RB_READ", hThread, NULL);

            while(1){
                rt_task_wait_period(NULL);

                RTIME now = rt_timer_read();
                if(first){
                    prev_time = now - period_ns;
                    first = false;
                }
                RTIME jitter = now - prev_time - period_ns;      // 지터 = 실제 - 목표
                prev_time = now;

                RTIME abs_jitter = llabs(jitter);
                if(abs_jitter > max_abs_jitter)
                    max_abs_jitter = abs_jitter;

                if(cnt % 1000 == 0){
                    // 전역 변수에 복사만!
                    rb_system::Update_Zitter_Measurement((int64_t)max_abs_jitter);
                    max_abs_jitter = 0;
                }
                cnt++;

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

