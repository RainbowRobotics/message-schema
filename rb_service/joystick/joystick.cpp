#define P_NAME  "joystick"


#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <stdio.h>
#include <iostream>

#include "joystick.h"
#include "common.h"

#include "rb_core/system.h"
#include "rb_motion/motion.h"

#define     JOY_NUM_OF_AXIS         8
#define     JOY_NUM_OF_BUTTON       12
#define     JOY_TH                  10000

enum JOYSTICK_DATA_ENUM{
    MAP_A = 0,
    MAP_B,
    MAP_X,
    MAP_Y,
    MAP_LB,
    MAP_RB,
    MAP_BACK,
    MAP_START,
    MAP_LOGITECH,
    MAP_LJOG,
    MAP_RJOG,
    MAP_NONE,

    MAP_LJOG_LR,//a0
    MAP_LJOG_UD,//a1
    MAP_LT,//a2

    MAP_RJOG_LR,//a3
    MAP_RJOG_UD,//a4
    MAP_RT,//a5

    MAP_ARW_LR,//a6
    MAP_ARW_UD,//a7

    MAP_NUMBER
};

namespace rb_joystick {
    namespace {
        pthread_t           hThread_joystick;
        int                 JoyFD;
        struct js_event     JoyEvent;
        float               JoyAxis[JOY_NUM_OF_AXIS];
        int                 JoyButton[JOY_NUM_OF_BUTTON];

        int                 JoyKey[MAP_NUMBER];
        int                 JoyKeyPrev[MAP_NUMBER];

        bool                flag_tickmode = false;

        bool                flag_stream_speed_L = false;

        void *thread_joystick(void *) {
            while (true) {
                if(sizeof(struct js_event) == read(JoyFD, &(JoyEvent), sizeof(struct js_event))){
                    switch(JoyEvent.type & ~JS_EVENT_INIT){
                        case JS_EVENT_AXIS:
                            if(JoyEvent.number < JOY_NUM_OF_AXIS)
                                JoyAxis[JoyEvent.number] = JoyEvent.value;
                            break;
                        case JS_EVENT_BUTTON:
                            if(JoyEvent.number < JOY_NUM_OF_BUTTON)
                                JoyButton[JoyEvent.number] = JoyEvent.value;
                            break;
                        default:
                            break;
                    }

                    for(int i=0; i<JOY_NUM_OF_BUTTON; i++)
                        JoyKey[i] = JoyButton[i];

                    if(JoyAxis[0] < -JOY_TH)
                        JoyKey[MAP_LJOG_LR] = 1;
                    else if(JoyAxis[0] > JOY_TH)
                        JoyKey[MAP_LJOG_LR] = -1;
                    else
                        JoyKey[MAP_LJOG_LR] = 0;

                    if(JoyAxis[1] < -27000)
                        JoyKey[MAP_LJOG_UD] = 1;
                    else if(JoyAxis[1] > 27000)
                        JoyKey[MAP_LJOG_UD] = -1;
                    else
                        JoyKey[MAP_LJOG_UD] = 0;

                    if(JoyAxis[2] > JOY_TH)
                        JoyKey[MAP_LT] = 1;
                    else
                        JoyKey[MAP_LT] = 0;

                    if(JoyAxis[3] < -JOY_TH)
                        JoyKey[MAP_RJOG_LR] = 1;
                    else if(JoyAxis[3] > JOY_TH)
                        JoyKey[MAP_RJOG_LR] = -1;
                    else
                        JoyKey[MAP_RJOG_LR] = 0;

                    if(JoyAxis[4] < -27000)
                        JoyKey[MAP_RJOG_UD] = 1;
                    else if(JoyAxis[4] > 27000)
                        JoyKey[MAP_RJOG_UD] = -1;
                    else
                        JoyKey[MAP_RJOG_UD] = 0;

                    if(JoyAxis[5] > JOY_TH)
                        JoyKey[MAP_RT] = 1;
                    else
                        JoyKey[MAP_RT] = 0;



                    if(JoyAxis[6] < -JOY_TH)
                        JoyKey[MAP_ARW_LR] = 1;
                    else if(JoyAxis[6] > JOY_TH)
                        JoyKey[MAP_ARW_LR] = -1;
                    else
                        JoyKey[MAP_ARW_LR] = 0;

                    if(JoyAxis[7] < -JOY_TH)
                        JoyKey[MAP_ARW_UD] = 1;
                    else if(JoyAxis[7] > JOY_TH)
                        JoyKey[MAP_ARW_UD] = -1;
                    else
                        JoyKey[MAP_ARW_UD] = 0;

                    for(int i = 0; i < JOY_NUM_OF_AXIS; ++i){
                        float temp_value = JoyAxis[i];
                        temp_value = rb_math::sign(temp_value) * rb_math::filt_Line(fabs(temp_value), 2000, 32000, 0, 1);
                        JoyAxis[i] = temp_value;
                    }
                }
                // --------------------------------------------------
                if(JoyKeyPrev[MAP_LOGITECH] == 1 && JoyKey[MAP_LOGITECH] == 0){
                    if(rb_system::Get_Is_Idle()){
                        rb_system::Set_Servo(1, 0);
                    }
                }

                if(JoyKey[MAP_RT] == 0 && JoyKeyPrev[MAP_RT] == 1){
                    if(!flag_tickmode)      std::cout<<"Joystick Mode: Normal -> Tick"<<std::endl;
                    if(flag_tickmode)       std::cout<<"Joystick Mode: Tick -> Normal"<<std::endl;
                    flag_tickmode ^= true;
                    flag_stream_speed_L = false;
                }

                if(JoyKey[MAP_A] == 1 && JoyKeyPrev[MAP_A] == 0){
                    if(rb_system::Get_Is_Idle()){
                        flag_stream_speed_L = true;
                    }
                }else if(JoyKey[MAP_A] == 0 && JoyKeyPrev[MAP_A] == 1){
                    flag_stream_speed_L = false;
                    rb_system::Call_MoveBreak(0.2);
                }

                if(flag_tickmode){
                    float tick_value[3] = {0, 0, 0};
                    bool shot_tick = false;
                    if(JoyKey[MAP_LJOG_UD] == 0 && JoyKeyPrev[MAP_LJOG_UD] != 0){
                        tick_value[0] = JoyKeyPrev[MAP_LJOG_UD];
                        shot_tick = true;
                    }else if(JoyKey[MAP_LJOG_LR] == 0 && JoyKeyPrev[MAP_LJOG_LR] != 0){
                        tick_value[1] = JoyKeyPrev[MAP_LJOG_LR];
                        shot_tick = true;
                    }else if(JoyKey[MAP_RJOG_UD] == 0 && JoyKeyPrev[MAP_RJOG_UD] != 0){
                        tick_value[2] = JoyKeyPrev[MAP_RJOG_UD];
                        shot_tick = true;
                    }

                    if(shot_tick && rb_system::Get_Is_Idle()){
                        std::cout<<"TICK: "<<tick_value[0]<<" / "<<tick_value[1]<<" / "<<tick_value[2]<<std::endl;
                        TARGET_INPUT delta = rb_math::Make_Input_Zero(FRAME_GLOBAL);
                        delta.target_value[0] = tick_value[0] * 20.;
                        delta.target_value[1] = tick_value[1] * 20.;
                        delta.target_value[2] = tick_value[2] * 20.;

                        auto [tFlag, tTar] = rb_motion::Calc_L_Relative(delta);
                        if(tFlag){
                            rb_motion::Start_Motion_L(tTar, 200, 2000, 1);
                        }
                    }
                }else{
                    if(flag_stream_speed_L){
                        TARGET_INPUT input = rb_math::Make_Input_Zero(FRAME_GLOBAL);
                        input.target_value[0] = JoyAxis[1] * -150.;
                        input.target_value[1] = JoyAxis[0] * -150.;
                        input.target_value[2] = JoyAxis[4] * -150.;
                        rb_motion::Start_Motion_SPEED_L(input, 0.004);
                    }
                }
                // --------------------------------------------------
                for(int i = 0; i < MAP_NUMBER; ++i){
                    JoyKeyPrev[i] = JoyKey[i];
                }
                std::this_thread::sleep_for(0.03s);
            }
            return nullptr;
        }
    }
    

    bool initialize(std::string domain, int th_cpu){
        if(connect()){
            if(rb_common::thread_create(thread_joystick, th_cpu, ("RB_" + domain + "_JOY"), hThread_joystick, NULL) != 0){
                return false;
            }
        }
        return true;
    }

    bool connect(){
        for(int i = 0; i < JOY_NUM_OF_AXIS; ++i){
            JoyAxis[i] = 0;
        }
        for(int i = 0; i < JOY_NUM_OF_BUTTON; ++i){
            JoyButton[i] = 0;
        }
        for(int i = 0; i < MAP_NUMBER; ++i){
            JoyKeyPrev[i] = 0;
            JoyKey[i] = 0;
        }

        if((JoyFD = open("/dev/input/js0", O_RDONLY)) == -1){
            return false;
        }
        
        int version;
        unsigned int	numAxis;
        unsigned int	numButton;
        char nameJoy[80];

        ioctl(JoyFD, JSIOCGVERSION, &version);
        ioctl(JoyFD, JSIOCGAXES, &numAxis);
        ioctl(JoyFD, JSIOCGBUTTONS, &numButton);
        ioctl(JoyFD, JSIOCGNAME(80), &nameJoy);

        // std::cout << "Joy Version: " << version<<std::endl;
        // std::cout << "Joy Connect: " << nameJoy << "(" << numAxis << ", " << numButton << ")"<<std::endl;

        fcntl(JoyFD, F_SETFL, O_NONBLOCK);	// use non-blocking methods
        return true;
    }

    bool disconnect(){
        if(JoyFD > 0){
            std::cout<<"Joy_Disconnect"<<std::endl;
            close(JoyFD);
        }
        return true;
    }

    bool refresh(){
        disconnect();
        return connect();
    }


}