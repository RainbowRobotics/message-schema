#ifndef IPC_H
#define IPC_H

#include <unistd.h>
#include <string>
#include <array>
#include <dof.h>

namespace rb_ipc {
    struct PUB_MESSAGE_ST{
        int type;
        int message_code;
        std::string sub_message;
    };

    struct PUB_LOG_ST{
        int level;
        std::string timestamp;
        std::string logcontents;
    };


    bool initialize(std::string domain, int th_cpu);
    void Publish_Message(PUB_MESSAGE_ST t_msg);
    void Publish_Log(PUB_LOG_ST t_log);

    void toPyFM_FlowControl(int option);
    int toFriend_ServoJ(std::string friend_domain, std::array<float, NO_OF_JOINT> &target_angles, float t1, float t2, float gain, float filter);
    int toFriend_ServoL(std::string friend_domain, std::array<float, NO_OF_CARTE> &target_cartes, float t1, float t2, float gain, float filter);
    int toFriend_PrintInfo(std::string friend_domain);
}

#endif // IPC_H
