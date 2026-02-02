#ifndef LAN2CAN_H
#define LAN2CAN_H

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <mutex>
#include <vector>
#include <deque>
#include <functional>

#include "canobserver.h"



#define RX_DATA_SIZE    8000

#define CAN_MAX_NUM         200
#define HAND_SHAKE_LAN_CNT  100

struct CAN_MSG{
    unsigned int    id;         // Identifier
    unsigned char   data[8];    // Data
    unsigned char   dlc;        // Data Length Code
    unsigned char   status;     // MB status
    unsigned char   channel;    // CAN channel (0, 1, 2, ...)
};

class lan2can
{
public:
    lan2can(int port, int ip_0, int ip_1, int ip_2, int ip_3, int th_cpu_connection, std::string domain);
    ~lan2can();

    bool                    LAN_connectionStatus;

    unsigned char           power_combined_stat;
    bool                    power_48V_in_stat;
    bool                    power_48V_out_stat;
    bool                    power_emg_sw_stat;
    bool                    power_pc_sw_stat;
    bool                    power_mc_sw_stat;
    bool                    power_gp_sw_stat;
    bool                    power_gp_A_port;
    bool                    power_gp_B_port;

    double                  power_48V_amp;
    
    unsigned char           jog_sig[8];

    
    void                    LAN_readData();
    void                    LAN_Flush();

    bool                    Compare_CAN_MSG_Is_Different(CAN_MSG mb1, CAN_MSG mb2);
    int                     CAN_writeData(CAN_MSG mb);

    void                    CAN_registerObserver(ICANObserver* obs);
    void                    CAN_unregisterObserver(ICANObserver* obs);
    void                    CAN_notifyCANObservers(int ch, int id, const unsigned char* data, int dlc);

    void                    Power_Command(int payload_data);
    void                    Power_register_state_callback(std::function<void()> cb);
    
private:
    std::deque<unsigned char> totalLanData;

    unsigned long           LAN_connectionThreadHandler;
    struct sockaddr_in      LAN_clientAddr;
    int                     LAN_fd_client;

    
    void                    LAN_writeData(const void *buf, size_t len);
    int                     LAN_createSocket(const char *addr, int port);
    int                     LAN_connectServer();
    int                     LAN_clientClose();

    static void             *LAN_connectionThread(void *arg);

    std::vector<ICANObserver*> CAN_observers;
    std::mutex              CAN_Observers_mutex;

    void                    CAN_clearBuffer();

    CAN_MSG                 CAN_TXring[CAN_MAX_NUM];
    int                     CAN_TXring_totalIndex;
    int                     CAN_TXring_headIndex;
    int                     CAN_TXring_tailIndex;
    std::mutex              CAN_TXring_mutex;

    bool                    power_command_flag;
    int                     power_command_payload;
    std::function<void()>   power_state_callback;

    std::string             L2C_IP;
    int                     L2C_PORT;
};

#endif // LAN2CAN_H
