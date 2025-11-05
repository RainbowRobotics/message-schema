#include "lan2can.h"
#include "common.h"
#include "dof.h"

#define P_NAME  "LAN2CAN"

lan2can::lan2can()
{
    LAN_connectionStatus = false;
    LAN_fd_client = 0;

    CAN_TXring_totalIndex = 0;
    CAN_TXring_headIndex = 0;
    CAN_TXring_tailIndex = 0;

    power_command_flag = false;
    power_command_payload = 0;
    rb_common::thread_create(LAN_connectionThread, 0, "RB_L2C_CONNECT", LAN_connectionThreadHandler, this);
}

lan2can::~lan2can(){
    LAN_connectionStatus = false;
    shutdown(LAN_fd_client, SHUT_WR);
    close(LAN_fd_client);
}

void *lan2can::LAN_connectionThread(void *arg){
    lan2can *lan = (lan2can *)arg;

    while(1){
        if(lan->LAN_connectionStatus == false){
            if(lan->LAN_fd_client == 0){
                if(lan->LAN_createSocket(SYSTEM_L2C_IP, SYSTEM_L2C_PORT) == true){
                    rb_common::log_push(LogLevel::Info, "Succeed to create socket", P_NAME);
                }
            }else{
                if(lan->LAN_connectServer() == true){
                    lan->LAN_connectionStatus = true;
                    rb_common::log_push(LogLevel::Info, "Succeed to connect l2c", P_NAME);
                }
            }
        }
        std::this_thread::sleep_for(1s);
    }
    return NULL;
}

void lan2can::LAN_writeData(const void *buf, size_t len){
    write(LAN_fd_client, buf, len);
}

void lan2can::LAN_readData(){

    static std::vector<unsigned char> totalLanData;

    static unsigned char rdata[RX_DATA_SIZE];
    static unsigned char pdata[RX_DATA_SIZE];
    int tcp_size = 0;

    if(LAN_connectionStatus == true){
        tcp_size = recv(LAN_fd_client, rdata, RX_DATA_SIZE, 0);
        totalLanData.insert(totalLanData.end(), &rdata[0], &rdata[tcp_size]);
        tcp_size = totalLanData.size();

        if(tcp_size > 0){
            while(1){
                tcp_size = totalLanData.size();
                if(tcp_size < 4){
                    return;
                }
                if(tcp_size >= 4){
                    int packet_length = 0;
                    if(totalLanData[0] == 0x24){
                        packet_length = (unsigned short)(totalLanData[1] | (totalLanData[2]<<8));
                        if(packet_length < 0){
                            rb_common::log_push(LogLevel::Warning, "Packet length under zero", P_NAME);
                        }
                    }else{
                        totalLanData.erase(totalLanData.begin(), totalLanData.begin()+1);
                        continue;
                    }

                    if(tcp_size < packet_length+3){
                        return;
                    }

                    if(packet_length == 1){
                        if(totalLanData[3] == 0x25){
                            totalLanData.erase(totalLanData.begin(), totalLanData.begin()+4);
                            continue;
                        }
                    }

                    if(tcp_size >= packet_length+3){
                        memcpy(pdata, &totalLanData[0], packet_length+3);
                        totalLanData.erase(totalLanData.begin(), totalLanData.begin()+(packet_length+3));
                        if(pdata[0] == 0x24 && pdata[packet_length+3-1] == 0x25){
                            int data_type = pdata[5];
                            switch(data_type){
                            case 0x00:
                            {
                                int num = (short)(pdata[6] | (pdata[7]<<8));
                                for(int j=0; j<num; j++){
                                    unsigned char can_data[12];
                                    for(int k=0; k<12; k++){
                                        can_data[k] = pdata[8+12*j+k];
                                    }

                                    int ch_type = can_data[0];
                                    int id = (short)(can_data[1] | (can_data[2]<<8));
                                    int dlc = can_data[3];
                                    unsigned char data[8];
                                    memcpy(data, &can_data[4], dlc);

                                    CAN_notifyCANObservers(ch_type, id, data, dlc);
                                }
                                break;
                            }
                            case 0x01:
                            {
                                // if(!power_48V_in_stat){
                                //     std::cout<<"PW_STAT"<<std::endl;
                                // }
                                unsigned char prev_power_combined_stat = power_combined_stat;
                                power_combined_stat = pdata[6];
                                power_48V_in_stat = (power_combined_stat >> 7) & 0x01;
                                power_48V_out_stat = (power_combined_stat >> 6) & 0x01;
                                power_emg_sw_stat = (power_combined_stat >> 5) & 0x01;
                                power_pc_sw_stat = (power_combined_stat >> 4) & 0x01;
                                power_mc_sw_stat = (power_combined_stat >> 3) & 0x01;
                                power_gp_sw_stat = (power_combined_stat >> 2) & 0x01;
                                power_gp_A_port = (power_combined_stat >> 1) & 0b01;
                                power_gp_B_port = (power_combined_stat >> 0) & 0b01;
                                power_48V_amp = ((double)pdata[7])/5.;
                                unsigned int temp_ch = pdata[8];
                                jog_sig[0] = (temp_ch >> 7) & 0x01;
                                jog_sig[1] = (temp_ch >> 6) & 0x01;
                                jog_sig[2] = (temp_ch >> 5) & 0x01;
                                jog_sig[3] = (temp_ch >> 4) & 0x01;

                                if(power_state_callback){
                                    if((prev_power_combined_stat != power_combined_stat)){
                                        power_state_callback();
                                    }
                                }
                                break;
                            }
                            default:
                                break;
                            }
                        }else{
                            rb_common::log_push(LogLevel::Warning, "Header footer not match", P_NAME);
                            for(std::vector<unsigned char>::iterator i  = totalLanData.begin() ; i < totalLanData.end(); i++ ){
                                if(*i == '$'){
                                    break;
                                }else{
                                    //printf("0x%x ",0xff&*i);
                                    totalLanData.erase(i);
                                }
                            }
                            //printf("\n");
                        }
                    }

                }
            }
        }
    }
}


int lan2can::LAN_createSocket(const char *addr, int port){
    LAN_fd_client = socket(AF_INET, SOCK_STREAM, 0);
    if(LAN_fd_client == -1){
        return false;
    }

    const char* interface_name = "enp0s31f6";  // 사용하려는 인터페이스 이름
    if (setsockopt(LAN_fd_client, SOL_SOCKET, SO_BINDTODEVICE, interface_name, strlen(interface_name)) < 0) {
        rb_common::log_push(LogLevel::Error, "SO_BINDTODEVICE failed (need root)", P_NAME);
        return false;
    }

    LAN_clientAddr.sin_addr.s_addr = inet_addr(addr);
    LAN_clientAddr.sin_family = AF_INET;
    LAN_clientAddr.sin_port = htons(port);

    int optval = 1;
    if(setsockopt(LAN_fd_client, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval)) == -1){
        return false;
    }
    if(setsockopt(LAN_fd_client, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1){
        return false;
    }
    if(setsockopt(LAN_fd_client, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval)) == -1){
        return false;
    }
    return true;
}

int lan2can::LAN_connectServer(){
    if(connect(LAN_fd_client, (struct sockaddr*)&LAN_clientAddr, sizeof(LAN_clientAddr)) == -1){
        return false;
    }
    return true;
}

int lan2can::LAN_clientClose(){
    return close(LAN_fd_client);
}

void lan2can::LAN_Flush(){
    static int cnt = HAND_SHAKE_LAN_CNT;
    static unsigned char send_byte[1024];

    if(LAN_connectionStatus){
        cnt--;

        if(CAN_TXring_headIndex != CAN_TXring_tailIndex){
            std::lock_guard<std::mutex> lock(CAN_TXring_mutex);  // 락 시작

            cnt = HAND_SHAKE_LAN_CNT;
            CAN_TXring_totalIndex = CAN_TXring_headIndex - CAN_TXring_tailIndex;
            if(CAN_TXring_totalIndex < 0){
                CAN_TXring_totalIndex += CAN_MAX_NUM;
            }

            if(CAN_TXring_totalIndex > 25){
                ;
            }

            send_byte[0] = 0x24;            // HEADER
            unsigned int dataLength = 4 + 2 + 12 * CAN_TXring_totalIndex;
            send_byte[1] = (dataLength)&0xFF;       // Total data length
            send_byte[2] = (dataLength>>8)&0xFF;    // Total data length
            send_byte[3] = 0;               // From (Master : 0, Slave(CAN) : 1, General interface board : 2)
            send_byte[4] = 1;               // To (Master : 0, Slave(CAN) : 1, General interface board : 2)
            send_byte[5] = 0;               // Data type (CAN Data : 0)
            send_byte[6] = (CAN_TXring_totalIndex)&0xFF;
            send_byte[7] = (CAN_TXring_totalIndex>>8)&0xFF;

            for(int idx = 0; idx < CAN_TXring_totalIndex; idx++){
                // data
                int temp_idx = CAN_TXring_tailIndex;
                send_byte[8 + (12 * idx)] = CAN_TXring[temp_idx].channel;// Channel
                unsigned int _CANID = CAN_TXring[temp_idx].id;

                send_byte[9 + (12 * idx)] = (_CANID)&0xFF;     // ID
                send_byte[10 + (12 * idx)]= (_CANID>>8)&0xFF;   // ID
                send_byte[11 + (12 * idx)]= CAN_TXring[temp_idx].dlc;               // Data Length Code
                for(int i=0; i<8; i++){
                    send_byte[12+i + (12 * idx)] = CAN_TXring[temp_idx].data[i];  // Data Fixed size(8bytes)
                }
                CAN_TXring_tailIndex = (CAN_TXring_tailIndex+1)%CAN_MAX_NUM;
            }

            send_byte[dataLength+3-1] = 0x25;           // FOOTER

            int now_tx_size = dataLength + 3;
            uint starting_point = dataLength+3;
            if(power_command_flag){
                send_byte[starting_point] = 0x24;                       starting_point++;
                send_byte[starting_point] = 5;                          starting_point++;
                send_byte[starting_point] = 0;                          starting_point++;
                send_byte[starting_point] = 0;                          starting_point++;
                send_byte[starting_point] = 1;                          starting_point++;
                send_byte[starting_point] = 3;                          starting_point++;
                send_byte[starting_point] = power_command_payload ;     starting_point++;
                send_byte[starting_point] = 0x25;                       starting_point++;
                now_tx_size += 8;
                power_command_flag = false;
            }

            LAN_writeData(send_byte, now_tx_size);
        }
        else{
            if(power_command_flag){
                unsigned int now_tx_size = 0;
                uint starting_point = 0;
                if(power_command_flag){
                    send_byte[starting_point] = 0x24;                       starting_point++;
                    send_byte[starting_point] = 5;                          starting_point++;
                    send_byte[starting_point] = 0;                          starting_point++;
                    send_byte[starting_point] = 0;                          starting_point++;
                    send_byte[starting_point] = 1;                          starting_point++;
                    send_byte[starting_point] = 3;                          starting_point++;
                    send_byte[starting_point] = power_command_payload;      starting_point++;
                    send_byte[starting_point] = 0x25;                       starting_point++;
                    now_tx_size += 8;
                    power_command_flag = false;
                }
                LAN_writeData(send_byte, now_tx_size);
            }else{
                if(cnt <= 0){
                    cnt = HAND_SHAKE_LAN_CNT;

                    send_byte[0] = 0x24;           // HEADER
                    send_byte[1] = 0x01;           // FOOTER
                    send_byte[2] = 0x00;           // FOOTER
                    send_byte[3] = 0x25;           // FOOTER
                    LAN_writeData(send_byte, 4);
                }
            }
        }
    }else{
        CAN_TXring_headIndex = CAN_TXring_tailIndex = 0;
    }
}

void lan2can::Power_Command(int payload_data){
    power_command_payload = payload_data;
    power_command_flag = true;
}

void lan2can::Power_register_state_callback(std::function<void()> cb) {
    power_state_callback = std::move(cb);
}

void lan2can::CAN_registerObserver(ICANObserver* obs) {
    std::lock_guard<std::mutex> lock(CAN_Observers_mutex);
    CAN_observers.push_back(obs);
}

void lan2can::CAN_unregisterObserver(ICANObserver* obs) {
    std::lock_guard<std::mutex> lock(CAN_Observers_mutex);
    CAN_observers.erase(std::remove(CAN_observers.begin(), CAN_observers.end(), obs), CAN_observers.end());
}

void lan2can::CAN_notifyCANObservers(int ch, int id, const unsigned char* data, int dlc) {
    std::lock_guard<std::mutex> lock(CAN_Observers_mutex);
    for (auto* obs : CAN_observers) {
        obs->onCANMessage(ch, id, data, dlc);
    }
}

bool lan2can::Compare_CAN_MSG_Is_Different(CAN_MSG mb1, CAN_MSG mb2){
    if(mb1.channel != mb2.channel)  return true;
    if(mb1.id != mb2.id)            return true;
    if(mb1.dlc != mb2.dlc)          return true;
    for(int i = 0; i < mb1.dlc; ++i){
        if(mb1.data[i] != mb2.data[i])  return true;
    }

    return false;
}

int lan2can::CAN_writeData(CAN_MSG mb){
    if(mb.id == 0 || mb.dlc == 0
    || mb.id >= 0x7FF || mb.dlc > 8
    || mb.channel > 1)  return 0;
    
    std::lock_guard<std::mutex> lock(CAN_TXring_mutex);  // 락 시작
    CAN_TXring[CAN_TXring_headIndex] = mb;
    CAN_TXring_headIndex = (CAN_TXring_headIndex+1)%CAN_MAX_NUM;
    return mb.dlc;
}

void lan2can::CAN_clearBuffer(){
    CAN_TXring_totalIndex = 0;
    CAN_TXring_headIndex = 0;
    CAN_TXring_tailIndex = 0;
}