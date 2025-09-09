#include "mobile.h"

MOBILE* MOBILE::instance(QObject* parent)
{
    static MOBILE* inst = nullptr;
    if(!inst && parent)
    {
        inst = new MOBILE(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

MOBILE::MOBILE(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr)
{
    cur_imu = Eigen::Vector3d(0,0,0);
}

MOBILE::~MOBILE()
{
    is_connected = false;
    msg_que.clear();

    recv_flag = false;
    if(recv_thread && recv_thread->joinable())
    {
        recv_thread->join();
    }
    recv_thread.reset();

    send_flag = false;
    if(send_thread && send_thread->joinable())
    {
        send_thread->join();
    }
    send_thread.reset();
}

void MOBILE::open()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        printf("[MOBILE] simulation mode\n");
        return;
    }

    // start loops
    recv_flag = true;
    recv_thread = std::make_unique<std::thread>(&MOBILE::recv_loop, this);

    send_flag = true;
    send_thread = std::make_unique<std::thread>(&MOBILE::send_loop, this);
}

void MOBILE::sync()
{
    time_sync();
    sync_st_time = get_time();
    is_sync = true;

    QString str;
    str.sprintf("[MOBILE] time sync, sync_st_time:%f", (double)sync_st_time);
    logger->write_log(str, "Green", true, false);
}

QString MOBILE::get_cur_pdu_state()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_pdu_state; // none, fail, good
    return res;
}

void MOBILE::set_cur_pdu_state(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_pdu_state = str; // none, fail, good
}

void MOBILE::set_is_inter_lock_foot(bool val)
{
    std::lock_guard<std::mutex> lock(mtx);
    is_inter_lock_foot = val;
}

void MOBILE::set_is_connected(bool val)
{
    is_connected.store(val);
}

void MOBILE::set_is_synced(bool val)
{
    is_synced.store(val);
}

void MOBILE::set_cur_pose(MOBILE_POSE mp)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_pose = mp;
}


double MOBILE::get_battery_soc()
{
    std::lock_guard<std::mutex> lock(mtx);
    double res = cur_status.tabos_soc;
    return res;
}

void MOBILE::set_cur_status(MOBILE_STATUS ms)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_status = ms;
}

MOBILE_POSE MOBILE::get_pose()
{
    std::lock_guard<std::mutex> lock(mtx);
    MOBILE_POSE res = cur_pose;
    return res;
}

MOBILE_POSE MOBILE::get_best_mo(double ref_t)
{
    std::lock_guard<std::mutex> lock(mtx);

    MOBILE_POSE res;
    double min_dt = std::numeric_limits<double>::max();
    for(size_t p = 0; p < pose_storage.size(); p++)
    {
        double dt = std::abs(pose_storage[p].t - ref_t);
        if(dt < min_dt)
        {
            min_dt = dt;
            res = pose_storage[p];
        }
    }

    return res;
}

MOBILE_STATUS MOBILE::get_status()
{
    std::lock_guard<std::mutex> lock(mtx);
    MOBILE_STATUS res = cur_status;
    return res;
}

MOBILE_SETTING MOBILE::get_setting()
{
    std::lock_guard<std::mutex> lock(mtx);
    MOBILE_SETTING res = cur_setting;
    return res;
}

Eigen::Vector3d MOBILE::get_imu()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector3d res = cur_imu;
    return res;
}

Eigen::Vector3d MOBILE::get_control_input()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector3d res(vx0, vy0, wz0);
    return res;
}

std::vector<MOBILE_IMU> MOBILE::get_imu_storage()
{
    std::lock_guard<std::mutex> lock(mtx);
    std::vector<MOBILE_IMU> res = imu_storage;
    return res;
}

std::vector<MOBILE_POSE> MOBILE::get_pose_storage()
{
    std::lock_guard<std::mutex> lock(mtx);
    std::vector<MOBILE_POSE> res = pose_storage;
    return res;
}

int MOBILE::get_pose_storage_size()
{
    std::lock_guard<std::mutex> lock(mtx);
    int res = pose_storage.size();
    return res;
}

int MOBILE::get_imu_storage_size()
{
    std::lock_guard<std::mutex> lock(mtx);
    int res = imu_storage.size();
    return res;
}

bool MOBILE::get_is_connected()
{
    return (bool)is_connected.load();
}

bool MOBILE::get_is_synced()
{
    return (bool)is_synced.load();
}

bool MOBILE::get_is_inter_lock_foot()
{
    return (bool)is_inter_lock_foot.load();
}

double MOBILE::get_last_pose_t()
{
    return (double)last_pose_t.load();
}

double MOBILE::get_process_time_mobile()
{
    return (double)process_time_mobile.load();
}

// for plot
QString MOBILE::get_pose_text()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = pose_text;
    return res;
}

// for mileage
double MOBILE::get_move_distance()
{
    std::lock_guard<std::mutex> lock(mtx);
    double res = (double)distance.load();
    return res;
}

QString MOBILE::get_status_text()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = status_text;
    return res;
}

// recv loop
void MOBILE::recv_loop()
{
    // pdu connection info
    const QString pdu_ip = "192.168.2.100";
    const int pdu_port = 1977;

    /*
    // check
    while(!ping(pdu_ip.toStdString()))
    {
        if(recv_flag == false)
        {
            logger->write_log("[MOBILE] pdu connection failed", "Red", true, false);
            return;
        }

        printf("[MOBILE] pdu ping check failed\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    }
    */

    // socket
    sockaddr_in server_addr;
    bzero((char*)&server_addr, sizeof(server_addr));
    server_addr.sin_family      = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(pdu_ip.toLocal8Bit().data());
    server_addr.sin_port        = htons(pdu_port);

    QString str;
    str.sprintf("[MOBILE] try connect, ip:%s, port:%d\n", pdu_ip.toLocal8Bit().data(), pdu_port);
    logger->write_log(str, "Green");

    // connection
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0)
    {
        logger->write_log("[MOBILE] socket create failed", "Red", true, false);
        return;
    }

    // set option
    {
        int val = 1;
        ::setsockopt(fd, SOL_TCP, TCP_NODELAY, &val, sizeof(val));
    }

    // set non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    int status = ::connect(fd, (sockaddr*)&server_addr, sizeof(server_addr));
    if(status < 0 && errno != EINPROGRESS)
    {
        logger->write_log("[MOBILE] connect failed", "Red", true, false);
        return;
    }

    // wait for connection with timeout
    fd_set writefds;
    struct timeval tv;
    tv.tv_sec = 10; // 10 seconds timeout
    tv.tv_usec = 0;

    FD_ZERO(&writefds);
    FD_SET(fd, &writefds);

    status = select(fd + 1, NULL, &writefds, NULL, &tv);
    if(status <= 0) // timeout or error
    {
        logger->write_log("[MOBILE] connect timeout or error", "Red", true, false);
        close(fd);
        return;
    }

    // socket error check after select()
    int so_error = 0;
    socklen_t len = sizeof(so_error);
    if(getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len) < 0)
    {
        QString errStr;
        errStr.sprintf("[MOBILE] getsockopt failed: %s", strerror(errno));
        logger->write_log(errStr, "Red", true, false);
        close(fd);
        return;
    }
    if(so_error!=0)
    {
        QString errStr;
        errStr.sprintf("[MOBILE] connect error after select: %s", strerror(so_error));
        logger->write_log(errStr, "Red", true, false);
        close(fd);
        return;
    }

    // set back to blocking mode
    fcntl(fd, F_SETFL, flags);

    is_connected = true;
    logger->write_log("[MOBILE] connected", "Green", true, false);

    std::vector<uchar> buf;
    int drop_cnt = MOBILE_INFO::drop_cnt;

    double pre_loop_time = get_time();

    printf("[MOBILE] recv loop start\n");
    while(recv_flag)
    {
        // storing buffer
        std::vector<uchar> recv_buf(MOBILE_INFO::recv_buf_size, 0);
        int num = read(fd, (char*)recv_buf.data(), recv_buf.size());
        if(num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        else if(num < 0)
        {
            logger->write_log("[MOBILE] read buffer size lower than 0", "Red");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // initial drop
        if(drop_cnt > 0)
        {
            drop_cnt--;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // storing packet
        buf.insert(buf.end(), recv_buf.begin(), recv_buf.begin()+num);

        /* parsing
         *
         * Header(1)
         * Data Size(2) -> pointing N
         * Dummy(2)
         * Type(1)
         * Data(N)
         * Footer(1)

         * S100    total 92   -> data 85
         * D400    total 133  -> data 126
         * Safety  total 186  -> data 199
         * Mecanum total 136  -> data 129
         */

        while((int)buf.size() > MOBILE_INFO::min_packet_size && recv_flag)
        {
            if(buf[0] == 0x24)
            {
                // Header
                int data_size = (unsigned short)(buf[1]|(buf[2]<<8));
                if(data_size + 7 <= buf.size())
                {
                    if(buf[data_size + 6] == 0x25)
                    {
                        // Footer
                        int index = 6;
                        const int dlc   = 1;
                        const int dlc_s = 2;
                        const int dlc_f = 4;

                        uchar *_buf = (uchar*)buf.data();

                        RobotType_PDU robot_type = RobotType_PDU::ROBOT_TYPE_UNKNOWN;
                        if(data_size == MOBILE_INFO::packet_size_d400)
                        {
                            robot_type = RobotType_PDU::ROBOT_TYPE_D400;
                        }
                        else if(data_size == MOBILE_INFO::packet_size_s100)
                        {
                            robot_type = RobotType_PDU::ROBOT_TYPE_S100;
                        }
                        else if(data_size == MOBILE_INFO::packet_size_mecanum)
                        {
                            robot_type = RobotType_PDU::ROBOT_TYPE_MECANUM;
                        }
                        else if(data_size == MOBILE_INFO::packet_size_safety)
                        {
                            robot_type = RobotType_PDU::ROBOT_TYPE_SAFETY;
                        }
                        else
                        {
                            std::cout << "wrong robot_type: " << static_cast<int>(robot_type) << ", data_size: " << data_size << std::endl;
                        }

                        if(_buf[5] == 0xA2)
                        {
                            // Normal Data
                            uint32_t tick;
                            memcpy(&tick, &_buf[index], dlc_f); index=index+dlc_f;

                            // calc mobile(pdu) & pc time
                            double pc_t     = get_time();
                            double mobile_t = tick * MOBILE_INFO::pdu_tick_resolution;

                            uint32_t recv_tick; float return_time;
                            if(robot_type != RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                memcpy(&recv_tick, &_buf[index], dlc_f);   index=index+dlc_f;
                                memcpy(&return_time, &_buf[index], dlc_f); index=index+dlc_f;
                            }

                            uint8_t connection_status_m0, connection_status_m1, connection_status_m2, connection_status_m3;
                            connection_status_m0 = _buf[index]; index=index+dlc;
                            connection_status_m1 = _buf[index]; index=index+dlc;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                connection_status_m2 = _buf[index]; index=index+dlc;
                                connection_status_m3 = _buf[index]; index=index+dlc;
                            }

                            float x_dot, y_dot, th_dot;
                            memcpy(&x_dot, &_buf[index], dlc_f);  index=index+dlc_f;
                            memcpy(&y_dot, &_buf[index], dlc_f);  index=index+dlc_f;
                            memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

                            float x, y, th;
                            memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
                            memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
                            memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

                            float local_v, local_w;
                            float local_vx, local_vy, local_wz;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                memcpy(&local_vx, &_buf[index], dlc_f);     index=index+dlc_f;
                                memcpy(&local_vy, &_buf[index], dlc_f);     index=index+dlc_f;
                                memcpy(&local_wz, &_buf[index], dlc_f);     index=index+dlc_f;
                            }
                            else
                            {
                                memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
                                memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;
                            }

                            uint8_t stat_m0, stat_m1;
                            uint8_t stat_m2, stat_m3;
                            stat_m0 = _buf[index];     index=index+dlc;
                            stat_m1 = _buf[index];     index=index+dlc;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                stat_m2 = _buf[index];     index=index+dlc;
                                stat_m3 = _buf[index];     index=index+dlc;
                            }

                            uint8_t temp_m0, temp_m1;
                            uint8_t temp_m2, temp_m3;
                            temp_m0 = _buf[index];     index=index+dlc;
                            temp_m1 = _buf[index];     index=index+dlc;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                temp_m2 = _buf[index];     index=index+dlc;
                                temp_m3 = _buf[index];     index=index+dlc;
                            }

                            uint8_t esti_temp_m0, esti_temp_m1;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_D400)
                            {
                                memcpy(&esti_temp_m0, &_buf[index], dlc);     index=index+dlc;
                                memcpy(&esti_temp_m1, &_buf[index], dlc);     index=index+dlc;
                            }

                            uint8_t cur_m0, cur_m1;
                            uint8_t cur_m2, cur_m3;
                            cur_m0 = _buf[index];     index=index+dlc;
                            cur_m1 = _buf[index];     index=index+dlc;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                cur_m2 = _buf[index];     index=index+dlc;
                                cur_m3 = _buf[index];     index=index+dlc;
                            }

                            uint8_t charge_state, power_state, motor_stop_state, remote_state;
                            uint8_t auto_manual_sw, brake_sw, reset_sw, stop_sw, start_sw;
                            uint8_t _om_state, _ri_state, remote_flag, bumper_state;
                            uint8_t safety_emo_pressed_1, safety_ref_meas_mismatch_1, safety_over_speed_1,safety_obs_detect_1,
                                    safety_speed_field_mismatch_1, safety_stop_state_flag_1, safety_interlock_stop_1, safety_bumper_stop_1;
                            uint8_t safety_emo_pressed_2, safety_ref_meas_mismatch_2, safety_over_speed_2,safety_obs_detect_2,
                                    safety_speed_field_mismatch_2, safety_stop_state_flag_2, safety_interlock_stop_2, safety_bumper_stop_2;

                            if(robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                memcpy(&auto_manual_sw, &_buf[index], dlc);       index=index+dlc;
                                memcpy(&brake_sw, &_buf[index], dlc);             index=index+dlc;
                                memcpy(&reset_sw, &_buf[index], dlc);             index=index+dlc;
                                memcpy(&stop_sw, &_buf[index], dlc);              index=index+dlc;
                                memcpy(&start_sw, &_buf[index], dlc);             index=index+dlc;

                                memcpy(&_om_state, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&_ri_state, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&charge_state, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&remote_flag, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&bumper_state, &_buf[index],dlc);        index=index+dlc;

                                memcpy(&safety_emo_pressed_1, &_buf[index], dlc);             index=index+dlc;
                                memcpy(&safety_ref_meas_mismatch_1, &_buf[index], dlc);       index=index+dlc;
                                memcpy(&safety_over_speed_1, &_buf[index], dlc);              index=index+dlc;
                                memcpy(&safety_obs_detect_1, &_buf[index], dlc);              index=index+dlc;
                                memcpy(&safety_speed_field_mismatch_1, &_buf[index], dlc);    index=index+dlc;
                                memcpy(&safety_stop_state_flag_1, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&safety_interlock_stop_1, &_buf[index] ,dlc);          index=index+dlc;
                                memcpy(&safety_bumper_stop_1, &_buf[index], dlc);             index=index+dlc;

                                memcpy(&safety_emo_pressed_2, &_buf[index], dlc);             index=index+dlc;
                                memcpy(&safety_ref_meas_mismatch_2, &_buf[index], dlc);       index=index+dlc;
                                memcpy(&safety_over_speed_2, &_buf[index], dlc);              index=index+dlc;
                                memcpy(&safety_obs_detect_2, &_buf[index], dlc);              index=index+dlc;
                                memcpy(&safety_speed_field_mismatch_2, &_buf[index], dlc);    index=index+dlc;
                                memcpy(&safety_stop_state_flag_2, &_buf[index], dlc);         index=index+dlc;
                                memcpy(&safety_interlock_stop_2, &_buf[index] ,dlc);          index=index+dlc;
                                memcpy(&safety_bumper_stop_2, &_buf[index], dlc);             index=index+dlc;
                            }
                            else
                            {
                                charge_state = _buf[index];     index=index+dlc;
                                power_state = _buf[index];      index=index+dlc;
                                motor_stop_state = _buf[index];        index=index+dlc;
                                remote_state = _buf[index];     index=index+dlc;
                            }

                            float bat_in, bat_out, bat_cur, power, total_used_power;
                            float lift_voltage_in, lift_voltage_out, lift_current;
                            float battery_voltage, charge_current, contact_voltage;
                            float motor_core_temp0, motor_core_temp1;

                            memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
                            memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
                            memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;

                            if(robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                memcpy(&lift_voltage_in, &_buf[index], dlc_f);               index=index+dlc_f;
                                memcpy(&lift_voltage_out, &_buf[index], dlc_f);              index=index+dlc_f;
                                memcpy(&lift_current, &_buf[index], dlc_f);                  index=index+dlc_f;

                                memcpy(&battery_voltage, &_buf[index], dlc_f);              index=index+dlc_f;
                                memcpy(&contact_voltage, &_buf[index], dlc_f);              index=index+dlc_f;
                                memcpy(&charge_current, &_buf[index], dlc_f);               index=index+dlc_f;
                            }

                            memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
                            memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                memcpy(&recv_tick, &_buf[index], dlc_f);        index=index+dlc_f;
                                memcpy(&return_time, &_buf[index], dlc_f);      index=index+dlc_f;
                            }

                            if(robot_type == RobotType_PDU::ROBOT_TYPE_D400)
                            {
                                memcpy(&charge_current, &_buf[index], dlc_f);               index=index+dlc_f;
                                memcpy(&contact_voltage, &_buf[index], dlc_f);               index=index+dlc_f;
                            }
                            else if(robot_type == RobotType_PDU::ROBOT_TYPE_S100 || robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                memcpy(&motor_core_temp0, &_buf[index], dlc_f);     index=index+dlc_f;
                                memcpy(&motor_core_temp1, &_buf[index], dlc_f);     index=index+dlc_f;
                            }

                            uint8_t state;
                            float q0, q1, q2, q3;
                            float imu_gyr_x=0.0, imu_gyr_y=0.0, imu_gyr_z=0.0;
                            float imu_acc_x=0.0, imu_acc_y=0.0, imu_acc_z=0.0;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_S100)
                            {
                                state = _buf[index];     index=index+dlc;
                            }
                            else if(robot_type == RobotType_PDU::ROBOT_TYPE_D400 || robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                memcpy(&q0, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&q1, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&q2, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&q3, &_buf[index], dlc_f);      index=index+dlc_f;

                                memcpy(&imu_gyr_x, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&imu_gyr_y, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&imu_gyr_z, &_buf[index], dlc_f);      index=index+dlc_f;

                                memcpy(&imu_acc_x, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&imu_acc_y, &_buf[index], dlc_f);      index=index+dlc_f;
                                memcpy(&imu_acc_z, &_buf[index], dlc_f);      index=index+dlc_f;
                            }

                            uint8_t inter_lock_state;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                inter_lock_state = _buf[index];     index=index+dlc;
                            }


                            uint8_t lidar_field;
                            short ref_dps_0, ref_dps_1;
                            short meas_dps_0, meas_dps_1;
                            uint8_t io_dout[16];
                            uint8_t io_din[16];
                            uint8_t io_adc[4];
                            uint8_t io_dac[4];

                            unsigned short _tabos_voltage;
                            short _tabos_current;
                            uint16_t _tabos_status;
                            unsigned short _tabos_ttf;
                            unsigned short _tabos_tte;
                            unsigned char _tabos_soc;
                            unsigned char _tabos_soh;
                            short _tabos_temperature;
                            unsigned short _tabos_rc;
                            unsigned short _tabos_ae;


                            if(robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                memcpy(&lidar_field, &_buf[index], dlc);                index=index+dlc;
                                memcpy(&ref_dps_0, &_buf[index], dlc_s);                index=index+dlc_s;
                                memcpy(&ref_dps_1, &_buf[index], dlc_s);                index=index+dlc_s;
                                memcpy(&meas_dps_0, &_buf[index], dlc_s);                index=index+dlc_s;
                                memcpy(&meas_dps_1, &_buf[index], dlc_s);                index=index+dlc_s;

                                for(int i=0; i<16; i++)
                                {
                                    memcpy(&io_dout[i], &_buf[index], dlc);                  index=index+dlc;
                                }
                                for(int i=0; i<16; i++)
                                {
                                    memcpy(&io_din[i], &_buf[index], dlc);                  index=index+dlc;
                                }
                                for(int i=0; i<4; i++)
                                {
                                    memcpy(&io_adc[i], &_buf[index], dlc);                  index=index+dlc;
                                }
                                for(int i=0; i<4; i++)
                                {
                                    memcpy(&io_dac[i], &_buf[index], dlc);                  index=index+dlc;
                                }

                                memcpy(&_tabos_voltage, &_buf[index], dlc_s);     index=index+dlc_s;
                                memcpy(&_tabos_current, &_buf[index], dlc_s);     index=index+dlc_s;
                                memcpy(&_tabos_status, &_buf[index], dlc_s);      index=index+dlc_s;
                                memcpy(&_tabos_ttf, &_buf[index], dlc_s);         index=index+dlc_s;
                                memcpy(&_tabos_tte, &_buf[index], dlc_s);         index=index+dlc_s;
                                memcpy(&_tabos_soc, &_buf[index], dlc);           index=index+dlc;
                                memcpy(&_tabos_soh, &_buf[index], dlc);           index=index+dlc;
                                memcpy(&_tabos_temperature, &_buf[index], dlc_s);  index=index+dlc_s;
                                memcpy(&_tabos_rc, &_buf[index], dlc_s);           index=index+dlc_s;
                                memcpy(&_tabos_ae, &_buf[index], dlc_s);           index=index+dlc_s;
                            }

                            // calc time offset
                            if(is_sync && pc_t > sync_st_time + 0.1)
                            {
                                is_sync = false;

                                double _mobile_t = recv_tick * MOBILE_INFO::pdu_tick_resolution;
                                double _offset_t = pc_t - _mobile_t;
                                offset_t = _offset_t;

                                is_synced = true;
                                printf("[MOBILE] sync, offset_t: %f\n", (double)offset_t);
                            }

                            // battery percentage value stabilization
                            if(is_first_receive)
                            {
                                is_first_receive = false;
                                input_voltage = bat_out;
                            }
                            if(bat_out < input_voltage && charge_state == 0)
                            {
                                input_voltage = bat_out;
                            }
                            if(bat_out >= input_voltage && charge_state != 0)
                            {
                                input_voltage = bat_out;
                            }
                            int bat_percent = calc_battery_percentage(input_voltage);

                            // received mobile pose update
                            MOBILE_POSE mobile_pose;
                            mobile_pose.t = mobile_t + offset_t;
                            mobile_pose.pose = Eigen::Vector3d(x, y, toWrap(th));
                            mobile_pose.vel = Eigen::Vector3d(local_v, 0, local_w);

                            // received mobile status update
                            MOBILE_STATUS mobile_status;
                            mobile_status.t = mobile_t + offset_t;

                            // motor
                            mobile_status.connection_m0 = connection_status_m0;
                            mobile_status.connection_m1 = connection_status_m1;
                            mobile_status.connection_m2 = connection_status_m2;
                            mobile_status.connection_m3 = connection_status_m3;
                            mobile_status.status_m0 = stat_m0;
                            mobile_status.status_m1 = stat_m1;
                            mobile_status.status_m2 = stat_m2;
                            mobile_status.status_m3 = stat_m3;
                            mobile_status.temp_m0 = temp_m0;
                            mobile_status.temp_m1 = temp_m1;
                            mobile_status.temp_m2 = temp_m2;
                            mobile_status.temp_m3 = temp_m3;
                            mobile_status.esti_temp_m0 = esti_temp_m0;
                            mobile_status.esti_temp_m1 = esti_temp_m1;
                            mobile_status.cur_m0 = cur_m0;
                            mobile_status.cur_m1 = cur_m1;
                            mobile_status.cur_m2 = cur_m2;
                            mobile_status.cur_m3 = cur_m3;
                            mobile_status.charge_state = charge_state;
                            mobile_status.power_state = power_state;
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                mobile_status.motor_stop_state = !(safety_emo_pressed_1||safety_emo_pressed_2);
                            }
                            else
                            {
                                mobile_status.motor_stop_state =  motor_stop_state;
                            }
                            mobile_status.remote_state = remote_state;
                            mobile_status.bat_in = bat_in;
                            mobile_status.bat_out = bat_out;
                            mobile_status.bat_current = bat_cur;
                            mobile_status.bat_percent = bat_percent;
                            mobile_status.lift_voltage_in = lift_voltage_in;
                            mobile_status.lift_voltage_out = lift_voltage_out;
                            mobile_status.lift_current = lift_current;
                            mobile_status.power = power;
                            mobile_status.total_power = total_used_power;
                            mobile_status.bat_voltage = battery_voltage;
                            mobile_status.charge_current = charge_current;
                            mobile_status.contact_voltage = contact_voltage;
                            mobile_status.recv_tick = recv_tick;
                            mobile_status.return_time = return_time;
                            mobile_status.core_temp0 = motor_core_temp0;
                            mobile_status.core_temp1 = motor_core_temp1;
                            mobile_status.state = state;

                            // safety
                            mobile_status.auto_manual_sw = auto_manual_sw;
                            mobile_status.brake_release_sw = brake_sw;
                            mobile_status.sw_reset = reset_sw;
                            mobile_status.sw_stop = stop_sw;
                            mobile_status.sw_start = start_sw;
                            mobile_status.bumper_state = bumper_state;

                            mobile_status.charge_state = charge_state;
                            mobile_status.ri_state = _ri_state;
                            mobile_status.om_state = _om_state;
                            mobile_status.remote_state = remote_flag;

                            mobile_status.safety_state_emo_pressed_1 = safety_emo_pressed_1;
                            mobile_status.safety_state_ref_meas_mismatch_1 = safety_ref_meas_mismatch_1;
                            mobile_status.safety_state_over_speed_1 = safety_over_speed_1;
                            mobile_status.safety_state_obstacle_detected_1 = safety_obs_detect_1;
                            mobile_status.safety_state_speed_field_mismatch_1 = safety_speed_field_mismatch_1;
                            mobile_status.safety_state_interlock_stop_1 = safety_interlock_stop_1;
                            mobile_status.safety_state_bumper_stop_1 = safety_bumper_stop_1;
                            mobile_status.operational_stop_state_flag_1 = safety_stop_state_flag_1;

                            mobile_status.safety_state_emo_pressed_2 = safety_emo_pressed_2;
                            mobile_status.safety_state_ref_meas_mismatch_2 = safety_ref_meas_mismatch_2;
                            mobile_status.safety_state_over_speed_2 = safety_over_speed_2;
                            mobile_status.safety_state_obstacle_detected_2 = safety_obs_detect_2;
                            mobile_status.safety_state_speed_field_mismatch_2 = safety_speed_field_mismatch_2;
                            mobile_status.safety_state_interlock_stop_2 = safety_interlock_stop_2;
                            mobile_status.safety_state_bumper_stop_2 = safety_bumper_stop_2;
                            mobile_status.operational_stop_state_flag_2 = safety_stop_state_flag_2;

                            mobile_status.lidar_field = lidar_field;

                            mobile_status.ref_dps[0] = ref_dps_0;
                            mobile_status.ref_dps[1] = ref_dps_1;
                            mobile_status.meas_dps[0] = meas_dps_0;
                            mobile_status.meas_dps[1] = meas_dps_1;

                            //dio
                            for(int i=0; i<8; i++)
                            {
                                mobile_status.mcu1_dio[i] = io_dout[i];
                            }
                            for(int i=0; i<8; i++)
                            {
                                mobile_status.mcu0_dio[i] = io_dout[i+8];
                            }

                            //din
                            for(int i=0; i<8; i++)
                            {
                                mobile_status.mcu1_din[i] = io_din[i];
                            }
                            for(int i=0; i<8; i++)
                            {
                                mobile_status.mcu0_din[i] = io_din[i+8];
                            }

                            mobile_status.tabos_voltage = _tabos_voltage * 0.01f;
                            mobile_status.tabos_current = _tabos_current * 0.01f;
                            mobile_status.tabos_status = _tabos_status;
                            mobile_status.tabos_ttf = _tabos_ttf;
                            mobile_status.tabos_tte = _tabos_tte;

                            mobile_status.tabos_soc = _tabos_soc;
                            mobile_status.tabos_soh = _tabos_soh;
                            mobile_status.tabos_temperature = _tabos_temperature * 0.1f;
                            mobile_status.tabos_rc = _tabos_rc * 0.01f;
                            mobile_status.tabos_ae = _tabos_ae * 0.1f;
                            
                            // imu
                            mobile_status.imu_gyr_x = imu_gyr_x * D2R;
                            mobile_status.imu_gyr_y = imu_gyr_y * D2R;
                            mobile_status.imu_gyr_z = imu_gyr_z * D2R;
                            mobile_status.imu_acc_x = imu_acc_x * ACC_G;
                            mobile_status.imu_acc_y = imu_acc_y * ACC_G;
                            mobile_status.imu_acc_z = imu_acc_z * ACC_G;

                            mobile_status.inter_lock_state = inter_lock_state;

                            // get orientation
                            //Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
                            Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
                            Eigen::Vector3d r = Sophus::SO3d::fitToSO3(R).log();

                            MOBILE_IMU imu;
                            imu.t = mobile_status.t;
                            imu.acc_x = mobile_status.imu_acc_x;
                            imu.acc_y = mobile_status.imu_acc_y;
                            imu.acc_z = mobile_status.imu_acc_z;
                            imu.gyr_x = mobile_status.imu_gyr_x;
                            imu.gyr_y = mobile_status.imu_gyr_y;
                            imu.gyr_z = mobile_status.imu_gyr_z;
                            imu.rx = r[0];
                            imu.ry = r[1];
                            imu.rz = r[2];

                            Eigen::Vector3d cmd = get_control_input();

                            // pose & status text
                            QString mobile_pose_str   = "[MOBILE_POSE]";
                            QString mobile_status_str = "[MOBILE_STATUS]";
                            if(robot_type == RobotType_PDU::ROBOT_TYPE_S100)
                            {

                                mobile_pose_str.sprintf("[MOBILE_POSE]\nt:%.3f\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f\ncmd:%.2f, %.2f, %.2f",
                                                          mobile_pose.t,
                                                          mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                                                          mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
                                                          cmd[0], cmd[1], cmd[2]*R2D);

                                mobile_status_str.sprintf("[MOBILE_STATUS]\nconnection(m0,m1):%d,%d, status(m0,m1):%d,%d\ntemp(m0,m1): %d,%d, cur(m0,m1):%.2f,%.2f\ncharge,power,emo,remote:%d,%d,%d,%d\nBAT(in,out,cur,per):%.3f,%.3f,%.3f,%d %\npower:%.3f, total power:%.3f\ncore_temp(m0,m1,state): %f, %f, %d",
                                                          mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1,
                                                          (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                                                          mobile_status.charge_state, mobile_status.power_state, mobile_status.motor_stop_state, mobile_status.remote_state,
                                                          mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,mobile_status.bat_percent,
                                                          mobile_status.power, mobile_status.total_power,
                                                          mobile_status.core_temp0, mobile_status.core_temp1, mobile_status.state);
                            }
                            else if(robot_type == RobotType_PDU::ROBOT_TYPE_D400 || robot_type == RobotType_PDU::ROBOT_TYPE_SAFETY)
                            {
                                mobile_pose_str.sprintf("[MOBILE_POSE]\nt:%.3f\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f\ncmd:%.2f, %.2f, %.2f",
                                                          mobile_pose.t,
                                                          mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                                                          mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
                                                          cmd[0], cmd[1], cmd[2]*R2D);

                                mobile_status_str.sprintf("[MOBILE_STATUS]\nconnection(m0,m1):%d,%d, status(m0,m1):%d,%d\n"
                                             "temp(m0,m1): %d,%d,(%d,%d), cur(m0,m1):%.2f,%.2f\n"
                                             "charge,om_state,emo,ri_state:%d,%d,%d,%d\n"
                                             "BAT(in,out,cur,per):%.3f,%.3f,%.3f,%d %\n"
                                             "power:%.3f, total power:%.3f, c.c:%.3f, c.v:%.3f\n"
                                             "gyr:%.2f,%.2f,%.2f acc:%.3f,%.3f,%.3f\n"
                                             "bms_v:%.2f, bms_a:%.2f, bms_ttf:%d, bms_tte:%d, bms_soc:%d \n" 
                                             "bms_soh:%d, bms_temp:%.2f, bms_rc:%.2f, bms_ae:%.2f, bms_sat:%d \n"
                                             "SFTY(emo,refm,spd,obs,sfld,intlk,op):{%d,%d},{%d,%d},{%d,%d},{%d,%d},{%d,%d},{%d,%d},{%d,%d}\n"
                                             "bumper:{%d,%d} lidar_field:%d)",
                                                          mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1, mobile_status.esti_temp_m0, mobile_status.esti_temp_m1,
                                                          (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                                                          mobile_status.charge_state, mobile_status.om_state, mobile_status.motor_stop_state, mobile_status.ri_state,
                                                          mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,mobile_status.bat_percent,
                                                          mobile_status.power, mobile_status.total_power, mobile_status.charge_current, mobile_status.contact_voltage,
                                                          mobile_status.imu_gyr_x, mobile_status.imu_gyr_y, mobile_status.imu_gyr_z,
                                                          mobile_status.imu_acc_x, mobile_status.imu_acc_y, mobile_status.imu_acc_z,
                                                          mobile_status.tabos_voltage, mobile_status.tabos_current, mobile_status.tabos_ttf, mobile_status.tabos_tte, mobile_status.tabos_soc, mobile_status.tabos_soh,
                                                          mobile_status.tabos_temperature, mobile_status.tabos_rc, mobile_status.tabos_ae, mobile_status.tabos_status,
                                                          mobile_status.safety_state_emo_pressed_1,         mobile_status.safety_state_emo_pressed_2,
                                                          mobile_status.safety_state_ref_meas_mismatch_1,   mobile_status.safety_state_ref_meas_mismatch_2,
                                                          mobile_status.safety_state_over_speed_1,          mobile_status.safety_state_over_speed_2,
                                                          mobile_status.safety_state_obstacle_detected_1,   mobile_status.safety_state_obstacle_detected_2,
                                                          mobile_status.safety_state_speed_field_mismatch_1,mobile_status.safety_state_speed_field_mismatch_2,
                                                          mobile_status.safety_state_interlock_stop_1,      mobile_status.safety_state_interlock_stop_2,
                                                          mobile_status.operational_stop_state_flag_1,      mobile_status.operational_stop_state_flag_2,
                                                          mobile_status.safety_state_bumper_stop_1,         mobile_status.safety_state_bumper_stop_2,
                                                          mobile_status.lidar_field);
                            }
                            else if(robot_type == RobotType_PDU::ROBOT_TYPE_MECANUM)
                            {
                                mobile_pose_str.sprintf("[MOBILE_POSE]\nt:%.3f\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f\ncmd:%.2f, %.2f, %.2f",
                                                          mobile_pose.t,
                                                          mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                                                          mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
                                                          cmd[0], cmd[1], cmd[2]*R2D);

                                mobile_status_str.sprintf("[MOBILE_STATUS]\nconnection:%d,%d,%d,%d\nstatus:%d,%d,%d,%d\ntemp:%d,%d,%d,%d, cur:%.2f,%.2f,%2f,%2f\ncharge,power,emo,remote:%d,%d,%d,%d\ncharge cur,vol:%.2f,%.2f\nBAT(in,out,cur):%.3f,%.3f,%.3f\npower:%.3f, total power:%.3f\ngyr:%.2f,%.2f,%.2f acc:%.3f,%.3f,%.3f\nTFB:%d",
                                                          mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.connection_m2, mobile_status.connection_m3,
                                                          mobile_status.status_m0, mobile_status.status_m1, mobile_status.status_m2, mobile_status.status_m3,
                                                          mobile_status.temp_m0, mobile_status.temp_m1, mobile_status.temp_m2, mobile_status.temp_m3,
                                                          (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0, (double)mobile_status.cur_m2/10.0, (double)mobile_status.cur_m3/10.0,
                                                          mobile_status.charge_state, mobile_status.power_state, mobile_status.motor_stop_state, mobile_status.remote_state,
                                                          mobile_status.charge_current, mobile_status.contact_voltage,
                                                          mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,
                                                          mobile_status.power, mobile_status.total_power,
                                                          mobile_status.imu_gyr_x, mobile_status.imu_gyr_y, mobile_status.imu_gyr_z,
                                                          mobile_status.imu_acc_x, mobile_status.imu_acc_y, mobile_status.imu_acc_z,
                                                          mobile_status.inter_lock_state);
                            }

                            // storing
                            mtx.lock();

                            pose_text = mobile_pose_str;
                            status_text = mobile_status_str;

                            cur_pose = mobile_pose;
                            cur_status = mobile_status;
                            cur_imu = r;

                            pose_storage.push_back(mobile_pose);
                            if(pose_storage.size() > MO_STORAGE_NUM)
                            {
                                pose_storage.erase(pose_storage.begin());
                            }

                            imu_storage.push_back(imu);
                            if(imu_storage.size() > MO_STORAGE_NUM)
                            {
                                imu_storage.erase(imu_storage.begin());
                            }
                            mtx.unlock();

                            // update last t
                            last_pose_t = mobile_pose.t;
                            last_imu_t = imu.t;
                        }
                        else if(_buf[5] == 0xA3)
                        {
                            // Safety Data

                            unsigned int _version;
                            memcpy(&_version, &_buf[index], dlc_f);     index=index+dlc_f;

                            unsigned char _robot_type;
                            memcpy(&_robot_type, &_buf[index], dlc);     index=index+dlc;

                            float _v_limit;
                            memcpy(&_v_limit, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _w_limit;
                            memcpy(&_w_limit, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _a_limit;
                            memcpy(&_a_limit, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _b_limit;
                            memcpy(&_b_limit, &_buf[index], dlc_f);     index=index+dlc_f;

                            float _v_limit_jog;
                            memcpy(&_v_limit_jog, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _w_limit_jog;
                            memcpy(&_w_limit_jog, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _a_limit_jog;
                            memcpy(&_a_limit_jog, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _b_limit_jog;
                            memcpy(&_b_limit_jog, &_buf[index], dlc_f);     index=index+dlc_f;

                            float _v_limit_monitor;
                            memcpy(&_v_limit_monitor, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _w_limit_monitor;
                            memcpy(&_w_limit_monitor, &_buf[index], dlc_f);     index=index+dlc_f;

                            float _w_s;
                            memcpy(&_w_s, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _w_r;
                            memcpy(&_w_r, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _gear;
                            memcpy(&_gear, &_buf[index], dlc_f);     index=index+dlc_f;
                            float _dir;
                            memcpy(&_dir, &_buf[index], dlc_f);     index=index+dlc_f;

                            float _safety_v_limit;
                            memcpy(&_safety_v_limit, &_buf[index], dlc_f);      index=index+dlc_f;
                            float _safety_w_limit;
                            memcpy(&_safety_w_limit, &_buf[index], dlc_f);      index=index+dlc_f;


                            MOBILE_SETTING mobile_setting;
                            mobile_setting.version = _version;
                            mobile_setting.robot_type = _robot_type;
                            mobile_setting.v_limit = _v_limit;
                            mobile_setting.w_limit = _w_limit;
                            mobile_setting.a_limit = _a_limit;
                            mobile_setting.b_limit = _b_limit;
                            mobile_setting.v_limit_jog = _v_limit_jog;
                            mobile_setting.w_limit_jog = _w_limit_jog;
                            mobile_setting.a_limit_jog = _a_limit_jog;
                            mobile_setting.b_limit_jog = _b_limit_jog;
                            mobile_setting.v_limit_monitor = _v_limit_monitor;
                            mobile_setting.w_limit_monitor = _w_limit_monitor;
                            mobile_setting.safety_v_limit = _safety_v_limit; //not used
                            mobile_setting.safety_w_limit = _safety_w_limit; //not used
                            mobile_setting.w_s = _w_s;
                            mobile_setting.w_r = _w_r;
                            mobile_setting.gear = _gear;
                            mobile_setting.dir = _dir;
                            // storing
                            mtx.lock();
                            cur_setting = mobile_setting;
                            mtx.unlock();
                        }

                        buf.erase(buf.begin(), buf.begin()+data_size+7);
                    }
                    else
                    {
                        qDebug() << "Footer Fail";
                        buf.erase(buf.begin(), buf.begin()+1);
                    }
                }
                else
                {
                    qDebug() << "Size Fail";
                    break;
                }
            }
            else
            {
                buf.erase(buf.begin(), buf.begin()+1);
            }
        }

        double cur_loop_time = get_time();
        process_time_mobile = cur_loop_time - pre_loop_time;

        f_distance += std::fabs(cur_pose.vel[0] * process_time_mobile);

        mtx.lock();
        distance = f_distance;
        mtx.unlock();

        pre_loop_time = cur_loop_time;

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MOBILE] recv loop stop\n");
}


// command func
void MOBILE::motor_on()
{

    // set wheel base, wheel radius , wheel dir, gear ratio
    {
        float wheel_base = config->get_robot_wheel_base();
        float wheel_radius = config->get_robot_wheel_radius();

        float wheel_dir = config->get_motor_direction();
        float gear_ratio = config->get_motor_gear_ratio();

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 104; // cmd motor init

        memcpy(&send_byte[8], &wheel_base, 4);
        memcpy(&send_byte[12], &wheel_radius, 4);

        memcpy(&send_byte[16], &wheel_dir, 4);
        memcpy(&send_byte[20], &gear_ratio, 4);

        send_byte[24] = 0x25;

        if(is_connected && !config->get_use_sim())
        {
            msg_que.push(send_byte);
        }
    }

    // set limit vel
    {
        float limit_v = config->get_motor_limit_v();
        float limit_a = config->get_motor_limit_v_acc();
        float limit_w = config->get_motor_limit_w() * D2R;
        float limit_w_acc = config->get_motor_limit_w_acc() * D2R;


        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 108; // cmd motor limit velocity

        memcpy(&send_byte[8], &limit_v, 4);
        memcpy(&send_byte[12], &limit_a, 4);
        memcpy(&send_byte[16], &limit_w, 4);
        memcpy(&send_byte[20], &limit_w_acc,4);

        send_byte[24] = 0x25;

        if(is_connected && !config->get_use_sim())
        {
            msg_que.push(send_byte);
        }
    }

    // set safety limit
    {

        float safety_limit_v = config->get_motor_safety_limit_v();
        float safety_limit_w = config->get_motor_safety_limit_w() * D2R;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 211; // cmd motor init

        memcpy(&send_byte[8], &safety_limit_v, 4); // m/s
        memcpy(&send_byte[12], &safety_limit_w, 4); // rad/s

        send_byte[24] = 0x25;

        if(is_connected && !config->get_use_sim())
        {
            msg_que.push(send_byte);
        }
    }

    // set init
    {
        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 100; // cmd motor init


        send_byte[24] = 0x25;

        if(is_connected && !config->get_use_sim())
        {
            msg_que.push(send_byte);
        }
    }

    logger->write_log("[MOBILE] motor lock on", "Green", true, false);
}

void MOBILE::motor_off()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 120; // cmd

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }

    logger->write_log("[MOBILE] motor lock off", "Green", true, false);
}

void MOBILE::move(double vx, double vy, double wz)
{
    // set last v,w
    vx0 = vx;
    vy0 = vy;
    wz0 = wz;

    bool is_inter_lock_foot = get_is_inter_lock_foot();
    if(is_inter_lock_foot)
    {
        logger->write_log("[MOBILE] motor inter lock foot", "Orange");
        return;
    }

    // packet
    float _vx = vx;
    float _vy = vy;
    float _wz = wz;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 10; // cmd move

    RobotModel robot_model = config->get_robot_model();
    if(robot_model == RobotModel::MECANUM)
    {
        memcpy(&send_byte[8],  &_vx, 4);  // param1 linear vel
        memcpy(&send_byte[12], &_vy, 4); // param2 linear vel
        memcpy(&send_byte[16], &_wz, 4); // param3 angular vel
    }
    else
    {
        memcpy(&send_byte[8],  &_vx, 4);  // param1 linear vel
        memcpy(&send_byte[12], &_wz, 4); // param2 angular vel
    }

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::move_linear_x(double d, double v)
{
    // set last v,w
    vx0 = v;
    vy0 = 0;
    wz0 = 0;

    // packet
    float _d = d;
    float _v = v;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 117; // cmd move linear x

    memcpy(&send_byte[8], &_d, 4); // param1 dist
    memcpy(&send_byte[12], &_v, 4); // param2 linear vel
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::move_linear_y(double d, double v)
{
    RobotModel robot_model = config->get_robot_model();
    if(robot_model != RobotModel::MECANUM)
    {
        return;
    }

    // set last v,w
    vx0 = 0;
    vy0 = 0;
    wz0 = 0;

    // packet
    float _d = d;
    float _v = v;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 118; // cmd move linear y

    memcpy(&send_byte[8], &_d, 4); // param1 dist
    memcpy(&send_byte[12], &_v, 4); // param2 linear vel
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::stop_charge()
{
    RobotModel robot_model = config->get_robot_model();
    if(robot_model != RobotModel::MECANUM)
    {
        return;
    }

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 130; // cmd stop charge
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::move_rotate(double th, double w)
{
    // set last v,w
    vx0 = 0;
    vy0 = 0;
    wz0 = w;

    // packet
    float _th = th;
    float _w = w;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;

    RobotModel robot_model = config->get_robot_model();
    if(robot_model == RobotModel::MECANUM)
    {
        send_byte[7] = 119; // cmd move rotate
    }
    else
    {
        send_byte[7] = 118; // cmd move rotate
    }

    memcpy(&send_byte[8], &_th, 4); // param1 rad
    memcpy(&send_byte[12], &_w, 4); // param2 angular vel
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::stop()
{
    // set last v,w
    vx0 = 0;
    vy0 = 0;
    wz0 = 0;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 20; // cmd stop
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.clear();
        msg_que.push(send_byte);
    }
}

void MOBILE::led(int target, int mode)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xB0;
    send_byte[6] = target; // 0~1
    send_byte[7] = mode; // cmd move

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::time_sync()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 121; // cmd

    float time = get_time();
    memcpy(&send_byte[8], &time, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}


//for safety
void MOBILE::robot_initialize()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 100; // cmd

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::robot_request()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA2;
    send_byte[6] = 0x00; // 0~1

    send_byte[7] = 0x00; // cmd

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearmismatch()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd

    int para1 = 0x01;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearoverspd()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd

    int para1 = 0x02;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearobs()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd

    int para1 = 0x03;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearfieldmis()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd


    int para1 = 0x04;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearinterlockstop()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd


    int para1 = 0x05;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::clearbumperstop()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x01; // cmd


    int para1 = 0x06;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}


void MOBILE::recover()
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x02; // cmd

    int para1 = 0x00;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::setlidarfield(unsigned int field)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 210; // cmd

    unsigned int para1 = field;
    memcpy(&send_byte[8], &para1, 4); // param1 rad
    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_limit_v_acc(double v, double a, double w, double b)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 108; // cmd

    float v_limit_para = (float)v;
    float a_limit_para = (float)a;
    float w_limit_para = (float)w;
    float b_limit_para = (float)b;

    memcpy(&send_byte[8], &v_limit_para, 4);
    memcpy(&send_byte[12], &a_limit_para, 4);
    memcpy(&send_byte[16], &w_limit_para, 4);
    memcpy(&send_byte[20], &b_limit_para, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_limit_v_acc_jog(double v, double a, double w, double b)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 212; // cmd

    float v_limit_jog_para = (float)v;
    float a_limit_jog_para = (float)a;
    float w_limit_jog_para = (float)w;
    float b_limit_jog_para = (float)b;


    memcpy(&send_byte[8], &v_limit_jog_para, 4);
    memcpy(&send_byte[12], &a_limit_jog_para, 4);
    memcpy(&send_byte[16], &w_limit_jog_para, 4);
    memcpy(&send_byte[20], &b_limit_jog_para, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_safety_v_acc_monitor(double v, double w)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 211; // cmd

    float v_limit_moni = (float)v;
    float w_limit_moni = (float)w;

    memcpy(&send_byte[8], &v_limit_moni, 4);
    memcpy(&send_byte[12], &w_limit_moni, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_safety_v_acc(double v, double w)
{

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 213; // cmd

    float safety_v_limit = (float)v;
    float safety_w_limit = (float)w;

    memcpy(&send_byte[8], &safety_v_limit, 4);
    memcpy(&send_byte[12], &safety_w_limit, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_opmode(double opmode)
{

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 214; // cmd

    float safety_opmode = (float)opmode;

    memcpy(&send_byte[8], &safety_opmode, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_wheel(double w_s, double w_r)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 104; // cmd

    float _w_s = (float)w_s; //m
    float _w_r = (float)w_r; //m


    memcpy(&send_byte[8], &_w_s, 4);
    memcpy(&send_byte[12], &_w_r, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_motor_onoff(double param)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 216; // cmd

    float parameter = (float)param; // 1 - power off  || 2 - power on

    memcpy(&send_byte[8], &parameter, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::make_ref_offset(double param)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 217; // cmd

    float parameter = (float)param; // 1 - power off  || 2 - power on

    memcpy(&send_byte[8], &parameter, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_detect_mode(double param)
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 218; // cmd

    float parameter = (float)param; // 1 -detect mode || 0 - detect mode not used

    memcpy(&send_byte[8], &parameter, 4);

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }

}

void MOBILE::lift_power_onoff(int param)
{

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xC0;
    send_byte[6] = 0x01; // lift -target
    send_byte[7] = 0x00; // cmd

    int para1 = param; // 1 on - 0 off

    memcpy(&send_byte[8], &para1, 4);
    send_byte[24] = 0x25;

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_IO_output(unsigned char [])
{
    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA1;
    send_byte[6] = 0x00; // 0~1
    send_byte[7] = 0x00; // cmd

    for(int i=0; i<16; i++)
    {
        send_byte[i+8] = cur_setting.d_out[i];
    }

    send_byte[24] = 0x25;

    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::set_IO_individual_output(unsigned char target, unsigned int n)
{

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA1;

    //notice
    //0~7 : MCU1 I/O pin
    //8~15: MCU2 I/O pin
    send_byte[6] = target; // target - 0 ~15
    send_byte[7] = 0x01; // command

    // n = 0 low
    // n = 1 high
    unsigned int para1 = n;
    memcpy(&send_byte[8], &para1, 4);

    send_byte[24] = 0x25;


    if(is_connected && !config->get_use_sim())
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::sem_io_speaker(unsigned int speak_num)
{
    uint8_t io_bitmask = 0;
    io_bitmask = speak_num & 0x0F;

    for (int i = 0; i < 4; ++i)
    {
        bool new_state = (io_bitmask >> i) & 0x01;

        if (speaker_io_state[i] != new_state)
        {
            speaker_io_state[i] = new_state;
            set_IO_individual_output(static_cast<unsigned char>(i), new_state ? 1 : 0);
        }
    }

}


// send loop
void MOBILE::send_loop()
{
    printf("[MOBILE] send loop start\n");
    while(send_flag)
    {
        if(is_connected)
        {
            // send
            std::vector<uchar> msg;
            if(msg_que.try_pop(msg))
            {
                ::send(fd, msg.data(), msg.size(), 0);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[MOBILE] send loop stop\n");
}

int MOBILE::calc_battery_percentage(float voltage)
{
    RobotModel robot_model = config->get_robot_model();
    if(robot_model == RobotModel::D400 || robot_model == RobotModel::MECANUM)
    {
        if(voltage <= voltage_lookup_table.front().voltage)
        {
            return voltage_lookup_table.front().capacity;
        }
        if(voltage >= voltage_lookup_table.back().voltage)
        {
            return voltage_lookup_table.back().capacity;
        }

        for(int i = 0; i < voltage_lookup_table.size() - 1; i++)
        {
            float v1 = voltage_lookup_table[i].voltage;
            float v2 = voltage_lookup_table[i+1].voltage;
            if (voltage >= v1 && voltage <= v2)
            {
                float c1 = voltage_lookup_table[i].capacity;
                float c2 = voltage_lookup_table[i+1].capacity;
                float ratio = (voltage - v1) / (v2 - v1 + 1e-6);

                return static_cast<int>(c1 + ratio * (c2 - c1) + 0.5f);
            }
        }
        return 0;
    }
    else if(robot_model == RobotModel::S100)
    {
        if(voltage >= S100_BAT_MAX_VOLTAGE)
        {
            return 100;
        }
        if(voltage <= S100_BAT_MIN_VOLTAGE)
        {
            return 0;
        }
        double percentage = (voltage - S100_BAT_MIN_VOLTAGE) / (S100_BAT_MAX_VOLTAGE - S100_BAT_MIN_VOLTAGE + 1e-6) * 100.0;

        return static_cast<int>(percentage + 0.5);
    }
    else
    {
        //printf("[MOBILE] Unknown ROBOT_PLATFORM: %s\n", platform_type.toStdString().c_str());
        return 0;
    }
}

void MOBILE::set_config_module(CONFIG* _config)
{
    config = _config;
}

void MOBILE::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}
