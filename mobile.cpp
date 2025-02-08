#include "mobile.h"

MOBILE::MOBILE(QObject *parent)
    : QObject{parent}
{
    cur_imu = Eigen::Vector3d(0,0,0);
}

MOBILE::~MOBILE()
{
    is_connected = false;
    msg_que.clear();

    // loops destroy
    if(recv_thread != NULL)
    {
        recv_flag = false;
        recv_thread->join();
        recv_thread = NULL;
    }

    if(send_thread != NULL)
    {
        send_flag = false;
        send_thread->join();
        send_thread = NULL;
    }
}

// interface func
void MOBILE::open()
{
    // check simulation mode
    if(config->USE_SIM == 1)
    {
        printf("[MOBILE] simulation mode\n");
        return;
    }

    // start loops
    if(recv_thread == NULL)
    {
        recv_flag = true;
        recv_thread = new std::thread(&MOBILE::recv_loop, this);
    }

    if(send_thread == NULL)
    {
        send_flag = true;
        send_thread = new std::thread(&MOBILE::send_loop, this);
    }
}

void MOBILE::sync()
{
    time_sync();
    sync_st_time = get_time();
    is_sync = true;

    QString str;
    str.sprintf("[MOBILE] time sync, sync_st_time:%f", (double)sync_st_time);
    logger->write_log(str, "DeepSkyBlue", true, false);
}

QString MOBILE::get_cur_pdu_state()
{
    mtx.lock();
    QString res = cur_pdu_state; // none, fail, good
    mtx.unlock();

    return res;
}

void MOBILE::set_cur_pdu_state(QString str)
{
    mtx.lock();
    cur_pdu_state = str; // none, fail, good
    mtx.unlock();
}

MOBILE_POSE MOBILE::get_pose()
{
    mtx.lock();
    MOBILE_POSE res = cur_pose;
    mtx.unlock();

    return res;
}

MOBILE_POSE MOBILE::get_best_mo(double ref_t)
{
    mtx.lock();

    MOBILE_POSE res;
    double min_dt = 99999999;
    for(size_t p = 0; p < pose_storage.size(); p++)
    {
        double dt = std::abs(pose_storage[p].t - ref_t);
        if(dt < min_dt)
        {
            min_dt = dt;
            res = pose_storage[p];
        }
    }

    mtx.unlock();

    return res;
}

MOBILE_STATUS MOBILE::get_status()
{
    mtx.lock();
    MOBILE_STATUS res = cur_status;
    mtx.unlock();

    return res;
}

Eigen::Vector3d MOBILE::get_imu()
{
    mtx.lock();
    Eigen::Vector3d res = cur_imu;
    mtx.unlock();

    return res;
}

Eigen::Vector3d MOBILE::get_control_input()
{
    mtx.lock();
    Eigen::Vector3d res(vx0, vy0, wz0);
    mtx.unlock();

    return res;
}

std::vector<MOBILE_IMU> MOBILE::get_imu_storage()
{
    mtx.lock();
    std::vector<MOBILE_IMU> res = imu_storage;
    mtx.unlock();

    return res;
}

std::vector<MOBILE_POSE> MOBILE::get_pose_storage()
{
    mtx.lock();
    std::vector<MOBILE_POSE> res = pose_storage;
    mtx.unlock();

    return res;
}

int MOBILE::get_pose_storage_size()
{
    mtx.lock();
    int res = pose_storage.size();
    mtx.unlock();

    return res;
}

int MOBILE::get_imu_storage_size()
{
    mtx.lock();
    int res = imu_storage.size();
    mtx.unlock();

    return res;
}

// for plot
#if defined(USE_SRV)
QString MOBILE::get_pose_text()
{
    MOBILE_POSE mobile_pose = get_pose();
    Eigen::Vector3d imu = get_imu();

    QString str;
    str.sprintf("[MOBILE_POSE]\nt:%.3f\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f",
                mobile_pose.t,
                mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D);
    return str;
}

QString MOBILE::get_status_text()
{
    MOBILE_STATUS mobile_status = get_status();
    QString str;
    str.sprintf("[MOBILE_STATUS]\nconnection(m0,m1):%d,%d, status(m0,m1):%d,%d\ntemp(m0,m1): %d,%d, cur(m0,m1):%.2f,%.2f\ncharge,power,emo,remote:%d,%d,%d,%d\nBAT(in,out,cur,per):%.3f,%.3f,%.3f,%d %\npower:%.3f, total power:%.3f\ncore_temp(m0,m1,state): %f, %f, %d",
                mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1,
                (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                mobile_status.charge_state, mobile_status.power_state, mobile_status.emo_state, mobile_status.remote_state,
                mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,mobile_status.bat_percent,
                mobile_status.power, mobile_status.total_power,
                mobile_status.core_temp0, mobile_status.core_temp1, mobile_status.state);
    return str;
}
#endif

#if defined(USE_AMR_400_LAKI) || defined(USE_AMR_400)
QString MOBILE::get_pose_text()
{
    MOBILE_POSE mobile_pose = get_pose();
    Eigen::Vector3d imu = get_imu();

    QString str;
    str.sprintf("[MOBILE_POSE]\nt:%.3f\npos:%.2f,%.2f,%.2f\nvel:%.2f, %.2f, %.2f,\nimu:%.2f, %.2f, %.2f",
                mobile_pose.t,
                mobile_pose.pose[0], mobile_pose.pose[1], mobile_pose.pose[2]*R2D,
                mobile_pose.vel[0], mobile_pose.vel[1], mobile_pose.vel[2]*R2D,
                imu[0]*R2D, imu[1]*R2D, imu[2]*R2D);
    return str;
}

QString MOBILE::get_status_text()
{
    MOBILE_STATUS mobile_status = get_status();
    QString str;
    str.sprintf("[MOBILE_STATUS]\nconnection(m0,m1):%d,%d, status(m0,m1):%d,%d\ntemp(m0,m1): %d,%d,(%d,%d), cur(m0,m1):%.2f,%.2f\ncharge,power,emo,remote:%d,%d,%d,%d\nBAT(in,out,cur,per):%.3f,%.3f,%.3f,%d %\npower:%.3f, total power:%.3f, c.c:%.3f, c.v:%.3f\ngyr:%.2f,%.2f,%.2f acc:%.3f,%.3f,%.3f",
                mobile_status.connection_m0, mobile_status.connection_m1, mobile_status.status_m0, mobile_status.status_m1, mobile_status.temp_m0, mobile_status.temp_m1, mobile_status.esti_temp_m0, mobile_status.esti_temp_m1,
                (double)mobile_status.cur_m0/10.0, (double)mobile_status.cur_m1/10.0,
                mobile_status.charge_state, mobile_status.power_state, mobile_status.emo_state, mobile_status.remote_state,
                mobile_status.bat_in, mobile_status.bat_out, mobile_status.bat_current,mobile_status.bat_percent,
                mobile_status.power, mobile_status.total_power, mobile_status.charge_current, mobile_status.contact_voltage,
                mobile_status.imu_gyr_x, mobile_status.imu_gyr_y, mobile_status.imu_gyr_z,
                mobile_status.imu_acc_x, mobile_status.imu_acc_y, mobile_status.imu_acc_z);
    return str;
}
#endif

#if defined(USE_SRV)
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
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(pdu_ip.toLocal8Bit().data());
    server_addr.sin_port = htons(pdu_port);

    QString str;
    str.sprintf("[MOBILE] try connect, ip:%s, port:%d\n", pdu_ip.toLocal8Bit().data(), pdu_port);
    logger->write_log(str, "DeepSkyBlue", true, false);

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
    if (status <= 0) // timeout or error
    {
        logger->write_log("[MOBILE] connect timeout or error", "Red", true, false);
        close(fd);
        return;
    }

    // set back to blocking mode
    fcntl(fd, F_SETFL, flags);

    is_connected = true;
    logger->write_log("[MOBILE] connected", "Green", true, false);

    // var init
    const int packet_size = 92;
    std::vector<uchar> buf;
    int drop_cnt = 10;

    printf("[MOBILE] recv loop start\n");
    while(recv_flag)
    {
        // storing buffer
        std::vector<uchar> recv_buf(2000, 0);
        int num = read(fd, (char*)recv_buf.data(), recv_buf.size());
        if(num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // initial drop
        if(drop_cnt > 0)
        {
            drop_cnt--;
            continue;
        }

        // storing packet
        buf.insert(buf.end(), recv_buf.begin(), recv_buf.begin()+num);

        // parsing
        while((int)buf.size() >= packet_size && recv_flag)
        {
            uchar *_buf = (uchar*)buf.data();
            if(_buf[0] == 0x24 && _buf[5] == 0xA2 && _buf[packet_size-1] == 0x25)
            {
                int index=6;
                int dlc=1;
                int dlc_f=4;

                uint32_t tick;
                memcpy(&tick, &_buf[index], dlc_f);     index=index+dlc_f;
                double mobile_t = tick*0.002;
                double pc_t = get_time();

                uint32_t recv_tick;
                memcpy(&recv_tick, &_buf[index], dlc_f);        index=index+dlc_f;

                float return_time;
                memcpy(&return_time, &_buf[index], dlc_f);      index=index+dlc_f;

                uint8_t connection_status_m0, connection_status_m1;
                connection_status_m0 = _buf[index];     index=index+dlc;
                connection_status_m1 = _buf[index];     index=index+dlc;

                float x_dot, y_dot, th_dot;
                memcpy(&x_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

                float x, y, th;
                memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

                float local_v, local_w;
                memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t stat_m0, stat_m1;
                stat_m0 = _buf[index];     index=index+dlc;
                stat_m1 = _buf[index];     index=index+dlc;

                uint8_t temp_m0, temp_m1;
                temp_m0 = _buf[index];     index=index+dlc;
                temp_m1 = _buf[index];     index=index+dlc;

                uint8_t cur_m0, cur_m1;
                cur_m0 = _buf[index];     index=index+dlc;
                cur_m1 = _buf[index];     index=index+dlc;

                uint8_t charge_state, power_state, emo_state, remote_state;
                charge_state = _buf[index];     index=index+dlc;
                power_state = _buf[index];      index=index+dlc;
                emo_state = _buf[index];        index=index+dlc;
                remote_state = _buf[index];     index=index+dlc;

                float bat_in, bat_out, bat_cur, power, total_used_power;
                memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
                memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
                memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

                float motor_core_temp0;
                float motor_core_temp1;
                memcpy(&motor_core_temp0, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&motor_core_temp1, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t state;
                state = _buf[index];     index=index+dlc;

                // calc time offset
                if(is_sync && pc_t > sync_st_time + 0.1)
                {
                    is_sync = false;

                    double _mobile_t = recv_tick*0.002;
                    double _offset_t = return_time - _mobile_t;
                    offset_t = _offset_t;

                    is_synced = true;
                    printf("[MOBILE] sync, offset_t: %f\n", (double)offset_t);
                }

                int bat_percent = cal_voltage(bat_out);

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
                mobile_status.status_m0 = stat_m0;
                mobile_status.status_m1 = stat_m1;
                mobile_status.temp_m0 = temp_m0;
                mobile_status.temp_m1 = temp_m1;
                mobile_status.cur_m0 = cur_m0;
                mobile_status.cur_m1 = cur_m1;
                mobile_status.charge_state = charge_state;
                mobile_status.power_state = power_state;
                mobile_status.emo_state = emo_state;
                mobile_status.remote_state = remote_state;
                mobile_status.bat_in = bat_in;
                mobile_status.bat_out = bat_out;
                mobile_status.bat_current = bat_cur;
                mobile_status.bat_percent = bat_percent;
                mobile_status.power = power;
                mobile_status.total_power = total_used_power;
                mobile_status.recv_tick = recv_tick;
                mobile_status.return_time = return_time;
                mobile_status.core_temp0 = motor_core_temp0;
                mobile_status.core_temp1 = motor_core_temp1;
                mobile_status.state = state;
                mobile_status.imu_acc_x = 0;
                mobile_status.imu_acc_y = 0;
                mobile_status.imu_acc_z = 0;
                mobile_status.imu_gyr_x = 0;
                mobile_status.imu_gyr_y = 0;
                mobile_status.imu_gyr_z = 0;

                MOBILE_IMU imu;
                imu.t = mobile_status.t;
                imu.acc_x = 0;
                imu.acc_y = 0;
                imu.acc_z = 0;
                imu.gyr_x = 0;
                imu.gyr_y = 0;
                imu.gyr_z = 0;
                imu.rx = 0;
                imu.ry = 0;
                imu.rz = 0;

                // storing
                mtx.lock();
                cur_pose = mobile_pose;
                cur_status = mobile_status;
                cur_imu = Eigen::Vector3d(0,0,0);

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

            // erase used packet
            buf.erase(buf.begin(), buf.begin() + packet_size);

            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MOBILE] recv loop stop\n");
}
#endif

#if defined(USE_AMR_400_LAKI) || defined(USE_AMR_400)
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
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(pdu_ip.toLocal8Bit().data());
    server_addr.sin_port = htons(pdu_port);

    QString str;
    str.sprintf("[MOBILE] try connect, ip:%s, port:%d\n", pdu_ip.toLocal8Bit().data(), pdu_port);
    logger->write_log(str, "DeepSkyBlue", true, false);

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
    if (status <= 0) // timeout or error
    {
        logger->write_log("[MOBILE] connect timeout or error", "Red", true, false);
        close(fd);
        return;
    }

    // set back to blocking mode
    fcntl(fd, F_SETFL, flags);

    is_connected = true;
    logger->write_log("[MOBILE] connected", "Green", true, false);

    // var init
    const int packet_size = 133;

    std::vector<uchar> buf;
    int drop_cnt = 10;

    printf("[MOBILE] recv loop start\n");
    while(recv_flag)
    {
        // storing buffer
        std::vector<uchar> recv_buf(2000, 0);
        int num = read(fd, (char*)recv_buf.data(), recv_buf.size());
        if(num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        //printf("recv_num : %d\n", num);

        // initial drop
        if(drop_cnt > 0)
        {
            drop_cnt--;
            continue;
        }

        // storing packet
        buf.insert(buf.end(), recv_buf.begin(), recv_buf.begin()+num);

        // parsing
        while((int)buf.size() >= packet_size && recv_flag)
        {
            uchar *_buf = (uchar*)buf.data();
            if(_buf[0] == 0x24 && _buf[5] == 0xA2 && _buf[packet_size-1] == 0x25)
            {
                int index=6;
                int dlc=1;
                int dlc_f=4;

                uint32_t tick;
                memcpy(&tick, &_buf[index], dlc_f);     index=index+dlc_f;
                double mobile_t = tick*0.002;

                uint32_t recv_tick;
                memcpy(&recv_tick, &_buf[index], dlc_f);        index=index+dlc_f;

                float return_time;
                memcpy(&return_time, &_buf[index], dlc_f);      index=index+dlc_f;

                uint8_t connection_status_m0, connection_status_m1;
                connection_status_m0 = _buf[index];     index=index+dlc;
                connection_status_m1 = _buf[index];     index=index+dlc;

                float x_dot, y_dot, th_dot;
                memcpy(&x_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y_dot, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th_dot, &_buf[index], dlc_f);    index=index+dlc_f;

                float x, y, th;
                memcpy(&x, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&y, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&th, &_buf[index], dlc_f);    index=index+dlc_f;

                float local_v, local_w;
                memcpy(&local_v, &_buf[index], dlc_f);     index=index+dlc_f;
                memcpy(&local_w, &_buf[index], dlc_f);     index=index+dlc_f;

                uint8_t stat_m0, stat_m1;
                stat_m0 = _buf[index];     index=index+dlc;
                stat_m1 = _buf[index];     index=index+dlc;

                uint8_t temp_m0, temp_m1;
                temp_m0 = _buf[index];     index=index+dlc;
                temp_m1 = _buf[index];     index=index+dlc;

                uint8_t esti_temp_m0, esti_temp_m1;
                memcpy(&esti_temp_m0, &_buf[index], dlc);     index=index+dlc;
                memcpy(&esti_temp_m1, &_buf[index], dlc);     index=index+dlc;

                uint8_t cur_m0, cur_m1;
                cur_m0 = _buf[index];     index=index+dlc;
                cur_m1 = _buf[index];     index=index+dlc;

                uint8_t charge_state, power_state, emo_state, remote_state;
                charge_state = _buf[index];     index=index+dlc;
                power_state = _buf[index];      index=index+dlc;
                emo_state = _buf[index];        index=index+dlc;
                remote_state = _buf[index];     index=index+dlc;

                float bat_in, bat_out, bat_cur, power, total_used_power;
                memcpy(&bat_in, &_buf[index], dlc_f);               index=index+dlc_f;
                memcpy(&bat_out, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&bat_cur, &_buf[index], dlc_f);              index=index+dlc_f;
                memcpy(&power, &_buf[index], dlc_f);                index=index+dlc_f;
                memcpy(&total_used_power, &_buf[index], dlc_f);     index=index+dlc_f;

                float charge_current, contact_voltage;
                memcpy(&charge_current, &_buf[index], dlc_f);               index=index+dlc_f;
                memcpy(&contact_voltage, &_buf[index], dlc_f);               index=index+dlc_f;

                float q0, q1, q2, q3;
                memcpy(&q0, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&q1, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&q2, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&q3, &_buf[index], dlc_f);      index=index+dlc_f;

                float imu_gyr_x, imu_gyr_y, imu_gyr_z;
                memcpy(&imu_gyr_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_z, &_buf[index], dlc_f);      index=index+dlc_f;

                float imu_acc_x, imu_acc_y, imu_acc_z;
                memcpy(&imu_acc_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_z, &_buf[index], dlc_f);      index=index+dlc_f;

                int bat_percent = cal_voltage(bat_out);
//                std::cout<<"bat_percent : "<<bat_percent;

                // calc time offset
                if(is_sync && get_time() > sync_st_time + 0.1)
                {
                    is_sync = false;

                    double _mobile_t = recv_tick*0.002;
                    double _offset_t = return_time - _mobile_t;
                    offset_t = _offset_t;
                    is_synced = true;
                    QString str; str.sprintf("[MOBILE] sync, offset_t: %f", (double)offset_t);
                    logger->write_log(str);
                }

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
                mobile_status.status_m0 = stat_m0;
                mobile_status.status_m1 = stat_m1;
                mobile_status.temp_m0 = temp_m0;
                mobile_status.temp_m1 = temp_m1;
                mobile_status.esti_temp_m0 = esti_temp_m0;
                mobile_status.esti_temp_m1 = esti_temp_m1;
                mobile_status.cur_m0 = cur_m0;
                mobile_status.cur_m1 = cur_m1;
                mobile_status.charge_state = charge_state;
                mobile_status.power_state = power_state;
                mobile_status.emo_state = emo_state;
                mobile_status.remote_state = remote_state;
                mobile_status.bat_in = bat_in;
                mobile_status.bat_out = bat_out;
                mobile_status.bat_current = bat_cur;
                mobile_status.bat_percent = bat_percent;
                mobile_status.power = power;
                mobile_status.total_power = total_used_power;
                mobile_status.charge_current = charge_current;
                mobile_status.contact_voltage = contact_voltage;
                mobile_status.recv_tick = recv_tick;
                mobile_status.return_time = return_time;

                // imu
                mobile_status.imu_gyr_x = imu_gyr_x * D2R;
                mobile_status.imu_gyr_y = imu_gyr_y * D2R;
                mobile_status.imu_gyr_z = imu_gyr_z * D2R;
                mobile_status.imu_acc_x = imu_acc_x * ACC_G;
                mobile_status.imu_acc_y = imu_acc_y * ACC_G;
                mobile_status.imu_acc_z = imu_acc_z * ACC_G;

                // get orientation
                Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
                //Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
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




                // storing
                mtx.lock();
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

            // erase used packet
            buf.erase(buf.begin(), buf.begin() + packet_size);

            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MOBILE] recv loop stop\n");
}
#endif

// command func
void MOBILE::motor_on()
{
    // set id
    {
        int id_l = config->MOTOR_ID_L;
        int id_r = config->MOTOR_ID_R;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 101; // cmd motor init

        memcpy(&send_byte[8], &id_r, 4);
        memcpy(&send_byte[12], &id_l, 4);
        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    // set wheel dir, gear ratio
    {
        float wheel_dir = config->MOTOR_DIR;
        float gear_ratio = config->MOTOR_GEAR_RATIO;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 105; // cmd motor init

        memcpy(&send_byte[8], &wheel_dir, 4);
        memcpy(&send_byte[12], &gear_ratio, 4);
        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    // set wheel base, wheel radius
    {
        float wheel_base = config->ROBOT_WHEEL_BASE;
        float wheel_radius = config->ROBOT_WHEEL_RADIUS;

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
        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    // set limit vel
    {
        float limit_v = config->MOTOR_LIMIT_V;
        float limit_w = config->MOTOR_LIMIT_W * D2R;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 102; // cmd motor init

        memcpy(&send_byte[8], &limit_v, 4);
        memcpy(&send_byte[12], &limit_w, 4);
        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    // set limit acc
    {
        float limit_v_acc = config->MOTOR_LIMIT_V_ACC;
        float limit_w_acc = config->MOTOR_LIMIT_W_ACC * D2R;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 103; // cmd motor init

        memcpy(&send_byte[8], &limit_v_acc, 4);
        memcpy(&send_byte[12], &limit_w_acc, 4);
        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    // set init + gain
    {
        float kp = config->MOTOR_GAIN_KP;
        float ki = config->MOTOR_GAIN_KI;
        float kd = config->MOTOR_GAIN_KD;

        std::vector<uchar> send_byte(25, 0);
        send_byte[0] = 0x24;

        uint16_t size = 6+8+8;
        memcpy(&send_byte[1], &size, 2); // size
        send_byte[3] = 0x00;
        send_byte[4] = 0x00;

        send_byte[5] = 0xA0;
        send_byte[6] = 0x00;
        send_byte[7] = 100; // cmd motor init

        memcpy(&send_byte[8], &kp, 4);
        memcpy(&send_byte[12], &ki, 4);
        memcpy(&send_byte[16], &kd, 4);

        send_byte[24] = 0x25;

        if(is_connected && config->USE_SIM == 0)
        {
            msg_que.push(send_byte);
        }
    }

    logger->write_log("[MOBILE] motor lock on", "DeepSkyBlue", true, false);
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

    if(is_connected && config->USE_SIM == 0)
    {
        msg_que.push(send_byte);
    }

    logger->write_log("[MOBILE] motor lock off", "DeepSkyBlue", true, false);
}

void MOBILE::move(double vx, double vy, double wz)
{
    // set last v,w
    vx0 = vx;
    vy0 = vy;
    wz0 = wz;

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

#if defined(USE_SRV) || defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
    memcpy(&send_byte[8], &_vx, 4); // param1 linear vel
    memcpy(&send_byte[12], &_wz, 4); // param2 angular vel
#endif

    send_byte[24] = 0x25;

    if(is_connected && config->USE_SIM == 0)
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::move_linear(double d, double v)
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
    send_byte[7] = 117; // cmd move linear

    memcpy(&send_byte[8], &_d, 4); // param1 dist
    memcpy(&send_byte[12], &_v, 4); // param2 linear vel
    send_byte[24] = 0x25;

    if(is_connected && config->USE_SIM == 0)
    {
        msg_que.push(send_byte);
    }
}

void MOBILE::stop_charge()
{
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
    if(is_connected && config->USE_SIM == 0)
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
    send_byte[7] = 118; // cmd move rotate

    memcpy(&send_byte[8], &_th, 4); // param1 rad
    memcpy(&send_byte[12], &_w, 4); // param2 angular vel
    send_byte[24] = 0x25;

    if(is_connected && config->USE_SIM == 0)
    {
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

    if(is_connected && config->USE_SIM == 0)
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

    if(is_connected && config->USE_SIM == 0)
    {
        msg_que.push(send_byte);
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

// 입력된 전압에 대응하는 용량 찾기
int MOBILE::cal_voltage(float voltage)
{
//    float f_voltage = voltage.toFloat();

    // 정확히 일치하는 전압을 찾기
    float capacity = -1;
    for (const auto& entry : volt_lookup_data)
    {
        if (voltage == entry.voltage)
        {
            capacity = entry.capacity;
            break;
        }
    }

    // 정확히 일치하는 값이 없으면 선형 보간법을 사용
    if (capacity == -1)
    {
        for (int i = 0; i < volt_lookup_data.size() - 1; ++i)
        {
            if (voltage > volt_lookup_data[i].voltage && voltage < volt_lookup_data[i + 1].voltage)
            {
                // 선형 보간법
                double x1 = volt_lookup_data[i].voltage;
                double y1 = volt_lookup_data[i].capacity;
                double x2 = volt_lookup_data[i + 1].voltage;
                double y2 = volt_lookup_data[i + 1].capacity;

                // 선형 보간 공식으로 용량 계산
                capacity = static_cast<int>(y1 + (voltage - x1) * (y2 - y1) / (x2 - x1));
                break;
            }
        }
    }

    return capacity;
}
