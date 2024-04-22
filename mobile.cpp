#include "mobile.h"

MOBILE::MOBILE(QObject *parent)
    : QObject{parent}
{

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
    printf("[MOBILE] connect\n");

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

    printf("[MOBILE] send time sync, sync_st_time:%f\n", (double)sync_st_time);
}

MOBILE_POSE MOBILE::get_pose()
{
    mtx.lock();
    MOBILE_POSE res = cur_pose;
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

Eigen::Vector3d MOBILE::get_rpy()
{
    mtx.lock();
    Eigen::Vector3d res = cur_rpy;
    mtx.unlock();

    return res;
}

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

        if(is_connected)
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

        if(is_connected)
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

        if(is_connected)
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

        if(is_connected)
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

        if(is_connected)
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

        if(is_connected)
        {
            msg_que.push(send_byte);
        }
    }

    printf("[MOBILE] motor lock on\n");
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

    if(is_connected)
    {
        msg_que.push(send_byte);
    }

    printf("[MOBILE] motor lock off\n");
}

void MOBILE::move(double vx, double vy, double wz)
{
    // set last v,w
    vx0 = vx;
    vy0 = vy;
    wz0 = wz;

    // packet
    float _v = vx;
    float _w = wz;

    std::vector<uchar> send_byte(25, 0);
    send_byte[0] = 0x24;

    uint16_t size = 6+8+8;
    memcpy(&send_byte[1], &size, 2); // size
    send_byte[3] = 0x00;
    send_byte[4] = 0x00;

    send_byte[5] = 0xA0;
    send_byte[6] = 0x00;
    send_byte[7] = 10; // cmd move

    memcpy(&send_byte[8], &_v, 4); // param1 linear vel
    memcpy(&send_byte[12], &_w, 4); // param2 angular vel
    send_byte[24] = 0x25;

    if(is_connected)
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

    if(is_connected)
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

    if(is_connected)
    {
        msg_que.push(send_byte);
    }
}

// recv loop
void MOBILE::recv_loop()
{
    // for imu
    imu_tools::ComplementaryFilter imu_filter;

    // pdu connection info
    const QString pdu_ip = "192.168.2.100";
    const int pdu_port = 1977;

    // socket
    sockaddr_in server_addr;
    bzero((char*)&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(pdu_ip.toLocal8Bit().data());
    server_addr.sin_port = htons(pdu_port);
    printf("[MOBILE] try connect, ip:%s, port:%d\n", pdu_ip.toLocal8Bit().data(), pdu_port);

    // connection
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if(fd < 0)
    {
        printf("[MOBILE] socket create failed\n");
        return;
    }

    // set option
    {
        int val = 1;
        ::setsockopt(fd, SOL_TCP, TCP_NODELAY, &val, sizeof(val));
    }

    int status = ::connect(fd, (sockaddr*)&server_addr, sizeof(server_addr));
    if(status < 0)
    {
        printf("[MOBILE] connect failed\n");
        return;
    }

    is_connected = true;
    printf("[MOBILE] connected\n");

    // var init
    const int packet_size = 134;
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
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        while((int)buf.size() >= packet_size)
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

                uint32_t recv_tick;
                memcpy(&recv_tick, &_buf[index], dlc_f);        index=index+dlc_f;

                float return_time;
                memcpy(&return_time, &_buf[index], dlc_f);      index=index+dlc_f;

                uint8_t rolloer_controller_state;
                memcpy(&rolloer_controller_state, &_buf[index], dlc);      index=index+dlc;

                uint8_t rolloer_sensor0, rolloer_sensor1, rolloer_sensor2, rolloer_sensor3;
                memcpy(&rolloer_sensor0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_sensor1, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_sensor2, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_sensor3, &_buf[index], dlc);      index=index+dlc;

                uint8_t rolloer_manual_sw0, rolloer_manual_sw1;
                memcpy(&rolloer_manual_sw0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_manual_sw1, &_buf[index], dlc);      index=index+dlc;

                uint8_t rolloer_blocking_state0, rolloer_blocking_state1;
                memcpy(&rolloer_blocking_state0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_blocking_state1, &_buf[index], dlc);      index=index+dlc;

                uint8_t rolloer_blocking_manual_sw0, rolloer_blocking_manual_sw1, rolloer_blocking_manual_sw2, rolloer_blocking_manual_sw3;
                memcpy(&rolloer_blocking_manual_sw0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_blocking_manual_sw1, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_blocking_manual_sw2, &_buf[index], dlc);      index=index+dlc;
                memcpy(&rolloer_blocking_manual_sw3, &_buf[index], dlc);      index=index+dlc;

                uint8_t orgo_on_init;
                memcpy(&orgo_on_init, &_buf[index], dlc);      index=index+dlc;

                uint8_t orgo_on_run;
                memcpy(&orgo_on_run, &_buf[index], dlc);      index=index+dlc;

                uint8_t orgo_pos_state0, orgo_pos_state1;
                memcpy(&orgo_pos_state0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&orgo_pos_state1, &_buf[index], dlc);      index=index+dlc;

                uint8_t orgo_manual_sw0, orgo_manual_sw1;
                memcpy(&orgo_manual_sw0, &_buf[index], dlc);      index=index+dlc;
                memcpy(&orgo_manual_sw1, &_buf[index], dlc);      index=index+dlc;

                uint32_t orgo_pos0, orgo_pos1;
                memcpy(&orgo_pos0, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&orgo_pos1, &_buf[index], dlc_f);      index=index+dlc_f;

                float imu_gyr_x, imu_gyr_y, imu_gyr_z;
                memcpy(&imu_gyr_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_gyr_z, &_buf[index], dlc_f);      index=index+dlc_f;

                float imu_acc_x, imu_acc_y, imu_acc_z;
                memcpy(&imu_acc_x, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_y, &_buf[index], dlc_f);      index=index+dlc_f;
                memcpy(&imu_acc_z, &_buf[index], dlc_f);      index=index+dlc_f;

                // calc time offset
                if(is_sync && pc_t > sync_st_time + 0.1)
                {
                    is_sync = false;

                    double _offset_t = return_time - recv_tick*0.002;
                    offset_t = _offset_t;

                    printf("[MOBILE] sync, offset_t: %f\n", (double)offset_t);
                }

                // received mobile pose update
                MOBILE_POSE mobile_pose;
                mobile_pose.t = mobile_t + offset_t;
                mobile_pose.pose = Eigen::Vector3d(x, y, toWrap(th));
                mobile_pose.vel = Eigen::Vector3d(x_dot, y_dot, th_dot);
                mobile_pose.vw = Eigen::Vector2d(local_v, local_w);

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
                mobile_status.power = power;
                mobile_status.total_power = total_used_power;
                mobile_status.recv_tick = recv_tick;
                mobile_status.return_time = return_time;

                // roller
                mobile_status.roller_controller_state = rolloer_controller_state;
                mobile_status.roller_sensor0 = rolloer_sensor0;
                mobile_status.roller_sensor1 = rolloer_sensor1;
                mobile_status.roller_sensor2 = rolloer_sensor2;
                mobile_status.roller_sensor3 = rolloer_sensor3;
                mobile_status.roller_manual_sw0 = rolloer_manual_sw0;
                mobile_status.roller_manual_sw1 = rolloer_manual_sw1;
                mobile_status.roller_blocking_state0 = rolloer_blocking_state0;
                mobile_status.roller_blocking_state1 = rolloer_blocking_state1;
                mobile_status.roller_blocking_manual_sw0 = rolloer_blocking_manual_sw0;
                mobile_status.roller_blocking_manual_sw1 = rolloer_blocking_manual_sw1;
                mobile_status.roller_blocking_manual_sw2 = rolloer_blocking_manual_sw2;
                mobile_status.roller_blocking_manual_sw3 = rolloer_blocking_manual_sw3;
                mobile_status.orgo_on_init = orgo_on_init;
                mobile_status.orgo_on_run = orgo_on_run;
                mobile_status.orgo_pos_state0 = orgo_pos_state0;
                mobile_status.orgo_pos_state1 = orgo_pos_state1;
                mobile_status.orgo_manual_sw0 = orgo_manual_sw0;
                mobile_status.orgo_manual_sw1 = orgo_manual_sw1;
                mobile_status.orgo_pos1 = orgo_pos1;
                mobile_status.orgo_pos0 = orgo_pos0;

                // imu
                mobile_status.imu_gyr_x = imu_gyr_x * D2R;
                mobile_status.imu_gyr_y = imu_gyr_y * D2R;
                mobile_status.imu_gyr_z = imu_gyr_z * D2R;
                mobile_status.imu_acc_x = imu_acc_x * ACC_G;
                mobile_status.imu_acc_y = imu_acc_y * ACC_G;
                mobile_status.imu_acc_z = imu_acc_z * ACC_G;

                // get orientation using complementary filter
                Eigen::Vector3d r(0, 0, 0);
                if(std::abs(imu_gyr_x) > 0 || std::abs(imu_gyr_y) > 0 || std::abs(imu_gyr_z) > 0 ||
                   std::abs(imu_acc_x) > 0 || std::abs(imu_acc_y) > 0 || std::abs(imu_acc_z) > 0)
                {
                    double q0, q1, q2, q3;
                    imu_filter.update(mobile_status.imu_acc_x, mobile_status.imu_acc_y, mobile_status.imu_acc_z,
                                      mobile_status.imu_gyr_x, mobile_status.imu_gyr_y, mobile_status.imu_gyr_z, 0.05);
                    imu_filter.getOrientation(q0, q1, q2, q3);
                    Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
                    r = Sophus::SO3d(R).log();
                }

                // storing
                mtx.lock();
                cur_pose = mobile_pose;
                cur_status = mobile_status;
                cur_rpy = r;

                pose_storage.push_back(mobile_pose);
                if(pose_storage.size() > 300)
                {
                    pose_storage.erase(pose_storage.begin());
                }
                mtx.unlock();
            }

            // erase used packet
            buf.erase(buf.begin(), buf.begin() + packet_size);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
}
