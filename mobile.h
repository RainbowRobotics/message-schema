#ifndef MOBILE_H
#define MOBILE_H

#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"

// third party
#include "complementary_filter.h"

// socket
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/tcp.h>

// qt
#include <QObject>

class MOBILE : public QObject
{
    Q_OBJECT
public:
    explicit MOBILE(QObject *parent = nullptr);
    ~MOBILE();

    // mutex
    std::mutex mtx;

    // socket
    int fd = 0;

    // interface func
    void open();
    void sync();

    MOBILE_POSE get_pose();    
    MOBILE_STATUS get_status();
    Eigen::Vector3d get_imu();
    Eigen::Vector3d get_control_input();
    std::vector<MOBILE_IMU> get_imu_storage();
    std::vector<MOBILE_POSE> get_pose_storage();
    int get_pose_storage_size();
    int get_imu_storage_size();

    // command func
    void motor_on();
    void motor_off();
    void move(double vx, double vy, double wz);
    void move_linear(double d, double v);
    void move_rotate(double th, double w);
    void led(int target, int mode);
    void time_sync();

    // recv loop
    std::atomic<bool> recv_flag = {false};
    std::thread* recv_thread = NULL;
    void recv_loop();

    // send loop
    std::atomic<bool> send_flag = {false};
    std::thread* send_thread = NULL;
    void send_loop();

    // storage
    tbb::concurrent_queue<std::vector<uchar>> msg_que;
    std::vector<MOBILE_POSE> pose_storage;
    std::vector<MOBILE_IMU> imu_storage;

    // vars
    MOBILE_POSE cur_pose;
    MOBILE_STATUS cur_status;
    Eigen::Vector3d cur_imu;

    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};
    std::atomic<bool> is_synced = {false};
    std::atomic<double> sync_st_time = {0};
    std::atomic<double> offset_t = {0};

    std::atomic<double> last_pose_t = {0};
    std::atomic<double> last_imu_t = {0};

    // last control input
    double vx0 = 0;
    double vy0 = 0;
    double wz0 = 0;

    // other modules
    CONFIG* config = NULL;
    LOGGER* logger = NULL;

    // for simulation
    std::atomic<bool> is_sim = {false};

Q_SIGNALS:

};

#endif // MOBILE_H
