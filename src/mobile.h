#ifndef MOBILE_H
#define MOBILE_H

#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"

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
    std::recursive_mutex mtx;

    // socket
    int fd = 0;

    // interface func
    void open();
    void sync();
    QString get_cur_pdu_state();
    void set_cur_pdu_state(QString str);

    MOBILE_POSE get_pose();
    MOBILE_POSE get_best_mo(double ref_t);
    MOBILE_STATUS get_status();
    MOBILE_SETTING get_setting();
    QString get_pose_text();
    QString get_status_text();
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
    void move_linear_x(double d, double v);
    void move_linear_y(double d, double v);
    void move_rotate(double th, double w);
    void stop();
    void led(int target, int mode);
    void time_sync();
    void stop_charge();

    // command safety
    void robot_initialize();
    void robot_request();
    void clearmismatch();
    void clearoverspd();
    void clearobs();
    void clearfieldmis();
    void clearinterlockstop();
    void clearbumperstop();
    void recover();
    void setlidarfield(unsigned int n);
    void set_limit_v_acc(double v, double a, double w, double b);
    void set_limit_v_acc_jog(double v, double a, double w, double b);
    void set_safety_v_acc_monitor(double v, double w);
    void set_safety_v_acc(double v,double w);
    void set_opmode(double opmode);
    void set_wheel(double w_s, double w_r);
    void set_motor_onoff(double param);
    void make_ref_offset(double var);
    void set_detect_mode(double var);
    void set_IO_output(unsigned char[]);


    // recv loop
    std::atomic<bool> recv_flag = {false};
    std::thread* recv_thread = NULL;
    void recv_loop();

    int calc_battery_percentage(float voltage);

    // send loop
    std::atomic<bool> send_flag = {false};
    std::thread* send_thread = NULL;
    void send_loop();

    // storage
    tbb::concurrent_queue<std::vector<uchar>> msg_que;
    std::vector<MOBILE_POSE> pose_storage;
    std::vector<MOBILE_IMU> imu_storage;

    std::atomic<bool> is_first_receive = {true};
    double input_voltage = 999.0;

    // vars
    MOBILE_POSE cur_pose;
    MOBILE_STATUS cur_status;
    MOBILE_SETTING cur_setting;
    Eigen::Vector3d cur_imu;
    QString cur_pdu_state = "none";

    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};
    std::atomic<bool> is_synced = {false};
    std::atomic<double> sync_st_time = {0};
    std::atomic<double> offset_t = {0};

    std::atomic<double> last_pose_t = {0};
    std::atomic<double> last_imu_t = {0};

    // last control input
    std::atomic<double> vx0 = {0};
    std::atomic<double> vy0 = {0};
    std::atomic<double> wz0 = {0};

    // other modules
    CONFIG* config = NULL;
    LOGGER* logger = NULL;


    QString pose_text;
    QString status_text;
};

#endif // MOBILE_H
