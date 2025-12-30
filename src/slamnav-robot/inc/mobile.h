#ifndef MOBILE_H
#define MOBILE_H

#include "slamnav_robot_types.h"
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
#include <QGamepad>
#include <QGamepadManager>

enum class RobotWheelModel
{
    ROBOT_WHEEL_MODEL_UNKNOWN = 0,
    ROBOT_WHEEL_MODEL_DD = 1,
    ROBOT_WHEEL_MODEL_QD= 2,
    ROBOT_WHEEL_MODEL_MECANUM = 3
};


enum class RobotType_PDU
{
    ROBOT_TYPE_UNKNOWN = 0,
    ROBOT_TYPE_D400    = 1,
    ROBOT_TYPE_S100    = 2,
    ROBOT_TYPE_MECANUM = 3,
    ROBOT_TYPE_SAFETY  = 4,
    ROBOT_TYPE_SAFETY_V2 = 5
};

struct MOBILE_INFO
{
    static constexpr int drop_cnt = 10;
    static constexpr int min_packet_size     = 4;
    static constexpr int packet_size_s100    = 85;
    static constexpr int packet_size_d400    = 126;
    static constexpr int packet_size_mecanum = 129;
    static constexpr int packet_size_safety  = 199;
    static constexpr int packet_size_safety_v2_high = 100;
    static constexpr int packet_size_safety_v2_mid = 50;
    static constexpr int packet_size_safety_v2_low = 150;

    static constexpr double pdu_tick_resolution = 0.002;

    static constexpr size_t recv_buf_size = 2000;
};

class MOBILE : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(MOBILE)
public:
    // make singleton
    static MOBILE* instance(QObject* parent = nullptr);

    // start mobile module
    void open();

    // software sync to pdu & pc
    void sync();

    void init_gamepad();

    /***********************
     * interface funcs
     ***********************/
    int get_pose_storage_size();
    int get_imu_storage_size();
    bool get_is_connected();
    bool get_is_synced();
    bool get_is_inter_lock_foot();
    bool get_is_speaker_on();
    double get_last_pose_t();
    double get_process_time_mobile();
    float get_res_linear_dist();
    float get_res_linear_remain_dist();
    QString get_cur_pdu_state();
    QString get_status_text();
    QString get_pose_text();
    bool get_is_auto_move();
    bool get_is_jog_pressed();

    double get_move_distance();

    double get_battery_soc();

    MOBILE_POSE get_pose();
    MOBILE_POSE get_best_mo(double ref_t);
    MOBILE_STATUS get_status();
    MOBILE_SETTING get_setting();
    Eigen::Vector3d get_imu();
    Eigen::Vector3d get_control_input();
    std::vector<MOBILE_IMU> get_imu_storage();
    std::vector<MOBILE_POSE> get_pose_storage();

    void set_is_synced(bool val);
    void set_is_connected(bool val);
    void set_cur_pdu_state(QString str);
    void set_is_inter_lock_foot(bool val);
    void set_is_auto_move(bool val);
    void set_is_jog_pressed(bool val);

    void set_jog_limit_v_acc(double val);
    void set_jog_limit_v_dcc(double val);
    void set_jog_limit_w_acc(double val);
    void set_jog_limit_w_dcc(double val);

    // this func only use simulation mode
    void set_cur_pose(MOBILE_POSE mp);
    void set_cur_status(MOBILE_STATUS ms);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);

    /***********************
     * mobile command (common)
     ***********************/
    void motor_on();
    void motor_off();
    void move(double vx, double vy, double wz);
    void moveQD(double vx, double vy, double wz, int hpp_side);
    void move_linear_x(double d, double v);
    void move_linear_y(double d, double v);
    void move_circular(double th, double w, int dir);
    void move_rotate(double th, double w);
    void stop();
    void led(int target, int mode);
    void time_sync();
    void stop_charge();

    /***********************
     * mobile command (safty)
     ***********************/
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
    void set_lx_ly(double lx, double ly);
    void set_opmode(double opmode);
    void set_wheel(double w_s, double w_r);
    void set_motor_onoff(double param);
    void make_ref_offset(double var);
    void set_detect_mode(double var);
    void set_IO_output(unsigned char[]);
    void set_IO_individual_output(unsigned char target, unsigned int n);
    void lift_power_onoff(int param);
    void sem_io_speaker(unsigned int speak_num);
    void config_parameter_send();
    void set_safety_parameter(int target, bool param);
    void xnergy_command(int command, float param);
private:
    explicit MOBILE(QObject *parent = nullptr);
    ~MOBILE();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;

    // socket
    int fd = 0;

    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);

    // recv loop
    std::atomic<bool> recv_flag = {false};
    std::unique_ptr<std::thread> recv_thread;
    void recv_loop();
    bool connect_to_pdu(const QString& ip, int port);
    void receive_data_loop();

    // send loop
    std::atomic<bool> send_flag = {false};
    std::unique_ptr<std::thread> send_thread;
    void send_loop();

    // jog loop
    std::atomic<bool> jog_flag = {false};
    std::unique_ptr<std::thread> jog_thread;
    void jog_loop();

    // storage
    tbb::concurrent_queue<std::vector<uchar>> msg_que;
    std::vector<MOBILE_POSE> pose_storage;
    std::vector<MOBILE_IMU> imu_storage;

    // calc battery percentage
    int calc_battery_percentage(float voltage);
    std::atomic<bool> is_first_receive = {true};
    double input_voltage = 999.0;

    // vars
    MOBILE_POSE cur_pose;
    MOBILE_STATUS cur_status;
    MOBILE_SETTING cur_setting;
    Eigen::Vector3d cur_imu;
    QString cur_pdu_state = "none";
    bool is_imu_used = false;
    uint8_t bms_type = 0x00; // 0x0A: not_used, 0x0B: bms_tabos

    RobotWheelModel wheel_model = RobotWheelModel::ROBOT_WHEEL_MODEL_UNKNOWN;

    std::atomic<double> bat_soc = 0.0;

    std::atomic<bool> is_inter_lock_foot = {false};
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};
    std::atomic<bool> is_synced = {false};
    std::atomic<bool> is_auto_move = {false};
    std::atomic<double> sync_st_time = {0};
    std::atomic<double> offset_t = {0};
    std::atomic<bool>  mobile_first_connected = {false};

    std::atomic<double> process_time_mobile = {0.0};

    std::atomic<double> distance = 0.0;

    std::atomic<double> last_pose_t = {0};
    std::atomic<double> last_imu_t = {0};

    // for gamepad
    std::atomic<bool> is_gamepad_connected = {false};
    std::atomic<double> last_gamepad_update_time = {0};
    std::atomic<double> cur_lx = {0.};
    std::atomic<double> cur_ly = {0.};
    std::atomic<double> cur_rx = {0.};
    std::atomic<bool> is_lb_pressed = {false};
    std::atomic<bool> is_rb_pressed = {false};

    // for jog
    std::atomic<bool> is_jog_pressed = {false};
    std::atomic<double> last_jog_update_time = {0};
    std::atomic<double> vx_target = {0.};
    std::atomic<double> vy_target = {0.};
    std::atomic<double> wz_target = {0.};
    std::atomic<double> jog_limit_v = {1.};
    std::atomic<double> jog_limit_w = {1.};
    std::atomic<double> jog_limit_v_acc = {1.};
    std::atomic<double> jog_limit_v_dcc = {1.};
    std::atomic<double> jog_limit_w_acc = {30.};
    std::atomic<double> jog_limit_w_dcc = {30.};

    // last control input
    std::atomic<double> vx0 = {0};
    std::atomic<double> vy0 = {0};
    std::atomic<double> wz0 = {0};

    QString pose_text;
    QString status_text;

    float f_distance = 0.0;

    // Gamepad
    QGamepad* gamepad;

    // device varibale
    bool speaker_io_state[4] = {false,};

public Q_SLOTS:
    void slot_jog_update(const Eigen::Vector3d& val);
    void slot_profile_move_stop();
};

#endif // MOBILE_H
