#ifndef COMM_MSA_H
#define COMM_MSA_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"
#include "localization.h"
#include "mapping.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "dockcontrol.h"

#include <sio_client.h>
#define BIND_EVENT(IO,EV,FN) IO->on(EV,FN)

// qt
#include <QObject>
#include <QMainWindow>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QCoreApplication>
#include <QProcess>

#include <filesystem>

struct COMM_MSA_INFO
{
    static constexpr unsigned int send_move_status_cnt = 10;
    static constexpr unsigned int send_status_cnt = 50;
};

class COMM_MSA : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_MSA)
public:
    static COMM_MSA* instance(QObject* parent = nullptr);

    // interface func
    void init();

    QString get_json(const QJsonObject& json, QString key);
    int get_json_int(const QJsonObject& json, QString key);
    double get_json_double(const QJsonObject& json, QString key);
    QString get_msa_text();

    QString get_multi_state();

    double get_process_time_path();
    double get_process_time_vobs();

    double get_max_process_time_path();
    double get_max_process_time_vobs();

    bool get_msa_connect_check();

    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_cam_module(CAM* _cam);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);
    void set_dockcontrol_module(DOCKCONTROL* _dctrl);
    void set_localization_module(LOCALIZATION* _loc);
    void set_mapping_module(MAPPING* _mapping);
    void set_global_path_update();
    void set_local_path_update();

    void send_safetyio_response(const DATA_SAFTYIO& msg);

private:
    explicit COMM_MSA(QObject *parent = nullptr);
    ~COMM_MSA();

    std::shared_mutex mtx;
    std::mutex send_mtx;

    // other modules
    CAM* cam;
    UNIMAP* unimap;
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    OBSMAP* obsmap;
    QObject* main;
    MAPPING* mapping;
    LIDAR_2D* lidar_2d;
    LIDAR_3D* lidar_3d;
    AUTOCONTROL* ctrl;
    DOCKCONTROL* dctrl;
    LOCALIZATION* loc;

    // vars
    std::unique_ptr<sio::client> io;
    std::atomic<int> last_send_kfrm_idx = {0};

    QTimer* send_timer;
    QTimer* reconnect_timer;
    QWebSocket* client;
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};

    QString robot_id = "";
    QString multi_state = "none"; // "none", "req_path", "recv_path"
    QString old_path = "";

    // for semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::unique_ptr<std::thread> semi_auto_init_thread;

    std::atomic<bool> is_local_path_update2 = {false};
    std::atomic<bool> is_global_path_update2 = {false};
    QMainWindow* get_main_window();
    bool is_main_window_valid();

    //    tbb::concurrent_queue<QString> send_queue;
    tbb::concurrent_queue<SOCKET_MESSAGE> send_queue;

    std::atomic<bool> is_recv_running = {false};
    tbb::concurrent_queue<QString> recv_queue;
    std::unique_ptr<std::thread> recv_thread;
    void recv_loop();

    std::mutex move_mtx;
    std::mutex load_mtx;
    std::mutex mapping_mtx;
    std::mutex localization_mtx;
    std::mutex control_mtx;
    std::mutex path_mtx;
    std::mutex vobs_mtx;
    std::mutex common_mtx;
    std::mutex response_mtx;
    std::mutex status_mtx;

    //MSA queue
    std::queue<DATA_MOVE> move_queue;
    std::queue<DATA_LOAD> load_queue;
    std::queue<DATA_CONTROL> control_queue;
    std::queue<DATA_MAPPING> mapping_queue;
    std::queue<DATA_LOCALIZATION> localization_queue;


    std::queue<DATA_PATH> path_queue;
    std::queue<DATA_VOBS> vobs_queue;
    std::queue<DATA_COMMON> common_queue;
    std::queue<std::function<void()>> response_queue;

    std::condition_variable move_cv;
    std::condition_variable load_cv;
    std::condition_variable localization_cv;
    std::condition_variable control_cv;
    std::condition_variable mapping_cv;
    std::condition_variable path_cv;
    std::condition_variable vobs_cv;
    std::condition_variable common_cv;
    std::condition_variable response_cv;
    std::condition_variable status_cv;

    std::atomic<int> lidar_view_frequency = {1};
    std::atomic<int> path_view_frequency  = {2};

    std::atomic<bool> is_move_running = {true};
    std::atomic<bool> is_load_running = {true};
    std::atomic<bool> is_mapping_running = {true};
    std::atomic<bool> is_control_running = {true};
    std::atomic<bool> is_localization_running = {true};
    std::atomic<bool> is_path_running = {true};
    std::atomic<bool> is_vobs_running = {true};
    std::atomic<bool> is_common_running = {true};
    std::atomic<bool> is_response_running = {true};
    std::atomic<bool> is_send_status_running = {true};
    std::atomic<bool> is_before_given_path = {false};

    std::unique_ptr<std::thread> move_thread;
    std::unique_ptr<std::thread> load_thread;
    std::unique_ptr<std::thread> control_thread;
    std::unique_ptr<std::thread> localization_thread;
    std::unique_ptr<std::thread> mapping_thread;
    std::unique_ptr<std::thread> path_thread;
    std::unique_ptr<std::thread> vobs_thread;
    std::unique_ptr<std::thread> common_thread;
    std::unique_ptr<std::thread> response_thread;
    std::unique_ptr<std::thread> status_thread;

    QString receive_msg;

    mutable std::shared_mutex msg_mtx;

    void move_loop();
    void control_loop();
    void localization_loop();
    void mapping_loop();
    void load_loop();
    void path_loop();
    void vobs_loop();
    void common_loop();
    void response_loop();
    void send_status_loop();

    //MSA handle
    void handle_move_cmd(const QJsonObject& data);
    void handle_load_cmd(const QJsonObject& data);
    void handle_mapping_cmd(const QJsonObject& data);
    void handle_localization_cmd(const QJsonObject& data);
    void handle_control_cmd(const QJsonObject& data);

    void handle_move_jog(const DATA_MOVE& msg);
    void handle_move_goal(DATA_MOVE& msg);
    void handle_move_stop(DATA_MOVE& msg);
    void handle_move_pause(DATA_MOVE& msg);
    void handle_move_resume(DATA_MOVE& msg);
//    void handle_move_linear(DATA_MOVE &msg);
    void handle_move_target(DATA_MOVE& msg);
    void handle_mapping(DATA_MAPPING msg);
    void handle_safetyio_cmd(const QJsonObject& data);

    void handle_localization_cmd(DATA_LOCALIZATION msg);

    void calc_remaining_time_distance(DATA_MOVE& msg);

    void handle_path_cmd(const QJsonObject& data);
    void handle_path(DATA_PATH& msg);
    void handle_path_move(DATA_PATH& msg);

    void handle_vobs_cmd(const QJsonObject& data);
    void handle_vobs(DATA_VOBS& msg);

    void handle_common_cmd(QString cmd, const QJsonObject& data);
    void handle_common_load_map(DATA_LOAD& msg);
    void handle_common_load_topo(DATA_LOAD& msg);
    void handle_common_load_config(DATA_LOAD& msg);

    void handle_common_random_seq(DATA_RANDOMSEQ& msg);

    void handle_common_docking_dock(DATA_DOCK& msg);
    void handle_common_docking_undock(DATA_DOCK& msg);

    void handle_common_loc_semiautoinit(DATA_LOCALIZATION& msg);
    void handle_common_loc_init(DATA_LOCALIZATION& msg);
    void handle_common_loc_start(DATA_LOCALIZATION& msg);
    void handle_common_loc_stop(DATA_LOCALIZATION& msg);
    void handle_common_loc_randominit(DATA_LOCALIZATION& msg);

    void handle_common_view_lidar(DATA_VIEW_LIDAR& msg);
    void handle_common_view_path(DATA_VIEW_PATH& msg);

    void handle_common_led(DATA_LED& msg);

    void handle_common_motor(DATA_MOTOR& msg);
    void send_safetyio_response(const QJsonObject& data);

    void handle_send_safetyIO(const QJsonObject& data);

    void slot_safety_io(DATA_SAFTYIO msg);
    QJsonValue convertItem(sio::message::ptr item);

    void send_profile_move_response(const DATA_MOVE& msg);

    std::atomic<double> end_time   = {0.0};
    std::atomic<double> start_time = {0.0};

    std::atomic<double> process_time_path = {0.0};
    std::atomic<double> process_time_vobs = {0.0};

    std::atomic<double> max_process_time_path = {-std::numeric_limits<double>::max()};
    std::atomic<double> max_process_time_vobs = {-std::numeric_limits<double>::max()};

    PATH last_sent_path;

    int send_cnt = 0;
    int lidar_view_cnt = 0;
    int path_view_cnt  = 0;

    QByteArray lastest_msg_str;

    unsigned char dio_arr_old[16] = {0};

    QString fms_cmd_direction = "";

    Eigen::Vector4d last_tgt_pose_vec = Eigen::Vector4d::Zero();

    QString given_method = "";

private Q_SLOTS:
    void send_loop();

    void connected();
    void disconnected();
    //    void recv_message(const QString &buf);
    void recv_message(sio::event& e);
    void recv_message_array(sio::event& ev);

    void reconnect_loop();

    void send_move_status();
    void send_status();
    void send_local_path();
    void send_global_path();
    void send_lidar_2d();
    void send_lidar_3d();
    void send_mapping_cloud();
    void send_system_status(double cpu_use, double cpu_temp);

    void slot_localization(DATA_LOCALIZATION msg);
    //    void slot_safety_io(DATA_SAFTYIO msg);

    //MSA
    void send_move_response(DATA_MOVE msg);
    void send_dock_response(const DATA_DOCK& msg);
    void send_localization_response(DATA_LOCALIZATION msg);
    void send_control_response(DATA_CONTROL msg);
    void send_mapping_response(DATA_MAPPING msg);
    void send_load_response(DATA_LOAD msg);

    void send_path_response(DATA_PATH msg);

    // for linear move -> direct cmd mobile class
//    void handle_move_linear(DATA_MOVE msg);
//    void handle_move_circular(DATA_MOVE msg);
//    void handle_move_rotate(DATA_MOVE msg);
    void slot_profile_move(DATA_MOVE msg);

Q_SIGNALS:
    void signal_send_move_status();
 void signal_profile_move(DATA_MOVE msg);
//    void signal_handle_move_linear(DATA_MOVE msg);
//    void signal_handle_move_circular(DATA_MOVE msg);
//    void signal_handle_move_rotate(DATA_MOVE msg);
};

#endif // COMM_MSA_H
