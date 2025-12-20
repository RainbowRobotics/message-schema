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
    static constexpr int reconnect_time = 3; // second
    static constexpr double move_status_send_time = 0.1; // second
    static constexpr double status_send_time = 0.5; // second
    static constexpr double mapping_cloud_send_time = 0.5; // second
    static constexpr double rtsp_cam_rgb_send_time = 1.0; // second
};

class COMM_MSA : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_MSA)
public:
    static COMM_MSA* instance(QObject* parent = nullptr);

    // interface func
    void init();

    // setter
    void set_global_path_update();
    void set_local_path_update();
    void set_last_receive_msg(QString val);
    void set_last_tgt_pose_vec(const Eigen::Vector4d& val);

    // getter
    bool    get_is_connected();
    QString get_last_receive_msg();
    double  get_process_time_path();
    double  get_process_time_vobs();
    double  get_max_process_time_path();
    double  get_max_process_time_vobs();
    Eigen::Vector4d get_last_tgt_pose_vec();

    // setter (othre modules)
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

    // start thread module
    void start_all_thread();
    void start_recv_thread();
    void start_move_thread();
    void start_load_thread();
    void start_mapping_thread();
    void start_localization_thread();
    void start_path_thread();
    void start_vobs_thread();
    void start_send_status_thread();
    void start_send_response_thread();

    void stop_all_thread();
    void stop_recv_thread();
    void stop_move_thread();
    void stop_load_thread();
    void stop_mapping_thread();
    void stop_localization_thread();
    void stop_path_thread();
    void stop_vobs_thread();
    void stop_send_status_thread();
    void stop_send_response_thread();

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

    QTimer* reconnect_timer;

    // vars
    std::unique_ptr<sio::client> rrs_socket;

    std::atomic<int> lidar_view_frequency = {1};
    std::atomic<int> path_view_frequency  = {2};

    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_local_path_update2 = {false};
    std::atomic<bool> is_global_path_update2 = {false};

    // for semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::unique_ptr<std::thread> semi_auto_init_thread;

    // save value
    PATH last_sent_path;
    QString last_receive_msg = "";
    std::atomic<int> last_send_kfrm_idx = {0};
    Eigen::Vector4d last_tgt_pose_vec = Eigen::Vector4d::Zero();

    std::atomic<double> end_time   = {0.0};
    std::atomic<double> start_time = {0.0};
    std::atomic<double> process_time_path = {0.0};
    std::atomic<double> process_time_vobs = {0.0};
    std::atomic<double> max_process_time_path = {-std::numeric_limits<double>::max()};
    std::atomic<double> max_process_time_vobs = {-std::numeric_limits<double>::max()};

    double last_lidar_view_time = 0.0;
    double last_path_view_time  = 0.0;

    unsigned char dio_arr_old[16] = {0};

    template<typename T>
    void add_to_obj(sio::object_message::ptr obj, const std::string& key, T value);

    sio::object_message::ptr create_localization_score_obj(const Eigen::Vector2d& loc_ieir,
                                                           const Eigen::Vector2d& mapping_ieir);
    sio::object_message::ptr create_pose_obj(const Eigen::Vector3d& cur_xi);
    sio::object_message::ptr create_velocity_obj(const MOBILE_POSE& mo);
    sio::object_message::ptr create_move_state_obj();
    sio::object_message::ptr create_motor_obj(const MOBILE_STATUS& ms, int motor_idx);
    sio::object_message::ptr create_robot_state_obj(const MOBILE_STATUS& ms);
    sio::object_message::ptr create_robot_io_obj(const MOBILE_STATUS& ms);
    sio::object_message::ptr create_power_obj(const MOBILE_STATUS& ms);
    sio::object_message::ptr create_map_info_obj();
    sio::object_message::ptr create_robot_info_obj();

    void recv_message(sio::event& ev);
    void recv_message_array(sio::event& ev);

    /* utils function */
    bool is_main_window_valid();
    QMainWindow* get_main_window();
    void calc_remaining_time_distance(DATA_MOVE& msg); // todo ---> move to autoctorl !
    QJsonObject sio_object_to_qt_json_object(const std::map<std::string, sio::message::ptr>& obj);
    QJsonValue convert_item(sio::message::ptr item);

    // get json utils
    int     get_json_int(const QJsonObject& json, QString key);
    double  get_json_double(const QJsonObject& json, QString key);
    QString get_json(const QJsonObject& json, QString key);

private Q_SLOTS:
    void connected();
    void disconnected();
    void reconnect_loop();

    // send status (pooling)
    void send_move_status();
    void send_status();
    void send_local_path();
    void send_global_path();
    void send_lidar_2d();
    void send_lidar_3d();
    void send_mapping_cloud();

    //  command response (callback)
    void send_move_response(const DATA_MOVE& msg);
    void send_dock_response(const DATA_DOCK& msg);
    void send_localization_response(const DATA_LOCALIZATION& msg);
    void send_mapping_response(const DATA_MAPPING& msg);
    void send_load_response(const DATA_LOAD& msg);
    void send_path_response(const DATA_PATH& msg);

private:
    std::mutex move_mtx;
    std::mutex load_mtx;
    std::mutex mapping_mtx;
    std::mutex localization_mtx;
    std::mutex path_mtx;
    std::mutex vobs_mtx;
    std::mutex response_mtx;
    std::mutex recv_mtx;

    std::mutex send_status_mtx;
    std::mutex send_response_mtx;

    // MSA queue
    std::queue<QString> recv_queue;
    std::queue<DATA_MOVE> move_queue;
    std::queue<DATA_LOAD> load_queue;
    std::queue<DATA_PATH> path_queue;
    std::queue<DATA_VOBS> vobs_queue;
    std::queue<DATA_MAPPING> mapping_queue;
    std::queue<DATA_LOCALIZATION> localization_queue;

    tbb::concurrent_queue<SOCKET_MESSAGE> send_status_queue;
    std::queue<SOCKET_MESSAGE> send_response_queue;

    std::condition_variable recv_cv;
    std::condition_variable move_cv;
    std::condition_variable load_cv;
    std::condition_variable path_cv;
    std::condition_variable vobs_cv;
    std::condition_variable mapping_cv;
    std::condition_variable localization_cv;
    std::condition_variable send_response_cv;

    std::atomic<bool> is_recv_running = {false};
    std::atomic<bool> is_move_running = {true};
    std::atomic<bool> is_load_running = {true};
    std::atomic<bool> is_mapping_running = {true};
    std::atomic<bool> is_localization_running = {true};
    std::atomic<bool> is_path_running = {true};
    std::atomic<bool> is_vobs_running = {true};
    std::atomic<bool> is_send_status_running = {true};
    std::atomic<bool> is_send_response_running = {true};
    std::atomic<bool> is_before_given_path = {false};

    // receive
    std::unique_ptr<std::thread> move_thread;
    std::unique_ptr<std::thread> load_thread;
    std::unique_ptr<std::thread> localization_thread;
    std::unique_ptr<std::thread> mapping_thread;
    std::unique_ptr<std::thread> path_thread;
    std::unique_ptr<std::thread> vobs_thread;
    std::unique_ptr<std::thread> recv_thread;

    // send
    std::unique_ptr<std::thread> send_response_thread;
    std::unique_ptr<std::thread> send_status_thread;

    void move_loop();
    void load_loop();
    void path_loop();
    void vobs_loop();
    void recv_loop();
    void mapping_loop();
    void localization_loop();
    void send_status_loop();
    void send_response_loop();

    /* main handler */
    void handle_move_cmd(const QJsonObject& data);
    void handle_load_cmd(const QJsonObject& data);
    void handle_path_cmd(const QJsonObject& data);
    void handle_vobs_cmd(const QJsonObject& data);
    void handle_mapping_cmd(const QJsonObject& data);
    void handle_localization_cmd(const QJsonObject& data);

    /* move handler */
    void handle_move_jog(const DATA_MOVE& msg);
    void handle_move_goal(DATA_MOVE& msg);
    void handle_move_stop(DATA_MOVE& msg);
    void handle_move_pause(DATA_MOVE& msg);
    void handle_move_resume(DATA_MOVE& msg);
    void handle_move_profile(DATA_MOVE& msg);
    void handle_move_target(DATA_MOVE& msg);

    /* load handler */
    void handle_load_map(DATA_LOAD& msg);
    void handle_load_topo(DATA_LOAD& msg);

    /* localization handler */
    void handle_localization_semiautoinit(DATA_LOCALIZATION& msg);
    void handle_localization_init(DATA_LOCALIZATION& msg);
    void handle_localization_start(DATA_LOCALIZATION& msg);
    void handle_localization_stop(DATA_LOCALIZATION& msg);
    void handle_localization_randominit(DATA_LOCALIZATION& msg);

    /* mapping handler */
    void handle_mapping_start(DATA_MAPPING& msg);
    void handle_mapping_stop(DATA_MAPPING& msg);
    void handle_mapping_save(DATA_MAPPING& msg);
    void handle_mapping_reload(DATA_MAPPING& msg);

    /* path handler */
    void handle_path(DATA_PATH& msg);

    /* vobs handler */
    void handle_vobs(DATA_VOBS& msg);
};

#endif // COMM_MSA_H