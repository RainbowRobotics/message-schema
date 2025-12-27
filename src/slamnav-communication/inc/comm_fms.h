#ifndef COMM_FMS_H
#define COMM_FMS_H

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

// qt
#include <QObject>
#include <QMainWindow>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

struct COMM_FMS_INFO
{
    static constexpr int reconnect_time = 3; // second
    static constexpr double move_status_send_time = 0.1; // second
    static constexpr double status_send_time = 0.5; // second
    static constexpr double mapping_cloud_send_time = 0.5; // second
    static constexpr double rtsp_cam_rgb_send_time = 1.0; // second
};

class COMM_FMS : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_FMS)
public:
    static COMM_FMS* instance(QObject* parent = nullptr);

    // interface func
    void init();

    QString get_json(const QJsonObject& json, QString key);
    QString get_multi_state();

    double get_process_time_path();
    double get_process_time_vobs();

    double get_max_process_time_path();
    double get_max_process_time_vobs();

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

private:
    explicit COMM_FMS(QObject *parent = nullptr);
    ~COMM_FMS();

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
    QTimer* send_timer;
    QTimer* reconnect_timer;
    QWebSocket* client;
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};

    QString robot_id = "";
    QString multi_state = "none"; // "none", "req_path", "recv_path"

    // for semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::unique_ptr<std::thread> semi_auto_init_thread;

    QMainWindow* get_main_window();
    bool is_main_window_valid();

    tbb::concurrent_queue<QString> send_queue;

    std::atomic<bool> is_recv_running = {false};
    tbb::concurrent_queue<QString> recv_queue;
    std::unique_ptr<std::thread> recv_thread;
    void recv_loop();

    void send_status_loop();

    std::mutex move_mtx;
    std::mutex path_mtx;
    std::mutex vobs_mtx;
    std::mutex common_mtx;
    std::mutex response_mtx;

    std::queue<DATA_MOVE> move_queue;
    std::queue<DATA_PATH> path_queue;
    std::queue<DATA_VOBS> vobs_queue;
    std::queue<DATA_COMMON> common_queue;
    std::queue<std::function<void()>> response_queue;

    std::condition_variable move_cv;
    std::condition_variable path_cv;
    std::condition_variable vobs_cv;
    std::condition_variable common_cv;
    std::condition_variable response_cv;

    std::atomic<bool> is_move_running = {true};
    std::atomic<bool> is_path_running = {true};
    std::atomic<bool> is_vobs_running = {true};
    std::atomic<bool> is_common_running = {true};
    std::atomic<bool> is_response_running = {true};
    std::atomic<bool> is_send_status_running = {true};

    std::unique_ptr<std::thread> move_thread;
    std::unique_ptr<std::thread> path_thread;
    std::unique_ptr<std::thread> vobs_thread;
    std::unique_ptr<std::thread> common_thread;
    std::unique_ptr<std::thread> response_thread;

    std::unique_ptr<std::thread> send_status_thread;

    void move_loop();
    void path_loop();
    void vobs_loop();
    void common_loop();
    void response_loop();

    void handle_move_cmd(const QJsonObject& data);
    void handle_move_jog(const DATA_MOVE& msg);
    void handle_move_goal(DATA_MOVE& msg);
    void handle_move_stop(DATA_MOVE& msg);
    void handle_move_pause(DATA_MOVE& msg);
    void handle_move_resume(DATA_MOVE& msg);
    void handle_move_target(DATA_MOVE& msg);
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

    std::atomic<double> end_time   = {0.0};
    std::atomic<double> start_time = {0.0};

    std::atomic<double> process_time_path = {0.0};
    std::atomic<double> process_time_vobs = {0.0};

    std::atomic<double> max_process_time_path = {-9999.}; // {-std::numeric_limits<double>::max()};
    std::atomic<double> max_process_time_vobs = {-9999.}; // {-std::numeric_limits<double>::max()};

private Q_SLOTS:
    void send_loop();
    void connected();
    void disconnected();
    void recv_message(const QString &buf);
    void reconnect_loop();

    void send_move_status();
    void send_move_response(DATA_MOVE msg);
    void send_path_response(DATA_PATH msg);

Q_SIGNALS:
    void signal_send_move_status();
};

#endif // COMM_FMS_H
