#ifndef COMM_RRS_H
#define COMM_RRS_H

// global defines
#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

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

// sio
#include <sio_client.h>
#define BIND_EVENT(IO,EV,FN) IO->on(EV,FN)

// qt
#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QNetworkInterface>
#include <QJsonArray>
#include <QProcess>

// stl
#include <shared_mutex>
#include <thread>
#include <atomic>
#include <memory>

struct COMM_RRS_INFO
{
    static constexpr unsigned int send_move_status_cnt = 10;
    static constexpr unsigned int send_status_cnt = 50;
};

class COMM_RRS : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_RRS)
public:
    static COMM_RRS* instance(QObject* parent = nullptr);

    // interface func
    void init();

    // util func
    QString get_json(sio::message::ptr const& data, const QString& key);
    void fillArray_memcpy(unsigned char* arr, int arr_size, const QJsonArray& jsonArr);

    QString get_multi_state();
    QByteArray get_last_msg();

    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_cam_module(CAM* _cam);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);
    void set_dockcontrol_module(DOCKCONTROL* _dctrl);
    void set_localization_module(LOCALIZATION* _loc);
    void set_mapping_module(MAPPING* _mapping);

    // for send path
    void set_global_path_update();
    void set_local_path_update();

private:
    explicit COMM_RRS(QObject *parent = nullptr);
    ~COMM_RRS();

    std::shared_mutex mtx;

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
    LIDAR_2D* lidar_3d;
    AUTOCONTROL* ctrl;
    DOCKCONTROL* dctrl;
    LOCALIZATION* loc;

    // vars    
    std::unique_ptr<sio::client> io;    
    std::atomic<int> last_send_kfrm_idx = {0};

    // flags
    QString multi_state;
    std::atomic<bool> is_connected = {false};

    std::atomic<bool> is_local_path_update2 = {false};
    std::atomic<bool> is_global_path_update2 = {false};

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::unique_ptr<std::thread> semi_auto_init_thread;

    QByteArray lastest_msg_str;

    // send timer
    QTimer* send_timer;
    std::atomic<int> lidar_view_frequency = {1};
    std::atomic<int> path_view_frequency  = {2};
    void send_lidar_2d();
    void send_lidar_3d();
    void send_status();
    void send_local_path();
    void send_global_path();
    void send_move_status();
    void send_mapping_cloud();

    int send_cnt = 0;
    int lidar_view_cnt = 0;
    int path_view_cnt  = 0;

public Q_SLOTS:
    void sio_connected();
    void sio_disconnected(sio::client::close_reason const& reason);
    void sio_error();

    void recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_load(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_randomseq(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_dock(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_view_lidar_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_view_path_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_led(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_motor(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_foot(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_field_set(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_safety_io_set(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);

    void recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_vobs(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_software_update(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);

    void slot_led(DATA_LED msg);
    void slot_move(DATA_MOVE msg);
    void slot_path(DATA_PATH msg);
    void slot_vobs(DATA_VOBS msg);
    void slot_load(DATA_LOAD msg);
    void slot_dock(DATA_DOCK msg);
    void slot_foot(DATA_FOOT msg);
    void slot_motor(DATA_MOTOR msg);
    void slot_mapping(DATA_MAPPING msg);
    void slot_randomseq(DATA_RANDOMSEQ msg);
    void slot_view_path(DATA_VIEW_PATH msg);
    void slot_field_set(DATA_FIELD msg);
    void slot_view_lidar(DATA_VIEW_LIDAR msg);
    void slot_localization(DATA_LOCALIZATION msg);
    void slot_software_update(DATA_SOFTWARE msg);
    void slot_safety_io(DATA_SAFTYIO msg);

    /* send command response */
    void send_move_response(const DATA_MOVE& msg);
    void send_localization_response(const DATA_LOCALIZATION& msg);
    void send_load_response(const DATA_LOAD& msg);
    void send_randomseq_response(const DATA_RANDOMSEQ& msg);
    void send_mapping_response(const DATA_MAPPING& msg);
    void send_dock_response(const DATA_DOCK& msg);
    void send_field_set_response(const DATA_FIELD& msg);
    void send_field_get_response(const DATA_FIELD& msg);
    void send_path_response(const DATA_PATH& msg);
    void send_software_update_response(const DATA_SOFTWARE& msg);
    void send_safetyio_response(const DATA_SAFTYIO& msg);

    void send_loop();

Q_SIGNALS:
    void signal_move(DATA_MOVE msg);
    void signal_localization(DATA_LOCALIZATION msg);
    void signal_load(DATA_LOAD msg);
    void signal_randomseq(DATA_RANDOMSEQ msg);
    void signal_mapping(DATA_MAPPING msg);
    void signal_dock(DATA_DOCK msg);
    void signal_view_lidar(DATA_VIEW_LIDAR msg);
    void signal_view_path(DATA_VIEW_PATH msg);
    void signal_led(DATA_LED msg);
    void signal_motor(DATA_MOTOR msg);
    void signal_foot(DATA_FOOT msg);
    void signal_field_set(DATA_FIELD msg);
    void signal_path(DATA_PATH msg);
    void signal_vobs(DATA_VOBS msg);
    void signal_software_update(DATA_SOFTWARE msg);
    void signal_safety_io(DATA_SAFTYIO msg);

};

#endif // COMM_RRS_H
