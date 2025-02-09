#ifndef COMM_RRS_H
#define COMM_RRS_H

// global defines
#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"
#include "code_reader.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "docking.h"
#include "lvx_loc.h"

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

class COMM_RRS : public QObject
{
    Q_OBJECT
public:
    explicit COMM_RRS(QObject *parent = nullptr);
    ~COMM_RRS();
    std::mutex mtx;

    // other modules
    QObject *main = NULL;
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    CAM *cam = NULL;
    CODE_READER *code = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;
    DOCKING *dctrl = NULL;
    LVX_LOC *lvx = NULL;

    // vars    
    std::unique_ptr<sio::client> io;    
    std::atomic<int> last_send_kfrm_idx = {0};

    // flags
    QString multi_state = "";
    std::atomic<bool> is_connected = {false};

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;

    // interface func
    void init();

    // send status
    void send_status();
    void send_move_status();

    // send path
    void send_local_path();
    void send_global_path();

    // send cloud
    void send_lidar();
    void send_mapping_cloud();

    // util func
    QString get_json(sio::message::ptr const& data, QString key);
    QString get_multi_state();

private Q_SLOTS:
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

    void recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_vobs_robots(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_vobs_closures(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);

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

    void signal_path(DATA_PATH msg);
    void signal_vobs_r(DATA_VOBS_R msg);
    void signal_vobs_c(DATA_VOBS_C msg);

private Q_SLOTS:
    void slot_move(DATA_MOVE msg);
    void slot_localization(DATA_LOCALIZATION msg);
    void slot_load(DATA_LOAD msg);
    void slot_randomseq(DATA_RANDOMSEQ msg);
    void slot_mapping(DATA_MAPPING msg);
    void slot_dock(DATA_DOCK msg);
    void slot_view_lidar(DATA_VIEW_LIDAR msg);
    void slot_view_path(DATA_VIEW_PATH msg);
    void slot_led(DATA_LED msg);
    void slot_motor(DATA_MOTOR msg);

    void slot_path(DATA_PATH msg);
    void slot_vobs_r(DATA_VOBS_R msg);
    void slot_vobs_c(DATA_VOBS_C msg);

    /* send command response */
    void send_move_response(DATA_MOVE msg);
    void send_localization_response(DATA_LOCALIZATION msg);
    void send_load_response(DATA_LOAD msg);
    void send_randomseq_response(DATA_RANDOMSEQ msg);
    void send_mapping_response(DATA_MAPPING msg);
    void send_dock_response(DATA_DOCK msg);
};

#endif // COMM_RRS_H
