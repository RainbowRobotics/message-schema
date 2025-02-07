#ifndef COMM_MS_H
#define COMM_MS_H

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

class COMM_MS : public QObject
{
    Q_OBJECT
public:
    explicit COMM_MS(QObject *parent = nullptr);
    ~COMM_MS();
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

    QString multi_state = "";

    // vars
    std::unique_ptr<sio::client> io;    
    std::atomic<int> last_send_kfrm_idx = {0};
    MOVE_INFO last_move_info;

    // flags
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_status_time = {0};
    std::atomic<double> last_send_move_status_time = {0};

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;

    // interface func
    void init();

    /* send slamnav status */
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
    void signal_move(DATA_MOVE dmove);
    void signal_localization(DATA_LOCALIZATION dloc);
    void signal_load(DATA_LOAD dload);
    void signal_randomseq(DATA_RANDOMSEQ drandomseq);
    void signal_mapping(DATA_MAPPING dmap);
    void signal_dock(DATA_DOCK ddock);
    void signal_view_lidar(DATA_VIEW_LIDAR dlidar);
    void signal_view_path(DATA_VIEW_PATH dpath);
    void signal_led(DATA_LED dled);
    void signal_motor(DATA_MOTOR dmotor);

    void signal_path(DATA_PATH dpath);
    void signal_vobs_r(DATA_VOBS_R dvobs_r);
    void signal_vobs_c(DATA_VOBS_C dvobs_c);

private Q_SLOTS:

    void slot_move(DATA_MOVE dmove);
    void slot_localization(DATA_LOCALIZATION dloc);
    void slot_load(DATA_LOAD dload);
    void slot_randomseq(DATA_RANDOMSEQ drandomseq);
    void slot_mapping(DATA_MAPPING dmap);
    void slot_dock(DATA_DOCK ddock);
    void slot_view_lidar(DATA_VIEW_LIDAR dlidar);
    void slot_view_path(DATA_VIEW_PATH dpath);
    void slot_led(DATA_LED dled);
    void slot_motor(DATA_MOTOR dmotor);

    void slot_path(DATA_PATH dpath);
    void slot_vobs_r(DATA_VOBS_R dvobs_r);
    void slot_vobs_c(DATA_VOBS_C dvobs_c);

    /* send command response */
    void send_move_response(DATA_MOVE dmove);
    void send_localization_response(DATA_LOCALIZATION dloc);
    void send_load_response(DATA_LOAD dload);
    void send_randomseq_response(DATA_RANDOMSEQ drandomseq);
    void send_mapping_response(DATA_MAPPING result);
    void send_dock_response(DATA_DOCK ddock);
};

#endif // COMM_MS_H
