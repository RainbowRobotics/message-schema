#ifndef COMM_MS_H
#define COMM_MS_H

// global defines
#include "global_defines.h"
#include "utils.h"

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

// sio
#include <sio_client.h>
#define BIND_EVENT(IO,EV,FN) IO->on(EV,FN)

// qt
#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
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

    // vars
    std::unique_ptr<sio::client> io;
    int reconnect_cnt = 0;
    QTimer reconnect_timer;
    std::atomic<int> last_send_kfrm_idx = {0};
    MOVE_INFO last_move_info;

    // flags
    std::atomic<bool> is_connected = {false};

    // interface func
    void init();

    void send_status();

    void send_global_path();
    void send_local_path();

    void send_lidar();
    void send_mapping_cloud();

    void send_mapping_start_response(QString result);
    void send_mapping_stop_response();
    void send_mapping_save_response(QString name, QString result);

    void send_mapload_response(QString name, QString result);    
    void send_localization_response(QString command, QString result);

    void send_move_target_response(double x, double y, double z, double rz, int preset, QString method, QString result, QString message);
    void send_move_goal_response(QString node_id, int preset, QString method, QString result, QString message);
    void send_move_pause_response(QString result);
    void send_move_resume_response(QString result);
    void send_move_stop_response(QString result);

    // util func
    QString get_json(sio::message::ptr const& data, QString key);

Q_SIGNALS:
    void signal_motorinit(double time);

    void signal_move_jog(double time, double vx, double vy, double wz);
    void signal_move_target(double time, double x, double y, double z, double rz, int preset, QString method);
    void signal_move_goal(double time, QString node_id, int preset, QString method);
    void signal_move_pause(double time);
    void signal_move_resume(double time);
    void signal_move_stop(double time);

    void signal_mapping_start(double time);
    void signal_mapping_stop(double time);
    void signal_mapping_save(double time, QString name);
    void signal_mapping_reload(double time);

    void signal_mapload(double time, QString name);

    void signal_localization_autoinit(double time);
    void signal_localization_init(double time, double x, double y, double z, double rz);
    void signal_localization_start(double time);
    void signal_localization_stop(double time);


private Q_SLOTS:
    void reconnect_loop();

    void sio_connected();
    void sio_disconnected(sio::client::close_reason const& reason);
    void sio_error();

    void recv_motorinit(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_mapload(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);

private Q_SLOTS:
    void slot_motorinit(double time);

    void slot_move_jog(double time, double vx, double vy, double wz);
    void slot_move_target(double time, double x, double y, double z, double rz, int preset, QString method);
    void slot_move_goal(double time, QString node_id, int preset, QString method);
    void slot_move_pause(double time);
    void slot_move_resume(double time);
    void slot_move_stop(double time);
    void slot_move_succeed(QString message);
    void slot_move_failed(QString message);

    void slot_mapping_start(double time);
    void slot_mapping_stop(double time);
    void slot_mapping_save(double time, QString name);
    void slot_mapping_reload(double time);

    void slot_mapload(double time, QString name);

    void slot_localization_autoinit(double time);
    void slot_localization_init(double time, double x, double y, double z, double rz);
    void slot_localization_start(double time);
    void slot_localization_stop(double time);

};

#endif // COMM_MS_H
