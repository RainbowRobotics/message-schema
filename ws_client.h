#ifndef WS_CLIENT_H
#define WS_CLIENT_H

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

class WS_CLIENT : public QObject
{
    Q_OBJECT
public:
    explicit WS_CLIENT(QObject *parent = nullptr);
    ~WS_CLIENT();

    // other modules
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
    QTimer reconnect_timer;

    // flags
    std::atomic<bool> is_connected = {false};

    // interface func
    void init();
    void send_status();
    void send_lidar();
    void send_mapping();

    // util func
    QString get_json(sio::message::ptr const& data, QString key);

Q_SIGNALS:
    void signal_motorinit(double time);
    void signal_move(double time, double vx, double vy, double wz);
    void signal_mapping_start(double time);
    void signal_mapping_stop(double time);
    void signal_mapping_save(double time, QString name);

private Q_SLOTS:
    void reconnect_loop();

    void sio_connected();
    void sio_disconnected(sio::client::close_reason const& reason);
    void sio_error();

    void recv_motorinit(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);
    void recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp);

};

#endif // WS_CLIENT_H
