#ifndef COMM_FMS_H
#define COMM_FMS_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "dockcontrol.h"
#include "lvx_loc.h"

// qt
#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class COMM_FMS : public QObject
{
    Q_OBJECT
public:
    explicit COMM_FMS(QObject *parent = nullptr);
    ~COMM_FMS();
    std::mutex mtx;

    // other modules
    QObject *main = NULL;
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;
    DOCKCONTROL *dctrl = NULL;
    LVX_LOC *lvx = NULL;

    // vars    
    QWebSocket client;
    QTimer reconnect_timer;
    int reconnect_cnt = 0;    
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};

    QString robot_id = "";
    QString multi_state = "none"; // "none", "req_path", "recv_path"

    // for semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);
    QString get_multi_state();

private Q_SLOTS:
    void recv_message(const QByteArray &buf);
    void reconnect_loop();
    void connected();
    void disconnected();

Q_SIGNALS:    
    void signal_send_move_status();

    void recv_move(DATA_MOVE msg);
    void recv_localization(DATA_LOCALIZATION msg);
    void recv_load(DATA_LOAD msg);
    void recv_randomseq(DATA_RANDOMSEQ msg);
    void recv_path(DATA_PATH msg);
    void recv_vobs_r(DATA_VOBS_R msg);
    void recv_vobs_c(DATA_VOBS_C msg);

private Q_SLOTS:
    void send_move_status();

    void slot_move(DATA_MOVE msg);
    void slot_localization(DATA_LOCALIZATION msg);
    void slot_load(DATA_LOAD msg);
    void slot_randomseq(DATA_RANDOMSEQ msg);
    void slot_path(DATA_PATH msg);
    void slot_vobs_r(DATA_VOBS_R msg);
    void slot_vobs_c(DATA_VOBS_C msg);
};

#endif // COMM_FMS_H
