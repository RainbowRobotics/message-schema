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
#include "docking.h"
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
    DOCKING *dctrl = NULL;
    LVX_LOC *lvx = NULL;

    // vars    
    QWebSocket client;
    QTimer reconnect_timer;
    int reconnect_cnt = 0;
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};
    QString robot_id = "";
    QString multi_state = "none"; // none, req_path, recv_path
    MOVE_INFO last_move_info;

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;

    DATA_MOVE get_data_move(QJsonObject dataObj);
    DATA_LOAD get_data_load(QJsonObject dataObj);
    DATA_LOCALIZATION get_data_localization(QJsonObject dataObj);
    DATA_RANDOMSEQ get_data_randomseq(QJsonObject dataObj);
    DATA_PATH get_data_path(QJsonObject dataObj);
    DATA_VOBS_R get_data_vobs_r(QJsonObject dataObj);
    DATA_VOBS_C get_data_vobs_c(QJsonObject dataObj);

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);
    QString get_multi_state();

    void send_move_response(DATA_MOVE dmove);
    void send_localization_response(DATA_LOCALIZATION dloc);
    void send_load_response(DATA_LOAD dload);
    void send_randomseq_response(DATA_RANDOMSEQ drandomseq);

Q_SIGNALS:
    void signal_send_info();

    void signal_move(DATA_MOVE dmove);
    void signal_localization(DATA_LOCALIZATION dloc);
    void signal_load(DATA_LOAD dload);
    void signal_randomseq(DATA_RANDOMSEQ drandomseq);

    void signal_path(DATA_PATH dpath);
    void signal_vobs_r(DATA_VOBS_R dvobs_r);
    void signal_vobs_c(DATA_VOBS_C dvobs_c);

private Q_SLOTS:

    void recv_message(const QByteArray &buf);
    void reconnect_loop();
    void connected();
    void disconnected();

private Q_SLOTS:

    // send slots
    void slot_send_info();

    void slot_move(DATA_MOVE dmove);
    void slot_localization(DATA_LOCALIZATION dloc);
    void slot_load(DATA_LOAD dload);
    void slot_randomseq(DATA_RANDOMSEQ drandomseq);

    void slot_path(DATA_PATH dpath);
    void slot_vobs_r(DATA_VOBS_R dvobs_r);
    void slot_vobs_c(DATA_VOBS_C dvobs_c);
};

#endif // COMM_FMS_H
