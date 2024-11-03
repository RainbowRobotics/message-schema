#ifndef COMM_UI_H
#define COMM_UI_H

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

// qt
#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class COMM_UI : public QObject
{
    Q_OBJECT
public:
    explicit COMM_UI(QObject *parent = nullptr);
    ~COMM_UI();
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

    QWebSocket client;
    QTimer reconnect_timer;
    int reconnect_cnt = 0;

    // vars
    std::atomic<bool> is_connected = {false};
    std::atomic<int> last_send_kfrm_idx = {0};
    MOVE_INFO last_move_info;

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;
    void semi_auto_init_loop();

public Q_SLOTS:
    void reconnect_loop();

    void connected();
    void disconnected();
    void recv_message(QString message);

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
    void signal_localization_semiautoinit(double time);
    void signal_localization_init(double time, double x, double y, double z, double rz);
    void signal_localization_start(double time);
    void signal_localization_stop(double time);

public Q_SLOTS:
    // recv slots
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
    void slot_localization_semiautoinit(double time);
    void slot_localization_semiautoinit_succeed(QString message);
    void slot_localization_semiautoinit_failed(QString message);
    void slot_localization_init(double time, double x, double y, double z, double rz);
    void slot_localization_start(double time);
    void slot_localization_stop(double time);

    // send slots
    void send_status();

    void send_global_path();
    void send_local_path();

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


};

#endif // COMM_UI_H
