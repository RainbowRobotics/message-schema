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
    QWebSocket client;
    QTimer reconnect_timer;
    int reconnect_cnt = 0;

    // other modules
    QObject *main = NULL;
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;

    // vars    
    std::atomic<bool> is_connected = {false};
    QString robot_id = "";
    QString multi_state = "none"; // none, req_path, recv_path

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);
    QString get_multi_state();

Q_SIGNALS:
    void signal_mapload(double time, QString name);
    void signal_init(double time);
    void signal_random_init(double time, QString seed);
    void signal_random_seq(double time);

    void signal_regist_id(QString id);

private Q_SLOTS:
    void reconnect_loop();
    void connected();
    void disconnected();

    void recv_message(const QByteArray &buf);

private Q_SLOTS:

    // send slots
    void slot_send_info();
    void slot_mapload(double time, QString name);
    void slot_init(double time);
    void slot_random_init(double time, QString seed);
    void slot_random_seq(double time);

};

#endif // COMM_FMS_H
