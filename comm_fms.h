#ifndef COMM_FMS_H
#define COMM_FMS_H

// global defines
#include "global_defines.h"
#include "utils.h"

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

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);

Q_SIGNALS:

private Q_SLOTS:
    void reconnect_loop();

    void connected();
    void disconnected();

    void recv_message(QString message);

private Q_SLOTS:

    // send slots
    void send_info();

};

#endif // COMM_FMS_H
