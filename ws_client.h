#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include "global_defines.h"
#include "utils.h"

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

    // params
    QString ws_addr = "ws://127.0.0.1:11337";
    QWebSocket client;
    QTimer reconnect_timer;
    std::atomic<bool> is_connected = {false};

    void init();
    void send_status();

Q_SIGNALS:
    void recv_command_motorinit(double time);
    void recv_command_move(double time, double vx, double vy, double wz);

private Q_SLOTS:
    void connected();
    void disconnected();
    void reconnect_loop();
    void recv_message(QString message);
};

#endif // WS_CLIENT_H
