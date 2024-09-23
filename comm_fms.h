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
    FMS_INFO fms_info;

    // funcs
    void init();

Q_SIGNALS:
    void signal_recv_path(std::vector<QString> node_path);
    void signal_recv_pause();
    void signal_recv_resume();
    void signal_recv_stop();

private Q_SLOTS:
    void reconnect_loop();

    void connected();
    void disconnected();
    void recv_message(QString message);

private Q_SLOTS:
    // recv slots
    void recv_path(std::vector<QString> node_path);
    void recv_pause();
    void recv_resume();
    void recv_stop();

    // send slots
    void slot_new_goal(Eigen::Matrix4d goal_tf, int preset);
    void slot_stop();

};

#endif // COMM_FMS_H
