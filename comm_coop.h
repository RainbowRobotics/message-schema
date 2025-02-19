#ifndef COMM_COOP_H
#define COMM_COOP_H

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
#include "lvx_loc.h"

// qt
#include <QObject>
#include <QTimer>
#include <QWebSocket>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>

class COMM_COOP : public QObject
{
    Q_OBJECT
public:
    explicit COMM_COOP(QObject *parent = nullptr);
    ~COMM_COOP();

    std::recursive_mutex mtx;
    QWebSocket client;
    QTimer reconnect_timer;

    // other modules
    QObject *main = NULL;
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;
    LVX_LOC *lvx = NULL;

    // vars
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};
    QString robot_id = "";

    // funcs
    void init();
    QString get_json(QJsonObject& json, QString key);

Q_SIGNALS:
    void signal_send_info();

private Q_SLOTS:
    void slot_send_info();

private Q_SLOTS:
    void reconnect_loop();
    void connected();
    void disconnected();
    void recv_message(const QByteArray &buf);

};

#endif // COMM_COOP_H
