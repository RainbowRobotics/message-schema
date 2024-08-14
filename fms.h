#ifndef FMS_H
#define FMS_H

// global defines
#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"
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

class FMS : public QObject
{
    Q_OBJECT
public:
    explicit FMS(QObject *parent = nullptr);
    ~FMS();
    std::mutex mtx;
    QWebSocket client;
    QTimer reconnect_timer;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    AUTOCONTROL *ctrl = NULL;

    // vars
    QString id = "";
    std::atomic<bool> is_connected = {false};

    // funcs
    void init();

Q_SIGNALS:

private Q_SLOTS:
    void reconnect_loop();

    void connected();
    void disconnected();
    void recv_message(QString message);

};

#endif // FMS_H
