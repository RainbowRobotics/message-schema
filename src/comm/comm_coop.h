#ifndef COMM_COOP_H
#define COMM_COOP_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "localization.h"
#include "mobile.h"
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

class COMM_COOP : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_COOP)
public:
    // make singleton
    static COMM_COOP* instance(QObject* parent = nullptr);

    // start coop module
    void init();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_localization_module(LOCALIZATION* _loc);
    void set_autocontrol_module(AUTOCONTROL *_ctrl);

private:
    explicit COMM_COOP(QObject *parent = nullptr);
    ~COMM_COOP();

    std::mutex mtx;
    QWebSocket client;
    QTimer* reconnect_timer;

    // other modules
    QObject* main;
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    LOCALIZATION *loc;
    UNIMAP *unimap;
    OBSMAP *obsmap;
    AUTOCONTROL *ctrl;

    // vars
    std::atomic<bool> is_connected = {false};
    std::atomic<double> last_send_time = {0};
    QString robot_id = "";

    // funcs
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
