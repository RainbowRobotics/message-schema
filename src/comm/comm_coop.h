#ifndef COMM_COOP_H
#define COMM_COOP_H

// global defines
#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

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

// stl
#include <shared_mutex>
#include <atomic>
#include <memory>

class COMM_COOP : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_COOP)
public:
    // make singleton
    static COMM_COOP* instance(QObject* parent = nullptr);

    // start coop module
    void init();

    // getter functions
    QString get_robot_id() const;
    bool is_connected() const;

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_localization_module(LOCALIZATION* _loc);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);

private:
    explicit COMM_COOP(QObject *parent = nullptr);
    ~COMM_COOP();

    mutable std::shared_mutex mtx;
    QWebSocket client;
    std::unique_ptr<QTimer> reconnect_timer;

    // other modules
    QObject* main{nullptr};
    CONFIG* config{nullptr};
    LOGGER* logger{nullptr};
    MOBILE* mobile{nullptr};
    LOCALIZATION* loc{nullptr};
    UNIMAP* unimap{nullptr};
    OBSMAP* obsmap{nullptr};
    AUTOCONTROL* ctrl{nullptr};

    // vars
    std::atomic<bool> is_connected_flag{false};
    std::atomic<double> last_send_time{0.0};
    QString robot_id;

    // utility functions
    QString get_json(const QJsonObject& json, const QString& key) const;
    void send_robot_info();
    void parse_control_message(const QJsonObject& data);
    void process_velocity_command(const QString& vel_str, double timestamp);
    bool should_send_message(double current_time) const;

Q_SIGNALS:
    void signal_send_info();

private Q_SLOTS:
    void slot_send_info();
    void reconnect_loop();
    void connected();
    void disconnected();
    void recv_message(const QByteArray &buf);
};

#endif // COMM_COOP_H
