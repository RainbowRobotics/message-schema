#ifndef WS_CLIENT_H
#define WS_CLIENT_H

#include "global_defines.h"
#include "utils.h"

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
