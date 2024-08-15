#include "fms.h"

FMS::FMS(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &FMS::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

FMS::~FMS()
{
    reconnect_timer.stop();

    if(is_connected)
    {
        client.close();
    }
}

void FMS::init()
{
    // set id
    QString _id;
    _id.sprintf("R_%lld", (long long)(get_time0()*1000));
    id = _id;
    printf("[FMS] ID: %s\n", _id.toLocal8Bit().data());

    reconnect_timer.start(3000);
    printf("[FMS] start reconnect timer\n", _id.toLocal8Bit().data());
}

void FMS::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr;
        server_addr.sprintf("ws://%s:12334", config->SERVER_IP.toLocal8Bit().data());
        client.open(QUrl(server_addr));
    }
}

void FMS::connected()
{
    if(!is_connected)
    {
        connect(&client, &QWebSocket::textMessageReceived, this, &FMS::recv_message);
        is_connected = true;

        printf("[FMS] connected\n");
    }
}

void FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::textMessageReceived, this, &FMS::recv_message);

        printf("[FMS] disconnected\n");
    }
}

void FMS::recv_message(QString message)
{

}
