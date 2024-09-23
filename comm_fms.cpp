#include "comm_fms.h"

COMM_FMS::COMM_FMS(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &COMM_FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_FMS::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));    
}

COMM_FMS::~COMM_FMS()
{
    reconnect_timer.stop();

    if(is_connected)
    {
        client.close();
    }
}

void COMM_FMS::init()
{
    // set id
    QString _id;
    _id.sprintf("R_%lld", (long long)(get_time0()*1000));

    mtx.lock();
    fms_info.id = _id;
    mtx.unlock();
    printf("[COMM_FMS] ID: %s\n", _id.toLocal8Bit().data());

    reconnect_timer.start(3000);
    printf("[COMM_FMS] start reconnect timer\n", _id.toLocal8Bit().data());
}

void COMM_FMS::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr;
        server_addr.sprintf("ws://%s:12334", config->SERVER_IP.toLocal8Bit().data());
        client.open(QUrl(server_addr));
    }
}

void COMM_FMS::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        connect(&client, &QWebSocket::textMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = true;
        printf("[COMM_FMS] connected\n");
    }
}

void COMM_FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::textMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = false;
        printf("[COMM_FMS] disconnected\n");
    }
}

// recv callback
void COMM_FMS::recv_message(QString message)
{

}

// recv slots
void COMM_FMS::recv_path(std::vector<QString> node_path)
{

}

void COMM_FMS::recv_pause()
{

}

void COMM_FMS::recv_resume()
{

}

void COMM_FMS::recv_stop()
{

}

// send slots
void COMM_FMS::slot_new_goal(Eigen::Matrix4d goal_tf, int preset)
{
    double time = get_time0();
    QString node_id = unimap->get_node_id_nn(goal_tf.block(0,3,3,1));

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "new_goal";
    rootObj["node_id"] = node_id;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_FMS] new_goal, node_id: %s, preset: %d, time: %f\n", node_id.toLocal8Bit().data(), preset, time);
}

void COMM_FMS::slot_stop()
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "stop";
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_FMS] stop, time: %f\n", time);
}









