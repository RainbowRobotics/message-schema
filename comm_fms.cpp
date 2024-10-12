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
    // make robot id
    QString _robot_id;
    _robot_id.sprintf("R_%lld", (long long)(get_time0()*1000));

    // update robot id
    robot_id = _robot_id;
    printf("[COMM_FMS] ID: %s\n", robot_id.toLocal8Bit().data());

    // start reconnect loop
    reconnect_timer.start(3000);
    printf("[COMM_FMS] start reconnect timer\n");
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
void COMM_FMS::send_info()
{
    double time = get_time0();
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Matrix4d goal_tf = ctrl->get_cur_goal_tf();
    QString cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
    QString goal_node_id = unimap->get_node_id_edge(goal_tf.block(0,3,3,1));
    QString state = ctrl->get_cur_state();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "info";
    rootObj["robot_id"] = robot_id;
    rootObj["cur_tf"] = TF_to_string(cur_tf);
    rootObj["goal_tf"] = TF_to_string(goal_tf);
    rootObj["cur_node_id"] = cur_node_id;
    rootObj["goal_node_id"] = goal_node_id;
    rootObj["state"] = state;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

