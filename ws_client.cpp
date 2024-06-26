#include "ws_client.h"

WS_CLIENT::WS_CLIENT(QObject *parent)
    : QObject(parent)
    , reconnect_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &WS_CLIENT::connected);
    connect(&client, &QWebSocket::disconnected, this, &WS_CLIENT::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

WS_CLIENT::~WS_CLIENT()
{
    client.close();
}

void WS_CLIENT::init()
{
    client.open(QUrl(ws_addr));
    reconnect_timer.start(1000);
}

void WS_CLIENT::connected()
{
    connect(&client, &QWebSocket::textMessageReceived, this, &WS_CLIENT::recv_message);
    is_connected = true;

    printf("[WS] server connected\n");
}

void WS_CLIENT::disconnected()
{
    is_connected = false;

    // clear parameters
    disconnect(&client, &QWebSocket::textMessageReceived, this, &WS_CLIENT::recv_message);

    //printf("[WS] server disconnected\n");
}

void WS_CLIENT::reconnect_loop()
{
    if(client.state() == QAbstractSocket::ConnectedState)
    {

    }
    else if(client.state() == QAbstractSocket::ConnectingState)
    {

    }
    else
    {
        if(is_connected == false)
        {
            client.open(QUrl(ws_addr));
        }
    }
}

void WS_CLIENT::recv_message(QString message)
{
    QJsonObject json = QJsonDocument::fromJson(message.toUtf8()).object();

    // move command parsing
    if(json["command"] == "move")
    {
        double vx = json["vx"].toString().toDouble(); // mps
        double vy = json["vy"].toString().toDouble();
        double wz = json["wz"].toString().toDouble(); // dps
        double time = json["time"].toString().toDouble()/1000; // UTC, msec -> sec

        Q_EMIT recv_command_move(time, vx, vy, wz);
        printf("[WS_RECV] MOVE, time:%.3f, vx:%.3f, vy:%.3f, wz:%.3f\n", time, vx, vy, wz);
    }

    // motorinit command parsing
    if(json["command"] == "motorinit")
    {
        double time = json["time"].toString().toDouble()/1000; // UTC, msec -> sec

        Q_EMIT recv_command_motorinit(time);
        printf("[WS_RECV] motorinit, time:%.3f\n", time);
    }
}

void WS_CLIENT::send_status()
{
    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "status";
    rootObj["time"] = QString::number((int)(time*1000), 10);

    // Adding the pose object
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

    QJsonObject poseObj;
    poseObj["x"] = QString::number(cur_xi[0], 'f', 3);
    poseObj["y"] = QString::number(cur_xi[1], 'f', 3);
    poseObj["rz"] = QString::number(cur_xi[2]*R2D, 'f', 3);
    rootObj["pose"] = poseObj;

    // Adding the velocity object
    MOBILE_POSE mo = mobile->get_pose();

    QJsonObject velObj;
    velObj["vx"] = QString::number(mo.vel[0], 'f', 3);
    velObj["vy"] = QString::number(mo.vel[1], 'f', 3);
    velObj["wz"] = QString::number(mo.vel[2]*R2D, 'f', 3);
    rootObj["vel"] = velObj;

    // Adding the motor array
    MOBILE_STATUS ms = mobile->get_status();

    QJsonArray motorArray;
    QJsonObject motorObj1;
    motorObj1["connection"] = (ms.connection_m0 == 1) ? "true" : "false";
    motorObj1["status"] = QString::number(ms.status_m0, 10);
    motorObj1["temp"] = QString::number(ms.temp_m0, 'f', 3);
    motorArray.append(motorObj1);

    QJsonObject motorObj2;
    motorObj2["connection"] = (ms.connection_m1 == 1) ? "true" : "false";
    motorObj2["status"] = QString::number(ms.status_m1, 10);
    motorObj2["temp"] = QString::number(ms.temp_m1, 'f', 3);
    motorArray.append(motorObj2);

    rootObj["motor"] = motorArray;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"] = QString::number(ms.bat_in, 'f', 3);
    powerObj["bat_out"] = QString::number(ms.bat_out, 'f', 3);
    powerObj["bat_current"] = QString::number(ms.bat_current, 'f', 3);
    powerObj["power"] = QString::number(ms.power, 'f', 3);
    powerObj["total_power"] = QString::number(ms.total_power, 'f', 3);
    rootObj["power"] = powerObj;

    // Adding the state object
    QString cur_loc_state = slam->get_cur_loc_state();

    QJsonObject stateObj;
    stateObj["power"] = (ms.power_state == 1) ? "true" : "false";
    stateObj["emo"] = (ms.emo_state == 1) ? "true" : "false";
    stateObj["charge"] = (ms.charge_state == 1) ? "true" : "false";
    stateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    rootObj["state"] = stateObj;

    // Adding the condition object
    Eigen::Vector2d ieir = slam->get_cur_ieir();

    QJsonObject conditionObj;
    conditionObj["inlier_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"] = QString::number(ieir[1], 'f', 3);
    rootObj["condition"] = conditionObj;

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[WS_SEND] status, time: %f\n", time);
}

void WS_CLIENT::send_lidar()
{
    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "lidar";
    rootObj["time"] = QString::number((int)(time*1000), 10);

    // lidar raw
    std::vector<Eigen::Vector3d> pts = lidar->get_cur_scan();
    QJsonArray pointArray;

    for(size_t p = 0; p < pts.size(); p++)
    {
        QJsonArray array;
        array.append(QString::number(pts[p][0], 'f', 3));
        array.append(QString::number(pts[p][1], 'f', 3));
        array.append(QString::number(pts[p][2], 'f', 3));
        array.append("100");

        pointArray.push_back(array);
    }
    rootObj["points"] = pointArray;

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[WS_SEND] lidar, time: %f\n", time);
}

void WS_CLIENT::send_mapping()
{

}
