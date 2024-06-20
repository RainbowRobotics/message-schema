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
    QJsonObject poseObj;
    poseObj["x"] = "0.3";
    poseObj["y"] = "0.2";
    poseObj["rz"] = "40.0";
    rootObj["pose"] = poseObj;

    // Adding the velocity object
    QJsonObject velObj;
    velObj["vx"] = "0";
    velObj["vy"] = "0";
    velObj["wz"] = "30.0";
    rootObj["vel"] = velObj;

    // Adding the motor array
    QJsonArray motorArray;
    QJsonObject motorObj1;
    motorObj1["connection"] = "true";
    motorObj1["status"] = "1";
    motorObj1["temp"] = "36.5";
    motorArray.append(motorObj1);

    QJsonObject motorObj2;
    motorObj2["connection"] = "true";
    motorObj2["status"] = "4";
    motorObj2["temp"] = "32.5";
    motorArray.append(motorObj2);

    rootObj["motor"] = motorArray;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"] = "48.2";
    powerObj["bat_out"] = "48.2";
    powerObj["bat_current"] = "0";
    powerObj["power"] = "0";
    powerObj["total_power"] = "0";
    rootObj["power"] = powerObj;

    // Adding the state object
    QJsonObject stateObj;
    stateObj["power"] = "true";
    stateObj["emo"] = "false";
    stateObj["charge"] = "false";
    stateObj["localization"] = "none"; // "none", "busy", "good", "fail"
    rootObj["state"] = stateObj;

    // Adding the condition object
    QJsonObject conditionObj;
    conditionObj["inlier_error"] = "0.05";
    conditionObj["inlier_ratio"] = "0.5";
    rootObj["condition"] = conditionObj;

    QJsonDocument doc(rootObj);
    QString str(doc.toJson());

    client.sendTextMessage(str);

    printf("[WS_SEND] status, time: %f\n", time);
}
