#include "ws_client.h"

WS_CLIENT::WS_CLIENT(QObject *parent)
    : QObject(parent)
    , io(new sio::client())
    , reconnect_timer(this)
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    sio::socket::ptr sock = io->socket();
    BIND_EVENT(sock, "motorinit", std::bind(&WS_CLIENT::recv_motorinit, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "move", std::bind(&WS_CLIENT::recv_move, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapping", std::bind(&WS_CLIENT::recv_mapping, this, _1, _2, _3, _4));

    io->set_open_listener(std::bind(&WS_CLIENT::sio_connected, this));
    io->set_close_listener(std::bind(&WS_CLIENT::sio_disconnected, this, _1));
    io->set_fail_listener(std::bind(&WS_CLIENT::sio_error, this));

    // timer
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

WS_CLIENT::~WS_CLIENT()
{
    io->close();
}

QString WS_CLIENT::get_json(sio::message::ptr const& data, QString key)
{
    return QString::fromStdString(data->get_map()[key.toStdString()]->get_string());
}

void WS_CLIENT::init()
{
    if(config->SIM_MODE == 1)
    {
        printf("[WS] simulation mode\n");
        return;
    }

    reconnect_timer.start(3000);
}

void WS_CLIENT::sio_connected()
{
    is_connected = true;
    printf("[WS] connected\n");
}

void WS_CLIENT::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;
    printf("[WS] disconnected\n");
}

void WS_CLIENT::sio_error()
{
    printf("[WS] some error\n");
}

void WS_CLIENT::reconnect_loop()
{
    if(is_connected == false)
    {
        io->connect("ws://localhost:11337");
    }
}

void WS_CLIENT::recv_motorinit(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        // action
        Q_EMIT signal_motorinit(time);

        printf("[WS_RECV] motorinit(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }
}

void WS_CLIENT::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double vx = get_json(data, "vx").toDouble();
        double vy = get_json(data, "vy").toDouble();
        double wz = get_json(data, "wz").toDouble();
        double time = get_json(data, "time").toDouble()/1000;

        // action
        Q_EMIT signal_move(time, vx, vy, wz);
        printf("[WS_RECV] move, t: %.3f, vx: %.3f, vy: %.3f, wz: %.3f\n", time, vx, vy, wz);
    }
}

void WS_CLIENT::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        // action
        if(command == "start")
        {
            Q_EMIT signal_mapping_start(time);
        }
        else if(command == "stop")
        {
            Q_EMIT signal_mapping_stop(time);
        }
        else if(command == "save")
        {
            QString name = get_json(data, "name");
            Q_EMIT signal_mapping_save(time, name);
        }

        printf("[WS_RECV] mapping(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
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
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("status", res);

    printf("[WS_SEND] status, time: %f\n", time);
}

void WS_CLIENT::send_lidar()
{
    double time = get_time();
    std::vector<Eigen::Vector3d> pts = lidar->get_cur_scan();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(size_t p = 0; p < pts.size(); p++)
    {
        sio::array_message::ptr jsonObj = sio::array_message::create();

        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][0], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][1], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][2], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(100, 'f', 3).toStdString()));

        jsonArray->get_vector().push_back(jsonObj);
    }

    // send
    io->socket()->emit("lidar_cloud", jsonArray);
    printf("[WS_SEND] lidar_cloud, time: %f\n", time);
}

void WS_CLIENT::send_mapping()
{

}
