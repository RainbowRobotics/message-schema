#include "comm_ms.h"

#include "mainwindow.h"

COMM_MS::COMM_MS(QObject *parent)
    : QObject(parent)
    , main(parent)
    , io(new sio::client())
    , reconnect_timer(this)
{
    // set recv callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    sio::socket::ptr sock = io->socket();
    io->set_logs_quiet();

    BIND_EVENT(sock, "motorinit", std::bind(&COMM_MS::recv_motorinit, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "move", std::bind(&COMM_MS::recv_move, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapping", std::bind(&COMM_MS::recv_mapping, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapload", std::bind(&COMM_MS::recv_mapload, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "localization", std::bind(&COMM_MS::recv_localization, this, _1, _2, _3, _4));

    io->set_open_listener(std::bind(&COMM_MS::sio_connected, this));
    io->set_close_listener(std::bind(&COMM_MS::sio_disconnected, this, _1));
    io->set_fail_listener(std::bind(&COMM_MS::sio_error, this));

    // connect recv signals -> recv slots
    connect(this, SIGNAL(signal_motorinit(double)), this, SLOT(slot_motorinit(double)));

    connect(this, SIGNAL(signal_move_jog(double, double, double, double)), this, SLOT(slot_move_jog(double, double, double, double)));
    connect(this, SIGNAL(signal_move_target(double, double, double, double, double, int, QString)), this, SLOT(slot_move_target(double, double, double, double, double, int, QString)));
    connect(this, SIGNAL(signal_move_goal(double, QString, int, QString)), this, SLOT(slot_move_goal(double, QString, int, QString)));
    connect(this, SIGNAL(signal_move_pause(double)), this, SLOT(slot_move_pause(double)));
    connect(this, SIGNAL(signal_move_resume(double)), this, SLOT(slot_move_resume(double)));
    connect(this, SIGNAL(signal_move_stop(double)), this, SLOT(slot_move_stop(double)));

    connect(this, SIGNAL(signal_mapping_start(double)), this, SLOT(slot_mapping_start(double)));
    connect(this, SIGNAL(signal_mapping_stop(double)), this, SLOT(slot_mapping_stop(double)));
    connect(this, SIGNAL(signal_mapping_save(double, QString)), this, SLOT(slot_mapping_save(double, QString)));
    connect(this, SIGNAL(signal_mapping_reload(double)), this, SLOT(slot_mapping_reload(double)));

    connect(this, SIGNAL(signal_mapload(double, QString)), this, SLOT(slot_mapload(double, QString)));

    connect(this, SIGNAL(signal_localization_autoinit(double)), this, SLOT(slot_localization_autoinit(double)));
    connect(this, SIGNAL(signal_localization_init(double, double, double, double, double)), this, SLOT(slot_localization_init(double, double, double, double, double)));
    connect(this, SIGNAL(signal_localization_start(double)), this, SLOT(slot_localization_start(double)));
    connect(this, SIGNAL(signal_localization_stop(double)), this, SLOT(slot_localization_stop(double)));

    // timer
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));
}

COMM_MS::~COMM_MS()
{
    reconnect_timer.stop();
    io->close();
}

QString COMM_MS::get_json(sio::message::ptr const& data, QString key)
{
    return QString::fromStdString(data->get_map()[key.toStdString()]->get_string());
}

void COMM_MS::init()
{
    //reconnect_timer.start(10000);
}

void COMM_MS::sio_connected()
{
    is_connected = true;
    printf("[COMM_MS] connected\n");
}

void COMM_MS::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;
    printf("[COMM_MS] disconnected\n");
}

void COMM_MS::sio_error()
{
    printf("[COMM_MS] some error\n");
}

void COMM_MS::reconnect_loop()
{
    if(is_connected == false)
    {
        io->connect("ws://localhost:11337");
        reconnect_cnt++;
        if(reconnect_cnt > 10)
        {
            reconnect_timer.stop();
            printf("[COMM_MS] server not opened, give up reconnect\n");
        }
    }
}


// recv parser -> emit recv signals
void COMM_MS::recv_motorinit(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        // action
        Q_EMIT signal_motorinit(time);

        printf("[COMM_MS] motorinit(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }
}

void COMM_MS::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");

        if(command == "jog")
        {
            double vx = get_json(data, "vx").toDouble();
            double vy = get_json(data, "vy").toDouble();
            double wz = get_json(data, "wz").toDouble();
            double time = get_json(data, "time").toDouble()/1000;

            // action
            Q_EMIT signal_move_jog(time, vx, vy, wz);
            printf("[COMM_MS] move, jog, t: %.3f, vel: %.3f, %.3f, %.3f\n", time, vx, vy, wz);
        }
        else if(command == "target")
        {
            double x = get_json(data, "x").toDouble();
            double y = get_json(data, "y").toDouble();
            double z = get_json(data, "z").toDouble();
            double rz = get_json(data, "rz").toDouble();
            int preset = get_json(data, "preset").toInt();
            QString method = get_json(data, "method");
            double time = get_json(data, "time").toDouble()/1000;

            // for response
            MOVE_INFO _last_move_info;
            _last_move_info.command = command;
            _last_move_info.x = x;
            _last_move_info.y = y;
            _last_move_info.z = z;
            _last_move_info.rz = rz;
            _last_move_info.preset = preset;
            _last_move_info.method = method;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();

            // action
            Q_EMIT signal_move_target(time, x, y, z, rz, preset, method);
            printf("[COMM_MS] move, target, t: %.3f, tgt: %.3f, %.3f, %.3f, %.3f, preset:%d, method:%s\n", time, x, y, z, rz, preset, method.toLocal8Bit().data());
        }
        else if(command == "goal")
        {
            QString id = get_json(data, "id");
            int preset = get_json(data, "preset").toInt();
            QString method = get_json(data, "method");
            double time = get_json(data, "time").toDouble()/1000;

            // for response
            MOVE_INFO _last_move_info;
            _last_move_info.command = command;
            _last_move_info.node_id = id;
            _last_move_info.preset = preset;
            _last_move_info.method = method;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();

            // action
            Q_EMIT signal_move_goal(time, id, preset, method);
            printf("[COMM_MS] move, goal, t: %.3f, tgt: %s, preset:%d, method:%s\n", time, id.toLocal8Bit().data(), preset, method.toLocal8Bit().data());
        }
        else if(command == "pause")
        {
            double time = get_json(data, "time").toDouble()/1000;
            Q_EMIT signal_move_pause(time);
        }
        else if(command == "resume")
        {
            double time = get_json(data, "time").toDouble()/1000;
            Q_EMIT signal_move_resume(time);
        }
        else if(command == "stop")
        {
            double time = get_json(data, "time").toDouble()/1000;
            Q_EMIT signal_move_stop(time);
        }
    }
}

void COMM_MS::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        // action
        if(command == "start")
        {
            last_send_kfrm_idx = 0;
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
        else if(command == "reload")
        {
            Q_EMIT signal_mapping_reload(time);
        }

        printf("[COMM_MS] mapping(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }
}

void COMM_MS::recv_mapload(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString name = get_json(data, "name");
        double time = get_json(data, "time").toDouble()/1000;

        Q_EMIT signal_mapload(time, name);

        printf("[COMM_MS] mapload(%s), t: %.3f\n", name.toLocal8Bit().data(), time);
    }
}

void COMM_MS::recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        if(command == "autoinit")
        {
            Q_EMIT signal_localization_autoinit(time);
        }
        else if(command == "init")
        {
            double x = get_json(data, "x").toDouble();
            double y = get_json(data, "y").toDouble();
            double z = get_json(data, "z").toDouble();
            double rz = get_json(data, "rz").toDouble();

            Q_EMIT signal_localization_init(time, x, y, z, rz);
        }
        else if(command == "start")
        {
            Q_EMIT signal_localization_start(time);
        }
        else if(command == "stop")
        {
            Q_EMIT signal_localization_stop(time);
        }

        printf("[COMM_MS] localization(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }
}


// send functions
void COMM_MS::send_status()
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "status";
    rootObj["time"] = QString::number((long long)(time*1000), 10);

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
    motorObj1["current"] = QString::number((double)ms.cur_m0/10.0, 'f', 3);
    motorArray.append(motorObj1);

    QJsonObject motorObj2;
    motorObj2["connection"] = (ms.connection_m1 == 1) ? "true" : "false";
    motorObj2["status"] = QString::number(ms.status_m1, 10);
    motorObj2["temp"] = QString::number(ms.temp_m1, 'f', 3);
    motorObj1["current"] = QString::number((double)ms.cur_m1/10.0, 'f', 3);
    motorArray.append(motorObj2);

    rootObj["motor"] = motorArray;

    // Adding the lidar array
    QJsonArray lidarArray;
    QJsonObject lidarObj1;
    lidarObj1["connection"] = lidar->is_connected_f ? "true" : "false";
    lidarObj1["port"] = "LAN";
    lidarObj1["serialnumber"] = "not yet";
    lidarArray.append(lidarObj1);

    QJsonObject lidarObj2;
    lidarObj2["connection"] = lidar->is_connected_b ? "true" : "false";
    lidarObj2["port"] = "LAN";
    lidarObj2["serialnumber"] = "not yet";
    lidarArray.append(lidarObj2);
    rootObj["lidar"] = lidarArray;

    // Adding the imu object
    Eigen::Vector3d imu = mobile->get_imu();

    QJsonObject imuObj;
    imuObj["gyr_x"] = QString::number(ms.imu_gyr_x*R2D, 'f', 3);
    imuObj["gyr_y"] = QString::number(ms.imu_gyr_y*R2D, 'f', 3);
    imuObj["gyr_z"] = QString::number(ms.imu_gyr_z*R2D, 'f', 3);
    imuObj["acc_x"] = QString::number(ms.imu_acc_x, 'f', 3);
    imuObj["acc_y"] = QString::number(ms.imu_acc_y, 'f', 3);
    imuObj["acc_z"] = QString::number(ms.imu_acc_z, 'f', 3);
    imuObj["imu_rx"] = QString::number(imu[0]*R2D, 'f', 3);
    imuObj["imu_ry"] = QString::number(imu[1]*R2D, 'f', 3);
    imuObj["imu_rz"] = QString::number(imu[2]*R2D, 'f', 3);
    rootObj["imu"] = imuObj;

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
    stateObj["map"] = unimap->map_dir.split("/").last();
    rootObj["state"] = stateObj;

    // Adding the condition object
    Eigen::Vector2d ieir = slam->get_cur_ieir();

    QString auto_state = "stop";
    if(ctrl->is_pause)
    {
        auto_state = "pause";
    }
    else if(ctrl->is_moving)
    {
        auto_state = "move";
    }

    QJsonObject conditionObj;
    conditionObj["inlier_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"] = QString::number(ieir[1], 'f', 3);
    conditionObj["mapping_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["mapping_ratio"] = QString::number(ieir[1], 'f', 3);
    conditionObj["auto_state"] = auto_state;
    conditionObj["obs_state"] = ctrl->get_obs_condition();
    rootObj["condition"] = conditionObj;

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("status", res);

    //printf("[COMM_MS] status, time: %f\n", time);
}

void COMM_MS::send_global_path()
{
    PATH path = ctrl->get_cur_global_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(size_t p = 0; p < path.pos.size(); p++)
    {
        Eigen::Vector3d P = path.pos[p];

        sio::array_message::ptr jsonObj = sio::array_message::create();
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[0], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[1], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[2], 'f', 3).toStdString()));
        jsonArray->get_vector().push_back(jsonObj);
    }

    // send
    io->socket()->emit("global_path", jsonArray);
    printf("[COMM_MS] global_path, num: %d\n", (int)path.pos.size());
}

void COMM_MS::send_local_path()
{
    PATH path = ctrl->get_cur_local_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(int p = 0; p < (int)path.pos.size(); p++)
    {
        if(p == 0 || p == (int)path.pos.size()-1 || p%10 == 0)
        {
            Eigen::Vector3d P = path.pos[p];

            sio::array_message::ptr jsonObj = sio::array_message::create();
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[2], 'f', 3).toStdString()));
            jsonArray->get_vector().push_back(jsonObj);
        }
    }

    // send
    io->socket()->emit("local_path", jsonArray);
    printf("[COMM_MS] local_path, num: %d\n", (int)path.pos.size());
}

void COMM_MS::send_lidar()
{
    double time = get_time0();

    TIME_POSE_PTS tpp = slam->get_cur_tpp();
    if(slam->is_loc && tpp.pts.size() > 0)
    {
        QJsonObject rootObj;

        // Adding the pose object
        Eigen::Matrix4d cur_tf = tpp.tf;
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

        sio::object_message::ptr rootObject = sio::object_message::create();
        sio::object_message::ptr poseObject = sio::object_message::create();
        poseObject->get_map()["x"] = sio::string_message::create(QString::number(cur_xi[0], 'f', 3).toStdString());
        poseObject->get_map()["y"] = sio::string_message::create(QString::number(cur_xi[1], 'f', 3).toStdString());
        poseObject->get_map()["rz"] = sio::string_message::create(QString::number(cur_xi[2]*R2D, 'f', 3).toStdString());
        rootObject->get_map()["pose"] = poseObject;

        sio::array_message::ptr jsonArray = sio::array_message::create();
        for(size_t p = 0; p < tpp.pts.size(); p++)
        {
            sio::array_message::ptr jsonObj = sio::array_message::create();

            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(tpp.pts[p][0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(tpp.pts[p][1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(tpp.pts[p][2], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(100, 'f', 3).toStdString()));

            jsonArray->get_vector().push_back(jsonObj);
        }

        rootObject->get_map()["data"] = jsonArray;

        // send
        io->socket()->emit("lidar_cloud", rootObject);
    }
    else
    {
        std::vector<Eigen::Vector3d> pts = lidar->get_cur_scan();

        QJsonObject rootObj;

        // Adding the pose object
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

        sio::object_message::ptr rootObject = sio::object_message::create();
        sio::object_message::ptr poseObject = sio::object_message::create();
        poseObject->get_map()["x"] = sio::string_message::create(QString::number(cur_xi[0], 'f', 3).toStdString());
        poseObject->get_map()["y"] = sio::string_message::create(QString::number(cur_xi[1], 'f', 3).toStdString());
        poseObject->get_map()["rz"] = sio::string_message::create(QString::number(cur_xi[2]*R2D, 'f', 3).toStdString());
        rootObject->get_map()["pose"] = poseObject;

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

        rootObject->get_map()["data"] = jsonArray;

        // send
        io->socket()->emit("lidar_cloud", rootObject);
    }

    //printf("[COMM_MS] lidar_cloud,  time: %f\n", time);
}

void COMM_MS::send_mapping_cloud()
{
    if(slam->is_slam && last_send_kfrm_idx < (int)slam->kfrm_storage.size())
    {
        // send kfrm
        double time = get_time0();

        slam->mtx.lock();
        KFRAME kfrm = slam->kfrm_storage[last_send_kfrm_idx];
        slam->mtx.unlock();

        sio::array_message::ptr jsonArray = sio::array_message::create();
        for(size_t p = 0; p < kfrm.pts.size(); p++)
        {
            if(kfrm.pts[p].do_cnt < config->SLAM_ICP_DO_ACCUM_NUM)
            {
                continue;
            }

            Eigen::Vector3d P(kfrm.pts[p].x, kfrm.pts[p].y, kfrm.pts[p].z);
            Eigen::Vector3d _P = kfrm.opt_G.block(0,0,3,3)*P + kfrm.opt_G.block(0,3,3,1);

            sio::array_message::ptr jsonObj = sio::array_message::create();

            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[2], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(kfrm.pts[p].r, 'f', 3).toStdString()));

            jsonArray->get_vector().push_back(jsonObj);
        }

        // send
        io->socket()->emit("mapping_cloud", jsonArray);
        printf("[COMM_MS] mapping_cloud, time: %f\n", time);
        last_send_kfrm_idx++;
    }
}

void COMM_MS::send_mapping_start_response(QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "start";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mapping", res);

    printf("[COMM_MS] mapping_response_start, time: %f\n", time);
}

void COMM_MS::send_mapping_stop_response()
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "stop";
    rootObj["result"] = "success";
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mapping", res);

    printf("[COMM_MS] mapping_response_stop, success, time: %f\n", time);
}

void COMM_MS::send_mapping_save_response(QString name, QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "save";
    rootObj["name"] = name;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mapping", res);

    printf("[COMM_MS] mapping_response_save, %s, %s, time: %f\n", name.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_MS::send_mapload_response(QString name, QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["name"] = name;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mapload", res);

    printf("[COMM_MS] mapload_response, %s, %s, time: %f\n", name.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_MS::send_localization_response(QString command, QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = command;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("localization", res);

    printf("[COMM_MS] localization_response, %s, %s, time: %f\n", command.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_MS::send_move_target_response(double x, double y, double z, double rz, int preset, QString method, QString result, QString message)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "target";
    rootObj["x"] = QString::number(x, 'f', 3);
    rootObj["y"] = QString::number(y, 'f', 3);
    rootObj["z"] = QString::number(z, 'f', 3);
    rootObj["rz"] = QString::number(rz, 'f', 3);
    rootObj["preset"] = QString::number(preset, 10);
    rootObj["method"] = method;
    rootObj["result"] = result;
    rootObj["message"] = message; // when result failed
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("move", res);

    printf("[COMM_MS] move_target_response, %s, %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_MS::send_move_goal_response(QString node_id, int preset, QString method, QString result, QString message)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "goal";
    rootObj["id"] = node_id;
    rootObj["preset"] = QString::number(preset, 10);
    rootObj["method"] = method;
    rootObj["result"] = result;
    rootObj["message"] = message; // when result failed
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("move", res);

    printf("[COMM_MS] move_goal_response, %s, %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_MS::send_move_pause_response(QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "pause";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("move", res);

    printf("[COMM_MS] move_pause_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}

void COMM_MS::send_move_resume_response(QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "resume";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("move", res);

    printf("[COMM_MS] move_resume_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}

void COMM_MS::send_move_stop_response(QString result)
{
    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "stop";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("move", res);

    printf("[COMM_MS] move_stop_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}


// recv slots
void COMM_MS::slot_motorinit(double time)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MotorInit();
}

void COMM_MS::slot_move_jog(double time, double vx, double vy, double wz)
{
    mobile->move(vx, vy, wz*D2R);
}

void COMM_MS::slot_move_target(double time, double x, double y, double z, double rz, int preset, QString method)
{
    if(method == "pp")
    {
        if(unimap->is_loaded == false)
        {
            QString result = "reject";
            QString message = "map not loaded";
            send_move_target_response(x, y, z, rz, preset, method, result, message);
            return;
        }

        if(slam->is_loc == false)
        {
            QString result = "reject";
            QString message = "no localization";
            send_move_target_response(x, y, z, rz, preset, method, result, message);
            return;
        }

        if(x < unimap->map_min_x || x > unimap->map_max_x || y < unimap->map_min_y || y > unimap->map_max_y)
        {
            QString result = "reject";
            QString message = "target location out of range";
            send_move_target_response(x, y, z, rz, preset, method, result, message);
            return;
        }

        Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(x,y,rz*D2R));
        goal_tf(2,3) = z;

        if(obsmap->is_tf_collision(goal_tf))
        {
            QString result = "reject";
            QString message = "target location occupied";
            send_move_target_response(x, y, z, rz, preset, method, result, message);
            return;
        }

        // pure pursuit
        ctrl->move_pp(goal_tf, preset);

        QString result = "accept";
        QString message = "";
        send_move_target_response(x, y, z, rz, preset, method, result, message);
    }
    else if(method == "tng")
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_target_response(x, y, z, rz, preset, method, result, message);
    }
    else
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_target_response(x, y, z, rz, preset, method, result, message);
    }
}

void COMM_MS::slot_move_goal(double time, QString node_id, int preset, QString method)
{
    if(method == "pp")
    {
        if(unimap->is_loaded == false)
        {
            QString result = "reject";
            QString message = "map not loaded";
            send_move_goal_response(node_id, preset, method, result, message);
            return;
        }

        if(slam->is_loc == false)
        {
            QString result = "reject";
            QString message = "no localization";
            send_move_goal_response(node_id, preset, method, result, message);
            return;
        }

        if(node_id == "")
        {
            QString result = "reject";
            QString message = "empty node id";
            send_move_goal_response(node_id, preset, method, result, message);
            return;
        }

        NODE* node = unimap->get_node_by_id(node_id);
        if(node_id == NULL)
        {
            QString result = "reject";
            QString message = "null node";
            send_move_goal_response(node_id, preset, method, result, message);
            return;
        }

        Eigen::Matrix4d goal_tf = node->tf;

        // pure pursuit
        ctrl->move_pp(goal_tf, preset);

        QString result = "accept";
        QString message = "";
        send_move_goal_response(node_id, preset, method, result, message);
    }
    else if(method == "tng")
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_goal_response(node_id, preset, method, result, message);
    }
    else
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_goal_response(node_id, preset, method, result, message);
    }
}

void COMM_MS::slot_move_pause(double time)
{
    ctrl->is_pause = true;

    QString result = "accept";
    send_move_pause_response(result);
}

void COMM_MS::slot_move_resume(double time)
{
    ctrl->is_pause = false;

    QString result = "accept";
    send_move_resume_response(result);
}

void COMM_MS::slot_move_stop(double time)
{
    ctrl->stop();

    QString result = "accept";
    send_move_stop_response(result);
}

void COMM_MS::slot_move_succeed(QString message)
{
    mtx.lock();
    MOVE_INFO _last_move_info = last_move_info;
    mtx.unlock();

    if(_last_move_info.command == "target")
    {
        send_move_target_response(_last_move_info.x, _last_move_info.y, _last_move_info.z, _last_move_info.rz, _last_move_info.preset, _last_move_info.method, "success", message);
    }
    else if(_last_move_info.command == "goal")
    {
        send_move_goal_response(_last_move_info.node_id, _last_move_info.preset, _last_move_info.method, "success", message);
    }
}

void COMM_MS::slot_move_failed(QString message)
{
    mtx.lock();
    MOVE_INFO _last_move_info = last_move_info;
    mtx.unlock();

    if(_last_move_info.command == "target")
    {
        send_move_target_response(_last_move_info.x, _last_move_info.y, _last_move_info.z, _last_move_info.rz, _last_move_info.preset, _last_move_info.method, "fail", message);
    }
    else if(_last_move_info.command == "goal")
    {
        send_move_goal_response(_last_move_info.node_id, _last_move_info.preset, _last_move_info.method, "fail", message);
    }
}

void COMM_MS::slot_mapping_start(double time)
{
    MainWindow* _main = (MainWindow*)main;
    if(lidar->is_connected_f)
    {
        _main->bt_MapBuild();
        send_mapping_start_response("success");
    }
    else
    {
        send_mapping_start_response("fail");
    }
}

void COMM_MS::slot_mapping_stop(double time)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MapSave();
    send_mapping_stop_response();
}

void COMM_MS::slot_mapping_save(double time, QString name)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MapSave();

    QString save_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + name;
    std::string command = "cp -r " + _main->map_dir.toStdString() + " " + save_dir.toStdString();
    int result = std::system(command.c_str());
    if(result == 0)
    {
        send_mapping_save_response(name, "success");
        printf("[COMM_MS] map save succeed, %s\n", save_dir.toLocal8Bit().data());
    }
    else
    {
        send_mapping_save_response(name, "fail");
        printf("[COMM_MS] map save failed, %s\n", save_dir.toLocal8Bit().data());
    }
}

void COMM_MS::slot_mapping_reload(double time)
{
    last_send_kfrm_idx = 0;
    printf("[COMM_MS] mapping reload\n");
}


void COMM_MS::slot_mapload(double time, QString name)
{
    MainWindow* _main = (MainWindow*)main;

    QString load_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + name;
    if(!load_dir.isNull())
    {
        _main->map_dir = load_dir;
        unimap->load_map(load_dir);
        _main->all_update();

        if(unimap->is_loaded)
        {
            send_mapload_response(name, "success");
        }
        else
        {
            send_mapload_response(name, "fail");
        }
    }
}


void COMM_MS::slot_localization_autoinit(double time)
{
    send_localization_response("autoinit", "fail");
}

void COMM_MS::slot_localization_init(double time, double x, double y, double z, double rz)
{
    if(unimap->is_loaded == false || lidar->is_connected_f == false)
    {
        send_localization_response("init", "fail");
        return;
    }

    // manual init
    slam->mtx.lock();
    slam->cur_tf = se2_to_TF(Eigen::Vector3d(x, y, rz*D2R));
    slam->mtx.unlock();

    send_localization_response("init", "success");
}

void COMM_MS::slot_localization_start(double time)
{
    slam->localization_start();
}

void COMM_MS::slot_localization_stop(double time)
{
    slam->localization_stop();
}


