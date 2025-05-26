#include "comm_rrs.h"

#include "mainwindow.h"

COMM_RRS::COMM_RRS(QObject *parent)
    : QObject(parent)
    , main(parent)
    , io(new sio::client())
{
    // set recv callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    sio::socket::ptr sock = io->socket();
    io->set_open_listener(std::bind(&COMM_RRS::sio_connected, this));
    io->set_close_listener(std::bind(&COMM_RRS::sio_disconnected, this, _1));
    io->set_fail_listener(std::bind(&COMM_RRS::sio_error, this));

    // bind recv callback function this func emit signal
    BIND_EVENT(sock, "move",            std::bind(&COMM_RRS::recv_move,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "localization",    std::bind(&COMM_RRS::recv_localization,      this, _1, _2, _3, _4));
    BIND_EVENT(sock, "load",            std::bind(&COMM_RRS::recv_load,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "randomseq",       std::bind(&COMM_RRS::recv_randomseq,         this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapping",         std::bind(&COMM_RRS::recv_mapping,           this, _1, _2, _3, _4));
    BIND_EVENT(sock, "dock",            std::bind(&COMM_RRS::recv_dock,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "lidarOnOff",      std::bind(&COMM_RRS::recv_view_lidar_on_off, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "pathOnOff",       std::bind(&COMM_RRS::recv_view_path_on_off,  this, _1, _2, _3, _4));
    BIND_EVENT(sock, "led",             std::bind(&COMM_RRS::recv_led,               this, _1, _2, _3, _4));
    BIND_EVENT(sock, "motor",           std::bind(&COMM_RRS::recv_motor,             this, _1, _2, _3, _4));
    BIND_EVENT(sock, "path",            std::bind(&COMM_RRS::recv_path,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobs",            std::bind(&COMM_RRS::recv_vobs,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobsRobots",      std::bind(&COMM_RRS::recv_vobs_robots,       this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobsClosures",    std::bind(&COMM_RRS::recv_vobs_closures,     this, _1, _2, _3, _4));

    // connect recv signals -> recv slots
    connect(this, SIGNAL(signal_move(DATA_MOVE)),                 this, SLOT(slot_move(DATA_MOVE)));
    connect(this, SIGNAL(signal_localization(DATA_LOCALIZATION)), this, SLOT(slot_localization(DATA_LOCALIZATION)));
    connect(this, SIGNAL(signal_load(DATA_LOAD)),                 this, SLOT(slot_load(DATA_LOAD)));
    connect(this, SIGNAL(signal_randomseq(DATA_RANDOMSEQ)),       this, SLOT(slot_randomseq(DATA_RANDOMSEQ)));
    connect(this, SIGNAL(signal_mapping(DATA_MAPPING)),           this, SLOT(slot_mapping(DATA_MAPPING)));
    connect(this, SIGNAL(signal_dock(DATA_DOCK)),                 this, SLOT(slot_dock(DATA_DOCK)));
    connect(this, SIGNAL(signal_view_lidar(DATA_VIEW_LIDAR)),     this, SLOT(slot_view_lidar(DATA_VIEW_LIDAR)));
    connect(this, SIGNAL(signal_view_path(DATA_VIEW_PATH)),       this, SLOT(slot_view_path(DATA_VIEW_PATH)));
    connect(this, SIGNAL(signal_led(DATA_LED)),                   this, SLOT(slot_led(DATA_LED)));
    connect(this, SIGNAL(signal_motor(DATA_MOTOR)),               this, SLOT(slot_motor(DATA_MOTOR)));
    connect(this, SIGNAL(signal_path(DATA_PATH)),                 this, SLOT(slot_path(DATA_PATH)));
    connect(this, SIGNAL(signal_vobs(DATA_VOBS)),                 this, SLOT(slot_vobs(DATA_VOBS)));
    connect(this, SIGNAL(signal_vobs_r(DATA_VOBS_R)),             this, SLOT(slot_vobs_r(DATA_VOBS_R)));
    connect(this, SIGNAL(signal_vobs_c(DATA_VOBS_C)),             this, SLOT(slot_vobs_c(DATA_VOBS_C)));
}

COMM_RRS::~COMM_RRS()
{
    io->socket()->off_all();
    io->socket()->off_error();
    io->sync_close();
}

QString COMM_RRS::get_json(sio::message::ptr const& data, QString key)
{
    if(!data)
    {
        return "";
    }

    auto data_map = data->get_map();
    auto it = data_map.find(key.toStdString());
    if(it == data_map.end() || !it->second)
    {
        return "";
    }

    if(it->second->get_flag() == sio::message::flag_string)
    {
        return QString::fromStdString(it->second->get_string());
    }

    return "";
}

QString COMM_RRS::get_multi_state()
{
    mtx.lock();
    QString res = multi_state;
    mtx.unlock();

    return res;
}

QByteArray COMM_RRS::get_last_msg()
{
    mtx.lock();
    QByteArray res = lastest_msg_str;
    mtx.unlock();

    return res;
}

void COMM_RRS::init()
{
    if(config->USE_COMM_RRS)
    {
        std::map<std::string, std::string> query;
        query["name"] = "slamnav";

        io->connect("ws://localhost:11337", query);
    }
}

void COMM_RRS::sio_connected()
{
    is_connected = true;
    ctrl->is_rrs = true;
    logger->write_log("[COMM_RRS] connected", "Green");
}

void COMM_RRS::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;
    ctrl->is_rrs = false;
    logger->write_log("[COMM_RRS] disconnected", "Green");
}

void COMM_RRS::sio_error()
{
    logger->write_log("[COMM_RRS] some error", "Red");
}

// recv parser -> emit recv signals
void COMM_RRS::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MOVE msg;
        msg.command = get_json(data, "command"); // "goal", "jog", "target", "pause", "resume", "stop"
        msg.method = get_json(data, "method");
        msg.goal_node_id = get_json(data, "goal_id");
        msg.preset = get_json(data, "preset").toInt();
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble()*D2R;
        msg.jog_val[0] = get_json(data, "vx").toDouble();
        msg.jog_val[1] = get_json(data, "vy").toDouble();
        msg.jog_val[2] = get_json(data, "wz").toDouble()*D2R;
        msg.escape_dir = get_json(data, "escape_dir");
        msg.time = get_json(data, "time").toDouble()/1000;

        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_move(msg);
    }
}

void COMM_RRS::recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOCALIZATION msg;
        msg.command = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"
        msg.seed = get_json(data, "seed");
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble();
        msg.time = get_json(data, "time").toDouble()/1000;

        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_localization(msg);
    }
}

void COMM_RRS::recv_load(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOAD msg;
        msg.command = get_json(data, "command"); // "mapload", "topoload", "configload"
        msg.map_name = get_json(data, "name");
        msg.time = get_json(data, "time").toDouble()/1000;

        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_load(msg);
    }
}

void COMM_RRS::recv_randomseq(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_RANDOMSEQ msg;
        msg.command = get_json(data, "command"); // "randomseq"
        msg.time = get_json(data, "time").toDouble()/1000;

        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_randomseq(msg);
    }
}

void COMM_RRS::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MAPPING msg;
        msg.command = get_json(data, "command"); // "start", "stop", "save", "name", "reload"
        msg.time = get_json(data, "time").toDouble()/1000;

        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_mapping(msg);
    }
}

void COMM_RRS::recv_dock(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_DOCK msg;
        msg.command = get_json(data, "command"); // "dock", "undock"
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_dock(msg);
    }
}

void COMM_RRS::recv_view_lidar_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_LIDAR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble()/1000;;

        // action
        logger->write_log(QString("[COMM_RRS] recv, lidar_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_view_lidar(msg);
    }
}

void COMM_RRS::recv_view_path_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_PATH msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        logger->write_log(QString("[COMM_RRS] recv, path_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_view_path(msg);
    }
}

void COMM_RRS::recv_led(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LED msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.led = get_json(data, "led");
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        logger->write_log(QString("[COMM_RRS] recv, led_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_led(msg);
    }
}

void COMM_RRS::recv_motor(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MOTOR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        logger->write_log(QString("[COMM_RRS] recv, motor_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_motor(msg);
    }
}

void COMM_RRS::recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_PATH msg;
        msg.command = get_json(data, "command"); // "path"
        msg.path = get_json(data, "path");
        msg.vobs_robots = get_json(data, "vobs_r");
        msg.vobs_closures = get_json(data, "vobs_c");
        msg.preset = get_json(data, "preset").toInt();
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        //logger->write_log(QString("[COMM_RRS] recv, command: %1, path: %2, time: %3").arg(msg.command).arg(msg.path). arg(msg.time), "Green");
        Q_EMIT signal_path(msg);
    }
}

void COMM_RRS::recv_vobs(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VOBS msg;
        msg.command = get_json(data, "command"); // "vobs"
        msg.vobs_robots = get_json(data, "vobs_r");
        msg.vobs_clousers = get_json(data, "vobs_c");
        msg.is_vobs_closures_change = get_json(data, "is_vobs_c");
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        QString res = QString("[COMM_RRS] recv, command: %1, vobs_r: %2, vobs_c: %3, time: %4").arg(msg.command).arg(msg.vobs_robots).arg(msg.vobs_clousers).arg(msg.time);
        printf("%s\n", res.toLocal8Bit().data());

        //logger->write_log(QString("[COMM_RRS] recv, command: %1, vobs: %2, time: %3").arg(msg.command).arg(msg.vobs).arg(msg.time), "Green");
        Q_EMIT signal_vobs(msg);
    }
}

void COMM_RRS::recv_vobs_robots(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VOBS_R msg;
        msg.command = get_json(data, "command"); // "vobs_robots"
        msg.vobs = get_json(data, "vobs");
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        QString res = QString("[COMM_RRS] recv, command: %1, vobs: %2, time: %3").arg(msg.command).arg(msg.vobs).arg(msg.time);
        printf("%s\n", res.toLocal8Bit().data());

        //logger->write_log(QString("[COMM_RRS] recv, command: %1, vobs: %2, time: %3").arg(msg.command).arg(msg.vobs).arg(msg.time), "Green");
        Q_EMIT signal_vobs_r(msg);
    }
}

void COMM_RRS::recv_vobs_closures(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VOBS_C msg;
        msg.command = get_json(data, "command"); // "vobs_closures"
        msg.vobs = get_json(data, "vobs");
        msg.time = get_json(data, "time").toDouble()/1000;

        // action
        //logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        Q_EMIT signal_vobs_c(msg);
    }
}

// send functions
void COMM_RRS::send_status()
{
    if(!is_connected)
    {
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;
    MOBILE_STATUS ms = mobile->get_status();

    // Adding the imu object
    Eigen::Vector3d imu = mobile->get_imu();
    QJsonObject imuObj;
    imuObj["acc_x"] = QString::number(ms.imu_acc_x, 'f', 3);
    imuObj["acc_y"] = QString::number(ms.imu_acc_y, 'f', 3);
    imuObj["acc_z"] = QString::number(ms.imu_acc_z, 'f', 3);
    imuObj["gyr_x"] = QString::number(ms.imu_gyr_x*R2D, 'f', 3);
    imuObj["gyr_y"] = QString::number(ms.imu_gyr_y*R2D, 'f', 3);
    imuObj["gyr_z"] = QString::number(ms.imu_gyr_z*R2D, 'f', 3);
    imuObj["imu_rx"] = QString::number(imu[0]*R2D, 'f', 3);
    imuObj["imu_ry"] = QString::number(imu[1]*R2D, 'f', 3);
    imuObj["imu_rz"] = QString::number(imu[2]*R2D, 'f', 3);
    rootObj["imu"] = imuObj;

    // Adding the lidar array
    QJsonArray lidarArray;
    QJsonObject lidarObj1;
    lidarObj1["connection"] = lidar->is_connected_f ? "true" : "false";
    lidarObj1["port"] = "USB";
    lidarObj1["serialnumber"] = "not yet";
    lidarArray.append(lidarObj1);

    QJsonObject lidarObj2;
    lidarObj2["connection"] = lidar->is_connected_b ? "true" : "false";
    lidarObj2["port"] = "LAN";
    lidarObj2["serialnumber"] = "not yet";
    lidarArray.append(lidarObj2);
    rootObj["lidar"] = lidarArray;

    // Adding the motor array
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
    motorObj2["current"] = QString::number((double)ms.cur_m1/10.0, 'f', 3);
    motorArray.append(motorObj2);
    rootObj["motor"] = motorArray;

    // Adding the condition object
    Eigen::Vector2d ieir = slam->get_cur_ieir();
    QJsonObject conditionObj;
    conditionObj["inlier_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"] = QString::number(ieir[1], 'f', 3);
    conditionObj["mapping_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["mapping_ratio"] = QString::number(ieir[1], 'f', 3);
    rootObj["condition"] = conditionObj;

    // Adding the state object
    QString cur_loc_state = slam->get_cur_loc_state();
    QString charge_st_string = "none";

    #if defined(USE_D400) || defined(USE_D400_LAKI) || defined(USE_MECANUM)
    if(ms.charge_state == CHARGE_STATE_IDLE)
    {
        charge_st_string = "none";
    }
    else if(ms.charge_state == CHARGE_STATE_TRIG_TO_CHARGE)
    {
        charge_st_string = "ready";
    }
    else if(ms.charge_state == CHARGE_STATE_BATTERY_ON)
    {
        charge_st_string = "battery_on";
    }
    else if(ms.charge_state == CHARGE_STATE_CHARGING)
    {
        charge_st_string = "charging";
    }
    else if(ms.charge_state == CHARGE_STATE_TRIG_TO_STOP_CHARGE)
    {
        charge_st_string = "finish";
    }
    else if(ms.charge_state == CHARGE_STATE_FAIL)
    {
        charge_st_string = "fail";
    }
    #endif

    #if defined(USE_S100)
    if(ms.charge_state == 0)
    {
        charge_st_string = "none";
    }
    else if(ms.charge_state == 1)
    {
        charge_st_string = "charging";
    }
    #endif

    bool is_dock = false;

    QJsonObject robotStateObj;
    robotStateObj["charge"] = charge_st_string;
    robotStateObj["dock"] = (is_dock == true) ? "true" : "false";
    robotStateObj["emo"] = (ms.motor_stop_state == 1) ? "true" : "false";
    robotStateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    robotStateObj["power"] = (ms.power_state == 1) ? "true" : "false";
    rootObj["robot_state"] = robotStateObj;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"] = QString::number(ms.bat_in, 'f', 3);
    powerObj["bat_out"] = QString::number(ms.bat_out, 'f', 3);
    powerObj["bat_current"] = QString::number(ms.bat_current, 'f', 3);
    powerObj["total_power"] = QString::number(ms.total_power, 'f', 3);
    powerObj["power"] = QString::number(ms.power, 'f', 3);

    #if defined(USE_S100) || defined(USE_D400) || defined(USE_D400_LAKI)
    powerObj["bat_percent"] = QString::number(ms.bat_percent);
    #endif
    #if defined(USE_MECANUM)
    powerObj["bat_percent"] = QString::number(0);
    #endif

    #if defined(USE_D400) || defined(USE_D400_LAKI) || defined(USE_MECANUM)
    powerObj["charge_current"] = QString::number(ms.charge_current, 'f', 3);
    powerObj["contact_voltage"] = QString::number(ms.contact_voltage, 'f', 3);
    #endif
    #if defined(USE_S100)
    powerObj["charge_current"] = QString::number(0.0, 'f', 3);
    powerObj["contact_voltage"] = QString::number(0.0, 'f', 3);
    #endif
    rootObj["power"] = powerObj;

    QJsonObject settingObj;
    settingObj["platform_type"] = config->PLATFORM_TYPE;
    settingObj["platform_name"] = config->PLATFORM_NAME;
    rootObj["setting"] = settingObj;

    QJsonObject mapObj;
    QString map_name = "";
    if(unimap->is_loaded == MAP_LOADED)
    {
        QString map_dir = unimap->map_dir;
        QStringList map_dir_list = map_dir.split("/");
        if(map_dir_list.size() > 0)
        {
            map_name = unimap->map_dir.split("/").last();
        }
    }

    QString map_status = "";
    if(config->USE_LVX == 1)
    {
        int is_loaded = (int)lvx->is_loaded;
        if(is_loaded == MAP_NOT_LOADED)
        {
            map_status = "none";
        }
        else if(is_loaded == MAP_LOADING)
        {
            map_status = "loading";
        }
        else if(is_loaded == MAP_LOADED)
        {
            map_status = "loaded";
        }
    }
    else
    {
        int is_loaded = (int)unimap->is_loaded;
        if(is_loaded == MAP_NOT_LOADED)
        {
            map_status = "none";
        }
        else if(is_loaded == MAP_LOADING)
        {
            map_status = "loading";
        }
        else if(is_loaded == MAP_LOADED)
        {
            map_status = "loaded";
        }
    }

    mapObj["map_name"] = map_name;
    mapObj["map_status"] = map_status;
    rootObj["map"] = mapObj;

    // Adding the time object
    double time = get_time();
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        io->socket()->emit("status", res);
    }
}

// send functions
void COMM_RRS::send_move_status()
{
    if(!is_connected)
    {
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;


    QString cur_node_id0 = ctrl->get_cur_node_id();

    // Adding the move state object
    QString auto_state = "stop";
    if(ctrl->is_pause)
    {
        auto_state = "pause";
    }
    else if(ctrl->is_moving)
    {
        auto_state = "move";
    }

    if(mobile->get_cur_pdu_state() != "good")
    {
        auto_state = "not ready";        
    }

    if(slam->get_cur_loc_state() != "good")
    {
        auto_state = "error";
    }

    if(ctrl->get_obs_condition() == "vir")
    {
        auto_state = "vir";
    }

    if(cur_node_id0 == "")
    {
        auto_state = "error";
    }

    QString dock_state = "stop";
    if(dctrl->is_pause)
    {
        dock_state = "pause";
    }
    else if(dctrl->is_moving)
    {
        dock_state = "move";
    }

    QString jog_state = "none";

    QJsonObject moveStateObj;
    moveStateObj["auto_move"] = auto_state; // "stop", "move", "pause", "error", "not ready", "vir"
    moveStateObj["dock_move"] = dock_state;
    moveStateObj["jog_move"] = jog_state;
    moveStateObj["obs"] = ctrl->get_obs_condition();
    moveStateObj["path"] = ctrl->get_multi_req(); // "none", "req_path", "recv_path"
    rootObj["move_state"] = moveStateObj;

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

    // Adding the cur_node object
    QString cur_node_id = ctrl->get_cur_node_id();
    QString cur_node_name = "";
    if(unimap->is_loaded == MAP_LOADED && cur_node_id != "")
    {
        NODE* node = unimap->get_node_by_id(cur_node_id);
        if(node != NULL)
        {
            cur_node_name = node->name;
        }
    }

    QJsonObject curNodeObj;
    curNodeObj["id"] = cur_node_id;
    curNodeObj["name"] = cur_node_name;
    curNodeObj["state"] = "";
    curNodeObj["x"] = QString::number(cur_xi[0], 'f', 3);
    curNodeObj["y"] = QString::number(cur_xi[1], 'f', 3);
    curNodeObj["rz"] = QString::number(cur_xi[2]*R2D, 'f', 3);
    rootObj["cur_node"] = curNodeObj;

    // Adding the goal_node object
    QString goal_state = ctrl->get_cur_goal_state();
    QString goal_node_id = ctrl->move_info.goal_node_id;
    QString goal_node_name = "";
    Eigen::Vector3d goal_xi(0,0,0);
    if(unimap->is_loaded == MAP_LOADED && goal_node_id != "")
    {
        NODE* node = unimap->get_node_by_id(goal_node_id);
        if(node != NULL)
        {
            goal_node_name = node->name;
            goal_xi = TF_to_se2(node->tf);
        }
    }

    QJsonObject goalNodeObj;
    goalNodeObj["id"] = goal_node_id;
    goalNodeObj["name"] = goal_node_name;
    goalNodeObj["state"] = goal_state; // "", "move", "complete", "fail", "obstacle", "cancel"
    goalNodeObj["x"] = QString::number(goal_xi[0], 'f', 3);
    goalNodeObj["y"] = QString::number(goal_xi[1], 'f', 3);
    goalNodeObj["rz"] = QString::number(goal_xi[2]*R2D, 'f', 3);
    rootObj["goal_node"] = goalNodeObj;

    // Adding the time object
    double time = get_time();
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        io->socket()->emit("moveStatus", res);
    }
}

void COMM_RRS::send_local_path()
{
    if(!is_connected)
    {
        return;
    }

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
    io->socket()->emit("localPath", jsonArray);    
}

void COMM_RRS::send_global_path()
{
    if(!is_connected)
    {
        return;
    }

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
    io->socket()->emit("globalPath", jsonArray);
}

void COMM_RRS::send_lidar()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    TIME_POSE_PTS tpp = slam->get_cur_tpp();
    if(slam->is_loc && tpp.pts.size() > 0)
    {
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
        io->socket()->emit("lidarCloud", rootObject);
    }
    else
    {
        std::vector<Eigen::Vector3d> pts = lidar->get_cur_scan();

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
        io->socket()->emit("lidarCloud", rootObject);
    }
}

void COMM_RRS::send_mapping_cloud()
{
    if(!is_connected)
    {
        return;
    }

    if(slam->is_slam && last_send_kfrm_idx < (int)slam->kfrm_storage.size())
    {
        // send kfrm
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
        io->socket()->emit("mappingCloud", jsonArray);
        last_send_kfrm_idx++;
    }
}

void COMM_RRS::slot_move(DATA_MOVE msg)
{
    MOBILE_STATUS ms = mobile->get_status();
    int bat_percent = ms.bat_percent;

    QString command = msg.command;
    if(command == "jog")
    {
        // action
        double vx = msg.jog_val[0];
        double vy = msg.jog_val[1];
        double wz = msg.jog_val[2];

        MainWindow* _main = (MainWindow*)main;
        _main->update_jog_values(vx, vy, wz);
    }
    else if(command == "target")
    {
        QString method = msg.method;
        if(method == "pp")
        {
            // action
            if(unimap->is_loaded != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx1800] map not loaded";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(slam->is_loc == false)
            {
                msg.result = "reject";
                msg.message = "[R0Px1800] no localization";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            double x = msg.tgt_pose_vec[0];
            double y = msg.tgt_pose_vec[1];
            if(x < unimap->map_min_x || x > unimap->map_max_x || y < unimap->map_min_y || y > unimap->map_max_y)
            {
                msg.result = "reject";
                msg.message = "[R0Tx1800] target location out of range";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
            Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0],pose_vec[1],pose_vec[3]*D2R));
            goal_tf(2,3) = pose_vec[2];
            if(obsmap->is_tf_collision(goal_tf))
            {
                msg.result = "reject";
                msg.message = "[R0Tx1801] target location occupied(static obs)";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(config->USE_MULTI)
            {
                msg.result = "reject";
                msg.message = "[R0Tx1802] target command not supported by multi. use goal_id";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            msg.result = "accept";
            msg.message = "";
            msg.bat_percent = bat_percent;

            send_move_response(msg);

            // pure pursuit
            Q_EMIT ctrl->signal_move(msg);
        }
        else
        {
            msg.result = "reject";
            msg.message = "[R0Sx1800]not supported";

            send_move_response(msg);
        }
    }
    else if(command == "goal" || command == "change_goal")
    {
        QString method = msg.method;
        if(method == "pp")
        {
            if(unimap->is_loaded != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx2000]map not loaded";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(slam->is_loc == false)
            {
                msg.result = "reject";
                msg.message = "[R0Px2000]no localization";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            QString goal_id = msg.goal_node_id;
            if(goal_id == "")
            {
                msg.result = "reject";
                msg.message = "[R0Nx2000]empty node id";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            NODE* node = unimap->get_node_by_id(goal_id);
            if(node == NULL)
            {
                node = unimap->get_node_by_name(goal_id);
                if(node == NULL)
                {
                    msg.result = "reject";
                    msg.message = "[R0Nx2001]can not find node";
                    msg.bat_percent = bat_percent;

                    send_move_response(msg);
                    return;
                }

                // convert name to id
                msg.goal_node_id = node->id;
                msg.goal_node_name = node->name;                
            }
            else
            {
                msg.goal_node_name = node->name;
            }

            Eigen::Matrix4d cur_tf = slam->get_cur_tf();
            Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
            msg.cur_pos = cur_pos;

            Eigen::Vector3d xi = TF_to_se2(node->tf);
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];

            msg.bat_percent = bat_percent;

            // calc eta (estimation time arrival)
            Eigen::Matrix4d goal_tf = node->tf;
            PATH global_path = ctrl->calc_global_path(goal_tf);
            if(global_path.pos.size() < 2)
            {
                msg.result = "accept";
                msg.message = "just change goal";
                msg.eta = 0.0;
            }
            else
            {
                // time align (first align + final align)
                CTRL_PARAM params = ctrl->load_preset(msg.preset);
                Eigen::Matrix4d cur_tf = slam->get_cur_tf();
                auto dtdr_st = dTdR(cur_tf, global_path.pose.front());
                double time_align = (dtdr_st[1] / (params.LIMIT_W*D2R + 1e-06)) * 2;

                // time driving
                double time_driving = 0.0;
                double remaining_dist = 0.0;
                for(size_t p = 0; p < global_path.pos.size()-1; p++)
                {
                    Eigen::Vector3d pos0 = global_path.pos[p];
                    Eigen::Vector3d pos1 = global_path.pos[p+1];
                    double dist = (pos1 - pos0).norm();

                    double ref_v0 = global_path.ref_v[p];
                    double ref_v1 = global_path.ref_v[p+1];
                    double v = (ref_v0 + ref_v1)/2;

                    time_driving += dist/(v+1e-06);
                    remaining_dist += dist;
                }

                if(time_driving == 0.0)
                {
                    time_driving = 9999.0;
                }

                double total_time = time_driving + time_align;

                msg.result = "accept";
                msg.message = "";
                msg.eta = total_time;
                msg.remaining_dist = remaining_dist;
            }

            send_move_response(msg);

            // pure pursuit
            Q_EMIT ctrl->signal_move(msg);
        }
        else if(method == "hpp")
        {
            msg.result = "reject";
            msg.message = "not supported yet";
            msg.bat_percent = bat_percent;
            send_move_response(msg);
        }
        else if(method == "tng")
        {
            msg.result = "reject";
            msg.message = "not supported yet";
            msg.bat_percent = bat_percent;
            send_move_response(msg);
        }
        else
        {
            msg.result = "reject";
            msg.message = "[R0Sx2000]not supported";
            msg.bat_percent = bat_percent;
            send_move_response(msg);
        }
    }
    else if(command == "pause")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        ctrl->is_pause = true;
    }
    else if(command == "resume")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        ctrl->is_pause = false;
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        MainWindow* _main = (MainWindow*)main;
        _main->bt_Emergency();
    }
    else if(command == "escape")
    {
        QString escape_dir = msg.escape_dir;
        ctrl->move_escape(escape_dir);
    }
}

void COMM_RRS::slot_mapping(DATA_MAPPING msg)
{
    QString command = msg.command;
    if(command == "start")
    {
        if(lidar->is_connected_f)
        {
            msg.result = "accept";
            msg.message = "";

            send_mapping_response(msg);

            last_send_kfrm_idx = 0;
            MainWindow* _main = (MainWindow*)main;
            _main->bt_MapBuild();
        }
        else
        {
            msg.result = "reject";
            msg.message = "[R0Lx0800]lidar not connected";

            send_mapping_response(msg);
        }
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";

        send_mapping_response(msg);

        MainWindow* _main = (MainWindow*)main;
        _main->bt_MapSave();
    }
    else if(command == "save")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->bt_MapSave();

        QString map_name = msg.map_name;
        QString save_dir = QDir::homePath() + "/maps/" + map_name;

        std::string command = "cp -r " + _main->map_dir.toStdString() + " '" + save_dir.toStdString() + "'";
        int result = std::system(command.c_str());
        if(result == 0)
        {
            msg.result = "success";
            msg.message = "";

            send_mapping_response(msg);
        }
        else
        {
            msg.result = "fail";
            msg.message = "[R1Cx2100]copy failed, check auto created folder";

            send_mapping_response(msg);
        }
    }
    else if(command == "reload")
    {
        msg.result = "accept";
        msg.message = "";

        send_mapping_response(msg);

        last_send_kfrm_idx = 0;
    }
}

void COMM_RRS::slot_load(DATA_LOAD msg)
{
    QString command = msg.command;
    if(command == "mapload")
    {
        QString map_name = msg.map_name;
        QString load_dir = QDir::homePath() + "/maps/" + map_name;
        if(!load_dir.isNull())
        {
            if(!QDir(load_dir).exists())
            {
                msg.result = "reject";
                msg.message = "[R0Mx0201]invalid map dir";

                send_load_response(msg);
                return;
            }

            slam->localization_stop();
            obsmap->clear();

            config->set_map_path(load_dir);

            MainWindow* _main = (MainWindow*)main;
            _main->map_dir = load_dir;
            unimap->load_map(load_dir);
            _main->all_update();

            if(config->USE_LVX)
            {
                QString path_3d_map = load_dir + "/map.las";
                lvx->map_load(path_3d_map);
            }

            if(unimap->is_loaded == MAP_LOADED)
            {
                msg.result = "success";
                msg.message = "";

                send_load_response(msg);
            }
            else
            {
                msg.result = "fail";
                msg.message = "[R0Mx0200]map not load";

                send_load_response(msg);
            }
        }
    }
    else if(command == "topoload")
    {
        msg.result = "reject";
        msg.message = "[R0Sx0301]not support yet";

        send_load_response(msg);
    }
    else if(command == "configload")
    {
        msg.result = "reject";
        msg.message = "[R0Sx0401]not support yet";

        send_load_response(msg);
    }
}

void COMM_RRS::slot_randomseq(DATA_RANDOMSEQ msg)
{
    QString command = msg.command;
    if(command == "randomseq")
    {
        msg.result = "accept";
        msg.message = "";

        send_randomseq_response(msg);

        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_seq();
    }
}

void COMM_RRS::slot_localization(DATA_LOCALIZATION msg)
{
    QString command = msg.command;
    if(command == "semiautoinit")
    {
        if(unimap->is_loaded != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx0602]not loaded map";

            send_localization_response(msg);
            return;
        }

        if(lidar->is_connected_f == false)
        {
            msg.result = "reject";
            msg.message = "[R0Lx0601]not connected front lidar";

            send_localization_response(msg);
            return;
        }

        #if defined(USE_D400) || defined(USE_D400_LAKI)
        if(lidar->is_connected_b == false)
        {
            msg.result = "reject";
            msg.message = "[R0Lx0602]not connected back lidar";

            send_localization_response(msg);
            return;
        }
        #endif

        if(slam->is_busy)
        {
            msg.result = "reject";
            msg.message = "[R0Rx0600]already running";

            send_localization_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_localization_response(msg);

        // do process
        logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);

        if(config->USE_LVX)
        {
            lvx->loc_stop();
        }
        slam->localization_stop();

        // semi auto init
        if(semi_auto_init_thread != NULL)
        {
            logger->write_log("[AUTO_INIT] recv_loc, thread already running.", "Orange", true, false);
            semi_auto_init_thread->join();
            semi_auto_init_thread = NULL;
        }

        semi_auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start, slam);        
    }
    else if(command == "init")
    {
        if(unimap->is_loaded != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx0702]not loaded map";

            send_localization_response(msg);
            return;
        }
        if(lidar->is_connected_f == false)
        {
            msg.result = "reject";
            msg.message = "[R0Lx0701]not connected front lidar";

            send_localization_response(msg);
            return;
        }

        #if defined(USE_D400) || defined(USE_D400_LAKI)
        if(lidar->is_connected_b == false)
        {
            msg.result = "reject";
            msg.message = "[R0Lx0702]not connected back lidar";

            send_localization_response(msg);
            return;
        }
        #endif

        msg.result = "accept";
        msg.message = "";
        send_localization_response(msg);

        // manual init
        double x = msg.tgt_pose_vec[0];
        double y = msg.tgt_pose_vec[1];
        double test = msg.tgt_pose_vec[2];
        double rz = msg.tgt_pose_vec[3];

        logger->write_log(QString("[COMM_RRS] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz*D2R).arg(msg.time), "Green");

        Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz*D2R));

        if(config->USE_LVX)
        {
            lvx->loc_stop();
        }
        slam->localization_stop();

        if(config->USE_LVX)
        {
            lvx->set_cur_tf(tf);
        }
        slam->set_cur_tf(tf);

        /*if(config->USE_LVX)
        {
            lvx->loc_start();
        }
        slam->localization_start();*/
    }
    else if(command == "start")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(config->USE_LVX)
        {
            lvx->loc_start();
        }
        slam->localization_start();
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(config->USE_LVX)
        {
            lvx->loc_stop();
        }
        slam->localization_stop();
    }
    else if(command == "randominit")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        QString seed = msg.seed;

        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_init(seed);
    }
}

void COMM_RRS::slot_dock(DATA_DOCK msg)
{
    QString command = msg.command;
    if(command == "dock")
    {
        msg.result = "accept";
        msg.message = "";
      
        MainWindow* _main = (MainWindow*)main;
        _main->bt_DockStart();
      
        send_dock_response(msg);
    }
    else if(command == "undock")
    {
        msg.result = "accept";
        msg.message = "";

        MainWindow* _main = (MainWindow*)main;
        _main->bt_UnDockStart();
      
        send_dock_response(msg);
    }
}

void COMM_RRS::slot_view_lidar(DATA_VIEW_LIDAR msg)
{
    QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = (MainWindow*)main;
            _main->lidar_view_frequency = msg.frequency;
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->lidar_view_frequency = -1;
    }
}

void COMM_RRS::slot_view_path(DATA_VIEW_PATH msg)
{
    QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = (MainWindow*)main;
            _main->path_view_frequency = msg.frequency;
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->path_view_frequency = -1;
    }
}

void COMM_RRS::slot_led(DATA_LED msg)
{
    QString command = msg.command;
    if(command == "on")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->is_user_led = true;

        QString led = msg.led;
        if(led == "none")
        {
            _main->user_led_color = LED_OFF;
        }
        else if(led == "red")
        {
            _main->user_led_color = LED_RED;
        }
        else if(led == "blue")
        {
            _main->user_led_color = LED_BLUE;
        }
        else if(led == "white")
        {
            _main->user_led_color = LED_WHITE;
        }
        else if(led == "green")
        {
            _main->user_led_color = LED_GREEN;
        }
        else if(led == "magenta")
        {
            _main->user_led_color = LED_MAGENTA;
        }
        else if(led == "yellow")
        {
            _main->user_led_color = LED_YELLOW;
        }
        else if(led == "red blink")
        {
            _main->user_led_color = LED_RED_BLINK;
        }
        else if(led == "blue blink")
        {
            _main->user_led_color = LED_BLUE_BLINK;
        }
        else if(led == "white blink")
        {
            _main->user_led_color = LED_WHITE_BLINK;
        }
        else if(led == "green blink")
        {
            _main->user_led_color = LED_GREEN_BLINK;
        }
        else if(led == "magenta blink")
        {
            _main->user_led_color = LED_MAGENTA_BLINK;
        }
        else if(led == "yellow blink")
        {
            _main->user_led_color = LED_YELLOW_BLINK;
        }

        msg.result = "accept";
        msg.message = "";
    }
    else if(command == "off")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->is_user_led = false;

        msg.result = "accept";
        msg.message = "";
    }
}

void COMM_RRS::slot_motor(DATA_MOTOR msg)
{
    QString command = msg.command;
    if(command == "on")
    {
        mobile->motor_on();

        msg.result = "accept";
        msg.message = "";
    }
    else if(command == "off")
    {
        mobile->motor_off();

        msg.result = "accept";
        msg.message = "";
    }
}

void COMM_RRS::slot_path(DATA_PATH msg)
{
    QString command = msg.command;
    if(command == "path")
    {
        QString path_str = msg.path;
        QStringList path_str_list = path_str.split(",");

        std::vector<QString> path;
        for(int p = 0; p < path_str_list.size(); p++)
        {
            path.push_back(path_str_list[p]);
        }

        QString _vobs_robots = msg.vobs_robots;
        QString _vobs_closures = msg.vobs_closures;

        int preset = msg.preset;
        ctrl->set_path(path, preset, _vobs_robots, _vobs_closures);

        msg.response = "true";

        msg.vobs_closures = "";
        msg.vobs_robots = "";
        msg.path = "";

        send_path_response(msg);
    }
    else if(command == "move")
    {
        std::pair<QString, QString> vobs_str = ctrl->get_vobs();
        QString vobs_r_str = vobs_str.first;
        QString vobs_c_str = vobs_str.second;

        std::vector<Eigen::Vector3d> vobs_r_list;
        {
            QStringList vobs_str_list = vobs_r_str.split("\n");
            if(vobs_str_list.size() > 0)
            {
                for(int p = 0; p < vobs_str_list.size(); p++)
                {
                    QStringList vobs_str_list2 = vobs_str_list[p].split(",");
                    if(vobs_str_list2.size() == 3)
                    {
                        Eigen::Vector3d P;
                        P[0] = vobs_str_list2[0].toDouble();
                        P[1] = vobs_str_list2[1].toDouble();
                        P[2] = vobs_str_list2[2].toDouble();
                        vobs_r_list.push_back(P);
                    }
                }
            }
        }

        std::vector<Eigen::Vector3d> vobs_c_list;
        {
            QStringList vobs_str_list = vobs_c_str.split(",");
            if(vobs_str_list.size() > 0)
            {
                for(int p = 0; p < vobs_str_list.size(); p++)
                {
                    QString node_id = vobs_str_list[p];
                    if(node_id != "")
                    {
                        NODE *node = unimap->get_node_by_id(node_id);
                        if(node != NULL)
                        {
                            vobs_c_list.push_back(node->tf.block(0,3,3,1));
                        }
                    }
                }
            }
        }

        // update vobs
        {
            obsmap->mtx.lock();
            obsmap->vobs_list_robots = vobs_r_list;
            obsmap->vobs_list_closures = vobs_c_list;
            obsmap->mtx.unlock();

            obsmap->update_vobs_map();
        }

        ctrl->move_path();
    }
}

void COMM_RRS::slot_vobs_r(DATA_VOBS_R msg)
{
    QString command = msg.command;
    if(command == "vobs_robots")
    {
        std::vector<Eigen::Vector3d> vobs_list;

        QString vobs_str = msg.vobs;
        QStringList vobs_str_list = vobs_str.split("\n");
        if(vobs_str_list.size() > 0)
        {
            for(int p = 0; p < vobs_str_list.size(); p++)
            {
                QStringList vobs_str_list2 = vobs_str_list[p].split(",");
                if(vobs_str_list2.size() == 3)
                {
                    Eigen::Vector3d P;
                    P[0] = vobs_str_list2[0].toDouble();
                    P[1] = vobs_str_list2[1].toDouble();
                    P[2] = vobs_str_list2[2].toDouble();
                    vobs_list.push_back(P);
                }
            }
        }

        // update vobs
        {
            obsmap->mtx.lock();
            obsmap->vobs_list_robots = vobs_list;
            obsmap->mtx.unlock();

            obsmap->update_vobs_map();
        }
    }
}

void COMM_RRS::slot_vobs(DATA_VOBS msg)
{
    QString command = msg.command;
    if(command == "vobs")
    {
        std::vector<Eigen::Vector3d> vobs_r_list;
        {
            QString vobs_str = msg.vobs_robots;
            QStringList vobs_str_list = vobs_str.split("\n");
            if(vobs_str_list.size() > 0)
            {
                for(int p = 0; p < vobs_str_list.size(); p++)
                {
                    QStringList vobs_str_list2 = vobs_str_list[p].split(",");
                    if(vobs_str_list2.size() == 3)
                    {
                        Eigen::Vector3d P;
                        P[0] = vobs_str_list2[0].toDouble();
                        P[1] = vobs_str_list2[1].toDouble();
                        P[2] = vobs_str_list2[2].toDouble();
                        vobs_r_list.push_back(P);
                    }
                }
            }
        }

        std::vector<Eigen::Vector3d> vobs_c_list;
        {
            QString vobs_str = msg.vobs_clousers;
            QStringList vobs_str_list = vobs_str.split(",");

            // set vobs
            for(int p = 0; p < vobs_str_list.size(); p++)
            {
                QString node_id = vobs_str_list[p];
                if(node_id != "")
                {
                    NODE *node = unimap->get_node_by_id(node_id);
                    if(node != NULL)
                    {
                        vobs_c_list.push_back(node->tf.block(0,3,3,1));
                    }
                }
            }
        }

        // update vobs
        {
            obsmap->mtx.lock();
            obsmap->vobs_list_robots = vobs_r_list;
            if(msg.is_vobs_closures_change == "true")
            {
                obsmap->vobs_list_closures = vobs_c_list;
            }
            obsmap->mtx.unlock();

            obsmap->update_vobs_map();
        }
    }
}

void COMM_RRS::slot_vobs_c(DATA_VOBS_C msg)
{
    QString command = msg.command;
    if(command == "vobs_closures")
    {
        QString vobs_str = msg.vobs;
        QStringList vobs_str_list = vobs_str.split(",");

        // set vobs
        std::vector<Eigen::Vector3d> vobs_list;
        for(int p = 0; p < vobs_str_list.size(); p++)
        {
            QString node_id = vobs_str_list[p];
            if(node_id != "")
            {
                NODE *node = unimap->get_node_by_id(node_id);
                if(node != NULL)
                {
                    vobs_list.push_back(node->tf.block(0,3,3,1));
                }
            }
        }

        // update vobs
        {
            obsmap->mtx.lock();
            obsmap->vobs_list_closures = vobs_list;
            obsmap->mtx.unlock();

            obsmap->update_vobs_map();
        }
    }
}

void COMM_RRS::send_move_response(DATA_MOVE msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["preset"] = QString::number(msg.preset, 10);
    obj["method"] = msg.method;
    obj["goal_id"] = msg.goal_node_id;
    obj["remaining_dist"] = QString::number(msg.remaining_dist, 'f', 3);
    obj["eta"] = QString::number(msg.eta, 'f', 3);
    obj["bat_percent"] = QString::number(msg.bat_percent, 'f', 3);

    // temporal patch
    QString response_goal_node_name = msg.goal_node_name;
    if(msg.goal_node_name.contains("AMR-WAITING-01"))
    {
        response_goal_node_name = "AMR-WAITING-01";
    }
    else if(msg.goal_node_name.contains("AMR-CHARGING-01"))
    {
        response_goal_node_name = "AMR-CHARGING-01";
    }
    else if(msg.goal_node_name.contains("AMR-PACKING-01"))
    {
        response_goal_node_name = "AMR-PACKING-01";
    }
    else if(msg.goal_node_name.contains("AMR-CONTAINER-01"))
    {
        response_goal_node_name = "AMR-CONTAINER-01";
    }
    obj["goal_name"] = response_goal_node_name;
    obj["cur_x"] = QString::number(msg.cur_pos[0], 'f', 3);
    obj["cur_y"] = QString::number(msg.cur_pos[1], 'f', 3);
    obj["cur_z"] = QString::number(msg.cur_pos[2], 'f', 3);
    obj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
    obj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
    obj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
    obj["rz"] = QString::number(msg.tgt_pose_vec[3]*R2D, 'f', 3);
    obj["vx"] = QString::number(msg.jog_val[0], 'f', 3);
    obj["vy"] = QString::number(msg.jog_val[1], 'f', 3);
    obj["wz"] = QString::number(msg.jog_val[2], 'f', 3);
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("moveResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_localization_response(DATA_LOCALIZATION msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
    obj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
    obj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
    obj["rz"] = QString::number(msg.tgt_pose_vec[3], 'f', 3);
    obj["seed"] = msg.seed;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("localizationResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_load_response(DATA_LOAD msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["name"] = msg.map_name;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("loadResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_randomseq_response(DATA_RANDOMSEQ msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("randomseqResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_mapping_response(DATA_MAPPING msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["name"] = msg.map_name;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mappingResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_dock_response(DATA_DOCK msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("dockResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_path_response(DATA_PATH msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;

    QString test = QString::number((long long)(msg.time*1000), 10);
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());
    printf("test %s\n", test.toLocal8Bit().data());

    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("pathResponse", res);
}

