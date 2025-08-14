#include "comm_rrs.h"

#include "mainwindow.h"

COMM_RRS* COMM_RRS::instance(QObject* parent)
{
    static COMM_RRS* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COMM_RRS(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

COMM_RRS::COMM_RRS(QObject *parent) : QObject(parent)
  , main(parent)
  , config(nullptr)
  , logger(nullptr)
  , mobile(nullptr)
  , unimap(nullptr)
  , obsmap(nullptr)
  , lidar_2d(nullptr)
  , loc(nullptr)
  , mapping(nullptr)
  , dctrl(nullptr)
{
    // set recv callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    io = std::make_unique<sio::client>();
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
    BIND_EVENT(sock, "swUpdate",        std::bind(&COMM_RRS::recv_software_update,   this, _1, _2, _3, _4));
    BIND_EVENT(sock, "footStatus",      std::bind(&COMM_RRS::recv_foot,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "fieldRequest",   std::bind(&COMM_RRS::recv_field_set,         this, _1, _2, _3, _4));

    // connect recv signals -> recv slots
    connect(this, &COMM_RRS::signal_move,            this, &COMM_RRS::slot_move);
    connect(this, &COMM_RRS::signal_localization,    this, &COMM_RRS::slot_localization);
    connect(this, &COMM_RRS::signal_load,            this, &COMM_RRS::slot_load);
    connect(this, &COMM_RRS::signal_randomseq,       this, &COMM_RRS::slot_randomseq);
    connect(this, &COMM_RRS::signal_mapping,         this, &COMM_RRS::slot_mapping);
    connect(this, &COMM_RRS::signal_dock,            this, &COMM_RRS::slot_dock);
    connect(this, &COMM_RRS::signal_field_set,       this, &COMM_RRS::slot_field_set);
    connect(this, &COMM_RRS::signal_view_lidar,      this, &COMM_RRS::slot_view_lidar);
    connect(this, &COMM_RRS::signal_view_path,       this, &COMM_RRS::slot_view_path);
    connect(this, &COMM_RRS::signal_led,             this, &COMM_RRS::slot_led);
    connect(this, &COMM_RRS::signal_motor,           this, &COMM_RRS::slot_motor);
    connect(this, &COMM_RRS::signal_path,            this, &COMM_RRS::slot_path);
    connect(this, &COMM_RRS::signal_vobs,            this, &COMM_RRS::slot_vobs);
    connect(this, &COMM_RRS::signal_software_update, this, &COMM_RRS::slot_software_update);
    connect(this, &COMM_RRS::signal_foot,            this, &COMM_RRS::slot_foot);

    send_timer = new QTimer(this);
    connect(send_timer, SIGNAL(timeout()), this, SLOT(send_loop()));
    send_timer->start(100);
}

COMM_RRS::~COMM_RRS()
{
    if(io)
    {
        io->socket()->off_all();
        io->socket()->off_error();
        io->sync_close();
    }

    if(semi_auto_init_thread && semi_auto_init_thread->joinable())
    {
        semi_auto_init_thread->join();
    }
    semi_auto_init_thread.reset();
}

QString COMM_RRS::get_json(sio::message::ptr const& data, const QString& key)
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

    sio::message::ptr val = it->second;

    if(val->get_flag() == sio::message::flag_string)
    {
        return QString::fromStdString(it->second->get_string());
    }

    if(val->get_flag() == sio::message::flag_integer)
    {
        return QString::number(val->get_int());
    }

    if(val->get_flag() == sio::message::flag_double)
    {
        return QString::number(val->get_double());
    }

    if(val->get_flag() == sio::message::flag_boolean)
    {
        return val->get_bool() ? "true" : "false";
    }

    return "";
}

QString COMM_RRS::get_multi_state()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return multi_state;
}

QByteArray COMM_RRS::get_last_msg()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return lastest_msg_str;
}

void COMM_RRS::init()
{
    if(!config)
    {
        printf("[COMM_RRS] Warning: config module not set\n");
        return;
    }

    if(config->get_use_rrs())
    {
        std::map<std::string, std::string> query;
        query["name"] = "slamnav";

        io->connect("ws://localhost:11337", query);
    }
}

void COMM_RRS::sio_connected()
{
    is_connected = true;
    if(ctrl)
    {
        ctrl->set_is_rrs(true);
    }
    if(logger)
    {
        logger->write_log("[COMM_RRS] connected", "Green");
    }
}

void COMM_RRS::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;
    if(ctrl)
    {
        ctrl->set_is_rrs(false);
    }
    if(logger)
    {
        logger->write_log("[COMM_RRS] disconnected", "Green");
    }
}

void COMM_RRS::sio_error()
{
    if(logger)
    {
        logger->write_log("[COMM_RRS] some error", "Red");
    }
}

// recv parser -> emit recv signals
void COMM_RRS::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
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
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble() * D2R;
        msg.jog_val[0] = get_json(data, "vx").toDouble();
        msg.jog_val[1] = get_json(data, "vy").toDouble();
        msg.jog_val[2] = get_json(data, "wz").toDouble() * D2R;
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_move(msg);
    }
}

void COMM_RRS::recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOCALIZATION msg;
        msg.command = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"
        msg.seed = get_json(data, "seed");
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble();
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_localization(msg);
    }
}

void COMM_RRS::recv_load(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOAD msg;
        msg.command = get_json(data, "command"); // "mapload", "topoload", "configload"
        msg.map_name = get_json(data, "name");
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_load(msg);
    }
}

void COMM_RRS::recv_randomseq(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_RANDOMSEQ msg;
        msg.command = get_json(data, "command"); // "randomseq"
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_randomseq(msg);
    }
}

void COMM_RRS::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MAPPING msg;
        msg.command = get_json(data, "command"); // "start", "stop", "save", "name", "reload"
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_mapping(msg);
    }
}

void COMM_RRS::recv_dock(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_DOCK msg;
        msg.command = get_json(data, "command"); // "dock", "undock"
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_dock(msg);
    }
}

void COMM_RRS::recv_field_set(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_FIELD msg;
        msg.command = get_json(data, "command"); // "set", "get"
        msg.set_field = get_json(data, "set_field").toInt();


        qDebug() << "recv_field_set_field:" << msg.set_field;
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_field_set(msg);
    }
}


void COMM_RRS::recv_view_lidar_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_LIDAR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, lidar_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_view_lidar(msg);
    }
}

void COMM_RRS::recv_view_path_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_PATH msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, path_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_view_path(msg);
    }
}

void COMM_RRS::recv_led(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LED msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.led = get_json(data, "led");
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, led_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_led(msg);
    }
}

void COMM_RRS::recv_motor(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MOTOR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, motor_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        Q_EMIT signal_motor(msg);
    }
}

void COMM_RRS::recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_PATH msg;
        msg.command = get_json(data, "command"); // "path"
        msg.path = get_json(data, "path");
        msg.vobs_robots = get_json(data, "vobs_r");
        msg.vobs_closures = get_json(data, "vobs_c");
        msg.preset = get_json(data, "preset").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        //logger->write_log(QString("[COMM_RRS] recv, command: %1, path: %2, time: %3").arg(msg.command).arg(msg.path). arg(msg.time), "Green");
        Q_EMIT signal_path(msg);
    }
}

void COMM_RRS::recv_vobs(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VOBS msg;
        msg.command = get_json(data, "command"); // "vobs"
        msg.vobs_robots = get_json(data, "vobs_r");
        msg.vobs_clousers = get_json(data, "vobs_c");
        msg.is_vobs_closures_change = get_json(data, "is_vobs_c");
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        QString res = QString("[COMM_RRS] recv, command: %1, vobs_r: %2, vobs_c: %3, time: %4").arg(msg.command).arg(msg.vobs_robots).arg(msg.vobs_clousers).arg(msg.time);
        printf("%s\n", res.toLocal8Bit().constData());

        //logger->write_log(QString("[COMM_RRS] recv, command: %1, vobs: %2, time: %3").arg(msg.command).arg(msg.vobs).arg(msg.time), "Green");
        Q_EMIT signal_vobs(msg);
    }
}

void COMM_RRS::recv_software_update(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_SOFTWARE msg;
        msg.version = get_json(data, "version");
        msg.time = get_json(data, "time").toDouble() / 1000;

        Q_EMIT signal_software_update(msg);
    }
}

void COMM_RRS::recv_foot(const std::string& name, const sio::message::ptr& data, bool hasAck, sio::message::list& ack_resp)
{
    // printf("[COMM_RRS][DEBUG] recv_foot called\n");

    if(data && data->get_flag() == sio::message::flag_object)
    {
        QString time_str = get_json(data, "time");
        double time_sec = get_json(data, "time").toDouble() / 1000;

        // printf("[COMM_RRS][DEBUG] time: %s\n", time_str.toStdString().c_str());

        // get foot
        sio::message::ptr foot_msg = data->get_map()["foot"];
        if(!foot_msg || foot_msg->get_flag() != sio::message::flag_object)
        {
            // printf("[COMM_RRS][DEBUG] 'foot' field missing or not object\n");
            return;
        }

        DATA_FOOT msg;
        msg.connection = get_json(foot_msg, "connection") == "true";
        msg.position   = get_json(foot_msg, "position").toInt();
        msg.is_down    = get_json(foot_msg, "is_down") == "true";
        msg.state      = get_json(foot_msg, "foot_state").toInt();
        msg.time       = time_sec;

        // get temperature_sensor
        sio::message::ptr temperature_sensor = data->get_map()["temperature_sensor"];
        if(!temperature_sensor || temperature_sensor->get_flag() != sio::message::flag_object)
        {
            // printf("[COMM_RRS][DEBUG] 'temperature_sensor' field missing or not object\n");
            return;
        }

        DATA_TEMPERATURE temperature_msg;
        temperature_msg.connection = get_json(temperature_sensor, "connection") == "true";
        temperature_msg.temperature_value   = get_json(temperature_sensor, "temperature_value").toFloat();
        temperature_msg.time       = time_sec;

//        qDebug()<<QString::number(temperature_msg.temperature_value);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        _main->temperature_value = temperature_msg.temperature_value;

//        qDebug()<<"temprature_msg.temperature_value : "<<temprature_msg.temperature_value;

        // debug
        // printf("[COMM_RRS][DEBUG] recv foot → conn: %d, pos: %d, down: %d, state: %d, time: %.3f\n",
        //        msg.connection, msg.position, msg.is_down, msg.state, msg.time);

        if(logger)
        {
            // logger->write_log(
            //             QString("[COMM_RRS] recv footState - conn: %1, pos: %2, down: %3, state: %4, time: %5")
            //             .arg(msg.connection)
            //             .arg(msg.position)
            //             .arg(msg.is_down)
            //             .arg(msg.state)
            //             .arg(msg.time),
            //             "Green"
            //             );
        }

        Q_EMIT signal_foot(msg);
    }
}

// send functions
void COMM_RRS::send_status()
{
    if(!is_connected || !config || !mobile || !unimap || !dctrl)
    {
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;
    MOBILE_STATUS ms = mobile->get_status();

    // Adding the imu object
    Eigen::Vector3d imu = mobile->get_imu();
    QJsonObject imuObj;
    imuObj["acc_x"]  = QString::number(ms.imu_acc_x, 'f', 3);
    imuObj["acc_y"]  = QString::number(ms.imu_acc_y, 'f', 3);
    imuObj["acc_z"]  = QString::number(ms.imu_acc_z, 'f', 3);
    imuObj["gyr_x"]  = QString::number(ms.imu_gyr_x * R2D, 'f', 3);
    imuObj["gyr_y"]  = QString::number(ms.imu_gyr_y * R2D, 'f', 3);
    imuObj["gyr_z"]  = QString::number(ms.imu_gyr_z * R2D, 'f', 3);
    imuObj["imu_rx"] = QString::number(imu[0] * R2D, 'f', 3);
    imuObj["imu_ry"] = QString::number(imu[1] * R2D, 'f', 3);
    imuObj["imu_rz"] = QString::number(imu[2] * R2D, 'f', 3);
    rootObj["imu"] = imuObj;

    // Adding the motor array
    QJsonArray motorArray;
    QJsonObject motorObj1;
    motorObj1["connection"] = (ms.connection_m0 == 1) ? "true" : "false";
    motorObj1["status"]     = QString::number(ms.status_m0);
    motorObj1["temp"]       = QString::number(ms.temp_m0, 'f', 3);
    motorObj1["current"]    = QString::number(static_cast<double>(ms.cur_m0) / 10.0, 'f', 3);
    motorArray.append(motorObj1);

    QJsonObject motorObj2;
    motorObj2["connection"] = (ms.connection_m1 == 1) ? "true" : "false";
    motorObj2["status"]     = QString::number(ms.status_m1);
    motorObj2["temp"]       = QString::number(ms.temp_m1, 'f', 3);
    motorObj2["current"]    = QString::number(static_cast<double>(ms.cur_m1) / 10.0, 'f', 3);
    motorArray.append(motorObj2);
    rootObj["motor"] = motorArray;

    // Adding the condition object
    Eigen::Vector2d ieir = loc->get_cur_ieir();
    QJsonObject conditionObj;
    conditionObj["inlier_error"]  = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"]  = QString::number(ieir[1], 'f', 3);
    conditionObj["mapping_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["mapping_ratio"] = QString::number(ieir[1], 'f', 3);
    rootObj["condition"] = conditionObj;

    // Adding the state object
    QString cur_loc_state = loc->get_cur_loc_state();
    QString charge_st_string = "none";

    QString platform_type = config->get_platform_type();
    if(platform_type == "D400" || platform_type == "MECANUM")
    {
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
    }
    else if(platform_type == "S100")
    {
        if(ms.charge_state == 0)
        {
            charge_st_string = "none";
        }
        else if(ms.charge_state == 1)
        {
            charge_st_string = "charging";
        }
    }
    bool is_dock = dctrl->get_dock_state();

    QJsonObject robotStateObj;
    robotStateObj["charge"]       = charge_st_string;
    robotStateObj["dock"]         = (is_dock == true) ? "true" : "false";
    robotStateObj["emo"]          = (ms.motor_stop_state == 1) ? "true" : "false";
    robotStateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    robotStateObj["power"]        = (ms.power_state == 1) ? "true" : "false";
    rootObj["robot_state"]        = robotStateObj;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"]      = QString::number(ms.bat_in, 'f', 3);
    powerObj["bat_out"]     = QString::number(ms.bat_out, 'f', 3);
    powerObj["bat_current"] = QString::number(ms.bat_current, 'f', 3);
    powerObj["total_power"] = QString::number(ms.total_power, 'f', 3);
    powerObj["power"] = QString::number(ms.power, 'f', 3);
    powerObj["bat_percent"] = QString::number(ms.bat_percent);
    powerObj["tabos_voltage"]   = QString::number(ms.tabos_voltage, 'f', 3);
    powerObj["tabos_current"]   = QString::number(ms.tabos_current, 'f', 3);
    powerObj["tabos_status"]    = QString::number(ms.tabos_status);
    powerObj["tabos_ttf"]       = QString::number(ms.tabos_ttf);
    powerObj["tabos_tte"]       = QString::number(ms.tabos_tte);
    powerObj["tabos_soc"]       = QString::number(ms.tabos_soc);
    powerObj["tabos_soh"]       = QString::number(ms.tabos_soh);
    powerObj["tabos_temp"]      = QString::number(ms.tabos_temperature, 'f', 3);
    powerObj["tabos_rc"]        = QString::number(ms.tabos_rc, 'f', 3);
    powerObj["tabos_ae"]        = QString::number(ms.tabos_ae, 'f' ,3);

    if(platform_type == "D400" || platform_type == "MECANUM")
    {
        powerObj["charge_current"] = QString::number(ms.charge_current, 'f', 3);
        powerObj["contact_voltage"] = QString::number(ms.contact_voltage, 'f', 3);
    }
    else if(platform_type == "S100")
    {
        powerObj["charge_current"] = QString::number(0.0, 'f', 3);
        powerObj["contact_voltage"] = QString::number(0.0, 'f', 3);
    }
    rootObj["power"] = powerObj;

    QJsonObject settingObj;
    settingObj["platform_type"] = platform_type;
    settingObj["platform_name"] = config->get_platform_name();
    rootObj["setting"] = settingObj;

    QJsonObject mapObj;
    QString map_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED)
    {
        QString map_dir = unimap->get_map_path();
        QStringList map_dir_list = map_dir.split("/");
        if(map_dir_list.size() > 0)
        {
            map_name = unimap->get_map_path().split("/").last();
        }
    }

    QString map_status = "";
    int is_loaded = static_cast<int>(unimap->get_is_loaded());
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

    mapObj["map_name"]   = map_name;
    mapObj["map_status"] = map_status;
    rootObj["map"] = mapObj;

    // usb temp sensor
    QJsonObject tempObj;
    tempObj["connection"] = QString::number(ms.bat_in, 'f', 3);
    tempObj["temp_sensor"] = QString::number(ms.bat_out, 'f', 3);

    // Adding the time object
    const double time = get_time();
    rootObj["time"] = QString::number(static_cast<long long>(time * 1000));

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        // std::cout << doc.toJson().toStdString() << std::endl;
        io->socket()->emit("status", res);
    }
}

// send functions
void COMM_RRS::send_move_status()
{
    if(!is_connected || !ctrl || !mobile || !unimap || !dctrl)
    {
        printf("is_connected : %d\n", (int)is_connected.load());
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;

    QString cur_node_id = loc->get_cur_node_id();

    // Adding the move state object
    QString auto_state = "stop";
    if(ctrl->get_is_pause())
    {
        auto_state = "pause";
    }
    else if(ctrl->get_is_moving())
    {
        auto_state = "move";
    }

    if(mobile->get_cur_pdu_state() != "good" || ctrl->get_multi_inter_lock())
    {
        auto_state = "not ready";
    }

    if(loc->get_cur_loc_state() != "good")
    {
        auto_state = "error";
    }

    if(ctrl->get_obs_condition() == "vir")
    {
        auto_state = "vir";
    }

    if(cur_node_id.isEmpty())
    {
        auto_state = "error";
    }

    QString dock_state = "stop";

    QString jog_state = "none";

    QJsonObject moveStateObj;
    moveStateObj["auto_move"] = auto_state; // "stop", "move", "pause", "error", "not ready", "vir"
    moveStateObj["dock_move"] = dock_state;
    moveStateObj["jog_move"] = jog_state;
    moveStateObj["obs"] = ctrl->get_obs_condition();
    moveStateObj["path"] = ctrl->get_multi_reqest_state(); // "none", "req_path", "recv_path"
    rootObj["move_state"] = moveStateObj;

    // Adding the pose object
    const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    const Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    QJsonObject poseObj;
    poseObj["x"] = QString::number(cur_xi[0], 'f', 3);
    poseObj["y"] = QString::number(cur_xi[1], 'f', 3);
    poseObj["rz"] = QString::number(cur_xi[2] * R2D, 'f', 3);
    rootObj["pose"] = poseObj;

    // Adding the velocity object
    MOBILE_POSE mo = mobile->get_pose();
    QJsonObject velObj;
    velObj["vx"] = QString::number(mo.vel[0], 'f', 3);
    velObj["vy"] = QString::number(mo.vel[1], 'f', 3);
    velObj["wz"] = QString::number(mo.vel[2] * R2D, 'f', 3);
    rootObj["vel"] = velObj;

    // Adding the cur_node object
    QString cur_node_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
    {
        NODE* node = unimap->get_node_by_id(cur_node_id);
        if(node != nullptr)
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
    curNodeObj["rz"] = QString::number(cur_xi[2] * R2D, 'f', 3);
    rootObj["cur_node"] = curNodeObj;

    // Adding the goal_node object
    QString goal_state = ctrl->get_cur_move_state();
    QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;
    QString goal_node_name = "";
    Eigen::Vector3d goal_xi(0, 0, 0);
    if(unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
    {
        NODE* node = unimap->get_node_by_id(goal_node_id);
        if(node != nullptr)
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
    goalNodeObj["rz"] = QString::number(goal_xi[2] * R2D, 'f', 3);
    rootObj["goal_node"] = goalNodeObj;

    // Adding the time object
    const double time = get_time();
    rootObj["time"] = QString::number(static_cast<long long>(time * 1000));

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        io->socket()->emit("moveStatus", res);
    }
}

void COMM_RRS::send_local_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }

    PATH path = ctrl->get_cur_local_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(size_t p = 0; p < path.pos.size(); p++)
    {
        if(p == 0 || p == path.pos.size() - 1 || p % 10 == 0)
        {
            const Eigen::Vector3d P = path.pos[p];

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
    if(!is_connected || !ctrl)
    {
        return;
    }

    PATH path = ctrl->get_cur_global_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(const Eigen::Vector3d& P : path.pos)
    {
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
    if(!is_connected || !loc || !lidar_2d || !lidar_3d)
    {
        return;
    }

    std::vector<Eigen::Vector3d> pts = lidar_2d->get_cur_frm().pts;
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    if(pts.size() > 0)
    {
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
    if(!is_connected || !mapping || !config)
    {
        return;
    }

    if(mapping->get_is_mapping() && last_send_kfrm_idx < static_cast<int>(mapping->get_kfrm_storage_size()))
    {
        // send kfrm
        KFRAME kfrm = mapping->get_kfrm(last_send_kfrm_idx);
        const int accum_num = config->get_mapping_icp_do_accum_num();

        Eigen::Matrix3d R0 = kfrm.opt_G.block(0, 0, 3, 3);
        Eigen::Vector3d t0 = kfrm.opt_G.block(0, 3, 3, 1);

        sio::array_message::ptr jsonArray = sio::array_message::create();
        for(const auto& pt : kfrm.pts)
        {
            if(pt.do_cnt < accum_num)
            {
                continue;
            }

            Eigen::Vector3d P(pt.x, pt.y, pt.z);
            Eigen::Vector3d _P = R0 * P + t0;

            sio::array_message::ptr jsonObj = sio::array_message::create();

            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[2], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pt.r, 'f', 3).toStdString()));

            jsonArray->get_vector().push_back(jsonObj);
        }

        // send
        io->socket()->emit("mappingCloud", jsonArray);
        last_send_kfrm_idx++;
    }
}

void COMM_RRS::slot_move(DATA_MOVE msg)
{
    if(!mobile || !unimap || !obsmap || !ctrl || !loc)
    {
        return;
    }

    MOBILE_STATUS ms = mobile->get_status();
    int bat_percent = ms.bat_percent;

    const QString command = msg.command;
    if(command == "jog")
    {
        // action
        const double vx = msg.jog_val[0];
        const double vy = msg.jog_val[1];
        const double wz = msg.jog_val[2];

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->update_jog_values(vx, vy, wz);
        }
    }
    else if(command == "target")
    {
        const QString method = msg.method;
        if(method == "pp")
        {
            // action
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx1800] map not loaded";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(!loc->get_is_loc())
            {
                msg.result = "reject";
                msg.message = "[R0Px1800] no localization";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            const double x = msg.tgt_pose_vec[0];
            const double y = msg.tgt_pose_vec[1];
            if(x < unimap->get_map_min_x() || x > unimap->get_map_max_x() || y < unimap->get_map_min_y() || y > unimap->get_map_max_y())
            {
                msg.result = "reject";
                msg.message = "[R0Tx1800] target location out of range";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            const Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
            const Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[3] * D2R));
            if(obsmap->is_tf_collision(goal_tf))
            {
                msg.result = "reject";
                msg.message = "[R0Tx1801] target location occupied(static obs)";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(config->get_use_multi())
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
        const QString method = msg.method;
        if(method == "pp")
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx2000]map not loaded";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(!loc->get_is_loc())
            {
                msg.result = "reject";
                msg.message = "[R0Px2000]no localization";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            if(mobile->get_is_inter_lock_foot())
            {
                msg.result = "reject";
                msg.message = "inter lock foot";

                send_move_response(msg);
                return;
            }

            QString goal_id = msg.goal_node_id;
            if(goal_id.isEmpty())
            {
                msg.result = "reject";
                msg.message = "[R0Nx2000]empty node id";
                msg.bat_percent = bat_percent;

                send_move_response(msg);
                return;
            }

            NODE* node = unimap->get_node_by_id(goal_id);
            if(!node)
            {
                node = unimap->get_node_by_name(goal_id);
                if(!node)
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

            const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
            const Eigen::Vector3d cur_pos = cur_tf.block(0, 3, 3, 1);
            msg.cur_pos = cur_pos;

            const Eigen::Vector3d xi = TF_to_se2(node->tf);
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2, 3);
            msg.tgt_pose_vec[3] = xi[2];

            msg.bat_percent = bat_percent;

            // calc eta (estimation time arrival)
            const Eigen::Matrix4d goal_tf = node->tf;
            PATH global_path = ctrl->calc_global_path(goal_tf);
            if(global_path.pos.size() < 2)
            {
                msg.result = "accept";
                msg.message = "success";
                msg.eta = 0.0;
            }
            else
            {
                msg.result = "accept";
                msg.message = "success";
            }

            send_move_response(msg);

            // pure pursuit
            Q_EMIT (ctrl->signal_move(msg));
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

        ctrl->set_is_pause(true);
    }
    else if(command == "resume")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        ctrl->set_is_pause(false);
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_Emergency();
        }
    }
}

void COMM_RRS::slot_mapping(DATA_MAPPING msg)
{
    const QString command = msg.command;
    if(command == "start")
    {
        if(lidar_2d && lidar_2d->get_is_connected())
        {
            msg.result = "accept";
            msg.message = "";

            send_mapping_response(msg);

            last_send_kfrm_idx = 0;
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                _main->bt_MapBuild();
            }
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

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_MapSave();
        }
    }
    else if(command == "save")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_MapSave();

            const QString map_name = msg.map_name;
            const QString save_dir = "/data/maps/" + map_name;

            const std::string command = "cp -r " + _main->get_map_path().toStdString() + " '" + save_dir.toStdString() + "'";
            const int result = std::system(command.c_str());
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
    const QString command = msg.command;
    if(command == "mapload")
    {
        const QString map_name = msg.map_name;
        //const QString load_dir = QDir::homePath() + "/data/maps/" + map_name;
        const QString load_dir = "/data/maps/" + map_name;

        //printf(Qload_dir);
        qDebug()<<load_dir;
        if(!load_dir.isNull())
        {
            if(!QDir(load_dir).exists())
            {
                msg.result = "reject";
                msg.message = "[R0Mx0201]invalid map dir";

                send_load_response(msg);
                return;
            }

            if(loc)
            {
                loc->stop();
            }
            if(obsmap)
            {
                obsmap->clear();
            }

            if(config)
            {
                config->set_map_path(load_dir);
            }

            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                _main->set_map_path(load_dir);
                if(unimap)
                {
                    unimap->load_map(load_dir);
                }
                _main->all_update();
            }

            if(unimap && unimap->get_is_loaded() == MAP_LOADED)
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
    const QString command = msg.command;
    if(command == "randomseq")
    {
        msg.result = "accept";
        msg.message = "";

        send_randomseq_response(msg);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->slot_sim_random_seq();
        }
    }
}

void COMM_RRS::slot_localization(DATA_LOCALIZATION msg)
{
    const QString command = msg.command;
    if(command == "semiautoinit")
    {
        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx0602]not loaded map";

            send_localization_response(msg);
            return;
        }

        if(!lidar_2d || !lidar_2d->get_is_connected())
        {
            msg.result = "reject";
            msg.message = "[R0Lx0601]not connected lidar";

            send_localization_response(msg);
            return;
        }

        if(!loc || loc->get_is_busy())
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
        if(logger)
        {
            logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);
        }

        if(loc)
        {
            loc->stop();
        }

        // semi auto init
        if(semi_auto_init_thread)
        {
            if(logger)
            {
                logger->write_log("[AUTO_INIT] recv_loc, thread already running.", "Orange", true, false);
            }
            if(semi_auto_init_thread->joinable())
            {
                semi_auto_init_thread->join();
            }
            semi_auto_init_thread.reset();
        }

        if(loc)
        {
            semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
        }
    }
    else if(command == "init")
    {
        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx0702]not loaded map";

            send_localization_response(msg);
            return;
        }
        if(!lidar_2d || !lidar_2d->get_is_connected())
        {
            msg.result = "reject";
            msg.message = "[R0Lx0701]not connected lidar";

            send_localization_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_localization_response(msg);

        // manual init
        const double x = msg.tgt_pose_vec[0];
        const double y = msg.tgt_pose_vec[1];
        const double test = msg.tgt_pose_vec[2];
        const double rz = msg.tgt_pose_vec[3];

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz * D2R).arg(msg.time), "Green");
        }

        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));
        if(loc)
        {
            loc->stop();
            loc->set_cur_tf(tf);
        }
    }
    else if(command == "start")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(loc->get_is_loc())
        {
            loc->stop();
        }

        const double x = msg.tgt_pose_vec[0];
        const double y = msg.tgt_pose_vec[1];
        const double rz = msg.tgt_pose_vec[3];
        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));

        if(loc)
        {
            loc->set_cur_tf(tf);
            loc->start();
        }
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(loc)
        {
            loc->stop();
        }
    }
    else if(command == "randominit")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        const QString seed = msg.seed;

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->slot_sim_random_init(seed);
        }
    }
}

void COMM_RRS::slot_field_set(DATA_FIELD msg)
{

    const QString command = msg.command;

    if(command == "set")
    {
        msg.result = "success";
        qDebug() << "slot msg.sef_field:" << msg.set_field;
        unsigned int set_field_ = msg.set_field;
        msg.message = "";

        qDebug() << "parsing filed set:" << set_field_;

        if(mobile)
        {
            MOBILE::instance()->setlidarfield(set_field_);
        }

        send_field_set_response(msg);
    }

    else if (command == "get")
    {
        msg.result = "success";
        msg.message = "";

        if(mobile)
        {
            MOBILE_STATUS ms = MOBILE::instance()->get_status();
            qDebug() << ms.lidar_field;
            msg.get_field = ms.lidar_field;
        }

        send_field_get_response(msg);

    }
}

void COMM_RRS::slot_dock(DATA_DOCK msg)
{
    const QString command = msg.command;
    if(command == "dock")
    {
        msg.result = "accept";
        msg.message = "";

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_DockStart();
        }

        send_dock_response(msg);
    }
    else if(command == "undock")
    {
        msg.result = "accept";
        msg.message = "";

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_UnDockStart();
        }

        send_dock_response(msg);
    }
}

void COMM_RRS::slot_view_lidar(DATA_VIEW_LIDAR msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                //_main->lidar_view_frequency = msg.frequency;
            }
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->lidar_view_frequency = -1;
        }
    }
}

void COMM_RRS::slot_view_path(DATA_VIEW_PATH msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                //_main->path_view_frequency = msg.frequency;
            }
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->path_view_frequency = -1;
        }
    }
}

void COMM_RRS::slot_led(DATA_LED msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->is_user_led = true;

            const QString led = msg.led;
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
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->is_user_led = false;

            msg.result = "accept";
            msg.message = "";
        }
    }
}

void COMM_RRS::slot_motor(DATA_MOTOR msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(mobile)
        {
            mobile->motor_on();
        }

        msg.result = "accept";
        msg.message = "";
    }
    else if(command == "off")
    {
        if(mobile)
        {
            mobile->motor_off();
        }

        msg.result = "accept";
        msg.message = "";
    }
}

void COMM_RRS::slot_path(DATA_PATH msg)
{
    const QString command = msg.command;
    if(command == "path")
    {
        if(ctrl)
        {
            ctrl->signal_path(msg);
        }
    }
}

void COMM_RRS::slot_vobs(DATA_VOBS msg)
{
    const QString command = msg.command;
    if(command == "vobs" && obsmap && unimap)
    {
        std::vector<Eigen::Vector3d> vobs_r_list;
        {
            const QString vobs_str = msg.vobs_robots;
            const QStringList vobs_str_list = vobs_str.split("\n", Qt::SkipEmptyParts);
            vobs_r_list.reserve(vobs_str_list.size());
            for(const QString& vobs_line : vobs_str_list)
            {
                const QStringList vobs_str_list2 = vobs_line.split(",", Qt::SkipEmptyParts);
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

        std::vector<Eigen::Vector3d> vobs_c_list;
        {
            const QString vobs_str = msg.vobs_clousers;
            const QStringList vobs_str_list = vobs_str.split(",", Qt::SkipEmptyParts);

            // set vobs
            vobs_c_list.reserve(vobs_str_list.size());
            for(const QString& node_id : vobs_str_list)
            {
                if(!node_id.isEmpty())
                {
                    NODE* node = unimap->get_node_by_id(node_id);
                    if(node)
                    {
                        vobs_c_list.push_back(node->tf.block(0, 3, 3, 1));
                    }
                }
            }
        }

        // update vobs
        {
            obsmap->update_vobs_list_robots(vobs_r_list);
            if(msg.is_vobs_closures_change == "true")
            {
                obsmap->update_vobs_list_closures(vobs_c_list);
            }
        }

        obsmap->update_vobs_map();
    }
}

void COMM_RRS::slot_software_update(DATA_SOFTWARE msg)
{
    if(!is_connected || !mobile)
    {
        return;
    }

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.charge_state == 0 || ms.motor_stop_state >= 1)
    {
        //bool is_not_charging = ms.charge_state != 1 ? true : false;
        if(ms.charge_state == 0)
        {
            msg.result = "false";
            msg.message = "not charging";
            send_software_update_response(msg);
        }

        if(ms.motor_stop_state >= 1)
        {
            msg.result = "false";
            msg.message = "emo released";
            send_software_update_response(msg);
        }
        return;
    }

    const QString version_str = "--version=" + msg.version;
    QString homePath = QDir::homePath();
    QString scriptPath = homePath + "/rainbow-deploy-kit/slamnav2/slamnav2-update.sh";

    QProcess process;
    QStringList arguments;
    arguments << version_str;

    process.start("bash", QStringList() << scriptPath << arguments);

    if(!process.waitForStarted())
    {
        msg.result = "false";
        msg.message = "process failed to start";
        send_software_update_response(msg);
        return;
    }

    if(!process.waitForFinished())
    {
        msg.result = "false";
        msg.message = "process did not finish";
        send_software_update_response(msg);
        return;
    }

    const int exitCode = process.exitCode();
    const QProcess::ExitStatus exitStatus = process.exitStatus();

    if(exitStatus == QProcess::NormalExit && exitCode == 0)
    {
        msg.result = "true";
        msg.message = "update succeeded";
    }
    else
    {
        msg.result = "false";

        QString stdOut = process.readAllStandardOutput();
        QString stdErr = process.readAllStandardError();

        msg.message = QString("exitCode:%1\nstdout:\n%2\nstderr:\n%3")
                .arg(exitCode)
                .arg(QString(stdOut))
                .arg(QString(stdErr));
    }

    send_software_update_response(msg);
}


// for interlock
void COMM_RRS::slot_foot(DATA_FOOT msg)
{
    if(msg.state == FOOT_STATE_DONE && msg.is_down == true)
    {
        // logger->write_log(QString("[MOBILE] slot_foot state:%1, set inter lock true").arg(msg.state), "Green");
        mobile->set_is_inter_lock_foot(true);
    }
    else
    {
        // logger->write_log(QString("[MOBILE] slot_foot state:%1, set inter lock false").arg(msg.state), "Green");
        mobile->set_is_inter_lock_foot(false);
    }
}

void COMM_RRS::send_move_response(const DATA_MOVE& msg)
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
    obj["rz"] = QString::number(msg.tgt_pose_vec[3] * R2D, 'f', 3);
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

void COMM_RRS::send_localization_response(const DATA_LOCALIZATION& msg)
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

void COMM_RRS::send_load_response(const DATA_LOAD& msg)
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

void COMM_RRS::send_randomseq_response(const DATA_RANDOMSEQ& msg)
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

void COMM_RRS::send_mapping_response(const DATA_MAPPING& msg)
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

void COMM_RRS::send_dock_response(const DATA_DOCK& msg)
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

void COMM_RRS::send_field_set_response(const DATA_FIELD& msg)
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
    io->socket()->emit("fieldResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_field_get_response(const DATA_FIELD& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["get_field"] = QString::number(msg.get_field);
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("fieldResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_path_response(const DATA_PATH& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("pathResponse", res);
}

void COMM_RRS::send_software_update_response(const DATA_SOFTWARE& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["version"] = msg.version;
    obj["applyReqUpdate"] = msg.result;
    obj["rejectReason"] = msg.message;

    printf("[SEND_SW_RESPONSE] version:%s, applyReqUpdate:%s, rejectReason:%s\n",
           msg.version.toStdString().c_str(),
           msg.result.toStdString().c_str(),
           msg.message.toStdString().c_str());

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("swUpdateResponse", res);
}

void COMM_RRS::set_config_module(CONFIG* _config)
{
    if(_config)
    {
        config = _config;
    }
}

void COMM_RRS::set_logger_module(LOGGER* _logger)
{
    if(_logger)
    {
        logger = _logger;
    }
}

void COMM_RRS::set_mobile_module(MOBILE* _mobile)
{
    if(_mobile)
    {
        mobile = _mobile;
    }
}

void COMM_RRS::set_lidar_2d_module(LIDAR_2D* _lidar)
{
    if(_lidar)
    {
        lidar_2d = _lidar;
    }
}

void COMM_RRS::set_cam_module(CAM* _cam)
{
    if(_cam)
    {
        cam = _cam;
    }
}

void COMM_RRS::set_localization_module(LOCALIZATION* _loc)
{
    if(_loc)
    {
        loc = _loc;
    }
}

void COMM_RRS::set_mapping_module(MAPPING* _mapping)
{
    if(_mapping)
    {
        mapping = _mapping;
    }
}

void COMM_RRS::set_unimap_module(UNIMAP* _unimap)
{
    if(_unimap)
    {
        unimap = _unimap;
    }
}

void COMM_RRS::set_obsmap_module(OBSMAP* _obsmap)
{
    if(_obsmap)
    {
        obsmap = _obsmap;
    }
}

void COMM_RRS::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(_ctrl)
    {
        ctrl = _ctrl;
    }
}

void COMM_RRS::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if(_dctrl)
    {
        dctrl = _dctrl;
    }
}

void COMM_RRS::send_loop()
{
    if(!is_connected)
    {
        return;
    }

    if(send_cnt % 2 == 0)
    {
        send_move_status();
    }

    if(send_cnt % 5 == 0)
    {
        send_status();
    }

    if(send_cnt > 10000)
    {
        send_cnt = 0;
    }

    send_cnt ++;
}
