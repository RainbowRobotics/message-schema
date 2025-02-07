#include "comm_ms.h"

#include "mainwindow.h"

COMM_MS::COMM_MS(QObject *parent)
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
    io->set_open_listener(std::bind(&COMM_MS::sio_connected, this));
    io->set_close_listener(std::bind(&COMM_MS::sio_disconnected, this, _1));
    io->set_fail_listener(std::bind(&COMM_MS::sio_error, this));
    io->set_reconnect_attempts(-1);

    // bind recv callback function this func emit signal
    BIND_EVENT(sock, "move",            std::bind(&COMM_MS::recv_move,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "localization",    std::bind(&COMM_MS::recv_localization,      this, _1, _2, _3, _4));
    BIND_EVENT(sock, "load",            std::bind(&COMM_MS::recv_load,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "randomseq",       std::bind(&COMM_MS::recv_randomseq,         this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapping",         std::bind(&COMM_MS::recv_mapping,           this, _1, _2, _3, _4));
    BIND_EVENT(sock, "dock",            std::bind(&COMM_MS::recv_dock,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "lidarOnOff",      std::bind(&COMM_MS::recv_view_lidar_on_off, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "pathOnOff",       std::bind(&COMM_MS::recv_view_path_on_off,  this, _1, _2, _3, _4));
    BIND_EVENT(sock, "led",             std::bind(&COMM_MS::recv_led,               this, _1, _2, _3, _4));
    BIND_EVENT(sock, "motor",           std::bind(&COMM_MS::recv_motor,             this, _1, _2, _3, _4));
    BIND_EVENT(sock, "path",            std::bind(&COMM_MS::recv_path,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobsRobots",      std::bind(&COMM_MS::recv_vobs_robots,       this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobsClosures",    std::bind(&COMM_MS::recv_vobs_closures,     this, _1, _2, _3, _4));

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
    connect(this, SIGNAL(signal_vobs_r(DATA_VOBS_R)),             this, SLOT(slot_vobs_r(DATA_VOBS_R)));
    connect(this, SIGNAL(signal_vobs_c(DATA_VOBS_C)),             this, SLOT(slot_vobs_c(DATA_VOBS_C)));
}

COMM_MS::~COMM_MS()
{
    io->socket()->off_all();
    io->socket()->off_error();
    io->close();
}

QString COMM_MS::get_json(sio::message::ptr const& data, QString key)
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

QString COMM_MS::get_multi_state()
{
    mtx.lock();
    QString res = multi_state;
    mtx.unlock();

    return res;
}

void COMM_MS::init()
{
    if(config->USE_WEB_UI)
    {
        std::map<std::string, std::string> query;
        query["name"] = "slamnav";

        io->connect("ws://localhost:11337", query);
    }
}

void COMM_MS::sio_connected()
{
    is_connected = true;

    ctrl->is_multi = true;

    QString str;
    str.sprintf("[COMM_MS] connected");
    logger->write_log(str, "Green");
}

void COMM_MS::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;

    ctrl->is_multi = false;

    QString str;
    str.sprintf("[COMM_MS] disconnected");
    logger->write_log(str, "Green");
}

void COMM_MS::sio_error()
{
    QString str;
    str.sprintf("[COMM_MS] some error");
    logger->write_log(str, "Red");
}

// recv parser -> emit recv signals
void COMM_MS::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_MOVE dmove;
        dmove.command = get_json(data, "command");
        if(dmove.command == "goal")
        {
            dmove.method = get_json(data, "method");
            dmove.goal_node_id = get_json(data, "goal_id");
            dmove.preset = get_json(data, "preset").toInt();
            dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dmove.jog_val = Eigen::Vector3d(0,0,0);
            dmove.time = time;

            // for response
            MOVE_INFO _last_move_info;
            _last_move_info.command = dmove.command;
            _last_move_info.node_id = dmove.goal_node_id;
            _last_move_info.preset = dmove.preset;
            _last_move_info.method = dmove.method;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();
        }
        else if(dmove.command == "jog")
        {
            double vx = get_json(data, "vx").toDouble();
            double vy = get_json(data, "vy").toDouble();
            double wz = get_json(data, "wz").toDouble();

            dmove.method = "";
            dmove.goal_node_id = "";
            dmove.preset = 0;
            dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dmove.jog_val = Eigen::Vector3d(vx,vy,wz);
            dmove.time = time;
        }
        else if(dmove.command == "target")
        {
            double x = get_json(data, "x").toDouble();
            double y = get_json(data, "y").toDouble();
            double z = get_json(data, "z").toDouble();
            double rz = get_json(data, "rz").toDouble();

            dmove.method = get_json(data, "method");
            dmove.goal_node_id = "";
            dmove.preset = get_json(data, "preset").toInt();
            dmove.tgt_pose_vec = Eigen::Vector4d(x,y,z,rz);
            dmove.jog_val = Eigen::Vector3d(0,0,0);
            dmove.time = time;

            // for response
            MOVE_INFO _last_move_info;
            _last_move_info.command = dmove.command;
            _last_move_info.x = dmove.tgt_pose_vec[0];
            _last_move_info.y = dmove.tgt_pose_vec[1];
            _last_move_info.z = dmove.tgt_pose_vec[2];
            _last_move_info.rz = dmove.tgt_pose_vec[3];
            _last_move_info.preset = dmove.preset;
            _last_move_info.method = dmove.method;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();
        }
        else if(dmove.command == "pause")
        {
            dmove.method = "";
            dmove.goal_node_id = "";
            dmove.preset = 0;
            dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dmove.jog_val = Eigen::Vector3d(0,0,0);
            dmove.time = time;
        }
        else if(dmove.command == "resume")
        {
            dmove.method = "";
            dmove.goal_node_id = "";
            dmove.preset = 0;
            dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dmove.jog_val = Eigen::Vector3d(0,0,0);
            dmove.time = time;
        }
        else if(dmove.command == "stop")
        {
            dmove.method = "";
            dmove.goal_node_id = "";
            dmove.preset = 0;
            dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dmove.jog_val = Eigen::Vector3d(0,0,0);
            dmove.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_move, not support command:%s, time:%.3f", dmove.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");

            dmove.result = "reject";
            dmove.message = str;
        }

        Q_EMIT signal_move(dmove);

        QString str;
        str.sprintf("[COMM_MS] success recv_move, command:%s, time: %.3f", dmove.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_LOCALIZATION dloc;
        dloc.command = get_json(data, "command");
        if(dloc.command == "autoinit")
        {
            dloc.seed = "";
            dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dloc.time = time;
        }
        else if(dloc.command == "semiautoinit")
        {
            dloc.seed = "";
            dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dloc.time = time;
        }
        else if(dloc.command == "init")
        {
            double x = get_json(data, "x").toDouble();
            double y = get_json(data, "y").toDouble();
            double z = get_json(data, "z").toDouble();
            double rz = get_json(data, "rz").toDouble();

            dloc.seed = "";
            dloc.tgt_pose_vec = Eigen::Vector4d(x,y,z,rz);
            dloc.time = time;
        }
        else if(dloc.command == "start")
        {
            dloc.seed = "";
            dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dloc.time = time;
        }
        else if(dloc.command == "stop")
        {
            dloc.seed = "";
            dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dloc.time = time;
        }
        else if(dloc.command == "randominit")
        {
            QString str0;
            str0.sprintf("dloc.start");
            logger->write_log(str0, "Green");

            QString str;
            dloc.seed = get_json(data, "seed");
            str.sprintf("dloc.seed:%s", dloc.seed.toLocal8Bit().data());
            logger->write_log(str, "Green");

            dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
            dloc.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_localization, command:%s, time:%.3f", dloc.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        Q_EMIT signal_localization(dloc);

        QString str;
        str.sprintf("[COMM_MS] success recv_localization, command:%s, time: %.3f", dloc.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_load(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_LOAD dload;
        dload.command = get_json(data, "command");
        if(dload.command == "mapload")
        {
            dload.map_name = get_json(data, "name");
            dload.time = time;
        }
        else if(dload.command == "topoload")
        {
            dload.map_name = get_json(data, "name");
            dload.time = time;
        }
        else if(dload.command == "configload")
        {
            dload.map_name = "";
            dload.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_load, not support command:%s, time:%.3f", dload.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        Q_EMIT signal_load(dload);

        QString str;
        str.sprintf("[COMM_MS] success recv_load, command:%s, time: %.3f", dload.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_randomseq(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_RANDOMSEQ drandomseq;
        drandomseq.command = get_json(data, "command");
        if(drandomseq.command == "randomseq")
        {
            drandomseq.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_randomseq, command:%s, time:%.3f", drandomseq.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        Q_EMIT signal_randomseq(drandomseq);

        QString str;
        str.sprintf("[COMM_MS] success recv_randomseq, command:%s, time: %.3f", drandomseq.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_MAPPING dmap;
        dmap.command = get_json(data, "command");
        if(dmap.command == "start")
        {
            dmap.map_name = "";
            dmap.time = time;
        }
        else if(dmap.command == "stop")
        {
            dmap.map_name = "";
            dmap.time = time;
        }
        else if(dmap.command == "save")
        {
            dmap.map_name = get_json(data, "name");
            dmap.time = time;
        }
        else if(dmap.command == "reload")
        {
            dmap.map_name = "";
            dmap.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_mapping, command:%s, time:%.3f", dmap.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        Q_EMIT signal_mapping(dmap);

        QString str;
        str.sprintf("[COMM_MS] success recv_mapping, command:%s, time: %.3f", dmap.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_dock(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_DOCK ddock;
        ddock.command = get_json(data, "command");
        if(ddock.command == "dock")
        {
            ddock.time = time;
        }
        else if(ddock.command == "undock")
        {
            ddock.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_dock, command:%s, time:%.3f", ddock.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_dock(ddock);

        QString str;
        str.sprintf("[COMM_MS] success recv_dock, command:%s, time: %.3f", ddock.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_view_lidar_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_VIEW_LIDAR dlidar;
        dlidar.command = get_json(data, "command");
        if(dlidar.command == "on")
        {
            dlidar.frequency = get_json(data, "frequency").toInt();
            dlidar.time = time;
        }
        else if(dlidar.command == "off")
        {
            dlidar.command = "off";
            dlidar.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_view_lidar_on_off, command:%s, time:%.3f", dlidar.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_view_lidar(dlidar);

        QString str;
        str.sprintf("[COMM_MS] success recv_view_lidar_on_off, command:%s, time: %.3f", dlidar.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_view_path_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_VIEW_PATH dpath;
        dpath.command = get_json(data, "command");
        if(dpath.command == "on")
        {
            bool is_conversion;
            int frequency = get_json(data, "frequency").toInt(&is_conversion);
            if(is_conversion)
            {
                dpath.frequency = frequency;
                dpath.time = time;
            }
            else
            {
                dpath.frequency = -1;
                dpath.time = time;
            }
        }
        else if(dpath.command == "off")
        {
            dpath.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_view_path_on_off, command:%s, time:%.3f", dpath.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_view_path(dpath);

        QString str;
        str.sprintf("[COMM_MS] success recv_path_on_off, command:%s, time: %.3f", dpath.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_led(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_LED dled;
        dled.command = get_json(data, "command");
        if(dled.command == "on")
        {
            dled.led = get_json(data, "led");
            dled.time = time;
        }
        else if(dled.command == "off")
        {
            dled.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_led, command:%s, time:%.3f", dled.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_led(dled);

        QString str;
        str.sprintf("[COMM_MS] success recv_led, command:%s, time: %.3f", dled.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_motor(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_MOTOR dmotor;
        dmotor.command = get_json(data, "command");
        if(dmotor.command == "on")
        {
            dmotor.time = time;
        }
        else if(dmotor.command == "off")
        {
            dmotor.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_motor, command:%s, time:%.3f", dmotor.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_motor(dmotor);

        QString str;
        str.sprintf("[COMM_MS] success recv_motor, command:%s, time: %.3f", dmotor.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_PATH dpath;
        dpath.command = get_json(data, "command");
        if(dpath.command == "path")
        {
            dpath.path = get_json(data, "path");
            dpath.preset = get_json(data, "preset").toInt();
            dpath.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_path, command:%s, time:%.3f", dpath.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_path(dpath);

        QString str;
        str.sprintf("[COMM_MS] success recv_path, command:%s, time: %.3f", dpath.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_vobs_robots(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_VOBS_R dvobs_r;
        dvobs_r.command = get_json(data, "command");
        if(dvobs_r.command == "vobs_robots")
        {
            dvobs_r.vobs = get_json(data, "vobs");
            dvobs_r.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_vobs_r, command:%s, time:%.3f", dvobs_r.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_vobs_r(dvobs_r);

        QString str;
        str.sprintf("[COMM_MS] success recv_vobs_r, command:%s, time: %.3f", dvobs_r.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

void COMM_MS::recv_vobs_closures(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data->get_flag() == sio::message::flag_object)
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        DATA_VOBS_C dvobs_c;
        dvobs_c.command = get_json(data, "command");
        if(dvobs_c.command == "vobs_closures")
        {
            dvobs_c.vobs = get_json(data, "vobs");
            dvobs_c.time = time;
        }
        else
        {
            QString str;
            str.sprintf("[COMM_MS] fail recv_vobs_c, command:%s, time:%.3f", dvobs_c.command.toLocal8Bit().data(), time);
            logger->write_log(str, "Red");
            return;
        }

        // action
        Q_EMIT signal_vobs_c(dvobs_c);

        QString str;
        str.sprintf("[COMM_MS] success recv_vobs_c, command:%s, time: %.3f", dvobs_c.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Green");
    }
}

// send functions
void COMM_MS::send_status()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time0();

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
    QString charge_st_string = "";
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

    bool is_dock = false;

    QJsonObject robotStateObj;
    robotStateObj["charge"] = charge_st_string;
    robotStateObj["dock"] = (is_dock == true) ? "true" : "false";
    robotStateObj["emo"] = (ms.emo_state == 1) ? "true" : "false";
    robotStateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    robotStateObj["power"] = (ms.power_state == 1) ? "true" : "false";
    rootObj["robot_state"] = robotStateObj;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"] = QString::number(ms.bat_in, 'f', 3);
    powerObj["bat_out"] = QString::number(ms.bat_out, 'f', 3);
    powerObj["bat_current"] = QString::number(ms.bat_current, 'f', 3);
    powerObj["bat_percent"] = QString::number(ms.bat_percent);
    powerObj["power"] = QString::number(ms.power, 'f', 3);
    powerObj["total_power"] = QString::number(ms.total_power, 'f', 3);
    powerObj["charge_current"] = QString::number(ms.charge_current, 'f', 3);
    powerObj["contact_voltage"] = QString::number(ms.contact_voltage, 'f', 3);
    rootObj["power"] = powerObj;

    QJsonObject settingObj;
    settingObj["platform_type"] = config->PLATFORM_TYPE;
    rootObj["setting"] = settingObj;

    QJsonObject mapObj;
    QString map_name = "";
    if(unimap->is_loaded)
    {
        QString map_dir = unimap->map_dir;
        QStringList map_dir_list = map_dir.split("/");
        if(map_dir_list.size() > 0)
        {
            map_name = unimap->map_dir.split("/").last();
        }
    }
    mapObj["map_name"] = map_name;
    rootObj["map"] = mapObj;

    // Adding the time object
    rootObj["time"] = time;

    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("status", res);
    last_send_status_time = time;
    //printf("[COMM_MS] send_status, time: %f\n", time);
}

// send functions
void COMM_MS::send_move_status()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time0();

    // Creating the JSON object
    QJsonObject rootObj;

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
    moveStateObj["auto_move"] = auto_state;
    moveStateObj["dock_move"] = dock_state;
    moveStateObj["jog_move"] = jog_state;
    moveStateObj["obs"] = ctrl->get_obs_condition();
    moveStateObj["path"] = ctrl->get_multi_req(); // none, req_path, recv_path
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
    QString cur_node_id = "";
    QString cur_node_name = "";
    if(unimap->is_loaded)
    {
        cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
        if(cur_node_id != "")
        {
            NODE* node = unimap->get_node_by_id(cur_node_id);
            if(node != NULL)
            {
                cur_node_name = node->name;
            }
        }
    }

    QJsonObject curNodeObj;
    curNodeObj["id"] = cur_node_id;
    curNodeObj["name"] = cur_node_name;
    curNodeObj["state"] = "";
    curNodeObj["x"] = cur_xi[0];
    curNodeObj["y"] = cur_xi[1];
    curNodeObj["rz"] = cur_xi[2]*R2D;
    rootObj["cur_node"] = curNodeObj;

    // Adding the goal_node object
    QString goal_node_id = "";
    QString goal_node_name = "";
    Eigen::Matrix4d goal_tf = ctrl->get_cur_goal_tf();
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);
    if(unimap->is_loaded)
    {
        goal_node_id = unimap->get_node_id_edge(goal_tf.block(0,3,3,1));
        if(goal_node_id != "")
        {
            NODE* node = unimap->get_node_by_id(goal_node_id);
            if(node != NULL)
            {
                goal_node_name = node->name;
            }
        }

        if(goal_tf.isIdentity())
        {
            goal_tf = cur_tf;
            goal_node_id = cur_node_id;
        }
    }

    QJsonObject goalNodeObj;
    goalNodeObj["id"] = goal_node_id;
    goalNodeObj["name"] = goal_node_name;
    goalNodeObj["state"] = ctrl->get_cur_goal_state();
    goalNodeObj["x"] = goal_xi[0];
    goalNodeObj["y"] = goal_xi[1];
    goalNodeObj["rz"] = goal_xi[2]*R2D;
    rootObj["goal_node"] = goalNodeObj;

    // Adding the time object
    rootObj["time"] = time;

    QJsonDocument doc(rootObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("moveStatus", res);
    last_send_move_status_time = time;
    //printf("[COMM_MS] send_move_status, time: %f\n", time);
}

void COMM_MS::send_local_path()
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
    //printf("[COMM_MS] send local_path, num: %d\n", (int)path.pos.size());
}

void COMM_MS::send_global_path()
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
    //printf("[COMM_MS] send_global_path, num: %d\n", (int)path.pos.size());
}

void COMM_MS::send_lidar()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time0();

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

    //printf("[COMM_MS] lidar_cloud,  time: %f\n", time);
}

void COMM_MS::send_mapping_cloud()
{
    if(!is_connected)
    {
        return;
    }

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
        io->socket()->emit("mappingCloud", jsonArray);

        //QString str;
        //str.sprintf("[COMM_MS] mapping_cloud, time: %f\n", time);
        //logger->write_log(str, "Green");
        last_send_kfrm_idx++;
    }
}

void COMM_MS::slot_move(DATA_MOVE dmove)
{
    QString command = dmove.command;
    if(command == "jog")
    {
        double vx = dmove.jog_val[0];
        double vy = dmove.jog_val[1];
        double wz = dmove.jog_val[2];

        MainWindow* _main = (MainWindow*)main;
        _main->update_jog_values(vx, vy, wz*D2R);

        dmove.result = "accept";
        dmove.message = "";
    }
    else if(command == "target")
    {
        QString method = dmove.method;
        if(method == "pp")
        {
            if(unimap->is_loaded == false)
            {
                dmove.result = "reject";
                dmove.message = "map not loaded";

                send_move_response(dmove);
                return;
            }

            if(slam->is_loc == false)
            {
                dmove.result = "reject";
                dmove.message = "no localization";

                send_move_response(dmove);
                return;
            }

            double x = dmove.tgt_pose_vec[0]; double y = dmove.tgt_pose_vec[1];
            if(x < unimap->map_min_x || x > unimap->map_max_x || y < unimap->map_min_y || y > unimap->map_max_y)
            {
                dmove.result = "reject";
                dmove.message = "target location out of range";

                send_move_response(dmove);
                return;
            }

            Eigen::Vector4d pose_vec = dmove.tgt_pose_vec;
            Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0],pose_vec[1],pose_vec[3]*D2R));
            goal_tf(2,3) = pose_vec[2];

            if(obsmap->is_tf_collision(goal_tf))
            {
                dmove.result = "reject";
                dmove.message = "target location occupied";

                send_move_response(dmove);
                return;
            }

            // pure pursuit
            int preset = dmove.preset;
            ctrl->move_pp(goal_tf, preset);

            dmove.result = "accept";
            dmove.message = "";
            send_move_response(dmove);
        }
        else
        {
            dmove.result = "reject";
            dmove.message = "not supported";
            send_move_response(dmove);
        }
    }
    else if(command == "goal")
    {
        QString method = dmove.method;
        if(method == "pp")
        {
            if(unimap->is_loaded == false)
            {
                dmove.result = "reject";
                dmove.message = "map not loaded";
                send_move_response(dmove);
                return;
            }

            if(slam->is_loc == false)
            {
                dmove.result = "reject";
                dmove.message = "no localization";
                send_move_response(dmove);
                return;
            }

            QString goal_id = dmove.goal_node_id;
            if(goal_id == "")
            {
                dmove.result = "reject";
                dmove.message = "empty node id";
                send_move_response(dmove);
                return;
            }

            NODE* node = unimap->get_node_by_id(goal_id);
            if(node == NULL)
            {
                node = unimap->get_node_by_name(goal_id);
                if(node == NULL)
                {
                    dmove.result = "reject";
                    dmove.message = "invalid node id";
                    send_move_response(dmove);
                    return;
                }
            }

            Eigen::Matrix4d goal_tf = node->tf;
            ctrl->set_goal(goal_id);

            // pure pursuit
            int preset = dmove.preset;
            ctrl->move_pp(goal_tf, preset);

            dmove.result = "accept";
            dmove.message = "";
            send_move_response(dmove);
        }
        else
        {
            dmove.result = "reject";
            dmove.message = "not supported";
            send_move_response(dmove);
        }
    }
    else if(command == "pause")
    {
        ctrl->is_pause = true;

        dmove.result = "accept";
        dmove.message = "";
        send_move_response(dmove);
    }
    else if(command == "resume")
    {
        ctrl->is_pause = false;

        dmove.result = "accept";
        dmove.message = "";
        send_move_response(dmove);
    }
    else if(command == "stop")
    {
        ctrl->stop();

        dmove.result = "accept";
        dmove.message = "";
        send_move_response(dmove);
    }
}

void COMM_MS::slot_mapping(DATA_MAPPING dmap)
{
    QString command = dmap.command;
    if(command == "start")
    {
        last_send_kfrm_idx = 0;
        MainWindow* _main = (MainWindow*)main;
        if(lidar->is_connected_f)
        {
            _main->bt_MapBuild();

            dmap.result = "accept";
            dmap.message = "";
        }
        else
        {
            dmap.result = "reject";
            dmap.message = "lidar not connected";
        }

        send_mapping_response(dmap);
    }
    else if(command == "stop")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->bt_MapSave();

        dmap.result = "accept";
        dmap.message = "";

        send_mapping_response(dmap);
    }
    else if(command == "save")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->bt_MapSave();

        QString map_name = dmap.map_name;
        QString save_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + map_name;
        std::string command = "cp -r " + _main->map_dir.toStdString() + " '" + save_dir.toStdString() + "'";
        int result = std::system(command.c_str());
        if(result == 0)
        {
            dmap.result = "success";
            dmap.message = "";

            send_mapping_response(dmap);
        }
        else
        {
            dmap.result = "fail";
            dmap.message = "";

            send_mapping_response(dmap);
        }
    }
    else if(command == "reload")
    {
        last_send_kfrm_idx = 0;

        dmap.result = "accept";
        dmap.message = "";

        send_mapping_response(dmap);
    }
}

void COMM_MS::slot_load(DATA_LOAD dload)
{
    QString command = dload.command;
    if(command == "mapload")
    {
        QString map_name = dload.map_name;
        MainWindow* _main = (MainWindow*)main;

        QString load_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + map_name;
        if(!load_dir.isNull())
        {
            if(!QDir(load_dir).exists())
            {
                dload.result = "reject";
                dload.message = "invalid map dir";

                send_load_response(dload);
                return;
            }

            slam->localization_stop();
            obsmap->clear();

            _main->map_dir = load_dir;
            unimap->load_map(load_dir);
            _main->all_update();

            if(unimap->is_loaded)
            {
                dload.result = "success";
                dload.message = "";

                send_load_response(dload);
            }
            else
            {
                dload.result = "fail";
                dload.message = "map not load";

                send_load_response(dload);
            }
        }
    }
    else if(command == "topoload")
    {
        dload.result = "reject";
        dload.message = "not support yet";

        send_load_response(dload);
    }
    else if(command == "configload")
    {
        dload.result = "reject";
        dload.message = "not support yet";

        send_load_response(dload);
    }
}

void COMM_MS::slot_randomseq(DATA_RANDOMSEQ drandomseq)
{
    QString command = drandomseq.command;
    if(command == "randomseq")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_seq();

        drandomseq.result = "accept";
        drandomseq.message = "";

        send_randomseq_response(drandomseq);
    }
}

void COMM_MS::slot_localization(DATA_LOCALIZATION dloc)
{
    QString command = dloc.command;
    if(command == "semiautoinit")
    {
        if(unimap->is_loaded == false)
        {
            dloc.result = "reject";
            dloc.message = "not loaded map";

            send_localization_response(dloc);
            return;
        }
        if(lidar->is_connected_f == false)
        {
            dloc.result = "reject";
            dloc.message = "not connected front lidar";

            send_localization_response(dloc);
            return;
        }

        #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
        if(lidar->is_connected_b == false)
        {
            dloc.result = "reject";
            dloc.message = "not connected back lidar";

            send_localization_response(dloc);
            return;
        }
        #endif

        if(slam->is_busy)
        {
            dloc.result = "reject";
            dloc.message = "already running";

            send_localization_response(dloc);
            return;
        }

        logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);
        slam->localization_stop();

        // semi auto init
        if(semi_auto_init_thread != NULL)
        {
            logger->write_log("[AUTO_INIT] recv_loc, thread already running.", "Orange", true, false);
            semi_auto_init_thread->join();
            semi_auto_init_thread = NULL;
        }

        dloc.result = "accept";
        dloc.message = "";
        send_localization_response(dloc);

        semi_auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start, slam);
    }
    else if(command == "init")
    {
        if(unimap->is_loaded == false)
        {
            dloc.result = "reject";
            dloc.message = "not loaded map";

            send_localization_response(dloc);
            return;
        }
        if(lidar->is_connected_f == false)
        {
            dloc.result = "reject";
            dloc.message = "not connected front lidar";

            send_localization_response(dloc);
            return;
        }

        #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
        if(lidar->is_connected_b == false)
        {
            dloc.result = "reject";
            dloc.message = "not connected back lidar";

            send_localization_response(dloc);
            return;
        }
        #endif

        double x = dloc.tgt_pose_vec[0];
        double y = dloc.tgt_pose_vec[1];
        double rz = dloc.tgt_pose_vec[3];

        // manual init
        slam->mtx.lock();
        slam->cur_tf = se2_to_TF(Eigen::Vector3d(x, y, rz*D2R));
        slam->mtx.unlock();

        dloc.result = "accept";
        dloc.message = "";
        send_localization_response(dloc);
    }
    else if(command == "start")
    {
        slam->localization_start();

        dloc.result = "accept";
        dloc.message = "";
        send_localization_response(dloc);
    }
    else if(command == "stop")
    {
        slam->localization_stop();

        dloc.result = "accept";
        dloc.message = "";
        send_localization_response(dloc);
    }
    else if(command == "randominit")
    {
        QString seed = dloc.seed;

        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_init(seed);

        dloc.result = "accept";
        dloc.message = "";
        send_localization_response(dloc);
    }
}

void COMM_MS::slot_dock(DATA_DOCK ddock)
{
    QString command = ddock.command;
    if(command == "dock")
    {
        dctrl->move();

        ddock.result = "accept";
        ddock.message = "";
        send_dock_response(ddock);
    }
    else if(command == "undock")
    {
        dctrl->undock();

        ddock.result = "accept";
        ddock.message = "";
        send_dock_response(ddock);
    }
}

void COMM_MS::slot_view_lidar(DATA_VIEW_LIDAR dlidar)
{
    QString command = dlidar.command;
    if(command == "on")
    {
        QString str;
        str.sprintf("dlidar.frequency: %d", dlidar.frequency);
        logger->write_log(str, "white");

        MainWindow* _main = (MainWindow*)main;
        if(dlidar.frequency > 0)
        {
            _main->lidar_view_hz = dlidar.frequency;
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->lidar_view_hz = -1;
    }
}

void COMM_MS::slot_view_path(DATA_VIEW_PATH dpath)
{
    QString command = dpath.command;
    if(command == "on")
    {
        QString str;
        str.sprintf("dpath.frequency: %d", dpath.frequency);
        logger->write_log(str, "white");

        MainWindow* _main = (MainWindow*)main;
        if(dpath.frequency > 0)
        {
            _main->path_view_hz = dpath.frequency;
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->path_view_hz = -1;
    }
}

void COMM_MS::slot_led(DATA_LED dled)
{
    QString command = dled.command;
    if(command == "on")
    {
        // do something

        dled.result = "accept";
        dled.message = "";
    }
    else if(command == "undock")
    {
        // do something

        dled.result = "accept";
        dled.message = "";
    }
}

void COMM_MS::slot_motor(DATA_MOTOR dmotor)
{
    QString command = dmotor.command;
    if(command == "on")
    {
        mobile->motor_on();

        dmotor.result = "accept";
        dmotor.message = "";
    }
    else if(command == "off")
    {
        mobile->motor_off();

        dmotor.result = "accept";
        dmotor.message = "";
    }
}

void COMM_MS::slot_path(DATA_PATH dpath)
{
    QString command = dpath.command;
    if(command == "path")
    {
        QString path_str = dpath.path;
        QStringList path_str_list = path_str.split(",");

        std::vector<QString> path;
        for(int p = 0; p < path_str_list.size(); p++)
        {
            path.push_back(path_str_list[p]);
        }

        int preset = dpath.preset;
        ctrl->move_pp(path, preset);
    }
}

void COMM_MS::slot_vobs_r(DATA_VOBS_R dvobs_r)
{
    QString command = dvobs_r.command;
    if(command == "dvobs_r")
    {
        std::vector<Eigen::Vector3d> vobs_list;

        QString vobs_str = dvobs_r.vobs;
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
        obsmap->mtx.lock();
        obsmap->vobs_list_robots = vobs_list;
        obsmap->mtx.unlock();

        obsmap->update_vobs_map();
    }
}

void COMM_MS::slot_vobs_c(DATA_VOBS_C dvobs_c)
{
    QString command = dvobs_c.command;
    if(command == "dvobs_c")
    {
        QString vobs_str = dvobs_c.vobs;
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
        obsmap->mtx.lock();
        obsmap->vobs_list_closures = vobs_list;
        obsmap->mtx.unlock();

        obsmap->update_vobs_map();
    }
}

void COMM_MS::send_move_response(DATA_MOVE dmove)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject moveResponseObj;
    moveResponseObj["command"] = dmove.command;
    moveResponseObj["result"] = dmove.result;
    moveResponseObj["message"] = dmove.message;
    moveResponseObj["preset"] = QString::number(dmove.preset, 10);
    moveResponseObj["method"] = dmove.method;
    moveResponseObj["goal_id"] = dmove.goal_node_id;
    moveResponseObj["x"] = QString::number(dmove.tgt_pose_vec[0], 'f', 3);
    moveResponseObj["y"] = QString::number(dmove.tgt_pose_vec[1], 'f', 3);
    moveResponseObj["z"] = QString::number(dmove.tgt_pose_vec[2], 'f', 3);
    moveResponseObj["rz"] = QString::number(dmove.tgt_pose_vec[3], 'f', 3);
    moveResponseObj["vx"] = QString::number(dmove.jog_val[0], 'f', 3);
    moveResponseObj["vy"] = QString::number(dmove.jog_val[1], 'f', 3);
    moveResponseObj["wz"] = QString::number(dmove.jog_val[2], 'f', 3);
    moveResponseObj["time"] = QString::number((long long)(dmove.time*1000), 10);

    QJsonDocument doc(moveResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("moveResponse", res);
}

void COMM_MS::send_localization_response(DATA_LOCALIZATION dloc)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject locResponseObj;
    locResponseObj["command"] = dloc.command;
    locResponseObj["result"] = dloc.result;
    locResponseObj["message"] = dloc.message;
    locResponseObj["x"] = QString::number(dloc.tgt_pose_vec[0], 'f', 3);
    locResponseObj["y"] = QString::number(dloc.tgt_pose_vec[1], 'f', 3);
    locResponseObj["z"] = QString::number(dloc.tgt_pose_vec[2], 'f', 3);
    locResponseObj["rz"] = QString::number(dloc.tgt_pose_vec[3], 'f', 3);
    locResponseObj["seed"] = dloc.seed;
    locResponseObj["time"] = QString::number((long long)(dloc.time*1000), 10);

    QJsonDocument doc(locResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("localizationResponse", res);
}

void COMM_MS::send_load_response(DATA_LOAD dload)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject loadResponseObj;
    loadResponseObj["command"] = dload.command;
    loadResponseObj["result"] = dload.result;
    loadResponseObj["message"] = dload.message;
    loadResponseObj["name"] = dload.map_name;
    loadResponseObj["time"] = QString::number((long long)(dload.time*1000), 10);

    QJsonDocument doc(loadResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("loadResponse", res);
}

void COMM_MS::send_randomseq_response(DATA_RANDOMSEQ drandomseq)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject randomseqResponseObj;
    randomseqResponseObj["command"] = drandomseq.command;
    randomseqResponseObj["result"] = drandomseq.result;
    randomseqResponseObj["message"] = drandomseq.message;
    randomseqResponseObj["time"] = QString::number((long long)(drandomseq.time*1000), 10);

    QJsonDocument doc(randomseqResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("randomseqResponse", res);
}

void COMM_MS::send_mapping_response(DATA_MAPPING dmap)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject mapResponseObj;
    mapResponseObj["command"] = dmap.command;
    mapResponseObj["result"] = dmap.result;
    mapResponseObj["message"] = dmap.message;
    mapResponseObj["name"] = dmap.map_name;
    mapResponseObj["time"] = QString::number((long long)(dmap.time*1000), 10);

    QJsonDocument doc(mapResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mappingResponse", res);
}

void COMM_MS::send_dock_response(DATA_DOCK ddock)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject dockResponseObj;
    dockResponseObj["command"] = ddock.command;
    dockResponseObj["result"] = ddock.result;
    dockResponseObj["message"] = ddock.message;
    dockResponseObj["time"] = QString::number((long long)(ddock.time*1000), 10);

    QJsonDocument doc(dockResponseObj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("dockResponse", res);
}

