#include "comm_msa.h"
#include "mainwindow.h"

COMM_MSA* COMM_MSA::instance(QObject* parent)
{
    static COMM_MSA* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COMM_MSA(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

COMM_MSA::COMM_MSA(QObject *parent)
    : QObject{parent}
    , main(parent)
{
    // set recv callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    io = std::make_unique<sio::client>();

    client = new QWebSocket(QString(), QWebSocketProtocol::VersionLatest, this);
    send_timer = new QTimer(this);
    reconnect_timer = new QTimer(this);

    sio::socket::ptr sock = io->socket("slamnav");
    io->set_open_listener(std::bind(&COMM_MSA::connected, this));
    io->set_close_listener(std::bind(&COMM_MSA::disconnected, this));
    //    io->set_close_listener(std::bind(&COMM_MSA::sio_disconnected, this, _1));
    //    io->set_fail_listener(std::bind(&COMM_MSA::sio_error, this));

    //    io->set_fail_listener(std::bind(&COMM_MSA::sio_error, this));
    //    io->set_client_disconnect_listener(std::bind(&COMM_MSA::sio_client_disconnected, this));

    //    //    new Socket Event!!
    BIND_EVENT(sock, "moveRequest",         std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    BIND_EVENT(sock, "pathResponse",        std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    BIND_EVENT(sock, "loadRequest",         std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    BIND_EVENT(sock, "localizationRequest", std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    //    BIND_EVENT(sock, "controlRequest",      std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    BIND_EVENT(sock, "mappingRequest",      std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));

    BIND_EVENT(sock, "controlRequest",      std::bind(&COMM_MSA::recv_message_array, this, std::placeholders::_1));
    //    BIND_EVENT(sock, "mappingRequest",      std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));

    connect(send_timer,      SIGNAL(timeout()),                 this, SLOT(send_loop()));
    connect(reconnect_timer, SIGNAL(timeout()),                 this, SLOT(reconnect_loop()));

    connect(this, &COMM_MSA::signal_profile_move,this, &COMM_MSA::slot_profile_move,Qt::QueuedConnection);


}

COMM_MSA::~COMM_MSA()
{
    io->socket("slamnav")->off_all();
    io->socket("slamnav")->off_error();
    io->sync_close();

    reconnect_timer->stop();

    if(is_connected)
    {
        client->deleteLater();
    }

    is_recv_running = false;
    if(recv_thread->joinable())
    {
        recv_thread->join();
    }

    is_move_running = false;
    move_cv.notify_all();
    if(move_thread->joinable())
    {
        move_thread->join();
    }
    is_load_running = false;
    load_cv.notify_all();
    if(load_thread->joinable())
    {
        load_thread->join();
    }

    is_mapping_running = false;
    mapping_cv.notify_all();
    if(mapping_thread->joinable())
    {
        mapping_thread->join();
    }

    is_control_running = false;
    control_cv.notify_all();
    if(control_thread->joinable())
    {
        control_thread->join();
    }

    is_localization_running = false;
    localization_cv.notify_all();
    if(localization_thread->joinable())
    {
        localization_thread->join();
    }


    is_path_running = false;
    path_cv.notify_all();
    if(path_thread->joinable())
    {
        path_thread->join();
    }

    is_vobs_running = false;
    vobs_cv.notify_all();
    if(vobs_thread->joinable())
    {
        vobs_thread->join();
    }

    is_common_running = false;
    common_cv.notify_all();
    if(common_thread->joinable())

    {
        common_thread->join();
    }

    is_response_running = false;
    response_cv.notify_all();
    if(response_thread && response_thread->joinable())
    {
        response_thread->join();
    }

    // for move status -> 100ms
    is_send_status_running = false;
    status_cv.notify_all();
    if(status_thread->joinable())
    {
        status_thread->join();
    }
}

void COMM_MSA::set_config_module(CONFIG* _config)
{
    if(_config)
    {
        config = _config;
    }
}

void COMM_MSA::set_logger_module(LOGGER* _logger)
{
    if(_logger)
    {
        logger = _logger;
    }
}

void COMM_MSA::set_mobile_module(MOBILE* _mobile)
{
    if(_mobile)
    {
        mobile = _mobile;
    }
}

void COMM_MSA::set_lidar_2d_module(LIDAR_2D* _lidar)
{
    if(_lidar)
    {
        lidar_2d = _lidar;
    }
}

void COMM_MSA::set_lidar_3d_module(LIDAR_3D* _lidar)
{
    if(_lidar)
    {
        lidar_3d = _lidar;
    }
}

void COMM_MSA::set_cam_module(CAM* _cam)
{
    if(_cam)
    {
        cam = _cam;
    }
}

void COMM_MSA::set_localization_module(LOCALIZATION* _loc)
{
    if(_loc)
    {
        loc = _loc;
    }
}

void COMM_MSA::set_mapping_module(MAPPING* _mapping)
{
    if(_mapping)
    {
        mapping = _mapping;
    }
}

void COMM_MSA::set_unimap_module(UNIMAP* _unimap)
{
    if(_unimap)
    {
        unimap = _unimap;
    }
}

void COMM_MSA::set_obsmap_module(OBSMAP* _obsmap)
{
    if(_obsmap)
    {
        obsmap = _obsmap;
    }
}

void COMM_MSA::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(_ctrl)
    {
        ctrl = _ctrl;
    }
}

void COMM_MSA::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if(_dctrl)
    {
        dctrl = _dctrl;
    }
}

void COMM_MSA::set_global_path_update()
{
    is_global_path_update2 = true;
}

void COMM_MSA::set_local_path_update()
{
    is_local_path_update2 = true;
}

QString COMM_MSA::get_json(const QJsonObject& json, QString key)
{
    return json[key].toString();
}

int COMM_MSA::get_json_int(const QJsonObject& json, QString key)
{
    return json[key].toInt();
}

double COMM_MSA::get_json_double(const QJsonObject& json, QString key)
{
    return json[key].toDouble();
}

QString COMM_MSA::get_multi_state()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return multi_state;
}

double COMM_MSA::get_process_time_path()
{
    return (double)process_time_path.load();
}

double COMM_MSA::get_process_time_vobs()
{
    return (double)process_time_vobs.load();
}

double COMM_MSA::get_max_process_time_path()
{
    return (double)max_process_time_path.load();
}

double COMM_MSA::get_max_process_time_vobs()
{
    return (double)max_process_time_vobs.load();
}

void COMM_MSA::init()
{
    if(!config)
    {
        logger->write_log("[COMM_RRS] config module not set", "Orange");
        spdlog::warn("[COMM_RRS] config module not set");
        return;
    }

    if(config->get_use_msa())
    {
        // update robot id
        robot_id = QString("R_%1").arg(static_cast<long long>(get_time0()*1000));
        //    logger->write_log(QString("[COMM_MSA] ID: %1").arg(robot_id));

        // start reconnect loop
        reconnect_timer->start(3000);
        logger->write_log("[COMM_MSA] start reconnect timer");

        if(recv_thread == nullptr)
        {
            is_recv_running = true;
            recv_thread = std::make_unique<std::thread>(&COMM_MSA::recv_loop, this);
        }

        if(move_thread == nullptr)
        {
            move_thread = std::make_unique<std::thread>(&COMM_MSA::move_loop, this);
        }

        if(load_thread == nullptr)
        {
            load_thread = std::make_unique<std::thread>(&COMM_MSA::load_loop, this);
        }
        if(mapping_thread == nullptr)
        {
            mapping_thread = std::make_unique<std::thread>(&COMM_MSA::mapping_loop, this);
        }
        if(localization_thread == nullptr)
        {
            localization_thread = std::make_unique<std::thread>(&COMM_MSA::localization_loop, this);
        }
        if(control_thread == nullptr)
        {
            control_thread = std::make_unique<std::thread>(&COMM_MSA::control_loop, this);
        }
        if(path_thread == nullptr)
        {
            path_thread = std::make_unique<std::thread>(&COMM_MSA::path_loop, this);
        }

        if(vobs_thread == nullptr)
        {
            vobs_thread = std::make_unique<std::thread>(&COMM_MSA::vobs_loop, this);
        }

        if(common_thread == nullptr)
        {
            common_thread = std::make_unique<std::thread>(&COMM_MSA::common_loop, this);
        }

        if(response_thread == nullptr)
        {
            response_thread = std::make_unique<std::thread>(&COMM_MSA::response_loop, this);
        }

        if(status_thread == nullptr)
        {
            status_thread = std::make_unique<std::thread>(&COMM_MSA::send_status_loop, this);
        }

        send_timer->start(3);
    }
}

void COMM_MSA::reconnect_loop()
{
    if(!is_connected)
    {
        if(!config || !client)
        {
            logger->write_log("[COMM_MSA] not ready to modules");
            return;
        }

        QString server_ip = config->get_server_ip();
        if(server_ip.isEmpty())
        {
            logger->write_log("[COMM_MSA] Invalid server ip");
            return;
        }
        io->connect("ws://localhost:15001");
//         io->connect("ws://10.108.1.31:15001");
        io->socket("slamnav");
    }
}

void COMM_MSA::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        logger->write_log("[COMM_MSA] connected");

        if(!ctrl)
        {
            logger->write_log("[COMM_MSA] not ready to modules");
            return;
        }

        ctrl->set_is_rrs(true);
    }
}

void COMM_MSA::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        logger->write_log("[COMM_MSA] disconnected");

        if(!ctrl)
        {
            logger->write_log("[COMM_MSA] not ready to modules");
            return;
        }

        ctrl->set_is_rrs(false);
    }
}

// send status
void COMM_MSA::send_move_status()
{

    if(!is_connected || !ctrl || !mobile || !unimap || !dctrl)
    {
        //        printf("is_connected : %d\n", (int)is_connected.load());
        return;
    }

    // Creating the JSON object
    sio::object_message::ptr rootObj = sio::object_message::create();

    QString cur_node_id = ctrl->get_cur_node_id();

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

    sio::object_message::ptr moveStateObj = sio::object_message::create();
    moveStateObj->get_map()["auto_move"] = sio::string_message::create(auto_state.toStdString()); // "stop", "move", "pause", "error", "not ready", "vir"
    moveStateObj->get_map()["dock_move"] = sio::string_message::create(dock_state.toStdString());
    moveStateObj->get_map()["jog_move"] = sio::string_message::create(jog_state.toStdString());
    moveStateObj->get_map()["obs"] = sio::string_message::create(ctrl->get_obs_condition().toStdString());
    moveStateObj->get_map()["path"] = sio::string_message::create(ctrl->get_multi_reqest_state().toStdString()); // "none", "req_path", "recv_path"
    rootObj->get_map()["move_state"] = moveStateObj;

    // Adding the pose object
    const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    const Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    sio::object_message::ptr poseObj = sio::object_message::create();
    poseObj->get_map()["x"]  = sio::double_message::create(cur_xi[0]);
    poseObj->get_map()["y"]  = sio::double_message::create(cur_xi[1]);
    poseObj->get_map()["rz"] = sio::double_message::create(cur_xi[2] * R2D);
    rootObj->get_map()["pose"] = poseObj;

    // Adding the velocity object
    MOBILE_POSE mo = mobile->get_pose();
    sio::object_message::ptr velObj = sio::object_message::create();

    velObj->get_map()["vx"] = sio::double_message::create(mo.vel[0]);
    velObj->get_map()["vy"] = sio::double_message::create(mo.vel[1]);
    velObj->get_map()["wz"] = sio::double_message::create(mo.vel[2] * R2D);
    rootObj->get_map()["vel"] = velObj;

    QString cur_node_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
    {
        cur_node_id = AUTOCONTROL::instance()->get_cur_node_id();
        NODE *cur_node = UNIMAP::instance()->get_node_by_id(cur_node_id);

        if(cur_node != nullptr)
        {
            cur_node_name = cur_node -> name;
        }
    }

    sio::object_message::ptr curNodeObj = sio::object_message::create();

    curNodeObj->get_map()["id"] = sio::string_message::create(cur_node_id.toStdString());
    curNodeObj->get_map()["name"] = sio::string_message::create(cur_node_name.toStdString());
    curNodeObj->get_map()["state"] = sio::string_message::create(auto_state.toStdString());
    curNodeObj->get_map()["x"] = sio::double_message::create(cur_xi[0]);
    curNodeObj->get_map()["y"] = sio::double_message::create(cur_xi[1]);
    curNodeObj->get_map()["rz"] =sio::double_message::create(cur_xi[2] * R2D);
    rootObj->get_map()["cur_node"] = curNodeObj;

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

    sio::object_message::ptr goalNodeObj = sio::object_message::create();

    goalNodeObj->get_map()["id"]    = sio::string_message::create(goal_node_id.toStdString());
    goalNodeObj->get_map()["name"]  = sio::string_message::create(goal_node_name.toStdString());
    goalNodeObj->get_map()["state"] = sio::string_message::create(goal_state.toStdString());

    goalNodeObj->get_map()["x"]  = sio::double_message::create(goal_xi[0]);
    goalNodeObj->get_map()["y"]  = sio::double_message::create(goal_xi[1]);
    goalNodeObj->get_map()["rz"] = sio::double_message::create(goal_xi[2] * R2D);

    rootObj->get_map()["goal_node"] = goalNodeObj;

    // Adding the time object
    const double time = get_time0();
    rootObj->get_map()["time"]   = sio::string_message::create(QString::number((long long)(time*1000)).toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "moveStatus";
    socket_msg.data = rootObj;  // 타입 그대로 전달

    send_queue.push(socket_msg);
}

void COMM_MSA::recv_message(sio::event& ev)
{
    sio::message::ptr msg = ev.get_message();

    if (msg)
    {
        // string
        if (msg->get_flag() == sio::message::flag_string)
        {
            std::string data = msg->get_string();
            std::cout << "받은 문자열 메시지: " << data << std::endl;
        }
        // number
        else if (msg->get_flag() == sio::message::flag_integer)
        {
            int val = msg->get_int();
            std::cout << "받은 정수 메시지: " << val << std::endl;
        }
        // JSON
        else if (msg->get_flag() == sio::message::flag_object)
        {
            auto obj = msg->get_map();
            QJsonObject json_obj;
            for (auto& kv : obj)
            {
                if (kv.second->get_flag() == sio::message::flag_string)
                {
                    json_obj.insert(QString::fromStdString(kv.first),
                                    QString::fromStdString(kv.second->get_string()));
                }
                else if (kv.second->get_flag() == sio::message::flag_integer)
                {
                    json_obj.insert(QString::fromStdString(kv.first),
                                    QJsonValue::fromVariant(QVariant::fromValue<qint64>(kv.second->get_int())));
                }
                else if (kv.second->get_flag() == sio::message::flag_double)
                {
                    json_obj.insert(QString::fromStdString(kv.first),
                                    QJsonValue(kv.second->get_double()));
                }
                else if (kv.second->get_flag() == sio::message::flag_boolean)
                {
                    json_obj.insert(QString::fromStdString(kv.first),
                                    QJsonValue(kv.second->get_bool()));
                }
                else if (kv.second->get_flag() == sio::message::flag_array || kv.second->get_flag() == sio::message::flag_object)
                {
                    QString key = QString::fromStdString(kv.first);
                    json_obj.insert(key, convertItem(kv.second)); // 재귀 호출
                }
            }

            QJsonObject root;
            root.insert("topic", QString::fromStdString(ev.get_name()));
            root.insert("data", json_obj);

            QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
//            qDebug()<<wrapped;

            std::shared_lock<std::shared_mutex> lock(msg_mtx);
            receive_msg = wrapped;

            recv_queue.push(wrapped);
        }
    }
}

void COMM_MSA::recv_message_array(sio::event& ev)
{
    sio::message::ptr msg = ev.get_message();
    if (!msg || msg->get_flag() != sio::message::flag_object)
    {
        return;
    }
    //       qDebug()<<"im hear!!!!!1";

    auto obj = msg->get_map();
    QJsonObject json_obj;

    // 최상위 단순 key/value 변환
    for (auto& kv : obj)
    {
        QString key = QString::fromStdString(kv.first);
        auto flag = kv.second->get_flag();

        if (flag == sio::message::flag_string)
        {
            json_obj.insert(key, QString::fromStdString(kv.second->get_string()));
        }
        else if (flag == sio::message::flag_integer)
        {
            json_obj.insert(key, QJsonValue::fromVariant(QVariant::fromValue<qint64>(kv.second->get_int())));
        }
        else if (flag == sio::message::flag_double)
        {
            json_obj.insert(key, kv.second->get_double());
        }
        else if (flag == sio::message::flag_boolean)
        {
            json_obj.insert(key, kv.second->get_bool());
        }
        else if (flag == sio::message::flag_array || flag == sio::message::flag_object)
        {
            json_obj.insert(key, convertItem(kv.second)); // 재귀 호출
        }
    }

    QString cmd = get_json(json_obj, "command"); // command 확인

    if (cmd == "setDigitalIO")
    {
        if (json_obj.contains("mcuDio"))
        {
            auto arrMsg = obj["mcuDio"];
            QJsonValue parsed = convertItem(arrMsg);
            json_obj.insert("mcuDio", parsed);
        }

        if (json_obj.contains("mcuDin"))
        {
            auto arrMsg = obj["mcuDin"];
            QJsonValue parsed = convertItem(arrMsg);
            json_obj.insert("mcuDin", parsed);
        }
    }

    QJsonObject root;
    root.insert("topic", QString::fromStdString(ev.get_name()));
    root.insert("data", json_obj);

    QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
//    qDebug() << "recive msg :" << wrapped;
    std::shared_lock<std::shared_mutex> lock(msg_mtx);
    receive_msg = wrapped;

    recv_queue.push(wrapped);
}

void COMM_MSA::recv_loop()
{
    while(is_recv_running)
    {
        QString recv_msg;
        if(recv_queue.try_pop(recv_msg))
        {
            QJsonObject root_obj = QJsonDocument::fromJson(recv_msg.toUtf8()).object();
            QString cmd = get_json(root_obj, "topic");
            QJsonObject data = root_obj.value("data").toObject();

            if(cmd == "moveRequest")
            {
                handle_move_cmd(data);
            }
            if(cmd == "pathResponse")
            {
                handle_path_cmd(data);
            }
            else if(cmd == "loadRequest")
            {
                handle_load_cmd(data);
            }
            else if(cmd == "localizationRequest")
            {
                handle_localization_cmd(data);
            }
            else if(cmd == "controlRequest")
            {
                handle_control_cmd(data);
            }
            else if(cmd == "mappingRequest")
            {
                handle_mapping_cmd(data);
            }

            logger->write_log(QString("[COMM_MSA] recv, command: %1, time: %2").arg(cmd).arg(get_time0()), "Green");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

QJsonValue COMM_MSA::convertItem(sio::message::ptr item)
{
    if (!item)
    {
        return QJsonValue();
    }

    switch (item->get_flag())
    {
    case sio::message::flag_string:
    {
        return QString::fromStdString(item->get_string());
    }
    case sio::message::flag_integer:
    {
        return QJsonValue::fromVariant(QVariant::fromValue<qint64>(item->get_int()));
    }
    case sio::message::flag_double:
    {
        return item->get_double();
    }
    case sio::message::flag_boolean:
    {
        return item->get_bool();
    }
    case sio::message::flag_array:
    {
        QJsonArray arr;
        for (auto& sub_item : item->get_vector())
        {
            arr.append(convertItem(sub_item)); // 재귀 호출
        }
        return arr;
    }
    case sio::message::flag_object:
    {
        QJsonObject obj;
        for (auto& kv : item->get_map())
            obj.insert(QString::fromStdString(kv.first), convertItem(kv.second));
        return obj;
    }
    default:
        return QJsonValue();
    }
}

void COMM_MSA::handle_path_cmd(const QJsonObject& data)
{
    if(is_before_given_path)
    {
        DATA_PATH msg;
        msg.path          = get_json(data, "path");

        QJsonArray path_array = data.value("path").toArray();

        if (path_array.isEmpty())
        {
            return;
        }

        QStringList path_list;
        for (const QJsonValue &val : path_array)
        {
            QString name = val.toString();
             NODE* node = unimap->get_node_by_name(name);
             if(node)
             {
                 path_list << node->id;
             }
        }

        msg.path = path_list.join(",");
        msg.time          = get_json(data, "time").toLongLong();
        msg.preset        = get_json_int(data, "preset");
        msg.action        = get_json(data, "action");
        msg.command       = get_json(data, "command");
        msg.vobs_closures = get_json(data, "vobs_c");
        //        qDebug()<<" msg.path : "<< msg.path;

        {
            std::lock_guard<std::mutex> lock(path_mtx);
            path_queue.push(msg);
        }
    }
    is_before_given_path = false;

    path_cv.notify_one();
}

void COMM_MSA::handle_vobs_cmd(const QJsonObject& data)
{
    DATA_VOBS msg;
    msg.time                    = get_json_double(data, "time")/1000;
    msg.command                 = get_json(data, "command");
    msg.vobs_robots             = get_json(data, "vobs_r");
    msg.vobs_clousers           = get_json(data, "vobs_c");
    msg.is_vobs_closures_change = get_json(data, "is_vobs_c");
    {
        std::lock_guard<std::mutex> lock(vobs_mtx);
        vobs_queue.push(msg);
    }

    vobs_cv.notify_one();
}

void COMM_MSA::handle_move_cmd(const QJsonObject& data)
{
    DATA_MOVE msg;
//    msg.id              = get_json(data, "id");
//    msg.time            = get_json_double(data, "time")/1000;
//    msg.method          = get_json(data, "method");
//    msg.preset          = get_json_int(data, "preset");
//    msg.command         = get_json(data, "command");
//    msg.direction       = get_json(data, "direction");
//    msg.jog_val[0]      = get_json_double(data, "vx");
//    msg.jog_val[1]      = get_json_double(data, "vy");
//    msg.jog_val[2]      = get_json_double(data, "wz");
//    msg.goal_node_id    = get_json(data, "goalId");
//    msg.tgt_pose_vec[0] = get_json_double(data, "x");
//    msg.tgt_pose_vec[1] = get_json_double(data, "y");
//    msg.tgt_pose_vec[2] = get_json_double(data, "z");
//    msg.tgt_pose_vec[3] = get_json_double(data, "rz")*D2R;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time") / 1000.0;
    msg.preset          = get_json_int(data, "preset");

    msg.command         = get_json(data, "command");
    msg.method          = get_json(data, "method");
    msg.direction       = get_json(data, "direction");
    // msg.dir           = get_json(data, "dir"); // 필요 시 사용

    msg.goal_node_id    = get_json(data, "goalId");
    msg.goal_node_name  = get_json(data, "goalName");  // JSON에 존재하면 추가

    // 위치 및 속도 정보
    msg.cur_pos.setZero(); // 필요 시 갱신
    msg.tgt_pose_vec[0] = get_json_double(data, "x");
    msg.tgt_pose_vec[1] = get_json_double(data, "y");
    msg.tgt_pose_vec[2] = get_json_double(data, "z");
    msg.tgt_pose_vec[3] = get_json_double(data, "rz") * D2R;

    msg.jog_val[0] = get_json_double(data, "vx");
    msg.jog_val[1] = get_json_double(data, "vy");
    msg.jog_val[2] = get_json_double(data, "wz");

    // 이동 파라미터
    msg.target          = get_json_double(data, "target");
    msg.speed           = get_json_double(data, "speed");
    msg.meassured_dist  = get_json_double(data, "measuredDist");

    // 진행 상태
    msg.remaining_dist  = get_json_double(data, "remainingDist");
    msg.remaining_time  = get_json_double(data, "remainingTime");
    msg.bat_percent     = get_json_int(data, "battery");

    // 결과
    msg.result          = get_json(data, "result");
    msg.message         = get_json(data, "message");

    {
        std::lock_guard<std::mutex> lock(move_mtx);
        move_queue.push(msg);
    }

    Q_EMIT signal_send_move_status();

    move_cv.notify_one();
}

void COMM_MSA::handle_load_cmd(const QJsonObject& data)
{
    // parsing
    DATA_LOAD msg;
    msg.id = data["id"].toString();
    msg.command = get_json(data, "command"); // "mapload", "topoload", "configload"
    msg.map_name = get_json(data, "mapName");
    msg.time = get_json(data, "time").toDouble() / 1000;

    {
        std::lock_guard<std::mutex> lock(load_mtx);
        load_queue.push(msg);
    }
    load_cv.notify_one();
}

void COMM_MSA::handle_control_cmd(const QJsonObject &data)
{
    DATA_CONTROL msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time")/1000;
    msg.command         = get_json(data, "command");

    if(msg.command == DATA_CONTROL::SetSafetyField)
    {
        msg.safetyField     = data["safetyField"].toString();
    }
    else if(msg.command == DATA_CONTROL::GetSafetyField)
    {
        msg.safetyField     = data["safetyField"].toString();
    }
    else if(msg.command == DATA_CONTROL::ResetSafetyField)
    {
        msg.resetField      = data["resetField"].toString();
    }
    else if(msg.command == DATA_CONTROL::LedControl)
    {
        msg.onoff           = data["onoff"].toBool();
        msg.color           = data["color"].toString();
    }
    else if(msg.command == DATA_CONTROL::LidarOnOff)
    {
        msg.onoff           = data["onoff"].toBool();
        msg.frequency       = data["frequency"].toInt();
    }
    else if(msg.command == DATA_CONTROL::PathOnOff)
    {
        msg.onoff           = data["onoff"].toBool();
        msg.frequency       = data["frequency"].toInt();
    }
    else if(msg.command == DATA_CONTROL::MotorOnOff)
    {
        msg.onoff           = data["onoff"].toBool();
    }
    else if(msg.command == DATA_CONTROL::setDigitalIO)
    {
        handle_safetyio_cmd(data);
        return;
    }
    else if(msg.command == DATA_CONTROL::getDigitalIO)
    {
        handle_send_safetyIO(data);
        return;
    }

    {
        std::lock_guard<std::mutex> lock(control_mtx);
        control_queue.push(msg);
    }
    control_cv.notify_one();


}
void COMM_MSA::handle_mapping_cmd(const QJsonObject& data)
{
    DATA_MAPPING msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time")/1000;
    msg.command         = get_json(data, "command");
    msg.map_name        = get_json(data, "mapName");
    {
        std::lock_guard<std::mutex> lock(mapping_mtx);
        mapping_queue.push(msg);
    }
    mapping_cv.notify_one();
}

void COMM_MSA::handle_localization_cmd(const QJsonObject& data)
{
    DATA_LOCALIZATION msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time")/1000;
    msg.command         = get_json(data, "command");

    if (msg.command == "init")
    {
        msg.tgt_pose_vec[0] = data["x"].toDouble();
        msg.tgt_pose_vec[1] = data["y"].toDouble();
        msg.tgt_pose_vec[2] = data["z"].toDouble();
        msg.tgt_pose_vec[3] = data["rz"].toDouble();

        // init 시점에 저장
        last_tgt_pose_vec = msg.tgt_pose_vec;
    }
    else if (msg.command == "start")
    {
        // init에서 저장된 좌표값 재사용
        msg.tgt_pose_vec = last_tgt_pose_vec;

//        qDebug().nospace() << "start uses last pose x=" << msg.tgt_pose_vec[0]
//                           << ", y=" << msg.tgt_pose_vec[1]
//                           << ", rz=" << msg.tgt_pose_vec[3];
    }

    {
        std::lock_guard<std::mutex> lock(localization_mtx);
        localization_queue.push(msg);
    }
    localization_cv.notify_one();
}
void COMM_MSA::handle_common_cmd(QString cmd, const QJsonObject& data)
{
    if(cmd == "load")
    {
        DATA_LOAD msg;
        msg.time     = get_json_double(data, "time")/1000;
        msg.command  = get_json(data, "command");
        msg.map_name = get_json(data, "name");

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::LOAD;
        cmsg.dload = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
//    else if(cmd == "localization")
//    {
//        DATA_LOCALIZATION msg;
//        msg.seed            = get_json(data, "seed");
//        msg.time            = get_json(data, "time").toDouble() / 1000.;
//        msg.command         = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"

//        msg.tgt_pose_vec[0] = get_json_double(data, "x");
//        msg.tgt_pose_vec[1] = get_json_double(data, "y");
//        msg.tgt_pose_vec[2] = get_json_double(data, "z");
//        msg.tgt_pose_vec[3] = get_json_double(data, "rz");

//        DATA_COMMON cmsg;
//        cmsg.type = DATA_COMMON::TYPE::LOCALIZATION;
//        cmsg.dlocalization = msg;
//        {
//            std::lock_guard<std::mutex> lock(common_mtx);
//            common_queue.push(cmsg);
//        }

//        common_cv.notify_one();
//    }
    else if(cmd == "randomseq")
    {
        DATA_RANDOMSEQ msg;
        msg.time    = get_json_double(data, "time") / 1000.;
        msg.command = get_json(data, "command");  // "randomseq"

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::RANDOMSEQ;
        cmsg.drandomseq = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "mapping")
    {
        DATA_MAPPING msg;
        msg.time    = get_json_double(data, "time")/1000;
        msg.command = get_json(data, "command"); // "start", "stop", "save", "name", "reload"
        msg.map_name = get_json(data, "name");

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::MAPPING;
        cmsg.dmapping = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "docking")
    {
        DATA_DOCK msg;
        msg.time    = get_json_double(data, "time") / 1000.;
        msg.command = get_json(data, "command"); // "dock", "undock"

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::DOCKING;
        cmsg.ddock = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "lidar_onoff")
    {
        DATA_VIEW_LIDAR msg;
        msg.time      = get_json_double(data, "time") / 1000.;
        msg.command   = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::VIEW_LIDAR;
        cmsg.dlidar = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "path_onoff")
    {
        DATA_VIEW_PATH msg;
        msg.time      = get_json_double(data, "time") / 1000.;
        msg.command   = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json_int(data, "frequency");

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::VIEW_PATH;
        cmsg.dpath = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "led")
    {
        DATA_LED msg;
        msg.led     = get_json(data, "led");
        msg.time    = get_json(data, "time").toDouble() / 1000.;
        msg.command = get_json(data, "command"); // "on", "off"

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::LED;
        cmsg.dled = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "motor")
    {
        DATA_MOTOR msg;
        msg.time    = get_json(data, "time").toDouble() / 1000.;
        msg.command = get_json(data, "command"); // "on", "off"

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::MOTOR;
        cmsg.dmotor = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
}

void COMM_MSA::handle_send_safetyIO(const QJsonObject& data)
{
    DATA_SAFTYIO msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time")/1000;
    msg.command         = get_json(data, "command");

    MOBILE_STATUS ms = mobile->get_status();
    sio::object_message::ptr rootObj = sio::object_message::create();
    rootObj->get_map()["id"] = sio::string_message::create(msg.id.toStdString());

    auto toSioArray = [](unsigned char arr[8]) {
        sio::array_message::ptr jsonArr = sio::array_message::create();
        for (int i = 0; i < 8; ++i) {
            jsonArr->get_vector().push_back(sio::int_message::create(arr[i]));
        }
        return jsonArr;
    };

    // mcuDio 배열 생성
    sio::array_message::ptr mcuDioArr = sio::array_message::create();
    mcuDioArr->get_vector().push_back(toSioArray(ms.mcu0_dio));
    mcuDioArr->get_vector().push_back(toSioArray(ms.mcu1_dio));

    // mcuDin 배열 생성
    sio::array_message::ptr mcuDinArr = sio::array_message::create();
    mcuDinArr->get_vector().push_back(toSioArray(ms.mcu0_din));
    mcuDinArr->get_vector().push_back(toSioArray(ms.mcu1_din));

    // rootObj에 직접 넣기
    rootObj->get_map()["mcuDio"] = mcuDioArr;
    rootObj->get_map()["mcuDin"] = mcuDinArr;

    msg.result = "success";

    // Adding the time object
    const double time = get_time0();
    rootObj->get_map()["time"] = sio::string_message::create(std::to_string(static_cast<long long>(time * 1000)));
    rootObj->get_map()["result"] = sio::string_message::create(msg.result.toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "controlResponse";
    socket_msg.data = rootObj;  // 타입 그대로 전달

    io->socket("slamnav")->emit(socket_msg.event.toStdString(), socket_msg.data);

    //    {
    //        std::lock_guard<std::mutex> lock(mapping_mtx);
    //        mapping_queue.push(msg);
    //    }
    //    mapping_cv.notify_one();
}

void COMM_MSA::handle_safetyio_cmd(const QJsonObject& data)
{
    DATA_SAFTYIO msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time")/1000;
    msg.command         = get_json(data, "command");

    QJsonArray totalArr = data.value("mcuDio").toArray();

    if (totalArr.size() < 2)
    {
        return;
    }

    // MCU0
    QJsonArray mcu0_arr = totalArr[0].toArray();
    for(int i = 0; i < 8 && i < mcu0_arr.size(); i++)
    {
        //        msg.mcu0_dio[i] = static_cast<unsigned char>(mcu0_arr[i].toInt());
        msg.mcu0_dio[i] = (mcu0_arr[i].toInt() != 0) ? 1 : 0;
    }

    // MCU1
    QJsonArray mcu1_arr = totalArr[1].toArray();
    for(int i = 0; i < 8 && i < mcu1_arr.size(); i++)
    {
        //        msg.mcu1_dio[i] = static_cast<unsigned char>(mcu1_arr[i].toInt());
        msg.mcu1_dio[i] = (mcu1_arr[i].toInt() != 0) ? 1 : 0;
    }

    slot_safety_io(msg);
    //    send_safetyio_response(data);

}

void COMM_MSA::send_safetyio_response(const QJsonObject& data)
{
    sio::object_message::ptr response = sio::object_message::create();

    // command
    QString command = data.value("command").toString();
    response->get_map()["command"] = sio::string_message::create(command.toStdString());

    // id
    QString id = data.value("id").toString();
    response->get_map()["id"] = sio::string_message::create(id.toStdString());

    // mcuDio
    QJsonArray totalArr = data.value("mcuDio").toArray();
    sio::array_message::ptr mcuDioArr = sio::array_message::create();

    auto toSioArray = [](const QJsonArray& arr) {
        sio::array_message::ptr sub = sio::array_message::create();
        for (int i = 0; i < arr.size(); ++i)
        {
            sub->get_vector().push_back(sio::int_message::create(arr[i].toInt()));
        }
        return sub;
    };

    for (int i = 0; i < totalArr.size(); ++i)
    {
        mcuDioArr->get_vector().push_back(toSioArray(totalArr[i].toArray()));
    }
    response->get_map()["mcuDio"] = mcuDioArr;

    // time
    QString time = data.value("time").toString();
    response->get_map()["time"] = sio::string_message::create(time.toStdString());

    // result
    response->get_map()["result"] = sio::string_message::create("success");

    io->socket("slamnav")->emit("controlResponse",response);

}


void COMM_MSA::move_loop()
{
    while(is_move_running)
    {
        std::unique_lock<std::mutex> lock(move_mtx);
        move_cv.wait(lock, [this]
        {
            return !move_queue.empty() || !is_move_running;
        });

        if(!is_move_running)
        {
            break;
        }

        if(move_queue.size() == 0)
        {
            continue;
        }

        DATA_MOVE msg = move_queue.front();

        move_queue.pop();
        lock.unlock();

        QString command = msg.command;
        if(command == "jog")
        {
            handle_move_jog(msg);
        }
        else if(command == "target")
        {
            handle_move_target(msg);
        }
        else if(command == "goal" || command == "change_goal")
        {
            handle_move_goal(msg);
        }
        else if(command == "pause")
        {
            handle_move_pause(msg);
        }
        else if(command == "resume")
        {
            handle_move_resume(msg);
        }
        else if(command == "stop")
        {
            handle_move_stop(msg);
        }
        else if(command == "xLinear" || command == "yLinear" || command == "circular" || command == "rotate")
        {
            Q_EMIT (signal_profile_move(msg));
        }
    }
}

void COMM_MSA::path_loop()
{
    while(is_path_running)
    {
        std::unique_lock<std::mutex> lock(path_mtx);
        path_cv.wait(lock, [this]
        {
            return !path_queue.empty() || !is_path_running;
        });

        if(!is_path_running)
        {
            break;
        }

        if(path_queue.size() == 0)
        {
            continue;
        }

        DATA_PATH msg = path_queue.front();
        path_queue.pop();
        lock.unlock();

        double st_time = get_time0();

//        QString command = msg.command;
        msg.preset = 0;
        msg.method = given_method;
        qDebug()<<"given_method : "<<given_method;

        handle_path_move(msg);

        double ed_time = get_time0();
        process_time_path = ed_time - st_time;
        if(max_process_time_path < process_time_path)
        {
            max_process_time_path = (double)process_time_path;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::vobs_loop()
{
    while(is_vobs_running)
    {
        std::unique_lock<std::mutex> lock(vobs_mtx);
        vobs_cv.wait(lock, [this]
        {
            return !vobs_queue.empty() || !is_vobs_running;
        });

        if(!is_vobs_running)
        {
            break;
        }

        if(vobs_queue.size() == 0)
        {
            continue;
        }

        DATA_VOBS msg = vobs_queue.front();
        vobs_queue.pop();
        lock.unlock();

        double st_time = get_time0();

        QString command = msg.command;
        if(command == "vobs")
        {
            handle_vobs(msg);
        }

        double ed_time = get_time0();
        process_time_vobs = ed_time - st_time;
        if(max_process_time_vobs < process_time_vobs)
        {
            max_process_time_vobs = (double)process_time_vobs;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::load_loop()
{
    while(is_load_running)
    {
        std::unique_lock<std::mutex> lock(load_mtx);
        load_cv.wait(lock, [this]
        {
            return !load_queue.empty() || !is_load_running;
        });

        if(!is_load_running)
        {
            break;
        }

        if(load_queue.size() == 0)
        {
            continue;
        }

        DATA_LOAD msg = load_queue.front();
        load_queue.pop();
        lock.unlock();

        QString command = msg.command;
        qDebug() << "Load Loop : " << msg.command;
        if(command == "loadMap")
        {
            handle_common_load_map(msg);
        }
        else if(command == "loadTopo")
        {
            handle_common_load_topo(msg);
        }
        else
        {
            msg.result = "reject";
            msg.message = "알 수 없는 command 값입니다.";
            send_load_response(msg);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::mapping_loop()
{
    while(is_mapping_running)
    {
        std::unique_lock<std::mutex> lock(mapping_mtx);
        mapping_cv.wait(lock, [this]
        {
            return !mapping_queue.empty() || !is_mapping_running;
        });

        if(!is_mapping_running)
        {
            break;
        }

        if(mapping_queue.size() == 0)
        {
            continue;
        }

        DATA_MAPPING msg = mapping_queue.front();
        handle_mapping(msg);

        mapping_queue.pop();
        lock.unlock();



        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::localization_loop()
{
    while(is_localization_running)
    {
        std::unique_lock<std::mutex> lock(localization_mtx);
        localization_cv.wait(lock, [this]
        {
            return !localization_queue.empty() || !is_localization_running;
        });

        if(!is_localization_running)
        {
            break;
        }

        if(localization_queue.size() == 0)
        {
            continue;
        }

        DATA_LOCALIZATION msg = localization_queue.front();
        localization_queue.pop();
        lock.unlock();

        QString command = msg.command;
        if(command == "semiautoinit")
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx0602]not loaded map";
                send_localization_response(msg);
                return;
            }

            QString loc_mode = config->get_loc_mode();
            if(loc_mode == "2D")
            {
                if(!lidar_2d->get_is_connected())
                {
                    msg.result = "reject";
                    msg.message = "[R0Lx0601]not connected lidar";
                    send_localization_response(msg);
                    return;

                }
            }
            else if(loc_mode == "3D")
            {
                if(!lidar_3d->get_is_connected())
                {
                    msg.result = "reject";
                    msg.message = "[R0Lx0601]not connected lidar";
                    send_localization_response(msg);

                    return;
                }
            }
            else
            {
                msg.result = "reject";
                msg.message = "[R0Mx0602] invalid lidar cnt";
                send_localization_response(msg);
                return;
            }

            if(loc->get_is_busy())
            {
                msg.result = "reject";
                msg.message = "[R0Rx0600]already running";
                send_localization_response(msg);
                return;
            }

            // do process
            logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);

            msg.result = "accept";
            msg.message = "";
            send_localization_response(msg);

            if(loc)
            {
                //                qDebug()<<"stop!!!!!";
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
                //                qDebug()<<"start!!!!!";
                semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
            }
        }
        else if(command == "init")
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = "[R0Mx0702]not loaded map";
                send_localization_response(msg);
                continue;
            }

            QString loc_mode = config->get_loc_mode();
            if(loc_mode == "2D")
            {
                if(!lidar_2d->get_is_connected())
                {
                    msg.result = "reject";
                    msg.message = "[R0Lx0601]not connected lidar";
                    send_localization_response(msg);
                    return;
                }
            }
            else if(loc_mode == "3D")
            {
                if(!lidar_3d->get_is_connected())
                {
                    msg.result = "reject";
                    msg.message = "[R0Lx0601]not connected lidar";
                    send_localization_response(msg);
                    return;
                }
            }
            else
            {
                msg.result = "reject";
                msg.message = "[R0Mx0602] invalid lidar cnt";
                send_localization_response(msg);
                return;
            }

            msg.result = "accept";
            msg.message = "";
            send_localization_response(msg);

            // manual init
            double x    = msg.tgt_pose_vec[0];
            double y    = msg.tgt_pose_vec[1];
            double test = msg.tgt_pose_vec[2];
            double rz   = msg.tgt_pose_vec[3];

            loc->stop();
            loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
            logger->write_log(QString("[COMM_MSA] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz*D2R).arg(msg.time), "Green");
        }
        else if(command == "start")
        {
            msg.result = "accept";
            msg.message = "";
            send_localization_response(msg);

            loc->stop();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            double x = msg.tgt_pose_vec[0];
            double y = msg.tgt_pose_vec[1];
            double rz = msg.tgt_pose_vec[3];

            loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
            loc->start();
        }
        else if(command == "stop")
        {
            msg.result = "accept";
            msg.message = "";
            send_localization_response(msg);

            loc->stop();
        }
        else if(command == "randominit")
        {
            if(is_main_window_valid())
            {
                msg.result = "reject";
                msg.message = "randominit 기능을 지원하지 않습니다.";
                send_localization_response(msg);
                //                msg.result = "accept";
                //                msg.message = "";

                //                QString seed = msg.seed;
                //                MainWindow* _main = (MainWindow*)main;
                //                QMetaObject::invokeMethod(_main, "slot_sim_random_init", Qt::QueuedConnection, Q_ARG(QString, seed));
            }
            else
            {
                msg.result = "reject";
                msg.message = "mainwindow module not available";
                send_localization_response(msg);

                logger->write_log("[COMM_MSA] MainWindow not available", "Red");
            }
        }
        else
        {
            msg.result = "reject";
            msg.message = "알 수 없는 command 입니다.";
            send_localization_response(msg);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::control_loop()
{
    while(is_control_running)
    {
        std::unique_lock<std::mutex> lock(control_mtx);
        control_cv.wait(lock, [this]
        {
            return !control_queue.empty() || !is_control_running;
        });

        if(!is_control_running)
        {
            break;
        }

        if(control_queue.size() == 0)
        {
            continue;
        }

        DATA_CONTROL msg = control_queue.front();
        control_queue.pop();
        lock.unlock();

        QString command = msg.command;
        if(command == DATA_CONTROL::Dock)
        {
            if(is_main_window_valid())
            {
                msg.result = "accept";
                msg.message = "";

                MainWindow* _main = (MainWindow*)main;
                QMetaObject::invokeMethod(_main, "bt_DockStart", Qt::QueuedConnection);
            }
            else
            {
                msg.result = "reject";
                msg.message = "mainwindow module not available";

                logger->write_log("[COMM_MSA] MainWindow not available", "Red");
            }
        }
        else if(command == DATA_CONTROL::Undock)
        {
            if(is_main_window_valid())
            {
                msg.result = "accept";
                msg.message = "";

                MainWindow* _main = (MainWindow*)main;
                QMetaObject::invokeMethod(_main, "bt_UnDockStart", Qt::QueuedConnection);
            }
            else
            {
                msg.result = "reject";
                msg.message = "mainwindow module not available";

                logger->write_log("[COMM_MSA] MainWindow not available", "Red");
            }
        }
        else if(command == DATA_CONTROL::RandomSeq)
        {
            if(is_main_window_valid())
            {
                msg.result = "accept";
                msg.message = "";

                MainWindow* _main = (MainWindow*)main;
                QMetaObject::invokeMethod(_main, "slot_sim_random_seq", Qt::QueuedConnection);
            }
            else
            {
                msg.result = "reject";
                msg.message = "mainwindow module not available";

                logger->write_log("[COMM_MSA] MainWindow not available for mapping", "Red");
            }
        }
        else if(command == DATA_CONTROL::LedControl)
        {
            MainWindow* _main = (MainWindow*)main;

            if(msg.onoff){
                _main->is_user_led = true;
                QString led = msg.color;
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

            }else{
                MainWindow* _main = (MainWindow*)main;
                _main->is_user_led = false;

                msg.result = "accept";
                msg.message = "";
            }
        }
        else if(command == DATA_CONTROL::LidarOnOff)
        {
            msg.result = "accept";
            msg.message = "";
            if(msg.onoff)
            {
                if(msg.frequency > 0)
                {
                    MainWindow* _main = (MainWindow*)main;
                    //_main->lidar_view_frequency = msg.frequency;
                }
            }
            else
            {
                MainWindow* _main = (MainWindow*)main;
                //_main->lidar_view_frequency = -1;
            }
        }
        else if(command == DATA_CONTROL::PathOnOff)
        {
            msg.result = "accept";
            msg.message = "";
            if(msg.onoff)
            {
                if(msg.frequency > 0)
                {
                    MainWindow* _main = (MainWindow*)main;
                    //_main->path_view_frequency = msg.frequency;
                }
            }
            else if(command == "off")
            {
                MainWindow* _main = (MainWindow*)main;
                //_main->path_view_frequency = -1;
            }
        }
        else if(command == DATA_CONTROL::MotorOnOff)
        {
            if(msg.onoff)
            {
                mobile->motor_on();

                msg.result = "accept";
                msg.message = "";
            }
            else
            {
                mobile->motor_off();

                msg.result = "accept";
                msg.message = "";
            }
        }
        else if(command == DATA_CONTROL::SetSafetyField)
        {
            msg.result = "success";
            unsigned int set_field_ = msg.safetyField.toInt();
            msg.message = "";

            if(mobile)
            {
                MOBILE::instance()->setlidarfield(set_field_);
            }
        }
        else if(command == DATA_CONTROL::GetSafetyField)
        {
            msg.result = "success";
            msg.message = "";

            if(mobile)
            {
                MOBILE_STATUS ms = MOBILE::instance()->get_status();
                msg.safetyField = ms.lidar_field;
            }
        }
        else if(command == DATA_CONTROL::ResetSafetyField)
        {
            if(msg.resetField == "bumper")
            {
                msg.result = "success";
                msg.message = "";

                if(mobile)
                {
                    mobile->clearbumperstop();
                }
            }

            else if (msg.resetField == "interlock")
            {
                msg.result = "success";
                msg.message = "";

                if(mobile)
                {
                    mobile->clearinterlockstop();
                }
            }

            else if (msg.resetField == "obstacle")
            {
                msg.result = "success";
                msg.message = "";

                if(mobile)
                {
                    mobile->clearobs();
                }
            }

            else if (msg.resetField == "operationStop")
            {
                msg.result = "success";
                msg.message = "";

                if(mobile)
                {
                    mobile->recover();
                }

            }else{
                msg.result = "reject";
                msg.message = "resetField 값이 잘못되었습니다.";
            }
        }
        else
        {
            msg.result = "reject";
            msg.message = "command 값이 잘못되었습니다.";
        }

        send_control_response(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


void COMM_MSA::common_loop()
{
    while(is_common_running)
    {
        std::unique_lock<std::mutex> lock(common_mtx);
        common_cv.wait(lock, [this]
        {
            return !common_queue.empty() || !is_common_running;
        });

        if(!is_common_running)
        {
            break;
        }

        if(common_queue.size() == 0)
        {
            continue;
        }

        DATA_COMMON cmsg = common_queue.front();
        common_queue.pop();
        lock.unlock();

        auto cmd = cmsg.type;
        if(cmd == DATA_COMMON::TYPE::LOAD)
        {
            DATA_LOAD msg = cmsg.dload;
            QString command = msg.command;
            if(command == "mapload")
            {
                handle_common_load_map(msg);
            }
            else if(command == "topoload")
            {
                handle_common_load_topo(msg);
            }
            else if(command == "configload")
            {
                handle_common_load_config(msg);
            }
        }
        else if(cmd == DATA_COMMON::TYPE::RANDOMSEQ)
        {
            DATA_RANDOMSEQ msg = cmsg.drandomseq;
            QString command = msg.command;
            if(command == "randomseq")
            {
                if(is_main_window_valid())
                {
                    msg.result = "accept";
                    msg.message = "";

                    MainWindow* _main = (MainWindow*)main;
                    QMetaObject::invokeMethod(_main, "slot_sim_random_seq", Qt::QueuedConnection);
                }
                else
                {
                    msg.result = "reject";
                    msg.message = "mainwindow module not available";

                    logger->write_log("[COMM_MSA] MainWindow not available for mapping", "Red");
                }
            }
        }
        else if(cmd == DATA_COMMON::TYPE::DOCKING)
        {
            DATA_DOCK msg = cmsg.ddock;
            QString command = msg.command;
            if(command == "dock")
            {
                if(is_main_window_valid())
                {
                    msg.result = "accept";
                    msg.message = "";

                    MainWindow* _main = (MainWindow*)main;
                    QMetaObject::invokeMethod(_main, "bt_DockStart", Qt::QueuedConnection);
                }
                else
                {
                    msg.result = "reject";
                    msg.message = "mainwindow module not available";

                    logger->write_log("[COMM_MSA] MainWindow not available", "Red");
                }
            }
            else if(command == "undock")
            {
                if(is_main_window_valid())
                {
                    msg.result = "accept";
                    msg.message = "";

                    MainWindow* _main = (MainWindow*)main;
                    QMetaObject::invokeMethod(_main, "bt_UnDockStart", Qt::QueuedConnection);
                }
                else
                {
                    msg.result = "reject";
                    msg.message = "mainwindow module not available";

                    logger->write_log("[COMM_MSA] MainWindow not available", "Red");
                }
            }
        }
//        else if(cmd == DATA_COMMON::TYPE::LOCALIZATION)
//        {
//            DATA_LOCALIZATION msg = cmsg.dlocalization;
//            QString command = msg.command;
//            if(command == "semiautoinit")
//            {
//                if(unimap->get_is_loaded() != MAP_LOADED)
//                {
//                    msg.result = "reject";
//                    msg.message = "[R0Mx0602]not loaded map";

//                    return;
//                }

//                QString loc_mode = config->get_loc_mode();
//                if(loc_mode == "2D")
//                {
//                    if(!lidar_2d->get_is_connected())
//                    {
//                        msg.result = "reject";
//                        msg.message = "[R0Lx0601]not connected lidar";
//                        return;
//                    }
//                }
//                else if(loc_mode == "3D")
//                {
//                    if(!lidar_3d->get_is_connected())
//                    {
//                        msg.result = "reject";
//                        msg.message = "[R0Lx0601]not connected lidar";
//                        return;
//                    }
//                }
//                else
//                {
//                    msg.result = "reject";
//                    msg.message = "[R0Mx0602] invalid lidar cnt";
//                    return;
//                }

//                if(loc->get_is_busy())
//                {
//                    msg.result = "reject";
//                    msg.message = "[R0Rx0600]already running";
//                    return;
//                }

//                // do process
//                logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);

//                msg.result = "accept";
//                msg.message = "";

//                loc->stop();

//                // semi auto init
//                if(semi_auto_init_thread != nullptr)
//                {
//                    if(semi_auto_init_thread->joinable())
//                    {
//                        semi_auto_init_thread->join();
//                        semi_auto_init_thread = nullptr;
//                        logger->write_log("[AUTO_INIT] recv_loc, semiauto init thread already running.", "Orange", true, false);
//                    }
//                    else
//                    {
//                        logger->write_log("[AUTO_INIT] recv_loc, start semiauto init thread.", "Green", true, false);
//                    }
//                }

//                semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
//            }
//            else if(command == "init")
//            {
//                if(unimap->get_is_loaded() != MAP_LOADED)
//                {
//                    msg.result = "reject";
//                    msg.message = "[R0Mx0702]not loaded map";

//                    continue;
//                }

//                QString loc_mode = config->get_loc_mode();
//                if(loc_mode == "2D")
//                {
//                    if(!lidar_2d->get_is_connected())
//                    {
//                        msg.result = "reject";
//                        msg.message = "[R0Lx0601]not connected lidar";
//                        return;
//                    }
//                }
//                else if(loc_mode == "3D")
//                {
//                    if(!lidar_3d->get_is_connected())
//                    {
//                        msg.result = "reject";
//                        msg.message = "[R0Lx0601]not connected lidar";
//                        return;
//                    }
//                }
//                else
//                {
//                    msg.result = "reject";
//                    msg.message = "[R0Mx0602] invalid lidar cnt";
//                    return;
//                }

//                msg.result = "accept";
//                msg.message = "";

//                // manual init
//                double x    = msg.tgt_pose_vec[0];
//                double y    = msg.tgt_pose_vec[1];
//                double test = msg.tgt_pose_vec[2];
//                double rz   = msg.tgt_pose_vec[3];

//                loc->stop();
//                loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
//                logger->write_log(QString("[COMM_MSA] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz*D2R).arg(msg.time), "Green");
//            }
//            else if(command == "start")
//            {
//                msg.result = "accept";
//                msg.message = "";

//                loc->stop();
//                std::this_thread::sleep_for(std::chrono::milliseconds(10));

//                double x = msg.tgt_pose_vec[0];
//                double y = msg.tgt_pose_vec[1];
//                double rz = msg.tgt_pose_vec[3];

//                qDebug()<<"sssssssssssssssssssssssssssssssss";
//                qDebug().nospace() << "x=" << x << ", y=" << y << ", rz=" << rz;
//                loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
//                loc->start();
//            }
//            else if(command == "stop")
//            {
//                msg.result = "accept";
//                msg.message = "";

//                loc->stop();
//            }
//            else if(command == "randominit")
//            {
//                if(is_main_window_valid())
//                {
//                    msg.result = "accept";
//                    msg.message = "";

//                    QString seed = msg.seed;
//                    MainWindow* _main = (MainWindow*)main;
//                    QMetaObject::invokeMethod(_main, "slot_sim_random_init", Qt::QueuedConnection, Q_ARG(QString, seed));
//                }
//                else
//                {
//                    msg.result = "reject";
//                    msg.message = "mainwindow module not available";

//                    logger->write_log("[COMM_MSA] MainWindow not available", "Red");
//                }
//            }
//        }
        else if(cmd == DATA_COMMON::TYPE::VIEW_LIDAR)
        {
            DATA_VIEW_LIDAR msg = cmsg.dlidar;
            QString command = msg.command;
            if(command == "on")
            {
                if(msg.frequency > 0)
                {
                    MainWindow* _main = (MainWindow*)main;
                    //_main->lidar_view_frequency = msg.frequency;
                }
            }
            else if(command == "off")
            {
                MainWindow* _main = (MainWindow*)main;
                //_main->lidar_view_frequency = -1;
            }
        }
        else if(cmd == DATA_COMMON::TYPE::VIEW_PATH)
        {
            DATA_VIEW_PATH msg = cmsg.dpath;
            QString command = msg.command;
            if(command == "on")
            {
                if(msg.frequency > 0)
                {
                    MainWindow* _main = (MainWindow*)main;
                    //_main->path_view_frequency = msg.frequency;
                }
            }
            else if(command == "off")
            {
                MainWindow* _main = (MainWindow*)main;
                //_main->path_view_frequency = -1;
            }
        }
        else if(cmd == DATA_COMMON::TYPE::LED)
        {
            DATA_LED msg = cmsg.dled;
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
        else if(cmd == DATA_COMMON::TYPE::MOTOR)
        {
            DATA_MOTOR msg = cmsg.dmotor;
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

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_MSA::send_local_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }


    PATH path = ctrl->get_cur_local_path();

    ///////////////////////////////////////////
    // compare with old path -> 2025.09.02 add!!
    if (path.pos.size() == last_sent_path.pos.size())
    {
        bool is_same = true;
        for (size_t i = 0; i < path.pos.size(); ++i)
        {
            if (!path.pos[i].isApprox(last_sent_path.pos[i], 1e-6)) // Eigen Vector3d compare
            {
                is_same = false;
                break;
            }
        }
        if (is_same)
        {
            return;
        }
    }
    last_sent_path = path;

    ///////////////////////////////////////////
    /// \brief jsonArray
    sio::object_message::ptr send_object = sio::object_message::create();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(size_t p = 0; p < path.pos.size(); p++)
    {
        if(p == 0 || p == path.pos.size() - 1 || p % 10 == 0)
        {
            const Eigen::Vector3d P = path.pos[p];

            sio::array_message::ptr jsonObj = sio::array_message::create();
            jsonObj->get_vector().push_back(sio::double_message::create(P[0]));
            jsonObj->get_vector().push_back(sio::double_message::create(P[1]));
            jsonObj->get_vector().push_back(sio::double_message::create(P[2]));
            jsonArray->get_vector().push_back(jsonObj);
        }
    }
    send_object->get_map()["path"] =   jsonArray;
    const double time = get_time0();
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(time*1000), 10).toStdString());

    // send
    //    io->socket()->emit("localPath", send_object);
    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "localPath";
    socket_msg.data = send_object;  // 타입 그대로 전달

    send_queue.push(socket_msg);
}

void COMM_MSA::handle_mapping(DATA_MAPPING msg)
{
    const QString command = msg.command;
    if(command == "mappingStart")
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
                //                qDebug()<<"bt_map build";
                //                _main->bt_MapBuild();

                QMetaObject::invokeMethod(_main, "bt_MapBuild", Qt::QueuedConnection);

            }
        }
        else
        {
            msg.result = "reject";
            msg.message = "[R0Lx0800]lidar not connected";

            send_mapping_response(msg);
        }
    }
    else if(command == "mappingStop")
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
    else if(command == "mappingSave")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main -> map_dir = "";
            if(msg.map_name != "")
            {
                _main ->change_map_name = true;
                _main -> map_dir =  msg.map_name;
            }

            _main->bt_MapSave();

            const QString map_name = msg.map_name;
            const QString save_dir = "/data/maps/" + map_name;

            bool found_csv = false;

            if (std::filesystem::exists(save_dir.toStdString()) && std::filesystem::is_directory(save_dir.toStdString()))
            {
                for (const auto& entry : std::filesystem::directory_iterator(save_dir.toStdString()))
                {
                    if (entry.is_regular_file() && entry.path().extension() == ".csv")
                    {
                        found_csv = true;
                        break;
                    }
                }
            }
            if (found_csv)
            {
                msg.result = "success";
                msg.message = "";

                send_mapping_response(msg);
            }
            else
            {
                msg.result = "fail";
                msg.message = "[R1Cx2100] csv file not found in map directory";

                send_mapping_response(msg);
            }

        }
    }
    else if(command == "mappingReload")
    {
        msg.result = "accept";
        msg.message = "";

        send_mapping_response(msg);

        last_send_kfrm_idx = 0;
    }
}


//void COMM_MSA::send_localization_response(const DATA_LOCALIZATION& msg)
//{
//    if(!is_connected)
//    {
//        return;
//    }

//    QJsonObject obj;
//    obj["command"] = msg.command;
//    obj["result"] = msg.result;
//    obj["message"] = msg.message;
//    obj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
//    obj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
//    obj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
//    obj["rz"] = QString::number(msg.tgt_pose_vec[3], 'f', 3);
//    obj["seed"] = msg.seed;
//    obj["time"] = QString::number((long long)(msg.time*1000), 10);

//    QJsonDocument doc(obj);
//    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
//    io->socket()->emit("localizationResponse", res);

//    // for plot
//    mtx.lock();
//    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
//    mtx.unlock();
//}

void COMM_MSA::send_global_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }
    PATH path = ctrl->get_cur_global_path();

    sio::object_message::ptr send_object = sio::object_message::create();
    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(const Eigen::Vector3d& P : path.pos)
    {
        sio::array_message::ptr jsonObj = sio::array_message::create();
        jsonObj->get_vector().push_back(sio::double_message::create(P[0]));
        jsonObj->get_vector().push_back(sio::double_message::create(P[1]));
        jsonObj->get_vector().push_back(sio::double_message::create(P[2]));
        jsonArray->get_vector().push_back(jsonObj);
    }

    send_object->get_map()["path"] =   jsonArray;
    const double time = get_time0();
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(time*1000), 10).toStdString());

    // send
    //    io->socket()->emit("globalPath", send_object);
    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "globalPath";
    socket_msg.data = send_object;

    send_queue.push(socket_msg);
}


void COMM_MSA::send_lidar_2d()
{
    if (!is_connected || !loc || !lidar_2d)
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
        poseObject->get_map()["x"] = sio::double_message::create(cur_xi[0]);
        poseObject->get_map()["y"] = sio::double_message::create(cur_xi[1]);
        poseObject->get_map()["rz"] = sio::double_message::create(cur_xi[2]*R2D);

        rootObject->get_map()["pose"] = poseObject;

        sio::array_message::ptr jsonArray = sio::array_message::create();

        // initalize vector, get pts -> 1 deg
        std::vector<Eigen::Vector3d> sample_pts(360, Eigen::Vector3d(NAN,NAN,NAN));

        for(size_t p=0; p<pts.size(); p++)
        {
            double yaw_rad = std::atan2(pts[p][1], pts[p][0]);
            double yaw_deg = yaw_rad * R2D;
            if(yaw_deg < 0)
            {
                yaw_deg += 360.0;
            }

            int idx = static_cast<int>(yaw_deg) % 360;
            double dist = std::sqrt(pts[p][0]*pts[p][0] + pts[p][1]*pts[p][1]);

            // if multiple values with similar angles appear, update the one with the shorter distance.
            if(std::isnan(sample_pts[idx][0]) || dist < std::sqrt(sample_pts[idx][0]*sample_pts[idx][0] + sample_pts[idx][1]*sample_pts[idx][1]))
            {
                sample_pts[idx] = Eigen::Vector3d(pts[p][0], pts[p][1], pts[p][2]);
            }
        }

        for(int i=0; i<360; i++)
        {
            // fill missing sample point with previous point
            if(std::isnan(sample_pts[i][0]))
            {
                int prev = (i-1+360)%360;
                sample_pts[i] = sample_pts[prev];  // add pre pts
            }

            sio::array_message::ptr jsonObj = sio::array_message::create();
            if(!std::isnan(sample_pts[i][0]))
            {
                jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][0]));
                jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][1]));
                jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][2]));
            }
            //            jsonObj->get_vector().push_back(sio::double_message::create(100));
            jsonArray->get_vector().push_back(jsonObj);
        }

        //                std::cout << jsonArray->get_vector().size() << std::endl;
        // Adding the time object
        const double time = get_time0();
        rootObject->get_map()["time"] = sio::double_message::create(static_cast<long long>(time * 1000));
        rootObject->get_map()["data"] = jsonArray;
        io->socket("slamnav")->emit("lidarCloud", rootObject);
    }
}


void COMM_MSA::send_lidar_3d()
{
    if (!is_connected || !loc || !lidar_2d || !lidar_3d)
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

            jsonObj->get_vector().push_back(sio::double_message::create(pts[p][0]));
            jsonObj->get_vector().push_back(sio::double_message::create(pts[p][1]));
            jsonObj->get_vector().push_back(sio::double_message::create(pts[p][2]));
            jsonObj->get_vector().push_back(sio::double_message::create(100));

            jsonArray->get_vector().push_back(jsonObj);
        }

        const double time = get_time0();
        rootObject->get_map()["time"] = sio::double_message::create(static_cast<long long>(time * 1000));
        rootObject->get_map()["data"] = jsonArray;
    }
}

void COMM_MSA::send_mapping_cloud()
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

            jsonObj->get_vector().push_back(sio::double_message::create(_P[0]));
            jsonObj->get_vector().push_back(sio::double_message::create(_P[1]));
            jsonObj->get_vector().push_back(sio::double_message::create(_P[2]));
            jsonObj->get_vector().push_back(sio::double_message::create(pt.r));

            jsonArray->get_vector().push_back(jsonObj);
        }

        sio::object_message::ptr rootObj = sio::object_message::create();
        rootObj->get_map()["data"]  = jsonArray;

        // Adding the time object
        const double time = get_time0();
        rootObj->get_map()["time"] = sio::double_message::create(static_cast<long long>(time * 1000));

        // send
        io->socket()->emit("mappingCloud", rootObj);
        last_send_kfrm_idx++;
    }
}

void COMM_MSA::response_loop()
{
    while(is_response_running)
    {
        std::unique_lock<std::mutex> lock(response_mtx);
        response_cv.wait(lock, [this]
        {
            return !response_queue.empty() || !is_response_running;
        });

        if(!is_response_running)
        {
            break;
        }

        if(response_queue.size() == 0)
        {
            continue;
        }

        while(!response_queue.empty())
        {
            auto response_func = response_queue.front();
            response_queue.pop();
            lock.unlock();

            response_func();

            lock.lock();
        }
    }
}

void COMM_MSA::send_path_response(DATA_PATH msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    sio::object_message::ptr robotObj = sio::object_message::create();
    robotObj->get_map()["robotSerial"] = sio::string_message::create(robot_id.toStdString());

    sio::object_message::ptr dataObj = sio::object_message::create();
    dataObj->get_map()["command"] = sio::string_message::create("path");
    dataObj->get_map()["time"]    = sio::double_message::create(msg.time);

    send_object->get_map()["robot"] = robotObj;
    send_object->get_map()["data"]  = dataObj;

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "pathResponse";
    socket_msg.data  = send_object;

    send_queue.push(socket_msg);
}

void COMM_MSA::send_status_loop()
{
    while(is_send_status_running)
    {
        //        std::unique_lock<std::mutex> lock(status_mtx);
        //        status_cv.wait(lock, [this]
        //        {
        //            return !common_queue.empty() || !is_send_status_running;
        //        });

        if(!is_send_status_running)
        {
            break;
        }

        // Synchronize with the development version
        // 100[ms]
        if(send_cnt % COMM_MSA_INFO::send_move_status_cnt == 0)
        {
            //            send_move_status();
            send_move_status();
        }

        // 500[ms]
        if(send_cnt % COMM_MSA_INFO::send_status_cnt == 0)
        {
            send_status();
            //            send_system_status(cpu_use, cpu_temp);
            //            send_mapping_cloud();
        }

        // for variable loop
        double time_lidar_view = 1.0/((double)lidar_view_frequency + 1e-06) * 10.0;
        if(time_lidar_view > 0)
        {
            if(lidar_view_cnt > time_lidar_view)
            {
                lidar_view_cnt = 0;

                if(CONFIG::instance()->get_use_lidar_2d())
                {
                    send_lidar_2d();
                }

                if(CONFIG::instance()->get_use_lidar_3d())
                {
                    send_lidar_3d();
                }
            }
            lidar_view_cnt++;
        }

        double time_path_view = 1.0/((double)path_view_frequency + 1e-06) * 10.0;
        time_path_view *= 10.0;
        if(time_path_view > 0)
        {
            if(path_view_cnt > time_path_view)
            {
                path_view_cnt = 0;
                if(is_local_path_update2)
                {
                    is_local_path_update2 = false;
                    send_local_path();
                }
                if(is_global_path_update2)
                {
                    is_global_path_update2 = false;
                    send_global_path();
                }
            }
            path_view_cnt++;
        }

        //         to give information video streaming data
        // core -> 4cam
        // Electrode -> 2 cam
        //if(config->get_use_rtsp() && config->get_use_cam() || config->get_use_cam_rgb() || config->get_use_cam_depth())
        if(config->get_use_rtsp() && config->get_use_cam())
        {
            if(send_cnt % 100 == 0)
            {
                std::vector<bool> rtsp_flag = cam->get_rtsp_flag();
                if(rtsp_flag.size() != 0)
                {
                    for(int p = 0; p < rtsp_flag.size(); p++)
                    {
                        QString msg = QString("[COMM] cam%1 rtsp writer %2").arg(p)
                                .arg(rtsp_flag[p] ? "open success" : "open failed");
                        logger->write_log(msg);
                    }
                }
            }
        }

        send_cnt++;
        if(send_cnt > 10000)
        {
            send_cnt = 0;
        }

        //        if(common_queue.size() == 0)
        //        {
        //            continue;
        //        }



        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// send functions
void COMM_MSA::send_status()
{
    if(!is_connected || !config || !mobile || !unimap || !dctrl)
    {
        return;
    }

    // Creating the JSON object
    //    QJsonObject rootObj;
    sio::object_message::ptr rootObj = sio::object_message::create();

    MOBILE_STATUS ms = mobile->get_status();

    // Adding the imu object
    Eigen::Vector3d imu = mobile->get_imu();
    sio::object_message::ptr imuObj = sio::object_message::create();
    imuObj->get_map()["acc_x"]  = sio::double_message::create(ms.imu_acc_x);
    imuObj->get_map()["acc_y"]  = sio::double_message::create(ms.imu_acc_y);
    imuObj->get_map()["acc_z"]  = sio::double_message::create(ms.imu_acc_z);
    imuObj->get_map()["gyr_x"]  = sio::double_message::create(ms.imu_gyr_x * R2D);
    imuObj->get_map()["gyr_y"]  = sio::double_message::create(ms.imu_gyr_y * R2D);
    imuObj->get_map()["gyr_z"]  = sio::double_message::create(ms.imu_gyr_z * R2D);
    imuObj->get_map()["imu_rx"] = sio::double_message::create(imu[0] * R2D);
    imuObj->get_map()["imu_ry"] = sio::double_message::create(imu[1] * R2D);
    imuObj->get_map()["imu_rz"] = sio::double_message::create(imu[2] * R2D);

    rootObj->get_map()["imu"] = imuObj;

    // Adding the motor array
    sio::array_message::ptr motorArray = sio::array_message::create();

    sio::object_message::ptr motorObj1 = sio::object_message::create();
    motorObj1->get_map()["connection"] = sio::bool_message::create((ms.connection_m0 == 1) ? "true" : "false");
    motorObj1->get_map()["status"]     = sio::double_message::create(ms.status_m0);
    motorObj1->get_map()["temp"]       = sio::double_message::create(ms.temp_m0);
    motorObj1->get_map()["current"]    = sio::double_message::create(static_cast<double>(ms.cur_m0) / 10.0);
    motorArray->get_vector().push_back(motorObj1);

    sio::object_message::ptr motorObj2 = sio::object_message::create();
    motorObj2->get_map()["connection"] = sio::string_message::create((ms.connection_m1 == 1) ? "true" : "false");
    motorObj2->get_map()["status"]     = sio::string_message::create(QString::number(ms.status_m1).toStdString());
    motorObj2->get_map()["temp"]       = sio::string_message::create(QString::number(ms.temp_m1, 'f', 3).toStdString());
    motorObj2->get_map()["current"]    = sio::string_message::create(QString::number(static_cast<double>(ms.cur_m1) / 10.0, 'f', 3).toStdString());
    motorArray->get_vector().push_back(motorObj2);

    rootObj->get_map()["motor"] = motorArray;
    // Adding the condition object
    Eigen::Vector2d ieir = loc->get_cur_ieir();

    sio::object_message::ptr conditionObj = sio::object_message::create();
    conditionObj->get_map()["inlier_error"]  = sio::double_message::create(ieir[0]);
    conditionObj->get_map()["inlier_ratio"]  = sio::double_message::create(ieir[1]);
    conditionObj->get_map()["mapping_error"] = sio::double_message::create(ieir[0]);
    conditionObj->get_map()["mapping_ratio"] = sio::double_message::create(ieir[1]);

    rootObj->get_map()["condition"] = conditionObj;

    // Adding the state object
    QString cur_loc_state = loc->get_cur_loc_state();
    QString charge_st_string = "none";

    RobotModel robot_model = config->get_robot_model();
    if(robot_model == RobotModel::D400 || robot_model == RobotModel::MECANUM)
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
    else if(robot_model == RobotModel::S100)
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

    sio::object_message::ptr robotStateObj = sio::object_message::create();
    robotStateObj->get_map()["charge"]       = sio::string_message::create(charge_st_string.toStdString());
    robotStateObj->get_map()["dock"]         = sio::bool_message::create((is_dock == true) ? "true" : "false");
    robotStateObj->get_map()["emo"]          = sio::bool_message::create((ms.motor_stop_state == 1) ? "true" : "false");
    robotStateObj->get_map()["localization"] = sio::string_message::create(cur_loc_state.toStdString()); // "none", "good", "fail"
    robotStateObj->get_map()["power"]        = sio::bool_message::create((ms.power_state == 1) ? "true" : "false");
    rootObj->get_map()["robot_state"]        = robotStateObj;

    auto toSioArray = [](unsigned char arr[8]) {
        sio::array_message::ptr jsonArr = sio::array_message::create();
        for (int i = 0; i < 8; ++i)
        {
            jsonArr->get_vector().push_back(sio::int_message::create(arr[i]));
        }
        return jsonArr;
    };

    //    QJsonObject robotIOObj;
    sio::object_message::ptr robotIOObj = sio::object_message::create();
    robotIOObj->get_map()["mcu0_dio"] = toSioArray(ms.mcu0_dio);
    robotIOObj->get_map()["mcu1_dio"] = toSioArray(ms.mcu1_dio);
    robotIOObj->get_map()["mcu0_din"] = toSioArray(ms.mcu0_din);
    robotIOObj->get_map()["mcu1_din"] = toSioArray(ms.mcu1_din);
    rootObj->get_map()["robot_safety_io_state"]  = robotIOObj;

    // Adding the power object
    sio::object_message::ptr powerObj = sio::object_message::create();
    powerObj->get_map()["bat_in"]         = sio::double_message::create(ms.bat_in);
    powerObj->get_map()["bat_out"]        = sio::double_message::create(ms.bat_out);
    powerObj->get_map()["bat_current"]    = sio::double_message::create(ms.bat_current);
    powerObj->get_map()["total_power"]    = sio::double_message::create(ms.total_power);
    powerObj->get_map()["power"]          = sio::double_message::create(ms.power);
    powerObj->get_map()["bat_percent"]    = sio::double_message::create(ms.bat_percent);
    powerObj->get_map()["tabos_voltage"]  = sio::double_message::create(ms.tabos_voltage);
    powerObj->get_map()["tabos_current"]  = sio::double_message::create(ms.tabos_current);
    powerObj->get_map()["tabos_status"]   = sio::double_message::create(ms.tabos_status);
    powerObj->get_map()["tabos_ttf"]      = sio::double_message::create(ms.tabos_ttf);
    powerObj->get_map()["tabos_tte"]      = sio::double_message::create(ms.tabos_tte);
    powerObj->get_map()["tabos_soc"]      = sio::double_message::create(ms.tabos_soc);
    powerObj->get_map()["tabos_soh"]      = sio::double_message::create(ms.tabos_soh);
    powerObj->get_map()["tabos_temp"]     = sio::double_message::create(ms.tabos_temperature);
    powerObj->get_map()["tabos_rc"]       = sio::double_message::create(ms.tabos_rc);
    powerObj->get_map()["tabos_ae"]       = sio::double_message::create(ms.tabos_ae);

    if(robot_model == RobotModel::D400 || robot_model == RobotModel::MECANUM)
    {
        powerObj->get_map()["charge_current"]  = sio::double_message::create(ms.charge_current);
        powerObj->get_map()["contact_voltage"] = sio::double_message::create(ms.contact_voltage);
    }
    else if(robot_model == RobotModel::S100)
    {
        powerObj->get_map()["charge_current"]  = sio::double_message::create(0.0);
        powerObj->get_map()["contact_voltage"] = sio::double_message::create(0.0);
    }
    rootObj->get_map()["power"] = powerObj;

    sio::object_message::ptr settingObj = sio::object_message::create();
    settingObj->get_map()["platform_type"] = sio::string_message::create(config->get_robot_type_str().toStdString());
    settingObj->get_map()["platform_name"] = sio::string_message::create("");
    rootObj->get_map()["setting"] = settingObj;

    sio::object_message::ptr mapObj = sio::object_message::create();
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

    mapObj->get_map()["map_name"]   =  sio::string_message::create(map_name.toStdString());
    mapObj->get_map()["map_status"] =  sio::string_message::create(map_status.toStdString());
    rootObj->get_map()["map"] = mapObj;


    // usb temp sensor
    sio::object_message::ptr tempObj = sio::object_message::create();
    tempObj->get_map()["connection"] = sio::double_message::create(ms.bat_in);
    tempObj->get_map()["temp_sensor"] = sio::double_message::create(ms.bat_out);

    // Adding the time object
    const double time = get_time0();
    rootObj->get_map()["time"] = sio::double_message::create(static_cast<long long>(time * 1000));

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "status";
    socket_msg.data = rootObj;  // 타입 그대로 전달

    send_queue.push(socket_msg);
}

void COMM_MSA::send_move_response(DATA_MOVE msg)
{
    if(!is_connected)
    {
        //        qDebug()<<"is not connected!!!!!!1";
        return;
    }

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

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["id"]         = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]    = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]     = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]    = sio::string_message::create(msg.message.toStdString());
    send_object->get_map()["method"]     = sio::string_message::create(msg.method.toStdString());
    send_object->get_map()["goalId"]     = sio::string_message::create(msg.goal_node_id.toStdString());
    //    send_object->get_map()["goalType"] = sio::string_message::create(msg.goal_node_type.toStdString());
    send_object->get_map()["goalName"]   = sio::string_message::create(response_goal_node_name.toStdString());


    send_object->get_map()["preset"]     = sio::int_message::create(msg.preset);
    send_object->get_map()["remaining_dist"] = sio::double_message::create(msg.remaining_dist);
    //    send_object->get_map()["eta"] = sio::double_message::create(msg.eta);
    send_object->get_map()["bat_percent"] = sio::int_message::create(msg.bat_percent);

    send_object->get_map()["cur_x"]  = sio::double_message::create(msg.cur_pos[0]);
    send_object->get_map()["cur_y"]  = sio::double_message::create(msg.cur_pos[1]);
    send_object->get_map()["cur_z"]  = sio::double_message::create(msg.cur_pos[2]);
    send_object->get_map()["x"]      = sio::double_message::create(msg.tgt_pose_vec[0]);
    send_object->get_map()["y"]      = sio::double_message::create(msg.tgt_pose_vec[1]);
    send_object->get_map()["z"]      = sio::double_message::create(msg.tgt_pose_vec[2]);
    send_object->get_map()["rz"]     = sio::double_message::create(msg.tgt_pose_vec[3]*R2D);
    send_object->get_map()["vx"]     = sio::double_message::create(msg.jog_val[0]);
    send_object->get_map()["vy"]     = sio::double_message::create(msg.jog_val[1]);
    send_object->get_map()["wz"]     = sio::double_message::create(msg.jog_val[2]);
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    //    SOCKET_MESSAGE socket_msg = {"moveResponse", buf};
    //    send_queue.push(socket_msg);

    //    qDebug() << "Move Response : " << msg.result;
    io->socket("slamnav")->emit("moveResponse", send_object);
}

void COMM_MSA::send_control_response(DATA_CONTROL msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["id"]         = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]    = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]     = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]    = sio::string_message::create(msg.message.toStdString());

    //    if(msg.command == DATA_CONTROL::Dock)
    //    {
    //    }
    //    else if(msg.command == DATA_CONTROL::Undock)
    //    {
    //    }
    //    else if(msg.command == DATA_CONTROL::RandomSeq)
    //    {
    //    }
    if(msg.command == DATA_CONTROL::Dock || msg.command == DATA_CONTROL::Undock || msg.command == DATA_CONTROL::RandomSeq)
    {
    }
    else if(msg.command == DATA_CONTROL::LedControl)
    {
        send_object->get_map()["onoff"] = sio::bool_message::create(msg.onoff);
        if(msg.onoff)
        {
            send_object->get_map()["color"]    = sio::string_message::create(msg.color.toStdString());
        }
    }
    else if(msg.command == DATA_CONTROL::LidarOnOff)
    {
        send_object->get_map()["onoff"] = sio::bool_message::create(msg.onoff);
        send_object->get_map()["frequency"] = sio::double_message::create(msg.frequency);
    }
    else if(msg.command == DATA_CONTROL::PathOnOff)
    {
        send_object->get_map()["onoff"] = sio::bool_message::create(msg.onoff);
        send_object->get_map()["frequency"] = sio::double_message::create(msg.frequency);
    }
    else if(msg.command == DATA_CONTROL::MotorOnOff)
    {
        send_object->get_map()["onoff"] = sio::bool_message::create(msg.onoff);
    }
    else if(msg.command == DATA_CONTROL::SetSafetyField)
    {
        send_object->get_map()["safetyField"]    = sio::string_message::create(msg.safetyField.toStdString());
    }
    else if(msg.command == DATA_CONTROL::GetSafetyField)
    {
        send_object->get_map()["safetyField"]    = sio::string_message::create(msg.safetyField.toStdString());
    }
    else if(msg.command == DATA_CONTROL::ResetSafetyField)
    {
        send_object->get_map()["resetField"]    = sio::string_message::create(msg.resetField.toStdString());
    }
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    io->socket("slamnav")->emit("controlResponse", send_object);

}

void COMM_MSA::send_localization_response(DATA_LOCALIZATION msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["id"]         = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]    = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]     = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]    = sio::string_message::create(msg.message.toStdString());

    if(msg.command == "init")
    {
        send_object->get_map()["x"] = sio::double_message::create(msg.tgt_pose_vec[0]);
        send_object->get_map()["y"] = sio::double_message::create(msg.tgt_pose_vec[1]);
        send_object->get_map()["z"] = sio::double_message::create(msg.tgt_pose_vec[2]);
        send_object->get_map()["th"] = sio::double_message::create(msg.tgt_pose_vec[3]);
    }
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    //    qDebug() << "localizationResponse " << msg.result;
    io->socket("slamnav")->emit("localizationResponse", send_object);
}

void COMM_MSA::send_load_response(DATA_LOAD msg)
{
    if(!is_connected)
    {
        //        qDebug()<<"is not connected!!!!!!1";
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["id"]         = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]    = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]     = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]    = sio::string_message::create(msg.message.toStdString());

    send_object->get_map()["mapName"]   = sio::string_message::create(msg.map_name.toStdString());

    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    io->socket("slamnav")->emit("loadResponse", send_object);
}

void COMM_MSA::send_mapping_response(DATA_MAPPING msg)
{
    if(!is_connected)
    {
        //        qDebug()<<"is not connected!!!!!!1";
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["id"]         = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]    = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]     = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]    = sio::string_message::create(msg.message.toStdString());

    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    io->socket("slamnav")->emit("mappingResponse", send_object);
}

//void COMM_MSA::send_ping_response(DATA_MOVE msg)
//{
//    if(!is_connected)
//    {
//        return;
//    }

//    QProcess process;
//    QString program = "ping";
//    QStringList arguments = { "-c", "1", host };   // Linux/macOS: ping -c 1 host

//    process.start(program, arguments);
//    process.waitForFinished();

//    QString output = process.readAllStandardOutput();
//    qDebug() << output;

//    if (process.exitCode() == 0)
//    {
//        qDebug() << "Ping success!";
//    }
//    else
//    {
//        qDebug() << "Ping failed!";
//    }

//    QJsonObject robotObj;
//    robotObj["robotSerial"] = robot_id;

//    QJsonObject rootObj;
//    rootObj["robot"] = robotObj;
//    rootObj["data"] = dataObj;

//    QJsonDocument doc(rootObj);
//    QString buf = doc.toJson(QJsonDocument::Compact);
//    send_queue.push(buf);
//}

void COMM_MSA::send_loop()
{
    if(!is_connected){
        return;
    }

    SOCKET_MESSAGE msg;
    if (send_queue.try_pop(msg))
    {
        std::lock_guard<std::mutex> lock(send_mtx);
        if(msg.event.toStdString() == "localizationResponse"){
            qDebug() << "SEND LOOP";
        }
        io->socket("slamnav")->emit(msg.event.toStdString(), msg.data);
    }
}

void COMM_MSA::handle_move_jog(const DATA_MOVE& msg)
{
    double vx = msg.jog_val[0];
    double vy = msg.jog_val[1];
    double wz = msg.jog_val[2];

    MainWindow* _main = qobject_cast<MainWindow*>(main);
    if(_main)
    {
        _main->update_jog_values(vx, vy, wz);
    }
}

void COMM_MSA::handle_move_target(DATA_MOVE &msg)
{
    if(!unimap || !loc || !obsmap || !config || !ctrl || !mobile)
    {
        msg.result = "reject";
        msg.message = "module not loaded";
        send_move_response(msg);
        return;
    }

    QString method = msg.method;
    if(method == "pp")
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx1800] map not loaded";
            send_move_response(msg);
            return;
        }

        if(!loc->get_is_loc())
        {
            msg.result = "reject";
            msg.message = "[R0Px1800] no localization";
            send_move_response(msg);
            return;
        }

        if(config->get_use_multi())
        {
            msg.result = "reject";
            msg.message = "[R0Tx1802] target command not supported by multi. use goal_id";
            send_move_response(msg);
            return;
        }

        double x = msg.tgt_pose_vec[0]; double y = msg.tgt_pose_vec[1];

        if(x < unimap->get_map_min_x() || x > unimap->get_map_max_x() || y < unimap->get_map_min_y() || y > unimap->get_map_max_y())
        {
            msg.result = "reject";
            msg.message = "[R0Tx1800] target location out of range";
            send_move_response(msg);
            return;
        }

        Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
        Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[3]*D2R));
        goal_tf(2,3) = pose_vec[2];
        if(obsmap->is_tf_collision(goal_tf))
        {
            msg.result = "reject";
            msg.message = "[R0Tx1801] target location occupied(static obs)";
            send_move_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_move_response(msg);

        Q_EMIT (ctrl->signal_move(msg));
    }
    else
    {
        msg.result = "reject";
        msg.message = "[R0Sx1800]not supported";

        send_move_response(msg);
    }
}

void COMM_MSA::handle_move_goal(DATA_MOVE &msg)
{
    QString method = msg.method;
    if(method == "pp"||method == "hpp")
    {
        given_method = method;
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = "[R0Mx1800] map not loaded";
            send_move_response(msg);
            return;
        }

        if(!loc->get_is_loc())
        {
            msg.result = "reject";
            msg.message = "[R0Px1800] no localization";
            send_move_response(msg);
            return;
        }

        QString goal_id = msg.goal_node_id;
        if(goal_id.isEmpty())
        {
            msg.result = "reject";
            msg.message = "[R0Nx2000]empty node id";
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
                send_move_response(msg);
                return;
            }

            // convert name to id
            msg.goal_node_id   = node->id;
            msg.goal_node_name = node->name;
        }
        else
        {
            msg.goal_node_name = node->name;
        }

        mobile->move(0,0,0);

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
        msg.cur_pos = cur_pos;

        Eigen::Vector3d xi = TF_to_se2(node->tf);
        msg.tgt_pose_vec[0] = xi[0];
        msg.tgt_pose_vec[1] = xi[1];
        msg.tgt_pose_vec[2] = node->tf(2,3);
        msg.tgt_pose_vec[3] = xi[2];

        // calc eta (estimation time arrival)
        Eigen::Matrix4d goal_tf = node->tf;
        PATH global_path = ctrl->calc_global_path(goal_tf);
        if(global_path.pos.size() < 2)
        {
            msg.result = "accept";
            msg.message = "just change goal";
            msg.remaining_time = 0.0;
            msg.remaining_dist = 0.0;
        }
        else
        {
            msg.result = "accept";
            msg.message = "";
            //            calc_remaining_time_distance(msg);
        }

        send_move_response(msg);

        if(config->get_use_multi())
        {
            is_before_given_path = true;
            fms_cmd_direction = msg.direction;

//            log_info("Move goal accepted — waiting for path from upper system");
            //            qDebug()<<"is multi mode is working";
        }
        else
        {
            Q_EMIT (ctrl->signal_move(msg));
        }

    }
    else if(method == "tng")
    {
        msg.result = "reject";
        msg.message = "not supported yet";
        send_move_response(msg);
    }
    else
    {
        msg.result = "reject";
        msg.message = "[R0Sx2000]not supported";
        send_move_response(msg);
    }
}

void COMM_MSA::handle_move_pause(DATA_MOVE &msg)
{
    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    ctrl->set_is_pause(true);
}

void COMM_MSA::handle_move_resume(DATA_MOVE &msg)
{
    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    ctrl->set_is_pause(false);
}

//void COMM_MSA::slot_profile_move(DATA_MOVE msg)
//{
//    msg.result = "accept";
//    msg.message = "";
//    send_move_response(msg);

//    const QString command = msg.command;

//    float target_linear_ = 0.0; // m or deg
//    float target_speed_ = 0.0; // m or deg
//    int direction_ = msg.direction.toInt(); // 0 : right, 1 : left

//    if(msg.command == "xLinear")
//    {
//        // setting target and speed unit
//        target_linear_ = static_cast<float>(msg.target);
//        target_speed_ = static_cast<float>(msg.speed);

//        if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
//        {
//            //qDebug() << "invalid target or speed";
//            //exception
//            msg.result = "reject";
//            msg.message = "invalid target or speed";

//            send_profile_move_response(msg);
//            return;
//        }

//        else
//        {
//            // first accept response----------
//            //qDebug() << "first accept response";
//            msg.result = "accept";
//            msg.message = "";
//            send_profile_move_response(msg);


//            //--------------------------------

//            AUTOCONTROL::instance()->set_is_moving(true);

//            MOBILE::instance()->move_linear_x(target_linear_, target_speed_);
//            double t = std::abs(target_linear_/target_speed_) + 0.5;

//            QTimer::singleShot(t*1000, [this, msg]() mutable
//            {
//                if (!this) return; // rrs thread is not alive

//                AUTOCONTROL::instance()->set_is_moving(false);

//                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
//                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

//                msg.result = "success";
//                msg.message = "";
//                msg.remaining_dist = res_linear_remain_dist;
//                msg.meassured_dist = res_linear_dist;
//                send_profile_move_response(msg);
//                return;
//            });
//        }
//    }
//    else if(msg.command == "yLinear")
//    {
//        // setting target and speed unit
//        target_linear_ = static_cast<float>(msg.target);
//        target_speed_ = static_cast<float>(msg.speed);

//        if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
//        {
//            //exception
//            msg.result = "reject";
//            msg.message = "invalid target or speed";

//            send_profile_move_response(msg);
//            return;
//        }
//        else
//        {
//            // first accept response----------
//            msg.result = "accept";
//            msg.message = "";
//            send_profile_move_response(msg);
//            //--------------------------------

//            AUTOCONTROL::instance()->set_is_moving(true);

//            MOBILE::instance()->move_linear_y(target_linear_, target_speed_);
//            double t = std::abs(target_linear_/target_speed_) + 0.5;

//            QTimer::singleShot(t*1000, [this, msg]() mutable
//            {
//                if (!this) return; // rrs thread is not alive

//                AUTOCONTROL::instance()->set_is_moving(false);

//                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
//                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

//                msg.result = "success";
//                msg.message = "";
//                msg.remaining_dist = res_linear_remain_dist;
//                msg.meassured_dist = res_linear_dist;
//                send_profile_move_response(msg);
//                return;
//            });
//        }
//    }
//    ctrl->set_is_pause(false);
//}

void COMM_MSA::slot_profile_move(DATA_MOVE msg)
{
    const QString command = msg.command;

    float target_linear_ = 0.0; // m or deg
    float target_speed_ = 0.0; // m or deg
    int direction_ = -1 ; // 0 : right, 1 : left

    if(command == "xLinear")
    {
        // setting target and speed unit
        target_linear_ = static_cast<float>(msg.target);
        target_speed_ = static_cast<float>(msg.speed);

        if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
        {
            //qDebug() << "invalid target or speed";
            //exception
            msg.result = "reject";
            msg.message = "invalid target or speed";

            send_profile_move_response(msg);
            return;
        }

        else
        {
            // first accept response----------
            //qDebug() << "first accept response";
            msg.result = "accept";
            msg.message = "";
            send_profile_move_response(msg);


            //--------------------------------

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_linear_x(target_linear_, target_speed_);
            double t = std::abs(target_linear_/target_speed_) + 0.5;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if (!this) return; // rrs thread is not alive

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_linear_remain_dist;
                msg.meassured_dist = res_linear_dist;
                send_profile_move_response(msg);
                return;
            });
        }
    }
    else if(command == "yLinear")
    {
        // setting target and speed unit
        target_linear_ = static_cast<float>(msg.target);
        target_speed_ = static_cast<float>(msg.speed);

        if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
        {
            //exception
            msg.result = "reject";
            msg.message = "invalid target or speed";

            send_profile_move_response(msg);
            return;
        }
        else
        {
            // first accept response----------
            msg.result = "accept";
            msg.message = "";
            send_profile_move_response(msg);
            //--------------------------------

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_linear_y(target_linear_, target_speed_);
            double t = std::abs(target_linear_/target_speed_) + 0.5;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if (!this) return; // rrs thread is not alive

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_linear_remain_dist;
                msg.meassured_dist = res_linear_dist;
                send_profile_move_response(msg);
                return;
            });
        }
    }
    else if(command == "circular")
    {
        // setting target and speed unit
        target_linear_ = static_cast<float>(msg.target * D2R);
        target_speed_ = static_cast<float>(msg.speed * D2R);

        // setting direction
        if(msg.dir == "right")
        {
            direction_ = 0;
        }
        else if(msg.dir == "left")
        {
            direction_ = 1;
        }

        if(fabs(target_linear_) > 360.0 || fabs(target_speed_) > 60.0)
        {
            //exception
            msg.result = "reject";
            msg.message = "invalid target or speed";

            send_profile_move_response(msg);
            return;
        }
        else
        {
            // first accept response----------
            msg.result = "accept";
            msg.message = "";
            send_profile_move_response(msg);


            //--------------------------------

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_circular(target_linear_, target_speed_, direction_);
            double t = std::abs(target_linear_/target_speed_) +1.0;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if (!this) return;

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_circular_dist = MOBILE::instance()->get_res_linear_dist();
                float res_circular_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

                // qDebug() << "res_circular_dist : " << res_circular_dist << " res_circular_remain_dist : " << res_circular_remain_dist;
                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_circular_remain_dist;
                msg.meassured_dist = res_circular_dist;

                send_profile_move_response(msg);
                return;
            });

        }
    }
    else if(command == "rotate")
    {
        // setting target and speed unit
        target_linear_ = static_cast<float>(msg.target * D2R);
        target_speed_ = static_cast<float>(msg.speed * D2R);

        // speed excption
        if(fabs(target_linear_) > 360.0 || fabs(target_speed_) > 60.0)
        {
            msg.result = "reject";
            msg.message = "invalid target or speed";

            send_profile_move_response(msg);
            return;
        }
        else
        {

            // first accept response----------
            msg.result = "accept";
            msg.message = "";
            send_profile_move_response(msg);
            //--------------------------------

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_rotate(target_linear_, target_speed_);
            double t = std::abs(target_linear_/target_speed_) + 0.5;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if (!this) return;

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();


                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_linear_remain_dist;
                msg.meassured_dist = res_linear_dist;
                send_profile_move_response(msg);
                return;
            });
        }
    }
    else if(command == "stop")
    {
        msg.result = "success";
        msg.message = "";

        MainWindow* _main = qobject_cast<MainWindow*>(main);

        if(_main)
        {
            _main->bt_MoveStop();
        }

        send_profile_move_response(msg);
    }
}
void COMM_MSA::handle_move_stop(DATA_MOVE &msg)
{
    if(is_main_window_valid())
    {
        msg.result = "accept";
        msg.message = "";
        send_move_response(msg);

        MainWindow* _main = (MainWindow*)main;
        QMetaObject::invokeMethod(_main, "bt_Emergency", Qt::QueuedConnection);
    }
    else
    {
        msg.result = "reject";
        msg.message = "mainwindow module not available";
        send_move_response(msg);

        logger->write_log("[COMM_MSA] MainWindow not available", "Red");
    }
}


//void COMM_MSA::handle_localization_cmd(const QJsonObject& data)
//{
//    DATA_LOCALIZATION msg;
//    msg.command = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"
//    msg.seed = get_json(data, "seed");
//    msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
//    msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
//    msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
//    msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble();
//    msg.time = get_json(data, "time").toDouble() / 1000;

//    {
//        std::lock_guard<std::mutex> lock(common_mtx);
//        common_queue.push(msg);
//    }


////    Q_EMIT signal_localization_status();

//    move_cv.notify_one();
//}

void COMM_MSA::slot_localization(DATA_LOCALIZATION msg)
{
    //    const QString command = msg.command;
    //    if(command == "semiautoinit")
    //    {
    //        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
    //        {
    //            msg.result = "reject";
    //            msg.message = "[R0Mx0602]not loaded map";

    //            send_localization_response(msg);
    //            return;
    //        }

    //        if(!lidar_2d || !lidar_2d->get_is_connected())
    //        {
    //            msg.result = "reject";
    //            msg.message = "[R0Lx0601]not connected lidar";

    //            send_localization_response(msg);
    //            return;
    //        }

    //        if(!loc || loc->get_is_busy())
    //        {
    //            msg.result = "reject";
    //            msg.message = "[R0Rx0600]already running";

    //            send_localization_response(msg);
    //            return;
    //        }

    //        msg.result = "accept";
    //        msg.message = "";
    //        send_localization_response(msg);

    //        // do process
    //        if(logger)
    //        {
    //            logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);
    //        }

    //        if(loc)
    //        {
    //            loc->stop();
    //        }

    //        // semi auto init
    //        if(semi_auto_init_thread)
    //        {
    //            if(logger)
    //            {
    //                logger->write_log("[AUTO_INIT] recv_loc, thread already running.", "Orange", true, false);
    //            }
    //            if(semi_auto_init_thread->joinable())
    //            {
    //                semi_auto_init_thread->join();
    //            }
    //            semi_auto_init_thread.reset();
    //        }

    //        if(loc)
    //        {
    //            semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
    //        }
    //    }
    //    else if(command == "init")
    //    {
    //        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
    //        {
    //            msg.result = "reject";
    //            msg.message = "[R0Mx0702]not loaded map";

    //            send_localization_response(msg);
    //            return;
    //        }
    //        if(!lidar_2d || !lidar_2d->get_is_connected())
    //        {
    //            msg.result = "reject";
    //            msg.message = "[R0Lx0701]not connected lidar";

    //            send_localization_response(msg);
    //            return;
    //        }

    //        msg.result = "accept";
    //        msg.message = "";
    //        send_localization_response(msg);

    //        // manual init
    //        const double x = msg.tgt_pose_vec[0];
    //        const double y = msg.tgt_pose_vec[1];
    //        const double test = msg.tgt_pose_vec[2];
    //        const double rz = msg.tgt_pose_vec[3];

    //        if(logger)
    //        {
    //            logger->write_log(QString("[COMM_MSA] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz * D2R).arg(msg.time), "Green");
    //        }

    //        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));
    //        if(loc)
    //        {
    //            loc->stop();
    //            loc->set_cur_tf(tf);
    //        }
    //    }
    //    else if(command == "start")
    //    {
    //        msg.result = "accept";
    //        msg.message = "";

    //        send_localization_response(msg);

    //        if(loc->get_is_loc())
    //        {
    //            loc->stop();
    //        }

    //        const double x = msg.tgt_pose_vec[0];
    //        const double y = msg.tgt_pose_vec[1];
    //        const double rz = msg.tgt_pose_vec[3];
    //        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));

    //        if(loc)
    //        {
    //            loc->set_cur_tf(tf);
    //            loc->start();
    //        }
    //    }
    //    else if(command == "stop")
    //    {
    //        msg.result = "accept";
    //        msg.message = "";

    //        send_localization_response(msg);

    //        if(loc)
    //        {
    //            loc->stop();
    //        }
    //    }
    //    else if(command == "randominit")
    //    {
    //        msg.result = "accept";
    //        msg.message = "";

    //        send_localization_response(msg);

    //        const QString seed = msg.seed;

    //        MainWindow* _main = qobject_cast<MainWindow*>(main);
    //        if(_main)
    //        {
    //            //_main->slot_sim_random_init(seed);
    //        }
    //    }
}
void COMM_MSA::calc_remaining_time_distance(DATA_MOVE &msg)
{
    // time align (first align + final align)
    CTRL_PARAM params = ctrl->get_cur_ctrl_params();
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();

    PATH global_path = ctrl->get_cur_global_path();

    Eigen::Vector2d dtdr = dTdR(cur_tf, global_path.pose.front());
    double time_align = (dtdr[1] / (params.LIMIT_W*D2R + 1e-06)) * 2;

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

    msg.remaining_dist = remaining_dist;
    msg.remaining_time = time_driving + time_align;
}

void COMM_MSA::handle_path_move(DATA_PATH& msg)
{
    double cur_time = get_time0();
    if((cur_time - msg.time) > 5.0)
    {
        logger->write_log(QString("[COMM_MSA] path cmd:move -> too long time after receieved ... %1").arg(cur_time - msg.time));
        return;
    }

    if(old_path != msg.path)
    {
        handle_path(msg);
//        ctrl->slot_path(fms_cmd_direction);
    }
    old_path = msg.path;
    given_method = "";
    fms_cmd_direction = "";
}

void COMM_MSA::handle_path(DATA_PATH& msg)
{
    // stop first
    mobile->move(0,0,0);

    // update vobs first
    std::vector<Eigen::Vector3d> vobs_c_list;
    {
        QString vobs_str = msg.vobs_closures;
        QStringList vobs_str_list = vobs_str.split(",");

        // set vobs
        for(int p = 0; p < vobs_str_list.size(); p++)
        {
            QString node_id = vobs_str_list[p];
            if(node_id != "")
            {
                NODE *node = unimap->get_node_by_id(node_id);
                if(node != nullptr)
                {
                    vobs_c_list.push_back(node->tf.block(0,3,3,1));
                }
            }
        }

        obsmap->set_vobs_list_closures(vobs_c_list);
        obsmap->update_vobs_map();
    }

    // and move path
    std::vector<QString> path;
    std::vector<int> step;
    {
        QString path_str = msg.path;
        QStringList path_str_list = path_str.split(",");
        QString path;
        for(int p = 0; p < path_str_list.size(); p++)
        {
//            path.push_back(path_str_list[p]);
            path += path_str_list[p]+",";
//            step.push_back((int)p);
        }

        DATA_PATH path_msg;
        path_msg.command = "goal";
        path_msg.path = path;
        path_msg.preset = 0;
        path_msg.method = msg.method;
//        msg.direction = direction;
//        qDebug()<<"path : "<<path;

         Q_EMIT (AUTOCONTROL::instance()->slot_path(path_msg));
//        ctrl->move(path, msg.preset);
//        ctrl->set_path(path, step, msg.preset, (long long)(msg.time));
    }

    send_path_response(msg);
}


void COMM_MSA::handle_vobs(DATA_VOBS& msg)
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
                if(node != nullptr)
                {
                    vobs_c_list.push_back(node->tf.block(0,3,3,1));
                }
            }
        }
    }

    // update vobs
    {
        obsmap->set_vobs_list_robots(vobs_r_list);
        if(msg.is_vobs_closures_change == "true")
        {
            obsmap->set_vobs_list_closures(vobs_c_list);
        }
        obsmap->update_vobs_map();
    }
}

void COMM_MSA::handle_common_load_map(DATA_LOAD& msg)
{
    QString map_name = msg.map_name;

    if(map_name != ""){
        QString load_dir = "/data/maps/" + map_name;

        if(!QDir(load_dir).exists())
        {
            msg.result = "reject";
            msg.message = "[R0Mx0201] invalid map dir";

            send_load_response(msg);
            return;
        }
        loc->stop();
        obsmap->clear();
        config->set_map_path(load_dir);

        msg.result = "accept";
        msg.message = "";
        send_load_response(msg);
        unimap->load_map(load_dir);

        MainWindow* _main = (MainWindow*)main;
        QMetaObject::invokeMethod(_main, "all_update", Qt::QueuedConnection);
    }
    else
    {
        msg.result = "reject";
        msg.message = "[R0Mx0201] invalid map dir";

        send_load_response(msg);
        return;
    }
}

void COMM_MSA::handle_common_load_topo(DATA_LOAD& msg)
{
    msg.result = "reject";
    msg.message = "[R0Sx0301]not support yet";
}

void COMM_MSA::handle_common_load_config(DATA_LOAD& msg)
{
    msg.result = "reject";
    msg.message = "[R0Sx0401]not support yet";
}

// for safetyio
void COMM_MSA::slot_safety_io(DATA_SAFTYIO msg)
{
    send_safetyio_response(msg);

    // packet
    for (int i = 0; i < 2; i++) // 0:mcu0, 1:mcu1
    {
        unsigned char* dio_arr = (i == 0) ? msg.mcu1_dio : msg.mcu0_dio;
        if (!dio_arr)
        {
            continue;
        }

        int offset = (i == 0) ? 0 : 8; //  target mcu1->0~7 ,mcu0 8~15
        for (int n = 0; n < 8; n++)
        {
            bool value = dio_arr[n]; // 0 or 1
            if(value != dio_arr_old[offset+n])
            {
                mobile->set_IO_individual_output(offset + n, value);
                dio_arr_old[offset + n] = value; //save change value
            }
        }
    }
}

void COMM_MSA::send_profile_move_response(const DATA_MOVE& msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    send_object->get_map()["command"]        = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]         = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]        = sio::string_message::create(msg.message.toStdString());
    send_object->get_map()["remain_dist"]    = sio::double_message::create(msg.remaining_dist);
    send_object->get_map()["meassured_dist"] = sio::double_message::create(msg.meassured_dist);
    send_object->get_map()["time"]           = sio::double_message::create((long long)(msg.time*1000));

    io->socket("slamnav")->emit("moveResponse", send_object);

//    // for plot
//    mtx.lock();
////    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
//    mtx.unlock();
}

void COMM_MSA::send_safetyio_response(const DATA_SAFTYIO& msg)
{
    if(!is_connected)
    {
        return;
    }
    sio::object_message::ptr send_obj = sio::object_message::create();

    send_obj->get_map()["command"] = sio::string_message::create(msg.command.toStdString());
    send_obj->get_map()["command"] = sio::string_message::create(msg.id.toStdString());

    // MCU 2차원 배열
    sio::array_message::ptr total_arr = sio::array_message::create();

    sio::array_message::ptr mcu0_arr = sio::array_message::create();
    for(int i = 0; i < 8; i++)
    {
        mcu0_arr->get_vector().push_back(sio::int_message::create(msg.mcu0_dio[i]));
    }
    total_arr->get_vector().push_back(mcu0_arr);

    sio::array_message::ptr mcu1_arr = sio::array_message::create();
    for(int i = 0; i < 8; i++)
    {
        mcu1_arr->get_vector().push_back(sio::int_message::create(msg.mcu1_dio[i]));
    }
    total_arr->get_vector().push_back(mcu1_arr);

    send_obj->get_map()["mcuDio"] = total_arr;

    send_obj->get_map()["time"] = sio::string_message::create(QString::number(static_cast<qint64>(msg.time * 1000)).toStdString());

    //    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("controlResponse", send_obj);
}


QMainWindow* COMM_MSA::get_main_window()
{
    return qobject_cast<QMainWindow*>(main);
}

bool COMM_MSA::get_msa_connect_check()
{
    std::shared_lock<std::shared_mutex> lock(msg_mtx);
    return is_connected;
}

QString COMM_MSA::get_msa_text()
{
    std::shared_lock<std::shared_mutex> lock(msg_mtx);
    return receive_msg;
}

bool COMM_MSA::is_main_window_valid()
{
    return (qobject_cast<QMainWindow*>(main) != nullptr);
}
