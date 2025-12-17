#include "comm_msa.h"
#include "mainwindow.h"

namespace
{
    const char* MODULE_NAME = "MSA";
}

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

    // set socket
    rrs_socket = std::make_unique<sio::client>();
    rrs_socket->set_open_listener(std::bind(&COMM_MSA::connected, this));
    rrs_socket->set_close_listener(std::bind(&COMM_MSA::disconnected, this));

    // command
    sio::socket::ptr sock = rrs_socket->socket("slamnav");
    BIND_EVENT(sock, "moveRequest",         std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
    BIND_EVENT(sock, "loadRequest",         std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
    BIND_EVENT(sock, "localizationRequest", std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
    BIND_EVENT(sock, "mappingRequest",      std::bind(&COMM_MSA::recv_message,       this, std::placeholders::_1));
    BIND_EVENT(sock, "controlRequest",      std::bind(&COMM_MSA::recv_message_array, this, std::placeholders::_1));

    // status
    BIND_EVENT(sock, "vobs", std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));
    BIND_EVENT(sock, "path", std::bind(&COMM_MSA::recv_message, this, std::placeholders::_1));

    // reconncetion
    reconnect_timer = new QTimer(this);
    connect(reconnect_timer, &QTimer::timeout, this, &COMM_MSA::reconnect_loop);
}

COMM_MSA::~COMM_MSA()
{
    rrs_socket->socket("slamnav")->off_all();
    rrs_socket->socket("slamnav")->off_error();
    rrs_socket->sync_close();

    reconnect_timer->stop();

    stop_recv_thread();
    stop_move_thread();
    stop_load_thread();
    stop_mapping_thread();
    stop_localization_thread();
    stop_path_thread();
    stop_vobs_thread();
    stop_send_status_thread();
    stop_send_response_thread();
}

void COMM_MSA::init()
{
    if(!config)
    {
        log_warn("config module not set");
        return;
    }

    if(!config->get_use_msa())
    {
        log_warn("not use msa");
        return;
    }

    // start reconnect loop
    reconnect_timer->start(COMM_MSA_INFO::reconnect_time * 1000);
    log_info("start reconnect timer");

    start_recv_thread();
    start_move_thread();
    start_load_thread();
    start_mapping_thread();
    start_localization_thread();
    start_path_thread();
    start_vobs_thread();
    start_send_status_thread();
    start_send_response_thread();
}

void COMM_MSA::reconnect_loop()
{
    if(is_connected)
    {
        log_debug("already connected");
        return;
    }

    rrs_socket->connect("ws://localhost:15001");
    rrs_socket->socket("slamnav");
}

void COMM_MSA::connected()
{
    if(is_connected)
    {
        log_info("already connected");
        return;
    }

    if(!ctrl)
    {
        log_error("not ready to modules");
        return;
    }
    ctrl->set_is_rrs(true);

    is_connected = true;
    log_info("connected to MSA server");
}

void COMM_MSA::disconnected()
{
    if(!is_connected)
    {
        log_info("already disconnected");
        return;
    }

    if(!ctrl)
    {
        log_error("not ready to modules");
        return;
    }
    ctrl->set_is_rrs(false);

    is_connected = false;
    log_info("disconnected to MSA server");
}

void COMM_MSA::recv_message(sio::event& ev)
{
    sio::message::ptr msg = ev.get_message();
    if(!msg)
    {
        return;
    }

    if(msg->get_flag() == sio::message::flag_string)
    {
        std::string data = msg->get_string();
        log_info("this data flag is string.. not object : {}" , data);
    }
    else if(msg->get_flag() == sio::message::flag_integer)
    {
        int val = msg->get_int();
        log_info("this data flag is int.. not object : {}" , val);
    }
    else if(msg->get_flag() == sio::message::flag_object)
    {
        auto obj = msg->get_map();
        QJsonObject json_obj = sio_object_to_qt_json_object(obj);

        QJsonObject root;
        root.insert("topic", QString::fromStdString(ev.get_name()));
        root.insert("data", json_obj);

        QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
        {
            std::lock_guard<std::mutex> lock(recv_mtx);
            recv_queue.push(wrapped);
        }

        set_last_receive_msg(wrapped);
    }
}

void COMM_MSA::recv_message_array(sio::event& ev)
{
    sio::message::ptr msg = ev.get_message();
    if(!msg || msg->get_flag() != sio::message::flag_object)
    {
        return;
    }

    auto obj = msg->get_map();
    QJsonObject json_obj = sio_object_to_qt_json_object(obj);
    QString cmd = get_json(json_obj, "command"); // command 확인
    if(cmd == "setDigitalIO")
    {
        if(json_obj.contains("mcuDio"))
        {
            auto msg_array = obj["mcuDio"];
            QJsonValue parsed = convert_item(msg_array);
            json_obj.insert("mcuDio", parsed);
        }

        if(json_obj.contains("mcuDin"))
        {
            auto msg_array = obj["mcuDin"];
            QJsonValue parsed = convert_item(msg_array);
            json_obj.insert("mcuDin", parsed);
        }
    }

    QJsonObject root;
    root.insert("topic", QString::fromStdString(ev.get_name()));
    root.insert("data", json_obj);

    QString wrapped = QString(QJsonDocument(root).toJson(QJsonDocument::Compact));
    {
        std::lock_guard<std::mutex> lock(recv_mtx);
        recv_queue.push(wrapped);
    }

    set_last_receive_msg(wrapped);
}

void COMM_MSA::recv_loop()
{
    while(is_recv_running)
    {
        std::unique_lock<std::mutex> lock(recv_mtx);
        recv_cv.wait(lock, [this]
        {
            return !recv_queue.empty() || !is_recv_running;
        });

        if(!is_recv_running)
        {
            break;
        }

        QString recv_msg = recv_queue.front();
        recv_queue.pop();
        lock.unlock();

        QJsonObject root_obj = QJsonDocument::fromJson(recv_msg.toUtf8()).object();
        QJsonObject data = root_obj.value("data").toObject();

        QString cmd = get_json(root_obj, "topic");
        if(cmd == "moveRequest")
        {
            handle_move_cmd(data);
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
            //handle_control_cmd(data);
        }
        else if(cmd == "mappingRequest")
        {
            handle_mapping_cmd(data);
        }
        else if(cmd == "path")
        {
            handle_path_cmd(data);
        }
        else if(cmd == "vobs")
        {
            handle_vobs_cmd(data);
        }

        log_info("recv, command: {}, time: {}", cmd.toStdString(), get_time0());
    }
}

QJsonObject COMM_MSA::sio_object_to_qt_json_object(const std::map<std::string, sio::message::ptr>& obj)
{
    QJsonObject json_obj;
    for(auto& kv : obj)
    {
        QString key = QString::fromStdString(kv.first);

        if(kv.second->get_flag() == sio::message::flag_string)
        {
            json_obj.insert(key, QString::fromStdString(kv.second->get_string()));
        }
        else if(kv.second->get_flag() == sio::message::flag_integer)
        {
            json_obj.insert(key, QJsonValue::fromVariant(QVariant::fromValue<qint64>(kv.second->get_int())));
        }
        else if(kv.second->get_flag() == sio::message::flag_double)
        {
            json_obj.insert(key, kv.second->get_double());
        }
        else if(kv.second->get_flag() == sio::message::flag_boolean)
        {
            json_obj.insert(key, kv.second->get_bool());
        }
        else if(kv.second->get_flag() == sio::message::flag_array || kv.second->get_flag() == sio::message::flag_object)
        {
            json_obj.insert(key, convert_item(kv.second)); // 재귀 호출
        }
    }

    return json_obj;
}

QJsonValue COMM_MSA::convert_item(sio::message::ptr item)
{
    if(!item)
    {
        return QJsonValue();
    }

    int flag = item->get_flag();
    if(flag == sio::message::flag_string)
    {
        return QString::fromStdString(item->get_string());
    }
    else if(flag == sio::message::flag_integer)
    {
        return QJsonValue::fromVariant(QVariant::fromValue<qint64>(item->get_int()));
    }
    else if(flag == sio::message::flag_double)
    {
        return item->get_double();
    }
    else if(flag == sio::message::flag_boolean)
    {
        return item->get_bool();
    }
    else if(flag == sio::message::flag_array)
    {
        QJsonArray arr;
        for(auto& sub_item : item->get_vector())
        {
            arr.append(convert_item(sub_item));
        }
        return arr;
    }
    else if(flag == sio::message::flag_object)
    {
        QJsonObject obj;
        for (auto& kv : item->get_map())
        {
            obj.insert(QString::fromStdString(kv.first), convert_item(kv.second));
        }
        return obj;
    }
    else
    {
        return QJsonValue();
    }
}

// send status
void COMM_MSA::send_move_status()
{
    if(!is_connected || !ctrl || !mobile || !unimap || !dctrl)
    {
        return;
    }

    QString cur_node_id  = ctrl->get_cur_node_id();
    QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;
    const MOBILE_POSE mo = mobile->get_pose();
    const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    const Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

    QString cur_node_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
    {
        cur_node_id = ctrl->get_cur_node_id();
        if(NODE* node = unimap->get_node_by_id(cur_node_id))
        {
            cur_node_name = node->name;
        }
    }

    QString goal_node_name = "";
    Eigen::Vector3d goal_xi(0,0,0);
    if(unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
    {
        if(NODE* node = unimap->get_node_by_id(goal_node_id))
        {
            goal_node_name = node->name;
            goal_xi = TF_to_se2(node->tf);
        }
    }

    // create move object
    sio::object_message::ptr obj_move_state = sio::object_message::create();
    obj_move_state->get_map()["auto_move"] = sio::string_message::create(ctrl->get_auto_state().toStdString()); // "stop", "move", "pause", "error", "not ready", "vir"
    obj_move_state->get_map()["dock_move"] = sio::string_message::create("stop");
    obj_move_state->get_map()["jog_move"] = sio::string_message::create("none");
    obj_move_state->get_map()["obs"] = sio::string_message::create(ctrl->get_obs_condition().toStdString());
    obj_move_state->get_map()["path"] = sio::string_message::create(ctrl->get_multi_reqest_state().toStdString()); // "none", "req_path", "recv_path"

    // create pose object
    sio::object_message::ptr obj_pose = sio::object_message::create();
    obj_pose->get_map()["x"]  = sio::double_message::create(cur_xi[0]);
    obj_pose->get_map()["y"]  = sio::double_message::create(cur_xi[1]);
    obj_pose->get_map()["rz"] = sio::double_message::create(cur_xi[2] * R2D);

    // create velocity object
    sio::object_message::ptr obj_velocity = sio::object_message::create();
    obj_velocity->get_map()["vx"] = sio::double_message::create(mo.vel[0]);
    obj_velocity->get_map()["vy"] = sio::double_message::create(mo.vel[1]);
    obj_velocity->get_map()["wz"] = sio::double_message::create(mo.vel[2] * R2D);

    // create current node object
    sio::object_message::ptr obj_cur_node = sio::object_message::create();
    obj_cur_node->get_map()["x"]     = sio::double_message::create(cur_xi[0]);
    obj_cur_node->get_map()["y"]     = sio::double_message::create(cur_xi[1]);
    obj_cur_node->get_map()["rz"]    = sio::double_message::create(cur_xi[2] * R2D);
    obj_cur_node->get_map()["id"]    = sio::string_message::create(cur_node_id.toStdString());
    obj_cur_node->get_map()["name"]  = sio::string_message::create(cur_node_name.toStdString());
    obj_cur_node->get_map()["state"] = sio::string_message::create("");

    // create goal node object
    sio::object_message::ptr obj_goal_node = sio::object_message::create();
    obj_goal_node->get_map()["x"]     = sio::double_message::create(goal_xi[0]);
    obj_goal_node->get_map()["y"]     = sio::double_message::create(goal_xi[1]);
    obj_goal_node->get_map()["rz"]    = sio::double_message::create(goal_xi[2] * R2D);
    obj_goal_node->get_map()["id"]    = sio::string_message::create(goal_node_id.toStdString());
    obj_goal_node->get_map()["name"]  = sio::string_message::create(goal_node_name.toStdString());
    obj_goal_node->get_map()["state"] = sio::string_message::create(ctrl->get_cur_move_state().toStdString());

    // create root object
    sio::object_message::ptr obj_root = sio::object_message::create();
    obj_root->get_map()["vel"]        = obj_velocity;
    obj_root->get_map()["pose"]       = obj_pose;
    obj_root->get_map()["cur_node"]   = obj_cur_node;
    obj_root->get_map()["goal_node"]  = obj_goal_node;
    obj_root->get_map()["move_state"] = obj_move_state;
    obj_root->get_map()["time"] = sio::string_message::create(QString::number(static_cast<long long>(get_time0()*1000)).toStdString());

    {
        std::lock_guard<std::mutex> sock_lock(send_mtx);
        rrs_socket->socket("slamnav")->emit("moveStatus", obj_root);
    }

    /*SOCKET_MESSAGE socket_msg;
    socket_msg.event = "moveStatus";
    socket_msg.data  = obj_root;
    send_status_queue.push(std::move(socket_msg));*/
}

void COMM_MSA::handle_path_cmd(const QJsonObject& data)
{
    DATA_PATH msg;
    msg.time              = get_json_double(data, "time")/1000;
    msg.path_str          = get_json(data, "path");
    msg.preset            = get_json_int(data, "preset");
    msg.command           = get_json(data, "command");
    msg.vobs_closures_str = get_json(data, "vobs_c");
    {
        std::lock_guard<std::mutex> lock(path_mtx);
        path_queue.push(std::move(msg));
        path_cv.notify_one();
    }
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
        vobs_queue.push(std::move(msg));
        vobs_cv.notify_one();
    }
}

void COMM_MSA::handle_move_cmd(const QJsonObject& data)
{
    DATA_MOVE msg;
    msg.id              = get_json(data, "id");
    msg.time            = get_json_double(data, "time") / 1000.0;
    msg.preset          = get_json_int(data, "preset");
    msg.command         = get_json(data, "command");
    msg.method          = get_json(data, "method");
    msg.direction       = get_json(data, "direction");
    msg.goal_node_id    = get_json(data, "goalId");
    msg.goal_node_name  = get_json(data, "goalName");
    msg.tgt_pose_vec[0] = get_json_double(data, "x");
    msg.tgt_pose_vec[1] = get_json_double(data, "y");
    msg.tgt_pose_vec[2] = get_json_double(data, "z");
    msg.tgt_pose_vec[3] = get_json_double(data, "rz") * D2R;
    msg.jog_val[0]      = get_json_double(data, "vx");
    msg.jog_val[1]      = get_json_double(data, "vy");
    msg.jog_val[2]      = get_json_double(data, "wz");
    msg.target          = get_json_double(data, "target");
    msg.speed           = get_json_double(data, "speed");
    msg.meassured_dist  = get_json_double(data, "measuredDist");
    msg.remaining_dist  = get_json_double(data, "remainingDist");
    msg.remaining_time  = get_json_double(data, "remainingTime");
    msg.bat_percent     = get_json_int(data, "battery");
    msg.result          = get_json(data, "result");
    msg.message         = get_json(data, "message");
    {
        std::lock_guard<std::mutex> lock(move_mtx);
        move_queue.push(std::move(msg));
        move_cv.notify_one();
    }
}

void COMM_MSA::handle_load_cmd(const QJsonObject& data)
{
    DATA_LOAD msg;
    msg.id       = get_json(data, "id");
    msg.time     = get_json(data, "time").toDouble() / 1000;
    msg.command  = get_json(data, "command");
    msg.map_name = get_json(data, "mapName");
    {
        std::lock_guard<std::mutex> lock(load_mtx);
        load_queue.push(msg);
        load_cv.notify_one();
    }
}

void COMM_MSA::handle_mapping_cmd(const QJsonObject& data)
{
    DATA_MAPPING msg;
    msg.id       = get_json(data, "id");
    msg.time     = get_json_double(data, "time")/1000;
    msg.command  = get_json(data, "command");
    msg.map_name = get_json(data, "mapName");
    {
        std::lock_guard<std::mutex> lock(mapping_mtx);
        mapping_queue.push(msg);
        mapping_cv.notify_one();
    }
}

void COMM_MSA::handle_localization_cmd(const QJsonObject& data)
{
    DATA_LOCALIZATION msg;
    msg.id      = get_json(data, "id");
    msg.time    = get_json_double(data, "time")/1000;
    msg.command = get_json(data, "command");

    if (msg.command == "init")
    {
        msg.tgt_pose_vec[0] = get_json_double(data, "x");
        msg.tgt_pose_vec[1] = get_json_double(data, "y");
        msg.tgt_pose_vec[2] = get_json_double(data, "z");
        msg.tgt_pose_vec[3] = get_json_double(data, "rz");

        set_last_tgt_pose_vec(msg.tgt_pose_vec);
    }
    else if (msg.command == "start")
    {
        msg.tgt_pose_vec = get_last_tgt_pose_vec();
    }

    {
        std::lock_guard<std::mutex> lock(localization_mtx);
        localization_queue.push(msg);
        localization_cv.notify_one();
    }
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
            handle_move_profile(msg);
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

        DATA_PATH msg = path_queue.front();
        path_queue.pop();
        lock.unlock();

        double st_time = get_time0();

        handle_path(msg);

        double ed_time = get_time0();
        process_time_path = ed_time - st_time;
        if(max_process_time_path < process_time_path)
        {
            max_process_time_path = static_cast<double>(process_time_path.load());
        }
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

        DATA_LOAD msg = load_queue.front();
        load_queue.pop();
        lock.unlock();

        QString command = msg.command;
        if(command == "loadMap")
        {
            handle_load_map(msg);
        }
        else if(command == "loadTopo")
        {
            handle_load_topo(msg);
        }
        else
        {
            msg.result = "reject";

            send_load_response(msg);
            log_error("Unknown error load loop");
        }
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

        DATA_MAPPING msg = std::move(mapping_queue.front());
        mapping_queue.pop();
        lock.unlock();

        const QString command = msg.command;
        if(command == "mappingStart")
        {
            handle_mapping_start(msg);
        }
        else if(command == "mappingStop")
        {
            handle_mapping_stop(msg);
        }
        else if(command == "mappingSave")
        {
            handle_mapping_save(msg);
        }
        else if(command == "mappingReload")
        {
            handle_mapping_reload(msg);
        }
        else
        {
            msg.result = "reject";
            send_mapping_response(msg);
        }
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

        DATA_LOCALIZATION msg = localization_queue.front();
        localization_queue.pop();
        lock.unlock();

        QString command = msg.command;
        if(command == "semiautoinit")
        {
            handle_localization_semiautoinit(msg);
        }
        else if(command == "init")
        {
            handle_localization_init(msg);
        }
        else if(command == "start")
        {
            handle_localization_start(msg);
        }
        else if(command == "stop")
        {
            handle_localization_stop(msg);
        }
        else if(command == "randominit")
        {
            handle_localization_randominit(msg);
        }
        else
        {
            msg.result = "reject";
            send_localization_response(msg);
        }
    }
}

void COMM_MSA::handle_localization_semiautoinit(DATA_LOCALIZATION& msg)
{
    if(unimap->get_is_loaded() != MAP_LOADED)
    {
        log_error("Map is not loaded");

        msg.result = "reject";
        send_localization_response(msg);
        return;
    }

    if(config->get_loc_mode() == "2D" && !lidar_2d->get_is_connected())
    {
        log_error("LIDAR 2D is not connected");

        msg.result = "reject";
        send_localization_response(msg);
        return;
    }
    else if(config->get_loc_mode() == "3D" && !lidar_3d->get_is_connected())
    {
        log_error("LIDAR 3D is not connected");

        msg.result = "reject";
        send_localization_response(msg);
        return;
    }

    if(loc->get_is_busy())
    {
        log_info("Localization is already running");

        msg.result = "reject";
        send_localization_response(msg);
        return;
    }

    msg.result = "accept";
    msg.message = "";
    send_localization_response(msg);

    log_info("recv_loc, start semi-auto init");
    loc->stop();

    // semi auto init
    if(semi_auto_init_thread)
    {
        log_info("recv_loc, thread already running.");

        if(semi_auto_init_thread->joinable())
        {
            semi_auto_init_thread->join();
        }
        semi_auto_init_thread.reset();
    }

    semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
}

void COMM_MSA::handle_localization_init(DATA_LOCALIZATION& msg)
{
    if(unimap->get_is_loaded() != MAP_LOADED)
    {
        log_error("Map is not loaded");

        msg.result = "reject";
        send_localization_response(msg);
    }

    QString loc_mode = config->get_loc_mode();
    if(loc_mode == "2D" && !lidar_2d->get_is_connected())
    {
        log_error("LIDAR 2D is not connected");

        msg.result = "reject";
        send_localization_response(msg);

        return;
    }
    else if(loc_mode == "3D" && !lidar_3d->get_is_connected())
    {
        log_error("LIDAR 3D is not connected");

        msg.result = "reject";
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

    log_info("recv, command: {}, (x,y,test,th,th_test): {}, {}, {}, {}, {}, time: {}", x, y, test, rz, rz*D2R, msg.time);
}

void COMM_MSA::handle_localization_start(DATA_LOCALIZATION& msg)
{
    double x = msg.tgt_pose_vec[0];
    double y = msg.tgt_pose_vec[1];
    double rz = msg.tgt_pose_vec[3];
    if(isnan(x) || isnan(y) || isnan(rz))
    {
        log_error("invalid params");

        msg.result = "reject";
        send_localization_response(msg);
        return;
    }

    log_info("recv_loc, start localization");

    msg.result = "accept";
    send_localization_response(msg);

    loc->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
    loc->start();
}

void COMM_MSA::handle_localization_stop(DATA_LOCALIZATION& msg)
{
    log_info("recv_loc, stop localization");

    msg.result = "accept";
    msg.message = "";
    send_localization_response(msg);

    loc->stop();
}

void COMM_MSA::handle_localization_randominit(DATA_LOCALIZATION& msg)
{
    if(!is_main_window_valid())
    {
        msg.result = "reject";
        msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::LOC_RANDOM_INIT);
        ERROR_MANAGER::instance()->logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::LOC_RANDOM_INIT);
        send_localization_response(msg);

        log_error("MainWindow not available for randominit");
    }

    // todo
}

void COMM_MSA::handle_mapping_start(DATA_MAPPING& msg)
{
    if(!lidar_2d->get_is_connected())
    {
        msg.result = "reject";
        send_mapping_response(msg);
        return;
    }

    last_send_kfrm_idx = 0;

    msg.result = "accept";
    msg.message = "";
    send_mapping_response(msg);

    if(MainWindow* _main = qobject_cast<MainWindow*>(main))
    {
        QMetaObject::invokeMethod(_main, "bt_MapBuild", Qt::QueuedConnection);
    }
}

void COMM_MSA::handle_mapping_stop(DATA_MAPPING& msg)
{
    msg.result = "accept";
    msg.message = "";

    send_mapping_response(msg);

    if(MainWindow* _main = qobject_cast<MainWindow*>(main))
    {
        _main->bt_MapSave();
    }
}

void COMM_MSA::handle_mapping_save(DATA_MAPPING& msg)
{
    MainWindow* _main = qobject_cast<MainWindow*>(main);
    if(!_main)
    {
        return;
    }

    _main->map_dir = "";
    if(msg.map_name != "")
    {
        _main->change_map_name = true;
        _main->map_dir =  msg.map_name;
    }
    _main->bt_MapSave();

    const QString map_name = msg.map_name;
    const QString save_dir = "/data/maps/" + map_name;

    bool found_csv = false;
    if(std::filesystem::exists(save_dir.toStdString()) && std::filesystem::is_directory(save_dir.toStdString()))
    {
        for(const auto& entry : std::filesystem::directory_iterator(save_dir.toStdString()))
        {
            if(entry.is_regular_file() && entry.path().extension() == ".csv")
            {
                found_csv = true;
                break;
            }
        }
    }

    if(!found_csv)
    {
        msg.result = "fail";
        send_mapping_response(msg);
        return;
    }

    msg.result = "success";
    send_mapping_response(msg);
}

void COMM_MSA::handle_mapping_reload(DATA_MAPPING& msg)
{
    last_send_kfrm_idx = 0;

    msg.result = "accept";
    msg.message = "";

    send_mapping_response(msg);
}

void COMM_MSA::send_dock_response(const DATA_DOCK& msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();
    send_object->get_map()["id"]      = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"] = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]  = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"] = sio::string_message::create(msg.message.toStdString());
    send_object->get_map()["time"]    = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "controlResponse";
    socket_msg.data  = send_object;
    {
        std::lock_guard<std::mutex> lock(send_response_mtx);
        send_response_queue.push(socket_msg);
    }
}

void COMM_MSA::send_local_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }

    PATH path = ctrl->get_cur_local_path();
    if(path.pos.size() == last_sent_path.pos.size())
    {
        bool is_same = true;
        for(size_t i = 0; i < path.pos.size(); i++)
        {
            if(!path.pos[i].isApprox(last_sent_path.pos[i], 1e-6))
            {
                is_same = false;
                break;
            }
        }
        if(is_same)
        {
            return;
        }
    }
    last_sent_path = path;

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

    send_object->get_map()["path"] = jsonArray;
    send_object->get_map()["time"] = sio::string_message::create(QString::number((long long)(get_time0()*1000), 10).toStdString());

    // send
    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "localPath";
    socket_msg.data  = send_object;
    send_status_queue.push(socket_msg);
}

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

    send_object->get_map()["path"] = jsonArray;
    send_object->get_map()["time"] = sio::string_message::create(QString::number((long long)(get_time0()*1000), 10).toStdString());

    // send
    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "globalPath";
    socket_msg.data  = send_object;
    send_status_queue.push(socket_msg);
}

void COMM_MSA::send_lidar_2d()
{
    if(!is_connected || !loc || !lidar_2d)
    {
        return;
    }

    std::vector<Eigen::Vector3d> pts = lidar_2d->get_cur_frm().pts;

    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    
    // Validate pose data
    if(std::isnan(cur_xi[0]) || std::isnan(cur_xi[1]) || std::isnan(cur_xi[2]))
    {
        return;
    }
    
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
            // Skip invalid points
            if(std::isnan(pts[p][0]) || std::isnan(pts[p][1]) || std::isnan(pts[p][2]))
            {
                continue;
            }

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

        // Fill missing points and create json array
        Eigen::Vector3d last_valid_pt(0, 0, 0);
        bool has_valid_pt = false;
        
        for(int i=0; i<360; i++)
        {
            // Update last valid point if current point is valid
            if(!std::isnan(sample_pts[i][0]))
            {
                last_valid_pt = sample_pts[i];
                has_valid_pt = true;
            }
            // Fill missing sample point with previous valid point
            else if(has_valid_pt)
            {
                sample_pts[i] = last_valid_pt;
            }
            // No valid points found yet, use zero
            else
            {
                sample_pts[i] = Eigen::Vector3d(0, 0, 0);
            }

            sio::array_message::ptr jsonObj = sio::array_message::create();
            jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][0]));
            jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][1]));
            jsonObj->get_vector().push_back(sio::double_message::create(sample_pts[i][2]));
            jsonArray->get_vector().push_back(jsonObj);
        }

        // Adding the time object
        rootObject->get_map()["time"] = sio::double_message::create(static_cast<long long>(get_time0() * 1000));
        rootObject->get_map()["data"] = jsonArray;

        // send
        SOCKET_MESSAGE socket_msg;
        socket_msg.event = "lidarCloud";
        socket_msg.data  = rootObject;
        send_status_queue.push(socket_msg);
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

        // send
        SOCKET_MESSAGE socket_msg;
        socket_msg.event = "3DlidarCloud";
        socket_msg.data  = rootObject;
        send_status_queue.push(socket_msg);
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
        rootObj->get_map()["data"] = jsonArray;
        rootObj->get_map()["time"] = sio::double_message::create(static_cast<long long>(get_time0() * 1000));

        // send
        SOCKET_MESSAGE socket_msg;
        socket_msg.event = "mappingCloud";
        socket_msg.data  = rootObj;
        send_status_queue.push(socket_msg);

        last_send_kfrm_idx++;
    }
}

void COMM_MSA::send_path_response(const DATA_PATH& msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();

    sio::object_message::ptr robotObj = sio::object_message::create();

    sio::object_message::ptr dataObj = sio::object_message::create();
    dataObj->get_map()["command"] = sio::string_message::create("path");
    dataObj->get_map()["time"] = sio::double_message::create(msg.time);

    send_object->get_map()["robot"] = robotObj;
    send_object->get_map()["data"]  = dataObj;

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "pathResponse";
    socket_msg.data  = send_object;
    send_status_queue.push(socket_msg);
}

void COMM_MSA::send_status_loop()
{
    double duration_time_move_status = get_time();
    double duration_time_status = get_time();
    double duration_time_mapping_cloud = get_time();
    double duration_time_rtsp_cam_rgb = get_time();

    while(is_send_status_running)
    {
        if(get_time() - duration_time_move_status >= COMM_MSA_INFO::move_status_send_time)
        {
            duration_time_move_status = get_time();
            send_move_status();
        }

        if(get_time() - duration_time_status >= COMM_MSA_INFO::status_send_time)
        {
            duration_time_status = get_time();
            send_status();
        }

        if(get_time() - duration_time_mapping_cloud >= COMM_MSA_INFO::mapping_cloud_send_time)
        {
            duration_time_mapping_cloud = get_time();
            send_mapping_cloud();
        }

        // for variable loop
        double time_lidar_view = 1.0/(static_cast<double>(lidar_view_frequency.load()) + 1e-06) * 10.0;
        if(time_lidar_view > 0)
        {
            if(lidar_view_cnt > time_lidar_view)
            {
                lidar_view_cnt = 0;
                CONFIG::instance()->get_use_lidar_2d() == true ? send_lidar_2d() : send_lidar_3d();
            }
            lidar_view_cnt++;
        }

        double time_path_view = 1.0/(static_cast<double>(path_view_frequency.load()) + 1e-06) * 10.0;
        time_path_view *= 10.0;
        if(time_path_view > 0)
        {
            if(path_view_cnt > time_path_view)
            {
                path_view_cnt = 0;
                if(is_local_path_update2.exchange(false))
                {
                    send_local_path();
                }
                if(is_global_path_update2.exchange(false))
                {
                    send_global_path();
                }
            }
            path_view_cnt++;
        }

        if(config->get_use_rtsp() && config->get_use_cam())
        {
            if(get_time() - duration_time_rtsp_cam_rgb > COMM_MSA_INFO::rtsp_cam_rgb_send_time)
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

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    //rootObj->get_map()["imu"] = imuObj;

    // Adding the motor array
    sio::array_message::ptr motorArray = sio::array_message::create();

    sio::object_message::ptr motorObj1 = sio::object_message::create();
    motorObj1->get_map()["connection"] = sio::bool_message::create((ms.connection_m0 == 1) ? "true" : "false");
    motorObj1->get_map()["status"]     = sio::double_message::create(ms.status_m0);
    motorObj1->get_map()["temp"]       = sio::double_message::create(ms.temp_m0);
    motorObj1->get_map()["current"]    = sio::double_message::create(static_cast<double>(ms.cur_m0) / 10.0);
    motorArray->get_vector().push_back(motorObj1);

    sio::object_message::ptr motorObj2 = sio::object_message::create();
    motorObj2->get_map()["connection"] = sio::bool_message::create((ms.connection_m1 == 1) ? "true" : "false");
    motorObj2->get_map()["status"]     = sio::double_message::create(ms.status_m1);
    motorObj2->get_map()["temp"]       = sio::double_message::create(ms.temp_m1);
    motorObj2->get_map()["current"]    = sio::double_message::create(static_cast<double>(ms.cur_m1) / 10.0);
    motorArray->get_vector().push_back(motorObj2);
    
    if(config->get_robot_wheel_type() == "MECANUM")
    {
        sio::object_message::ptr motorObj3 = sio::object_message::create();
        motorObj3->get_map()["connection"] = sio::bool_message::create((ms.connection_m2 == 1) ? "true" : "false");
        motorObj3->get_map()["status"]     = sio::double_message::create(ms.status_m2);
        motorObj3->get_map()["temp"]       = sio::double_message::create(ms.temp_m2);
        motorObj3->get_map()["current"]    = sio::double_message::create(static_cast<double>(ms.cur_m2) / 10.0);
        motorArray->get_vector().push_back(motorObj1);

        sio::object_message::ptr motorObj4 = sio::object_message::create();
        motorObj4->get_map()["connection"] = sio::bool_message::create((ms.connection_m3 == 1) ? "true" : "false");
        motorObj4->get_map()["status"]     = sio::double_message::create(ms.status_m3);
        motorObj4->get_map()["temp"]       = sio::double_message::create(ms.temp_m3);
        motorObj4->get_map()["current"]    = sio::double_message::create(static_cast<double>(ms.cur_m3) / 10.0);
        motorArray->get_vector().push_back(motorObj4);
    }
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

    sio::object_message::ptr robotStateObj   = sio::object_message::create();
    robotStateObj->get_map()["charge"]       = sio::string_message::create(charge_st_string.toStdString());
    robotStateObj->get_map()["dock"]         = sio::bool_message::create((is_dock == true) ? "true" : "false");
    robotStateObj->get_map()["emo"]          = sio::bool_message::create((ms.motor_stop_state == 1) ? "true" : "false");
    robotStateObj->get_map()["localization"] = sio::string_message::create(cur_loc_state.toStdString());
    robotStateObj->get_map()["power"]        = sio::bool_message::create((ms.power_state == 1) ? "true" : "false");
    robotStateObj->get_map()["sss_recovery"] = sio::bool_message::create(ms.sss_recovery_state == 1);
    robotStateObj->get_map()["sw_reset"]     = sio::bool_message::create(ms.sw_reset == 1);
    robotStateObj->get_map()["sw_stop"]      = sio::bool_message::create(ms.sw_stop == 1);
    robotStateObj->get_map()["sw_start"]     = sio::bool_message::create(ms.sw_start == 1);

    robotStateObj->get_map()["sf_obs_detect"] = sio::bool_message::create(
                ms.safety_state_obstacle_detected_1 || ms.safety_state_obstacle_detected_2);
    robotStateObj->get_map()["sf_bumper_detect"] = sio::bool_message::create(
                ms.safety_state_bumper_stop_1 || ms.safety_state_bumper_stop_2);
    robotStateObj->get_map()["sf_operational_stop"] = sio::bool_message::create(
                ms.operational_stop_state_flag_1 || ms.operational_stop_state_flag_2);
    rootObj->get_map()["robot_state"]        = robotStateObj;

    auto toSioArray = [](unsigned char arr[8]) 
    {
        sio::array_message::ptr jsonArr = sio::array_message::create();
        for(int i = 0; i < 8; ++i)
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

    sio::object_message::ptr mapObj = sio::object_message::create();
    mapObj->get_map()["map_name"]   = sio::string_message::create(map_name.toStdString());
    mapObj->get_map()["map_status"] = sio::string_message::create(map_status.toStdString());
    rootObj->get_map()["map"] = mapObj;

    // usb temp sensor
    sio::object_message::ptr tempObj  = sio::object_message::create();
    tempObj->get_map()["connection"]  = sio::double_message::create(ms.bat_in);
    tempObj->get_map()["temp_sensor"] = sio::double_message::create(ms.bat_out);

    // Adding the time object
    const double time = get_time0();
    rootObj->get_map()["time"] = sio::double_message::create(static_cast<long long>(time * 1000));

    {
        std::lock_guard<std::mutex> sock_lock(send_mtx);
        rrs_socket->socket("slamnav")->emit("status", rootObj);
    }

    /*SOCKET_MESSAGE socket_msg;
    socket_msg.event = "status";
    socket_msg.data  = rootObj;
    send_status_queue.push(socket_msg);*/
}

void COMM_MSA::send_move_response(const DATA_MOVE& msg)
{
    if(!is_connected)
    {
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
    send_object->get_map()["id"]             = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"]        = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]         = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"]        = sio::string_message::create(msg.message.toStdString());
    send_object->get_map()["method"]         = sio::string_message::create(msg.method.toStdString());
    send_object->get_map()["goalId"]         = sio::string_message::create(msg.goal_node_id.toStdString());
    send_object->get_map()["goalName"]       = sio::string_message::create(response_goal_node_name.toStdString());
    send_object->get_map()["preset"]         = sio::int_message::create(msg.preset);
    send_object->get_map()["cur_x"]          = sio::double_message::create(msg.cur_pos[0]);
    send_object->get_map()["cur_y"]          = sio::double_message::create(msg.cur_pos[1]);
    send_object->get_map()["cur_z"]          = sio::double_message::create(msg.cur_pos[2]);
    send_object->get_map()["x"]              = sio::double_message::create(msg.tgt_pose_vec[0]);
    send_object->get_map()["y"]              = sio::double_message::create(msg.tgt_pose_vec[1]);
    send_object->get_map()["z"]              = sio::double_message::create(msg.tgt_pose_vec[2]);
    send_object->get_map()["rz"]             = sio::double_message::create(msg.tgt_pose_vec[3]*R2D);
    send_object->get_map()["vx"]             = sio::double_message::create(msg.jog_val[0]);
    send_object->get_map()["vy"]             = sio::double_message::create(msg.jog_val[1]);
    send_object->get_map()["wz"]             = sio::double_message::create(msg.jog_val[2]);
    send_object->get_map()["time"]           = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());
    send_object->get_map()["bat_percent"]    = sio::int_message::create(msg.bat_percent);
    send_object->get_map()["remaining_dist"] = sio::double_message::create(msg.remaining_dist);
    send_object->get_map()["meassured_dist"] = sio::double_message::create(msg.meassured_dist);

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "moveResponse";
    socket_msg.data  = send_object;
    {
        std::lock_guard<std::mutex> lock(send_response_mtx);
        send_response_queue.push(socket_msg);
        send_response_cv.notify_one();
    }
}

void COMM_MSA::send_localization_response(const DATA_LOCALIZATION& msg)
{
    if(!is_connected)
    {
        return;
    }

    sio::object_message::ptr send_object = sio::object_message::create();
    send_object->get_map()["id"]      = sio::string_message::create(msg.id.toStdString());
    send_object->get_map()["command"] = sio::string_message::create(msg.command.toStdString());
    send_object->get_map()["result"]  = sio::string_message::create(msg.result.toStdString());
    send_object->get_map()["message"] = sio::string_message::create(msg.message.toStdString());
    send_object->get_map()["x"]       = sio::double_message::create(msg.tgt_pose_vec[0]);
    send_object->get_map()["y"]       = sio::double_message::create(msg.tgt_pose_vec[1]);
    send_object->get_map()["z"]       = sio::double_message::create(msg.tgt_pose_vec[2]);
    send_object->get_map()["th"]      = sio::double_message::create(msg.tgt_pose_vec[3]);
    send_object->get_map()["time"]    = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "localizationResponse";
    socket_msg.data  = send_object;
    {
        std::lock_guard<std::mutex> lock(send_response_mtx);
        send_response_queue.push(socket_msg);
        send_response_cv.notify_one();
    }
}

void COMM_MSA::send_load_response(const DATA_LOAD& msg)
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
    send_object->get_map()["mapName"]    = sio::string_message::create(msg.map_name.toStdString());
    send_object->get_map()["time"]       = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "loadResponse";
    socket_msg.data  = send_object;
    {
        std::lock_guard<std::mutex> lock(send_response_mtx);
        send_response_queue.push(socket_msg);
        send_response_cv.notify_one();
    }
}

void COMM_MSA::send_mapping_response(const DATA_MAPPING& msg)
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
    send_object->get_map()["time"]   = sio::string_message::create(QString::number((long long)(msg.time*1000), 10).toStdString());

    SOCKET_MESSAGE socket_msg;
    socket_msg.event = "mappingResponse";
    socket_msg.data  = send_object;
    {
        std::lock_guard<std::mutex> lock(send_response_mtx);
        send_response_queue.push(socket_msg);
        send_response_cv.notify_one();
    }
}

void COMM_MSA::send_response_loop()
{
    while(is_send_response_running)
    {
        std::unique_lock<std::mutex> lock(send_response_mtx);
        send_response_cv.wait(lock, [this]
        {
            return !send_response_queue.empty() || !is_send_response_running;
        });

        if(!is_send_response_running)
        {
            break;
        }

        SOCKET_MESSAGE msg = std::move(send_response_queue.front());
        send_response_queue.pop();
        lock.unlock();

        std::lock_guard<std::mutex> sock_lock(send_mtx);
        rrs_socket->socket("slamnav")->emit(msg.event.toStdString(), msg.data);
    }
}

void COMM_MSA::handle_move_jog(const DATA_MOVE& msg)
{
    double vx = msg.jog_val[0];
    double vy = msg.jog_val[1];
    double wz = msg.jog_val[2]*D2R;

    if(MainWindow* _main = qobject_cast<MainWindow*>(main))
    {
        _main->update_jog_values(vx, vy, wz);
    }
}

void COMM_MSA::handle_move_target(DATA_MOVE &msg)
{
    if(!unimap || !loc || !obsmap || !config || !ctrl || !mobile)
    {
        msg.result = "reject";
        send_move_response(msg);
        return;
    }

    QString method = msg.method;
    if(method == "pp")
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        if(!loc->get_is_loc())
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        if(config->get_use_multi())
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        double x = msg.tgt_pose_vec[0]; double y = msg.tgt_pose_vec[1];
        if(x < unimap->get_map_min_x() || x > unimap->get_map_max_x() || y < unimap->get_map_min_y() || y > unimap->get_map_max_y())
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
        Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[3]*D2R));
        goal_tf(2,3) = pose_vec[2];
        if(obsmap->is_tf_collision(goal_tf))
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_move_response(msg);

        Q_EMIT (ctrl->signal_move_single(msg));
    }
    else if(method == "hpp")
    {
        if(config->get_robot_type() == RobotType::MECANUM_Q150 || config->get_robot_type() == RobotType::MECANUM_VALEO || config->get_robot_type() == RobotType::SEC_CORE)
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                send_move_response(msg);
                return;
            }
            if(!loc->get_is_loc())
            {
                msg.result = "reject";
                send_move_response(msg);
                return;
            }

            QString goal_id = msg.goal_node_id;
            if(goal_id.isEmpty())
            {
                msg.result = "reject";
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
            const Eigen::Vector3d xi = TF_to_se2(node->tf);
            msg.cur_pos = cur_pos;
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2, 3);
            msg.tgt_pose_vec[3] = xi[2];

            const Eigen::Matrix4d goal_tf = node->tf;
            PATH global_path = ctrl->calc_global_path(goal_tf);
            if(global_path.pos.size() < 2)
            {
                msg.remaining_time = 0.0;
            }

            msg.result = "accept";
            send_move_response(msg);

            Q_EMIT (ctrl->signal_move_single(msg));
        }
        else
        {
            msg.result = "reject";
            send_move_response(msg);
        }
    }
    else
    {
        msg.result = "reject";
        send_move_response(msg);
    }
}

void COMM_MSA::handle_move_goal(DATA_MOVE &msg)
{
    QString method = msg.method;
    if(method == "pp" || method == "hpp")
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        if(!loc->get_is_loc())
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        QString goal_id = msg.goal_node_id;
        if(goal_id.isEmpty())
        {
            msg.result = "reject";
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
        }

        send_move_response(msg);
        Q_EMIT (ctrl->signal_move_single(msg));
    }
    else if(method == "tng")
    {
        msg.result = "reject";
        send_move_response(msg);
    }
    else
    {
        msg.result = "reject";
        send_move_response(msg);
    }
}

void COMM_MSA::handle_move_pause(DATA_MOVE& msg)
{
    ctrl->set_is_pause(true);

    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);
}

void COMM_MSA::handle_move_resume(DATA_MOVE& msg)
{
    ctrl->set_is_pause(false);

    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);
}

void COMM_MSA::handle_move_profile(DATA_MOVE& msg)
{
    float target_linear_ = 0.f;
    float target_speed_  = 0.f;

    const QString command = msg.command;
    if(command == "xLinear")
    {
        // setting target and speed unit
        target_linear_ = static_cast<float>(msg.target);
        target_speed_  = static_cast<float>(msg.speed);

        if(fabs(target_linear_) > 10.0 || fabs(target_speed_) > 1.5)
        {
            msg.result = "reject";
            send_move_response(msg);
            return;
        }

        // first accept response----------
        msg.result  = "accept";
        msg.message = "";
        send_move_response(msg);

        //--------------------------------
        AUTOCONTROL::instance()->set_is_moving(true);
        MOBILE::instance()->move_linear_x(target_linear_, target_speed_);

        double t = std::abs(target_linear_/target_speed_) + 0.5;
        QTimer::singleShot(t*1000, [this, msg]() mutable
        {
            if(!this)
            {
                return;
            }

            AUTOCONTROL::instance()->set_is_moving(false);

            float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
            float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

            msg.result = "success";
            msg.message = "";
            msg.remaining_dist = res_linear_remain_dist;
            msg.meassured_dist = res_linear_dist;
            send_move_response(msg);
        });
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
            send_move_response(msg);
            return;
        }
        else
        {
            msg.result = "accept";
            msg.message = "";
            send_move_response(msg);

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_linear_y(target_linear_, target_speed_);
            double t = std::abs(target_linear_/target_speed_) + 0.5;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if(!this)
                {
                    return;
                }

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_linear_dist = MOBILE::instance()->get_res_linear_dist();
                float res_linear_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_linear_remain_dist;
                msg.meassured_dist = res_linear_dist;
                send_move_response(msg);
                return;
            });
        }
    }
    else if(command == "circular")
    {
        int direction_ = -1;
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
            send_move_response(msg);
            return;
        }
        else
        {
            msg.result = "accept";
            msg.message = "";
            send_move_response(msg);

            AUTOCONTROL::instance()->set_is_moving(true);

            MOBILE::instance()->move_circular(target_linear_, target_speed_, direction_);
            double t = std::abs(target_linear_/target_speed_) +1.0;

            QTimer::singleShot(t*1000, [this, msg]() mutable
            {
                if (!this) return;

                AUTOCONTROL::instance()->set_is_moving(false);

                float res_circular_dist = MOBILE::instance()->get_res_linear_dist();
                float res_circular_remain_dist = MOBILE::instance()->get_res_linear_remain_dist();

                msg.result = "success";
                msg.message = "";
                msg.remaining_dist = res_circular_remain_dist;
                msg.meassured_dist = res_circular_dist;

                send_move_response(msg);
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
            send_move_response(msg);
            return;
        }
        else
        {
            msg.result = "accept";
            msg.message = "";
            send_move_response(msg);

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
                send_move_response(msg);
                return;
            });
        }
    }
    else if(command == "stop")
    {
        if(MainWindow* _main = qobject_cast<MainWindow*>(main))
        {
            _main->bt_MoveStop();

            msg.result = "success";
            msg.message = "";
            send_move_response(msg);
        }
        else
        {
            msg.result = "fail";
            msg.message = "";
            send_move_response(msg);
        }
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
        msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MOVE_UNKNOWN_ERROR, ERROR_MANAGER::MOVE_STOP);
        ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MOVE_UNKNOWN_ERROR, ERROR_MANAGER::MOVE_STOP);
        send_move_response(msg);
    }
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

void COMM_MSA::handle_path(DATA_PATH& msg)
{
    // update vobs first
    std::vector<Eigen::Vector3d> vobs_c_list;
    {
        QString vobs_str = msg.vobs_closures_str;
        QStringList vobs_str_list = vobs_str.split(",");

        // set vobs
        for(int p = 0; p < vobs_str_list.size(); p++)
        {
            QString node_id = vobs_str_list[p];
            if(node_id != "")
            {
                NODE* node = unimap->get_node_by_id(node_id);
                if(node != nullptr)
                {
                    vobs_c_list.push_back(node->tf.block(0,3,3,1));
                }
            }
        }

        // update vobs
        obsmap->set_vobs_list_closures(vobs_c_list);
        obsmap->update_vobs_map();

        msg.vobs_closures = std::move(vobs_c_list);
    }

    // and move path
    std::vector<QString> path;
    std::vector<int> step;
    {
        QString path_str = msg.path_str;
        QStringList path_str_list = path_str.split(",");
        for(int p = 0; p < path_str_list.size(); p++)
        {
            path.push_back(path_str_list[p]);
            step.push_back(p);
        }

        msg.path = std::move(path);
        msg.step = std::move(step);

        Q_EMIT (AUTOCONTROL::instance()->signal_move_multi(msg));
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
                if(NODE* node = unimap->get_node_by_id(node_id))
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

void COMM_MSA::handle_load_map(DATA_LOAD& msg)
{
    QString map_name = msg.map_name;

    if(map_name != "")
    {
        QString load_dir = "/data/maps/" + map_name;

        if(!QDir(load_dir).exists())
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);
            ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);

            send_load_response(msg);
            return;
        }

        QString map_exist_msg = unimap->is_load_map_check(load_dir);
        if(map_exist_msg == "no 2d map!")
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MAP_LOAD_NO_2D_MAP, ERROR_MANAGER::LOAD_MAP);
            ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_LOAD_NO_2D_MAP, ERROR_MANAGER::LOAD_MAP);


            send_load_response(msg);
            return;
        }
        if(config->get_use_lidar_3d() == true)
        {
            if(map_exist_msg == "no 3d map!")
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MAP_LOAD_NO_3D_MAP, ERROR_MANAGER::LOAD_MAP);
                ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_LOAD_NO_3D_MAP, ERROR_MANAGER::LOAD_MAP);

                send_load_response(msg);
                return;
            }
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
        msg.message = ERROR_MANAGER::instance()->getErrorMessage(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);
        ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_LOAD_INVALID_DIR, ERROR_MANAGER::LOAD_MAP);

        send_load_response(msg);
        return;
    }
}

void COMM_MSA::handle_load_topo(DATA_LOAD& msg)
{
    if(!unimap)
    {
        msg.result = "reject";
        send_load_response(msg);
        return;
    }

    // Check if map is loaded
    if(unimap->get_is_loaded() != MAP_LOADED)
    {
        msg.result = "reject";
        send_load_response(msg);
        return;
    }

    // Get current map path
    QString map_dir = unimap->get_map_path();
    if(map_dir.isEmpty())
    {
        msg.result = "reject";
        send_load_response(msg);
        return;
    }

    // Check if topo.json exists
    QString topo_path = map_dir + "/topo.json";
    QFileInfo topo_info(topo_path);
    if(!topo_info.exists() || !topo_info.isFile())
    {
        msg.result = "reject";
        send_load_response(msg);
        return;
    }

    // Load topo
    log_info("Loading topo from: {}", topo_path.toStdString());
    bool success = unimap->load_topo();
    
    if(success)
    {
        msg.result = "accept";
        msg.message = "";
        log_info("Successfully loaded topo.json");
    }
    else
    {
        msg.result = "reject";
        log_error("Failed to load topo.json");
    }
    
    send_load_response(msg);
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

Eigen::Vector4d COMM_MSA::get_last_tgt_pose_vec()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return last_tgt_pose_vec;
}

void COMM_MSA::set_last_tgt_pose_vec(const Eigen::Vector4d& val)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    last_tgt_pose_vec = val;
}

QMainWindow* COMM_MSA::get_main_window()
{
    return qobject_cast<QMainWindow*>(main);
}

bool COMM_MSA::get_is_connected()
{
    return is_connected.load();
}

QString COMM_MSA::get_last_receive_msg()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return last_receive_msg;
}

void COMM_MSA::set_last_receive_msg(QString val)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    last_receive_msg = val;
}

bool COMM_MSA::is_main_window_valid()
{
    return (qobject_cast<QMainWindow*>(main) != nullptr);
}

void COMM_MSA::set_config_module(CONFIG* _config)
{
    if(!_config)
    {
        return;
    }

    config = _config;
}

void COMM_MSA::set_logger_module(LOGGER* _logger)
{
    if(!_logger)
    {
        return;
    }

    logger = _logger;
}

void COMM_MSA::set_mobile_module(MOBILE* _mobile)
{
    if(!_mobile)
    {
        return;
    }

    mobile = _mobile;
}

void COMM_MSA::set_lidar_2d_module(LIDAR_2D* _lidar)
{
    if(!_lidar)
    {
        return;
    }

    lidar_2d = _lidar;
}

void COMM_MSA::set_lidar_3d_module(LIDAR_3D* _lidar)
{
    if(!_lidar)
    {
        return;
    }

    lidar_3d = _lidar;
}

void COMM_MSA::set_cam_module(CAM* _cam)
{
    if(!_cam)
    {
        return;
    }

    cam = _cam;
}

void COMM_MSA::set_localization_module(LOCALIZATION* _loc)
{
    if(!_loc)
    {
        return;
    }

    loc = _loc;
}

void COMM_MSA::set_mapping_module(MAPPING* _mapping)
{
    if(!_mapping)
    {
        return;
    }

    mapping = _mapping;
}

void COMM_MSA::set_unimap_module(UNIMAP* _unimap)
{
    if(!_unimap)
    {
        return;
    }

    unimap = _unimap;
}

void COMM_MSA::set_obsmap_module(OBSMAP* _obsmap)
{
    if(!_obsmap)
    {
        return;
    }

    obsmap = _obsmap;
}

void COMM_MSA::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(!_ctrl)
    {
        return;
    }

    ctrl = _ctrl;
}

void COMM_MSA::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if(!_dctrl)
    {
        return;
    }

    dctrl = _dctrl;
}

void COMM_MSA::start_recv_thread()
{
    if(recv_thread == nullptr)
    {
        is_recv_running = true;
        recv_thread = std::make_unique<std::thread>(&COMM_MSA::recv_loop, this);
    }
}

void COMM_MSA::start_move_thread()
{
    if(move_thread == nullptr)
    {
        move_thread = std::make_unique<std::thread>(&COMM_MSA::move_loop, this);
    }
}

void COMM_MSA::start_load_thread()
{
    if(load_thread == nullptr)
    {
        load_thread = std::make_unique<std::thread>(&COMM_MSA::load_loop, this);
    }
}

void COMM_MSA::start_mapping_thread()
{
    if(mapping_thread == nullptr)
    {
        mapping_thread = std::make_unique<std::thread>(&COMM_MSA::mapping_loop, this);
    }
}

void COMM_MSA::start_localization_thread()
{
    if(localization_thread == nullptr)
    {
        localization_thread = std::make_unique<std::thread>(&COMM_MSA::localization_loop, this);
    }
}

void COMM_MSA::start_path_thread()
{
    if(path_thread == nullptr)
    {
        path_thread = std::make_unique<std::thread>(&COMM_MSA::path_loop, this);
    }
}

void COMM_MSA::start_vobs_thread()
{
    if(vobs_thread == nullptr)
    {
        vobs_thread = std::make_unique<std::thread>(&COMM_MSA::vobs_loop, this);
    }
}

void COMM_MSA::start_send_status_thread()
{
    if(send_status_thread == nullptr)
    {
        is_send_status_running = true;
        send_status_thread = std::make_unique<std::thread>(&COMM_MSA::send_status_loop, this);
    }
}

void COMM_MSA::start_send_response_thread()
{
    if(send_response_thread == nullptr)
    {
        send_response_thread = std::make_unique<std::thread>(&COMM_MSA::send_response_loop, this);
    }
}

void COMM_MSA::stop_recv_thread()
{
    is_recv_running = false;
    if(recv_thread->joinable())
    {
        recv_thread->join();
    }
}

void COMM_MSA::stop_move_thread()
{
    is_move_running = false;
    move_cv.notify_all();
    if(move_thread->joinable())
    {
        move_thread->join();
    }
}

void COMM_MSA::stop_load_thread()
{
    is_load_running = false;
    load_cv.notify_all();
    if(load_thread->joinable())
    {
        load_thread->join();
    }
}

void COMM_MSA::stop_mapping_thread()
{
    is_mapping_running = false;
    mapping_cv.notify_all();
    if(mapping_thread->joinable())
    {
        mapping_thread->join();
    }
}

void COMM_MSA::stop_localization_thread()
{
    is_localization_running = false;
    localization_cv.notify_all();
    if(localization_thread->joinable())
    {
        localization_thread->join();
    }
}

void COMM_MSA::stop_path_thread()
{
    is_path_running = false;
    path_cv.notify_all();
    if(path_thread->joinable())
    {
        path_thread->join();
    }
}

void COMM_MSA::stop_vobs_thread()
{
    is_vobs_running = false;
    vobs_cv.notify_all();
    if(vobs_thread->joinable())
    {
        vobs_thread->join();
    }
}

void COMM_MSA::stop_send_status_thread()
{
    is_send_status_running = false;
    if(send_status_thread->joinable())
    {
        send_status_thread->join();
    }
}

void COMM_MSA::stop_send_response_thread()
{
    is_send_response_running = false;
    send_response_cv.notify_all();
    if(send_response_thread->joinable())
    {
        send_response_thread->join();
    }
}