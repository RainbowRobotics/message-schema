#include "comm_fms.h"
#include "mainwindow.h"

COMM_FMS* COMM_FMS::instance(QObject *parent)
{
    static COMM_FMS* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COMM_FMS(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }

    return inst;
}

COMM_FMS::COMM_FMS(QObject *parent)
    : QObject{parent}
{
    client = new QWebSocket(QString(), QWebSocketProtocol::VersionLatest, this);
    send_timer = new QTimer(this);
    reconnect_timer = new QTimer(this);

    connect(client, &QWebSocket::connected, this, &COMM_FMS::connected);
    connect(client, &QWebSocket::disconnected, this, &COMM_FMS::disconnected);
    connect(client, &QWebSocket::textMessageReceived, this, &COMM_FMS::recv_message);

    connect(send_timer,      SIGNAL(timeout()),                 this, SLOT(send_loop()));
    connect(reconnect_timer, SIGNAL(timeout()),                 this, SLOT(reconnect_loop()));
    connect(this,            SIGNAL(signal_send_move_status()), this, SLOT(send_move_status()));
}

COMM_FMS::~COMM_FMS()
{
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

    is_send_status_running = false;
    if(send_status_thread->joinable())
    {
        send_status_thread->join();
    }

    is_move_running = false;
    move_cv.notify_all();
    if(move_thread->joinable())
    {
        move_thread->join();
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
}

QString COMM_FMS::get_json(const QJsonObject& json, QString key)
{
    return json[key].toString();
}

QString COMM_FMS::get_multi_state()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return multi_state;
}

double COMM_FMS::get_process_time_path()
{
    return (double)process_time_path.load();
}

double COMM_FMS::get_process_time_vobs()
{
    return (double)process_time_vobs.load();
}

double COMM_FMS::get_max_process_time_path()
{
    return (double)max_process_time_path.load();
}

double COMM_FMS::get_max_process_time_vobs()
{
    return (double)max_process_time_vobs.load();
}

void COMM_FMS::init()
{
    // update robot id
    robot_id = QString("R_%1").arg(static_cast<long long>(get_time()*1000));
    //robot_id = "R_0001";
    logger->write_log(QString("[COMM_FMS] ID: %1").arg(robot_id));

    // start reconnect loop
    reconnect_timer->start(3000);
    logger->write_log("[COMM_FMS] start reconnect timer");

    if(recv_thread == nullptr)
    {
        is_recv_running = true;
        recv_thread = std::make_unique<std::thread>(&COMM_FMS::recv_loop, this);
    }

    if(send_status_thread == nullptr)
    {
        is_send_status_running = true;
        send_status_thread = std::make_unique<std::thread>(&COMM_FMS::send_status_loop, this);
    }

    if(move_thread == nullptr)
    {
        move_thread = std::make_unique<std::thread>(&COMM_FMS::move_loop, this);
    }

    if(path_thread == nullptr)
    {
        path_thread = std::make_unique<std::thread>(&COMM_FMS::path_loop, this);
    }

    if(vobs_thread == nullptr)
    {
        vobs_thread = std::make_unique<std::thread>(&COMM_FMS::vobs_loop, this);
    }

    if(common_thread == nullptr)
    {
        common_thread = std::make_unique<std::thread>(&COMM_FMS::common_loop, this);
    }

    if(response_thread == nullptr)
    {
        response_thread = std::make_unique<std::thread>(&COMM_FMS::response_loop, this);
    }

    send_timer->start(10);
}

void COMM_FMS::reconnect_loop()
{
    if(!is_connected)
    {
        if(!config || !client)
        {
            logger->write_log("[COMM_FMS] not ready to modules");
            return;
        }

        QString server_ip = config->get_server_ip();
        if(server_ip.isEmpty())
        {
            logger->write_log("[COMM_FMS] Invalid server ip");
            return;
        }

        std::cout << "reconnecting ..." << std::endl;

        QString server_addr = QString("ws://%1:12334").arg(server_ip);
        client->open(QUrl(server_addr));
    }
}

void COMM_FMS::connected()
{
    if(is_connected)
    {
        return;
    }

    is_connected = true;
    logger->write_log("[COMM_FMS] connected");

    if(!ctrl)
    {
        logger->write_log("[COMM_FMS] not ready to modules");
        return;
    }

    ctrl->set_is_rrs(true);
}

void COMM_FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        logger->write_log("[COMM_FMS] disconnected");

        if(!ctrl)
        {
            logger->write_log("[COMM_FMS] not ready to modules");
            return;
        }

        ctrl->set_is_rrs(false);
    }
}

// send status
void COMM_FMS::send_move_status()
{
    if(!is_connected)
    {
        return;
    }

    // get time
    double time = get_time0();

    QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;
    Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
    if(unimap->get_is_loaded() == MAP_LOADED && goal_node_id != "")
    {
        NODE* node = unimap->get_node_by_id(goal_node_id);
        if(node != nullptr)
        {
            goal_tf =  node->tf;
        }
    }

    // convert
    Eigen::Vector3d cur_xi  = TF_to_se2(loc->get_cur_tf());
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);

    // Creating the JSON object
    QJsonObject robotObj;
    robotObj["robotSerial"] = robot_id;

    QJsonObject dataObj;
    dataObj["title"]        = "moveStatus";
    dataObj["time"]         = QString::number((long long)(time*1000), 10);

    QJsonObject curObj;
    curObj["id"]            = ctrl->get_cur_node_id();
    curObj["x"]             = QString::number(cur_xi[0],     'f', 3);
    curObj["y"]             = QString::number(cur_xi[1],     'f', 3);
    curObj["rz"]            = QString::number(cur_xi[2]*R2D, 'f', 3);
    dataObj["cur_node"]     = curObj;

    QJsonObject goalObj;
    goalObj["id"]           = goal_node_id;
    goalObj["x"]            = QString::number(goal_xi[0],     'f', 3);
    goalObj["y"]            = QString::number(goal_xi[1],     'f', 3);
    goalObj["rz"]           = QString::number(goal_xi[2]*R2D, 'f', 3);
    dataObj["goal_node"]    = goalObj;

    int step = ctrl->get_last_step();

    QJsonObject moveObj;
    moveObj["step"]         = QString::number(step, 10);
    moveObj["path"]         = ctrl->get_multi_reqest_state();
    moveObj["auto_move"]    = ctrl->get_auto_state();
    moveObj["path_time"]    = QString::number(ctrl->get_global_path_time(), 10);
    dataObj["move_state"]   = moveObj;

    QJsonObject processObj;
    processObj["max_obs_time"]  = QString::number(get_max_process_time_vobs(), 'f', 3);
    processObj["max_path_time"] = QString::number(get_max_process_time_path(), 'f', 3);
    dataObj["process_time"]     = processObj;

    QJsonObject rootObj;
    rootObj["robot"] = robotObj;
    rootObj["data"]  = dataObj;

    QJsonDocument doc(rootObj);
    QString buf = doc.toJson(QJsonDocument::Compact);

    send_queue.push(buf);
}

// recv callback
void COMM_FMS::recv_message(const QString &buf)
{
    recv_queue.push(buf);
}

void COMM_FMS::recv_loop()
{
    while(is_recv_running)
    {
        QString recv_msg;
        if(recv_queue.try_pop(recv_msg))
        {
            QJsonObject root_obj = QJsonDocument::fromJson(recv_msg.toUtf8()).object();
            if(get_json(root_obj, "robotSerial") != robot_id)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            QString cmd = get_json(root_obj, "topic");
            QJsonObject data = root_obj.value("data").toObject();

            // parsing
            if(cmd == "move")
            {
                handle_move_cmd(data);
            }
            else if(cmd == "path")
            {
                handle_path_cmd(data);
            }
            else if(cmd == "vobs")
            {
                handle_vobs_cmd(data);
            }
            else if(cmd == "load" || cmd == "localization" || cmd == "randomseq" || cmd == "mapping" || cmd == "docking" || cmd == "lidar_onoff" || cmd == "path_onoff" ||
                    cmd == "led"  || cmd == "motor")
            {
                handle_common_cmd(cmd, data);
            }

            logger->write_log(QString("[COMM_FMS] recv, command: %1, time: %2").arg(cmd).arg(get_time()), "Green");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_FMS::handle_path_cmd(const QJsonObject& data)
{
    DATA_PATH msg;
    msg.path_str          = get_json(data, "path");
    msg.time              = get_json(data, "time").toLongLong();
    msg.preset            = get_json(data, "preset").toInt();
    msg.command           = get_json(data, "command");
    msg.vobs_closures_str = get_json(data, "vobs_c");
    {
        std::lock_guard<std::mutex> lock(path_mtx);
        path_queue.push(msg);
    }

    path_cv.notify_one();
}

void COMM_FMS::handle_vobs_cmd(const QJsonObject& data)
{
    DATA_VOBS msg;
    msg.time                    = get_json(data, "time").toDouble()/1000;
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

void COMM_FMS::handle_move_cmd(const QJsonObject& data)
{
    DATA_MOVE msg;
    msg.time            = get_json(data, "time").toDouble()/1000;
    msg.method          = get_json(data, "method");
    msg.preset          = get_json(data, "preset").toInt();
    msg.command         = get_json(data, "command");
    msg.jog_val[0]      = get_json(data, "vx").toDouble();
    msg.jog_val[1]      = get_json(data, "vy").toDouble();
    msg.jog_val[2]      = get_json(data, "wz").toDouble();
    msg.goal_node_id    = get_json(data, "goal_id");
    msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
    msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
    msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
    msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble()*D2R;

    QJsonDocument doc(data);
    QString json_str = doc.toJson(QJsonDocument::Compact);
    std::cout << "json_str: " << json_str.toStdString() << std::endl;

    {
        std::lock_guard<std::mutex> lock(move_mtx);
        move_queue.push(msg);
        move_cv.notify_one();
    }
}

void COMM_FMS::handle_common_cmd(QString cmd, const QJsonObject& data)
{
    if(cmd == "load")
    {
        DATA_LOAD msg;
        msg.time     = get_json(data, "time").toDouble()/1000;
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
    else if(cmd == "localization")
    {
        DATA_LOCALIZATION msg;
        msg.seed            = get_json(data, "seed");
        msg.time            = get_json(data, "time").toDouble() / 1000.;
        msg.command         = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble();

        DATA_COMMON cmsg;
        cmsg.type = DATA_COMMON::TYPE::LOCALIZATION;
        cmsg.dlocalization = msg;
        {
            std::lock_guard<std::mutex> lock(common_mtx);
            common_queue.push(cmsg);
        }

        common_cv.notify_one();
    }
    else if(cmd == "randomseq")
    {
        DATA_RANDOMSEQ msg;
        msg.time    = get_json(data, "time").toDouble() / 1000.;
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
        msg.time    = get_json(data, "time").toDouble()/1000;
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
        msg.time    = get_json(data, "time").toDouble() / 1000.;
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
        msg.time      = get_json(data, "time").toDouble() / 1000.;
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
        msg.time      = get_json(data, "time").toDouble() / 1000.;
        msg.command   = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();

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

void COMM_FMS::move_loop()
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

        std::cout << "command: " << command.toStdString() << std::endl;

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
    }
}

void COMM_FMS::path_loop()
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

        double st_time = get_time();

        QString command = msg.command;
        logger->write_log(QString("[COMM_FMS] recv path, command: %1, time: %2").arg(command).arg(get_time()), "Green");

        if(command == "path")
        {
            handle_path(msg);
        }
        else if(command == "move")
        {
            handle_path_move(msg);
        }

        double ed_time = get_time();
        process_time_path = ed_time - st_time;
        if(max_process_time_path < process_time_path)
        {
            max_process_time_path = (double)process_time_path;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_FMS::vobs_loop()
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

        double st_time = get_time();

        QString command = msg.command;
        if(command == "vobs")
        {
            handle_vobs(msg);
        }

        double ed_time = get_time();
        process_time_vobs = ed_time - st_time;
        if(max_process_time_vobs < process_time_vobs)
        {
            max_process_time_vobs = (double)process_time_vobs;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void COMM_FMS::common_loop()
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
                //if(is_main_window_valid())
                //{
                //    msg.result = "accept";
                //    msg.message = "";
//
                //    //MainWindow* _main = (MainWindow*)main;
                //    //QMetaObject::invokeMethod(_main, "slot_sim_random_seq", Qt::QueuedConnection);
                //}
                //else
                //{
                //    msg.result = "reject";
                //    msg.message = "mainwindow module not available";
//
                //    logger->write_log("[COMM_RRS] MainWindow not available for mapping", "Red");
                //}
            }
        }
        else if(cmd == DATA_COMMON::TYPE::DOCKING)
        {
            DATA_DOCK msg = cmsg.ddock;
            QString command = msg.command;
            if(command == "dock")
            {
                //if(is_main_window_valid())
                //{
                //    msg.result = "accept";
                //    msg.message = "";
//
                //    //MainWindow* _main = (MainWindow*)main;
                //    //QMetaObject::invokeMethod(_main, "bt_DockStart", Qt::QueuedConnection);
                //}
                //else
                //{
                //    msg.result = "reject";
                //    msg.message = "mainwindow module not available";
//
                //    logger->write_log("[COMM_RRS] MainWindow not available", "Red");
                //}
            }
            else if(command == "undock")
            {
                //if(is_main_window_valid())
                //{
                //    msg.result = "accept";
                //    msg.message = "";
//
                //    //MainWindow* _main = (MainWindow*)main;
                //    //QMetaObject::invokeMethod(_main, "bt_UnDockStart", Qt::QueuedConnection);
                //}
                //else
                //{
                //    msg.result = "reject";
                //    msg.message = "mainwindow module not available";
//
                //    logger->write_log("[COMM_RRS] MainWindow not available", "Red");
                //}
            }
        }
        else if(cmd == DATA_COMMON::TYPE::LOCALIZATION)
        {
            DATA_LOCALIZATION msg = cmsg.dlocalization;
            QString command = msg.command;
            if(command == "semiautoinit")
            {
                if(unimap->get_is_loaded() != MAP_LOADED)
                {
                    msg.result = "reject";
                    msg.message = "[R0Mx0602]not loaded map";

                    return;
                }

                QString loc_mode = config->get_loc_mode();
                if(loc_mode == "2D")
                {
                    if(!lidar_2d->get_is_connected())
                    {
                        msg.result = "reject";
                        msg.message = "[R0Lx0601]not connected lidar";
                        return;
                    }
                }
                else if(loc_mode == "3D")
                {
                    if(!lidar_3d->get_is_connected())
                    {
                        msg.result = "reject";
                        msg.message = "[R0Lx0601]not connected lidar";
                        return;
                    }
                }
                else
                {
                    msg.result = "reject";
                    msg.message = "[R0Mx0602] invalid lidar cnt";
                    return;
                }

                if(loc->get_is_busy())
                {
                    msg.result = "reject";
                    msg.message = "[R0Rx0600]already running";
                    return;
                }

                // do process
                logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);

                msg.result = "accept";
                msg.message = "";

                loc->stop();

                // semi auto init
                if(semi_auto_init_thread != nullptr)
                {
                    if(semi_auto_init_thread->joinable())
                    {
                        semi_auto_init_thread->join();
                        semi_auto_init_thread = nullptr;
                        logger->write_log("[AUTO_INIT] recv_loc, semiauto init thread already running.", "Orange", true, false);
                    }
                    else
                    {
                        logger->write_log("[AUTO_INIT] recv_loc, start semiauto init thread.", "Green", true, false);
                    }
                }

                semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
            }
            else if(command == "init")
            {
                if(unimap->get_is_loaded() != MAP_LOADED)
                {
                    msg.result = "reject";
                    msg.message = "[R0Mx0702]not loaded map";

                    continue;
                }

                QString loc_mode = config->get_loc_mode();
                if(loc_mode == "2D")
                {
                    if(!lidar_2d->get_is_connected())
                    {
                        msg.result = "reject";
                        msg.message = "[R0Lx0601]not connected lidar";
                        return;
                    }
                }
                else if(loc_mode == "3D")
                {
                    if(!lidar_3d->get_is_connected())
                    {
                        msg.result = "reject";
                        msg.message = "[R0Lx0601]not connected lidar";
                        return;
                    }
                }
                else
                {
                    msg.result = "reject";
                    msg.message = "[R0Mx0602] invalid lidar cnt";
                    return;
                }

                msg.result = "accept";
                msg.message = "";

                // manual init
                double x    = msg.tgt_pose_vec[0];
                double y    = msg.tgt_pose_vec[1];
                double test = msg.tgt_pose_vec[2];
                double rz   = msg.tgt_pose_vec[3];

                loc->stop();
                loc->set_cur_tf(se2_to_TF(Eigen::Vector3d(x, y, rz*D2R)));
                logger->write_log(QString("[COMM_FMS] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz*D2R).arg(msg.time), "Green");
            }
            else if(command == "start")
            {
                msg.result = "accept";
                msg.message = "";

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

                loc->stop();
            }
            else if(command == "randominit")
            {
                //if(is_main_window_valid())
                //{
                //    //msg.result = "accept";
                //    //msg.message = "";
////
                //    //QString seed = msg.seed;
                //    //MainWindow* _main = (MainWindow*)main;
                //    //QMetaObject::invokeMethod(_main, "slot_sim_random_init", Qt::QueuedConnection, Q_ARG(QString, seed));
                //}
                //else
                //{
                //    msg.result = "reject";
                //    msg.message = "mainwindow module not available";
//
                //    logger->write_log("[COMM_RRS] MainWindow not available", "Red");
                //}
            }
        }
        else if(cmd == DATA_COMMON::TYPE::VIEW_LIDAR)
        {
            DATA_VIEW_LIDAR msg = cmsg.dlidar;
            QString command = msg.command;
            if(command == "on")
            {
                if(msg.frequency > 0)
                {
                    //MainWindow* _main = (MainWindow*)main;
                    //_main->lidar_view_frequency = msg.frequency;
                }
            }
            else if(command == "off")
            {
                //MainWindow* _main = (MainWindow*)main;
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
                    //MainWindow* _main = (MainWindow*)main;
                    //_main->path_view_frequency = msg.frequency;
                }
            }
            else if(command == "off")
            {
                //MainWindow* _main = (MainWindow*)main;
                //_main->path_view_frequency = -1;
            }
        }
        else if(cmd == DATA_COMMON::TYPE::LED)
        {
            DATA_LED msg = cmsg.dled;
            QString command = msg.command;
            if(command == "on")
            {

                msg.result = "accept";
                msg.message = "";
            }
            else if(command == "off")
            {

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

void COMM_FMS::response_loop()
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

void COMM_FMS::send_path_response(DATA_PATH msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject robotObj;
    robotObj["robotSerial"] = robot_id;

    QJsonObject dataObj;
    dataObj["command"] = "path";
    dataObj["time"] = QString::number((long long)(msg.time), 10);

    QJsonObject rootObj;
    rootObj["robot"] = robotObj;
    rootObj["data"] = dataObj;

    QJsonDocument doc(rootObj);
    QString buf = doc.toJson(QJsonDocument::Compact);
    send_queue.push(buf);
}

void COMM_FMS::send_move_response(DATA_MOVE msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject robotObj;
    robotObj["robotSerial"] = robot_id;

    QJsonObject dataObj;
    dataObj["title"] = "moveResponse";
    dataObj["command"] = msg.command;
    dataObj["result"] = msg.result;
    dataObj["message"] = msg.message;
    dataObj["preset"] = QString::number(msg.preset, 10);
    dataObj["method"] = msg.method;
    dataObj["goal_id"] = msg.goal_node_id;

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
    dataObj["goal_name"] = response_goal_node_name;

    //obj["goal_name"] = msg.goal_node_name;
    dataObj["cur_x"] = QString::number(msg.cur_pos[0], 'f', 3);
    dataObj["cur_y"] = QString::number(msg.cur_pos[1], 'f', 3);
    dataObj["cur_z"] = QString::number(msg.cur_pos[2], 'f', 3);
    dataObj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
    dataObj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
    dataObj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
    dataObj["rz"] = QString::number(msg.tgt_pose_vec[3]*R2D, 'f', 3);
    dataObj["vx"] = QString::number(msg.jog_val[0], 'f', 3);
    dataObj["vy"] = QString::number(msg.jog_val[1], 'f', 3);
    dataObj["wz"] = QString::number(msg.jog_val[2], 'f', 3);
    dataObj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonObject rootObj;
    rootObj["robot"] = robotObj;
    rootObj["data"] = dataObj;

    QJsonDocument doc(rootObj);
    QString buf = doc.toJson(QJsonDocument::Compact);
    send_queue.push(buf);
}

void COMM_FMS::send_loop()
{
    QString buf;
    if(send_queue.try_pop(buf))
    {
        std::lock_guard<std::mutex> lock(send_mtx);
        client->sendTextMessage(buf);
    }
}

void COMM_FMS::handle_move_jog(const DATA_MOVE& msg)
{

}

void COMM_FMS::handle_move_target(DATA_MOVE &msg)
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

void COMM_FMS::handle_move_goal(DATA_MOVE &msg)
{
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

        QString goal_id = msg.goal_node_id;
        if(goal_id.isEmpty() )
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

        send_move_response(msg);

        Q_EMIT (ctrl->signal_move(msg));
    }
    else if(method == "hpp")
    {
        msg.result = "reject";
        msg.message = "not supported yet";
        send_move_response(msg);
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

void COMM_FMS::handle_move_pause(DATA_MOVE &msg)
{
    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    ctrl->set_is_pause(true);
}

void COMM_FMS::handle_move_resume(DATA_MOVE &msg)
{
    msg.result = "accept";
    msg.message = "";
    send_move_response(msg);

    ctrl->set_is_pause(false);
}

void COMM_FMS::handle_move_stop(DATA_MOVE &msg)
{
    //if(is_main_window_valid())
    //{
    //    msg.result = "accept";
    //    msg.message = "";
    //    send_move_response(msg);
//
    //    //MainWindow* _main = (MainWindow*)main;
    //    //QMetaObject::invokeMethod(_main, "bt_Emergency", Qt::QueuedConnection);
    //}
    //else
    //{
    //    msg.result = "reject";
    //    msg.message = "mainwindow module not available";
    //    send_move_response(msg);
//
    //    logger->write_log("[COMM_RRS] MainWindow not available", "Red");
    //}
}

void COMM_FMS::calc_remaining_time_distance(DATA_MOVE &msg)
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

void COMM_FMS::handle_path(DATA_PATH& msg)
{
    // stop first
    mobile->move(0,0,0);

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
    {
        QString path_str = msg.path_str;
        QStringList path_str_list = path_str.split(",");

        std::vector<QString> path;
        for(int p = 0; p < path_str_list.size(); p++)
        {
            path.push_back(path_str_list[p]);
        }

        ctrl->set_path(path, msg.preset, (long long)(msg.time));
    }

    send_path_response(msg);
}

void COMM_FMS::handle_path_move(DATA_PATH& msg)
{
    ctrl->signal_move_multi();
}

void COMM_FMS::handle_vobs(DATA_VOBS& msg)
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

void COMM_FMS::handle_common_load_map(DATA_LOAD& msg)
{
    QString map_name = msg.map_name;
    QString load_dir = "/data/maps/" + map_name;
    if(!load_dir.isNull())
    {
        if(!QDir(load_dir).exists())
        {
            msg.result = "reject";
            msg.message = "[R0Mx0201] invalid map dir";
            return;
        }

        loc->stop();
        obsmap->clear();
        config->set_map_path(load_dir);

        msg.result = "accept";
        msg.message = "";
        unimap->load_map(load_dir);

        //MainWindow* _main = (MainWindow*)main;
        //QMetaObject::invokeMethod(_main, "all_update", Qt::QueuedConnection);
    }
}

void COMM_FMS::handle_common_load_topo(DATA_LOAD& msg)
{
    msg.result = "reject";
    msg.message = "[R0Sx0301]not support yet";
}

void COMM_FMS::handle_common_load_config(DATA_LOAD& msg)
{
    msg.result = "reject";
    msg.message = "[R0Sx0401]not support yet";
}

void COMM_FMS::set_config_module(CONFIG* _config)
{
    if(!_config)
    {
        return;
    }

    config = _config;
}

void COMM_FMS::set_logger_module(LOGGER* _logger)
{
    if(!_logger)
    {
        return;
    }

    logger = _logger;
}

void COMM_FMS::set_mobile_module(MOBILE* _mobile)
{
    if(!_mobile)
    {
        return;
    }

    mobile = _mobile;
}

void COMM_FMS::set_lidar_2d_module(LIDAR_2D* _lidar)
{
    if(!_lidar)
    {
        return;
    }

    lidar_2d = _lidar;
}

void COMM_FMS::set_lidar_3d_module(LIDAR_3D* _lidar)
{
    if(!_lidar)
    {
        return;
    }

    lidar_3d = _lidar;
}

void COMM_FMS::set_cam_module(CAM* _cam)
{
    if(!_cam)
    {
        return;
    }

    cam = _cam;
}

void COMM_FMS::set_localization_module(LOCALIZATION* _loc)
{
    if(!_loc)
    {
        return;
    }

    loc = _loc;
}

void COMM_FMS::set_mapping_module(MAPPING* _mapping)
{
    if(!_mapping)
    {
        return;
    }

    mapping = _mapping;
}

void COMM_FMS::set_unimap_module(UNIMAP* _unimap)
{
    if(!_unimap)
    {
        return;
    }

    unimap = _unimap;
}

void COMM_FMS::set_obsmap_module(OBSMAP* _obsmap)
{
    if(!_obsmap)
    {
        return;
    }

    obsmap = _obsmap;
}

void COMM_FMS::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(!_ctrl)
    {
        return;
    }

    ctrl = _ctrl;
}

void COMM_FMS::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if(!_dctrl)
    {
        return;
    }

    dctrl = _dctrl;
}

void COMM_FMS::send_status_loop()
{
    double duration_time_move_status = get_time();

    while(is_send_status_running)
    {
        if(get_time() - duration_time_move_status >= COMM_FMS_INFO::move_status_send_time)
        {
            duration_time_move_status = get_time();
            send_move_status();
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
