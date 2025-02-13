#include "comm_fms.h"
#include "mainwindow.h"

COMM_FMS::COMM_FMS(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
    , main(parent)
{
    connect(&client, &QWebSocket::connected, this, &COMM_FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_FMS::disconnected);
    connect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_FMS::recv_message);

    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));    

    connect(this, SIGNAL(signal_send_move_status()), this, SLOT(send_move_status()));

    connect(this, SIGNAL(recv_move(DATA_MOVE)), this, SLOT(slot_move(DATA_MOVE)));
    connect(this, SIGNAL(recv_load(DATA_LOAD)), this, SLOT(slot_load(DATA_LOAD)));
    connect(this, SIGNAL(recv_localization(DATA_LOCALIZATION)), this, SLOT(slot_localization(DATA_LOCALIZATION)));
    connect(this, SIGNAL(recv_randomseq(DATA_RANDOMSEQ)), this, SLOT(slot_randomseq(DATA_RANDOMSEQ)));
    connect(this, SIGNAL(recv_path(DATA_PATH)), this, SLOT(slot_path(DATA_PATH)));
    connect(this, SIGNAL(recv_vobs_r(DATA_VOBS_R)), this, SLOT(slot_vobs_r(DATA_VOBS_R)));
    connect(this, SIGNAL(recv_vobs_c(DATA_VOBS_C)), this, SLOT(slot_vobs_c(DATA_VOBS_C)));
}

COMM_FMS::~COMM_FMS()
{
    reconnect_timer.stop();

    if(is_connected)
    {
        client.close();
    }
}

QString COMM_FMS::get_json(QJsonObject& json, QString key)
{
    return json[key].toString();
}

QString COMM_FMS::get_multi_state()
{
    mtx.lock();
    QString res = multi_state;
    mtx.unlock();

    return res;
}

void COMM_FMS::init()
{
    // make robot id
    QString _robot_id;
    _robot_id.sprintf("R_%lld", (long long)(get_time()*1000));

    // update robot id
    robot_id = _robot_id;
    printf("[COMM_FMS] ID: %s\n", robot_id.toLocal8Bit().data());

    // start reconnect loop
    reconnect_timer.start(3000);
    printf("[COMM_FMS] start reconnect timer\n");
}

void COMM_FMS::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr;
        server_addr.sprintf("ws://%s:12334", config->SERVER_IP.toLocal8Bit().data());
        client.open(QUrl(server_addr));
    }
}

void COMM_FMS::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        ctrl->is_multi = true;
        printf("[COMM_FMS] connected\n");
    }
}

void COMM_FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        ctrl->is_multi = false;
        printf("[COMM_FMS] disconnected\n");
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
    double time = get_time();

    // get cur_tf
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();

    // get cur_node_id
    QString cur_node_id = "";
    if(unimap->is_loaded)
    {
        MainWindow* _main = (MainWindow*)main;
        _main->mtx.lock();
        cur_node_id = _main->last_node_id;
        _main->mtx.unlock();
    }

    // get goal_node_id
    QString goal_node_id = "";
    Eigen::Matrix4d goal_tf = Eigen::Matrix4d::Identity();
    if(unimap->is_loaded)
    {
        ctrl->mtx.lock();
        if(ctrl->move_info.command == "goal")
        {
            goal_node_id = ctrl->move_info.goal_node_id;
            if(goal_node_id != "")
            {
                NODE* node = unimap->get_node_by_id(goal_node_id);
                if(node != NULL)
                {
                    goal_tf =  node->tf;
                }
                else
                {
                    goal_node_id = "";
                    goal_tf.setIdentity();
                }
            }
        }
        ctrl->mtx.unlock();
    }

    // get path request
    QString multi_req = ctrl->get_multi_req(); // "none", "req_path", "recv_path"

    // get state
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

    // convert
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);

    // Creating the JSON object
    QJsonObject rootObj;

    QJsonObject robotObj;
    robotObj["robotSerial"] = robot_id;
    rootObj["robot"] = robotObj;

    QJsonObject dataObj;
    dataObj["time"] = QString::number((long long)(time*1000), 10);

    QJsonObject curObj;
    curObj["id"] = cur_node_id;
    curObj["x"] = QString::number(cur_xi[0], 'f', 3);
    curObj["y"] = QString::number(cur_xi[1], 'f', 3);
    curObj["rz"] = QString::number(cur_xi[2]*R2D, 'f', 3);
    dataObj["cur_node"] = curObj;

    QJsonObject goalObj;
    goalObj["id"] = goal_node_id;
    goalObj["x"] = QString::number(goal_xi[0], 'f', 3);
    goalObj["y"] = QString::number(goal_xi[1], 'f', 3);
    goalObj["rz"] = QString::number(goal_xi[2]*R2D, 'f', 3);
    dataObj["goal_node"] = goalObj;

    QJsonObject moveObj;
    moveObj["path"] = multi_req;
    moveObj["auto_move"] = auto_state;
    dataObj["move_state"] = moveObj;
    rootObj["data"] = dataObj;

    QJsonDocument doc(rootObj);
    QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
}

// recv callback
void COMM_FMS::recv_message(const QByteArray &buf)
{
    QString message = qUncompress(buf);
    QJsonObject root_obj = QJsonDocument::fromJson(message.toUtf8()).object();
    if(get_json(root_obj, "robotSerial") != robot_id)
    {
        return;
    }

    QString topic = get_json(root_obj, "topic");    
    QJsonObject data_obj = root_obj.value("data").toObject();

    // parsing
    if(topic == "load")
    {
        DATA_LOAD msg;
        msg.command = get_json(data_obj, "command");
        msg.map_name = get_json(data_obj, "name");
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_load(msg);
    }
    else if(topic == "localization")
    {
        DATA_LOCALIZATION msg;
        msg.command = get_json(data_obj, "command");
        msg.tgt_pose_vec[0] = get_json(data_obj, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data_obj, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data_obj, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data_obj, "rz").toDouble();
        msg.seed = get_json(data_obj, "seed");
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_localization(msg);
    }
    else if(topic == "randomseq")
    {
        DATA_RANDOMSEQ msg;
        msg.command = get_json(data_obj, "command");
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_randomseq(msg);
    }
    else if(topic == "move")
    {
        DATA_MOVE msg;
        msg.command = get_json(data_obj, "command");
        msg.preset = get_json(data_obj, "preset").toInt();
        msg.method = get_json(data_obj, "method");
        msg.goal_node_id = get_json(data_obj, "goal_id");
        msg.tgt_pose_vec[0] = get_json(data_obj, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data_obj, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data_obj, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data_obj, "rz").toDouble()*D2R;
        msg.jog_val[0] = get_json(data_obj, "vx").toDouble();
        msg.jog_val[1] = get_json(data_obj, "vy").toDouble();
        msg.jog_val[2] = get_json(data_obj, "wz").toDouble();
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_move(msg);
    }
    else if(topic == "path")
    {
        DATA_PATH msg;
        msg.command = get_json(data_obj, "command");
        msg.path = get_json(data_obj, "path");
        msg.preset = get_json(data_obj, "preset").toInt();
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_path(msg);
    }
    else if(topic == "vobsRobots")
    {
        DATA_VOBS_R msg;
        msg.command = get_json(data_obj, "command");
        msg.vobs = get_json(data_obj, "vobs");
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_vobs_r(msg);
    }
    else if(topic == "vobsClosures")
    {
        DATA_VOBS_C msg;
        msg.command = get_json(data_obj, "command");
        msg.vobs = get_json(data_obj, "vobs");
        msg.time = get_json(data_obj, "time").toDouble()/1000;
        Q_EMIT recv_vobs_c(msg);
    }
}

void COMM_FMS::slot_load(DATA_LOAD msg)
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
                msg.message = "invalid map dir";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
                return;
            }

            slam->localization_stop();
            obsmap->clear();

            MainWindow* _main = (MainWindow*)main;
            _main->map_dir = load_dir;
            unimap->load_map(load_dir);
            _main->all_update();

            if(config->USE_LVX)
            {
                QString path_3d_map = load_dir + "/map.las";
                lvx->map_load(path_3d_map);
            }

            if(unimap->is_loaded)
            {
                msg.result = "success";
                msg.message = "";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
            }
            else
            {
                msg.result = "fail";
                msg.message = "map not load";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
            }
        }
    }
    else if(command == "topoload")
    {
        msg.result = "reject";
        msg.message = "not support yet";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "configload")
    {
        msg.result = "reject";
        msg.message = "not support yet";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
}

void COMM_FMS::slot_randomseq(DATA_RANDOMSEQ msg)
{
    QString command = msg.command;
    if(command == "randomseq")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_seq();

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
}

void COMM_FMS::slot_localization(DATA_LOCALIZATION msg)
{
    QString command = msg.command;
    if(command == "semiautoinit")
    {
        if(unimap->is_loaded == false)
        {
            msg.result = "reject";
            msg.message = "not loaded map";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }

        if(lidar->is_connected_f == false)
        {
            msg.result = "reject";
            msg.message = "not connected front lidar";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }

        #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
        if(lidar->is_connected_b == false)
        {
            msg.result = "reject";
            msg.message = "not connected back lidar";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }
        #endif

        if(slam->is_busy)
        {
            msg.result = "reject";
            msg.message = "already running";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }

        msg.result = "accept";
        msg.message = "";
        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());

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
        if(unimap->is_loaded == false)
        {
            msg.result = "reject";
            msg.message = "not loaded map";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }
        if(lidar->is_connected_f == false)
        {
            msg.result = "reject";
            msg.message = "not connected front lidar";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }

        #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
        if(lidar->is_connected_b == false)
        {
            msg.result = "reject";
            msg.message = "not connected back lidar";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
            return;
        }
        #endif

        // manual init
        double x = msg.tgt_pose_vec[0];
        double y = msg.tgt_pose_vec[1];
        double rz = msg.tgt_pose_vec[3];
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

        if(config->USE_LVX)
        {
            lvx->loc_start();
        }
        slam->localization_start();

        msg.result = "accept";
        msg.message = "";
        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "start")
    {
        if(config->USE_LVX)
        {
            lvx->loc_start();
        }
        slam->localization_start();

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "stop")
    {
        if(config->USE_LVX)
        {
            lvx->loc_stop();
        }
        slam->localization_stop();

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "randominit")
    {
        QString seed = msg.seed;

        MainWindow* _main = (MainWindow*)main;
        _main->slot_sim_random_init(seed);

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
}

void COMM_FMS::slot_move(DATA_MOVE msg)
{
    QString command = msg.command;
    if(command == "goal")
    {
        QString method = msg.method;
        if(method == "pp")
        {
            if(unimap->is_loaded == false)
            {
                msg.result = "reject";
                msg.message = "map not loaded";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
                return;
            }

            if(slam->is_loc == false)
            {
                msg.result = "reject";
                msg.message = "no localization";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
                return;
            }

            QString goal_id = msg.goal_node_id;
            if(goal_id == "")
            {
                msg.result = "reject";
                msg.message = "empty node id";

                printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                            msg.result.toLocal8Bit().data(),
                                                                            msg.message.toLocal8Bit().data());
                return;
            }

            NODE* node = unimap->get_node_by_id(goal_id);
            if(node == NULL)
            {
                node = unimap->get_node_by_name(goal_id);
                if(node == NULL)
                {
                    msg.result = "reject";
                    msg.message = "can not find node";

                    printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                                msg.result.toLocal8Bit().data(),
                                                                                msg.message.toLocal8Bit().data());
                    return;
                }

                msg.goal_node_id = node->id;
                msg.goal_node_name = node->name;
            }
            else
            {
                msg.goal_node_name = node->name;
            }

            Eigen::Matrix4d cur_tf = slam->get_cur_tf();
            msg.cur_pos = cur_tf.block(0,3,3,1);

            Eigen::Vector3d xi = TF_to_se2(node->tf);
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2,3);
            msg.tgt_pose_vec[3] = xi[2];

            // pure pursuit
            ctrl->move(msg);
            msg.result = "accept";
            msg.message = "";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
        }
        else
        {
            msg.result = "reject";
            msg.message = "not supported";

            printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                        msg.result.toLocal8Bit().data(),
                                                                        msg.message.toLocal8Bit().data());
        }
    }
    else if(command == "pause")
    {
        ctrl->is_pause = true;

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "resume")
    {
        ctrl->is_pause = false;

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
    else if(command == "stop")
    {
        MainWindow* _main = (MainWindow*)main;
        _main->bt_Emergency();

        msg.result = "accept";
        msg.message = "";

        printf("[COMM_FMS] command: %s, result: %s, message: %s\n", msg.command.toLocal8Bit().data(),
                                                                    msg.result.toLocal8Bit().data(),
                                                                    msg.message.toLocal8Bit().data());
    }
}

void COMM_FMS::slot_path(DATA_PATH msg)
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

        int preset = msg.preset;
        ctrl->move_pp(path, preset);
    }
}

void COMM_FMS::slot_vobs_r(DATA_VOBS_R msg)
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
        obsmap->mtx.lock();
        obsmap->vobs_list_robots = vobs_list;
        obsmap->mtx.unlock();

        obsmap->update_vobs_map();
    }
}

void COMM_FMS::slot_vobs_c(DATA_VOBS_C msg)
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
        obsmap->mtx.lock();
        obsmap->vobs_list_closures = vobs_list;
        obsmap->mtx.unlock();

        obsmap->update_vobs_map();
    }
}
