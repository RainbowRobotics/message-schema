#include "comm_fms.h"
#include "mainwindow.h"

COMM_FMS::COMM_FMS(QObject *parent)
    : QObject{parent}
    , reconnect_timer(this)
    , main(parent)
{
    connect(&client, &QWebSocket::connected, this, &COMM_FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_FMS::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));    

    connect(this, SIGNAL(signal_send_info()), this, SLOT(slot_send_info()));

    connect(this, SIGNAL(signal_move(DATA_MOVE)), this, SLOT(slot_move(DATA_MOVE)));
    connect(this, SIGNAL(signal_load(DATA_LOAD)), this, SLOT(slot_load(DATA_LOAD)));
    connect(this, SIGNAL(signal_localization(DATA_LOCALIZATION)), this, SLOT(slot_localization(DATA_LOCALIZATION)));
    connect(this, SIGNAL(signal_randomseq(DATA_RANDOMSEQ)), this, SLOT(slot_randomseq(DATA_RANDOMSEQ)));
    connect(this, SIGNAL(signal_path(DATA_PATH)), this, SLOT(slot_path(DATA_PATH)));
    connect(this, SIGNAL(signal_vobs_r(DATA_VOBS_R)), this, SLOT(slot_vobs_r(DATA_VOBS_R)));
    connect(this, SIGNAL(signal_vobs_c(DATA_VOBS_C)), this, SLOT(slot_vobs_c(DATA_VOBS_C)));
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
        connect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = true;
        printf("[COMM_FMS] connected\n");
    }
}

void COMM_FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::binaryMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = false;
        printf("[COMM_FMS] disconnected\n");
    }
}

// recv callback
void COMM_FMS::recv_message(const QByteArray &buf)
{
    QString message = qUncompress(buf);
    QJsonObject obj = QJsonDocument::fromJson(message.toUtf8()).object();
    if(get_json(obj, "robotSerial") != robot_id)
    {
        return;
    }

    QString body = get_json(obj, "body");
    QString data_str = get_json(obj, "data");
    QJsonObject dataObj = QJsonDocument::fromJson(data_str.toUtf8()).object();

    // parsing
    if(body == "load")
    {
        DATA_LOAD dload = get_data_load(dataObj);
        Q_EMIT signal_load(dload);
    }
    else if(body == "localization")
    {
        DATA_LOCALIZATION dloc = get_data_localization(dataObj);
        Q_EMIT signal_localization(dloc);
    }
    else if(body == "randomseq")
    {
        DATA_RANDOMSEQ drandomseq = get_data_randomseq(dataObj);
        Q_EMIT signal_randomseq(drandomseq);
    }
    else if(body == "move")
    {
        DATA_MOVE dmove = get_data_move(dataObj);
        Q_EMIT signal_move(dmove);
    }
    else if(body == "path")
    {
        DATA_PATH dpath = get_data_path(dataObj);
        Q_EMIT signal_path(dpath);
    }
    else if(body == "vobsRobots")
    {
        DATA_VOBS_R dvobs_r = get_data_vobs_r(dataObj);
        Q_EMIT signal_vobs_r(dvobs_r);
    }
    else if(body == "vobsClosures")
    {
        DATA_VOBS_C dvobs_c = get_data_vobs_c(dataObj);
        Q_EMIT signal_vobs_c(dvobs_c);
    }
}

DATA_MOVE COMM_FMS::get_data_move(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_MOVE dmove;
    dmove.command = get_json(dataObj, "command");
    if(dmove.command == "goal")
    {
        dmove.method = get_json(dataObj, "method");
        dmove.goal_node_id = get_json(dataObj, "goal_id");
        dmove.preset = get_json(dataObj, "preset").toInt();
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
        double vx = get_json(dataObj, "vx").toDouble();
        double vy = get_json(dataObj, "vy").toDouble();
        double wz = get_json(dataObj, "wz").toDouble();

        dmove.method = "";
        dmove.goal_node_id = "";
        dmove.preset = 0;
        dmove.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
        dmove.jog_val = Eigen::Vector3d(vx,vy,wz);
        dmove.time = time;
    }
    else if(dmove.command == "target")
    {
        double x = get_json(dataObj, "x").toDouble();
        double y = get_json(dataObj, "y").toDouble();
        double z = get_json(dataObj, "z").toDouble();
        double rz = get_json(dataObj, "rz").toDouble();

        dmove.method = get_json(dataObj, "method");
        dmove.goal_node_id = "";
        dmove.preset = get_json(dataObj, "preset").toInt();
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
        str.sprintf("[COMM_FMS] fail recv_move, not support command:%s, time:%.3f\n", dmove.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_MOVE();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_move, command:%s, time: %.3f\n", dmove.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_LOAD COMM_FMS::get_data_load(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_LOAD dload;
    dload.command = get_json(dataObj, "command");
    if(dload.command == "mapload")
    {
        dload.map_name = get_json(dataObj, "name");
        dload.time = time;
    }
    else if(dload.command == "topoload")
    {
        dload.map_name = get_json(dataObj, "name");
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
        str.sprintf("[COMM_FMS] fail recv_load, not support command:%s, time:%.3f\n", dload.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_LOAD();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_load, command:%s, time: %.3f\n", dload.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_LOCALIZATION COMM_FMS::get_data_localization(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_LOCALIZATION dloc;
    dloc.command = get_json(dataObj, "command");
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
        double x = get_json(dataObj, "x").toDouble();
        double y = get_json(dataObj, "y").toDouble();
        double z = get_json(dataObj, "z").toDouble();
        double rz = get_json(dataObj, "rz").toDouble();

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
        dloc.seed = get_json(dataObj, "seed");
        dloc.tgt_pose_vec = Eigen::Vector4d(0,0,0,0);
        dloc.time = time;
    }
    else
    {
        QString str;
        str.sprintf("[COMM_FMS] fail recv_localization, command:%s, time:%.3f\n", dloc.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_LOCALIZATION();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_localization, command:%s, time: %.3f\n", dloc.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_RANDOMSEQ COMM_FMS::get_data_randomseq(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_RANDOMSEQ drandomseq;
    drandomseq.command = get_json(dataObj, "command");
    if(drandomseq.command == "randomseq")
    {
        drandomseq.time = time;
    }
    else
    {
        QString str;
        str.sprintf("[COMM_FMS] fail recv_randomseq, command:%s, time:%.3f\n", drandomseq.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_RANDOMSEQ();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_randomseq, command:%s, time: %.3f\n", drandomseq.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_PATH COMM_FMS::get_data_path(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_PATH dpath;
    dpath.command = get_json(dataObj, "command");
    if(dpath.command == "path")
    {
        dpath.path = get_json(dataObj, "path");
        dpath.preset = get_json(dataObj, "preset").toInt();
        dpath.time = time;
    }
    else
    {
        QString str;
        str.sprintf("[COMM_FMS] fail recv_path, command:%s, time:%.3f\n", dpath.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_PATH();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_dpath, command:%s, time: %.3f\n", dpath.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_VOBS_R COMM_FMS::get_data_vobs_r(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_VOBS_R dvobs_r;
    dvobs_r.command = get_json(dataObj, "command");
    if(dvobs_r.command == "vobs_robots")
    {
        dvobs_r.vobs = get_json(dataObj, "vobs");
        dvobs_r.time = time;
    }
    else
    {
        QString str;
        str.sprintf("[COMM_FMS] fail recv_vobs_r, command:%s, time:%.3f\n", dvobs_r.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_VOBS_R();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_vobs_r, command:%s, time: %.3f\n", dvobs_r.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

DATA_VOBS_C COMM_FMS::get_data_vobs_c(QJsonObject dataObj)
{
    // parsing
    double time = get_json(dataObj, "time").toDouble()/1000;

    DATA_VOBS_C dvobs_c;
    dvobs_c.command = get_json(dataObj, "command");
    if(dvobs_c.command == "vobs_closures")
    {
        dvobs_c.vobs = get_json(dataObj, "vobs");
        dvobs_c.time = time;
    }
    else
    {
        QString str;
        str.sprintf("[COMM_FMS] fail recv_vobs_c, command:%s, time:%.3f\n", dvobs_c.command.toLocal8Bit().data(), time);
        logger->write_log(str, "Red");
        return DATA_VOBS_C();
    }

    QString str;
    str.sprintf("[COMM_FMS] success recv_vobs_c, command:%s, time: %.3f\n", dvobs_c.command.toLocal8Bit().data(), time);
    logger->write_log(str, "Green");
}

// recv slots
void COMM_FMS::slot_move(DATA_MOVE dmove)
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

void COMM_FMS::slot_load(DATA_LOAD dload)
{
    QString command = dload.command;
    if(command == "mapload")
    {
        QString map_name = dload.map_name;
        MainWindow* _main = (MainWindow*)main;

        QString load_dir = QDir::homePath() + "/maps/" + map_name;
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

            if(config->USE_LVX)
            {
                QString path_3d_map = load_dir + "/map.las";
                lvx->map_load(path_3d_map);
            }

            _main->all_update();
            _main->set_mapping_view();

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

void COMM_FMS::slot_localization(DATA_LOCALIZATION dloc)
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

void COMM_FMS::slot_randomseq(DATA_RANDOMSEQ drandomseq)
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

// send slots
void COMM_FMS::slot_send_info()
{
    if(!is_connected)
    {
        return;
    }

    MainWindow* _main = (MainWindow*)main;
    _main->mtx.lock();
    QString cur_node_id = _main->last_node_id;
    _main->mtx.unlock();

    double time = get_time();
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();        
    Eigen::Matrix4d goal_tf = ctrl->get_cur_goal_tf();
    QString goal_node_id = unimap->get_node_id_edge(goal_tf.block(0,3,3,1));
    if(goal_tf.isIdentity())
    {
        goal_tf = cur_tf;
        goal_node_id = cur_node_id;
    }

    QString request = ctrl->get_multi_req(); // none, req_path, recv_path

    QString state = "stop"; // stop, move, pause, error
    if(ctrl->is_moving)
    {
        state = "move";
    }

    if(mobile->get_cur_pdu_state() != "good")
    {
        state = "pause";
    }

    if(slam->get_cur_loc_state() != "good")
    {
        state = "error";
    }

    mtx.lock();
    multi_state = state;
    mtx.unlock();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["command"] = "info";
    rootObj["robot_id"] = robot_id;
    rootObj["cur_tf"] = TF_to_string(cur_tf);
    rootObj["goal_tf"] = TF_to_string(goal_tf);
    rootObj["cur_node_id"] = cur_node_id;
    rootObj["goal_node_id"] = goal_node_id;
    rootObj["request"] = request;
    rootObj["state"] = state;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    if(time - last_send_time >= 0.05)
    {
        QJsonDocument doc(rootObj);
        QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
        client.sendBinaryMessage(buf);
        last_send_time = time;
    }
}

void COMM_FMS::slot_path(DATA_PATH dpath)
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

void COMM_FMS::slot_vobs_r(DATA_VOBS_R dvobs_r)
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

void COMM_FMS::slot_vobs_c(DATA_VOBS_C dvobs_c)
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

void COMM_FMS::send_move_response(DATA_MOVE dmove)
{
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
    QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
}

void COMM_FMS::send_localization_response(DATA_LOCALIZATION dloc)
{
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
    QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
}

void COMM_FMS::send_load_response(DATA_LOAD dload)
{
    QJsonObject loadResponseObj;
    loadResponseObj["command"] = dload.command;
    loadResponseObj["result"] = dload.result;
    loadResponseObj["message"] = dload.message;
    loadResponseObj["name"] = dload.map_name;
    loadResponseObj["time"] = QString::number((long long)(dload.time*1000), 10);

    QJsonDocument doc(loadResponseObj);
    QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
}

void COMM_FMS::send_randomseq_response(DATA_RANDOMSEQ drandomseq)
{
    QJsonObject randomseqResponseObj;
    randomseqResponseObj["command"] = drandomseq.command;
    randomseqResponseObj["result"] = drandomseq.result;
    randomseqResponseObj["message"] = drandomseq.message;
    randomseqResponseObj["time"] = QString::number((long long)(drandomseq.time*1000), 10);

    QJsonDocument doc(randomseqResponseObj);
    QByteArray buf = qCompress(doc.toJson(QJsonDocument::Compact));
    client.sendBinaryMessage(buf);
}
