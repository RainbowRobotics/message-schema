#include "comm_ui.h"
#include "mainwindow.h"

COMM_UI::COMM_UI(QObject *parent)
    : QObject{parent}
    , main(parent)
    , reconnect_timer(this)
{
    // for websocket
    connect(&client, &QWebSocket::connected, this, &COMM_UI::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_UI::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));

    // connect recv signals -> recv slots
    connect(this, SIGNAL(signal_motorinit(double)), this, SLOT(slot_motorinit(double)));

    connect(this, SIGNAL(signal_move_jog(double, double, double, double)), this, SLOT(slot_move_jog(double, double, double, double)));
    connect(this, SIGNAL(signal_move_target(double, double, double, double, double, int, QString)), this, SLOT(slot_move_target(double, double, double, double, double, int, QString)));
    connect(this, SIGNAL(signal_move_goal(double, QString, int, QString, Eigen::Vector3d)), this, SLOT(slot_move_goal(double, QString, int, QString, Eigen::Vector3d)));
    connect(this, SIGNAL(signal_move_docking(double, QString, double, double, double)), this, SLOT(slot_move_docking(double, QString, double, double, double)));
    connect(this, SIGNAL(signal_move_pause(double)), this, SLOT(slot_move_pause(double)));
    connect(this, SIGNAL(signal_move_resume(double)), this, SLOT(slot_move_resume(double)));
    connect(this, SIGNAL(signal_move_stop(double)), this, SLOT(slot_move_stop(double)));

    connect(this, SIGNAL(signal_mapping_start(double)), this, SLOT(slot_mapping_start(double)));
    connect(this, SIGNAL(signal_mapping_stop(double)), this, SLOT(slot_mapping_stop(double)));
    connect(this, SIGNAL(signal_mapping_save(double, QString)), this, SLOT(slot_mapping_save(double, QString)));
    connect(this, SIGNAL(signal_mapping_reload(double)), this, SLOT(slot_mapping_reload(double)));

    connect(this, SIGNAL(signal_mapload(double, QString)), this, SLOT(slot_mapload(double, QString)));

    connect(this, SIGNAL(signal_localization_autoinit(double)), this, SLOT(slot_localization_autoinit(double)));
    connect(this, SIGNAL(signal_localization_semiautoinit(double)), this, SLOT(slot_localization_semiautoinit(double)));
    connect(this, SIGNAL(signal_localization_init(double, double, double, double, double)), this, SLOT(slot_localization_init(double, double, double, double, double)));
    connect(this, SIGNAL(signal_localization_start(double)), this, SLOT(slot_localization_start(double)));
    connect(this, SIGNAL(signal_localization_stop(double)), this, SLOT(slot_localization_stop(double)));

    connect(this, SIGNAL(signal_sync_infos(double, QString,QString, QString, QString)), this, SLOT(slot_sync_infos(double, QString,QString,QString,QString)));
    connect(this, SIGNAL(signal_adjustment_config(double, QString, QString)), this, SLOT(slot_adjustment_config(double, QString, QString)));

}

COMM_UI::~COMM_UI()
{
    reconnect_timer.stop();

    if(is_connected)
    {
        client.close();
    }
}

QString COMM_UI::get_json(QJsonObject& json, QString key)
{
    return json[key].toString();
}

void COMM_UI::init()
{
    if(config->USE_QTUI)
    {
        reconnect_timer.start(3000);
        printf("[COMM_UI] start reconnect timer\n");
    }
}

void COMM_UI::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr = "ws://192.168.3.4:13359";
        //QString server_addr = "ws://127.0.0.1:13359";
        client.open(QUrl(server_addr));

        /*
        reconnect_cnt++;
        if(reconnect_cnt > 5)
        {
            reconnect_timer.stop();
            printf("[COMM_UI] server not opened, give up reconnect\n");
        }
        */
    }
}

void COMM_UI::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        connect(&client, &QWebSocket::textMessageReceived, this, &COMM_UI::recv_message);
        printf("[COMM_UI] connected\n");
    }
}

void COMM_UI::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::textMessageReceived, this, &COMM_UI::recv_message);
        printf("[COMM_UI] disconnected\n");
    }
}

void COMM_UI::recv_message(QString message)
{
    std::cout << "[COMM_UI] recv:\n" << message.toStdString() << std::endl;

    QJsonObject data = QJsonDocument::fromJson(message.toUtf8()).object();

    // recv moving
    if(get_json(data, "type") == "motorinit")
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        // action
        Q_EMIT signal_motorinit(time);

        printf("[COMM_UI] motorinit(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }

    // recv move
    if(get_json(data, "type") == "move")
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
            printf("[COMM_UI] move, jog, t: %.3f, vel: %.3f, %.3f, %.3f\n", time, vx, vy, wz);
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
            printf("[COMM_UI] move, target, t: %.3f, tgt: %.3f, %.3f, %.3f, %.3f, preset:%d, method:%s\n", time, x, y, z, rz, preset, method.toLocal8Bit().data());
        }
        else if(command == "goal")
        {
            QString id = get_json(data, "id");
            int preset = get_json(data, "preset").toInt();
            QString method = get_json(data, "method");
            double offset_x = get_json(data, "docking_offset_x").toDouble();
            double offset_y = get_json(data, "docking_offset_y").toDouble();
            double offset_th = get_json(data, "docking_offset_th").toDouble();
            double time = get_json(data, "time").toDouble()/1000;

            // for response
            MOVE_INFO _last_move_info;
            _last_move_info.command = command;
            _last_move_info.node_id = id;
            _last_move_info.preset = preset;
            _last_move_info.method = method;
            _last_move_info.docking_offset_x = offset_x;
            _last_move_info.docking_offset_y = offset_y;
            _last_move_info.docking_offset_th = offset_th;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();

            // action
            Q_EMIT signal_move_goal(time, id, preset, method, Eigen::Vector3d(offset_x, offset_y, offset_th));
            printf("[COMM_UI] move, goal, t: %.3f, tgt: %s, preset:%d, method:%s\n", time, id.toLocal8Bit().data(), preset, method.toLocal8Bit().data());
        }
        else if(command == "docking")
        {
            QString method = get_json(data, "method");
            double offset_x = get_json(data, "offset_x").toDouble();
            double offset_y = get_json(data, "offset_y").toDouble();
            double offset_th = get_json(data, "offset_th").toDouble();
            double time = get_json(data, "time").toDouble()/1000;

            MOVE_INFO _last_move_info;
            _last_move_info.command = command;
            _last_move_info.method = method;

            mtx.lock();
            last_move_info = _last_move_info;
            mtx.unlock();

            // action
            Q_EMIT signal_move_docking(time, method, offset_x, offset_y, offset_th);
            printf("[COMM_UI] move, docking, t: %.3f, method:%s, offset(x,y,th): %.3f, %.3f, %.3f\n", time, method.toLocal8Bit().data(), offset_x, offset_y, offset_th);
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

    // recv mapping
    if(get_json(data, "type") == "mapping")
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

        printf("[COMM_UI] mapping(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }

    // recv mapload
    if(get_json(data, "type") == "mapload")
    {
        // parsing
        QString name = get_json(data, "name");
        double time = get_json(data, "time").toDouble()/1000;

        Q_EMIT signal_mapload(time, name);

        printf("[COMM_UI] mapload(%s), t: %.3f\n", name.toLocal8Bit().data(), time);
    }

    // recv localization
    if(get_json(data, "type") == "localization")
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        if(command == "autoinit")
        {
            Q_EMIT signal_localization_autoinit(time);
        }
        else if(command == "semiautoinit")
        {
            Q_EMIT signal_localization_semiautoinit(time);
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

        printf("[COMM_UI] localization(%s), t: %.3f\n", command.toLocal8Bit().data(), time);
    }

    // recv localization
    if(get_json(data, "type") == "sync")
    {
        // parsing
        QString command = get_json(data, "command");
        QString dir = get_json(data, "directory");
        QString user = get_json(data, "user");
        QString ip = get_json(data, "ip");
        QString password = get_json(data, "password");

        double time = get_json(data, "time").toDouble()/1000;

        if(command == "infos")
        {
            Q_EMIT signal_sync_infos(time, dir,user, ip, password);
        }
    }

    if(get_json(data, "type") == "view_change")
    {
        // parsing
        double time = get_json(data, "time").toDouble()/1000;

        QString view_control_command = get_json(data, "view_control");
        QString view_type_command = get_json(data, "view_type");

        if(view_control_command.size() != 0)
        {
            Q_EMIT signal_comm_ui_view_control(time, view_control_command);
        }
        else if(view_type_command.size() !=0)
        {
            Q_EMIT signal_comm_ui_view_type(time, view_type_command);
        }
    }

    if(get_json(data, "type") == "change_config")
    {
        // parsing
        QString command = get_json(data, "command");
        double time = get_json(data, "time").toDouble()/1000;

        QString config_val = get_json(data, "configs");
        QString val = get_json(data, "val");

        if(command == "configs")
        {
            Q_EMIT signal_adjustment_config(time, config_val, val);
        }
    }
}

// recv slots
void COMM_UI::slot_motorinit(double time)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MotorInit();
}

void COMM_UI::slot_move_jog(double time, double vx, double vy, double wz)
{
    MainWindow* _main = (MainWindow*)main;
    _main->update_jog_values(vx, vy, wz*D2R);
}

void COMM_UI::slot_move_target(double time, double x, double y, double z, double rz, int preset, QString method)
{
    QString result = "reject";
    QString message = "not supported";
    send_move_target_response(x, y, z, rz, preset, method, result, message);
}

void COMM_UI::slot_move_goal(double time, QString node_id, int preset, QString method, Eigen::Vector3d offset_val)
{
    if(method == "hpp")
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
        if(node == NULL)
        {
            node = unimap->get_node_by_name(node_id);
            if(node == NULL)
            {
                QString result = "reject";
                QString message = "invalid node id";
                send_move_goal_response(node_id, preset, method, result, message);
                return;
            }
        }

        double ref_offset_x = 0.0, ref_offset_y = 0.0, ref_offset_th = 0.0;
        QString info = node->info;
        NODE_INFO res;
        if(parse_info(info, "BQR_OFFSET", res))
        {
            ref_offset_x = res.bqr_code_offset[0];
            ref_offset_y = res.bqr_code_offset[1];
            ref_offset_th = res.bqr_code_offset[2];

            if(ref_offset_x != offset_val[0] || ref_offset_y != offset_val[1] || ref_offset_th != offset_val[2])
            {
                NODE_INFO res1;
                if(parse_info(info, "BQR_CODE_NUM", res1))
                {
                    int bqr_code_num = res1.bqr_code_num;
                    if(bqr_code_num > 0)
                    {
                        QString id = node->id;
                        QString new_info;
                        new_info.sprintf("BQR_CODE_NUM,%d\nBQR_OFFSET,%f,%f,%f", bqr_code_num, offset_val[0], offset_val[1], offset_val[2]);

                        unimap->edit_node_info(id, new_info);
                        unimap->save_annotation();
                    }
                }
            }
        }

        dctrl->set_node_type(node->type);
        dctrl->set_dock_offset(offset_val);

        Eigen::Matrix4d tf = node->tf;
        Eigen::Vector3d pos = tf.block(0,3,3,1);
        Eigen::Vector3d xi = TF_to_se2(tf);

        DATA_MOVE msg;
        msg.command = "goal";
        msg.method = "hpp";
        msg.tgt_pose_vec = Eigen::Vector4d(pos[0], pos[1], pos[2], xi[2]);
        msg.goal_node_id = node->id;
        msg.goal_node_name = node->name;
        msg.time = get_time();
        msg.preset = 0;

        // pure pursuit
        ctrl->move(msg);

        QString result = "accept";
        QString message = "";
        send_move_goal_response(node_id, preset, method, result, message);
    }
    else
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_goal_response(node_id, preset, method, result, message);
    }
}

void COMM_UI::slot_move_docking(double time, QString method, double offset_x, double offset_y, double offset_th)
{
    if(std::abs(offset_x) > 0.03 || std::abs(offset_y) > 0.03)
    {
        QString result = "reject";
        QString message = "offset x,y must lower 3cm";
        send_move_docking_response(method, offset_x, offset_y, offset_th, result, message);
    }

    if(method == "bqr")
    {
        double p_gain = 1.0;
        double d_gain = 1.0;
        dctrl->move(p_gain, d_gain, offset_x, offset_y, offset_th);

        QString result = "accept";
        QString message = "";
        send_move_docking_response(method, offset_x, offset_y, offset_th, result, message);
    }
    else
    {
        QString result = "reject";
        QString message = "not supported";
        send_move_docking_response(method, offset_x, offset_y, offset_th, result, message);
    }
}

void COMM_UI::slot_move_pause(double time)
{
    ctrl->is_pause = true;

    QString result = "accept";
    send_move_pause_response(result);
}

void COMM_UI::slot_move_resume(double time)
{
    ctrl->is_pause = false;

    QString result = "accept";
    send_move_resume_response(result);
}

void COMM_UI::slot_move_stop(double time)
{
    ctrl->stop();

    QString result = "accept";
    send_move_stop_response(result);
}

void COMM_UI::slot_move_succeed(QString message)
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

void COMM_UI::slot_move_failed(QString message)
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

void COMM_UI::slot_mapping_start(double time)
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

void COMM_UI::slot_mapping_stop(double time)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MapSave();
    send_mapping_stop_response();
}

void COMM_UI::slot_mapping_save(double time, QString name)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_MapSave();

    QString save_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + name;
    std::string command = "cp -r " + _main->map_dir.toStdString() + " " + save_dir.toStdString();
    int result = std::system(command.c_str());
    if(result == 0)
    {
        send_mapping_save_response(name, "success");
        printf("[COMM_UI] map save succeed, %s\n", save_dir.toLocal8Bit().data());
    }
    else
    {
        send_mapping_save_response(name, "fail");
        printf("[COMM_UI] map save failed, %s\n", save_dir.toLocal8Bit().data());
    }
}

void COMM_UI::slot_mapping_reload(double time)
{
    last_send_kfrm_idx = 0;
    printf("[COMM_UI] mapping reload\n");
}

void COMM_UI::slot_mapload(double time, QString name)
{
    MainWindow* _main = (MainWindow*)main;

    QString load_dir = QStandardPaths::writableLocation(QStandardPaths::HomeLocation) + "/maps/" + name;
    if(!load_dir.isNull())
    {
        if(slam->is_busy)
        {
            send_mapload_response(name, "fail");
            return;
        }

        if(_main->semi_auto_init_thread != NULL)
        {
            semi_auto_init_flag = false;
            semi_auto_init_thread->join();
            semi_auto_init_thread = NULL;
        }

        _main->bqr_localization_timer.stop();
        slam->localization_stop();
        obsmap->clear();

        _main->map_dir = load_dir;
        unimap->load_map(load_dir);
        _main->all_update();

        if(unimap->is_loaded)
        {
            send_mapload_response(name, "success");
            _main->bqr_localization_timer.start(1000);
        }
        else
        {
            send_mapload_response(name, "fail");
        }
    }
}

void COMM_UI::slot_localization_autoinit(double time)
{
    send_localization_response("autoinit", "fail");
}

void COMM_UI::slot_localization_init(double time, double x, double y, double z, double rz)
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

void COMM_UI::slot_localization_semiautoinit(double time)
{
    if(slam->is_busy)
    {
        send_localization_response("semiautoinit", "fail");
        return;
    }

    logger->write_log("[AUTO_INIT] try semi-auto init", "Green", true, false);
    slam->localization_stop();

    // semi auto init
    if(semi_auto_init_thread != NULL)
    {
        logger->write_log("[AUTO_INIT] thread already running.", "Orange", true, false);
        semi_auto_init_thread->join();
        semi_auto_init_thread = NULL;
    }

    semi_auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start, slam);
    return;
}

void COMM_UI::slot_localization_start(double time)
{
    slam->localization_start();
}

void COMM_UI::slot_localization_stop(double time)
{
    slam->localization_stop();
}

void COMM_UI::slot_sync_infos(double time, QString dir,QString user, QString ip, QString password)
{
    if(!is_connected)
    {
        return;
    }

    QFileInfo map_info("/home/rainbow/maps");
    if (!map_info.exists() || !map_info.isDir())
    {
        send_sync_infos_response("fail", "invalid path");
        return;
    }

    std::string destination_path = dir.toStdString();

    QString command;
    command.append("sshpass -p '");
    command.append(password);
    command.append("' scp -r -o StrictHostKeyChecking=no -P 22 ");
    command.append("/home/rainbow/maps");
    command.append(" ");
    command.append(user+"@");
    command.append(ip+":");

    std::string command2 = command.toStdString();
    command2.append(destination_path);

    std::system(command2.c_str());
    printf("[COMM_UI] sync_infos, success, time: %f\n", time);

    send_sync_infos_response("success", "");
}

void COMM_UI::slot_adjustment_config(double time, QString config_val, QString val)
{
    if(config_val == "DOCKING_EXTENDED_CONTROL_TIME")
    {
        double _val = val.toDouble();
        if(_val < 0 || _val > 60.0)
        {
            send_adjustment_config_response("fail", "value is invalid");
            return;
        }

        config->DOCK_EXTENDED_CONTROL_TIME = _val;
        printf("[COMM_UI] adjustment_config, DOCKING_EXTENDED_CONTROL_TIME success, time: %f, val: %f\n", time, _val);
        send_adjustment_config_response("success", "DOCKING_EXTENDED_CONTROL_TIME");
    }
    else if(config_val == "OBS_DEADZONE")
    {
        double _val = val.toDouble();
        if(_val < 0.1 || _val > 0.5)
        {
            send_adjustment_config_response("fail", "value is invalid");
            return;
        }

        config->OBS_DEADZONE_DYN = _val;
        printf("[COMM_UI] adjustment_config, OBS_DEADZONE success, time: %f, val: %f\n", time, _val);
        send_adjustment_config_response("success", "OBS_DEADZONE");
    }
    else if(config_val == "OBS_AVOID")
    {
        double _val = val.toInt();
        if(_val != 0 || _val != 1)
        {
            send_adjustment_config_response("fail", "value is not 0 or 1");
            return;
        }

        config->OBS_AVOID = _val;
        printf("[COMM_UI] adjustment_config, OBS_DEADZONE success, time: %f, val: %d\n", time, _val);
        send_adjustment_config_response("success", "OBS_AVOID");
    }
}

// send functions
void COMM_UI::send_status()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "status";
    rootObj["command"] = "status";
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // Adding the pose object
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);

    QJsonObject poseObj;
    poseObj["x"] = QString::number(cur_xi[0], 'f', 4);
    poseObj["y"] = QString::number(cur_xi[1], 'f', 4);
    poseObj["rz"] = QString::number(cur_xi[2]*R2D, 'f', 4);
    rootObj["pose"] = poseObj;

    // Adding the velocity object
    MOBILE_POSE mo = mobile->get_pose();

    QJsonObject velObj;
    velObj["vx"] = QString::number(mo.vel[0], 'f', 4);
    velObj["vy"] = QString::number(mo.vel[1], 'f', 4);
    velObj["wz"] = QString::number(mo.vel[2]*R2D, 'f', 4);
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
    motorObj2["current"] = QString::number((double)ms.cur_m1/10.0, 'f', 3);
    motorArray.append(motorObj2);

    #if defined(USE_MECANUM)
    QJsonObject motorObj3;
    motorObj3["connection"] = (ms.connection_m2 == 1) ? "true" : "false";
    motorObj3["status"] = QString::number(ms.status_m2, 10);
    motorObj3["temp"] = QString::number(ms.temp_m2, 'f', 3);
    motorObj3["current"] = QString::number((double)ms.cur_m2/10.0, 'f', 3);
    motorArray.append(motorObj3);

    QJsonObject motorObj4;
    motorObj4["connection"] = (ms.connection_m3 == 1) ? "true" : "false";
    motorObj4["status"] = QString::number(ms.status_m3, 10);
    motorObj4["temp"] = QString::number(ms.temp_m3, 'f', 3);
    motorObj4["current"] = QString::number((double)ms.cur_m3/10.0, 'f', 3);
    motorArray.append(motorObj4);
    #endif

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
    #if defined(USE_D400) || defined(USE_D400_LAKI) || defined(USE_MECANUM)
    powerObj["charge_current"] = QString::number(ms.charge_current, 'f', 3);
    powerObj["contact_voltage"] = QString::number(ms.contact_voltage, 'f', 3);
    #endif
    #if defined(USE_S100) || defined(USE_MECANUM_OLD)
    powerObj["charge_current"] = QString::number(0, 'f', 3);
    powerObj["contact_voltage"] = QString::number(0, 'f', 3);
    #endif
    rootObj["power"] = powerObj;

    // Adding the state object
    QString cur_loc_state = slam->get_cur_loc_state();

    QJsonObject stateObj;
    stateObj["power"] = (ms.power_state == 1) ? "true" : "false";
    stateObj["emo"] = (ms.motor_stop_state == 1) ? "true" : "false";
    stateObj["charge"] = QString::number(ms.charge_state, 10);
    stateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    stateObj["map"] = unimap->map_dir.split("/").last();
    rootObj["state"] = stateObj;

    QJsonObject codeObj;
    codeObj["detect"] = (bqr->is_recv_data == true) ? "true" : "false";
    codeObj["code_num"] = QString::number(bqr->code_num);
    codeObj["err_x"] = QString::number(bqr->err_x, 'f', 4);
    codeObj["err_y"] = QString::number(bqr->err_y, 'f', 4);
    codeObj["err_th"] = QString::number(bqr->err_th*R2D, 'f', 4);
    rootObj["code"] = codeObj;

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

    QString dock_state = "stop";
    if(dctrl->is_pause)
    {
        dock_state = "pause";
    }
    else if(dctrl->is_moving)
    {
        dock_state = "move";
    }

    bool is_moving = (std::abs(mo.vel[0]) > 0.05 || std::abs(mo.vel[1]) > 0.05 || std::abs(mo.vel[2]) > 0.1*D2R);

    QString moving_state = "";
    if(ctrl->is_moving == true)
    {
        moving_state = "auto";
    }
    else if(dctrl->is_moving == true)
    {
        moving_state = "docking";
    }
    else if(ctrl->is_moving == false && dctrl->is_moving == false && is_moving == true)
    {
        moving_state = "manual";
    }
    else
    {
        moving_state = "none";
    }

    QJsonObject conditionObj;
    conditionObj["inlier_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"] = QString::number(ieir[1], 'f', 3);
    conditionObj["mapping_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["mapping_ratio"] = QString::number(ieir[1], 'f', 3);
    conditionObj["auto_state"] = auto_state;
    conditionObj["docking_state"] = dock_state;
    conditionObj["moving_state"] = moving_state;
    conditionObj["obs_state"] = ctrl->get_obs_condition();
    rootObj["condition"] = conditionObj;

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

void COMM_UI::send_img(const cv::Mat& img)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    QByteArray IMGByte((char*)(img.data), FIXED_SEND_IMG_HEIGHT*FIXED_SEND_IMG_WIDTH*3);
    client.sendBinaryMessage(IMGByte);
}

void COMM_UI::send_global_path()
{
    if(!is_connected)
    {
        return;
    }

    PATH path = ctrl->get_cur_global_path();

    QJsonArray arr_path_x;
    QJsonArray arr_path_y;
    QJsonArray arr_path_th;

    for(size_t p = 0; p < path.pos.size(); p++)
    {
        Eigen::Vector3d P = path.pos[p];

        arr_path_x.append(P[0]);
        arr_path_y.append(P[1]);
        arr_path_th.append(P[2]);
    }

    QJsonObject rootObj;
    rootObj["command"] = "Global_PATH";
    rootObj["path_x"] = arr_path_x;
    rootObj["path_y"] = arr_path_y;
    rootObj["path_th"] = arr_path_th;

    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

void COMM_UI::send_local_path()
{
    if(!is_connected)
    {
        return;
    }

    PATH path = ctrl->get_cur_local_path();

    QJsonArray arr_path_x;
    QJsonArray arr_path_y;
    QJsonArray arr_path_th;

    QJsonArray arr_path_pose;

    for(size_t p = 0; p < path.pos.size(); p++)
    {
        Eigen::Vector3d P = path.pos[p];

        arr_path_x.append(P[0]);
        arr_path_y.append(P[1]);
        arr_path_th.append(P[2]);
    }

    // Serialize path poses into JSON arrays (for 4x4 matrix)
    for (size_t p = 0; p < path.pose.size(); p++)
    {
        Eigen::Matrix4d mat_local = path.pose[p];
        QJsonArray mat_array;

        // Convert matrix to a QJsonArray
        for (int i = 0; i < 4; i++)
        {
            QJsonArray row_array;
            for (int j = 0; j < 4; j++)
            {
                row_array.append(mat_local(i, j));  // Append matrix element
            }
            mat_array.append(row_array);  // Append row array to matrix array
        }

        arr_path_pose.append(mat_array);  // Append the full matrix to the pose array
    }

    QJsonObject rootObj;
    rootObj["command"] = "Local_PATH";
    rootObj["path_x"] = arr_path_x;
    rootObj["path_y"] = arr_path_y;
    rootObj["path_th"] = arr_path_th;
    rootObj["path_pose"] = arr_path_pose;

    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

void COMM_UI::send_mapping_start_response(QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "mapping";
    rootObj["command"] = "start";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] mapping_response_start, time: %f\n", time);
}

void COMM_UI::send_mapping_stop_response()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "mapping";
    rootObj["command"] = "stop";
    rootObj["result"] = "success";
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] mapping_response_stop, success, time: %f\n", time);
}

void COMM_UI::send_mapping_save_response(QString name, QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "mapping";
    rootObj["command"] = "save";
    rootObj["name"] = name;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] mapping_response_save, %s, %s, time: %f\n", name.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_UI::send_mapload_response(QString name, QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "mapload";
    rootObj["name"] = name;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] mapload_response, %s, %s, time: %f\n", name.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_UI::send_localization_response(QString command, QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "localization";
    rootObj["command"] = command;
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] localization_response, %s, %s, time: %f\n", command.toLocal8Bit().data(), result.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_target_response(double x, double y, double z, double rz, int preset, QString method, QString result, QString message)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
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
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_target_response, %s, %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_goal_response(QString node_id, int preset, QString method, QString result, QString message)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
    rootObj["command"] = "goal";
    rootObj["id"] = node_id;
    rootObj["preset"] = QString::number(preset, 10);
    rootObj["method"] = method;
    rootObj["result"] = result;
    rootObj["message"] = message; // when result failed
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_goal_response, %s, %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_docking_response(QString method, double offset_x, double offset_y, double offset_th, QString result, QString message)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
    rootObj["command"] = "docking";
    rootObj["method"] = method;
    rootObj["result"] = result;
    rootObj["message"] = message; // when result failed
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_docking_response, %s, %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_pause_response(QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
    rootObj["command"] = "pause";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_pause_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_resume_response(QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
    rootObj["command"] = "resume";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_resume_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}

void COMM_UI::send_move_stop_response(QString result)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "move";
    rootObj["command"] = "stop";
    rootObj["result"] = result;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] move_stop_response, %s, time: %f\n", result.toLocal8Bit().data(), time);
}

void COMM_UI::send_sync_infos_response(QString result, QString message)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "sync";
    rootObj["result"] = result;
    rootObj["message"] = message;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] sync_infos_response, %s, msg: %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}

void COMM_UI::send_adjustment_config_response(QString result, QString message)
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time();

    // Creating the JSON object
    QJsonObject rootObj;

    // Adding the command and time
    rootObj["type"] = "change_config";
    rootObj["result"] = result;
    rootObj["message"] = message;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);

    printf("[COMM_UI] adjustment_config_response, %s, msg: %s, time: %f\n", result.toLocal8Bit().data(), message.toLocal8Bit().data(), time);
}
