#include "comm_old.h"

COMM_OLD::COMM_OLD(QObject *parent)
    : QObject{parent}
    , cmd_server(this)
    , data_server(this)
    , auto_timer(this)
    , docking_timer(this)
    , cmd_timer(this)
{
    connect(this,           &COMM_OLD::signal_cmd_recv,   this,  &COMM_OLD::slot_cmd_recv);
    connect(&cmd_server,    &QTcpServer::newConnection,   this,  &COMM_OLD::cmd_connected);
    connect(&data_server,   &QTcpServer::newConnection,   this,  &COMM_OLD::data_connected);
    connect(&docking_timer, &QTimer::timeout,             this,  &COMM_OLD::docking_loop);
    connect(&auto_timer,    &QTimer::timeout,             this,  &COMM_OLD::auto_loop);
    connect(&cmd_timer,     &QTimer::timeout,             this,  &COMM_OLD::cmd_loop);
    cmd_timer.start(100);
}

COMM_OLD::~COMM_OLD()
{
    if(is_cmd_connected == true)
    {
        cmd_disconnected();
    }

    if(is_data_connected == true)
    {
        mtx_data_client.lock();
        auto it = data_clients.begin();
        while (it != data_clients.end())
        {
            it->second->deleteLater();
            it = data_clients.erase(it);
        }
        mtx_data_client.unlock();

        data_client_cnt = 0;
        logger->write_log("[KAI] data socket disconnected", "Red");
    }

    cmd_server.close();
    data_server.close();

    if(auto_init_thread != NULL)
    {
        auto_init_flag = false;
        auto_init_thread->join();
        auto_init_thread = NULL;
    }
}

void COMM_OLD::open()
{
    // check simulation mode
    if(config->USE_SIM == 1)
    {
        printf("[KAI] simulation mode\n");
        return;
    }

    // open cmd server port
    if(!cmd_server.listen(QHostAddress::AnyIPv4, 7000))
    {
        disconnect(&cmd_server,  &QTcpServer::newConnection, this, &COMM_OLD::cmd_connected);
        logger->write_log("[KAI] cmd server not start", "Red");
    }
    else
    {
        logger->write_log("[KAI] cmd server start", "Green");
    }

    // open data server port
    if(!data_server.listen(QHostAddress::AnyIPv4, 7001))
    {
        disconnect(&data_server,  &QTcpServer::newConnection, this, &COMM_OLD::data_connected);
        logger->write_log("[KAI] data server not start", "Red");
    }
    else
    {
        logger->write_log("[KAI] data server start", "Green");
    }
}

void COMM_OLD::cmd_connected()
{
    if(is_cmd_connected == false)
    {
        cmd_client = cmd_server.nextPendingConnection();
        if(!cmd_client)
        {
            logger->write_log("[KAI] cmd socket connection failed", "Red");
            return;
        }

        is_cmd_connected = true;
        cmd_client_cnt = 1;

        int keep_alive = 1;
        int fd = cmd_client->socketDescriptor();
        setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keep_alive, sizeof(keep_alive));

        int keep_idle = 5;      // 10초 동안 데이터가 없으면 Keep-Alive 시작
        int keep_interval = 2;  // 5초 간격으로 Keep-Alive 패킷 전송
        int keep_count = 3;     // 3번 응답이 없으면 연결 종료
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(keep_idle));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keep_interval, sizeof(keep_interval));
        setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keep_count, sizeof(keep_count));

        connect(cmd_client, &QTcpSocket::readyRead, this, &COMM_OLD::cmd_readyRead);
        connect(cmd_client, &QTcpSocket::disconnected, this, &COMM_OLD::cmd_disconnected);

        logger->write_log("[KAI] cmd socket connected", "Green");
    }
    else
    {
        QTcpSocket *dummy = cmd_server.nextPendingConnection();
        dummy->close();
        dummy->deleteLater();

        logger->write_log("[KAI] cmd socket already connected", "Orange");
    }
}

void COMM_OLD::data_connected()
{
    QTcpSocket *_data_client = data_server.nextPendingConnection();
    if(!_data_client)
    {
        logger->write_log("[KAI] data socket connection failed", "Red");
        return;
    }

    is_data_connected = true;
    data_client_cnt = data_client_cnt + 1;

    int keep_alive = 1;
    int fd = _data_client->socketDescriptor();
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &keep_alive, sizeof(keep_alive));

    int keep_idle = 5;      // 10초 동안 데이터가 없으면 Keep-Alive 시작
    int keep_interval = 2;  // 5초 간격으로 Keep-Alive 패킷 전송
    int keep_count = 3;     // 3번 응답이 없으면 연결 종료
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &keep_idle, sizeof(keep_idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &keep_interval, sizeof(keep_interval));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &keep_count, sizeof(keep_count));

    connect(_data_client, &QTcpSocket::readyRead, this, &COMM_OLD::data_readyRead);
    connect(_data_client, &QTcpSocket::disconnected, this, &COMM_OLD::data_disconnected);

    QString socket_id;
    socket_id.sprintf("%p", _data_client);

    mtx_data_client.lock();
    data_clients[socket_id] = _data_client;
    mtx_data_client.unlock();

    QString str;
    str.sprintf("[KAI] data socket connected. id:%s", socket_id.toLocal8Bit().data());
    logger->write_log(str, "Green");
}

void COMM_OLD::cmd_disconnected()
{
    if(cmd_client)
    {
        is_cmd_connected = false;
        cmd_client_cnt = 0;

        cmd_client->disconnect();
        cmd_client->deleteLater();

        logger->write_log("[KAI] cmd socket disconnected", "Red");
    }
    else
    {
        logger->write_log("[KAI] cmd socket disconnected failed: cmd_client is Null", "Red");
    }
}

void COMM_OLD::data_disconnected()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    if (!socket)
    {
        logger->write_log("[KAI] No socket to disconnect", "Red");
        return;
    }

    QString id;
    id.sprintf("%p", socket);

    mtx_data_client.lock();
    auto it = data_clients.find(id);
    if (it != data_clients.end())
    {
        it->second->disconnect();
        it->second->deleteLater();
        data_clients.erase(it);
    }
    mtx_data_client.unlock();

    if(data_client_cnt > 0)
    {
        data_client_cnt = data_client_cnt - 1;
    }

    QString str;
    str.sprintf("[KAI] data socket disconnected. id:%s", id.toLocal8Bit().data());
    logger->write_log(str, "Red");
}

void COMM_OLD::cmd_loop()
{
    QString msg;
    if(cmd_que.try_pop(msg))
    {
        if(cmd_client && cmd_client->isOpen())
        {
            cmd_client->write(msg.toStdString().c_str(), msg.size());
        }
    }
}

void COMM_OLD::cmd_readyRead()
{
    if(!logger)
    {
        printf("[KAI] logger is nullptr.\n");
        return;
    }
    if(!unimap || !ctrl || !dctrl || !bqr)
    {
        logger->write_log("[KAI] some module is nullptr.", "Red");
        return;
    }
    if (!cmd_client || !cmd_client->isOpen())
    {
        logger->write_log("[KAI] cmd_client is nullptr", "Red");
        return;
    }

    logger->write_log("[KAI] start cmd_readyRead func", "Green");

    // check recv data size
    QByteArray data = cmd_client->readAll();
    QString data_str = QString(data.data());
    if(data_str.isEmpty() || data_str == "")
    {
        Q_EMIT signal_cmd_recv("CMD_NOT_RECV\n");

        logger->write_log("[KAI] received empty data", "Red");
        return;
    }

    std::string data_raw_str = data_str.toStdString();
    for (size_t i = 0; i < data_raw_str.length(); ++i)
    {
        if (data_raw_str[i] == '\n')
        {
            processing_command(cur_msg);
            cur_msg = "";
        }
        else
        {
            cur_msg += data_raw_str[i];
        }
    }
}

void COMM_OLD::processing_command(QString data_str)
{
    QString str0;
    str0.sprintf("[KAI] first received data is: %s", data_str.toLocal8Bit().data());
    logger->write_log(str0, "Green");

    // data format: cmd(param_xx,param_xx, ...)
    QStringList msg_list = data_str.split("(");
    if(msg_list.size() != 2)
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_NOT_RECV\n");

        QString str;
        str.sprintf("[KAI] data size must 2. but data size is:%d", (int)msg_list.size());
        logger->write_log(str, "Red");
        return;
    }

    // start to split cmd, params list
    QString cmd = msg_list[0];
    QString params_list = msg_list[1];
    if(params_list.size() == 0)
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_NOT_RECV\n");

        logger->write_log("[KAI] data out of range 2", "Red");
        return;
    }

    if(params_list.back() == ")")
    {
        params_list.chop(1);
    }

    // start cmd func
    QString str1;
    str1.sprintf("[KAI] cmd: %s, params: %s", cmd.toLocal8Bit().data(), params_list.toLocal8Bit().data());
    logger->write_log(str1, "Green");

    if(cmd == "move_waypoint")
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        // params[0]: node id, params[1]: speed percentage
        QStringList params = params_list.split(",");
        if(params.size() != 2)
        {
            // send received cmd response
            log_fail(FAIL_AUTO_INVALID_PARAMS_SIZE, "[KAI] move_waypoint fail. wrong params_list size:" + QString::number(params_list.size()));
            return;
        }

        QString node_id = params[0]; node_id = node_id.remove(" ");
        int percent = params[1].toInt();

        // check node is valid
        NODE* node = unimap->get_node_by_id(node_id);
        if(node != NULL)
        {
            Eigen::Matrix4d tf = node->tf;
            ctrl->move_hpp(tf, percent);
            auto_timer.start(100);

            mtx.lock();
            last_goal_id = node_id;
            mtx.unlock();

            QString str;
            str.sprintf("[KAI] move_waypoint success. node: %s, percent: %d", node_id.toLocal8Bit().data(), percent);
            logger->write_log(str , "Green");
            return;
        }
        else
        {
            // send received cmd response
            log_fail(FAIL_AUTO_INVALID_NODE, "[KAI] move_waypoint fail. invalid node id:" + node_id);
            return;
        }
    }
    else if(cmd == "move_qralign")
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        // initial value
        double p_gain = 1.0; double d_gain = 1.0;
        double x_offset = 0.0; double y_offset = 0.0; double th_offset = 0.0;

        // params[0]: docking stop or start flag, [1,2]: p,d gain, [3,4,5]:x,y,th offset
        QStringList params = params_list.split(",");
        if(params.size() == 1)
        {
            // do nothing
        }
        else if(params.size() == 3)
        {
            p_gain = std::clamp(params[1].toDouble(), 0.0, 2.0);
            d_gain = std::clamp(params[2].toDouble(), 0.0, 2.0);
        }
        else if(params.size() == 6)
        {
            p_gain = std::clamp(params[1].toDouble(), 0.0, 2.0);
            d_gain = std::clamp(params[2].toDouble(), 0.0, 2.0);
            x_offset  = std::clamp(params[3].toDouble(), -10.0, 10.0);
            y_offset  = std::clamp(params[4].toDouble(), -10.0, 10.0);
            th_offset = std::clamp(params[5].toDouble(), -100.0, 100.0);
        }
        else
        {
            // send received cmd response
            log_fail(FAIL_DOCING_INVALID_PARAMS_SIZE, "[KAI][DOCKING] wrong params_list size:" + QString::number(params_list.size()) + ", params" + params_list);
            return;
        }

        int val = params[0].toInt();
        if(val == MOVE_STOP)
        {
            // send received cmd response
            Q_EMIT signal_cmd_recv("DONE\n");

            // docking seq stop
            dctrl->stop();
            logger->write_log("[KAI] move_qralign sucess. cmd:stop.", "Green", true, false);
        }
        else if(val == MOVE_RUN)
        {
            // bottom qr sensor not connected
            if(!bqr->is_connected)
            {
                log_fail(FAIL_DOCKING_CODE_READER_NOT_CONNECTED, "[KAI] move_qralign fail. code reader not connected.");
                return;
            }

            // bottom qr code not detected
            if(!bqr->is_recv_data)
            {
                log_fail(FAIL_DOCKING_CODE_NOT_DETECTED, "[KAI] move_qralign fail, not detected code");
                return;
            }

            // docking start
            dctrl->move(p_gain, d_gain, x_offset, y_offset, th_offset);
            docking_timer.start(100);

            QString str_qr;
            str_qr.sprintf("[KAI] move_qralign success. cmd:move, params:(gain(p,d), offset(x,y,th)) %.3f,%.3f,%.3f,%.3f,%.3f", p_gain, d_gain, x_offset, y_offset, th_offset);
            logger->write_log(str_qr, "Green", true, false);
        }
        else
        {
            log_fail(FAIL_DOCKING_INVALID_PARAMS, "[KAI] move_qralign fail. invalid params: " + QString::number(val));
        }
    }
    else if(cmd == "stop")
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        ctrl->stop();
        dctrl->stop();

        auto_timer.stop();
        docking_timer.stop();

        logger->write_log("[KAI][STOP] stop", "Green", true, false);
        Q_EMIT signal_cmd_recv("DONE\n");
    }
    else if(cmd == "load_map")
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        QString path = params_list;
        if(path.isEmpty() || path == "")
        {
            log_fail(FAIL_UNIMAP_INVALID_PATH, "[KAI][UNIMAP] invalid path");
            return;
        }

        QDir dir(path);
        if(!dir.exists("cloud.csv") || dir.exists("topo.json"))
        {
            log_fail(FAIL_UNIMAP_NO_FOUND_CLOUD_TOPO, "[KAI][UNIMAP] no found cloud or topo");
            return;
        }

        if(slam->is_busy)
        {
            log_fail(FAIL_UNIMAP_LOAD_FAIL, "[KAI][UNIMAP] failed load map");
            return;
        }

        unimap->load_map(path);
        if(unimap->is_loaded == true)
        {
            QString str = "DONE\n";
            Q_EMIT signal_cmd_recv(str);

            logger->write_log("[KAI][UNIMAP] success load map:" + path , "Green", true, false);
        }
        else
        {
            log_fail(FAIL_UNIMAP_LOAD_FAIL, "[KAI][UNIMAP] failed load map");
            return;
        }
    }
    else if(cmd == "localization_init")
    {
        // send received cmd response
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        // params[0]: node_id
        QStringList params = params_list.split(",");
        if(params.size() == 1)
        {
            if(params[0] != "")
            {
                QString node_id = params[0];
                NODE* node = unimap->get_node_by_id(node_id);
                if(node != NULL)
                {
                    if(slam->is_busy)
                    {
                        // send received cmd response
                        log_fail(FAIL_LOC_NO_INIT, "[KAI] localization_init fail. already running");
                        return;
                    }

                    slam->localization_stop();

                    if(auto_init_thread != NULL)
                    {
                        logger->write_log("[AUTO_INIT] thread already running.", "Orange", true, false);
                        auto_init_thread->join();
                        auto_init_thread = NULL;
                    }

                    Eigen::Matrix4d tf = node->tf;
                    std::vector<Eigen::Matrix4d> tfs;
                    tfs.push_back(tf);

                    auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start_spec, slam, tfs);
                    return;
                }
                else
                {
                    log_fail(FAIL_LOC_INVALID_PATH, "[KAI][LOC] invalid path");
                    return;
                }
            }
            else
            {
                std::vector<Eigen::Matrix4d> _tfs;
                std::vector<QString> _nodes = unimap->get_nodes();
                for(auto& it : _nodes)
                {
                    NODE* node = unimap->get_node_by_id(it);
                    if(node == NULL)
                    {
                        continue;
                    }

                    QString info = node->info;
                    NODE_INFO res;
                    if(parse_info(info, "BQR_CODE_NUM", res))
                    {
                        int bqr_code_num = res.bqr_code_num;
                        if(bqr_code_num < 0)
                        {
                            continue;
                        }
                    }

                    Eigen::Matrix4d tf = node->tf;
                    _tfs.push_back(tf);
                }

                if(_tfs.size() != 0)
                {
                    slam->localization_stop();
                    if(auto_init_thread != NULL)
                    {
                        logger->write_log("[AUTO_INIT] thread already running.", "Orange", true, false);
                        auto_init_thread->join();
                        auto_init_thread = NULL;
                    }

                    auto_init_thread = new std::thread(&SLAM_2D::semi_auto_init_start_spec, slam, _tfs);
                }

                logger->write_log("[KAI][LOC] localization start(code)", "Green", true, false);
                return;
            }
        }
        else
        {
            log_fail(FAIL_LOC_INVALID_PARAMS_SIZE, "[KAI][LOC] wrong params_list size:" + QString::number(params_list.size()));
            return;
        }
    }
    else if(cmd == "dead_reckoning")
    {
        Q_EMIT signal_cmd_recv("CMD_RECV\n");

        // method(x,y,deg,stop), dist(m,deg), velocity(m/s,deg/s)
        QStringList params = params_list.split(",");
        if(params.size() == 3)
        {
            QString method = params[0];
            if(method == "x")
            {
                double d = params[1].toDouble();
                double v = params[2].toDouble();

                if(!isfinite(d) || !isfinite(v))
                {
                    return;
                }

                mobile->move_linear_x(d, v);
            }
            else if(method == "y")
            {
                double d = params[1].toDouble();
                double v = params[2].toDouble();

                if(!isfinite(d) || !isfinite(v))
                {
                    return;
                }

                mobile->move_linear_y(d, v);
            }
            else if(method == "deg")
            {
                double th = params[1].toDouble()*D2R;
                double w = params[2].toDouble()*D2R;

                if(!isfinite(th) || !isfinite(w))
                {
                    return;
                }

                mobile->move_rotate(th, w);
            }
            else if(method == "stop")
            {
                mobile->stop();
            }
        }
        else
        {
            log_fail(FAIL_RECKONING_INVALID_PARAMS_SIZE, "[KAI] cmd:dead_reckoning. wrong params_list size:" + QString::number(params_list.size()));
            return;
        }
    }
    else
    {
        Q_EMIT signal_cmd_recv("CMD_NOT_RECV\n");
    }

    logger->write_log("[KAI] end cmd_readyRead func", "Green", true, false);
}

void COMM_OLD::data_readyRead()
{
    QTcpSocket *socket = static_cast<QTcpSocket*>(sender());
    if(socket == nullptr)
    {
        logger->write_log("[KAI] data socket is null", "Red", true, false);
        return;
    }

    QByteArray data = socket->readAll();
    if(data.isNull() || data.isEmpty())
    {
        logger->write_log("[KAI] data socket's data is null", "Red", true, false);
        return;
    }

    QString msg = QString(data.data());
    if(msg == "reqsimple")
    {
        QString send_msg = get_req_simple();
        socket->write(send_msg.toStdString().c_str(), send_msg.size());
    }

    if(msg == "reqdata")
    {
        QString send_msg = get_req_data();
        socket->write(send_msg.toStdString().c_str(), send_msg.size());
    }
}

void COMM_OLD::auto_loop()
{
    if(ctrl->fsm_state == AUTO_FSM_COMPLETE)
    {
        QString str = "DONE\n";
        Q_EMIT signal_cmd_recv(str);
        auto_timer.stop();
    }
}

void COMM_OLD::docking_loop()
{
    if(dctrl->fsm_state == DOCK_FSM_COMPLETE)
    {
        QString str = "DONE\n";
        Q_EMIT signal_cmd_recv(str);
        docking_timer.stop();
    }
}

QString COMM_OLD::get_req_simple()
{
    QString is_moving = QString::number(0, 10);
    if(ctrl->is_moving || dctrl->is_moving)
    {
        is_moving = QString::number(1, 10);
    }

    MOBILE_STATUS ms = mobile->get_status();
    QString system_stat = QString::number(ms.power_state, 10);

    QString loc_state = get_kai_loc_state();
    QString auto_state = get_kai_auto_state();
    QString docking_state = get_kai_docking_state();
    QString obs_condition = get_kai_obs_state();

    QString is_bqr_detected = QString::number(0, 10);
    if(bqr->is_recv_data == true)
    {
        is_bqr_detected = QString::number(1, 10);
    }

    BQR_INFO cur_bqr = bqr->get_cur_bqr();
    QString bqr_num_str = QString::number(cur_bqr.code_num);
    double bqr_x = (double)bqr->err_x - (double)bqr->offset_x * 0.001;
    double bqr_y = (double)bqr->err_y - (double)bqr->offset_y * 0.001;
    double bqr_th = (double)bqr->err_th - (double)bqr->offset_th * D2R;

    QString bqr_x_str = QString::number(bqr_x, 'f', 3);
    QString bqr_y_str = QString::number(bqr_y, 'f', 3);
    QString bqr_th_str = QString::number(bqr_th*R2D, 'f', 3);

    QString operating_mode = "none";
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(ms.operation_state == ROBOT_JOYSTICK_MODE)
    {
        operating_mode = "joystick";
    }
    else if(ms.operation_state == ROBOT_AUTO_MODE)
    {
        operating_mode = "pc";
    }
    #endif

    QString str = QString("{%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12}")
                    .arg(is_moving)
                    .arg(system_stat)
                    .arg(loc_state)
                    .arg(auto_state)
                    .arg(docking_state)
                    .arg(obs_condition)
                    .arg(is_bqr_detected)
                    .arg(bqr_num_str)
                    .arg(bqr_x_str)
                    .arg(bqr_y_str)
                    .arg(bqr_th_str)
                    .arg(operating_mode)
                    .append("\n");

    return str;
}

QString COMM_OLD::get_req_data()
{
    QString is_moving = QString::number(0, 10);
    if(ctrl->is_moving || dctrl->is_moving)
    {
        is_moving = QString::number(1, 10);
    }

    QString system_stat = QString::number(1, 10);
    QString map_dir = unimap->map_dir.split("/").back();

    // cur x,y,th
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pose = TF_to_se2(cur_tf);
    QString cur_x = QString::number(cur_pose[0], 'f', 3);
    QString cur_y = QString::number(cur_pose[1], 'f', 3);
    QString cur_th = QString::number(cur_pose[2]*R2D, 'f', 3);

    // cur velocity
    MOBILE_POSE mo = mobile->get_pose();
    QString vel_vx = QString::number(mo.vel[0], 'f', 3);
    QString vel_vy = QString::number(mo.vel[1], 'f', 3);
    QString vel_rz = QString::number(mo.vel[2]*R2D, 'f', 3);

    // cur motor state
    MOBILE_STATUS ms = mobile->get_status();
    QString m0_stat = ""; QString m1_stat = "";
    QString m2_stat = ""; QString m3_stat = "";
    m0_stat = QString::number(ms.status_m0, 10);
    m1_stat = QString::number(ms.status_m1, 10);
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    m2_stat = QString::number(ms.status_m2, 10);
    m3_stat = QString::number(ms.status_m3, 10);
    #endif

    // cur motot current
    QString m0_cur_stat = ""; QString m1_cur_stat = "";
    QString m2_cur_stat = ""; QString m3_cur_stat = "";
    m0_cur_stat = QString::number(ms.cur_m0/10.0);
    m1_cur_stat = QString::number(ms.cur_m1/10.0);
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    m2_cur_stat = QString::number(ms.cur_m2/10.0);
    m3_cur_stat = QString::number(ms.cur_m3/10.0);
    #endif

    // cur motor temp
    QString m0_temp_stat = ""; QString m1_temp_stat = "";
    QString m2_temp_stat = ""; QString m3_temp_stat = "";
    m0_temp_stat = QString::number(ms.temp_m0, 10);
    m1_temp_stat = QString::number(ms.temp_m1, 10);
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    m2_temp_stat = QString::number(ms.temp_m2, 10);
    m3_temp_stat = QString::number(ms.temp_m3, 10);
    #endif

    // cur state
    QString ms_power = QString::number(ms.power_state, 10);
    QString ms_emo = QString::number(ms.motor_stop_state, 10);
    QString ms_charge = QString::number(ms.charge_state, 10);

    // cur bat state
    QString ms_bat_in = QString::number(ms.bat_in, 'f', 3);
    QString ms_bat_out = QString::number(ms.bat_out, 'f', 3);
    QString ms_bat_cur = QString::number(ms.bat_current, 'f', 3);
    QString ms_bat_power = QString::number(ms.power, 'f', 3);

    // cur condition
    QString loc_state = get_kai_loc_state();
    QString auto_state = get_kai_auto_state();
    QString docking_state = get_kai_docking_state();
    QString obs_condition = get_kai_obs_state();

    QString is_bqr_detected = QString::number(0, 10);
    if(bqr->is_recv_data == true)
    {
        is_bqr_detected = QString::number(1, 10);
    }

    BQR_INFO cur_bqr = bqr->get_cur_bqr();
    QString bqr_num_str = QString::number(cur_bqr.code_num);
    double bqr_x = (double)bqr->err_x - (double)bqr->offset_x * 0.001;
    double bqr_y = (double)bqr->err_y - (double)bqr->offset_y * 0.001;
    double bqr_th = (double)bqr->err_th - (double)bqr->offset_th * D2R;

    QString bqr_x_str = QString::number(bqr_x, 'f', 3);
    QString bqr_y_str = QString::number(bqr_y, 'f', 3);
    QString bqr_th_str = QString::number(bqr_th*R2D, 'f', 3);

    QString node_size  = QString::number(unimap->nodes.size(), 10);

    QString operating_mode = "none";
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(ms.operation_state == ROBOT_JOYSTICK_MODE)
    {
        operating_mode = "joystick";
    }
    else if(ms.operation_state == ROBOT_AUTO_MODE)
    {
        operating_mode = "pc";
    }
    #endif

    QString str = QString("{%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12,%13,%14,%15,%16,%17,%18,%19,%20,%21,%22,%23,%24,%25,%26,%27,%28,%29,%30,%31,%32,%33,%34,%35,%36,%37,%38,%39}")
                    .arg(is_moving)
                    .arg(system_stat)
                    .arg(map_dir)
                    .arg(cur_x)
                    .arg(cur_y)
                    .arg(cur_th)
                    .arg(vel_vx)
                    .arg(vel_vy)
                    .arg(vel_rz)
                    .arg(m0_stat)
                    .arg(m1_stat)
                    .arg(m2_stat)
                    .arg(m3_stat)
                    .arg(m0_cur_stat)
                    .arg(m1_cur_stat)
                    .arg(m2_cur_stat)
                    .arg(m3_cur_stat)
                    .arg(m0_temp_stat)
                    .arg(m1_temp_stat)
                    .arg(m2_temp_stat)
                    .arg(m3_temp_stat)
                    .arg(ms_power)
                    .arg(ms_emo)
                    .arg(ms_charge)
                    .arg(ms_bat_in)
                    .arg(ms_bat_out)
                    .arg(ms_bat_cur)
                    .arg(ms_bat_power)
                    .arg(loc_state)
                    .arg(auto_state)
                    .arg(docking_state)
                    .arg(obs_condition)
                    .arg(is_bqr_detected)
                    .arg(bqr_num_str)
                    .arg(bqr_x_str)
                    .arg(bqr_y_str)
                    .arg(bqr_th_str)
                    .arg(node_size)
                    .arg(operating_mode)
                    .append("\n");

    return str;
}

QString COMM_OLD::get_kai_loc_state()
{
    if(slam == NULL)
    {
        return QString::number(-1, 10);
    }

    QString loc_state = slam->get_cur_loc_state();
    if(loc_state == "none")
    {
        loc_state = QString::number(0, 10);
    }
    else if(loc_state == "fail")
    {
        loc_state = QString::number(1, 10);
    }
    else if(loc_state == "good")
    {
        loc_state = QString::number(2, 10);
    }
    else
    {
        loc_state = QString::number(-1, 10);
    }

    return loc_state;
}

QString COMM_OLD::get_kai_auto_state()
{
    QString auto_state = QString::number(-1, 10);
    if(ctrl == NULL)
    {
        return auto_state; // none
    }

    if(ctrl->is_pause)
    {
        auto_state = QString::number(1, 10); // pause
    }
    else if(ctrl->is_moving)
    {
        auto_state = QString::number(2, 10); // moving
    }
    else
    {
        auto_state = QString::number(0, 10); // stop
    }

    return auto_state;
}

QString COMM_OLD::get_kai_docking_state()
{
    QString docking_state = QString::number(-1, 10);
    if(dctrl == NULL)
    {
        return docking_state; // none
    }

    if(dctrl->is_pause)
    {
        docking_state = QString::number(1, 10); // "pause"
    }
    else if(dctrl->is_moving)
    {
        docking_state = QString::number(2, 10); // "moving"
    }
    else
    {
        docking_state = QString::number(0, 10);
    }

    return docking_state;
}

QString COMM_OLD::get_kai_obs_state()
{
    if(ctrl == NULL)
    {
        return QString::number(-1, 10);
    }

    QString obs_condition = ctrl->get_obs_condition();
    if(obs_condition == "none")
    {
        obs_condition = QString::number(0, 10);
    }
    else if(obs_condition == "far")
    {
        obs_condition = QString::number(1, 10);
    }
    else if(obs_condition == "near")
    {
        obs_condition = QString::number(2, 10);
    }
    else
    {
        obs_condition = QString::number(-1, 10);
    }

    return obs_condition;
}

void COMM_OLD::slot_cmd_recv(QString str)
{
    cmd_que.push(str);
}

void COMM_OLD::slot_semiautoinit_successed()
{
    logger->write_log("[KAI] localization start" , "Green", true, false);

    QString str = "DONE\n";
    Q_EMIT signal_cmd_recv(str);
}

void COMM_OLD::slot_semiautoinit_failed()
{
    log_fail(FAIL_LOC_NO_INIT, "[KAI] init failed");
}

void COMM_OLD::log_fail(int err_code, const QString& msg)
{
    Q_EMIT signal_cmd_recv("FAILED(" + QString::number(err_code) + ")\n");
    logger->write_log(msg, "Red", true, false);
}
