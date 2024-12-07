#include "comm_fms.h"
#include "mainwindow.h"

COMM_FMS::COMM_FMS(QObject *parent)
    : QObject{parent}
    , main(parent)
    , reconnect_timer(this)
{
    connect(&client, &QWebSocket::connected, this, &COMM_FMS::connected);
    connect(&client, &QWebSocket::disconnected, this, &COMM_FMS::disconnected);
    connect(&reconnect_timer, SIGNAL(timeout()), this, SLOT(reconnect_loop()));    

    connect(this, SIGNAL(signal_mapload(double, QString)), this, SLOT(slot_mapload(double, QString)));
    connect(this, SIGNAL(signal_init(double)), this, SLOT(slot_init(double)));
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
    _robot_id.sprintf("R_%lld", (long long)(get_time0()*1000));

    // update robot id
    robot_id = _robot_id;
    printf("[COMM_FMS] ID: %s\n", robot_id.toLocal8Bit().data());

    Q_EMIT signal_regist_id(_robot_id);

    // start reconnect loop
    if(config->USE_FMS)
    {
        reconnect_timer.start(3000);
        printf("[COMM_FMS] start reconnect timer\n");
    }
}

void COMM_FMS::reconnect_loop()
{
    if(is_connected == false)
    {
        QString server_addr;
        server_addr.sprintf("ws://%s:12334", config->SERVER_IP.toLocal8Bit().data());
        client.open(QUrl(server_addr));

        reconnect_cnt++;

        /*
        if(reconnect_cnt > 5)
        {
            reconnect_timer.stop();
            printf("[COMM_FMS] server not opened, give up reconnect\n");
        }
        */
    }
}

void COMM_FMS::connected()
{
    if(!is_connected)
    {
        is_connected = true;
        connect(&client, &QWebSocket::textMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = true;
        printf("[COMM_FMS] connected\n");
    }
}

void COMM_FMS::disconnected()
{
    if(is_connected)
    {
        is_connected = false;
        disconnect(&client, &QWebSocket::textMessageReceived, this, &COMM_FMS::recv_message);

        ctrl->is_multi = false;
        printf("[COMM_FMS] disconnected\n");
    }
}

// recv callback
void COMM_FMS::recv_message(QString message)
{
    QJsonObject data = QJsonDocument::fromJson(message.toUtf8()).object();
    if(get_json(data, "robot_id") != robot_id)
    {
        return;
    }

    // parsing
    if(get_json(data, "type") == "load")
    {
        QString map_name = get_json(data, "map_name");
        double time = get_json(data, "time").toDouble()/1000;

        Q_EMIT signal_mapload(time, map_name);
        printf("[COMM_FMS] recv_load, map_name: %s, time: %.3f\n", map_name.toLocal8Bit().data(), time);
    }
    else if(get_json(data, "type") == "init")
    {
        double time = get_json(data, "time").toDouble()/1000;

        Q_EMIT signal_init(time);
        printf("[COMM_FMS] recv_init, time: %.3f\n", time);
    }
    else if(get_json(data, "type") == "goal")
    {
        QString goal_id = get_json(data, "goal_id");
        double time = get_json(data, "time").toDouble()/1000;

        ctrl->set_goal(goal_id);
        printf("[COMM_FMS] recv_goal, goal_id: %s, time: %.3f\n", goal_id.toLocal8Bit().data(), time);
    }
    else if(get_json(data, "type") == "path")
    {
        QString path_str = get_json(data, "path");
        int preset = get_json(data, "preset").toInt();        
        double time = get_json(data, "time").toDouble()/1000;

        QStringList path_str_list = path_str.split(",");
        std::vector<QString> path;
        for(int p = 0; p < path_str_list.size(); p++)
        {
            path.push_back(path_str_list[p]);
        }

        ctrl->move_pp(path, preset);
        printf("[COMM_FMS] recv_path, num: %d, preset: %d, time: %.3f\n", (int)path.size(), preset, time);
    }
    else if(get_json(data, "type") == "stop")
    {
        double time = get_json(data, "time").toDouble()/1000;

        ctrl->stop();
        printf("[COMM_FMS] recv_stop, time: %.3f\n", time);
    }
    else if(get_json(data, "type") == "vobs_robots")
    {
        double time = get_json(data, "time").toDouble()/1000;
        std::vector<Eigen::Vector3d> vobs_list;

        QString vobs_str = get_json(data, "vobs_str");
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

        /*
        if(vobs_list.size() > 0)
        {
            printf("[COMM_FMS] recv_vobs_robots, num: %d, time: %.3f\n", (int)vobs_list.size(), time);
        }
        */
    }
    else if(get_json(data, "type") == "vobs_closures")
    {
        double time = get_json(data, "time").toDouble()/1000;

        QString vobs_str = get_json(data, "vobs_str");
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

        /*
        if(vobs_list.size() > 0)
        {
            printf("[COMM_FMS] recv_vobs_closures, num: %d, time: %.3f\n", (int)vobs_list.size(), time);
        }
        */
    }
}

// recv slots
void COMM_FMS::slot_mapload(double time, QString name)
{
    MainWindow* _main = (MainWindow*)main;

    QString load_dir = QDir::homePath() + "/maps/" + name;
    if(!load_dir.isNull())
    {
        slam->localization_stop();
        obsmap->clear();

        _main->map_dir = load_dir;
        unimap->load_map(load_dir);
        _main->all_update();
    }
}

void COMM_FMS::slot_init(double time)
{
    MainWindow* _main = (MainWindow*)main;
    _main->bt_LocInitSemiAuto();
}

// send slots
void COMM_FMS::slot_send_info()
{
    if(!is_connected)
    {
        return;
    }

    double time = get_time0();
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();    
    QString cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
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
    rootObj["type"] = "info";
    rootObj["robot_id"] = robot_id;
    rootObj["cur_tf"] = TF_to_string(cur_tf);
    rootObj["goal_tf"] = TF_to_string(goal_tf);
    rootObj["cur_node_id"] = cur_node_id;
    rootObj["goal_node_id"] = goal_node_id;    
    rootObj["request"] = request;
    rootObj["state"] = state;
    rootObj["time"] = QString::number((long long)(time*1000), 10);

    // send
    QJsonDocument doc(rootObj);
    QString str(doc.toJson());
    client.sendTextMessage(str);
}

