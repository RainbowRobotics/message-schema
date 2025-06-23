#include "autocontrol.h"

AUTOCONTROL* AUTOCONTROL::instance(QObject* parent)
{
    static AUTOCONTROL* inst = nullptr;
    if(!inst && parent)
    {
        inst = new AUTOCONTROL(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

AUTOCONTROL::AUTOCONTROL(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    unimap(nullptr),
    obsmap(nullptr),
    loc(nullptr)
{
    connect(this, SIGNAL(signal_move(DATA_MOVE)), this, SLOT(slot_move(DATA_MOVE)));
    connect(this, SIGNAL(signal_path(DATA_PATH)), this, SLOT(slot_path(DATA_PATH)));
}

AUTOCONTROL::~AUTOCONTROL()
{
    // control loop stop
    control_flag = false;
    if(control_thread && control_thread->joinable())
    {
        control_thread->join();
    }
    control_thread.reset();
}

void AUTOCONTROL::init()
{
    params = load_preset(0);
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path = QCoreApplication::applicationDirPath() + "/configs/" + config->get_platform_name() + "/preset_" + QString::number(preset) + ".json";

    QFileInfo info(preset_path);
    if(info.exists() && info.isFile())
    {
        QFile file(preset_path);
        if(file.open(QIODevice::ReadOnly))
        {
            QByteArray data = file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
            printf("[PRESET] %s load\n", preset_path.toLocal8Bit().data());

            res.LIMIT_V = obj["LIMIT_V"].toString().toDouble();
            printf("[PRESET] LIMIT_V :%f\n", res.LIMIT_V);

            res.LIMIT_W = obj["LIMIT_W"].toString().toDouble();
            printf("[PRESET] LIMIT_W :%f\n", res.LIMIT_W);

            res.LIMIT_V_ACC = obj["LIMIT_V_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_V_ACC :%f\n", res.LIMIT_V_ACC);

            res.LIMIT_V_DCC = obj["LIMIT_V_DCC"].toString().toDouble();
            printf("[PRESET] LIMIT_V_DCC :%f\n", res.LIMIT_V_DCC);

            res.LIMIT_W_ACC = obj["LIMIT_W_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_W_ACC :%f\n", res.LIMIT_W_ACC);

            res.LIMIT_PIVOT_W = obj["LIMIT_PIVOT_W"].toString().toDouble();
            printf("[PRESET] LIMIT_PIVOT_W :%f\n", res.LIMIT_PIVOT_W);

            res.ST_V = obj["ST_V"].toString().toDouble();
            printf("[PRESET] ST_V :%f\n", res.ST_V);

            res.ED_V = obj["ED_V"].toString().toDouble();
            printf("[PRESET] ED_V :%f\n", res.ED_V);

            res.DRIVE_T = obj["DRIVE_T"].toString().toDouble();
            printf("[PRESET] DRIVE_T :%f\n", res.DRIVE_T);

            res.DRIVE_H = obj["DRIVE_H"].toString().toDouble();
            printf("[PRESET] DRIVE_H :%f\n", res.DRIVE_H);

            res.DRIVE_A = obj["DRIVE_A"].toString().toDouble();
            printf("[PRESET] DRIVE_A :%f\n", res.DRIVE_A);

            res.DRIVE_B = obj["DRIVE_B"].toString().toDouble();
            printf("[PRESET] DRIVE_B :%f\n", res.DRIVE_B);

            res.DRIVE_L = obj["DRIVE_L"].toString().toDouble();
            printf("[PRESET] DRIVE_L :%f\n", res.DRIVE_L);

            res.DRIVE_K = obj["DRIVE_K"].toString().toDouble();
            printf("[PRESET] DRIVE_K :%f\n", res.DRIVE_K);

            res.DRIVE_EPS = obj["DRIVE_EPS"].toString().toDouble();
            printf("[PRESET] DRIVE_EPS :%f\n", res.DRIVE_EPS);

            file.close();
        }
    }
    else
    {
        printf("[AUTO] invalid preset path\n");
    }

    return res;
}

PATH AUTOCONTROL::get_cur_global_path()
{
    std::lock_guard<std::mutex> lock(mtx);
    PATH res = cur_global_path;
    return res;
}

PATH AUTOCONTROL::get_cur_local_path()
{
    std::lock_guard<std::mutex> lock(mtx);
    PATH res = cur_local_path;
    return res;
}

void AUTOCONTROL::set_cur_goal_state(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_move_state = str;
}

void AUTOCONTROL::set_is_rrs(bool val)
{
    is_rrs.store(val);
}

void AUTOCONTROL::set_is_moving(bool val)
{
    is_moving.store(val);
}

void AUTOCONTROL::set_is_pause(bool val)
{
    is_pause.store(val);
}

void AUTOCONTROL::set_is_debug(bool val)
{
    is_debug.store(val);
}

QString AUTOCONTROL::get_cur_move_state()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_move_state;
    return res;
}

QString AUTOCONTROL::get_obs_condition()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_obs_condition;
    return res;
}

int AUTOCONTROL::get_fsm_state()
{
    return (int)fsm_state.load();
}

DATA_MOVE AUTOCONTROL::get_cur_move_info()
{
    std::lock_guard<std::mutex> lock(mtx);
    DATA_MOVE res = cur_move_info;
    return res;
}

Eigen::Vector3d AUTOCONTROL::get_last_cur_pos()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector3d res = last_cur_pos;
    return res;
}

Eigen::Vector3d AUTOCONTROL::get_last_tgt_pos()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector3d res = last_tgt_pos;
    return res;
}

Eigen::Vector3d AUTOCONTROL::get_last_local_goal()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector3d res = last_local_goal;
    return res;
}

void AUTOCONTROL::set_obs_condition(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_obs_condition = str;
}

QString AUTOCONTROL::get_multi_reqest_state()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_multi_req;
    return res;
}

void AUTOCONTROL::set_multi_req(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_multi_req = str;
}

void AUTOCONTROL::stop()
{
    // control loop stop
    if(control_thread != NULL)
    {
        control_flag = false;
        control_thread->join();
        control_thread = NULL;
    }

    // stop
    mobile->move(0, 0, 0);

    // control params clear
    is_moving = false;
    is_pause = false;

    // comm params clear
    set_multi_req("none");
    set_obs_condition("none");
    set_cur_goal_state("cancel");

    // obsmap clear
    obsmap->clear();

    // path clear
    clear_path();
}

void AUTOCONTROL::clear_path()
{
    {
        std::lock_guard<std::mutex> lock(mtx);
        cur_global_path = PATH();
        cur_local_path = PATH();
    }

    Q_EMIT signal_global_path_updated();
    Q_EMIT signal_local_path_updated();
}

void AUTOCONTROL::slot_move(DATA_MOVE msg)
{
    // fill goal node name
    if(msg.goal_node_id != "" && msg.goal_node_name == "")
    {
        NODE* node = unimap->get_node_by_id(msg.goal_node_id);
        if(node)
        {
            msg.goal_node_name = node->name;
        }
    }

    {
        std::lock_guard<std::mutex> lock(mtx);
        cur_move_info = msg;
    }

    if(msg.command == "goal")
    {
        if(is_rrs && config->get_use_multi())
        {
            qDebug() << "[AUTO] req_path, move";
            set_multi_req("req_path");
            set_cur_goal_state("move");
        }
        else
        {
            Eigen::Matrix4d tf = ZYX_to_TF(msg.tgt_pose_vec[0], msg.tgt_pose_vec[1], msg.tgt_pose_vec[2], 0, 0, msg.tgt_pose_vec[3]);
            move(tf, msg.preset);
        }
    }
    else if(msg.command == "change_goal")
    {
        qDebug() << "[AUTO] just change goal";
    }
}

void AUTOCONTROL::slot_path(DATA_PATH msg)
{
    const QString path_str = msg.path;
    const QStringList path_str_list = path_str.split(",", Qt::SkipEmptyParts);

    std::vector<QString> path;
    path.reserve(path_str_list.size());
    for(const QString& path_item : path_str_list)
    {
        path.push_back(path_item);
    }

    const int preset = msg.preset;

    move(path, preset);
}

void AUTOCONTROL::move(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // load preset
    params = load_preset(preset);

    // calc global path
    PATH path = calc_global_path(goal_tf);
    if(path.pos.size() > 0)
    {
        // enque global path
        global_path_que.clear();
        global_path_que.push(path);
    }

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }
}

void AUTOCONTROL::move(std::vector<QString> node_path, int preset)
{
    if(node_path.size() == 0 || preset == -1)
    {
        return;
    }

    // symmetric cut
    std::vector<std::vector<QString>> path_list = symmetric_cut(node_path);

    // loop cut
    std::vector<std::vector<QString>> path_list2;
    for(size_t p = 0; p < path_list.size(); p++)
    {
        std::vector<std::vector<QString>> res = loop_cut(path_list[p]);
        for(size_t q = 0; q < res.size(); q++)
        {
            path_list2.push_back(res[q]);
        }
    }

    if(path_list2.size() == 0)
    {
        logger->write_log("[AUTO] move_pp, path_list2 empty");
        stop();
        return;
    }

    // set flag
    set_multi_req("recv_path");

    logger->write_log("[AUTO] move_pp, recv path check");
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        logger->write_log(QString("[AUTO] path_%1").arg(p));
        for(size_t q = 0; q < path_list2[p].size(); q++)
        {
            logger->write_log(QString("[AUTO] %1").arg(path_list2[p][q]));
        }
    }

    QString final_goal_node_id = "";
    QString final_goal_node_name = "";
    {
        std::lock_guard<std::mutex> lock(mtx);
        final_goal_node_id = cur_move_info.goal_node_id;
        final_goal_node_name = cur_move_info.goal_node_name;
        logger->write_log(QString("[AUTO] final_goal: %1, %2").arg(final_goal_node_id).arg(final_goal_node_name));
    }

    // set path
    std::vector<PATH> tmp_storage;
    for(size_t p = 0; p < path_list2.size(); p++)
    {
        // enque path
        PATH path = calc_global_path(path_list2[p], p == 0);

        // check final path
        if(p == path_list2.size()-1)
        {
            NODE* node = unimap->get_node_by_id(path_list2[p].back());
            if(node == NULL)
            {
                logger->write_log("[AUTO] move_pp, path invalid");
                qDebug() << "[AUTO] node null";
                stop();
                return;
            }

            if(final_goal_node_name.contains("AMR-WAITING-01") && node->name.contains("AMR-WAITING-01"))
            {
                path.is_final = true;
                logger->write_log("[AUTO] move_pp, waiting, path set final");
            }
            else if(final_goal_node_name.contains("AMR-CHARGING-01") && node->name.contains("AMR-CHARGING-01"))
            {
                path.is_final = true;
                logger->write_log("[AUTO] move_pp, charging, path set final");
            }
            else if(final_goal_node_name.contains("AMR-PACKING-01") && node->name.contains("AMR-PACKING-01"))
            {
                path.is_final = true;
                logger->write_log("[AUTO] move_pp, packing, path set final");
            }
            else if(final_goal_node_name.contains("AMR-CONTAINER-01") && node->name.contains("AMR-CONTAINER-01"))
            {
                path.is_final = true;
                logger->write_log("[AUTO] move_pp, container, path set final");
            }
            else if(final_goal_node_id == node->id)
            {
                path.is_final = true;
                logger->write_log("[AUTO] move_pp, path set final");
            }
        }

        tmp_storage.push_back(path);
    }

    // control loop shutdown but robot still moving
    if(control_flag)
    {
        // check path overlap or reset
        bool is_curve = false;
        std::vector<Eigen::Matrix4d> merged_tf_list;
        for(size_t p = 0; p < tmp_storage.size(); p++)
        {
            for(size_t q = 0; q < tmp_storage[p].pose.size(); q++)
            {
                merged_tf_list.push_back(tmp_storage[p].pose[q]);
            }
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        for(int p = 0; p < std::min<int>((int)merged_tf_list.size(), 20); p++) // 2m
        {
            Eigen::Vector3d xi = TF_to_se2(merged_tf_list[p]);
            double th = deltaRad(xi[2], cur_xi[2]);
            if(std::abs(th) > 30.0*D2R)
            {
                is_curve = true;
                break;
            }
        }

        is_path_overlap = true;
        control_flag = false;
        control_thread->join();
        control_thread.reset();

        if(is_curve)
        {
            mobile->move(0,0,0);
            logger->write_log("[AUTO] move_pp, curve detected, stop");
        }
        else
        {
            logger->write_log("[AUTO] move_pp, no curve, just change path");
        }
    }

    // set global path
    global_path_que.clear();
    for(size_t p = 0; p < tmp_storage.size(); p++)
    {
        global_path_que.push(tmp_storage[p]);
    }

    // load preset
    params = load_preset(preset);

    // start control loop
    if(control_flag == false)
    {
        control_flag = true;
        control_thread = std::make_unique<std::thread>(&AUTOCONTROL::control_loop, this);
    }
}

std::vector<QString> AUTOCONTROL::remove_duplicates(std::vector<QString> node_path)
{
    if(node_path.size() < 2)
    {
        return node_path;
    }

    std::vector<QString> res;
    res.push_back(node_path[0]);

    for(size_t p = 1; p < node_path.size(); p++)
    {
        if(res.back() == node_path[p])
        {
            continue;
        }

        res.push_back(node_path[p]);
    }

    return res;
}

std::vector<std::vector<QString>> AUTOCONTROL::symmetric_cut(std::vector<QString> node_path)
{
    std::vector<QString> path = remove_duplicates(node_path);
    if(path.size() >= 3)
    {
        // find symmetric
        std::vector<int> split_idxs;
        for(size_t p = 1; p < path.size()-1; p++)
        {
            if(path[p-1] == path[p+1])
            {
                split_idxs.push_back(p);
            }
        }

        // split
        if(split_idxs.size() == 0)
        {
            std::vector<std::vector<QString>> res;
            res.push_back(path);
            return res;
        }
        else
        {
            std::vector<std::vector<QString>> res;

            int idx0 = 0;
            for(size_t i = 0; i < split_idxs.size(); i++)
            {
                int idx1 = split_idxs[i];
                std::vector<QString> sub_path(path.begin() + idx0, path.begin() + (idx1+1));
                res.push_back(sub_path);
                idx0 = idx1;
            }

            if(idx0 < (int)path.size()-1)
            {
                std::vector<QString> sub_path(path.begin() + idx0, path.end());
                res.push_back(sub_path);
            }

            return res;
        }
    }
    else
    {
        std::vector<std::vector<QString>> res;
        res.push_back(path);
        return res;
    }
}

std::vector<std::vector<QString>> AUTOCONTROL::loop_cut(std::vector<QString> node_path)
{
    std::vector<std::vector<QString>> res;
    if(node_path.size() < 4)
    {
        res.push_back(node_path);
        return res;
    }

    // check loop
    std::vector<QString> tmp;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        auto it = std::find(tmp.begin(), tmp.end(), node_path[p]);
        if(it == tmp.end())
        {
            tmp.push_back(node_path[p]);
        }
        else
        {
            QString _last_node_id = tmp.back();
            res.push_back(tmp);

            tmp.clear();
            tmp.push_back(_last_node_id);
            tmp.push_back(node_path[p]);
        }
    }

    // set last segment
    res.push_back(tmp);
    return res;
}

bool AUTOCONTROL::get_is_moving()
{
    return (bool)is_moving.load();
}

bool AUTOCONTROL::get_is_pause()
{
    return (bool)is_pause.load();
}

bool AUTOCONTROL::get_is_debug()
{
    return (bool)is_debug.load();
}

Eigen::Matrix4d AUTOCONTROL::get_approach_pose(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, Eigen::Matrix4d cur_tf)
{
    Eigen::Matrix4d res = Eigen::Matrix4d::Identity();

    std::vector<Eigen::Matrix4d> src;
    src.push_back(tf0);
    src.push_back(tf1);

    std::vector<Eigen::Matrix4d> dst = path_resampling(src, LOCAL_PATH_STEP);

    double min_d = 99999999;
    for(size_t p = 0; p < dst.size(); p++)
    {
        double d = calc_dist_2d(dst[p].block(0,3,3,1) - cur_tf.block(0,3,3,1));
        if(d < min_d)
        {
            min_d = d;
            res = dst[p];
        }
    }

    return res;
}

PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf)
{
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();

    // get st node id
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    QString st_node_id = unimap->get_node_id_edge(cur_pos);
    if(st_node_id == "")
    {
        logger->write_log("[AUTO] st_node_id empty");
        return PATH();
    }

    // set ed node id
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    QString ed_node_id = unimap->get_node_id_edge(goal_pos);
    if(ed_node_id == "")
    {
        logger->write_log("[AUTO] ed_node_id empty");
        return PATH();
    }

    // topology path finding
    std::vector<QString> node_path = calc_node_path(st_node_id, ed_node_id);
    if(node_path.size() == 0)
    {
        logger->write_log("[AUTO] node_path empty");
        return PATH();
    }

    // convert metric path
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString node_id = node_path[p];
        NODE* node = unimap->get_node_by_id(node_id);
        if(!node)
        {
            logger->write_log(QString("[AUTO] %s: node null").arg(node_id));
            return PATH();
        }
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(node_pose.size() == 1)
    {
        node_pose.insert(node_pose.begin(), cur_tf);
    }
    else
    {
        // approach point
        Eigen::Matrix4d app = get_approach_pose(node_pose[0], node_pose[1], cur_tf);
        node_pose.erase(node_pose.begin());
        node_pose.insert(node_pose.begin(), app);
        node_pose.insert(node_pose.begin(), cur_tf);
    }

    // add goal pos
    if(!node_pose.back().isApprox(goal_tf))
    {
        node_pose.push_back(goal_tf);
    }

    // divide and smooth metric path
    std::vector<Eigen::Matrix4d> path_pose = reorientation_path(node_pose);
    path_pose = path_resampling(path_pose, GLOBAL_PATH_STEP);

    std::vector<Eigen::Vector3d> path_pos;
    for(size_t p = 0; p < path_pose.size(); p++)
    {
        path_pos.push_back(path_pose[p].block(0,3,3,1));
    }

    // set ref_v
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, GLOBAL_PATH_STEP);
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.t = get_time();
    res.node = node_path;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = goal_tf;
    res.is_final = true;
    return res;
}

PATH AUTOCONTROL::calc_global_path(std::vector<QString> node_path, bool add_cur_tf)
{
    NODE* ed_node = unimap->get_node_by_id(node_path.back());
    if(ed_node == NULL)
    {
        return PATH();
    }

    Eigen::Matrix4d ed_tf = ed_node->tf;

    // convert metric path
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString node_id = node_path[p];
        NODE* node = unimap->get_node_by_id(node_id);
        if(node == NULL)
        {
            logger->write_log(QString("[AUTO] %1: node null").arg(node_id));
            return PATH();
        }
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(add_cur_tf)
    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        if(node_pose.size() == 1)
        {
            node_pose.insert(node_pose.begin(), cur_tf);
        }
        else
        {
            Eigen::Matrix4d app = get_approach_pose(node_pose[0], node_pose[1], cur_tf);
            node_pose.erase(node_pose.begin());
            node_pose.insert(node_pose.begin(), app);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
    }

    // add ed pos
    if(!node_pose.back().isApprox(ed_tf))
    {
        node_pose.push_back(ed_tf);
    }

    // divide and smooth metric path
    std::vector<Eigen::Matrix4d> path_pose = reorientation_path(node_pose);
    path_pose = path_resampling(path_pose, GLOBAL_PATH_STEP);

    std::vector<Eigen::Vector3d> path_pos;
    for(size_t p = 0; p < path_pose.size(); p++)
    {
        path_pos.push_back(path_pose[p].block(0,3,3,1));
    }

    // set ref_v
    std::vector<double> ref_v;
    if(add_cur_tf)
    {
        Eigen::Vector3d control_input = mobile->get_control_input();
        calc_ref_v(path_pose, ref_v, std::max<double>(params.ST_V, control_input[0]), GLOBAL_PATH_STEP);
    }
    else
    {
        calc_ref_v(path_pose, ref_v, params.ST_V, GLOBAL_PATH_STEP);
    }
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.t = get_time();
    res.node = node_path;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = ed_tf;
    return res;
}

std::vector<QString> AUTOCONTROL::calc_node_path(QString st_node_id, QString ed_node_id)
{
    // set st node, ed node
    NODE* st_node = unimap->get_node_by_id(st_node_id);
    NODE* ed_node = unimap->get_node_by_id(ed_node_id);
    if(st_node == nullptr || ed_node == nullptr)
    {
        printf("[AUTO] st_node or ed_node null\n");
        return std::vector<QString>();
    }

    printf("[AUTO] topo_path_finding, st: %s -> ed: %s\n", st_node->id.toLocal8Bit().data(), ed_node->id.toLocal8Bit().data());

    // no need path finding
    if(st_node->id == ed_node->id)
    {
        printf("[AUTO] st_node same as ed_node, just set ed_node\n");

        std::vector<QString> res;
        res.push_back(ed_node_id);
        return res;
    }

    // astar search
    std::vector<ASTAR_NODE*> open_set;
    std::vector<ASTAR_NODE*> close_set;

    ASTAR_NODE *ed = new ASTAR_NODE();
    ed->node = ed_node;

    ASTAR_NODE *st = new ASTAR_NODE();
    st->node = st_node;
    st->g = 0;
    st->h = (ed->node->tf.block(0,3,3,1) - st->node->tf.block(0,3,3,1)).norm();
    st->f = st->g + st->h;
    open_set.push_back(st);

    while(open_set.size() > 0)
    {
        // get best node
        int cur_idx = 0;
        ASTAR_NODE *cur = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur->f)
            {
                cur = open_set[p];
                cur_idx = p;
            }
        }

        // pop open_set, push close_set
        open_set.erase(open_set.begin()+cur_idx);
        close_set.push_back(cur);

        // found goal
        if(cur->node->id == ed->node->id)
        {
            std::vector<QString> res;

            ASTAR_NODE* _cur = cur;
            while(_cur != NULL)
            {
                res.push_back(_cur->node->id);
                _cur = _cur->parent;
            }

            std::reverse(res.begin(), res.end());

            printf("[AUTO] topo_path_finding complete, num:%d\n", (int)res.size());
            return res;
        }

        // append child node
        std::vector<QString> around;
        for(size_t p = 0; p < cur->node->linked.size(); p++)
        {
            bool is_close = false;
            for(size_t q = 0; q < close_set.size(); q++)
            {
                if(close_set[q]->node->id == cur->node->linked[p])
                {
                    is_close = true;
                    break;
                }
            }

            if(is_close == false)
            {
                around.push_back(cur->node->linked[p]);
            }
        }

        // calc child node
        for(size_t p = 0; p < around.size(); p++)
        {
            // calc heuristics
            NODE* node = unimap->get_node_by_id(around[p]);
            if(node == nullptr)
            {
                continue;
            }

            double g = cur->g + (node->tf.block(0,3,3,1) - cur->node->tf.block(0,3,3,1)).norm();
            double h = (node->tf.block(0,3,3,1) - ed->node->tf.block(0,3,3,1)).norm();
            double f = g + h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t q = 0; q < open_set.size(); q++)
            {
                if(open_set[q]->node->id == node->id)
                {
                    is_open_set = true;
                    open_node = open_set[q];
                    break;
                }
            }

            if(is_open_set)
            {
                if(g < open_node->g)
                {
                    open_node->parent = cur;
                    open_node->g = g;
                    open_node->h = h;
                    open_node->f = f;
                    continue;
                }
                else
                {
                    continue;
                }
            }

            // add new child to open set
            ASTAR_NODE *child = new ASTAR_NODE();
            child->parent = cur;
            child->node = node;
            child->g = g;
            child->h = h;
            child->f = f;
            open_set.push_back(child);
        }
    }

    printf("[AUTO] topo_path_finding, failed\n");
    return std::vector<QString>();
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_resampling(const std::vector<Eigen::Vector3d>& src, double step)
{
    if (src.size() < 2)
    {
        return src;
    }

    std::vector<Eigen::Vector3d> sampled_path;
    sampled_path.push_back(src.front());

    Eigen::Vector3d pre = src.front();
    double accumulated_dist = 0.0;

    for (size_t i = 1; i < src.size(); i++)
    {
        Eigen::Vector3d cur = src[i];
        double segment_length = (cur - pre).norm();

        while(accumulated_dist + segment_length >= step)
        {
            double t = (step - accumulated_dist) / segment_length;
            Eigen::Vector3d pt = pre + t*(cur-pre);
            sampled_path.push_back(pt);

            pre = pt;
            segment_length -= (step - accumulated_dist);
            accumulated_dist = 0.0;
        }

        accumulated_dist += segment_length;
        pre = cur;
    }

    if (sampled_path.back() != src.back())
    {
        sampled_path.push_back(src.back());
    }

    return sampled_path;
}

std::vector<Eigen::Matrix4d> AUTOCONTROL::path_resampling(const std::vector<Eigen::Matrix4d>& src, double step)
{
    if(src.size() < 2)
    {
        return src;
    }

    std::vector<Eigen::Matrix4d> sampled_path;
    sampled_path.push_back(src.front());

    Eigen::Matrix4d pre = src.front();
    Eigen::Vector3d pre_translation = pre.block<3,1>(0,3);
    Eigen::Quaterniond pre_rotation(pre.block<3,3>(0,0));
    double accumulated_dist = 0.0;

    for(size_t i = 1; i < src.size(); i++)
    {
        Eigen::Matrix4d cur = src[i];
        Eigen::Vector3d cur_translation = cur.block<3,1>(0,3);
        Eigen::Quaterniond cur_rotation(cur.block<3,3>(0,0));

        double segment_length = (cur_translation - pre_translation).norm();

        while(accumulated_dist + segment_length >= step)
        {
            double t = (step - accumulated_dist) / segment_length;

            Eigen::Vector3d interpolated_translation = pre_translation + t * (cur_translation - pre_translation);
            Eigen::Quaterniond interpolated_rotation = pre_rotation.slerp(t, cur_rotation);

            Eigen::Matrix4d interpolated_transform = Eigen::Matrix4d::Identity();
            interpolated_transform.block<3,3>(0,0) = interpolated_rotation.toRotationMatrix();
            interpolated_transform.block<3,1>(0,3) = interpolated_translation;

            sampled_path.push_back(interpolated_transform);

            pre_translation = interpolated_translation;
            pre_rotation = interpolated_rotation;

            segment_length -= (step - accumulated_dist);
            accumulated_dist = 0.0;
        }

        accumulated_dist += segment_length;
        pre_translation = cur_translation;
        pre_rotation = cur_rotation;
    }

    if(sampled_path.back() != src.back())
    {
        sampled_path.push_back(src.back());
    }

    return sampled_path;
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_ccma(const std::vector<Eigen::Vector3d>& src)
{
    // init params
    const int w_ma = 5; // 5
    const int w_cc = 3; // 3
    const int w_ccma = w_ma + w_cc + 1;

    // not enough points
    if(src.size() < w_ccma)
    {
        return src;
    }
    // padding func
    auto add_padding = [](std::vector<Eigen::Vector3d> points, int n_padding)->std::vector<Eigen::Vector3d>
    {
        std::vector<Eigen::Vector3d> padded;

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.front());
        }

        padded.insert(padded.end(), points.begin(), points.end());

        for (int i = 0; i < n_padding; ++i)
        {
            padded.push_back(points.back());
        }

        return padded;
    };

    // calc unit vector function
    auto get_unit_vector = [](const Eigen::Vector3d vec)->Eigen::Vector3d
    {
        double norm = vec.norm();
        if(norm > 0.0)
        {
            return vec/norm;
        }
        else
        {
            return vec;
        }
    };

    // calc pascal's triangle distribution
    std::function<std::vector<int>(int)> get_pascal_row = [&](int row_index)
    {
        std::vector<int> cur_row(1, 1);
        if(row_index == 0)
        {
            return cur_row;
        }

        std::vector<int> prev = get_pascal_row(row_index - 1);

        for(int idx = 1; idx < (int)prev.size(); idx++)
        {
            int cur = prev[idx-1] + prev[idx];
            cur_row.push_back(cur);
        }

        cur_row.push_back(1);
        return cur_row;
    };

    // calc weight kernels
    std::vector<std::vector<double>> weights_ma;
    for(int i = 0; i < w_ma + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_ma.push_back(tmp);
    }

    std::vector<std::vector<double>> weights_cc;
    for(int i = 0; i < w_cc + 1; i++)
    {
        int pascal_row_index = i * 2;
        std::vector<int> row = get_pascal_row(pascal_row_index);
        int row_sum = std::accumulate(row.begin(), row.end(), 0);

        std::vector<double> tmp;
        for(int j = 0; j < (int)row.size(); j++)
        {
            tmp.push_back((double)row[j]/row_sum);
        }

        weights_cc.push_back(tmp);
    }

    // convert 2d points to 3d points
    std::vector<Eigen::Vector3d> points;
    for(size_t p = 0; p < src.size(); p++)
    {
        points.push_back(Eigen::Vector3d(src[p][0], src[p][1], 0));
    }
    points = add_padding(points, w_ccma);

    // calc moving average points
    std::vector<Eigen::Vector3d> points_ma;
    for(int i = w_ma; i < (int)points.size() - w_ma; i++)
    {
        // convolution
        Eigen::Vector3d pt(0,0,0);
        for(int j = -w_ma; j <= w_ma; j++)
        {
            pt += weights_ma[w_ma][w_ma+j]*points[i+j];
        }

        points_ma.push_back(pt);
    }

    // calc curvature vector
    std::vector<Eigen::Vector3d> curvature_vector(points_ma.size(), Eigen::Vector3d(0,0,0));
    for (size_t p = 1; p < points_ma.size() - 1; p++)
    {
        Eigen::Vector3d p0 = points_ma[p-1];
        Eigen::Vector3d p1 = points_ma[p];
        Eigen::Vector3d p2 = points_ma[p+1];

        Eigen::Vector3d v1 = p1 - p0;
        Eigen::Vector3d v2 = p2 - p1;
        Eigen::Vector3d cross = v1.cross(v2);
        double cross_norm = cross.norm();

        double curvature = 0.0;
        if (cross_norm > 0.0)
        {
            double radius = v1.norm()*v2.norm()*(p2-p0).norm()/(2*cross_norm);
            curvature = 1.0/radius;
        }
        else
        {
            curvature = 0;
        }

        curvature_vector[p] = curvature * get_unit_vector(cross);
    }

    // calc curvatures
    std::vector<double> curvatures(curvature_vector.size(), 0);
    for(size_t p = 0; p < curvature_vector.size(); p++)
    {
        curvatures[p] = curvature_vector[p].norm();
    }

    // calc alphas
    std::vector<double> alphas(points_ma.size(), 0);
    for (size_t p = 1; p < points_ma.size()-1; p++)
    {
        if (curvatures[p] > 0.0)
        {
            double radius = 1.0/curvatures[p];
            double dist_neighbors = (points_ma[p+1] - points_ma[p-1]).norm();
            alphas[p] = std::sin((dist_neighbors/2)/radius);
        }
        else
        {
            alphas[p] = 0.0;
        }
    }

    // calc radii_ma
    std::vector<double> radii_ma(alphas.size(), 0);
    for (size_t i = 1; i < alphas.size()-1; i++)
    {
        radii_ma[i] = weights_ma[w_ma][w_ma];
        for (int k = 1; k <= w_ma; k++)
        {
            radii_ma[i] += 2*std::cos(alphas[i]*k) * weights_ma[w_ma][w_ma+k];
        }

        // Apply threshold
        radii_ma[i] = std::max(0.35, radii_ma[i]);
    }

    // calc ccma
    std::vector<Eigen::Vector3d> points_ccma(points.size() - 2*w_ccma, Eigen::Vector3d(0,0,0));
    for (size_t idx = 0; idx < points_ccma.size(); idx++)
    {
        // Get tangent vector for the shifting point
        Eigen::Vector3d unit_tangent = get_unit_vector(points_ma[w_cc + idx + 1 + 1] - points_ma[w_cc + idx - 1 + 1]);

        // Calculate the weighted shift
        Eigen::Vector3d shift(0,0,0);
        for (int idx_cc = 0; idx_cc < 2*w_cc + 1; idx_cc++)
        {
            if (curvatures[idx + idx_cc + 1] > 0.0)
            {
                Eigen::Vector3d u = get_unit_vector(curvature_vector[idx + idx_cc + 1]);
                double weight = weights_cc[w_cc][idx_cc];
                double shift_magnitude = (1.0/curvatures[idx + idx_cc + 1]) * (1.0/radii_ma[idx + idx_cc + 1] - 1);
                shift += u * weight * shift_magnitude;
            }
        }

        // Reconstruction
        points_ccma[idx] = points_ma[idx + w_cc + 1] + unit_tangent.cross(shift);
    }

    // result
    std::vector<Eigen::Vector3d> res(src.size(), Eigen::Vector3d(0, 0, 0));
    res[0] = src[0];
    for(size_t p = 1; p < points_ccma.size()-1; p++)
    {
        res[p] = Eigen::Vector3d(points_ccma[p][0], points_ccma[p][1], src[p][2]);
    }
    res.back() = src.back();
    return res;
}

void AUTOCONTROL::calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step)
{
    if(src.size() == 1)
    {
        std::vector<double> _ref_v(src.size(), st_v);
        ref_v = _ref_v;
        return;
    }

    const int ld = params.DRIVE_L/step;

    std::vector<double> _ref_th(src.size(), 0);
    for(size_t p = 0; p < src.size(); p++)
    {
        int cur_idx = p-ld;
        if(cur_idx < 0)
        {
            cur_idx = 0;
        }

        int tgt_idx = p+ld;
        if(tgt_idx > (int)src.size()-1)
        {
            tgt_idx = src.size()-1;
        }

        if(cur_idx == tgt_idx)
        {
            continue;
        }

        double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
        double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
        double th = std::atan2(dy, dx);
        _ref_th[p] = th;
    }

    std::vector<double> _ref_v(src.size(), params.LIMIT_V);
    for(size_t p = 0; p < src.size(); p++)
    {
        int cur_idx = p-ld;
        if(cur_idx < 0)
        {
            cur_idx = 0;
        }

        int tgt_idx = p+ld;
        if(tgt_idx > (int)src.size()-1)
        {
            tgt_idx = src.size()-1;
        }

        if(cur_idx == tgt_idx)
        {
            continue;
        }

        double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
        double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(std::atan2(dy, dx), _ref_th[cur_idx]) * params.DRIVE_H;

        double t_v = err_d/params.LIMIT_V;
        double t_w = std::abs(err_th)/(params.LIMIT_W*D2R);
        double t = std::max<double>(t_v, t_w);
        double v = err_d/t;
        _ref_v[p] = v;
    }
    _ref_v[0] = st_v;

    // update result
    ref_v = _ref_v;
}

std::vector<double> AUTOCONTROL::smoothing_v(const std::vector<double>& src, double path_step)
{
    const double v_limit = params.LIMIT_V;
    const double v_acc = params.LIMIT_V_ACC;
    const double v_dcc = params.LIMIT_V_DCC;

    std::vector<double> list0(src.size());
    double v0 = src.front();
    for(int p = 0; p < (int)src.size(); p++)
    {
        double v1 = std::sqrt(2*v_acc*path_step + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(src[p] > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = src[p];
        }

        list0[p] = v0;
    }

    std::vector<double> list1(src.size());
    v0 = src.back();
    for(int p = (int)src.size()-1; p >= 0; p--)
    {
        double v1 = std::sqrt(2*v_dcc*path_step + v0*v0);
        if(v1 > v_limit)
        {
            v1 = v_limit;
        }

        if(src[p] > v0)
        {
            v0 = v1;
        }
        else
        {
            v0 = src[p];
        }

        list1[p] = v0;
    }

    std::vector<double> res(src.size());
    for(size_t p = 0; p < src.size(); p++)
    {
        res[p] = std::min<double>(list0[p], list1[p]);
    }
    return res;
}

// for local path planning
std::vector<Eigen::Matrix4d> AUTOCONTROL::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d _cur_tf)
{
    std::vector<Eigen::Matrix4d> res;

    double vx = cur_vel[0];
    double vy = cur_vel[1];
    double wz = cur_vel[2];

    double pre_x = 0;
    double pre_y = 0;
    double pre_th = 0;

    for(double t = 0; t <= predict_t; t += dt)
    {
        pre_th = pre_th + wz*dt;

        Eigen::Matrix3d pre_rot = Eigen::Matrix3d::Zero();

        pre_rot << std::cos(pre_th), -std::sin(pre_th), 0,
                   std::sin(pre_th),  std::cos(pre_th), 0,
                                  0,                 0, 1;

        Eigen::Vector3d local_vel = Eigen::Vector3d(vx, vy, 0);
        Eigen::Vector3d vel = pre_rot*local_vel;

        pre_x += vel[0]*dt;
        pre_y += vel[1]*dt;

        Eigen::Matrix4d G = se2_to_TF(Eigen::Vector3d(pre_x, pre_y, pre_th));
        Eigen::Matrix4d predict_G = _cur_tf*G;
        res.push_back(predict_G);
    }

    return res;
}

int AUTOCONTROL::get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos)
{
    if(path.size() == 1)
    {
        return 0;
    }
    else
    {
        int min_idx = 0;
        double min_d = 99999999;
        for(size_t p = 0; p < path.size()-1; p++)
        {
            double d = calc_seg_dist(path[p], path[p+1], cur_pos);
            if(d < min_d)
            {
                min_d = d;

                double d0 = (path[p]-cur_pos).norm();
                double d1 = (path[p+1]-cur_pos).norm();
                if(d0 < d1)
                {
                    min_idx = p;
                }
                else
                {
                    min_idx = p+1;
                }
            }
        }

        return min_idx;
    }
}

PATH AUTOCONTROL::calc_local_path(PATH& global_path)
{
    // get cur params
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get global path segment
    std::vector<Eigen::Matrix4d> _path_pose;
    std::vector<Eigen::Vector3d> _path_pos;
    int range = config->get_obs_local_goal_dist()/GLOBAL_PATH_STEP;
    int st_idx = saturation(cur_idx - 10, 0, global_path.pos.size()-2);
    int ed_idx = saturation(cur_idx + range, 0, global_path.pos.size()-1);
    for(int p = st_idx; p <= ed_idx; p++)
    {
        _path_pose.push_back(global_path.pose[p]);
        _path_pos.push_back(global_path.pos[p]);
    }
    double st_v = global_path.ref_v[st_idx];

    if(_path_pose.size() == 1)
    {
        std::vector<double> ref_v;
        ref_v.push_back(st_v);

        // set result
        PATH res;
        res.t = get_time();
        res.pose = _path_pose;
        res.pos = _path_pos;
        res.ref_v = ref_v;
        res.ed_tf = _path_pose.back(); // local goal
        return res;
    }
    else
    {
        // resampling
        std::vector<Eigen::Matrix4d> path_pose = reorientation_path(_path_pose);
        path_pose = path_resampling(path_pose, LOCAL_PATH_STEP);

        std::vector<Eigen::Vector3d> path_pos;
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pos.push_back(path_pose[p].block(0,3,3,1));
        }

        // ccma
        path_pos = path_ccma(path_pos);
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pose[p].block(0,3,3,1) = path_pos[p];
        }

        // calc ref_v
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, st_v, LOCAL_PATH_STEP);

        // check global path end
        double d = calc_dist_2d(global_path.ed_tf.block(0,3,3,1) - path_pos.back());
        if(d < config->get_drive_goal_dist())
        {
            ref_v.back() = params.ED_V;
        }

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

        // set result
        PATH res;
        res.t = get_time();
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_v = ref_v;
        res.ed_tf = path_pose.back(); // local goal
        return res;
    }
}

PATH AUTOCONTROL::calc_avoid_path(PATH& global_path)
{
    double st_time = get_time();

    // get cur params
    Eigen::Vector3d cur_vel = mobile->get_pose().vel;
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get local goal
    int tgt_idx = cur_idx + config->get_obs_local_goal_dist()/GLOBAL_PATH_STEP;
    if(tgt_idx > (int)global_path.pose.size()-1)
    {
        tgt_idx = global_path.pose.size()-1;
    }

    // local goal collision check
    Eigen::Matrix4d tgt_tf = global_path.pose[tgt_idx];
    {
        const int range = 0.5/GLOBAL_PATH_STEP;
        int _tgt_idx = tgt_idx;
        for(int p = cur_idx; p <= tgt_idx - range; p++)
        {
            bool is_collision = false;
            for(int q = 0; q <= range; q++)
            {
                int i = p + q;
                if(obsmap->is_tf_collision(global_path.pose[i], false, config->get_obs_path_margin_x(), config->get_obs_path_margin_y()))
                {
                    is_collision = true;
                    break;
                }
            }

            if(!is_collision)
            {
                _tgt_idx = p + range;
            }
        }

        // update
        tgt_idx = _tgt_idx;
        tgt_tf = global_path.pose[_tgt_idx];
    }

    // set st, ed
    Eigen::Matrix4d st_tf = cur_tf;
    Eigen::Matrix4d ed_tf = tgt_tf;

    // for plot
    {
        std::lock_guard<std::mutex> lock(mtx);
        last_cur_pos = st_tf.block(0,3,3,1);
        last_local_goal = ed_tf.block(0,3,3,1);
    }

    // solve
    std::vector<Eigen::Matrix4d> path_pose = obsmap->calc_path(st_tf, ed_tf);
    if(path_pose.size() > 0)
    {
        // sample and interpolation
        path_pose = reorientation_path(path_pose);
        path_pose = path_resampling(path_pose, LOCAL_PATH_STEP);
        std::vector<Eigen::Vector3d> path_pos;
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pos.push_back(path_pose[p].block(0,3,3,1));
        }

        // set ref_v
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, params.ST_V, LOCAL_PATH_STEP);
        ref_v.back() = params.ED_V;

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

        // set result
        PATH res;
        res.t = get_time();
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_v = ref_v;
        res.ed_tf = path_pose.back(); // avoid goal

        printf("[AUTO] hybrid astar lpp, exact solution found, dt: %f\n", get_time()-st_time);
        return res;
    }
    else
    {
        printf("[AUTO] hybrid astar lpp, solution failed\n");
        return PATH();
    }
}

// check condition
int AUTOCONTROL::is_everything_fine()
{
    // check localization
    QString loc_state = loc->get_cur_loc_state();
    if(loc_state == "none" || loc_state == "fail")
    {
        logger->write_log("[AUTO] localization fail, auto-drive stop", "Red", true, false);
        return DRIVING_FAILED;
    }

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.connection_m0 != 1 || ms.connection_m1 != 1)
    {
        logger->write_log("[AUTO] failed (motor not connected)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(ms.status_m0 > 1 || ms.status_m1 > 1)
    {
        int motor_err_code = ms.status_m0 > 1 ? ms.status_m0 : ms.status_m1;
        if(motor_err_code == MOTOR_ERR_MOD)
        {
            logger->write_log("[AUTO] failed (motor error MOD, 2)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_JAM)
        {
            logger->write_log("[AUTO] failed (motor error JAM, 4)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_CUR)
        {
            logger->write_log("[AUTO] failed (motor error CUR, 8)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_BIG)
        {
            logger->write_log("[AUTO] failed (motor error BIG, 16)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_IN)
        {
            logger->write_log("[AUTO] failed (motor error IN, 32)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_PSI)
        {
            logger->write_log("[AUTO] failed (motor error:PS1|2, 64)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_NON)
        {
            logger->write_log("[AUTO] failed (motor error NON, 128)", "Red", true, false);
        }
        return DRIVING_FAILED;
    }

    if(ms.charge_state == 1)
    {
        logger->write_log("[AUTO] failed (robot charging)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(ms.motor_stop_state == 0)
    {
        logger->write_log("[AUTO] not ready (emo pushed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    if(ms.status_m0 == 0 && ms.status_m1 == 0)
    {
        logger->write_log("[AUTO] not ready (motor lock offed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    return DRIVING_FINE;
}

void AUTOCONTROL::control_loop()
{
    // set flag
    is_moving = true;

    set_multi_req("recv_path");
    set_obs_condition("none");
    set_cur_goal_state("move");

    // check global path
    logger->write_log(QString("[AUTO] global path que size: %1").arg((int)global_path_que.unsafe_size()));
    PATH global_path;
    if(global_path_que.try_pop(global_path))
    {
        // update global path and goal_tf
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_global_path = global_path;
            logger->write_log(QString("[AUTO] deque global path, size: %1").arg(cur_global_path.pose.size()));
        }

        Q_EMIT signal_global_path_updated();
    }

    if(global_path.pose.size() == 0)
    {
        logger->write_log(QString("[AUTO] global path invalid"));
        set_multi_req("none");
        set_obs_condition("none");
        set_cur_goal_state("fail");
        return;
    }

    // update goal
    Eigen::Matrix4d goal_tf = global_path.ed_tf;
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);

    // set initial state
    fsm_state = AUTO_FSM_FIRST_ALIGN;

    // check already goal
    if(global_path_que.unsafe_size() == 0)
    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d dxi = TF_to_se2(cur_tf.inverse()*goal_tf);
        double err_d = calc_dist_2d(dxi);
        double err_th = std::abs(dxi[2]);
        if(err_d < config->get_drive_goal_dist())
        {
            if(!global_path.is_final)
            {
                // temp goal reached
                mobile->move(0, 0, 0);
                is_moving = false;
                is_pause = false;

                set_multi_req("none");
                set_obs_condition("none");
                set_cur_goal_state("move");

                clear_path();

                fsm_state = AUTO_FSM_COMPLETE;
                logger->write_log(QString("[AUTO] COMPLETE (already temp goal), err_d:%1").arg(err_d));
                return;
            }

            if(err_th < config->get_drive_goal_th()*D2R)
            {
                // final goal reached
                mobile->move(0, 0, 0);
                is_moving = false;
                is_pause = false;

                set_multi_req("none");
                set_obs_condition("none");
                set_cur_goal_state("complete");

                clear_path();

                // move response
                Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
                {
                    DATA_MOVE _move_info;
                    _move_info.cur_pos = cur_pos;
                    _move_info.result = "success";
                    _move_info.message = "already goal";
                    _move_info.time = get_time();
                    Q_EMIT signal_move_response(_move_info);

                    std::lock_guard<std::mutex> lock(mtx);
                    cur_move_info = _move_info;
                }

                fsm_state = AUTO_FSM_COMPLETE;
                logger->write_log(QString("[AUTO] COMPLETE (already goal), err_d: %1, err_th: %2").arg(err_d).arg(err_th*R2D));
                return;
            }
            else
            {
                // jump to final align
                fsm_state = AUTO_FSM_FINAL_ALIGN;
            }
        }
    }
    //logger->write_log(QString("[AUTO] initial fsm state: %1").arg(AUTO_FSM_STATE_STR[fsm_state]));

    // path storage
    PATH local_path;
    PATH avoid_path;

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;
    double pre_err_th = 0;

    // for obs
    int obs_state = AUTO_OBS_CHECK;
    int cur_obs_val = OBS_NONE;
    double obs_wait_st_time = 0;

    int loop_cnt = 0;
    logger->write_log("[AUTO] b_loop_pp start");
    while(control_flag)
    {
        loop_cnt++;
        if(loop_cnt % 20 == 0)
        {
            qDebug() << "[AUTO] b_loop_pp alive fsm_state: " << fsm_state << ", time: " << QString::number(get_time(), 'f', 3);
        }

        // get current status
        Eigen::Matrix4d cur_tf     = loc->get_cur_tf();
        Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
        Eigen::Vector3d cur_xi     = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos    = cur_tf.block(0,3,3,1);
        Eigen::Vector3d cur_vel    = mobile->get_control_input();

        // for plot
        {
            std::lock_guard<std::mutex> lock(mtx);
            last_cur_pos = cur_pos;
        }

        // check everything
        int is_good_everything = is_everything_fine();
        if(is_good_everything == DRIVING_FAILED)
        {
            // clear
            mobile->move(0, 0, 0);
            is_moving = false;
            is_pause = false;

            set_multi_req("none");
            set_obs_condition("none");
            set_cur_goal_state("fail");

            clear_path();

            // send move response
            {
                DATA_MOVE _move_info;
                _move_info.cur_pos = cur_pos;
                _move_info.result = "fail";
                _move_info.message = "something wrong";
                _move_info.time = get_time();
                Q_EMIT signal_move_response(_move_info);

                std::lock_guard<std::mutex> lock(mtx);
                cur_move_info = _move_info;
            }

            fsm_state = AUTO_FSM_COMPLETE;
            logger->write_log("[AUTO] something wrong (fail)");
            return;
        }
        else if(is_good_everything == DRIVING_NOT_READY)
        {
            // clear
            mobile->move(0, 0, 0);
            is_moving = false;
            is_pause = false;

            set_multi_req("none");
            set_obs_condition("none");
            set_cur_goal_state("fail");

            clear_path();

            // move response
            //if(config->USE_RRS)
            {
                mtx.lock();
                cur_move_info.cur_pos = cur_pos;
                cur_move_info.result = "fail";
                cur_move_info.message = "not ready";
                cur_move_info.time = get_time();
                mtx.unlock();

                Q_EMIT signal_move_response(cur_move_info);
            }

            fsm_state = AUTO_FSM_COMPLETE;
            logger->write_log("[AUTO] something wrong (not ready)");
            return;
        }

        // pause
        if(is_pause)
        {
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // calc local path
        if(fsm_state != AUTO_FSM_FINAL_ALIGN)
        {
            if(avoid_path.pos.size() > 0)
            {
                // clear avoid path
                double goal_err_d = calc_dist_2d(avoid_path.pos.back() - goal_pos);
                if(goal_err_d >= config->get_drive_goal_dist())
                {
                    int avoid_idx = get_nn_idx(avoid_path.pos, cur_pos);
                    if(avoid_idx > avoid_path.pos.size()*0.9)
                    {
                        // clear avoid path
                        avoid_path = PATH();
                        logger->write_log("[AUTO] avoid_path complete");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }

                // set avoid path to local path
                local_path = avoid_path;

                // for plot
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_local_path = local_path;
                    last_local_goal = local_path.ed_tf.block(0,3,3,1);
                }

                Q_EMIT signal_local_path_updated();
            }
            else
            {
                // calc local path
                if(get_time() - local_path.t > 0.2)
                {
                    // update local path
                    local_path = calc_local_path(global_path);

                    // for plot
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_local_path = local_path;
                        last_local_goal = local_path.ed_tf.block(0,3,3,1);
                    }

                    Q_EMIT signal_local_path_updated();
                }
            }
        }

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + params.DRIVE_L/LOCAL_PATH_STEP;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            Eigen::Matrix4d tgt_tf  = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_pos = local_path.pos[tgt_idx];
            {
                std::lock_guard<std::mutex> lock(mtx);
                last_tgt_pos = tgt_pos;
            }

            // calc error
            double dx = local_path.pos[tgt_idx][0] - local_path.pos[cur_idx][0];
            double dy = local_path.pos[tgt_idx][1] - local_path.pos[cur_idx][1];
            double err_th = deltaRad(std::atan2(dy,dx), cur_xi[2]);
            if(pre_err_th == 0)
            {
                pre_err_th = err_th;
            }

            // goal check
            if(std::abs(err_th) < config->get_drive_goal_th()*D2R)
            {
                extend_dt = 0;
                pre_err_th = 0;
                mobile->move(cur_vel[0], 0, 0);

                fsm_state = AUTO_FSM_DRIVING;
                logger->write_log(QString("[AUTO] FIRST_ALIGN -> DRIVING, err_th:%1").arg(err_th*R2D));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // for safe
            double v0 = mobile->get_pose().vel[0];
            if(std::abs(v0) > 0.1)
            {
                w = 0;
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(0, 0, w), 0.2, config->get_obs_predict_time(), cur_tf);
            cur_obs_val = obsmap->is_path_collision(traj, true);
            if(cur_obs_val != OBS_NONE)
            {
                mobile->move(0, 0, 0);

                set_cur_goal_state("obstacle");

                obs_state = AUTO_OBS_CHECK;
                fsm_state = AUTO_FSM_OBS;
                logger->write_log(QString("[AUTO] FIRST_ALIGN -> OBS, err_th:%1").arg(err_th*R2D));
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // send control
            if(!is_debug)
            {
                mobile->move(0, 0, w);
            }
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + params.DRIVE_L/LOCAL_PATH_STEP;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            Eigen::Matrix4d tgt_tf  = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_pos = local_path.pos[tgt_idx];
            {
                std::lock_guard<std::mutex> lock(mtx);
                last_tgt_pos = tgt_pos;
            }

            // goal check
            int max_idx = global_path.pos.size()-1;
            int goal_idx = get_nn_idx(global_path.pos, cur_pos);
            if(max_idx - goal_idx < 2)
            {
                Eigen::Vector3d _goal_pos = cur_tf_inv.block(0,0,3,3)*goal_pos + cur_tf_inv.block(0,3,3,1);
                double v0 = cur_vel[0];
                double v = saturation(config->get_drive_goal_approach_gain()*_goal_pos[0], v0 - params.LIMIT_V_DCC*dt, v0 + params.LIMIT_V_DCC*dt);
                v = saturation(v, -params.ED_V, params.ED_V);
                mobile->move(v, 0, 0);

                extend_dt += dt;
                if(extend_dt > config->get_drive_extended_control_time())
                {
                    double goal_err_d = calc_dist_2d(_goal_pos);

                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    if(global_path.is_final)
                    {
                        fsm_state = AUTO_FSM_FINAL_ALIGN;
                        logger->write_log(QString("[AUTO] DRIVING -> FINAL_ALIGN, err_d:%1").arg(goal_err_d));
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        // deque
                        PATH _global_path;
                        if(global_path_que.try_pop(_global_path))
                        {
                            // update global path
                            global_path = _global_path;
                            logger->write_log(QString("[AUTO] next global path, deque global path, size: %1").arg(global_path.pos.size()));

                            // update global goal
                            goal_tf = global_path.ed_tf;
                            goal_pos = goal_tf.block(0,3,3,1);
                            goal_xi = TF_to_se2(goal_tf);

                            // clear local path
                            local_path = calc_local_path(global_path);
                            avoid_path = PATH();

                            {
                                std::lock_guard<std::mutex> lock(mtx);

                                // update global path and goal_tf
                                cur_global_path = global_path;

                                // update local path
                                cur_local_path = local_path;
                                last_local_goal = local_path.ed_tf.block(0,3,3,1);

                                Q_EMIT signal_global_path_updated();
                                Q_EMIT signal_local_path_updated();
                            }

                            // return to first align
                            extend_dt = 0;
                            pre_err_th = 0;
                            mobile->move(0, 0, 0);

                            fsm_state = AUTO_FSM_FIRST_ALIGN;
                            logger->write_log(QString("[AUTO] next global path, DRIVING -> FIRST_ALIGN, err_d:%1").arg(goal_err_d));
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            continue;
                        }
                        else
                        {
                            // finish temp goal reached
                            extend_dt = 0;
                            pre_err_th = 0;
                            mobile->move(0, 0, 0);
                            is_moving = false;
                            is_pause = false;

                            set_multi_req("none");
                            set_obs_condition("none");
                            set_cur_goal_state("move");

                            clear_path();

                            fsm_state = AUTO_FSM_COMPLETE;
                            logger->write_log(QString("[AUTO] DRIVING -> COMPLETE(temp goal), err_d:%1").arg(goal_err_d));
                            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            return;
                        }
                    }
                }

                // for real time loop
                double cur_loop_time = get_time();
                double delta_loop_time = cur_loop_time - pre_loop_time;
                if(delta_loop_time < dt)
                {
                    int sleep_ms = (dt-delta_loop_time)*1000;
                    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
                }
                else
                {
                    logger->write_log(QString("[AUTO] loop time drift, dt:%1").arg(delta_loop_time));
                }
                pre_loop_time = get_time();
                continue;

                //std::this_thread::sleep_for(std::chrono::milliseconds(50));
                //continue;
            }

            // obs decel
            QString _obs_condition = "none";
            double obs_v = config->get_obs_map_min_v();
            for(double vv = config->get_obs_map_min_v(); vv <= params.LIMIT_V+0.01; vv += 0.025)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, config->get_obs_predict_time(), cur_tf);

                bool is_collision = false;
                for(size_t p = 0; p < traj.size(); p++)
                {
                    cur_obs_val = obsmap->is_tf_collision(traj[p], true, config->get_obs_safe_margin_x(), config->get_obs_safe_margin_y());
                    if(cur_obs_val != OBS_NONE)
                    {
                        is_collision = true;
                        break;
                    }
                }

                if(is_collision)
                {
                    _obs_condition = "far";
                    break;
                }

                obs_v = vv;
            }

            // obs stop
            {
                int chk_idx = cur_idx + config->get_obs_deadzone()/LOCAL_PATH_STEP;
                if(chk_idx > (int)local_path.pos.size()-1)
                {
                    chk_idx = local_path.pos.size()-1;
                }

                std::vector<Eigen::Matrix4d> traj;
                for(int p = cur_idx; p <= chk_idx; p++)
                {
                    traj.push_back(local_path.pose[p]);
                }

                cur_obs_val = obsmap->is_path_collision(traj, true, 0, 0, 0, 10);
                if(cur_obs_val != OBS_NONE)
                {
                    mobile->move(0, 0, 0);

                    set_cur_goal_state("obstacle");

                    obs_state = AUTO_OBS_CHECK;
                    fsm_state = AUTO_FSM_OBS;
                    logger->write_log(QString("[AUTO] DRIVING -> OBS, cur_obs_val:%1, fsm_state:%2").arg(cur_obs_val).arg((int)fsm_state));
                    qDebug() << "driving -> obs";
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }

            // for mobile server
            set_obs_condition(_obs_condition);

            // calc heading error
            Eigen::Matrix4d _tgt_tf = cur_tf_inv*tgt_tf;
            Eigen::Vector3d _tgt_xi = TF_to_se2(_tgt_tf);
            double err_d = calc_dist_2d(_tgt_xi);
            double err_th = _tgt_xi[2];
            if(pre_err_th == 0)
            {
                pre_err_th = err_th;
            }

            // calc cross track error
            double cte = calc_cte(local_path.pose, cur_pos);

            // get ref_v
            double ref_v = local_path.ref_v[cur_idx];

            // calc control input
            double v0 = cur_vel[0];
            double v = std::min<double>((params.LIMIT_V/params.DRIVE_L)*err_d, ref_v);
            v = saturation(v, 0.0, obs_v);
            v = saturation(v, v0 - config->get_motor_limit_v_acc()*dt, v0 + config->get_motor_limit_v_acc()*dt);
            v = saturation(v, -params.LIMIT_V, params.LIMIT_V);

            double th = (params.DRIVE_A * err_th)
                        + (params.DRIVE_B * (err_th-pre_err_th)/dt)
                        + std::atan2(params.DRIVE_K * cte, v + params.DRIVE_EPS);
            th = saturation(th, -45.0*D2R, 45.0*D2R);
            pre_err_th = err_th;

            double w0 = cur_vel[2];
            double w = std::tan(th) / params.DRIVE_L;
            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - params.DRIVE_T*std::abs(w/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(v/params.LIMIT_V);
            v *= scale_v;
            w *= scale_w;

            // deadzone v
            double d_v = config->get_drive_v_deadzone();
            if(std::abs(v) < d_v)
            {
                v = sgn(v)*d_v;
            }

            // deadzone w
            double d_w = config->get_drive_w_deadzone()*D2R;
            if(std::abs(w) < d_w)
            {
                w = 0;
            }
            else
            {
                w = sgn(w)*(std::abs(w)-d_w);
            }

            // send control
            if(!is_debug)
            {
                mobile->move(v, 0, w);
            }
            //printf("v:%f, w:%f, err_th:%f, cte:%f, ref_v:%f, obs_v:%f\n", v, w*R2D, err_th*R2D, cte, ref_v, obs_v);
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            // calc heading error
            double err_th = deltaRad(goal_xi[2], cur_xi[2]);
            if(pre_err_th == 0)
            {
                pre_err_th = err_th;
            }

            // goal check
            if(std::abs(err_th) < config->get_drive_goal_th()*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->get_drive_extended_control_time())
                {
                    mobile->move(0, 0, 0);
                    is_moving = false;
                    is_pause = false;

                    set_multi_req("none");
                    set_obs_condition("none");
                    set_cur_goal_state("complete");

                    clear_path();

                    // response
                    //if(config->USE_RRS)
                    {
                        DATA_MOVE _move_info;
                        _move_info.cur_pos = cur_pos;
                        _move_info.result = "success";
                        _move_info.message = "very good";
                        _move_info.time = get_time();
                        Q_EMIT signal_move_response(_move_info);

                        std::lock_guard<std::mutex> lock(mtx);
                    }

                    fsm_state = AUTO_FSM_COMPLETE;
                    logger->write_log(QString("[AUTO] FINAL ALIGN COMPLETE(good), err_th: %1").arg(err_th*R2D));
                    return;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = intp_tf(cur_tf, goal_tf, 0.2, 10.0*D2R);
            cur_obs_val = obsmap->is_path_collision(traj, true);
            if(cur_obs_val == OBS_DYN)
            {
                mobile->move(0, 0, 0);

                set_cur_goal_state("obstacle");

                obs_wait_st_time = get_time();
                obs_state = AUTO_OBS_WAIT2;
                fsm_state = AUTO_FSM_OBS;

                logger->write_log("[AUTO] FINAL ALIGN -> OBS");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // send control
            if(!is_debug)
            {
                mobile->move(0, 0, w);
            }
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(obs_state == AUTO_OBS_CHECK)
            {
                qDebug() << "obs check";

                if(cur_obs_val == OBS_DYN)
                {
                    // for mobile server
                    set_obs_condition("near");

                    if(config->get_obs_avoid_mode() == 0)
                    {
                        // mode 0, just wait
                        obs_wait_st_time = get_time();
                        obs_state = AUTO_OBS_WAIT;
                        logger->write_log("[AUTO] avoid mode 0, OBS_WAIT");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                    else
                    {
                        // mode 1, avoid
                        obs_state = AUTO_OBS_AVOID;
                        logger->write_log("[AUTO] avoid mode 1, OBS_AVOID");
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                        continue;
                    }
                }
                else if(cur_obs_val == OBS_VIR)
                {
                    // for mobile server
                    set_obs_condition("vir");

                    // for vobs works
                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_VIR;
                    logger->write_log("[AUTO] OBS_VIR");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                else
                {
                    // just wait
                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    logger->write_log("[AUTO] OBS_WAIT");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_AVOID)
            {
                // calc avoid path
                avoid_path = calc_avoid_path(global_path);
                if(avoid_path.pos.size() > 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    set_cur_goal_state("move");

                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    logger->write_log("[AUTO] avoid path found, OBS_AVOID -> FIRST_ALIGN");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
                else
                {
                    obs_state = AUTO_OBS_RECOVERY;
                    logger->write_log("[AUTO] avoid path failed, OBS_AVOID -> OBS_RECOVERY");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_RECOVERY)
            {
                std::vector<Eigen::Vector3d> obs_pts = obsmap->get_obs_pts();

                Eigen::Vector3d vel(0,0,0);
                double max_d = 0;
                for(int deg = -30; deg <= 30; deg += 10)
                {
                    std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(-0.1, 0, deg*D2R), 0.1, 1.0, cur_tf);
                    if(obsmap->is_path_collision(traj))
                    {
                        continue;
                    }

                    double min_d = 99999999;
                    for(size_t p = 0; p < obs_pts.size(); p++)
                    {
                        double d = calc_dist_2d(obs_pts[p] - traj.back().block(0,3,3,1));
                        if(d < min_d)
                        {
                            min_d = d;
                        }
                    }

                    if(min_d > max_d)
                    {
                        max_d = min_d;
                        vel[0] = -0.1;
                        vel[1] = 0;
                        vel[2] = deg*D2R;
                    }
                }

                for(int deg = -30; deg <= 30; deg += 10)
                {
                    std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(0.1, 0, deg*D2R), 0.1, 1.0, cur_tf);
                    if(obsmap->is_path_collision(traj))
                    {
                        continue;
                    }

                    double min_d = 99999999;
                    for(size_t p = 0; p < obs_pts.size(); p++)
                    {
                        double d = calc_dist_2d(obs_pts[p] - traj.back().block(0,3,3,1));
                        if(d < min_d)
                        {
                            min_d = d;
                        }
                    }

                    if(min_d > max_d)
                    {
                        max_d = min_d;
                        vel[0] = 0.1;
                        vel[1] = 0;
                        vel[2] = deg*D2R;
                    }
                }

                double margin_dx = config->get_obs_safe_margin_x();
                double margin_dy = config->get_obs_safe_margin_y();

                double margin_d = std::sqrt(margin_dx*margin_dx+ margin_dy*margin_dy);
                if(max_d > config->get_robot_radius() + margin_d)
                {
                    mobile->move(0, 0, 0);
                    obs_state = AUTO_OBS_AVOID;
                    logger->write_log(QString("[AUTO] max_d: %1, OBS_RECOVERY -> OBS_AVOID").arg(max_d));
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                double v0 = cur_vel[0];
                double w0 = cur_vel[2];
                double v = vel[0];
                double w = vel[2];
                v = saturation(v, v0 - params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_ACC*dt);
                v = saturation(v, -params.LIMIT_V, params.LIMIT_V);
                w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
                w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

                if(!is_debug)
                {
                    mobile->move(v, 0, w);
                }
            }
            else if(obs_state == AUTO_OBS_WAIT)
            {
                if(get_time() - obs_wait_st_time > 1.5)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    set_cur_goal_state("move");

                    cur_obs_val = OBS_NONE;
                    obs_state = AUTO_OBS_CHECK;
                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    logger->write_log("[AUTO] OBS_WAIT -> FIRST_ALIGN");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_WAIT2)
            {
                if(get_time() - obs_wait_st_time > 1.5)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    set_cur_goal_state("move");

                    fsm_state = AUTO_FSM_FINAL_ALIGN;
                    logger->write_log("[AUTO] OBS_WAIT -> FINAL_ALIGN");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_VIR)
            {
                // check
                std::vector<Eigen::Vector3d> vir_pts = obsmap->get_vir_pts();
                if(vir_pts.size() == 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    logger->write_log("[AUTO] OBS_VIR(no pts) -> OBS_WAIT");
                    continue;
                }

                // global to local
                std::vector<Eigen::Vector3d> _vir_pts;
                for(size_t p = 0; p < vir_pts.size(); p++)
                {
                    Eigen::Vector3d P = vir_pts[p];
                    Eigen::Vector3d _P = cur_tf_inv.block(0,0,3,3)*P + cur_tf_inv.block(0,3,3,1);
                    _vir_pts.push_back(_P);
                }

                // find min pt
                double min_d = 99999999;
                Eigen::Vector3d min_pt(0, 0, 0);
                for(size_t p = 0; p < _vir_pts.size(); p++)
                {
                    Eigen::Vector3d P = _vir_pts[p];
                    double d = calc_dist_2d(P);
                    if(d < min_d)
                    {
                        min_d = d;
                        min_pt = P;
                    }
                }

                // calc v
                double v = -0.1*sgn(min_pt[0]);

                // check
                if(min_d >= config->get_robot_radius() + 0.05 || get_time() - obs_wait_st_time > 0.5)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    logger->write_log(QString("[AUTO] OBS_VIR -> OBS_WAIT, min_d: %1").arg(min_d));
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    continue;
                }

                if(!is_debug)
                {
                    mobile->move(v, 0, 0);
                }
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            logger->write_log(QString("[AUTO] loop time drift, dt:%1").arg(delta_loop_time));
        }
        pre_loop_time = get_time();
    }

    if(is_path_overlap)
    {
        is_path_overlap = false;
        set_obs_condition("none");
        logger->write_log("[AUTO] path overlap, b_loop_pp stop");
        return;
    }

    // manual stopped
    mobile->move(0, 0, 0);
    is_moving = false;
    is_pause = false;

    set_multi_req("none");
    set_obs_condition("none");
    set_cur_goal_state("cancel");

    clear_path();

    // response
    //if(config->USE_RRS)
    {
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        DATA_MOVE _move_info;
        _move_info.cur_pos = cur_pos;
        _move_info.result = "fail";
        _move_info.message = "manual stopped";
        _move_info.time = get_time();
        Q_EMIT signal_move_response(_move_info);

        std::lock_guard<std::mutex> lock(mtx);
        cur_move_info = _move_info;
    }

    fsm_state = AUTO_FSM_COMPLETE;
    logger->write_log("[AUTO] path stop, b_loop_pp stop");
}

void AUTOCONTROL::set_config_module(CONFIG* _config)
{
    config = _config;
}

void AUTOCONTROL::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void AUTOCONTROL::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}

void AUTOCONTROL::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}

void AUTOCONTROL::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

void AUTOCONTROL::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}
