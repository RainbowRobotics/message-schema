#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
    last_tgt_pos.setZero();    
    last_local_goal.setZero();

    cur_goal_tf.setIdentity();
}

AUTOCONTROL::~AUTOCONTROL()
{
    // control loop stop
    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }
    mobile->move(0, 0, 0);
    is_moving = false;
    clear_path();
}

void AUTOCONTROL::init()
{
    params = load_preset(0);
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path = "";

    // config module init
    #ifdef USE_SRV
    preset_path = QCoreApplication::applicationDirPath() + "/config/SRV/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400_PROTO
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_PROTO/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400_LAKI
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_LAKI/" + "preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_KAI
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_KAI/" + "preset_" + QString::number(preset) + ".json";
    #endif

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

    return res;
}

PATH AUTOCONTROL::get_cur_global_path()
{
    mtx.lock();
    PATH res = cur_global_path;
    mtx.unlock();

    return res;
}

Eigen::Matrix4d AUTOCONTROL::get_cur_goal_tf()
{
    mtx.lock();
    Eigen::Matrix4d res = cur_goal_tf;
    mtx.unlock();

    return res;
}

PATH AUTOCONTROL::get_cur_local_path()
{
    mtx.lock();
    PATH res = cur_local_path;
    mtx.unlock();

    return res;
}

void AUTOCONTROL::clear_path()
{
    mtx.lock();
    cur_global_path = PATH();
    cur_local_path = PATH();
    mtx.unlock();

    Q_EMIT signal_global_path_updated();
    Q_EMIT signal_local_path_updated();
}

QString AUTOCONTROL::get_obs_condition()
{
    mtx.lock();
    QString res = obs_condition;
    mtx.unlock();

    return res;
}

void AUTOCONTROL::stop()
{
    if(is_moving == false)
    {
        return;
    }

    // control loop stop
    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }
    mobile->move(0, 0, 0);

    // clear params
    is_moving = false;
    is_pause = false;    
    clear_path();
}

void AUTOCONTROL::set_goal(QString goal_id)
{
    NODE* node = unimap->get_node_by_id(goal_id);
    if(node == NULL)
    {
        return;
    }

    mtx.lock();
    cur_goal_tf = node->tf;
    mtx.unlock();

    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    QString cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
    QString goal_node_id = unimap->get_node_id_edge(node->tf.block(0,3,3,1));
    if(cur_node_id == goal_node_id)
    {
        printf("[AUTO] refine moving\n");
        Eigen::Vector2d dtdr = dTdR(cur_tf, node->tf);
        if(dtdr[0] > config->DRIVE_GOAL_D || dtdr[1] > config->DRIVE_GOAL_TH*D2R)
        {
            move_pp(node->tf, 0); // preset0
            return;
        }
    }

    // set flag
    is_goal = true;
}

void AUTOCONTROL::move_pp(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // load preset
    params = load_preset(preset);

    // calc global path
    PATH global_path = calc_global_path(goal_tf);
    if(global_path.pos.size() > 0)
    {
        // update global path and goal_tf
        mtx.lock();
        cur_global_path = global_path;
        cur_goal_tf = goal_tf;
        mtx.unlock();

        Q_EMIT signal_global_path_updated();
        printf("[AUTO] global path found\n");
    }

    // start control loop
    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_pp, this);
}

void AUTOCONTROL::move_pp(std::vector<QString> node_path, int preset, int is_align)
{
    // stop
    stop();

    // obs clear
    obsmap->clear();

    // load preset
    params = load_preset(preset);

    // update global path
    PATH path = calc_global_path(node_path, is_align);

    mtx.lock();
    cur_global_path = path;
    mtx.unlock();

    Q_EMIT signal_global_path_updated();
    printf("[AUTO] global path found\n");

    // start control loop
    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_pp, this);
}

PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf)
{
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();

    // get st node id    
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    QString st_node_id = unimap->get_node_id_edge(cur_pos);
    if(st_node_id == "")
    {
        printf("[AUTO] st_node_id empty\n");
        return PATH();
    }

    // set ed node id
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    QString ed_node_id = unimap->get_node_id_edge(goal_pos);
    if(ed_node_id == "")
    {
        printf("[AUTO] ed_node_id empty\n");
        return PATH();
    }
    
    // topology path finding
    std::vector<QString> node_path = topo_path_finding(st_node_id, ed_node_id);
    if(node_path.size() == 0)
    {
        printf("[AUTO] node_path empty\n");
        return PATH();
    }

    // convert metric path
    std::vector<Eigen::Vector3d> node_pos;
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString name = node_path[p];
        NODE* node = unimap->get_node_by_id(name);
        if(node == NULL)
        {
            printf("[AUTO] %s: node null\n", name.toLocal8Bit().data());
            return PATH();
        }

        Eigen::Vector3d pos = node->tf.block(0,3,3,1);
        node_pos.push_back(pos);
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(node_pos.size() == 1)
    {
        node_pos.insert(node_pos.begin(), cur_pos);
        node_pose.insert(node_pose.begin(), cur_tf);
    }
    else
    {
        if(check_point_on_segment(node_pos[0], node_pos[1], cur_pos))
        {
            node_pos.erase(node_pos.begin());
            node_pose.erase(node_pose.begin());

            node_pos.insert(node_pos.begin(), cur_pos);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
        else
        {
            node_pos.insert(node_pos.begin(), cur_pos);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
    }

    // add goal pos    
    node_pos.push_back(goal_pos);
    node_pose.push_back(goal_tf);

    // divide and smooth metric path    
    std::vector<Eigen::Vector3d> path_pos = path_resampling(node_pos, GLOBAL_PATH_STEP);
    path_pos = path_ccma(path_pos);

    // calc pose
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // set ref_v    
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, GLOBAL_PATH_STEP);
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.t = get_time();
    res.nodes = node_path;
    res.pose = path_pose;
    res.pos = path_pos;    
    res.ref_v = ref_v;    
    res.ed_tf = goal_tf;
    res.is_align = 1;
    return res;
}

PATH AUTOCONTROL::calc_global_path(std::vector<QString> node_path, int is_align)
{
    NODE* ed_node = unimap->get_node_by_id(node_path.back());
    if(ed_node == NULL)
    {
        return PATH();
    }

    Eigen::Matrix4d ed_tf = ed_node->tf;
    Eigen::Vector3d ed_pos = ed_tf.block(0,3,3,1);

    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

    // convert metric path
    std::vector<Eigen::Vector3d> node_pos;
    std::vector<Eigen::Matrix4d> node_pose;
    for(size_t p = 0; p < node_path.size(); p++)
    {
        QString name = node_path[p];
        NODE* node = unimap->get_node_by_id(name);
        if(node == NULL)
        {
            printf("[AUTO] %s: node null\n", name.toLocal8Bit().data());
            return PATH();
        }

        Eigen::Vector3d pos = node->tf.block(0,3,3,1);
        node_pos.push_back(pos);
        node_pose.push_back(node->tf);
    }

    // add cur pos
    if(node_pos.size() == 1)
    {
        node_pos.insert(node_pos.begin(), cur_pos);
        node_pose.insert(node_pose.begin(), cur_tf);
    }
    else
    {
        if(!check_point_on_segment(node_pos[0], node_pos[1], cur_pos))
        {
            node_pos.insert(node_pos.begin(), cur_pos);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
        else
        {
            node_pos.erase(node_pos.begin());
            node_pose.erase(node_pose.begin());

            node_pos.insert(node_pos.begin(), cur_pos);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
    }

    // add ed pos
    node_pos.push_back(ed_pos);
    node_pose.push_back(ed_tf);

    // divide and smooth metric path
    std::vector<Eigen::Vector3d> path_pos = path_resampling(node_pos, GLOBAL_PATH_STEP);
    //path_pos = path_ccma(path_pos);

    // calc pose
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // set ref_v
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, GLOBAL_PATH_STEP);
    ref_v.back() = params.ED_V;

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.t = get_time();
    res.nodes = node_path;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
    res.ed_tf = ed_tf;
    res.is_align = is_align;
    return res;
}

std::vector<QString> AUTOCONTROL::topo_path_finding(QString st_node_id, QString ed_node_id)
{
    // set st node, ed node
    NODE* st_node = unimap->get_node_by_id(st_node_id);
    NODE* ed_node = unimap->get_node_by_id(ed_node_id);
    if(st_node == NULL || ed_node == NULL)
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
    sampled_path.push_back(src.front()); // 시작 점 추가

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
        sampled_path.push_back(src.back()); // 끝 점 추가
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

std::vector<Eigen::Vector3d> AUTOCONTROL::sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step, bool use_ccma)
{
    std::vector<Eigen::Vector3d> res = src;
    res = path_resampling(res, large_step);
    if(use_ccma)
    {
        res = path_ccma(res);
    }
    res = path_resampling(res, small_step);
    return res;
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
        _ref_th[p] = std::atan2(dy, dx);
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

    /*
    // for debug
    printf("chk_ref_v\n");
    for(size_t p = 0; p < ref_v.size(); p++)
    {
        printf("p:%d, v:%f\n", (int)p, ref_v[p]);
    }
    printf("\n");
    */
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
std::vector<Eigen::Matrix4d> AUTOCONTROL::calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0)
{
    std::vector<Eigen::Matrix4d> res;

    float vx=cur_vel[0];
    float vy=cur_vel[1];
    float wz=cur_vel[2];

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
        Eigen::Matrix4d predict_G = G0*G;
        res.push_back(predict_G);
    }

    return res;
}

int AUTOCONTROL::get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos)
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

int AUTOCONTROL::get_next_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos)
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

    int res = min_idx + 1;
    if(res > (int)path.size()-1)
    {
        res = path.size()-1;
    }

    return res;
}

PATH AUTOCONTROL::calc_local_path()
{
    // get params
    PATH global_path = get_cur_global_path();

    // get cur params
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get global path segment    
    std::vector<Eigen::Vector3d> _path_pos;
    int range = config->OBS_LOCAL_GOAL_D/GLOBAL_PATH_STEP;
    int st_idx = saturation(cur_idx - 10, 0, global_path.pos.size()-2);
    int ed_idx = saturation(cur_idx + range, 0, global_path.pos.size()-1);
    for(int p = st_idx; p <= ed_idx; p++)
    {
        _path_pos.push_back(global_path.pos[p]);
    }

    std::vector<Eigen::Vector3d> path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP, LOCAL_PATH_STEP);
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // calc ref_v        
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, LOCAL_PATH_STEP);

    PATH last_local_path = get_cur_local_path();
    if(last_local_path.pos.size() > 0)
    {
        int idx0 = get_nn_idx(last_local_path.pos, cur_pos);
        int idx1 = get_nn_idx(path_pos, cur_pos);

        for(int p = 0; p <= idx1; p++)
        {
            ref_v[p] = last_local_path.ref_v[idx0];
        }
    }

    double d = calc_dist_2d(global_path.pos.back() - path_pos.back());
    if(d < config->DRIVE_GOAL_D)
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

PATH AUTOCONTROL::calc_avoid_path()
{
    double st_time = get_time();

    // get global path
    PATH global_path = get_cur_global_path();

    // get cur params
    Eigen::Vector3d cur_vel = mobile->get_pose().vel;
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get local goal
    int tgt_idx = std::min<int>(cur_idx + config->OBS_LOCAL_GOAL_D/GLOBAL_PATH_STEP, (int)global_path.pose.size()-1);
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
                if(obsmap->is_tf_collision(global_path.pose[i], config->OBS_PATH_MARGIN_X, config->OBS_PATH_MARGIN_Y))
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
    mtx.lock();
    last_cur_pos = st_tf.block(0,3,3,1);
    last_local_goal = ed_tf.block(0,3,3,1);
    mtx.unlock();

    // solve
    std::vector<Eigen::Matrix4d> path_pose = obsmap->calc_path(st_tf, ed_tf);
    if(path_pose.size() > 0)
    {
        // adding some global path
        int idx0 = get_next_idx(global_path.pos, path_pose.back().block(0,3,3,1));
        int idx1 = idx0 + config->OBS_DEADZONE/GLOBAL_PATH_STEP;
        if(idx1 > (int)global_path.pos.size()-1)
        {
            idx1 = global_path.pos.size()-1;
        }

        for(int p = idx0; p <= idx1; p++)
        {
            path_pose.push_back(global_path.pose[p]);
        }

        path_pose = path_resampling(path_pose, GLOBAL_PATH_STEP);
        std::vector<Eigen::Vector3d> path_pos;
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pos.push_back(path_pose[p].block(0,3,3,1));
        }
        path_pos = path_ccma(path_pos);

        path_pos = path_resampling(path_pos, LOCAL_PATH_STEP);
        path_pose = calc_path_tf(path_pos);

        /*
        path_pose = path_resampling(path_pose, LOCAL_PATH_STEP);
        std::vector<Eigen::Vector3d> path_pos;
        for(size_t p = 0; p < path_pose.size(); p++)
        {
            path_pos.push_back(path_pose[p].block(0,3,3,1));
        }
        */

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
    QString loc_state = slam->get_cur_loc_state();
    if(loc_state == "none" || loc_state == "fail")
    {
        logger->write_log("[AUTO] localization fail, auto-drive stop", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(slam->is_qa == true)
    {
        logger->write_log("[AUTO] qa working, auto-drive stop", "Red", true, false);
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

    if(ms.emo_state == 0)
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

// loops
void AUTOCONTROL::b_loop_pp()
{
    // set flag
    is_moving = true;

    // check global path
    PATH global_path = get_cur_global_path();
    if(global_path.pos.size() == 0)
    {
        // no global path
        mobile->move(0, 0, 0);
        is_moving = false;
        clear_path();

        Q_EMIT signal_move_failed("no global path");
        printf("[AUTO] global path init failed\n");
        return;
    }

    // update goal
    Eigen::Matrix4d goal_tf = global_path.ed_tf;
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);

    // check goal
    fsm_state = AUTO_FSM_FIRST_ALIGN;
    Eigen::Vector2d dtdr = dTdR(slam->get_cur_tf(), goal_tf);
    if(dtdr[0] < config->DRIVE_GOAL_D)
    {
        if(std::abs(dtdr[1]) < config->DRIVE_GOAL_TH*D2R)
        {
            // already goal
            mobile->move(0, 0, 0);
            is_moving = false;
            clear_path();

            Q_EMIT signal_move_failed("already goal");
            printf("[AUTO] already goal\n");
            return;
        }
        else
        {
            if(global_path.is_align)
            {
                // do final align
                fsm_state = AUTO_FSM_FINAL_ALIGN;
                printf("[AUTO] jump to FINAL_ALIGN\n");
            }
        }
    }

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
    double obs_wait_st_time = 0;

    printf("[AUTO] b_loop_pp start\n");
    while(b_flag)
    {
        // pause
        if(is_pause)
        {
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // check everything
        int is_good_everything = is_everything_fine();
        if(is_good_everything == DRIVING_FAILED)
        {
            mobile->move(0, 0, 0);
            is_moving = false;
            clear_path();

            Q_EMIT signal_move_failed("something wrong");
            printf("[AUTO] something wrong (failed)\n");
            return;
        }
        else if(is_good_everything == DRIVING_NOT_READY)
        {
            mobile->move(0, 0, 0);
            is_moving = false;
            clear_path();

            Q_EMIT signal_move_failed("something wrong (not ready)");
            printf("[AUTO] something wrong (not ready)\n");
            return;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        // calc local path
        if(fsm_state != AUTO_FSM_FINAL_ALIGN)
        {
            // check avoid path
            if(avoid_path.pos.size() > 0)
            {
                // clear avoid path
                double goal_d = calc_dist_2d(avoid_path.pos.back() - goal_pos);
                if(goal_d >= config->DRIVE_GOAL_D)
                {
                    int avoid_idx = get_nn_idx(avoid_path.pos, cur_pos);
                    if(avoid_idx > avoid_path.pos.size()*0.9)
                    {
                        // clear avoid path
                        avoid_path = PATH();
                        printf("[AUTO] avoid_path complete\n");
                        continue;
                    }
                }

                // set avoid path to local path
                if(local_path != avoid_path)
                {
                    local_path = avoid_path;

                    // update local path
                    mtx.lock();
                    cur_local_path = local_path;
                    last_local_goal = local_path.ed_tf.block(0,3,3,1);
                    mtx.unlock();

                    Q_EMIT signal_local_path_updated();
                }
            }
            else
            {
                // calc local path
                if(get_time() - local_path.t > 0.2)
                {
                    PATH _local_path = calc_local_path();
                    if(local_path != _local_path)
                    {
                        local_path = _local_path;

                        // update local path
                        mtx.lock();
                        cur_local_path = local_path;
                        last_local_goal = local_path.ed_tf.block(0,3,3,1);
                        mtx.unlock();

                        Q_EMIT signal_local_path_updated();
                    }
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

            Eigen::Matrix4d tgt_tf = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_xi = TF_to_se2(tgt_tf);
            Eigen::Vector3d tgt_pos = tgt_tf.block(0,3,3,1);

            mtx.lock();
            last_cur_pos = cur_pos;
            last_tgt_pos = tgt_pos;
            mtx.unlock();

            // calc heading error
            double err_th = deltaRad(tgt_xi[2], cur_xi[2]);

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // goal check
            if(std::abs(err_th) < config->DRIVE_GOAL_TH*D2R)
            {
                extend_dt = 0;
                pre_err_th = 0;
                mobile->move(0, 0, 0);

                fsm_state = AUTO_FSM_DRIVING;
                printf("[AUTO] FIRST_ALIGN -> DRIVING, err_th:%f\n", err_th*R2D);
                continue;
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = intp_tf(cur_tf, tgt_tf, 0.2, 10.0*D2R);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);

                obs_state = AUTO_OBS_CHECK;
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] FIRST_ALIGN -> OBS, err_th:%f\n", err_th*R2D);
                continue;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // find tgt            
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);

            // obs decel
            QString _obs_condition = "none";

            double obs_v = config->OBS_MAP_MIN_V;
            for(double vv = config->OBS_MAP_MIN_V; vv <= params.LIMIT_V+0.01; vv += 0.01)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, config->OBS_PREDICT_TIME, cur_tf);

                bool is_collision = false;
                for(size_t p = 0; p < traj.size(); p++)
                {
                    if(obsmap->is_tf_collision(traj[p], config->OBS_SAFE_MARGIN_X, config->OBS_SAFE_MARGIN_Y, true))
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
                int chk_idx = cur_idx + config->OBS_DEADZONE/LOCAL_PATH_STEP;
                if(chk_idx > (int)local_path.pos.size()-1)
                {
                    chk_idx = local_path.pos.size()-1;
                }

                std::vector<Eigen::Matrix4d> traj;
                for(int p = cur_idx; p <= chk_idx; p++)
                {
                    traj.push_back(local_path.pose[p]);
                }

                if(obsmap->is_path_collision(traj, 0, 0, 0, 10, true))
                {
                    obs_v = 0;
                }
            }

            // for mobile server
            mtx.lock();
            obs_condition = _obs_condition;
            mtx.unlock();

            // stop due to obstacle
            if(obs_v == 0)
            {
                mobile->move(0, 0, 0);

                obs_state = AUTO_OBS_CHECK;
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] DRIVING -> OBS\n");
                continue;
            }

            // calc ref_v            
            double goal_err_d = calc_dist_2d(goal_pos - cur_pos);
            double ref_v = local_path.ref_v[cur_idx];

            // calc heading error
            int tgt_idx = cur_idx + params.DRIVE_L/LOCAL_PATH_STEP;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            double dx = local_path.pos[tgt_idx][0] - local_path.pos[cur_idx][0];
            double dy = local_path.pos[tgt_idx][1] - local_path.pos[cur_idx][1];
            double err_th = deltaRad(std::atan2(dy,dx), cur_xi[2]);
            if(cur_idx == tgt_idx)
            {
                err_th = saturation(err_th, -10.0*D2R, 10.0*D2R);
            }

            // calc cross track error
            double cte = calc_cte(local_path.pose, cur_pos);

            // for plot
            mtx.lock();
            last_cur_pos = cur_pos;
            last_tgt_pos = local_path.pos[tgt_idx];
            mtx.unlock();

            // calc control input
            double v0 = cur_vel[0];
            double v = ref_v;            
            v = saturation(v, 0.0, obs_v);
            v = saturation(v, v0 - 2.0*params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_ACC*dt);
            v = saturation(v, -params.LIMIT_V, params.LIMIT_V);

            double th = (params.DRIVE_A * err_th)
                        + (params.DRIVE_B * (err_th-pre_err_th)/dt)
                        + std::atan2(params.DRIVE_K * cte, v + params.DRIVE_EPS);
            th = saturation(th, -45.0*D2R, 45.0*D2R);
            pre_err_th = err_th;

            double w = std::tan(th) / params.DRIVE_L;
            double w0 = cur_vel[2];
            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - params.DRIVE_T*std::abs(w/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(v/params.LIMIT_V);
            v *= scale_v;
            w *= scale_w;

            // deadzone w
            double d_w = 1.5*D2R;
            if(std::abs(w) < d_w)
            {
                w = 0;
            }
            else
            {
                w = sgn(w)*(std::abs(w)-d_w);
            }

            // goal check
            if(goal_err_d < config->DRIVE_GOAL_D || cur_idx == (int)local_path.pos.size()-1)
            {
                // check local sign
                Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
                Eigen::Vector3d _goal_pos = cur_tf_inv.block(0,0,3,3)*goal_pos + cur_tf_inv.block(0,3,3,1);
                v = saturation(config->DRIVE_GOAL_APPROACH_GAIN*_goal_pos[0], params.ED_V, v);

                extend_dt += dt;
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME || _goal_pos[0] < 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    if(global_path.is_align)
                    {
                        fsm_state = AUTO_FSM_FINAL_ALIGN;
                        printf("[AUTO] DRIVING -> FINAL_ALIGN, err_d:%f\n", goal_err_d);
                    }
                    else
                    {
                        // no final align finish
                        mobile->move(0, 0, 0);
                        is_moving = false;
                        clear_path();

                        Q_EMIT signal_move_succeed("early stopped, due to no align");

                        fsm_state = AUTO_FSM_COMPLETE;
                        printf("[AUTO] FINAL ALIGN COMPLETE(no align), err_th: %.3f\n", err_th*R2D);
                        return;
                    }

                    continue;
                }
            }

            // send control
            mobile->move(v, 0, w);
            //printf("v:%f, w:%f, err_th:%f, cte:%f, ref_v:%f, obs_v:%f, goal_v:%f\n", v, w*R2D, err_th*R2D, cte, ref_v, obs_v, goal_v);
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            // calc heading error
            double err_th = deltaRad(goal_xi[2], cur_xi[2]);

            // pivot control
            double kp = params.DRIVE_A;
            double kd = params.DRIVE_B;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // goal check
            if(std::abs(err_th) < config->DRIVE_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME)
                {
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);
                    is_moving = false;
                    clear_path();

                    Q_EMIT signal_move_succeed("very good");

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] FINAL ALIGN COMPLETE(good), err_th: %.3f\n", err_th*R2D);
                    return;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = intp_tf(cur_tf, goal_tf, 0.2, 10.0*D2R);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);
                is_moving = false;
                clear_path();

                Q_EMIT signal_move_succeed("early stopped, due to obstacle");

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] FINAL ALIGN COMPLETE(early stop), err_th: %.3f\n", err_th*R2D);
                return;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(obs_state == AUTO_OBS_CHECK)
            {
                // for mobile server
                mtx.lock();
                obs_condition = "near";
                mtx.unlock();

                if(config->OBS_AVOID == 0)
                {
                    // mode 0, just wait
                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    printf("[AUTO] avoid mode 0, OBS_WAIT\n");
                    continue;
                }
                else
                {
                    // mode 1, avoid
                    obs_state = AUTO_OBS_AVOID;
                    printf("[AUTO] avoid mode 1, OBS_AVOID\n");
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_AVOID)
            {
                // calc avoid path                
                avoid_path = calc_avoid_path();
                if(avoid_path.pos.size() > 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    printf("[AUTO] avoid path found, OBS_AVOID -> FIRST_ALIGN\n");
                    continue;
                }
                else
                {
                    obs_state = AUTO_OBS_RECOVERY;
                    printf("[AUTO] avoid path failed, OBS_AVOID -> OBS_RECOVERY\n");
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

                double margin_d = std::sqrt(config->OBS_SAFE_MARGIN_X*config->OBS_SAFE_MARGIN_X +
                                            config->OBS_SAFE_MARGIN_Y*config->OBS_SAFE_MARGIN_Y);
                if(max_d > config->ROBOT_RADIUS + margin_d)
                {
                    mobile->move(0, 0, 0);
                    obs_state = AUTO_OBS_AVOID;
                    printf("[AUTO] max_d: %f, OBS_RECOVERY -> OBS_AVOID\n", max_d);
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

                mobile->move(v, 0, w);
            }
            else if(obs_state == AUTO_OBS_WAIT)
            {
                if(get_time() - obs_wait_st_time > 1.0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    printf("[AUTO] OBS_WAIT -> FIRST_ALIGN\n");
                    continue;
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
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }

    mobile->move(0, 0, 0);
    is_moving = false;
    clear_path();

    Q_EMIT signal_move_succeed("stopped");
    printf("[AUTO] b_loop_pp stop\n");
}
