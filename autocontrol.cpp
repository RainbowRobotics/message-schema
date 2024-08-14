#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
    last_tgt_pos.setZero();    
    last_local_goal.setZero();
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
}

void AUTOCONTROL::init()
{
    params = load_preset(0);
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path;
    preset_path.sprintf("/preset/preset_%d.json", preset);
    preset_path = QCoreApplication::applicationDirPath() + preset_path;

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
    // control loop stop
    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }
    mobile->move(0, 0, 0);
    is_moving = false;
    is_pause = false;

    clear_path();
}

void AUTOCONTROL::move_pp(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // load preset
    params = load_preset(preset);

    // start control loop
    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_pp, this, goal_tf);
}

void AUTOCONTROL::move_hpp(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // load preset
    params = load_preset(preset);

    // start control loop
    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_hpp, this, goal_tf);
}

void AUTOCONTROL::move_tng(Eigen::Matrix4d goal_tf, int preset)
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // load preset
    params = load_preset(preset);

    // start control loop
    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_tng, this, goal_tf);
}

PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf, bool is_hpp)
{
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();

    // get st node id    
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    QString st_node_id = unimap->get_node_id_nn(cur_pos);
    if(st_node_id == "")
    {
        printf("[AUTO] st_node_id empty\n");
        return PATH();
    }

    // set ed node id
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    QString ed_node_id = unimap->get_node_id_nn(goal_pos);
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
        if(!check_point_on_segment(node_pos[0], node_pos[1], cur_pos))
        {
            node_pos.insert(node_pos.begin(), cur_pos);
            node_pose.insert(node_pose.begin(), cur_tf);
        }
    }

    // add goal pos
    if(goal_pos != node_pos.back())
    {
        node_pos.push_back(goal_pos);
        node_pose.push_back(goal_tf);
    }

    // divide and smooth metric path    
    std::vector<Eigen::Vector3d> path_pos = path_resampling(node_pos, GLOBAL_PATH_STEP);
    //path_pos = path_ccma(path_pos);

    // calc pose
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // set ref_v    
    std::vector<double> ref_v;
    calc_ref_v(path_pose, ref_v, params.ST_V, GLOBAL_PATH_STEP);
    ref_v.back() = params.ED_V;

    // for hpp
    if(is_hpp)
    {        
        // calc node ref_v, ref_w for hpp
        std::vector<double> ref_v0;
        calc_ref_v0(node_pose, ref_v0);

        for(size_t p = 0; p < node_pos.size(); p++)
        {
            int idx = get_nn_idx(path_pos, node_pos[p]);
            if(ref_v0[p] < ref_v[idx])
            {
                ref_v[idx] = ref_v0[p];
            }
        }
    }

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.t = get_time();
    res.nodes = node_path;
    res.pose = path_pose;
    res.pos = path_pos;    
    res.ref_v = ref_v;    
    res.goal_tf = goal_tf;
    return res;
}

std::vector<cv::Vec2i> AUTOCONTROL::path_finding(const cv::Mat& map, cv::Vec2i st, cv::Vec2i ed)
{
    // no need to path planning in this case
    if(st == ed)
    {
        std::vector<cv::Vec2i> res;
        res.push_back(ed);
        return res;
    }

    // obs_map CV_8U(obstacle is 255)
    const int w = map.cols;
    const int h = map.rows;

    // make open set and close set
    std::vector<ASTAR_NODE*> open_set;
    cv::Mat close_set(h, w, CV_8U, cv::Scalar(0));

    // set end node
    ASTAR_NODE *ed_node = new ASTAR_NODE();
    ed_node->pos = ed;

    // set start node
    ASTAR_NODE *st_node = new ASTAR_NODE();
    st_node->pos = st;
    st_node->g = 0;
    st_node->h = cv::norm(ed_node->pos-st_node->pos);
    st_node->f = st_node->g + st_node->h;
    open_set.push_back(st_node);

    // result storage
    while(open_set.size() > 0)
    {
        // get the current node
        int cur_node_idx = 0;
        ASTAR_NODE *cur_node = open_set.front();
        for(size_t p = 0; p < open_set.size(); p++)
        {
            if(open_set[p]->f < cur_node->f)
            {
                cur_node = open_set[p];
                cur_node_idx = p;
            }
        }

        // pop current off open list, add to closed list
        open_set.erase(open_set.begin()+cur_node_idx);
        close_set.ptr<uchar>(cur_node->pos[1])[cur_node->pos[0]] = 255;

        // found the goal
        if(ed_node->pos == cur_node->pos)
        {
            std::vector<cv::Vec2i> res;

            ASTAR_NODE* _cur_node = cur_node;
            while(_cur_node != NULL)
            {
                res.push_back(_cur_node->pos);
                _cur_node = _cur_node->parent;
            }

            std::reverse(res.begin(), res.end());
            return res;
        }

        // append child nodes
        std::vector<cv::Vec2i> around_pts;

        int search_range = 1;
        for(int my = -search_range; my <= search_range; my++)
        {
            for(int mx = -search_range; mx <=search_range; mx++)
            {
                if(mx == 0 && my == 0)
                {
                    continue;
                }

                int u = cur_node->pos[0] + mx;
                int v = cur_node->pos[1] + my;
                if(u < 0 || u >= w || v < 0 || v >= h)
                {
                    continue;
                }

                // obstacle
                if(map.ptr<uchar>(v)[u] == 255)
                {
                    continue;
                }

                // close set
                if(close_set.ptr<uchar>(v)[u] == 255)
                {
                    continue;
                }

                around_pts.push_back(cv::Vec2i(u,v));
            }
        }

        for(size_t p = 0; p < around_pts.size(); p++)
        {
            // calc heuristics
            int u = around_pts[p][0];
            int v = around_pts[p][1];
            cv::Vec2i child_pos(u, v);

            double child_g = cur_node->g + cv::norm(child_pos - cur_node->pos);
            double child_h = cv::norm(child_pos - ed_node->pos);
            double child_f = child_g + child_h;

            // check open set
            bool is_open_set = false;
            ASTAR_NODE* open_node = NULL;
            for(size_t p = 0; p < open_set.size(); p++)
            {
                if(open_set[p]->pos == child_pos)
                {
                    is_open_set = true;
                    open_node = open_set[p];
                    break;
                }
            }

            // change better parent
            if(is_open_set)
            {
                if(child_g < open_node->g)
                {
                    open_node->parent = cur_node;
                    open_node->g = child_g;
                    open_node->h = child_h;
                    open_node->f = child_f;
                    continue;
                }
                else
                {
                    continue;
                }
            }

            // add new child to open set
            ASTAR_NODE *node = new ASTAR_NODE();
            node->parent = cur_node;
            node->pos = child_pos;
            node->g = child_g;
            node->h = child_h;
            node->f = child_f;
            open_set.push_back(node);
        }
    }

    // path finding failed
    return std::vector<cv::Vec2i>();
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
            for(size_t p = 0; p < open_set.size(); p++)
            {
                if(open_set[p]->node->id == node->id)
                {
                    is_open_set = true;
                    open_node = open_set[p];
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

std::vector<Eigen::Vector3d> AUTOCONTROL::sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step)
{
    std::vector<Eigen::Vector3d> res = src;
    res = path_resampling(res, large_step);
    res = path_ccma(res);
    res = path_resampling(res, small_step);
    //res = path_ccma(res);
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

void AUTOCONTROL::calc_ref_v0(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v)
{
    ref_v.resize(src.size(), params.LIMIT_V);
    if(src.size() == 1)
    {
        return;
    }

    for(size_t p = 0; p < src.size()-1; p++)
    {
        Eigen::Vector3d xi0 = TF_to_se2(src[p]);
        Eigen::Vector3d xi1 = TF_to_se2(src[p+1]);

        double err_d = calc_dist_2d(xi1 - xi0);
        double err_th = std::abs(deltaRad(xi1[2], xi0[2]));

        double t_v = err_d/params.LIMIT_V;
        double t_w = err_th/(params.LIMIT_W*D2R);
        double t = std::max<double>(t_v, t_w);
        double v = saturation(err_d/t, 0, params.LIMIT_V);

        ref_v[p] = v;
    }
}

void AUTOCONTROL::calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step)
{
    if(src.size() == 1)
    {
        std::vector<double> _ref_v(src.size(), st_v);
        ref_v = _ref_v;
        return;
    }

    const int ld = std::ceil(params.DRIVE_L/step);

    std::vector<double> _ref_v(src.size(), params.LIMIT_V);
    for(size_t p = 0; p < src.size()-1; p++)
    {
        int cur_idx = p;
        int tgt_idx = p+ld;
        if(tgt_idx > (int)src.size()-1)
        {
            tgt_idx = src.size()-1;
        }

        Eigen::Vector3d xi0 = TF_to_se2(src[cur_idx]);
        Eigen::Vector3d xi1 = TF_to_se2(src[tgt_idx]);

        double err_d = calc_dist_2d(xi1 - xi0);
        double err_th = deltaRad(xi1[2], xi0[2]) * params.DRIVE_H;

        double t_v = err_d/params.LIMIT_V;
        double t_w = std::abs(err_th)/(params.LIMIT_W*D2R);
        double t = std::max<double>(t_v, t_w);
        double v = err_d/t;
        _ref_v[p] = v;
    }

    _ref_v[_ref_v.size()-1] = _ref_v[_ref_v.size()-2];
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

    res.erase(res.begin());
    return res;
}

bool AUTOCONTROL::is_state_valid(const ompl::base::State *state) const
{
    const auto *cur_state = state->as<ompl::base::DubinsStateSpace::StateType>();
    const double x = cur_state->getX();
    const double y = cur_state->getY();
    const double th = cur_state->getYaw();

    // check collision
    const double margin_x = config->OBS_OMPL_MARGIN_X;
    const double margin_y = config->OBS_OMPL_MARGIN_Y;
    Eigen::Matrix4d robot_tf = se2_to_TF(Eigen::Vector3d(x,y,th));    
    if(obsmap->is_tf_collision_with_area(robot_tf, avoid_area, margin_x, margin_y))
    {
        // is collision        
        return false;
    }

    // no collision
    return true;
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

PATH AUTOCONTROL::calc_local_path()
{
    // get params
    PATH global_path = get_cur_global_path();

    // get cur params
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // local path storage
    std::vector<Eigen::Vector3d> path_pos;
    std::vector<Eigen::Matrix4d> path_pose;
    if(config->OBS_AVOID == 2) // mode0: no avoid, mode1: ompl, mode2: eband + ompl
    {
        // params
        const double margin_x = config->OBS_EB_MARGIN_X;
        const double margin_y = config->OBS_EB_MARGIN_Y;
        const double range = 2.0;

        // check last local path
        PATH last_local_path = get_cur_local_path();
        if(last_local_path.pos.size() > 0)
        {
            bool is_goal = global_path.goal_tf.block(0,3,3,1).isApprox(last_local_path.pos.back());
            int st_idx = get_nn_idx(last_local_path.pos, cur_pos);
            if(!obsmap->is_path_collision(last_local_path.pose, st_idx, 20) && (st_idx < (int)last_local_path.pos.size()/2 || is_goal))
            {
                return last_local_path;
            }
        }

        // get global path segment
        std::vector<Eigen::Vector3d> _path_pos;
        _path_pos.push_back(cur_pos);

        int st_idx = std::min<int>(cur_idx+1, (int)global_path.pos.size()-1);
        for(int p = st_idx; p < (int)global_path.pos.size(); p++)
        {
            if(p == 0 || p == (int)global_path.pos.size()-1 || p%2 == 0)
            {
                double d = calc_dist_2d(global_path.pos[p] - global_path.pos[cur_idx]);
                if(d <= config->OBS_LOCAL_GOAL_D)
                {
                    _path_pos.push_back(global_path.pos[p]);
                }
                else
                {
                    break;
                }
            }
        }

        std::vector<Eigen::Matrix4d> _path_pose = calc_path_tf(_path_pos);

        // use eband
        double global_goal_d = calc_dist_2d(global_path.pos.back()-cur_pos);
        if(global_goal_d > 1.5 && config->OBS_AVOID)
        {
            // left modified path
            int sum_cnt_l = 0;
            std::vector<Eigen::Vector3d> modified_path_pos_l;
            for(size_t p = 0; p < _path_pose.size(); p++)
            {
                if(p == 0)
                {
                    modified_path_pos_l.push_back(_path_pos[p]);
                    continue;
                }

                double dy = std::abs((_path_pose[p-1].inverse()*_path_pose[p])(1,3));
                if(!obsmap->is_tf_collision(_path_pose[p], margin_x, margin_y) && dy < 0.3)
                {
                    modified_path_pos_l.push_back(_path_pos[p]);
                    continue;
                }

                Eigen::Vector3d modified_pos = _path_pos[p];

                int min_cnt0 = 99999999;
                int min_cnt1 = 99999999;
                for(double dy = -0.3; dy < 1.0; dy += 0.05)
                {
                    Eigen::Matrix4d offset_tf = Eigen::Matrix4d::Identity();
                    offset_tf(1,3) = dy;

                    Eigen::Matrix4d modified_tf = _path_pose[p]*offset_tf;

                    int cnt0 = 0;
                    int cnt1 = 0;
                    bool is_ok = obsmap->get_tf_collision_cnt(modified_tf, margin_x, margin_y, range, cnt0, cnt1);
                    if(is_ok && cnt0 < min_cnt0)
                    {
                        min_cnt0 = cnt0;
                        min_cnt1 = cnt1;
                        modified_pos = modified_tf.block(0,3,3,1);
                    }
                }

                modified_path_pos_l.push_back(modified_pos);
                sum_cnt_l += (min_cnt0 + min_cnt1);
            }

            // right modified path
            int sum_cnt_r = 0;
            std::vector<Eigen::Vector3d> modified_path_pos_r;
            for(size_t p = 0; p < _path_pose.size(); p++)
            {
                if(p == 0)
                {
                    modified_path_pos_r.push_back(_path_pos[p]);
                    continue;
                }

                double dy = std::abs((_path_pose[p-1].inverse()*_path_pose[p])(1,3));
                if(!obsmap->is_tf_collision(_path_pose[p], margin_x, margin_y) && dy < 0.3)
                {
                    modified_path_pos_r.push_back(_path_pos[p]);
                    continue;
                }

                Eigen::Vector3d modified_pos = _path_pos[p];

                int min_cnt0 = 99999999;
                int min_cnt1 = 99999999;
                for(double dy = -0.3; dy < 1.0; dy += 0.05)
                {
                    Eigen::Matrix4d offset_tf = Eigen::Matrix4d::Identity();
                    offset_tf(1,3) = -dy;

                    Eigen::Matrix4d modified_tf = _path_pose[p]*offset_tf;

                    int cnt0 = 0;
                    int cnt1 = 0;
                    bool is_ok = obsmap->get_tf_collision_cnt(modified_tf, margin_x, margin_y, range, cnt0, cnt1);
                    if(is_ok && cnt0 < min_cnt0)
                    {
                        min_cnt0 = cnt0;
                        min_cnt1 = cnt1;
                        modified_pos = modified_tf.block(0,3,3,1);
                    }
                }

                modified_path_pos_r.push_back(modified_pos);
                sum_cnt_r += (min_cnt0 + min_cnt1);
            }

            // calc length
            double length_l = calc_length(modified_path_pos_l);
            double length_r = calc_length(modified_path_pos_r);

            // cost
            double cost_l = sum_cnt_l + length_l;
            double cost_r = sum_cnt_r + length_r;

            // select one
            std::vector<Eigen::Vector3d> modified_path_pos;
            if(cost_l < cost_r)
            {
                modified_path_pos = modified_path_pos_l;
            }
            else
            {
                modified_path_pos = modified_path_pos_r;
            }

            if(modified_path_pos.size() >= 3)
            {
                // eb init
                std::vector<BUBBLE> eb;
                for(size_t p = 0; p < _path_pos.size(); p++)
                {
                    BUBBLE b;
                    b.ref = _path_pos[p];
                    b.pos = modified_path_pos[p];
                    eb.push_back(b);
                }

                // eband method
                if(eb.size() >= 3)
                {
                    // eb iteration
                    double k_i = 20.0;
                    double k_e = 10.0;
                    double c = 1.0;
                    double dt = 0.05;
                    double vel_limit = 0.35;
                    int max_iter = 5;
                    for(int iter = 0; iter < max_iter; iter++)
                    {
                        // band deformation
                        std::vector<BUBBLE> _eb;
                        _eb.push_back(eb.front());

                        for(size_t p = 1; p < eb.size()-1; p++)
                        {
                            BUBBLE b = eb[p];

                            // calc forces
                            Eigen::Vector3d f_i = (eb[p-1].pos - b.pos) + (eb[p+1].pos - b.pos);
                            Eigen::Vector3d f_e = obsmap->get_obs_force(b.pos, b.r);
                            Eigen::Vector3d f = (k_i * f_i) + (k_e * f_e) - (c * b.vel);

                            // Runge-Kutta 4th (f = ma -> a = f/m, but m is 1 so f = a)
                            Eigen::Vector3d k1_v = f * dt;
                            Eigen::Vector3d k1_p = b.vel * dt;

                            Eigen::Vector3d k2_v = f * dt;
                            Eigen::Vector3d k2_p = (b.vel + k1_v * 0.5) * dt;

                            Eigen::Vector3d k3_v = f * dt;
                            Eigen::Vector3d k3_p = (b.vel + k2_v * 0.5) * dt;

                            Eigen::Vector3d k4_v = f * dt;
                            Eigen::Vector3d k4_p = (b.vel + k3_v) * dt;

                            // velocity update
                            b.vel = b.vel + (k1_v + 2 * k2_v + 2 * k3_v + k4_v) / 6.0;

                            // position update
                            b.pos = b.pos + (k1_p + 2 * k2_p + 2 * k3_p + k4_p) / 6.0;

                            // velocity saturation
                            double v_norm = b.vel.norm();
                            v_norm = saturation(v_norm, -vel_limit, vel_limit);
                            b.vel = v_norm*b.vel.normalized();

                            // add
                            _eb.push_back(b);
                        }

                        // set last bubble and update
                        _eb.push_back(eb.back());
                        eb = _eb;
                    }

                    // update eb to path_pos
                    std::vector<Eigen::Vector3d> _path_pos2;
                    for(size_t p = 0; p < eb.size(); p++)
                    {
                        _path_pos2.push_back(eb[p].pos);
                    }
                    _path_pos = _path_pos2;
                }
            }
        }

        // path resampling
        path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP*2, LOCAL_PATH_STEP);
        path_pose = calc_path_tf(path_pos);

        // final path check
        if(obsmap->is_path_collision(path_pose, 0, 20))
        {
            printf("[AUTO] eband path failed\n");
            return PATH();
        }

        // calc ref_v
        double ref_v0 = params.ST_V;
        if(last_local_path.pos.size() > 0)
        {
            Eigen::Matrix4d _cur_tf = slam->get_cur_tf();
            Eigen::Vector3d _cur_pos = _cur_tf.block(0,3,3,1);
            int _cur_idx = get_nn_idx(last_local_path.pos, _cur_pos);
            ref_v0 = last_local_path.ref_v[_cur_idx];
        }

        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, ref_v0, LOCAL_PATH_STEP);
        bool is_goal = global_path.goal_tf.block(0,3,3,1).isApprox(path_pos.back());
        if(is_goal)
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
        res.goal_tf = path_pose.back();
        return res;
    }
    else
    {
        // get global path segment
        std::vector<Eigen::Vector3d> _path_pos;
        int st_idx = std::min<int>(cur_idx, (int)global_path.pos.size()-2);
        for(int p = st_idx; p < (int)global_path.pos.size(); p++)
        {
            double d = calc_dist_2d(global_path.pos[p] - global_path.pos[cur_idx]);
            if(d <= config->OBS_LOCAL_GOAL_D)
            {
                _path_pos.push_back(global_path.pos[p]);
            }
            else
            {
                break;
            }
        }
        std::vector<Eigen::Matrix4d> _path_pose = calc_path_tf(_path_pos);

        // no use eband
        path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP, LOCAL_PATH_STEP);
        path_pose = calc_path_tf(path_pos);

        // calc ref_v
        double ref_v0 = params.ST_V;
        PATH last_local_path = get_cur_local_path();
        if(last_local_path.pos.size() > 0)
        {
            Eigen::Matrix4d _cur_tf = slam->get_cur_tf();
            Eigen::Vector3d _cur_pos = _cur_tf.block(0,3,3,1);
            int _cur_idx = get_nn_idx(last_local_path.pos, _cur_pos);
            ref_v0 = last_local_path.ref_v[_cur_idx];
        }

        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_v, ref_v0, LOCAL_PATH_STEP);
        bool is_goal = global_path.goal_tf.block(0,3,3,1).isApprox(path_pos.back());
        if(is_goal)
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
        res.goal_tf = path_pose.back();
        return res;
    }
}

PATH AUTOCONTROL::calc_avoid_path()
{
    const double margin_x = config->OBS_OMPL_MARGIN_X;
    const double margin_y = config->OBS_OMPL_MARGIN_Y;

    // get global path
    PATH global_path = get_cur_global_path();

    // get cur params
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

    // get local goal
    int local_goal_idx = std::min<int>(cur_idx + config->OBS_LOCAL_GOAL_D/GLOBAL_PATH_STEP, (int)global_path.pose.size()-1);
    Eigen::Matrix4d local_goal_tf = global_path.pose[local_goal_idx];
    Eigen::Vector3d local_goal_xi = TF_to_se2(local_goal_tf);

    // set state space
    auto stateSpace(std::make_shared<ompl::base::DubinsStateSpace>(0.001, true));

    ompl::base::RealVectorBounds stateBounds(2);
    stateBounds.setLow(0, cur_pos[0] - config->OBS_MAP_RANGE); stateBounds.setHigh(0, cur_pos[0] + config->OBS_MAP_RANGE);
    stateBounds.setLow(1, cur_pos[1] - config->OBS_MAP_RANGE); stateBounds.setHigh(1, cur_pos[1] + config->OBS_MAP_RANGE);
    stateSpace->setBounds(stateBounds);

    ompl::geometric::SimpleSetup ss(stateSpace);

    // set validator
    ss.setStateValidityChecker(std::bind(&AUTOCONTROL::is_state_valid, this, std::placeholders::_1));

    // start
    Eigen::Vector3d st_xi = cur_xi;
    Eigen::Matrix4d st_tf = se2_to_TF(st_xi);
    if(obsmap->is_tf_collision(st_tf, margin_x, margin_y))
    {
        bool is_found = false;
        double min_d = 99999999;
        Eigen::Vector3d min_xi = st_xi;
        for(double dx = -0.1; dx <= 0.1 + 0.001; dx += 0.05)
        {
            for(double dy = -0.1; dy <= 0.1 + 0.001; dy += 0.05)
            {
                Eigen::Vector3d xi;
                xi[0] = st_xi[0] + dx;
                xi[1] = st_xi[1] + dy;
                xi[2] = st_xi[2];

                double d = calc_dist_2d(xi - st_xi);

                Eigen::Matrix4d tf = se2_to_TF(xi);
                if(!obsmap->is_tf_collision(tf, margin_x, margin_y) && d < min_d)
                {
                    min_d = d;
                    min_xi = xi;
                    is_found = true;
                }
            }
        }

        if(is_found)
        {
            st_xi = min_xi;
            st_tf = se2_to_TF(st_xi);
            printf("[AUTO] ompl, st_xi, shifted\n");
        }
    }

    // end
    Eigen::Vector3d ed_xi = local_goal_xi;
    Eigen::Matrix4d ed_tf = se2_to_TF(ed_xi);
    if(obsmap->is_tf_collision(ed_tf, margin_x, margin_y))
    {
        bool is_found = false;
        double min_d = 99999999;
        Eigen::Vector3d min_xi = ed_xi;
        for(double dx = -1.0; dx <= 1.0 + 0.001; dx += 0.1)
        {
            for(double dy = -1.0; dy <= 1.0 + 0.001; dy += 0.1)
            {
                Eigen::Vector3d xi;
                xi[0] = ed_xi[0] + dx;
                xi[1] = ed_xi[1] + dy;
                xi[2] = ed_xi[2];

                double d = calc_dist_2d(xi-ed_xi);

                Eigen::Matrix4d tf = se2_to_TF(xi);
                if(!obsmap->is_tf_collision(tf, margin_x, margin_y) && d < min_d)
                {
                    min_d = d;
                    min_xi = xi;
                    is_found = true;
                }
            }
        }

        if(is_found)
        {
            ed_xi = min_xi;
            ed_tf = se2_to_TF(ed_xi);

            local_goal_tf = ed_tf;
            local_goal_xi = ed_xi;
            printf("[AUTO] ompl, ed_xi, shifted\n");
        }
        else
        {
            printf("[AUTO] ompl, ed_xi, obs stucked\n");
            return PATH();
        }
    }

    // calc avoid area
    avoid_area = obsmap->calc_avoid_area(global_path.pose, st_tf, ed_tf, margin_x, margin_y);

    // set start pose
    ompl::base::ScopedState<ompl::base::DubinsStateSpace> start(ss.getStateSpace());
    start->setX(st_xi[0]);
    start->setY(st_xi[1]);
    start->setYaw(st_xi[2]);

    // set end pose
    ompl::base::ScopedState<ompl::base::DubinsStateSpace> end(ss.getStateSpace());
    end->setX(ed_xi[0]);
    end->setY(ed_xi[1]);
    end->setYaw(ed_xi[2]);

    // set start and goal
    ss.setStartAndGoalStates(start, end);

    // set objective
    auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation());
    ss.getProblemDefinition()->setOptimizationObjective(objective);

    // set planner
    //auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    auto planner = std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // solve
    ompl::base::PlannerStatus solved = ss.solve(1.5);
    if(solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
        // solution interpolation
        ss.simplifySolution();

        auto path = ss.getSolutionPath();
        ss.getPathSimplifier()->simplifyMax(path);
        if(path.getStateCount() >= 2)
        {
            double sum_d = 0;
            for(size_t p = 0; p < path.getStateCount()-1; p++)
            {
                const ompl::base::DubinsStateSpace::StateType *state0 = path.getState(p)->as<ompl::base::DubinsStateSpace::StateType>();
                const ompl::base::DubinsStateSpace::StateType *state1 = path.getState(p+1)->as<ompl::base::DubinsStateSpace::StateType>();

                double dx = state1->getX() - state0->getX();
                double dy = state1->getY() - state0->getY();
                double d = std::sqrt(dx*dx + dy*dy);
                sum_d += d;
            }
            path.interpolate(sum_d/LOCAL_PATH_STEP);
        }
        ss.getPathSimplifier()->smoothBSpline(path);

        // set pos
        std::vector<Eigen::Vector3d> _path_pos;
        for(size_t p = 0; p < path.getStateCount(); p++)
        {
            const ompl::base::DubinsStateSpace::StateType *state = path.getState(p)->as<ompl::base::DubinsStateSpace::StateType>();
            _path_pos.push_back(Eigen::Vector3d(state->getX(), state->getY(), 0));
        }

        // resampling
        std::vector<Eigen::Vector3d> path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP*2, LOCAL_PATH_STEP);
        std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

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
        res.goal_tf = local_goal_tf;

        printf("[AUTO] ompl lpp, exact solution found\n");
        return res;
    }
    else
    {
        printf("[AUTO] ompl lpp, solution failed\n");
        return PATH();
    }
}

// check condition
bool AUTOCONTROL::is_everything_fine()
{
    QString loc_state = slam->get_cur_loc_state();
    if(loc_state == "none" || loc_state == "fail")
    {
        return false;
    }

    return true;
}

// loops
void AUTOCONTROL::b_loop_pp(Eigen::Matrix4d goal_tf)
{
    // set flag
    is_moving = true;

    // global goal
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    Eigen::Vector3d goal_xi = TF_to_se2(goal_tf);

    // path storage
    PATH global_path;
    PATH local_path;
    PATH avoid_path;

    // check goal
    Eigen::Vector2d dtdr = dTdR(slam->get_cur_tf(), goal_tf);
    if(dtdr[0] < config->DRIVE_GOAL_D)
    {
        if(std::abs(dtdr[1]) < config->DRIVE_GOAL_TH*D2R)
        {
            // already goal
            mobile->move(0, 0, 0);
            is_moving = false;

            Q_EMIT signal_move_failed("already goal");
            printf("[AUTO] already goal\n");
            return;
        }
        else
        {
            // do final align
            fsm_state = AUTO_FSM_FINAL_ALIGN;
            printf("[AUTO] AUTO_FSM_FINAL_ALIGN\n");
        }
    }
    else
    {
        // do first align
        fsm_state = AUTO_FSM_FIRST_ALIGN;        
        printf("[AUTO] AUTO_FSM_FIRST_ALIGN\n");

        // calc global path
        global_path = calc_global_path(goal_tf);
        if(global_path.pos.size() == 0)
        {
            // no global path
            mobile->move(0, 0, 0);
            is_moving = false;

            Q_EMIT signal_move_failed("no global path");
            printf("[AUTO] global path init failed\n");
            return;
        }
        else
        {
            // update global path
            mtx.lock();
            cur_global_path = global_path;
            mtx.unlock();

            Q_EMIT signal_global_path_updated();
            printf("[AUTO] global path found\n");
        }

        // calc local path
        local_path = calc_local_path();
        if(local_path.pos.size() == 0)
        {
            avoid_path = calc_avoid_path();
            if(avoid_path.pos.size() > 0)
            {
                local_path = avoid_path;
            }
            else
            {
                mobile->move(0, 0, 0);
                is_moving = false;

                Q_EMIT signal_move_failed("no local path");
                printf("[AUTO] local path init failed\n");
                return;
            }
        }

        // update local path
        mtx.lock();
        cur_local_path = local_path;
        last_local_goal = local_path.goal_tf.block(0,3,3,1);
        mtx.unlock();

        Q_EMIT signal_local_path_updated();
    }

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();    

    // control params
    double extend_dt = 0;
    double pre_err_th = 0;

    // for obs
    int obs_state = AUTO_OBS_CHECK;
    double obs_recovery_st_time = 0;
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
        if(is_everything_fine() == false)
        {
            mobile->move(0, 0, 0);
            is_moving = false;
            Q_EMIT signal_move_failed("something wrong");

            printf("[AUTO] something wrong\n");
            break;
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
                // check avoid path goal
                if(calc_dist_2d(avoid_path.pos.back() - cur_pos) < config->DRIVE_GOAL_D)
                {
                    // clear avoid path
                    avoid_path = PATH();

                    printf("[AUTO] avoid_path complete\n");
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                else
                {
                    // check
                    if(local_path != avoid_path)
                    {
                        // set avoid path to local path
                        local_path = avoid_path;

                        // update local path
                        mtx.lock();
                        cur_local_path = local_path;
                        last_local_goal = local_path.goal_tf.block(0,3,3,1);
                        mtx.unlock();

                        Q_EMIT signal_local_path_updated();
                    }
                }
            }
            else
            {
                // calc local path using eband
                if(get_time() - local_path.t > 0.2)
                {
                    PATH _local_path = calc_local_path();
                    if(_local_path.pos.size() > 0)
                    {
                        if(local_path != _local_path)
                        {
                            local_path = _local_path;

                            // update local path
                            mtx.lock();
                            cur_local_path = local_path;
                            last_local_goal = local_path.goal_tf.block(0,3,3,1);
                            mtx.unlock();

                            Q_EMIT signal_local_path_updated();
                        }
                    }
                    else
                    {
                        if(fsm_state != AUTO_FSM_OBS)
                        {
                            // no local path -> means local path collision
                            mobile->move(0, 0, 0);

                            obs_state = AUTO_OBS_CHECK;
                            fsm_state = AUTO_FSM_OBS;
                            printf("[AUTO] calc local path failed, OBS_CHECK\n");
                        }
                    }
                }
            }
        }

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + 10;
            if(tgt_idx > (int)local_path.pos.size()-1)
            {
                tgt_idx = local_path.pos.size()-1;
            }

            Eigen::Matrix4d tgt_tf = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_xi = TF_to_se2(tgt_tf);
            Eigen::Vector3d tgt_pos = tgt_tf.block(0,3,3,1);            

            mtx.lock();
            last_tgt_pos = tgt_pos;
            mtx.unlock();

            // calc heading error
            double err_th = deltaRad(tgt_xi[2], cur_xi[2]);

            // pivot control
            double kp = 1.0;
            double kd = 0.05;
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
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, 0.2, config->OBS_PREDICT_TIME, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);

                obs_state = AUTO_OBS_CHECK;
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] FIRST_ALIGN -> OBS_CHECK, err_th:%f\n", err_th*R2D);
                continue;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // find tgt            
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx;

            Eigen::Matrix4d tgt_tf = local_path.pose[tgt_idx];            
            Eigen::Vector3d tgt_pos = tgt_tf.block(0,3,3,1);

            // obs decel
            QString _obs_condition = "none";
            double obs_v = 0.1;
            for(double vv = 0.1; vv <= params.LIMIT_V+0.01; vv += 0.1)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, config->OBS_PREDICT_TIME, cur_tf);

                bool is_collision = false;
                for(size_t p = 0; p < traj.size(); p++)
                {
                    if(obsmap->is_tf_collision_dynamic(traj[p], 0.3, 0.3))
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
                    if(p == cur_idx || p == chk_idx || p%10 == 0)
                    {
                        traj.push_back(local_path.pose[p]);
                    }
                }

                std::vector<Eigen::Matrix4d> traj2 = calc_trajectory(cur_vel, 0.2, 1.0, cur_tf);
                traj.insert(traj.end(), traj2.begin(), traj2.end());

                if(obsmap->is_path_collision(traj, 0, 1, 0.05, 0.05))
                {
                    printf("[AUTO] obs deadzone reached\n");
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
            double goal_v = saturation(0.75 * goal_err_d, params.ED_V, params.LIMIT_V);
            double ref_v = local_path.ref_v[cur_idx];

            // calc heading error
            int ld_idx = cur_idx + std::ceil(params.DRIVE_L/LOCAL_PATH_STEP);
            if(ld_idx > (int)local_path.pos.size()-1)
            {
                ld_idx = local_path.pos.size()-1;
            }

            double dx = local_path.pos[ld_idx][0] - local_path.pos[cur_idx][0];
            double dy = local_path.pos[ld_idx][1] - local_path.pos[cur_idx][1];
            double err_th = deltaRad(std::atan2(dy,dx), cur_xi[2]);
            if(cur_idx == ld_idx)
            {
                err_th = 0;
            }

            // calc cross track error
            double cte = calc_cte(local_path.pose, cur_pos);

            // for plot
            mtx.lock();
            last_cur_pos = cur_pos;
            last_tgt_pos = tgt_pos;
            mtx.unlock();

            // calc control input
            double v0 = cur_vel[0];
            double v = ref_v;            
            v = saturation(v, v0 - params.LIMIT_V_DCC*dt, v0 + params.LIMIT_V_ACC*dt);
            v = saturation(v, 0, goal_v);
            v = saturation(v, 0, obs_v);

            double th = (params.DRIVE_A * err_th) + (params.DRIVE_B * (err_th-pre_err_th)/dt) + std::atan2(params.DRIVE_K * cte, v + params.DRIVE_EPS);
            th = saturation(th, -45.0*D2R, 45.0*D2R);
            pre_err_th = err_th;

            double w0 = cur_vel[2];
            double w = (v * std::tan(th)) / params.DRIVE_L;
            w = saturation(w, w0 - (params.LIMIT_W_ACC*D2R)*dt, w0 + (params.LIMIT_W_ACC*D2R)*dt);
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - params.DRIVE_T*std::abs(w/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(v/params.LIMIT_V);
            v *= scale_v;
            w *= scale_w;            

            // goal check
            if(goal_err_d < config->DRIVE_GOAL_D || cur_idx == (int)local_path.pos.size()-1)
            {
                // check local sign
                Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
                Eigen::Vector3d _goal_pos = cur_tf_inv.block(0,0,3,3)*goal_pos + cur_tf_inv.block(0,3,3,1);

                extend_dt += dt;                
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME || _goal_pos[0] < 0)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    fsm_state = AUTO_FSM_FINAL_ALIGN;
                    printf("[AUTO] DRIVING -> FINAL_ALIGN, err_d:%f\n", goal_err_d);
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
            double kp = 1.0;
            double kd = 0.05;
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
                    Q_EMIT signal_move_succeed("very good");

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] FINAL ALIGN COMPLETE(good), err_th: %.3f\n", err_th*R2D);
                    return;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, 0.2, config->OBS_PREDICT_TIME, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);

                is_moving = false;
                Q_EMIT signal_move_succeed("early stopped");

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] FINAL ALIGN COMPLETE(OBS), err_th: %.3f\n", err_th*R2D);
                return;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(obs_state == AUTO_OBS_CHECK)
            {
                mtx.lock();
                obs_condition = "near";
                mtx.unlock();

                if(config->OBS_AVOID == 0)
                {
                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    printf("[AUTO] no avoid allowed, OBS_WAIT\n");
                    continue;
                }
                else
                {
                    // check stuck
                    const double margin_x = config->OBS_OMPL_MARGIN_X;
                    const double margin_y = config->OBS_OMPL_MARGIN_Y;
                    if(obsmap->is_tf_collision(cur_tf, margin_x, margin_y))
                    {
                        obs_recovery_st_time = get_time();
                        obs_state = AUTO_OBS_RECOVERY;
                        printf("[AUTO] is collision, try recovery\n");
                        continue;
                    }
                    else
                    {
                        obs_state = AUTO_OBS_OMPL;
                        printf("[AUTO] no collision, try OMPL\n");
                        continue;
                    }
                }
            }
            else if(obs_state == AUTO_OBS_RECOVERY)
            {
                // reverse moving
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(-params.ST_V, 0, 0), 0.2, 1.5, cur_tf);
                if(obsmap->is_path_collision(traj) || get_time() - obs_recovery_st_time > 1.5)
                {
                    mobile->move(0, 0, 0);

                    if(config->OBS_AVOID == 1)
                    {
                        obs_state = AUTO_OBS_OMPL;
                        printf("[AUTO] stop recovery, try OMPL\n");
                        continue;
                    }
                    else
                    {
                        obs_wait_st_time = get_time();
                        obs_state = AUTO_OBS_WAIT;
                        printf("[AUTO] stop recovery, OBS_WAIT\n");
                        continue;
                    }
                }

                mobile->move(-params.ST_V, 0, 0);
            }
            else if(obs_state == AUTO_OBS_OMPL)
            {
                // calc avoid path                
                avoid_path = calc_avoid_path();
                if(avoid_path.pos.size() > 0)
                {
                    pre_err_th = 0;
                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    printf("[AUTO] avoid path found, OMPL -> FIRST_ALIGN\n");
                    continue;
                }
                else
                {
                    obs_wait_st_time = get_time();
                    obs_state = AUTO_OBS_WAIT;
                    printf("[AUTO] OMPL failed, OBS_WAIT\n");
                    continue;
                }
            }
            else if(obs_state == AUTO_OBS_WAIT)
            {
                if(get_time() - obs_wait_st_time > 1.0)
                {
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
    Q_EMIT signal_move_succeed("stopped");

    printf("[AUTO] b_loop_pp stop\n");
}

void AUTOCONTROL::b_loop_hpp(Eigen::Matrix4d goal_tf)
{
    // set flag
    is_moving = true;

    // calc global path
    PATH global_path = calc_global_path(goal_tf, true);
    if(global_path.nodes.size() == 0)
    {
        // no global path
        mobile->move(0, 0, 0);
        is_moving = false;

        Q_EMIT signal_move_failed("no global path");
        printf("[AUTO] global path init failed\n");
        return;
    }
    else
    {
        // update global path
        mtx.lock();
        cur_global_path = global_path;
        mtx.unlock();

        Q_EMIT signal_global_path_updated();
        printf("[AUTO] global path found\n");
    }

    // get metric node path (for holonomic)
    std::vector<Eigen::Vector3d> path_pos;
    std::vector<double> path_th;
    for(size_t p = 0; p < global_path.nodes.size(); p++)
    {
        NODE* node = unimap->get_node_by_id(global_path.nodes[p]);
        if(node == NULL)
        {
            // no global path
            mobile->move(0, 0, 0);
            is_moving = false;

            Q_EMIT signal_move_failed("topology wrong\n");
            printf("[AUTO] global path init failed\n");
            return;
        }

        Eigen::Vector3d xi = TF_to_se2(node->tf);
        path_pos.push_back(node->tf.block(0,3,3,1));
        path_th.push_back(xi[2]);
    }

    // set init state
    fsm_state = AUTO_FSM_DRIVING;
    printf("[AUTO] AUTO_FSM_DRIVING\n");

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;
    double pre_err_th = 0;

    // for obs
    double obs_wait_st_time = 0;

    // set first tgt idx
    int tgt_idx = 0;

    printf("[AUTO] b_loop_hpp start\n");
    while(b_flag)
    {
        // pause
        if(is_pause)
        {
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // check
        if(is_everything_fine() == false)
        {
            mobile->move(0, 0, 0);
            is_moving = false;

            printf("[AUTO] something wrong\n");
            break;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        // finite state machine
        if(fsm_state == AUTO_FSM_DRIVING)
        {
            // calc error and ref vel
            Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
            Eigen::Vector3d tgt_pos = path_pos[tgt_idx];
            Eigen::Vector3d _tgt_pos = cur_tf_inv.block(0,0,3,3)*tgt_pos + cur_tf_inv.block(0,3,3,1);
            double tgt_th = path_th[tgt_idx];

            double err_x = _tgt_pos[0];
            double err_y = _tgt_pos[1];
            double err_th = deltaRad(tgt_th, cur_xi[2]);

            double err_d = std::sqrt(err_x*err_x + err_y*err_y);
            double dir_x = err_x/err_d;
            double dir_y = err_y/err_d;

            int ref_idx = get_nn_idx(global_path.pos, cur_pos);
            double ref_v = global_path.ref_v[ref_idx];

            double goal_err_d = calc_dist_2d(path_pos.back() - cur_pos);
            double goal_err_th = deltaRad(path_th.back(), cur_xi[2]);
            double goal_v = 0.75*goal_err_d;

            // obs decel
            QString _obs_condition = "none";
            double obs_v = 0.1;
            for(double vv = 0.1; vv <= params.LIMIT_V+0.01; vv += 0.1)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(dir_x*vv, dir_y*vv, 0), 0.2, config->OBS_PREDICT_TIME, cur_tf);

                bool is_collision = false;
                for(size_t p = 0; p < traj.size(); p++)
                {
                    if(obsmap->is_tf_collision_dynamic(traj[p], 0.3, 0.3))
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
                double stop_t = std::min<double>(config->OBS_DEADZONE, goal_err_d)/0.1;
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(dir_x*0.1, dir_y*0.1, 0), 0.5, stop_t, cur_tf);
                if(obsmap->is_path_collision(traj))
                {
                    obs_v = 0;
                    _obs_condition = "near";
                }
            }

            // for mobile server
            mtx.lock();
            obs_condition = _obs_condition;
            mtx.unlock();

            // stop due to obstacle
            if(obs_v == 0)
            {
                pre_err_th = 0;
                mobile->move(0, 0, 0);

                obs_wait_st_time = get_time();
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] DRIVING -> OBS\n");
                continue;
            }

            // calc control input
            double ref_v0 = std::sqrt(cur_vel[0]*cur_vel[0] + cur_vel[1]*cur_vel[1]);            
            ref_v = saturation(ref_v, ref_v0 - params.LIMIT_V_ACC*dt, ref_v0 + params.LIMIT_V_ACC*dt);
            ref_v = saturation(ref_v, 0, goal_v);
            ref_v = saturation(ref_v, 0, obs_v);

            double kp_w = 1.0;
            double kd_w = 0.1;

            double vx = dir_x*ref_v;
            double vy = dir_y*ref_v;
            double wz = kp_w*err_th + kd_w*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            wz = saturation(wz, cur_vel[2] - (params.LIMIT_W_ACC*D2R)*dt, cur_vel[2] + (params.LIMIT_W_ACC*D2R)*dt);            
            wz = saturation(wz, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // scaling
            double scale_v = 1.0 - params.DRIVE_T*std::abs(wz/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(ref_v/params.LIMIT_V);
            vx *= scale_v;
            vy *= scale_v;
            wz *= scale_w;

            // tgt check            
            if(calc_dist_2d(tgt_pos - cur_pos) < config->DRIVE_GOAL_D)
            {
                tgt_idx++;
                if(tgt_idx > (int)path_pos.size()-1)
                {
                    tgt_idx = path_pos.size()-1;
                }
            }

            // goal check            
            if(std::abs(goal_err_d) < config->DRIVE_GOAL_D && std::abs(goal_err_th) < config->DRIVE_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME)
                {
                    extend_dt = 0;

                    if(code->is_recv_data == true)
                    {
                        mobile->move(0, 0, 0);
                        dctrl->move(goal_tf);
                        fsm_state = AUTO_FSM_DOCKING;
                        printf("[AUTO] DRIVE COMPLETE, Found code Start Docking. goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);

                        std::this_thread::sleep_for(std::chrono::milliseconds(100));
                        continue;
                    }

                    mobile->move(0, 0, 0);
                    is_moving = false;
                    Q_EMIT signal_move_succeed("very good");

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] DRIVE COMPLETE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                    return;
                }
            }

            // send control
            mobile->move(vx, vy, wz);
        }
        else if(fsm_state == AUTO_FSM_DOCKING)
        {
            if(dctrl->fsm_state == DOCKING_FSM_COMPLETE)
            {
                mobile->move(0, 0, 0);
                is_moving = false;
                Q_EMIT signal_move_succeed("very good");

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] DOCKING COMPLETE\n");
                return;
            }

            else if(dctrl->fsm_state == DOCKING_FSM_FAILED)
            {
                mobile->move(0, 0, 0);
                is_moving = false;
                Q_EMIT signal_move_succeed("very good");

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] DOCKING FAILED\n");
                return;
            }
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(get_time() - obs_wait_st_time > 1.0)
            {
                extend_dt = 0;
                pre_err_th = 0;

                fsm_state = AUTO_FSM_DRIVING;
                printf("[AUTO] OBS_WAIT -> DRIVING\n");
                continue;
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

    is_moving = false;
    Q_EMIT signal_move_succeed("stopped");

    printf("[AUTO] b_loop_hpp stop\n");
}

void AUTOCONTROL::b_loop_tng(Eigen::Matrix4d goal_tf)
{
    // set flag
    is_moving = true;

    // calc global path
    PATH global_path = calc_global_path(goal_tf);
    if(global_path.nodes.size() == 0)
    {
        // no global path
        mobile->move(0, 0, 0);
        is_moving = false;

        Q_EMIT signal_move_failed("no global path");
        printf("[AUTO] global path init failed\n");
        return;
    }
    else
    {
        // update global path
        mtx.lock();
        cur_global_path = global_path;
        mtx.unlock();

        Q_EMIT signal_global_path_updated();
        printf("[AUTO] global path found\n");
    }

    // get metric node path
    std::vector<Eigen::Vector3d> path_pos;
    std::vector<double> path_th;
    for(size_t p = 0; p < global_path.nodes.size(); p++)
    {
        NODE* node = unimap->get_node_by_id(global_path.nodes[p]);
        if(node == NULL)
        {
            // no global path
            mobile->move(0, 0, 0);
            is_moving = false;

            Q_EMIT signal_move_failed("topology wrong\n");
            printf("[AUTO] global path init failed\n");
            return;
        }
        Eigen::Vector3d xi = TF_to_se2(node->tf);
        path_pos.push_back(node->tf.block(0,3,3,1));
        path_th.push_back(xi[2]);
    }

    // set init state
    fsm_state = AUTO_FSM_FIRST_ALIGN;
    printf("[AUTO] AUTO_FSM_FIRST_ALIGN\n");

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();    

    // control params
    double extend_dt = 0;
    double pre_err_d = 0;
    double pre_err_th = 0;

    // for obs
    double obs_wait_st_time = 0;

    // set first tgt idx
    Eigen::Matrix4d cur_tf0 = slam->get_cur_tf();
    Eigen::Vector3d cur_pos0 = cur_tf0.block(0,3,3,1);

    int tgt_idx = 0;
    if(path_pos.size() >= 2)
    {
        if(calc_dist_2d(path_pos[0] - cur_pos0) < config->DRIVE_GOAL_D)
        {
            // set new tgt idx
            int tgt_idx0 = tgt_idx;
            int tgt_idx1 = tgt_idx+1;

            tgt_idx = tgt_idx1;
            for(size_t p = tgt_idx+1; p < path_pos.size(); p++)
            {
                if(!check_same_line(path_pos[tgt_idx0], path_pos[tgt_idx1], path_pos[p]))
                {
                    break;
                }

                tgt_idx = p;
            }
        }
    }

    printf("[AUTO] b_loop_tng start\n");
    while(b_flag)
    {
        // pause
        if(is_pause)
        {
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // check
        if(is_everything_fine() == false)
        {
            mobile->move(0, 0, 0);
            is_moving = false;

            printf("[AUTO] something wrong\n");
            break;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        // get target node
        Eigen::Vector3d tgt_pos = path_pos[tgt_idx];

        // for plot
        mtx.lock();
        last_cur_pos = cur_pos;
        last_tgt_pos = tgt_pos;
        mtx.unlock();

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // calc heading error
            double dx = tgt_pos[0] - cur_pos[0];
            double dy = tgt_pos[1] - cur_pos[1];
            double err_th = deltaRad(std::atan2(dy, dx), cur_xi[2]);

            // pivot control
            double kp = 1.0;
            double kd = 0.05;
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
                    extend_dt = 0;
                    pre_err_d = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    fsm_state = AUTO_FSM_DRIVING;
                    printf("[AUTO] FIRST_ALIGN -> DRIVING, err_th:%f\n", err_th*R2D);
                    continue;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, 0.2, config->OBS_PREDICT_TIME, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mtx.lock();
                obs_condition = "near";
                mtx.unlock();

                mobile->move(0, 0, 0);

                obs_wait_st_time = get_time();
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] FIRST_ALIGN -> OBS_WAIT, err_th:%f\n", err_th*R2D);
                continue;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // calc errors
            double dx = tgt_pos[0] - cur_pos[0];
            double dy = tgt_pos[1] - cur_pos[1];
            double err_d = std::sqrt(dx*dx + dy*dy);
            double err_th = deltaRad(std::atan2(dy, dx), cur_xi[2]);
            if(std::abs(err_th) > 5.0*D2R)
            {
                err_th = sgn(err_th)*(5.0*D2R);
            }

            // obs decel
            QString _obs_condition = "none";
            double obs_v = 0.1;
            for(double vv = 0.1; vv <= params.LIMIT_V+0.01; vv += 0.1)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, config->OBS_PREDICT_TIME, cur_tf);

                bool is_collision = false;
                for(size_t p = 0; p < traj.size(); p++)
                {
                    if(obsmap->is_tf_collision_dynamic(traj[p], 0.3, 0.3))
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
                double stop_t = std::min<double>(config->OBS_DEADZONE, err_d)/0.1;
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(0.1, 0, 0), 0.5, stop_t, cur_tf);
                if(obsmap->is_path_collision(traj))
                {
                    obs_v = 0;
                    _obs_condition = "near";
                }
            }

            // for mobile server
            mtx.lock();
            obs_condition = _obs_condition;
            mtx.unlock();

            // stop due to obs
            if(obs_v == 0)
            {
                mobile->move(0, 0, 0);

                obs_wait_st_time = get_time();
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] DRIVING -> OBS\n");
                continue;
            }

            // calc control input
            double kp_v = 0.75;
            double kd_v = 0.05;
            double v0 = cur_vel[0];
            double v = saturation(kp_v*err_d + kd_v*(err_d - pre_err_d)/dt, params.ED_V, params.LIMIT_V);
            pre_err_d = err_d;

            v = saturation(v, 0, v0 + params.LIMIT_V_ACC*dt);
            v = saturation(v, 0, params.LIMIT_V);
            v = saturation(v, 0, obs_v);

            double kp_w = 1.0;
            double kd_w = 0.05;
            double w0 = cur_vel[2];
            double w = kp_w*err_th + kd_w*(err_th - pre_err_th)/dt;
            pre_err_th = err_th;

            w = saturation(w, w0 - (params.LIMIT_W_ACC*D2R)*dt, w0 + (params.LIMIT_W_ACC*D2R)*dt);
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // limit
            double scale_v = 1.0 - params.DRIVE_T*std::abs(w/(params.LIMIT_W*D2R));
            double scale_w = 1.0 - params.DRIVE_T*std::abs(v/params.LIMIT_V);
            v *= scale_v;
            w *= scale_w;

            // goal check
            if(std::abs(err_d) < config->DRIVE_GOAL_D)
            {
                Eigen::Matrix4d cur_tf_inv = cur_tf.inverse();
                Eigen::Vector3d _tgt_pos = cur_tf_inv.block(0,0,3,3)*tgt_pos + cur_tf_inv.block(0,3,3,1);

                extend_dt += dt;
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME || _tgt_pos[0] < 0)
                {
                    extend_dt = 0;

                    if(tgt_idx == (int)path_pos.size()-1)
                    {
                        pre_err_d = 0;
                        pre_err_th = 0;
                        mobile->move(0, 0, 0);

                        fsm_state = AUTO_FSM_FINAL_ALIGN;
                        printf("[AUTO] DRIVING -> FINAL_ALIGN, err_d:%f\n", err_d);
                        continue;
                    }
                    else
                    {
                        pre_err_d = 0;
                        pre_err_th = 0;
                        mobile->move(0, 0, 0);

                        // set new tgt idx
                        int tgt_idx0 = tgt_idx;
                        int tgt_idx1 = tgt_idx+1;

                        tgt_idx = tgt_idx1;
                        for(size_t p = tgt_idx+1; p < path_pos.size(); p++)
                        {
                            if(!check_same_line(path_pos[tgt_idx0], path_pos[tgt_idx1], path_pos[p]))
                            {
                                break;
                            }
                            tgt_idx = p;
                        }

                        fsm_state = AUTO_FSM_FIRST_ALIGN;
                        printf("[AUTO] DRIVING -> FIRST_ALIGN, tgt_idx: %d->%d, err_d:%f\n", tgt_idx0, tgt_idx, err_d);
                        continue;
                    }

                    // decel only
                    v = saturation(v, 0, v0);
                    w = 0;
                }
            }

            // send control
            mobile->move(v, 0, w);
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            // calc heading error
            double err_th = deltaRad(path_th.back(), cur_xi[2]);

            // pivot control
            double kp = 1.0;
            double kd = 0.05;
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
                    mobile->move(0, 0, 0);
                    is_moving = false;
                    Q_EMIT signal_move_succeed("very good");

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] FINAL ALIGN COMPLETE, err_th: %.3f\n", err_th*R2D);
                    return;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(cur_vel, 0.2, config->OBS_PREDICT_TIME, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mtx.lock();
                obs_condition = "near";
                mtx.unlock();

                mobile->move(0, 0, 0);
                is_moving = false;
                Q_EMIT signal_move_succeed("early stopped");

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] FINAL ALIGN COMPLETE(OBS), err_th: %.3f\n", err_th*R2D);
                return;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(get_time() - obs_wait_st_time > 1.0)
            {
                extend_dt = 0;
                pre_err_d = 0;
                pre_err_th = 0;

                fsm_state = AUTO_FSM_FIRST_ALIGN;
                printf("[AUTO] OBS -> FIRST_ALIGN\n");
                continue;
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

    is_moving = false;
    Q_EMIT signal_move_succeed("stopped");

    printf("[AUTO] b_loop_tng stop\n");
}

