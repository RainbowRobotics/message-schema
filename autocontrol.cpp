#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent)
    : QObject{parent}
{
}

AUTOCONTROL::~AUTOCONTROL()
{
    stop();
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path;
    preset_path.sprintf("/preset_%d.json", preset);
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

            res.LIMIT_W_ACC = obj["LIMIT_W_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_W_ACC :%f\n", res.LIMIT_W_ACC);

            res.LIMIT_PIVOT_W = obj["LIMIT_PIVOT_W"].toString().toDouble();
            printf("[PRESET] LIMIT_PIVOT_W :%f\n", res.LIMIT_PIVOT_W);

            res.PP_MIN_LD = obj["PP_MIN_LD"].toString().toDouble();
            printf("[PRESET] PP_MIN_LD :%f\n", res.PP_MIN_LD);

            res.PP_MAX_LD = obj["PP_MAX_LD"].toString().toDouble();
            printf("[PRESET] PP_MAX_LD :%f\n", res.PP_MAX_LD);

            res.PP_ST_V = obj["PP_ST_V"].toString().toDouble();
            printf("[PRESET] PP_ST_V :%f\n", res.PP_ST_V);

            res.PP_ED_V = obj["PP_ED_V"].toString().toDouble();
            printf("[PRESET] PP_ED_V :%f\n", res.PP_ED_V);

            file.close();
        }
    }

    return res;
}

GLOBAL_PATH AUTOCONTROL::get_cur_global_path()
{
    mtx.lock();
    GLOBAL_PATH res = cur_global_path;
    mtx.unlock();

    return res;
}

void AUTOCONTROL::stop()
{
    if(fsm_thread != NULL)
    {
        fsm_flag = false;
        fsm_thread->join();
        fsm_thread = NULL;
    }

    mtx.lock();
    cur_global_path = GLOBAL_PATH();
    mtx.unlock();
}

void AUTOCONTROL::move_pp(Eigen::Matrix4d goal_tf, int preset)
{
    // pure pursuit

    // set preset
    params = load_preset(preset);

    // global path finding
    GLOBAL_PATH _cur_global_path = calc_global_path(goal_tf, 0);

    // update global path
    mtx.lock();
    cur_global_path = _cur_global_path;
    mtx.unlock();

    // control loop start
    if(fsm_thread != NULL)
    {
        fsm_flag = false;
        fsm_thread->join();
        fsm_thread = NULL;
    }

    fsm_flag = true;
    fsm_thread = new std::thread(&AUTOCONTROL::fsm_loop_pp, this);
}

void AUTOCONTROL::move_hpp(Eigen::Matrix4d goal_tf, int preset)
{
    // holonomic pure pursuit

    // set preset
    params = load_preset(preset);

    // global path finding
    GLOBAL_PATH _cur_global_path = calc_global_path(goal_tf, 1);

    // update global path
    mtx.lock();
    cur_global_path = _cur_global_path;
    mtx.unlock();

    // control loop start
    if(fsm_thread != NULL)
    {
        fsm_flag = false;
        fsm_thread->join();
        fsm_thread = NULL;
    }

    fsm_flag = true;
    fsm_thread = new std::thread(&AUTOCONTROL::fsm_loop_hpp, this);
}

void AUTOCONTROL::move_tng(Eigen::Matrix4d goal_tf, int preset)
{
    // turn and go

    // set preset
    params = load_preset(preset);

    // global path finding
    GLOBAL_PATH _cur_global_path = calc_global_path(goal_tf, 1);

    // update global path
    mtx.lock();
    cur_global_path = _cur_global_path;
    mtx.unlock();

    // control loop start
    if(fsm_thread != NULL)
    {
        fsm_flag = false;
        fsm_thread->join();
        fsm_thread = NULL;
    }

    fsm_flag = true;
    fsm_thread = new std::thread(&AUTOCONTROL::fsm_loop_tng, this);
}

GLOBAL_PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf, int type)
{
    if(type == 0)
    {
        // divide, smooth path planning

        // get goal nn node
        Eigen::Vector3d goal = goal_tf.block(0,3,3,1);
        QString goal_node_id = unimap->get_node_id_nn(goal);
        if(goal_node_id == "")
        {
            printf("[AUTO] calc_global_path, check nodes\n");
            return GLOBAL_PATH();
        }

        // path finding
        std::vector<QString> node_path = topo_path_finding(goal_node_id);
        if(node_path.size() == 0)
        {
            printf("[AUTO] calc global path failed\n");
            return GLOBAL_PATH();
        }

        // convert metric path
        std::vector<Eigen::Vector3d> metric_path;
        for(size_t p = 0; p < node_path.size(); p++)
        {
            QString name = node_path[p];
            NODE* node = unimap->get_node_by_id(name);
            if(node == NULL)
            {
                printf("[AUTO] calc global path failed\n");
                return GLOBAL_PATH();
            }

            Eigen::Vector3d pos = node->tf.block(0,3,3,1);
            metric_path.push_back(pos);
        }

        // maybe st == ed
        if(metric_path.size() == 1)
        {
            Eigen::Vector3d cur_pos = slam->get_cur_tf().block(0,3,3,1);
            metric_path.insert(metric_path.begin(), cur_pos);
        }

        // add goal pos
        if(goal != metric_path.back())
        {
            metric_path.push_back(goal);
        }

        // divide global path and smoothing using ccma
        std::vector<Eigen::Vector3d> path = path_dividing(metric_path, GLOBAL_PATH_STEP);
        path = path_ccma(path);

        // set ref_v
        std::vector<double> ref_v = calc_ref_v(path);

        // smoothing ref_v
        ref_v = smoothing_v(ref_v);

        // set result
        GLOBAL_PATH res;
        res.nodes = node_path;
        res.pos = path;
        res.ref_v = ref_v;
        res.goal_tf = goal_tf;
        return res;
    }
    else if(type == 1)
    {
        // no divide, no smoothing, just node-metric path

        // get goal nn node
        Eigen::Vector3d goal = goal_tf.block(0,3,3,1);
        QString goal_node_id = unimap->get_node_id_nn(goal);
        if(goal_node_id == "")
        {
            printf("[AUTO] calc_global_path, check nodes\n");
            return GLOBAL_PATH();
        }

        // path finding
        std::vector<QString> node_path = topo_path_finding(goal_node_id);
        if(node_path.size() == 0)
        {
            printf("[AUTO] calc global path failed\n");
            return GLOBAL_PATH();
        }

        // convert metric path
        std::vector<Eigen::Vector3d> metric_path;
        for(size_t p = 0; p < node_path.size(); p++)
        {
            QString name = node_path[p];
            NODE* node = unimap->get_node_by_id(name);
            if(node == NULL)
            {
                printf("[AUTO] calc global path failed\n");
                return GLOBAL_PATH();
            }

            Eigen::Vector3d pos = node->tf.block(0,3,3,1);
            metric_path.push_back(pos);
        }

        // maybe st == ed
        if(metric_path.size() == 1)
        {
            Eigen::Vector3d cur_pos = slam->get_cur_tf().block(0,3,3,1);
            metric_path.insert(metric_path.begin(), cur_pos);
        }

        // add goal pos
        if(goal != metric_path.back())
        {
            metric_path.push_back(goal);
        }

        // set ref_v
        std::vector<double> ref_v(metric_path.size(), params.LIMIT_V);

        // set result
        GLOBAL_PATH res;
        res.nodes = node_path;
        res.pos = metric_path;
        res.ref_v = ref_v;
        res.goal_tf = goal_tf;
        return res;
    }
    else if(type == 2)
    {
        // using hybrid a star search, ignore topology
    }
}

std::vector<QString> AUTOCONTROL::topo_path_finding(QString goal_node_id)
{
    // cur robot pose
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

    // set st node, ed node
    NODE* st_node = unimap->get_edge_nn(cur_pos);
    NODE* ed_node = unimap->get_node_by_id(goal_node_id);
    if(st_node == NULL || ed_node == NULL)
    {
        printf("[AUTO] global path finding, failed\n");
        return std::vector<QString>();
    }

    printf("[AUTO] st: %s -> ed: %s\n", st_node->id.toLocal8Bit().data(), ed_node->id.toLocal8Bit().data());

    // no need path finding
    if(st_node->id == ed_node->id)
    {
        printf("[AUTO] equal st, ed node\n");

        std::vector<QString> res;
        res.push_back(goal_node_id);
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

            printf("[AUTO] global path finding, success, path num:%d\n", (int)res.size());
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

    printf("[AUTO] global path finding, failed, no way\n");
    return std::vector<QString>();
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_dividing(std::vector<Eigen::Vector3d> src, double step)
{
    std::vector<Eigen::Vector3d> divided_path;

    if(src.empty())
    {
        return divided_path;
    }

    divided_path.push_back(src[0]);

    for (size_t i = 1; i < src.size(); ++i)
    {
        Eigen::Vector3d prev_point = src[i - 1];
        Eigen::Vector3d curr_point = src[i];
        Eigen::Vector3d segment = curr_point - prev_point;
        double segment_length = segment.norm();

        if (segment_length <= step)
        {
            if (i == src.size() - 1)
            {
                divided_path.push_back(curr_point);
            }
        }
        else
        {
            int num_divisions = static_cast<int>(segment_length / step);
            Eigen::Vector3d step_vec = segment.normalized() * step;

            for (int j = 1; j <= num_divisions; ++j)
            {
                Eigen::Vector3d new_point = prev_point + step_vec * j;
                divided_path.push_back(new_point);
            }

            if (i == src.size() - 1)
            {
                divided_path.push_back(curr_point);
            }
            else
            {
                Eigen::Vector3d last_point = prev_point + step_vec * num_divisions;
                if ((curr_point - last_point).norm() > step * 0.5)
                {
                    divided_path.push_back(curr_point);
                }
            }
        }
    }

    return divided_path;
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_ccma(std::vector<Eigen::Vector3d> src)
{
    // init params
    const int w_ma = 10; // 5
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

std::vector<double> AUTOCONTROL::calc_ref_v(std::vector<Eigen::Vector3d> src)
{
    std::vector<double> res(src.size(), params.PP_ST_V);
    if(src.size() >= 3)
    {
        for(size_t p = 1; p < src.size()-1; p++)
        {
            double kappa = calc_curvature(src[p-1], src[p], src[p+1]);
            if(kappa == 0)
            {
                res[p] = params.LIMIT_V;
            }
            else
            {
                double w = params.LIMIT_V * kappa;
                w = std::min<double>(w, params.LIMIT_W*D2R);

                double v = w / kappa;
                v = std::min(v, params.LIMIT_V);
                res[p] = v;
            }
        }
    }

    res.front() = params.PP_ST_V;
    res.back() = params.PP_ED_V;
    return res;
}

std::vector<double> AUTOCONTROL::smoothing_v(std::vector<double> src)
{
    const double v_limit = params.LIMIT_V;
    const double v_acc = params.LIMIT_V_ACC;
    const double path_step = GLOBAL_PATH_STEP;

    std::vector<double> list0(src.size());
    double v0 = src.front();
    for(int p = 0; p < (int)src.size()-1; p++)
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

        list1[p] = v0;
    }

    std::vector<double> res(src.size());
    for(int p = 0; p < (int)src.size()-1; p++)
    {
        res[p] = std::min<double>(list0[p], list1[p]);
    }
    return res;
}

// for local path planning


// check condition
bool AUTOCONTROL::is_everything_fine()
{
    return true;
}

std::pair<int, Eigen::Vector3d> AUTOCONTROL::get_nn_pos(GLOBAL_PATH& path, Eigen::Vector3d cur_pos)
{
    int min_idx = 0;
    Eigen::Vector3d min_pos;
    double min_d = 99999999;
    for(size_t p = 0; p < path.pos.size()-1; p++)
    {
        auto res = calc_seg_pt_dist(path.pos[p], path.pos[p+1], cur_pos);
        if(res.second < min_d)
        {
            min_pos = res.first;
            min_d = res.second;
            min_idx = p;
        }
    }

    return std::make_pair(min_idx, min_pos);
}

Eigen::Vector3d AUTOCONTROL::get_tgt(GLOBAL_PATH& path, double dist, Eigen::Vector3d nn_pos, int nn_idx)
{
    bool is_found = false;
    Eigen::Vector3d intersection;
    for(size_t i = nn_idx; i < path.pos.size()-1; ++i)
    {
        if(calc_seg_sphere_intersection(path.pos[i], path.pos[i+1], nn_pos, dist, intersection))
        {
            is_found = true;
            break;
        }
    }

    if(is_found)
    {
        return intersection;
    }
    else
    {
        return path.pos.back();
    }
}

// loops
void AUTOCONTROL::fsm_loop_pp()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[AUTO] fsm_loop_pp start\n");
    while(fsm_flag)
    {





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

    printf("[AUTO] fsm_loop_pp stop\n");
}

void AUTOCONTROL::fsm_loop_hpp()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[AUTO] fsm_loop_hpp start\n");
    while(fsm_flag)
    {



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

    printf("[AUTO] fsm_loop_hpp stop\n");
}

void AUTOCONTROL::fsm_loop_tng()
{
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[AUTO] fsm_loop_tng start\n");
    while(fsm_flag)
    {



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

    printf("[AUTO] fsm_loop_tng stop\n");
}


