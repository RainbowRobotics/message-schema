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

    clear_path();
}

void AUTOCONTROL::move_pp(Eigen::Matrix4d goal_tf, int preset)
{
    // pure pursuit

    // obs clear first
    obsmap->clear();

    // set preset
    params = load_preset(preset);

    // global path finding
    PATH _cur_global_path = calc_global_path(goal_tf);
    if(_cur_global_path.pos.size() > 0)
    {
        // stop first
        stop();

        // update global path
        mtx.lock();
        cur_global_path = _cur_global_path;
        mtx.unlock();

        Q_EMIT signal_global_path_updated();

        // start control loop
        b_flag = true;
        b_thread = new std::thread(&AUTOCONTROL::b_loop_pp, this);
    }
    else
    {
        printf("[AUTO] no global path\n");
    }
}

void AUTOCONTROL::move_hpp(Eigen::Matrix4d goal_tf, int preset)
{
    // holonomic pure pursuit

    // set preset
    params = load_preset(preset);

    // global path finding
    PATH _cur_global_path = calc_global_path(goal_tf);

    // update global path
    mtx.lock();
    cur_global_path = _cur_global_path;
    mtx.unlock();

    Q_EMIT signal_global_path_updated();

    // control loop start
    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }

    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_hpp, this);
}

void AUTOCONTROL::move_tng(Eigen::Matrix4d goal_tf, int preset)
{
    // turn and go

    // set preset
    params = load_preset(preset);

    // global path finding    
    PATH _cur_global_path;

    // update global path
    mtx.lock();
    cur_global_path = _cur_global_path;
    mtx.unlock();

    Q_EMIT signal_global_path_updated();

    // control loop start
    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }

    b_flag = true;
    b_thread = new std::thread(&AUTOCONTROL::b_loop_tng, this);
}

PATH AUTOCONTROL::calc_global_path(Eigen::Matrix4d goal_tf)
{
    // check already goal reached
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    if((goal_tf.block(0,3,3,1) - cur_tf.block(0,3,3,1)).norm() < config->DRIVE_GOAL_D)
    {
        printf("[AUTO] near goal\n");
        return PATH();
    }

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
    std::vector<Eigen::Vector3d> metric_path;
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
        metric_path.push_back(pos);
    }

    // add cur pos
    if(metric_path.size() == 1)
    {
        metric_path.insert(metric_path.begin(), cur_pos);
    }
    else
    {
        if(!check_point_on_segment(metric_path[0], metric_path[1], cur_pos))
        {
            metric_path.insert(metric_path.begin(), cur_pos);
        }
    }

    // add goal pos
    if(goal_pos != metric_path.back())
    {
        metric_path.push_back(goal_pos);
    }

    // divide and smooth metric path    
    std::vector<Eigen::Vector3d> path_pos = path_resampling(metric_path, GLOBAL_PATH_STEP);
    path_pos = path_ccma(path_pos);

    // calc pose
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // set result
    PATH res;
    res.nodes = node_path;
    res.pose = path_pose;
    res.pos = path_pos;    
    res.goal_tf = goal_tf;
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
    return res;
}

std::vector<Eigen::Vector3d> AUTOCONTROL::path_ccma(const std::vector<Eigen::Vector3d>& src)
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

void AUTOCONTROL::calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_th, std::vector<double>& ref_v, double st_v)
{
    const int ld = 30; // 50?

    std::vector<double> _ref_th(src.size(), 0);
    for(size_t p = 0; p < src.size()-1; p++)
    {
        int cur_idx = p;
        int tgt_idx = p+ld;
        if(tgt_idx > (int)src.size()-1)
        {
            tgt_idx = src.size()-1;
        }

        double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
        double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
        double th = std::atan2(dy, dx);
        _ref_th[cur_idx] = th;
        _ref_th[cur_idx+1] = th;
    }

    std::vector<double> _ref_v(src.size(), params.LIMIT_V);
    for(size_t p = 0; p < src.size()-1; p++)
    {
        int cur_idx = p;
        int tgt_idx = p+ld;
        if(tgt_idx > (int)src.size()-1)
        {
            tgt_idx = src.size()-1;
        }

        double dx = src[tgt_idx](0,3) - src[cur_idx](0,3);
        double dy = src[tgt_idx](1,3) - src[cur_idx](1,3);
        double err_d = std::sqrt(dx*dx + dy*dy);
        double err_th = deltaRad(_ref_th[tgt_idx], _ref_th[cur_idx]) * params.DRIVE_H;

        double t_v = err_d/params.LIMIT_V;
        double t_w = std::abs(err_th)/(params.LIMIT_W*D2R);
        double t = std::max<double>(t_v, t_w);
        double v = err_d/t;

        int idx0 = cur_idx;
        int idx1 = cur_idx + 100;
        if(idx1 > (int)src.size()-1)
        {
            idx1 = src.size()-1;
        }

        for(int q = idx0; q <= idx1; q++)
        {
            if(v < _ref_v[q])
            {
                _ref_v[q] = v;
            }
        }
    }
    _ref_v.front() = st_v;

    // update result
    ref_th = _ref_th;
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

std::vector<double> AUTOCONTROL::gaussian_filter(const std::vector<double>& src, int mask, double sigma)
{
    cv::Mat kernel = cv::getGaussianKernel(mask, sigma, CV_64F);

    std::vector<double> res(src.size(), 0);
    for(int p = 0; p < (int)src.size(); p++)
    {
        double sum = 0;
        for(int q = 0; q < mask; q++)
        {
            int i = p - mask/2 + q;
            if(i < 0 || i >= (int)src.size())
            {
                continue;
            }

            sum += src[i]*kernel.ptr<double>(q)[0];
        }
        res[p] = sum;
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
    for(double t = dt; t <= predict_t; t += dt) // maybe relexation obs stuck condition..?
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

int AUTOCONTROL::get_valid_idx(std::vector<Eigen::Matrix4d>& path, int st_idx)
{
    const double margin_x = config->OBS_OMPL_MARGIN_X;
    const double margin_y = config->OBS_OMPL_MARGIN_Y;

    for(int p = st_idx; p < (int)path.size(); p++)
    {
        if(!obsmap->is_tf_collision(path[p], margin_x, margin_y))
        {
            printf("[AUTO] found valid idx, %d -> %d\n", st_idx, p);
            return p;
        }
    }

    printf("[AUTO] no valid idx\n");
    return st_idx;
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

Eigen::Vector3d AUTOCONTROL::refine_force(Eigen::Vector3d f, Eigen::Vector3d P0, Eigen::Vector3d P1)
{
    Eigen::Vector3d dir = P1-P0;
    double d2 = dir.squaredNorm();
    double alpha = f.dot(dir)/d2;
    Eigen::Vector3d _f = alpha*dir;
    Eigen::Vector3d res_f = f - _f;
    return res_f;
}

PATH AUTOCONTROL::calc_local_path()
{
    // get params
    PATH global_path = get_cur_global_path();

    // get cur params
    Eigen::Matrix4d cur_tf = slam->get_cur_tf();
    Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);
    int cur_idx = get_nn_idx(global_path.pos, cur_pos);

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

    // get path segment
    std::vector<Eigen::Vector3d> _path_pos;
    _path_pos.push_back(cur_pos);

    int st_idx = std::min<int>(cur_idx+1, (int)global_path.pos.size()-1);
    bool is_global_goal = false;    
    for(int p = st_idx; p < (int)global_path.pos.size(); p++)
    {
        double d = calc_dist_2d(global_path.pos[p] - global_path.pos[cur_idx]);
        if(d <= config->OBS_LOCAL_GOAL_D)
        {
            _path_pos.push_back(global_path.pos[p]);
            if(p == (int)global_path.pos.size()-1)
            {
                is_global_goal = true;
            }
        }
        else
        {
            break;
        }
    }

    std::vector<Eigen::Matrix4d> _path_pose = calc_path_tf(_path_pos);

    // eband
    double global_goal_d = calc_dist_2d(global_path.pos.back()-cur_pos);
    if(global_goal_d > 1.0)
    {
        const double margin_x = config->OBS_EB_MARGIN_X;
        const double margin_y = config->OBS_EB_MARGIN_Y;
        const double range = 2.0;

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

            if(!obsmap->is_tf_collision(_path_pose[p], margin_x, margin_y))
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

            if(!obsmap->is_tf_collision(_path_pose[p], margin_x, margin_y))
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

        // calc similarity
        double similarity_l = 1.0 - calc_similarity(last_local_path.pos, modified_path_pos_l);
        double similarity_r = 1.0 - calc_similarity(last_local_path.pos, modified_path_pos_r);

        // calc length
        double length_l = calc_length(modified_path_pos_l);
        double length_r = calc_length(modified_path_pos_r);

        // cost
        double wa = 1.0;
        double wb = 1.0;
        double wc = 1.0;

        double cost_l = wa*sum_cnt_l + wb*similarity_l + wc*length_l;
        double cost_r = wa*sum_cnt_r + wb*similarity_r + wc*length_r;

        printf("cost_l:%f, %f, %f, cost_r:%f, %f, %f\n",
               wa*sum_cnt_l, wb*similarity_l, wc*length_l,
               wa*sum_cnt_r, wb*similarity_r, wc*length_r);

        // select one
        std::vector<Eigen::Vector3d> modified_path_pos;
        if(cost_l < cost_r)
        {
            modified_path_pos = modified_path_pos_l;
            printf("l, %f, %f\n", cost_l, cost_r);
        }
        else
        {
            modified_path_pos = modified_path_pos_r;
            printf("r, %f, %f\n", cost_l, cost_r);
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
    std::vector<Eigen::Vector3d> path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP*2, LOCAL_PATH_STEP);
    std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

    // check
    if(obsmap->is_path_collision(path_pose, 0, 20))
    {
        return PATH();
    }
    else
    {
        // set ref_v
        std::vector<double> ref_th;
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_th, ref_v, params.LIMIT_V);
        if(is_global_goal)
        {
            ref_v.back() = params.ED_V;
        }

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

        // set result
        PATH res;
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_th = ref_th;
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
    int _local_goal_idx = std::min<int>(cur_idx + config->OBS_LOCAL_GOAL_D/GLOBAL_PATH_STEP, (int)global_path.pose.size()-1);
    int local_goal_idx = get_valid_idx(global_path.pose, _local_goal_idx);
    Eigen::Matrix4d local_goal_tf = global_path.pose[local_goal_idx];
    Eigen::Vector3d local_goal_xi = TF_to_se2(local_goal_tf);

    // set state space
    auto stateSpace(std::make_shared<ompl::base::DubinsStateSpace>(0.001, false));

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
        printf("[AUTO] ompl, st_xi, obs stucked\n");
        return PATH();
    }

    // end
    Eigen::Vector3d ed_xi = local_goal_xi;
    Eigen::Matrix4d ed_tf = se2_to_TF(ed_xi);
    if(obsmap->is_tf_collision(ed_tf, margin_x, margin_y))
    {
        bool is_found = false;
        double min_d = 99999999;
        Eigen::Vector3d min_xi = ed_xi;
        for(double dx = -0.5; dx <= 0.5; dx += 0.1)
        {
            for(double dy = -0.5; dy <= 0.5; dy += 0.1)
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
    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    //auto planner = std::make_shared<ompl::geometric::PRMstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // solve
    ompl::base::PlannerStatus solved = ss.solve(3.0);
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
        std::vector<Eigen::Vector3d> path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP, LOCAL_PATH_STEP);
        std::vector<Eigen::Matrix4d> path_pose = calc_path_tf(path_pos);

        // set ref_v
        std::vector<double> ref_th;
        std::vector<double> ref_v;
        calc_ref_v(path_pose, ref_th, ref_v, params.ST_V);
        ref_v.back() = params.ED_V;

        // smoothing ref_v
        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

        // set result
        PATH res;
        res.pose = path_pose;
        res.pos = path_pos;
        res.ref_th = ref_th;
        res.ref_v = ref_v;
        res.goal_tf = local_goal_tf;

        printf("[AUTO] ompl lpp, exact solution\n");
        return res;
    }
    else
    {
        printf("[AUTO] ompl lpp, no exact solution\n");
        return PATH();
    }
}

// check condition
bool AUTOCONTROL::is_everything_fine()
{
    return true;
}

// loops
void AUTOCONTROL::b_loop_pp()
{
    // set params
    is_moving = true;

    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();
    double extend_dt = 0;

    // get global path
    PATH global_path = get_cur_global_path();    
    if(global_path.pos.size() == 0)
    {
        mobile->move(0, 0, 0);
        is_moving = false;

        printf("[AUTO] global path init failed\n");
        return;
    }

    Eigen::Vector3d goal_pos = global_path.goal_tf.block(0,3,3,1);
    Eigen::Vector3d goal_xi = TF_to_se2(global_path.goal_tf);

    // for avoid path
    PATH avoid_path;

    // calc initial local path    
    PATH local_path = calc_local_path();
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

            printf("[AUTO] local path init failed\n");
            return;
        }
    }
    double last_local_path_t = get_time();

    // update local path
    mtx.lock();
    cur_local_path = local_path;
    last_local_goal = local_path.goal_tf.block(0,3,3,1);
    mtx.unlock();

    Q_EMIT signal_local_path_updated();


    fsm_state = AUTO_FSM_FIRST_ALIGN;
    int obs_state = 0;
    double reverse_st_t = 0;
    double pre_err_th = 0;

    printf("[AUTO] b_loop_pp start\n");
    while(b_flag)
    {
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

        // goal error d
        double goal_err_d = calc_dist_2d(goal_pos - cur_pos);
        double goal_err_th = deltaRad(goal_xi[2], cur_xi[2]);

        // check avoid path
        if(avoid_path.pos.size() > 0)
        {
            // check avoid path goal
            if(calc_dist_2d(avoid_path.pos.back() - cur_pos) < config->DRIVE_GOAL_D)
            {
                // clear avoid path
                mobile->move(0, 0, 0);
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
            if(get_time() - last_local_path_t > 0.2)
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
                    mobile->move(0, 0, 0);

                    obs_state = 0;
                    fsm_state = AUTO_FSM_OBS;
                    printf("[AUTO] DRIVING -> OBS\n");
                }

                last_local_path_t = get_time();
            }
        }

        // for debug
        //fsm_state = AUTO_FSM_DEBUG;

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx + 1;
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

            // calc error
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
                pre_err_th = 0;
                mobile->move(0, 0, 0);                

                fsm_state = AUTO_FSM_DRIVING;
                printf("[AUTO] FIRST_ALIGN -> DRIVING, err_th:%f\n", err_th*R2D);
                continue;
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(cur_vel[0], cur_vel[1], cur_vel[2]), 0.1, 0.3, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);

                obs_state = 0;
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
            int tgt_idx = cur_idx;

            Eigen::Matrix4d tgt_tf = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_xi = TF_to_se2(tgt_tf);
            Eigen::Vector3d tgt_pos = tgt_tf.block(0,3,3,1);

            // obs decel
            double obs_v = 0.1;
            for(double vv = 0.1; vv <= params.LIMIT_V+0.01; vv += 0.1)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.2, 1.5, cur_tf);
                if(!obsmap->is_path_collision(traj))
                {
                    obs_v = vv;
                }
                else
                {
                    break;
                }
            }

            // obs stop
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(cur_vel[0], cur_vel[1], cur_vel[2]), 0.1, 0.3, cur_tf);
                if(obsmap->is_path_collision(traj))
                {
                    obs_v = 0;
                }
            }

            if(obs_v == 0)
            {
                mobile->move(0, 0, 0);

                obs_state = 0;
                fsm_state = AUTO_FSM_OBS;
                printf("[AUTO] DRIVING -> OBS\n");
                continue;
            }

            // calc ref_v            
            double goal_v = 0.75 * goal_err_d;
            double ref_v = local_path.ref_v[cur_idx];

            // calc error theta
            double err_th = deltaRad(local_path.ref_th[cur_idx], cur_xi[2]);

            // calc cross track error
            double s = check_lr(tgt_xi[0], tgt_xi[1], tgt_xi[2], cur_pos[0], cur_pos[1]);            
            double cte = -std::copysign(calc_min_dist(local_path.pos, cur_pos), s);

            // for plot
            mtx.lock();
            last_cur_pos = cur_pos;
            last_tgt_pos = tgt_pos;
            mtx.unlock();

            // calc control input
            double v0 = cur_vel[0];
            double v = saturation(ref_v, v0 - params.LIMIT_V_DCC*dt, v0 + params.LIMIT_V_ACC*dt);
            v = saturation(v, 0, obs_v);
            v = saturation(v, 0, goal_v);

            double th = (params.DRIVE_A * err_th) + (params.DRIVE_B * (err_th-pre_err_th)/dt) + std::atan2(params.DRIVE_K * cte, v + params.DRIVE_EPS);           
            th = saturation(th, -45.0*D2R, 45.0*D2R);
            pre_err_th = err_th;

            double w = (v * std::tan(th)) / params.DRIVE_L;
            w = saturation(w, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            // goal check
            if(goal_err_d < config->DRIVE_GOAL_D)
            {
                extend_dt += dt;                
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME)
                {
                    extend_dt = 0;
                    pre_err_th = 0;
                    mobile->move(0, 0, 0);

                    fsm_state = AUTO_FSM_FINAL_ALIGN;
                    printf("[AUTO] DRIVING -> FINAL_ALIGN, goal_err_d:%f\n", goal_err_d);
                    continue;
                }

                // decel only
                v = saturation(v, 0, v0);
            }

            // send control
            mobile->move(v, 0, w);

            //printf("v:%f, w:%f, err_th:%f, cte:%f, ref_v:%f, obs_v:%f, goal_v:%f\n", v, w*R2D, err_th*R2D, cte, ref_v, obs_v, goal_v);
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            // pivot control
            double kp = 1.0;
            double kd = 0.05;
            double w0 = cur_vel[2];
            double w = kp*goal_err_th + kd*(goal_err_th - pre_err_th)/dt;
            pre_err_th = goal_err_th;

            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // goal check
            if(std::abs(goal_err_th) < config->DRIVE_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME)
                {
                    mobile->move(0, 0, 0);
                    is_moving = false;

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] FINAL ALIGN COMPLETE, goal_err_th: %.3f\n", goal_err_th*R2D);
                    break;
                }
            }

            // obs check
            std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(cur_vel[0], cur_vel[1], cur_vel[2]), 0.1, 0.3, cur_tf);
            if(obsmap->is_path_collision(traj))
            {
                mobile->move(0, 0, 0);
                is_moving = false;

                fsm_state = AUTO_FSM_COMPLETE;
                printf("[AUTO] FINAL ALIGN COMPLETE(OBS), goal_err_th: %.3f\n", goal_err_th*R2D);
                continue;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_OBS)
        {
            if(obs_state == 0)
            {
                // check stuck
                const double margin_x = config->OBS_OMPL_MARGIN_X;
                const double margin_y = config->OBS_OMPL_MARGIN_Y;
                if(obsmap->is_tf_collision(cur_tf, margin_x, margin_y))
                {
                    reverse_st_t = get_time();
                    obs_state = 1;
                    printf("[AUTO] OBS, RECOVERY\n");
                    continue;
                }
                else
                {
                    obs_state = 2;
                    printf("[AUTO] OBS, OMPL\n");
                    continue;
                }
            }
            else if(obs_state == 1)
            {
                // reverse moving
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(-params.ST_V, 0, 0), 0.1, 1.5, cur_tf);
                if(obsmap->is_path_collision(traj) || get_time() - reverse_st_t > 1.5)
                {
                    printf("[AUTO] OBS, STOP RECOVERY -> OMPL\n");

                    mobile->move(0, 0, 0);
                    obs_state = 2;
                    continue;
                }

                mobile->move(-params.ST_V, 0, 0);
            }
            else if(obs_state == 2)
            {
                // calc avoid path                
                avoid_path = calc_avoid_path();
                if(avoid_path.pos.size() > 0)
                {
                    pre_err_th = 0;
                    fsm_state = AUTO_FSM_FIRST_ALIGN;
                    printf("[AUTO] OBS -> FIRST_ALIGN\n");
                    continue;
                }
                else
                {
                    printf("[AUTO] OBS, OMPL failed\n");
                    obs_state = 0;
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
    printf("[AUTO] b_loop_pp stop\n");
}

void AUTOCONTROL::b_loop_hpp()
{
    is_moving = true;

    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[AUTO] b_loop_hpp start\n");
    while(b_flag)
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

    is_moving = false;
    printf("[AUTO] b_loop_hpp stop\n");
}

void AUTOCONTROL::b_loop_tng()
{
    is_moving = true;

    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // get global path
    PATH global_path = get_cur_global_path();
    if(global_path.pos.size() == 0)
    {
        mobile->move(0, 0, 0);
        is_moving = false;

        printf("[AUTO] global path init failed\n");
        return;
    }

    Eigen::Vector3d goal_pos = global_path.goal_tf.block(0,3,3,1);
    Eigen::Vector3d goal_xi = TF_to_se2(global_path.goal_tf);

    fsm_state = AUTO_FSM_FIRST_ALIGN;

    printf("[AUTO] b_loop_tng start\n");
    while(b_flag)
    {
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


        // goal error d
        double goal_err_d = calc_dist_2d(goal_pos - cur_pos);
        double goal_err_th = deltaRad(goal_xi[2], cur_xi[2]);

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {

        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {

        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {

        }
        else if(fsm_state == AUTO_FSM_OBS)
        {

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
    printf("[AUTO] b_loop_tng stop\n");
}

