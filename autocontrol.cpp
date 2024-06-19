#include "autocontrol.h"

AUTOCONTROL::AUTOCONTROL(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
    last_tgt_pos.setZero();
    last_replan_st.setZero();
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
    use_local_path = false;

    // local path loop stop
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void AUTOCONTROL::init()
{
    a_flag = true;
    a_thread = new std::thread(&AUTOCONTROL::a_loop, this);
}

CTRL_PARAM AUTOCONTROL::load_preset(int preset)
{
    CTRL_PARAM res;

    // read
    QString preset_path;
    preset_path.sprintf("/preset_%d.json", preset);
    preset_path = QDir::homePath() + "/RB_CONFIG" + preset_path;

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

            res.MIN_LD = obj["MIN_LD"].toString().toDouble();
            printf("[PRESET] MIN_LD :%f\n", res.MIN_LD);

            res.MAX_LD = obj["MAX_LD"].toString().toDouble();
            printf("[PRESET] MAX_LD :%f\n", res.MAX_LD);

            res.MIN_V = obj["MIN_V"].toString().toDouble();
            printf("[PRESET] MIN_V :%f\n", res.MIN_V);

            res.MIN_W = obj["MIN_W"].toString().toDouble();
            printf("[PRESET] MIN_W :%f\n", res.MIN_W);

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
    use_local_path = false;
}

void AUTOCONTROL::move_pp(Eigen::Matrix4d goal_tf, int preset)
{
    // pure pursuit

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
    QString st_node_id = unimap->get_node_id_los(cur_pos);
    if(st_node_id == "")
    {
        printf("[AUTO] st_node_id empty\n");
        return PATH();
    }

    // set ed node id
    Eigen::Vector3d goal_pos = goal_tf.block(0,3,3,1);
    QString ed_node_id = unimap->get_node_id_los(goal_pos);
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
    if(!check_point_on_segment(metric_path[0], metric_path[1], cur_pos))
    {
        metric_path.insert(metric_path.begin(), cur_pos);
    }

    // add goal pos
    if(goal_pos != metric_path.back())
    {
        metric_path.push_back(goal_pos);
    }

    // divide and smooth metric path
    std::vector<Eigen::Vector3d> path_pos = path_dividing(metric_path, GLOBAL_PATH_STEP);
    path_pos = path_ccma(path_pos);

    // calc pose
    std::vector<Eigen::Matrix4d> path_pose;    
    for(size_t p = 0; p < path_pos.size()-1; p++)
    {
        path_pose.push_back(calc_tf(path_pos[p], path_pos[p+1]));
    }

    Eigen::Matrix4d final_tf = path_pose.back();
    final_tf.block(0,3,3,1) = path_pos.back();
    path_pose.push_back(final_tf);

    // set ref_v
    std::vector<double> ref_v = calc_ref_v(path_pose, params.MIN_V);

    // smoothing ref_v
    ref_v = smoothing_v(ref_v, GLOBAL_PATH_STEP);

    // set result
    PATH res;
    res.pose = path_pose;
    res.pos = path_pos;
    res.ref_v = ref_v;
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

std::vector<Eigen::Vector3d> AUTOCONTROL::path_dividing(const std::vector<Eigen::Vector3d>& src, double step)
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

    if (divided_path.back() != src.back())
    {
        divided_path.push_back(src.back());
    }

    return divided_path;
}

std::vector<Eigen::Matrix4d> AUTOCONTROL::path_dividing(const std::vector<Eigen::Matrix4d>& src, double step)
{
    if(src.size() < 2)
    {
        return src;
    }

    std::vector<Eigen::Matrix4d> res;
    for(size_t p = 0; p < src.size() - 1; p++)
    {
        Eigen::Matrix4d st_tf = src[p];
        Eigen::Matrix4d ed_tf = src[p+1];

        Eigen::Vector3d st_tr = st_tf.block(0,3,3,1);
        Eigen::Vector3d ed_tr = ed_tf.block(0,3,3,1);

        Eigen::Quaterniond st_quat(st_tf.block<3,3>(0,0));
        Eigen::Quaterniond ed_quat(ed_tf.block<3,3>(0,0));

        double l = (ed_tr - st_tr).norm();
        int num_step = std::max((int)(l / step), 1);

        for(int q = 0; q < num_step; q++)
        {
            double alpha = (double)q/num_step;

            Eigen::Vector3d interp_tr = st_tr + alpha * (ed_tr - st_tr);
            Eigen::Quaterniond interp_quat = st_quat.slerp(alpha, ed_quat);

            Eigen::Matrix4d TF = Eigen::Matrix4d::Identity();
            TF.block(0,0,3,3) = interp_quat.toRotationMatrix();
            TF.block(0,3,3,1) = interp_tr;
            res.push_back(TF);
        }
    }

    res.push_back(src.back());
    return res;
}

std::vector<Eigen::Vector3d> AUTOCONTROL::sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step)
{
    if(src.empty())
    {
        return src;
    }

    std::vector<Eigen::Vector3d> sampled;
    sampled.push_back(src.front());

    for(size_t p = 1; p < src.size(); p++)
    {
        bool is_ok = true;
        for(size_t q = 0; q < sampled.size(); q++)
        {
            double d = (sampled[q] - src[p]).norm();
            if(d < large_step)
            {
                is_ok = false;
                break;
            }
        }

        if(is_ok)
        {
            sampled.push_back(src[p]);
        }
    }

    if(sampled.back() != src.back())
    {
        sampled.push_back(src.back());
    }

    std::vector<Eigen::Vector3d> res = path_dividing(sampled, small_step);
    res = path_ccma(res);
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

std::vector<double> AUTOCONTROL::calc_ref_v(const std::vector<Eigen::Matrix4d>& src, double st_v)
{
    std::vector<double> curvatures(src.size(), 0);
    for(size_t p = 0; p < src.size()-1; p++)
    {
        Eigen::Matrix3d R0 = src[p].block(0,0,3,3);
        Eigen::Matrix3d R1 = src[p+1].block(0,0,3,3);
        Eigen::Matrix3d dR = R0.transpose()*R1;
        Eigen::AngleAxisd angle_axis(dR);
        Eigen::Vector3d rot_vec = angle_axis.angle()*angle_axis.axis();
        double kappa = rot_vec.norm();
        curvatures[p] = kappa;
    }
    curvatures = gaussian_filter(curvatures, 10, 1.5);

    std::vector<double> res(src.size(), params.LIMIT_V);
    for(size_t p = 0; p < curvatures.size(); p++)
    {
        double kappa = curvatures[p];
        double scale_factor = saturation(1.0 - kappa*config->DRIVE_H, 0.0, 1.0);
        double adjusted_v = params.LIMIT_V * scale_factor;
        if(adjusted_v < params.MIN_V)
        {
            adjusted_v = params.MIN_V;
        }

        res[p] = adjusted_v;        
    }

    res.front() = st_v;
    return res;
}

std::vector<double> AUTOCONTROL::smoothing_v(const std::vector<double>& src, double path_step)
{
    const double v_limit = params.LIMIT_V;
    const double v_acc = params.LIMIT_V_ACC;

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
    for(double t = 0; t < predict_t; t += dt)
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

        //Eigen::Matrix4d G = ZYX_to_TF(pre_x, pre_y, 0, 0, 0, pre_th);
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

    // check boundary
    if(x < unimap->map_min_x || x > unimap->map_max_x || y < unimap->map_min_y || y > unimap->map_max_y)
    {
        return false;
    }

    // check collision
    Eigen::Matrix4d robot_tf = se2_to_TF(Eigen::Vector3d(x,y,th));
    if(obsmap->is_collision(obs_map, obs_tf, robot_tf, avoid_area))
    {
        // is collision        
        return false;
    }

    // no collision    
    return true;
}

bool AUTOCONTROL::is_path_valid(std::vector<Eigen::Matrix4d>& path, bool first_pivot)
{
    // get current obs map
    cv::Mat _obs_map;
    Eigen::Matrix4d _obs_tf;
    obsmap->get_obs_map(_obs_map, _obs_tf);
    cv::Mat _avoid_area = cv::Mat(obsmap->h, obsmap->w, CV_8U, cv::Scalar(255));

    if(first_pivot)
    {
        if(obsmap->is_pivot_collision(_obs_map, _obs_tf, path.front(), _avoid_area))
        {
            // is collision
            return false;
        }
    }

    for(size_t p = 0; p < path.size(); p++)
    {
        if(obsmap->is_collision(_obs_map, _obs_tf, path[p], _avoid_area))
        {
            // is collision
            return false;
        }
    }

    // no collision
    return true;
}

int AUTOCONTROL::get_valid_idx(cv::Mat& _obs_map, Eigen::Matrix4d& _obs_tf, cv::Mat& _avoid_area, std::vector<Eigen::Matrix4d>& path, int st_idx)
{
    for(int p = st_idx; p < (int)path.size(); p++)
    {
        if(obsmap->is_collision(_obs_map, _obs_tf, path[p], _avoid_area) == false)
        {
            return p;
        }
    }

    return st_idx;
}

void AUTOCONTROL::a_loop()
{
    // debug message off
    ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

    // local path calculation loop    
    const double calc_time = 0.5;
    const double turn_radius = 0.001;

    int pre_local_goal_idx = 0;

    printf("[AUTO] a_loop start\n");
    while(a_flag)
    {
        if(use_local_path == true)
        {
            PATH global_path = get_cur_global_path();
            if(global_path.pos.size() > 0)
            {
                // get obs map
                obsmap->get_obs_map(obs_map, obs_tf);

                // hidden margin
                cv::dilate(obs_map, obs_map, cv::Mat());

                // set avoid area
                const int radius = std::ceil(config->ROBOT_RADIUS/obsmap->gs);
                const int diameter = 2*radius + 1;
                cv::Mat _avoid_area = cv::Mat(obsmap->h, obsmap->w, CV_8U, cv::Scalar(0));
                Eigen::Matrix4d inv_obs_tf = obs_tf.inverse();
                for(size_t p = 0; p < global_path.pos.size()-1; p++)
                {
                    Eigen::Vector3d P0 = global_path.pos[p];
                    Eigen::Vector3d P1 = global_path.pos[p+1];

                    Eigen::Vector3d _P0 = inv_obs_tf.block(0,0,3,3)*P0 + inv_obs_tf.block(0,3,3,1);
                    Eigen::Vector3d _P1 = inv_obs_tf.block(0,0,3,3)*P1 + inv_obs_tf.block(0,3,3,1);

                    cv::Vec2i uv0 = obsmap->xy_uv(_P0[0], _P0[1]);
                    cv::Vec2i uv1 = obsmap->xy_uv(_P1[0], _P1[1]);
                    cv::line(_avoid_area, cv::Point(uv0[0], uv0[1]), cv::Point(uv1[0], uv1[1]), cv::Scalar(255), diameter);

                    if(p == 0)
                    {
                        cv::circle(_avoid_area, cv::Point(uv0[0], uv0[1]), diameter, cv::Scalar(255), -1);
                    }

                    if(p == global_path.pos.size()-1)
                    {
                        cv::circle(_avoid_area, cv::Point(uv1[0], uv1[1]), diameter, cv::Scalar(255), -1);
                    }
                }

                cv::Mat _avoid_area2 = cv::Mat(obsmap->h, obsmap->w, CV_8U, cv::Scalar(0));
                for(int i = 0; i < obsmap->h; i++)
                {
                    for(int j = 0; j < obsmap->w; j++)
                    {
                        if(obs_map.ptr<uchar>(i)[j] == 255 && _avoid_area.ptr<uchar>(i)[j] == 255)
                        {
                            cv::circle(_avoid_area2, cv::Point(j,i), diameter, cv::Scalar(255), -1);
                        }
                    }
                }
                cv::bitwise_or(_avoid_area, _avoid_area2, avoid_area);
                cv::dilate(avoid_area, avoid_area, cv::Mat());

                // get current status
                Eigen::Matrix4d cur_tf = slam->get_cur_tf();
                Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
                Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);                
                int cur_idx = get_nn_idx(global_path.pos, cur_pos);

                // get local goal idx
                int _local_goal_idx = std::min<int>(cur_idx + config->OBS_TARGET_DIST1/GLOBAL_PATH_STEP, (int)global_path.pose.size()-1);
                int local_goal_idx = get_valid_idx(obs_map, obs_tf, avoid_area, global_path.pose, _local_goal_idx);
                if(local_goal_idx < pre_local_goal_idx)
                {
                    local_goal_idx = pre_local_goal_idx;
                }
                pre_local_goal_idx = local_goal_idx;

                // get local goal
                Eigen::Matrix4d local_goal_tf = global_path.pose[local_goal_idx];
                Eigen::Vector3d local_goal_xi = TF_to_se2(local_goal_tf);
                Eigen::Vector3d local_goal_pos = local_goal_tf.block(0,3,3,1);

                // update for plot
                mtx.lock();
                last_local_goal = local_goal_pos;
                mtx.unlock();

                PATH pre_local_path = get_cur_local_path();
                if(pre_local_path.pos.size() == 0)
                {
                    // set state space
                    auto stateSpace(std::make_shared<ompl::base::DubinsStateSpace>(turn_radius, false));

                    ompl::base::RealVectorBounds stateBounds(2);
                    stateBounds.setLow(0, cur_pos[0] - config->OBS_MAP_RANGE); stateBounds.setHigh(0, cur_pos[0] + config->OBS_MAP_RANGE);
                    stateBounds.setLow(1, cur_pos[1] - config->OBS_MAP_RANGE); stateBounds.setHigh(1, cur_pos[1] + config->OBS_MAP_RANGE);
                    stateSpace->setBounds(stateBounds);

                    ompl::geometric::SimpleSetup ss(stateSpace);

                    // validator
                    ss.setStateValidityChecker(std::bind(&AUTOCONTROL::is_state_valid, this, std::placeholders::_1));

                    // set start pose
                    ompl::base::ScopedState<ompl::base::DubinsStateSpace> start(ss.getStateSpace());
                    start->setX(cur_xi[0]);
                    start->setY(cur_xi[1]);
                    start->setYaw(cur_xi[2]);

                    // set end pose
                    ompl::base::ScopedState<ompl::base::DubinsStateSpace> end(ss.getStateSpace());
                    end->setX(local_goal_xi[0]);
                    end->setY(local_goal_xi[1]);
                    end->setYaw(local_goal_xi[2]);

                    ss.setStartAndGoalStates(start, end);

                    auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation());
                    ss.getProblemDefinition()->setOptimizationObjective(objective);

                    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
                    ss.setPlanner(planner);

                    // solve
                    ompl::base::PlannerStatus solved = ss.solve(calc_time);                    
                    if(solved)
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
                        std::vector<Eigen::Matrix4d> path_pose;
                        for(size_t p = 0; p < path_pos.size()-1; p++)
                        {
                            Eigen::Matrix4d tf = calc_tf(path_pos[p], path_pos[p+1]);
                            path_pose.push_back(tf);
                        }
                        path_pose.push_back(se2_to_TF(local_goal_xi));

                        // set ref_v
                        std::vector<double> ref_v = calc_ref_v(path_pose, params.MIN_V);

                        // smoothing ref_v
                        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

                        // set result
                        PATH res;
                        res.pose = path_pose;
                        res.pos = path_pos;
                        res.ref_v = ref_v;
                        res.goal_tf = local_goal_tf;

                        // update local path
                        mtx.lock();
                        cur_local_path = res;
                        mtx.unlock();

                        //printf("[AUTO] solution found, first time\n");
                    }
                }
                else
                {
                    // get replan point from pre_local_path
                    int _cur_idx = get_nn_idx(pre_local_path.pos, cur_pos);
                    int replan_st_idx = obsmap->get_conflict_idx(obs_map, obs_tf, pre_local_path.pose, avoid_area, _cur_idx);
                    Eigen::Vector3d replan_st_xi = TF_to_se2(pre_local_path.pose[replan_st_idx]);
                    //printf("[AUTO] _cur_idx:%d, replan_idx:%d, pre_path_num:%d\n", _cur_idx, replan_st_idx, (int)pre_local_path.pos.size());

                    // for plot
                    mtx.lock();
                    last_replan_st = pre_local_path.pos[replan_st_idx];
                    mtx.unlock();

                    // set state space
                    auto stateSpace(std::make_shared<ompl::base::DubinsStateSpace>(turn_radius, false));
                    ompl::base::RealVectorBounds stateBounds(2);
                    stateBounds.setLow(0, cur_pos[0] - config->OBS_MAP_RANGE); stateBounds.setHigh(0, cur_pos[0] + config->OBS_MAP_RANGE);
                    stateBounds.setLow(1, cur_pos[1] - config->OBS_MAP_RANGE); stateBounds.setHigh(1, cur_pos[1] + config->OBS_MAP_RANGE);
                    stateSpace->setBounds(stateBounds);

                    // set ss
                    ompl::geometric::SimpleSetup ss(stateSpace);

                    // validator
                    ss.setStateValidityChecker(std::bind(&AUTOCONTROL::is_state_valid, this, std::placeholders::_1));

                    // set start pose
                    ompl::base::ScopedState<ompl::base::DubinsStateSpace> start(ss.getStateSpace());
                    start->setX(replan_st_xi[0]);
                    start->setY(replan_st_xi[1]);
                    start->setYaw(replan_st_xi[2]);

                    // set end pose
                    ompl::base::ScopedState<ompl::base::DubinsStateSpace> end(ss.getStateSpace());
                    end->setX(local_goal_xi[0]);
                    end->setY(local_goal_xi[1]);
                    end->setYaw(local_goal_xi[2]);

                    ss.setStartAndGoalStates(start, end);

                    auto objective = std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation());
                    ss.getProblemDefinition()->setOptimizationObjective(objective);

                    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
                    ss.setPlanner(planner);

                    // solve
                    ompl::base::PlannerStatus solved = ss.solve(calc_time);
                    if(solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
                    {
                        // for merge path
                        std::vector<Eigen::Vector3d> _path_pos;
                        for(int p = _cur_idx; p < replan_st_idx; p++)
                        {
                            _path_pos.push_back(pre_local_path.pos[p]);
                        }

                        // solution interpolation
                        ss.simplifySolution();

                        auto path = ss.getSolutionPath();
                        ss.getPathSimplifier()->simplifyMax(path);
                        if(path.getStateCount() >= 2)
                        {
                            double sum_d = 0;
                            for(size_t p = 0; p < path.getStateCount()-1; p++)
                            {
                                const ompl::base::SE2StateSpace::StateType *state0 = path.getState(p)->as<ompl::base::SE2StateSpace::StateType>();
                                const ompl::base::SE2StateSpace::StateType *state1 = path.getState(p+1)->as<ompl::base::SE2StateSpace::StateType>();

                                double dx = state1->getX() - state0->getX();
                                double dy = state1->getY() - state0->getY();
                                double d = std::sqrt(dx*dx + dy*dy);
                                sum_d += d;
                            }
                            path.interpolate(sum_d/LOCAL_PATH_STEP);
                        }
                        ss.getPathSimplifier()->smoothBSpline(path);

                        // set pos
                        for(size_t p = 0; p < path.getStateCount(); p++)
                        {
                            const ompl::base::DubinsStateSpace::StateType *state = path.getState(p)->as<ompl::base::DubinsStateSpace::StateType>();
                            _path_pos.push_back(Eigen::Vector3d(state->getX(), state->getY(), 0));
                        }

                        // resampling
                        std::vector<Eigen::Vector3d> path_pos = sample_and_interpolation(_path_pos, GLOBAL_PATH_STEP, LOCAL_PATH_STEP);
                        std::vector<Eigen::Matrix4d> path_pose;
                        for(size_t p = 0; p < path_pos.size()-1; p++)
                        {
                            Eigen::Matrix4d tf = calc_tf(path_pos[p], path_pos[p+1]);
                            path_pose.push_back(tf);
                        }
                        path_pose.push_back(se2_to_TF(local_goal_xi));

                        // set ref_v
                        std::vector<double> ref_v = calc_ref_v(path_pose, pre_local_path.ref_v[_cur_idx]);

                        // smoothing ref_v
                        ref_v = smoothing_v(ref_v, LOCAL_PATH_STEP);

                        // set result
                        PATH res;
                        res.pose = path_pose;
                        res.pos = path_pos;
                        res.ref_v = ref_v;
                        res.goal_tf = local_goal_tf;

                        // update
                        mtx.lock();
                        cur_local_path = res;
                        mtx.unlock();

                        //printf("[AUTO] solution found\n");
                    }
                }
            }
        }
        else
        {
            // clear local path
            mtx.lock();
            cur_local_path = PATH();
            mtx.unlock();

            pre_local_goal_idx = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[AUTO] a_loop stop\n");
}

// check condition
bool AUTOCONTROL::is_everything_fine()
{
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

// loops
void AUTOCONTROL::b_loop_pp()
{
    // set params
    is_moving = true;
    use_local_path = true;

    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();
    double extend_dt = 0;

    PATH global_path = get_cur_global_path();
    int fsm_state = AUTO_FSM_FIRST_ALIGN;

    printf("[AUTO] b_loop_pp start\n");
    while(b_flag)
    {
        // check
        if(is_everything_fine() == false)
        {
            mobile->move(0, 0, 0);
            is_moving = false;
            use_local_path = false;

            printf("[AUTO] something wrong\n");
            break;
        }

        // get current local path
        PATH local_path = get_cur_local_path();
        if(local_path.pos.size() == 0)
        {
            printf("[AUTO] wait local path\n");

            pre_loop_time = get_time();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // get current status
        Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
        Eigen::Matrix4d cur_tf = slam->get_cur_tf();
        Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
        Eigen::Vector3d cur_pos = cur_tf.block(0,3,3,1);

        // finite state machine
        if(fsm_state == AUTO_FSM_FIRST_ALIGN)
        {
            // find tgt
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = cur_idx;
            for(int p = cur_idx; p < (int)local_path.pos.size(); p++)
            {
                double d = (local_path.pos[p] - local_path.pos[cur_idx]).norm();
                if(d > params.MIN_LD)
                {
                    tgt_idx = p;
                    break;
                }
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
            double kd = 0.1;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*w0;
            w = saturation(w, w0 - params.LIMIT_W_ACC*D2R*dt, w0 + params.LIMIT_W_ACC*D2R*dt);
            w = saturation(w, -params.LIMIT_PIVOT_W*D2R, params.LIMIT_PIVOT_W*D2R);

            // goal check
            if(std::abs(err_th) < config->DRIVE_GOAL_TH*D2R)
            {
                mobile->move(0, 0, 0);

                fsm_state = AUTO_FSM_DRIVING;
                pre_loop_time = get_time();

                printf("[AUTO] FIRST_ALIGN -> DRIVING, err_th:%f\n", err_th*R2D);
                continue;
            }

            // send control
            mobile->move(0, 0, w);
        }
        else if(fsm_state == AUTO_FSM_DRIVING)
        {
            // find tgt            
            int cur_idx = get_nn_idx(local_path.pos, cur_pos);
            int tgt_idx = local_path.pos.size()-1;
            for(int p = cur_idx; p < (int)local_path.pos.size(); p++)
            {
                double d = (local_path.pos[p] - local_path.pos[cur_idx]).norm();
                if(d > params.MIN_LD)
                {
                    tgt_idx = p;
                    break;
                }
            }

            Eigen::Matrix4d tgt_tf = local_path.pose[tgt_idx];
            Eigen::Vector3d tgt_xi = TF_to_se2(tgt_tf);
            Eigen::Vector3d tgt_pos = tgt_tf.block(0,3,3,1);

            mtx.lock();
            last_tgt_pos = tgt_pos;
            mtx.unlock();

            // obs_v
            double obs_v = params.LIMIT_V;
            for(double vv = 0; vv <= params.LIMIT_V; vv += 0.1)
            {
                std::vector<Eigen::Matrix4d> traj = calc_trajectory(Eigen::Vector3d(vv, 0, 0), 0.1, 1.0, cur_tf);
                if(obsmap->is_collision(traj))
                {
                    obs_v = vv;
                    break;
                }
            }

            // calc ref_v            
            double goal_d = (global_path.goal_tf.block(0,3,3,1) - cur_pos).norm();

            double goal_v = goal_d;
            double path_v = local_path.ref_v[cur_idx];
            double ref_v = std::min<double>({path_v, obs_v, goal_v});

            // calc error theta
            double err_th = deltaRad(tgt_xi[2], cur_xi[2]);

            // calc cross track error
            double ld = (tgt_xi.block(0,0,2,1) - cur_xi.block(0,0,2,1)).norm();
            Eigen::Vector3d front_pos = cur_tf.block(0,0,3,3)*Eigen::Vector3d(ld, 0, 0) + cur_tf.block(0,3,3,1);
            double s = check_lr(tgt_xi[0], tgt_xi[1], tgt_xi[2], front_pos[0], front_pos[1]);
            double cte = -std::copysign(std::sqrt(std::pow(tgt_xi[0] - front_pos[0], 2) + std::pow(tgt_xi[1] - front_pos[1], 2)), s);

            // calc control input
            double v0 = cur_vel[0];
            double v = ref_v;
            v = saturation(v, v0 - 2.0*params.LIMIT_V_ACC*dt, v0 + params.LIMIT_V_ACC*dt);
            v = saturation(v, 0, params.LIMIT_V);

            double th = err_th + std::atan2(config->DRIVE_K * cte, v + config->DRIVE_EPS);
            th = saturation(th, -90.0*D2R, 90.0*D2R);

            double w = (v/config->ROBOT_WHEEL_BASE) * std::tan(th);            
            if(std::abs(w) > params.LIMIT_W*D2R)
            {
                double ratio = (params.LIMIT_W*D2R)/std::abs(w);
                w = (params.LIMIT_W*D2R) * (w/std::abs(w));
                v = v * ratio;
            }            

            // goal check
            if(goal_d < config->DRIVE_GOAL_D)
            {
                extend_dt += dt;
                printf("v:%f, w:%f\n", v, w);
                if(extend_dt > config->DRIVE_EXTENDED_CONTROL_TIME)
                {
                    extend_dt = 0;
                    mobile->move(0, 0, 0);

                    fsm_state = AUTO_FSM_FINAL_ALIGN;
                    pre_loop_time = get_time();

                    printf("[AUTO] DRIVING -> FINAL_ALIGN, goal_d:%f\n", goal_d);
                    continue;
                }
            }

            // send control
            mobile->move(v, 0, w);            
        }
        else if(fsm_state == AUTO_FSM_FINAL_ALIGN)
        {
            // get goal state
            Eigen::Vector3d goal_xi = TF_to_se2(global_path.goal_tf);

            // calc error
            double err_th = deltaRad(goal_xi[2], cur_xi[2]);

            // pivot control
            double kp = 1.5;
            double kd = 0.1;
            double w0 = cur_vel[2];
            double w = kp*err_th + kd*w0;
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
                    use_local_path = false;

                    fsm_state = AUTO_FSM_COMPLETE;
                    printf("[AUTO] FINAL ALIGN COMPLETE, err_th: %.3f\n", err_th*R2D);
                    break;
                }
            }

            // send control
            mobile->move(0, 0, w);
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
    use_local_path = false;
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

    printf("[AUTO] b_loop_tng start\n");
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
    printf("[AUTO] b_loop_tng stop\n");
}

void AUTOCONTROL::b_loop_pivot()
{
    is_moving = true;

    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    printf("[AUTO] b_loop_pivot start\n");
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
    printf("[AUTO] b_loop_pivot stop\n");
}


