#include "localization.h"

LOCALIZATION::LOCALIZATION(QObject *parent) : QObject{parent}
{
    cur_tf.setIdentity();
}

LOCALIZATION::~LOCALIZATION()
{
    // stop loc loops
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void LOCALIZATION::start()
{
    if(is_loc)
    {
        printf("[LOCALIZATION] already running\n");
        return;
    }

    // set flag
    is_loc = true;

    if(config->LOC_MODE == "2D")
    {
        if(a_thread == NULL)
        {
            a_flag = true;
            a_thread = new std::thread(&LOCALIZATION::a_loop_2d, this);
        }
    }
    else if(config->LOC_MODE == "3D")
    {
        if(a_thread == NULL)
        {
            a_flag = true;
            a_thread = new std::thread(&LOCALIZATION::a_loop_3d, this);
        }
    }
    else
    {
        printf("[LOCALIZATION] unknown LOC_MODE: %s\n", config->LOC_MODE.toStdString().c_str());
        is_loc = false;
    }

    if(b_thread == NULL)
    {
        b_flag = true;
        b_thread = new std::thread(&LOCALIZATION::b_loop, this);
    }

    if(obs_thread == NULL)
    {
        obs_flag = true;
        obs_thread = new std::thread(&LOCALIZATION::obs_loop, this);
    }

    printf("[LOCALIZATION(%s)] start\n", config->LOC_MODE.toStdString().c_str());
}

void LOCALIZATION::stop()
{
    is_loc = false;

    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }

    printf("[LOCALIZATION(%s)] stop\n", config->LOC_MODE.toStdString().c_str());
}

void LOCALIZATION::set_cur_tf(Eigen::Matrix4d tf)
{
    mtx.lock();
    cur_tf = tf;
    mtx.unlock();
}

Eigen::Matrix4d LOCALIZATION::get_cur_tf()
{
    mtx.lock();
    Eigen::Matrix4d res = cur_tf;
    mtx.unlock();

    return res;
}

Eigen::Vector2d LOCALIZATION::get_cur_ieir()
{
    mtx.lock();
    Eigen::Vector2d res = cur_ieir;
    mtx.unlock();

    return res;
}

QString LOCALIZATION::get_cur_loc_state()
{
    mtx.lock();
    QString res = cur_loc_state;
    mtx.unlock();

    return res;
}

void LOCALIZATION::set_cur_loc_state(QString str)
{
    mtx.lock();
    cur_loc_state = str;
    mtx.unlock();
}

QString LOCALIZATION::get_info_text()
{
    Eigen::Vector2d cur_ieir = get_cur_ieir();
    QString loc_state = get_cur_loc_state();
    Eigen::Vector3d cur_xi = TF_to_se2(get_cur_tf());


    QString str;
    str.sprintf("[LOC]\npos:%.2f,%.2f,%.2f\nerr:%f\nloc_t: %.3f, %.3f, %.3f\nieir: (%.3f, %.3f)\nloc_state: %s",
                cur_xi[0], cur_xi[1], cur_xi[2]*R2D,
                (double)cur_tf_err, (double)proc_time_loc_a, (double)proc_time_loc_b, (double)proc_time_obs,
                cur_ieir[0], cur_ieir[1],
                loc_state.toLocal8Bit().data());
    return str;
}

Eigen::Vector2d LOCALIZATION::calc_ieir(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G)
{
    if(pts.size() < 100)
    {
        return Eigen::Vector2d(1.0, 0);
    }

    double err_sum = 0;
    int cnt = 0;
    int cnt_all = 0;
    for(size_t p = 0; p < pts.size(); p += 8)
    {
        cnt_all++;

        Eigen::Vector3d P = pts[p];
        Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
        std::vector<int> nn_idxs = unimap->knn_search_idx(_P, 1, config->LOC_SURFEL_RANGE);
        if(nn_idxs.size() == 0)
        {
            continue;
        }

        Eigen::Vector3d nn_pt = unimap->map_pts[nn_idxs[0]];
        double d = (nn_pt - _P).norm();
        if(d < config->LOC_INLIER_CHECK_DIST)
        {
            err_sum += d;
            cnt++;
        }
    }

    Eigen::Vector2d res;
    res[0] = err_sum/cnt;
    res[1] = (double)cnt/cnt_all;
    return res;
}

void LOCALIZATION::a_loop_2d()
{
    printf("[LOCALIZATION(2D)] a_loop start\n");
    while(a_flag)
    {

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LOCALIZATION(2D)] a_loop stop\n");
}

void LOCALIZATION::a_loop_3d()
{
    IMU pre_imu;
    Eigen::Matrix4d _cur_tf = get_cur_tf();
    Eigen::Matrix4d _pre_tf = _cur_tf;

    // lidar localization (10hz)
    lidar_3d->merged_que.clear();

    printf("[LOCALIZATION(3D)] a_loop start\n");
    while(a_flag)
    {
        TIME_PTS frm;
        if(lidar_3d->merged_que.try_pop(frm))
        {
            if(unimap->is_loaded != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            double st_time = get_time();

            // initial guess
            // Eigen::Vector3d delta_xi = TF_to_se2(_pre_tf.inverse()*_cur_tf);
            // Eigen::Matrix4d G = _cur_tf*ZYX_to_TF(delta_xi[0], delta_xi[1], 0, 0, 0, delta_xi[2]);
            // G = _cur_tf;
            Eigen::Matrix4d G = _cur_tf;

            // imu
            IMU cur_imu = lidar_3d->get_best_imu(frm.t, 0);
            if(pre_imu.t != 0)
            {
                Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(pre_imu.rx, pre_imu.ry, pre_imu.rz)).matrix();
                Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(cur_imu.rx, cur_imu.ry, cur_imu.rz)).matrix();
                G.block(0,0,3,3) = G.block(0,0,3,3)*(R0.inverse()*R1);
            }

            // icp
            std::vector<Eigen::Vector3d> dsk = frm.pts;
            double err = map_icp(dsk, G);

            // check ieir
            cur_ieir = calc_ieir(dsk, G);

            // check error
            if(err < config->LOC_ICP_ERROR_THRESHOLD)
            {
                // for loc b loop
                TIME_POSE tp;
                tp.t = frm.t;
                tp.tf = _cur_tf;
                tp_que.push(tp);

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = _cur_tf;
                tpp.pts = dsk;
                tpp_que.push(tpp);

                // update
                set_cur_tf(G);

                // local to global deskewed point
                std::vector<Eigen::Vector3d> pts(dsk.size());
                for(size_t p = 0; p < dsk.size(); p++)
                {
                    Eigen::Vector3d P(dsk[p][0], dsk[p][1], dsk[p][2]);
                    Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                    pts[p] = _P;
                }
                plot_cur_pts_que.push(pts);

                // for next
                _pre_tf = _cur_tf;
                _cur_tf = G;
                pre_imu = cur_imu;
            }

            // set localization info for plot
            cur_tf_err = err;
            proc_time_loc_a = get_time() - st_time;


            // for speed
            lidar_3d->merged_que.clear();

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LOCALIZATION(3D)] a_loop stop\n");
}

void LOCALIZATION::b_loop()
{
    // loop params
    const double dt = 0.02;
    double pre_loop_time = get_time();

    // clear storage
    mtx.lock();
    tp_storage.clear();
    mtx.unlock();

    // for aruco fusion
    double pre_aruco_t = 0;
    std::vector<std::pair<Eigen::Matrix4d, Eigen::Matrix4d>> tf_storage;

    // params
    MOBILE_POSE pre_mo = mobile->get_pose();

    printf("[LOCALIZATION] b_loop start\n");
    while(b_flag)
    {
        MOBILE_POSE cur_mo = mobile->get_pose();
        if(cur_mo.t > pre_mo.t && config->LOC_ICP_ODO_FUSION_RATIO != 0.0)
        {
            double st_time = get_time();

            Eigen::Matrix4d pre_mo_tf = se2_to_TF(pre_mo.pose);
            Eigen::Matrix4d cur_mo_tf = se2_to_TF(cur_mo.pose);

            // get current tf
            Eigen::Matrix4d _cur_tf = get_cur_tf();

            // update odo_tf
            Eigen::Matrix4d delta_tf = pre_mo_tf.inverse()*cur_mo_tf;
            Eigen::Matrix4d odo_tf = _cur_tf*delta_tf;
            pre_mo_tf = cur_mo_tf;

            // icp-odometry fusion
            TIME_POSE tp;
            if(tp_que.try_pop(tp) && !config->USE_SIM)
            {
                // time delay compensation
                Eigen::Matrix4d tf0 = se2_to_TF(mobile->get_best_mo(tp.t).pose);
                Eigen::Matrix4d tf1 = se2_to_TF(cur_mo.pose);
                Eigen::Matrix4d mo_dtf = tf0.inverse()*tf1;
                Eigen::Matrix4d icp_tf = tp.tf * mo_dtf;

                double alpha = config->LOC_ICP_ODO_FUSION_RATIO; // 1.0 means odo_tf 100%

                // for odometry slip
                Eigen::Vector2d dtdr = dTdR(odo_tf, icp_tf);
                if(std::abs(dtdr[1]) > 10.0*D2R)
                {
                    alpha = 0;
                    printf("[LOCALIZATION] slip detection, alpha set 0, dth: %f\n", dtdr[1]*R2D);
                }

                // interpolation
                Eigen::Matrix4d dtf = icp_tf.inverse()*odo_tf;
                Eigen::Matrix4d fused_tf = icp_tf*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                // update
                _cur_tf = fused_tf;
            }
            else
            {
                // update
                _cur_tf = odo_tf;
            }

            /*
            // aruco fusion
            TIME_POSE_ID aruco_tpi = aruco->get_cur_tpi();
            if(aruco_tpi.t > pre_aruco_t && config->USE_SIM == 0)
            {
                double d = calc_dist_2d(aruco_tpi.tf.block(0,3,3,1));
                if(d < config->LOC_ARUCO_ODO_FUSION_DIST)
                {
                    NODE* node = unimap->get_node_by_name(QString::number(aruco_tpi.id, 10));
                    if(node != NULL)
                    {
                        // parse tf
                        Eigen::Matrix4d T_g_m0 = node->tf; // stored global marker tf
                        Eigen::Matrix4d T_m_r = aruco_tpi.tf.inverse();

                        // time delay compensation
                        Eigen::Matrix4d tf0 = se2_to_TF(mobile->get_best_mo(aruco_tpi.t).pose);
                        Eigen::Matrix4d tf1 = se2_to_TF(cur_mo.pose);
                        Eigen::Matrix4d mo_dtf = tf0.inverse()*tf1;

                        Eigen::Matrix4d T_g_r = T_g_m0*T_m_r*mo_dtf;
                        Eigen::Matrix4d aruco_tf = se2_to_TF(TF_to_se2(T_g_r));

                        // interpolation
                        double alpha = config->LOC_ARUCO_ODO_FUSION_RATIO; // 1.0 mean odometry only
                        if(std::abs(cur_mo.vel[2]) > 10.0*D2R)
                        {
                            alpha = 1.0; // odometry only
                        }

                        Eigen::Matrix4d dtf = aruco_tf.inverse()*_cur_tf;
                        Eigen::Matrix4d fused_tf = aruco_tf*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                        // update
                        _cur_tf = fused_tf;
                        pre_aruco_t = aruco_tpi.t;
                    }
                }
            }
            */

            // update
            set_cur_tf(_cur_tf);

            mtx.lock();
            TIME_POSE fused_tp;
            fused_tp.t = cur_mo.t;
            fused_tp.tf = _cur_tf;
            tp_storage.push_back(fused_tp);
            if(tp_storage.size() > 300)
            {
                tp_storage.erase(tp_storage.begin());
            }
            mtx.unlock();

            // for next operation
            pre_mo = cur_mo;

            // update processing time
            proc_time_loc_b = get_time() - st_time;
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
            printf("[LOCALIZATION] b_loop loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[LOCALIZATION] b_loop stop\n");
}

void LOCALIZATION::obs_loop()
{
    printf("[LOCALIZATION] obs_loop start\n");
    while(obs_flag)
    {
        double st_time = get_time();

        if(config->USE_SIM)
        {
            if(unimap->is_loaded == MAP_LOADED)
            {
                Eigen::Matrix4d _cur_tf = get_cur_tf();
                obsmap->update_obs_map_sim(_cur_tf);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        TIME_POSE_PTS tpp;
        if(tpp_que.try_pop(tpp))
        {
            if(tpp.pts.size() == 0)
            {
                continue;
            }

            // todo: add 2D or 3D pts

            // todo: add cam pts

            // update obstacle map
            obsmap->update_obs_map(tpp);

            // for speed
            tpp_que.clear();

            // update processing time
            proc_time_obs = get_time() - st_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[LOCALIZATION] obs_loop stop\n");
}

// 2D
double LOCALIZATION::map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        idx_list.push_back(p);

        Eigen::Vector3d P = G.block(0,0,3,3)*frm.pts[p] + G.block(0,3,3,1);
        pts.push_back(P);
    }

    // solution
    Eigen::Matrix4d _G = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = max_iter0;
    double lambda = lambda0;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = config->LOC_ICP_COST_THRESHOLD*config->LOC_ICP_COST_THRESHOLD;
    const int num_feature = std::min<int>(idx_list.size(), config->LOC_ICP_MAX_FEATURE_NUM);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;
        for(size_t p = 0; p < idx_list.size(); p++)
        {
            // get index
            int i = idx_list[p];
            double dist = calc_dist_2d(frm.pts[i]);
            if(dist < config->LIDAR_2D_MIN_RANGE)
            {
                continue;
            }

            // local to global
            Eigen::Vector3d P1 = pts[i];
            Eigen::Vector3d _P1 = _G.block(0,0,3,3)*P1 + _G.block(0,3,3,1);

            // knn points
            Eigen::Vector3d P0(0, 0, 0);
            {
                const int pt_num = config->LOC_SURFEL_NUM;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                int cnt = 0;
                int idx0 = ret_near_idxs[0];
                double sq_d_max = config->LOC_SURFEL_RANGE*config->LOC_SURFEL_RANGE;
                for(int q = 0; q < pt_num; q++)
                {
                    int idx = ret_near_idxs[q];
                    double dx = cloud.pts[idx].x - cloud.pts[idx0].x;
                    double dy = cloud.pts[idx].y - cloud.pts[idx0].y;
                    double sq_d = dx*dx + dy*dy;
                    if(sq_d > sq_d_max)
                    {
                        continue;
                    }

                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                    cnt++;
                }
                P0 /= cnt;
            }

            // rmt
            double rmt = 1.0;
            if(iter >= 1)
            {
                rmt = tm1/tm0;
                if(rmt > 1.0)
                {
                    rmt = 1.0;
                }
            }

            // point to point distance
            double cost = (_P1 - P0).squaredNorm();
            if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + rmt_sigma)
            {
                continue;
            }

            // jacobian
            Eigen::Vector3d xi = TF_to_se2(_G);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // additional weight
            double weight = 1.0 + 0.05*dist;

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = weight;
            memcpy(cj.J, J, sizeof(double)*3);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // check num
            if((int)cj_set.size() == num_feature)
            {
                break;
            }
        }

        // num of correspondence
        num_correspondence = cj_set.size();
        if(num_correspondence < 100)
        {
            printf("[map_icp] not enough correspondences, iter: %d, num: %d->%d!!\n", iter, (int)frm.pts.size(), num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size() / 2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for (size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }
        std::nth_element(vars.begin(), vars.begin() + vars.size() / 2, vars.end());
        double sigma = vars[vars.size() / 2];
        if(sigma < sigma_eps)
        {
            sigma = sigma_eps;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = t_dist_v0;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= w;
        }

        // make matrix
        double _A[3*3] = { 0, };
        double _b[3] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for (int y = 0; y < 3; y++)
            {
                for (int x = 0; x < 3; x++)
                {
                    _A[y * 3 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            err += std::abs(w*c);
            err_cnt += w;
        }
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 3, 3> A(_A);
        A += 1e-6*Eigen::Matrix<double, 3, 3>::Identity();

        Eigen::Matrix<double, 3, 1> b(_b);
        Eigen::Matrix<double, 3, 3> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 3, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        if(err < last_err)
        {
            lambda *= lambda_dec;
        }
        else
        {
            lambda *= lambda_inc;
        }
        last_err = err;

        // pose update
        Eigen::Vector3d xi;
        xi[0] = X(0, 0);
        xi[1] = X(1, 0);
        xi[2] = X(2, 0);

        _G = se2_to_TF(xi)*_G;
        refine_pose(_G);

        // for rmt
        tm0 = tm1;
        tm1 = _G.block(0,3,3,1).norm();

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    G = _G*G;

    if(last_err > first_err+0.01 || last_err > config->LOC_ICP_ERROR_THRESHOLD)
    {
        printf("[map_icp] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);
        //return 9999;
    }

    return last_err;
}


// 3D
double LOCALIZATION::map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    for(size_t p = 0; p < pts.size(); p++)
    {
        // set idxs
        idx_list.push_back(p);
    }

    // solution
    Eigen::Matrix4d _G = G;

    // optimization param
    const int max_iter = max_iter0;
    double lambda = lambda0;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    // loop
    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        // shuffle
        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());
        const int num_feature = std::min<int>(idx_list.size(), config->LOC_ICP_MAX_FEATURE_NUM);
        std::vector<int> surfel_cnt(3,0);

        // calc cost jacobian
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;
        for(size_t k = 0; k < idx_list.size(); k++)
        {
            // get index
            int i = idx_list[k];

            // local to global
            Eigen::Vector3d P = pts[i];
            Eigen::Vector3d P1 = _G.block(0,0,3,3)*P + _G.block(0,3,3,1);

            // calc correspondence
            const int _nn_num = 1; // config->LVX_SURFEL_NN_NUM;
            std::vector<int> nn_idxs = unimap->knn_search_idx(P1, _nn_num, config->LOC_SURFEL_RANGE);
            if((int)nn_idxs.size() != _nn_num)
            {
                continue;
            }

            // calc surfel
            Eigen::Vector3d N0(0,0,0);
            Eigen::Vector3d P0(0,0,0);
            for(size_t p = 0; p < nn_idxs.size(); p++)
            {
                int idx = nn_idxs[p];
                P0 += unimap->map_pts[idx];
                N0 += unimap->map_nor[idx];
            }

            P0 /= nn_idxs.size();
            N0 /= nn_idxs.size();
            N0.normalize();

            // balance filter
            int axis_idx = get_major_axis(N0);
            if(surfel_cnt[axis_idx] > num_feature*config->LOC_SURFEL_BALANCE)
            {
                continue;
            }

            // rmt
            double rmt = 1.0;
            if(iter >= 1)
            {
                rmt = tm1/tm0;
                if(rmt > 1.0)
                {
                    rmt = 1.0;
                }
            }

            // calc cost
            double cost = N0.dot(P1-P0);
            if(std::abs(cost) > config->LOC_COST_THRESHOLD || std::abs(cost) > rmt*std::abs(cost) + rmt_sigma)
            {
                continue;
            }

            // Compute Jacobian using Lie Algebra
            Eigen::Matrix<double, 3, 6> dP1_dxi; // Jacobian of SE(3) transformation
            dP1_dxi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); // Translation part
            dP1_dxi.block<3, 3>(0, 3) = -Sophus::SO3d::hat(P1);      // Rotation part

            // Cost Jacobian: N0^T * dP1/dxi
            Eigen::Matrix<double, 1, 6> jacobian = N0.transpose() * dP1_dxi;
            if(!jacobian.allFinite())
            {
                continue;
            }

            double J[6];
            J[0]  = jacobian(0,0);
            J[1]  = jacobian(0,1);
            J[2]  = jacobian(0,2);
            J[3]  = jacobian(0,3);
            J[4]  = jacobian(0,4);
            J[5]  = jacobian(0,5);

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = 1.0;
            memcpy(cj.J, J, sizeof(double)*12);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // update surfel count
            surfel_cnt[axis_idx]++;

            // check num
            if((int)cj_set.size() == num_feature)
            {
                break;
            }
        }

        // num of correspondence
        num_correspondence = cj_set.size();
        if(num_correspondence < 100)
        {
            printf("[map_icp] not enough correspondences, iter: %d, num: %d!!\n", iter, num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size()/2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for(size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }

        std::nth_element(vars.begin(), vars.begin() + vars.size()/2, vars.end());
        double sigma = std::max(1.4826 * vars[vars.size()/2], sigma_eps);

        // calc weight
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = t_dist_v0;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= std::clamp(w, 0.01, 10.0);
        }

        // make matrix
        double _A[6*6] = { 0, };
        double _b[6] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for(int y = 0; y < 6; y++)
            {
                for(int x = 0; x < 6; x++)
                {
                    _A[y * 6 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            err += std::abs(c);
            err_cnt += w;
        }
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 6, 6> A(_A);
        A += 1e-6*Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> b(_b);
        Eigen::Matrix<double, 6, 6> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 6, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        double err_ratio = (last_err-err)/last_err;
        if(err_ratio > 0.1)
        {
            //lambda *= lambda_dec;
            lambda = std::max(lambda * lambda_dec, 1e-6);
        }
        else if(err_ratio < 0.01)
        {
            //lambda *= lambda_inc;
            lambda = std::min(lambda * lambda_inc, 1e6);
        }
        last_err = err;

        // pose update
        Sophus::Vector6d xi;
        xi[0] = X(0, 0); // tx
        xi[1] = X(1, 0); // ty
        xi[2] = X(2, 0); // tz
        xi[3] = X(3, 0); // rx
        xi[4] = X(4, 0); // ry
        xi[5] = X(5, 0); // rz

        _G = Sophus::SE3d::exp(xi).matrix()*_G;

        // for rmt
        tm0 = tm1;
        tm1 = (_G.block(0,3,3,1).norm() + _G.block(0,3,3,1).norm())/2;

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    G = _G;

    // for debug
    // printf("[SLAM_3D] map_icp, i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);

    return last_err;

}
