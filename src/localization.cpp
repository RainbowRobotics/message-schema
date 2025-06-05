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
            Eigen::Vector2d ieir = calc_ieir(dsk, G);

            // update
            cur_tf_t = frm.t;
            cur_tf_err = err;
            cur_tf_ie = ieir[0];
            cur_tf_ir = ieir[1];
            set_cur_tf(G);

            // for next
            _pre_tf = _cur_tf;
            _cur_tf = G;
            pre_imu = cur_imu;

            // local to global deskewed point
            std::vector<Eigen::Vector3d> pts(dsk.size());
            for(size_t p = 0; p < dsk.size(); p++)
            {
                Eigen::Vector3d P(dsk[p][0], dsk[p][1], dsk[p][2]);
                Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                pts[p] = _P;
            }
            plot_cur_pts_que.push(pts);

            // for speed
            lidar_3d->merged_que.clear();

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LOCALIZATION(3D)] a_loop stop\n");
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
