#include "mapping.h"

MAPPING::MAPPING(QObject *parent) : QObject{parent}
{
}

MAPPING::~MAPPING()
{
    stop();

    if(live_tree != NULL)
    {
        delete live_tree;
    }
}

void MAPPING::start()
{

    if(is_mapping == false)
    {
        printf("[MAPPING] start\n");

        // stop first
        stop();

        // clear objects
        clear();

        // start loop
        if(a_thread == NULL)
        {
            a_flag = true;
            a_thread = new std::thread(&MAPPING::a_loop, this);
        }

        if(b_thread == NULL)
        {
            b_flag = true;
            b_thread = new std::thread(&MAPPING::b_loop, this);
        }

        // set flag
        is_mapping = true;
    }
    else
    {
        printf("[MAPPING] already running\n");
    }
}

void MAPPING::stop()
{
    // stop loc
    loc->stop();

    // stop slam loops
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }

    if(b_thread != NULL)
    {
        b_flag = false;
        b_thread->join();
        b_thread = NULL;
    }

    is_mapping = false;
}

void MAPPING::clear()
{
    // clear storages
    mtx.lock();
    clear_pose_graph();

    kfrm_que.clear();
    kfrm_update_que.clear();
    kfrm_storage.clear();

    if(live_tree != NULL)
    {
        delete live_tree;
        live_tree = NULL;
        live_cloud.pts.clear();
    }
    live_tree = new KD_TREE_XYZR(3, live_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    pgo = PGO();
    mtx.unlock();

    loc->set_cur_tf(Eigen::Matrix4d::Identity());
}

void MAPPING::a_loop()
{
    const int window_size = config->SLAM_WINDOW_SIZE;
    const double voxel_size = config->SLAM_VOXEL_SIZE;
    const double do_cnt_check_range = 1.0*voxel_size;
    const int erase_gap = config->SLAM_ICP_DO_ERASE_GAP;
    const int accum_num = config->SLAM_ICP_DO_ACCUM_NUM;

    int frm_cnt = 0;
    int add_cnt = 0;
    int kfrm_id = 0;

    // last frame
    FRAME frm0;

    // using new lidar frame
    lidar_2d->merged_que.clear();

    printf("[MAPPING] a_loop start\n");
    while(a_flag)
    {
        FRAME frm;
        if(lidar_2d->merged_que.try_pop(frm))
        {
            // for processing time
            double st_time = get_time();

            if(frm_cnt == 0)
            {
                // get cur_tf
                Eigen::Matrix4d G = loc->get_cur_tf();

                // local to global
                std::vector<PT_XYZR> dsk;
                #pragma omp parallel for
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Vector3d _P = frm.pts[p];

                    if(config->PLATFORM_TYPE == "S100")
                    {
                        double dx = 1.0;
                        if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
                           _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                        {
                            continue;
                        }
                    }

                    Eigen::Vector3d P = G.block(0,0,3,3)*_P + G.block(0,3,3,1);
                    Eigen::Vector3d V = (P - G.block(0,3,3,1)).normalized();

                    PT_XYZR pt;
                    pt.x = P[0];
                    pt.y = P[1];
                    pt.z = P[2];
                    pt.vx = V[0];
                    pt.vy = V[1];
                    pt.vz = V[2];
                    pt.r = frm.reflects[p];
                    pt.k0 = add_cnt;
                    pt.k = add_cnt;
                    pt.do_cnt = 10;
                    dsk.push_back(pt);
                }

                // add tree
                mtx.lock();
                for(size_t p = 0; p < dsk.size(); p++)
                {
                    live_cloud.pts.push_back(dsk[p]);
                }
                live_tree->buildIndex();
                mtx.unlock();

                // update global tf
                loc->set_cur_tf(G);

                // update previous frame
                frm0 = frm;

                // update tree add count
                add_cnt++;
            }
            else
            {
                // get last global tf
                Eigen::Matrix4d G = loc->get_cur_tf();

                // initial guess
                Eigen::Matrix4d delta_tf = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm.mo.pose);
                G = G * delta_tf;

                // check moving
                double moving_d = calc_dist_2d(delta_tf.block(0,3,3,1));
                double moving_th = std::abs(TF_to_se2(delta_tf)[2]);
                if(moving_d > 0.05 || moving_th > 2.5*D2R)
                {
                    // pose estimation
                    double err = frm_icp(*live_tree, live_cloud, frm, G);
                    Eigen::Vector2d ieir = loc->calc_ieir(*live_tree, live_cloud, frm, G);
                    if(err < config->LOC_ICP_ERROR_THRESHOLD)
                    {
                        // local to global
                        std::vector<PT_XYZR> dsk;
                        #pragma omp parallel for
                        for(size_t p = 0; p < frm.pts.size(); p++)
                        {
                            Eigen::Vector3d _P = frm.pts[p];

                            if(config->PLATFORM_TYPE == "S100")
                            {
                                double dx = 1.0;
                                if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
                                   _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                                {
                                    continue;
                                }
                            }

                            Eigen::Vector3d P = G.block(0,0,3,3)*_P + G.block(0,3,3,1);
                            Eigen::Vector3d V = (P - G.block(0,3,3,1)).normalized();

                            PT_XYZR pt;
                            pt.x = P[0];
                            pt.y = P[1];
                            pt.z = P[2];
                            pt.vx = V[0];
                            pt.vy = V[1];
                            pt.vz = V[2];
                            pt.r = frm.reflects[p];
                            pt.k0 = add_cnt;
                            pt.k = add_cnt;
                            pt.do_cnt = 0;
                            dsk.push_back(pt);
                        }

                        // index sort by reflect
                        std::vector<size_t> dsk_idxs(dsk.size());
                        for(size_t p = 0; p < dsk_idxs.size(); p++)
                        {
                            dsk_idxs[p] = p;
                        }
                        std::sort(dsk_idxs.begin(), dsk_idxs.end(), [&dsk](size_t i1, size_t i2){ return dsk[i1].r > dsk[i2].r; });

                        // live_tree works
                        std::vector<int> dsk_mask(dsk.size(), 1); // 0: skip, 1: add
                        std::vector<int> live_cloud_mask(live_cloud.pts.size(), 1); // 0: del, 1: keep
                        for(size_t p = 0; p < dsk_idxs.size(); p++)
                        {
                            int i = dsk_idxs[p];
                            PT_XYZR& pt = dsk[i];

                            // dynamic object count up
                            {
                                double query_pt[3] = {pt.x, pt.y, pt.z};
                                double sq_radius = do_cnt_check_range*do_cnt_check_range;
                                std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;

                                nanoflann::SearchParameters params;
                                live_tree->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);
                                for(const auto& it : res_idxs)
                                {
                                    unsigned int idx = it.first;
                                    if(live_cloud.pts[idx].k < pt.k)
                                    {
                                        live_cloud.pts[idx].do_cnt++;
                                        live_cloud.pts[idx].k = pt.k;
                                    }
                                }
                            }

                            // check nn pt
                            {
                                // find nn point
                                std::vector<unsigned int> res_idxs(1);
                                std::vector<double> res_sq_dists(1);
                                double query_pt[3] = {pt.x, pt.y, pt.z};

                                live_tree->knnSearch(&query_pt[0], 1, &res_idxs[0], &res_sq_dists[0]);
                                if(std::sqrt(res_sq_dists[0]) < voxel_size)
                                {
                                    int nn_idx = res_idxs[0];
                                    if(live_cloud.pts[nn_idx].r < pt.r && live_cloud_mask[nn_idx] == 1)
                                    {
                                        pt.do_cnt = live_cloud.pts[nn_idx].do_cnt; // keep do_cnt
                                        pt.vx = live_cloud.pts[nn_idx].vx; // keep view vector
                                        pt.vy = live_cloud.pts[nn_idx].vy;
                                        pt.vz = live_cloud.pts[nn_idx].vz;

                                        live_cloud_mask[nn_idx] = 0; // del
                                    }
                                    else
                                    {
                                        dsk_mask[i] = 0; // skip
                                    }
                                }
                            }
                        }

                        // filtering old pt and dynamic object
                        int del_k = add_cnt - window_size;
                        for(size_t p = 0; p < live_cloud.pts.size(); p++)
                        {
                            if((live_cloud.pts[p].k <= add_cnt - erase_gap && live_cloud.pts[p].do_cnt < accum_num) ||
                                live_cloud.pts[p].k < del_k)
                            {
                                live_cloud_mask[p] = 0;
                            }
                        }

                        // collect valid pts
                        std::vector<PT_XYZR> pts;
                        for(size_t p = 0; p < live_cloud.pts.size(); p++)
                        {
                            if(live_cloud_mask[p] != 0)
                            {
                                pts.push_back(live_cloud.pts[p]);
                            }
                        }

                        for(size_t p = 0; p < dsk.size(); p++)
                        {
                            if(dsk_mask[p] != 0)
                            {
                                pts.push_back(dsk[p]);
                            }
                        }

                        // update tree
                        mtx.lock();
                        delete live_tree;
                        live_cloud.pts = pts;
                        live_tree = new KD_TREE_XYZR(3, live_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
                        live_tree->buildIndex();
                        mtx.unlock();

                        // update global tf
                        loc->set_cur_tf(G);

                        // update previous frame
                        frm0 = frm;

                        // update tree add count
                        add_cnt++;

                        // make keyframe
                        if(add_cnt % config->SLAM_KFRM_UPDATE_NUM == 0)
                        {
                            // set keyframe
                            KFRAME kfrm;
                            kfrm.id = kfrm_id;
                            kfrm.pts = live_cloud.pts;
                            kfrm.G = G;
                            kfrm.opt_G = G;
                            kfrm_que.push(kfrm);
                            printf("[MAPPING] keyframe created, id:%d\n", kfrm.id);

                            // inc keyframe id
                            kfrm_id++;
                        }

                        // update ieir
                        mtx.lock();
                        loc->cur_ieir = ieir;
                        mtx.unlock();
                    }
                    else
                    {
                        mtx.lock();
                        loc->cur_ieir[0] = err;
                        mtx.unlock();
                    }
                }
            }

            // update frame count
            frm_cnt++;

            // update processing time
            proc_time_map_a = get_time() - st_time;

            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MAPPING] a_loop stop\n");
}

void MAPPING::b_loop()
{
    printf("[MAPPING] b_loop start\n");
    while(b_flag)
    {
        KFRAME kfrm;
        if(kfrm_que.try_pop(kfrm))
        {
            double st_time = get_time();

            printf("[MAPPING] kfrm:%d processing start\n", kfrm.id);

            // keyframe pts transform global to kfrm local
            const int k0 = kfrm.id*config->SLAM_KFRM_UPDATE_NUM;
            const int k1 = kfrm.id*config->SLAM_KFRM_UPDATE_NUM + config->SLAM_KFRM_UPDATE_NUM;

            std::vector<PT_XYZR> pts;
            Eigen::Matrix4d G_inv = kfrm.G.inverse();
            for(size_t p = 0; p < kfrm.pts.size(); p++)
            {
                PT_XYZR pt = kfrm.pts[p];
                if(pt.k0 >= k0 && pt.k0 < k1)
                {
                    if(pt.do_cnt < config->SLAM_ICP_DO_ACCUM_NUM)
                    {
                        continue;
                    }

                    // global to keyframe local
                    Eigen::Vector3d P(pt.x, pt.y, pt.z);
                    Eigen::Vector3d _P = G_inv.block(0,0,3,3)*P + G_inv.block(0,3,3,1);

                    pt.x = _P[0];
                    pt.y = _P[1];
                    pt.z = _P[2];

                    pts.push_back(pt);
                }
            }

            // update
            kfrm.pts = pts;

            // make pose graph
            if(kfrm_storage.size() == 0)
            {
                pgo.add_node(kfrm.id, Eigen::Matrix4d::Identity(), kfrm.G, 0.1);
                kfrm_storage.push_back(kfrm);
                kfrm_update_que.push(kfrm.id);
            }
            else
            {
                Eigen::Matrix4d dG = kfrm_storage.back().G.inverse()*kfrm.G;
                kfrm.opt_G = kfrm_storage.back().opt_G*dG;

                pgo.add_node(kfrm.id, dG, kfrm.G, 0.1);
                kfrm_storage.push_back(kfrm);
                kfrm_update_que.push(kfrm.id);
            }

            // loop closing process
            if(kfrm_storage.size() >= 2)
            {
                // check loop closure
                for(size_t p = 0; p < kfrm_storage.size()-1; p++)
                {
                    int p0 = p;
                    int p1 = kfrm_storage.size()-1;

                    double d = (kfrm_storage[p1].opt_G.block(0,3,3,1) - kfrm_storage[p0].opt_G.block(0,3,3,1)).norm();
                    if(d < config->SLAM_KFRM_LC_TRY_DIST)
                    {
                        // check overlap ratio
                        std::vector<Eigen::Vector3d> pts0;
                        for(size_t i = 0; i < kfrm_storage[p0].pts.size(); i+=4)
                        {
                            Eigen::Vector3d P;
                            P[0] = kfrm_storage[p0].pts[i].x;
                            P[1] = kfrm_storage[p0].pts[i].y;
                            P[2] = kfrm_storage[p0].pts[i].z;

                            Eigen::Vector3d _P = kfrm_storage[p0].opt_G.block(0,0,3,3)*P + kfrm_storage[p0].opt_G.block(0,3,3,1);
                            pts0.push_back(_P);
                        }

                        std::vector<Eigen::Vector3d> pts1;
                        for(size_t i = 0; i < kfrm_storage[p1].pts.size(); i+=4)
                        {
                            Eigen::Vector3d P;
                            P[0] = kfrm_storage[p1].pts[i].x;
                            P[1] = kfrm_storage[p1].pts[i].y;
                            P[2] = kfrm_storage[p1].pts[i].z;

                            Eigen::Vector3d _P = kfrm_storage[p1].opt_G.block(0,0,3,3)*P + kfrm_storage[p1].opt_G.block(0,3,3,1);
                            pts1.push_back(_P);
                        }

                        double overlap_ratio = calc_overlap_ratio(pts0, pts1);
                        if(overlap_ratio < config->SLAM_KFRM_LC_TRY_OVERLAP)
                        {
                            printf("[MAPPING] less overlap, id:%d-%d, d:%f, overlap:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, d, overlap_ratio);
                            continue;
                        }

                        printf("[MAPPING] try loop closing, id:%d-%d, d:%f, overlap:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, d, overlap_ratio);

                        Eigen::Matrix4d dG = kfrm_storage[p0].opt_G.inverse()*kfrm_storage[p1].opt_G;
                        double err = kfrm_icp(kfrm_storage[p0], kfrm_storage[p1], dG);
                        if(err < config->SLAM_ICP_ERROR_THRESHOLD)
                        {
                            pgo.add_lc(kfrm_storage[p0].id, kfrm_storage[p1].id, dG, err);
                            printf("[MAPPING] pose graph optimization, id:%d-%d, err:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, err);

                            // optimal pose update
                            std::vector<Eigen::Matrix4d> opt_poses = pgo.get_optimal_poses();
                            for(int q = kfrm_storage[p0].id; q < (int)kfrm_storage.size(); q++)
                            {
                                kfrm_storage[q].opt_G = opt_poses[q];
                                kfrm_update_que.push(q);
                            }
                            break;
                        }
                        else
                        {
                            printf("[MAPPING] loop closing failed\n");
                        }
                    }
                }
            }

            printf("[MAPPING] kfrm:%d processing complete\n", kfrm.id);

            // update processing time
            proc_time_map_b = get_time() - st_time;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[MAPPING] b_loop stop\n");
}

// algorithms
double MAPPING::frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        Eigen::Vector3d _P = frm.pts[p];

        if(config->PLATFORM_TYPE == "S100")
        {
            double dx = 1.0;
            if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
                    _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
            {
                continue;
            }
        }

        idx_list.push_back(p);
        Eigen::Vector3d P = G.block(0,0,3,3)*_P + G.block(0,3,3,1);
        pts.push_back(P);
    }

    // solution
    Eigen::Matrix4d _G = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = loc->max_iter0;
    double lambda = loc->lambda0;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = config->SLAM_ICP_COST_THRESHOLD*config->SLAM_ICP_COST_THRESHOLD;
    const int num_feature = std::min<int>(idx_list.size(), config->SLAM_ICP_MAX_FEATURE_NUM);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;

        Eigen::Matrix4d cur_G = _G*G;
        for(size_t p = 0; p < idx_list.size(); p++)
        {
            // get index
            int i = idx_list[p];

            // local to global
            Eigen::Vector3d P1 = pts[i];
            Eigen::Vector3d _P1 = _G.block(0,0,3,3)*P1 + _G.block(0,3,3,1);
            Eigen::Vector3d V1 = (_P1 - cur_G.block(0,3,3,1)).normalized();

            // knn points
            int nn_idx = 0;
            Eigen::Vector3d V0;
            Eigen::Vector3d P0(0, 0, 0);
            {
                const int pt_num = config->LOC_SURFEL_NUM;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                V0 = Eigen::Vector3d(cloud.pts[nn_idx].vx, cloud.pts[nn_idx].vy, cloud.pts[nn_idx].vz);

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

            // view filter
            if(compare_view_vector(V0, V1, config->SLAM_ICP_VIEW_THRESHOLD*D2R))
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

            // point to point distance
            double cost = (_P1 - P0).squaredNorm();
            if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + loc->rmt_sigma)
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

            // dynamic object filter
            if(cloud.pts[nn_idx].do_cnt > 0 && cloud.pts[nn_idx].do_cnt < config->SLAM_ICP_DO_ACCUM_NUM)
            {
                continue;
            }

            // additional weight
            double dist = calc_dist_2d(frm.pts[i]);
            double weight = 1.0 + 0.02*dist;

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
            printf("[frm_icp] not enough correspondences, %d!!\n", num_correspondence);
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
        if(sigma < loc->sigma_eps)
        {
            sigma = loc->sigma_eps;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = loc->t_dist_v0;
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
            //err += std::sqrt(std::abs(c));
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
            lambda *= loc->lambda_dec;
        }
        else
        {
            lambda *= loc->lambda_inc;
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

    if(last_err > first_err+0.01 || last_err > config->SLAM_ICP_ERROR_THRESHOLD)
    {
        printf("[frm_icp] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence,
               first_err, last_err, convergence, get_time()-t_st);

        return 9999;
    }

    return last_err;
}

double MAPPING::kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
{
    // for processing time
    double t_st = get_time();

    // build kd_tree
    XYZR_CLOUD cloud;
    for(size_t p = 0; p < frm0.pts.size(); p++)
    {
        cloud.pts.push_back(frm0.pts[p]);
    }

    KD_TREE_XYZR tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // points random selection
    std::vector<int> idx_list;
    std::vector<PT_XYZR> pts;
    for(size_t p = 0; p < frm1.pts.size(); p++)
    {
        idx_list.push_back(p);

        Eigen::Vector3d P(frm1.pts[p].x, frm1.pts[p].y, frm1.pts[p].z);
        Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);

        PT_XYZR pt = frm1.pts[p];
        pt.x = _P[0];
        pt.y = _P[1];
        pt.z = _P[2];
        pts.push_back(pt);
    }

    // solution
    Eigen::Matrix4d _dG = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = loc->max_iter0;
    double lambda = loc->lambda0;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = config->SLAM_ICP_COST_THRESHOLD*config->SLAM_ICP_COST_THRESHOLD;
    const int num_feature = std::min<int>(idx_list.size(), config->SLAM_ICP_MAX_FEATURE_NUM);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());

        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;

        for(size_t p = 0; p < idx_list.size(); p++)
        {
            // get index
            int i = idx_list[p];

            // local to global
            Eigen::Vector3d P1(pts[i].x, pts[i].y, pts[i].z);
            Eigen::Vector3d _P1 = _dG.block(0,0,3,3)*P1 + _dG.block(0,3,3,1);

            // knn points
            int nn_idx = 0;
            Eigen::Vector3d P0(0, 0, 0);
            {
                const int pt_num = config->LOC_SURFEL_NUM;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];

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
            if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + loc->rmt_sigma)
            {
                continue;
            }

            // jacobian
            Eigen::Vector3d xi = TF_to_se2(_dG);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // dynamic object filter
            if(cloud.pts[nn_idx].do_cnt < config->SLAM_ICP_DO_ACCUM_NUM)
            {
                continue;
            }

            // additional weight
            double weight = 1.0;

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
            printf("[frm_icp] not enough correspondences, %d!!\n", num_correspondence);
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
        if(sigma < loc->sigma_eps)
        {
            sigma = loc->sigma_eps;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = loc->t_dist_v0;
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
            //err += std::abs(w*c);
            err += std::sqrt(std::abs(c));
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
            lambda *= loc->lambda_dec;
        }
        else
        {
            lambda *= loc->lambda_inc;
        }
        last_err = err;

        // pose update
        Eigen::Vector3d xi;
        xi[0] = X(0, 0);
        xi[1] = X(1, 0);
        xi[2] = X(2, 0);

        _dG = se2_to_TF(xi)*_dG;
        refine_pose(_dG);

        // for rmt
        tm0 = tm1;
        tm1 = _dG.block(0,3,3,1).norm();

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    dG = _dG*dG;

    // check
    printf("[kfrm_icp] id:%d-%d, i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", frm0.id, frm1.id, iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);
    //if(last_err > first_err+0.01 || last_err > config->SLAM_ICP_ERROR_THRESHOLD)
    if(last_err > first_err+0.01)
    {
        return 9999;
    }

    return last_err;
}

double MAPPING::calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1)
{
    // Lambda to compute AABB from points
    auto computeAABB = [](const std::vector<Eigen::Vector3d>& points)
    {
        Eigen::Vector3d min_corner = points[0];
        Eigen::Vector3d max_corner = points[0];
        for(const auto& p : points)
        {
            min_corner = min_corner.cwiseMin(p);
            max_corner = max_corner.cwiseMax(p);
        }

        for(int i = 0; i < 3; i++)
        {
            if(min_corner[i] == max_corner[i])
            {
                min_corner[i] -= 0.05;
                max_corner[i] += 0.05;
            }

        }

        return std::make_pair(min_corner, max_corner);
    };

    // Lambda to compute volume of an AABB
    auto volume = [](const Eigen::Vector3d& min_corner, const Eigen::Vector3d& max_corner)
    {
        return (max_corner - min_corner).prod();
    };

    // Lambda to compute intersection volume of two AABBs
    auto intersectionVolume = [](const Eigen::Vector3d& min_a, const Eigen::Vector3d& max_a,
                                 const Eigen::Vector3d& min_b, const Eigen::Vector3d& max_b)
    {
        Eigen::Vector3d inter_min = min_a.cwiseMax(min_b);
        Eigen::Vector3d inter_max = max_a.cwiseMin(max_b);
        Eigen::Vector3d inter_size = inter_max - inter_min;
        if ((inter_size.array() < 0).any())
        {
            return 0.0; // No intersection
        }

        return inter_size.prod();
    };

    // Compute AABBs for both point clouds
    auto [min0, max0] = computeAABB(pts0);
    auto [min1, max1] = computeAABB(pts1);

    // Compute volumes
    double vol0 = volume(min0, max0);
    double vol1 = volume(min1, max1);
    double interVol = intersectionVolume(min0, max0, min1, max1);

    // Compute overlap ratio
    double unionVol = vol0 + vol1 - interVol;
    return (unionVol == 0) ? 0.0 : interVol / unionVol;
}

void MAPPING::clear_pose_graph()
{
    pgo.clear();
}

void MAPPING::last_lc()
{
    // try last lc
    if(kfrm_storage.size() >= 2)
    {
        KFRAME kfrm0 = kfrm_storage.front();
        KFRAME kfrm1 = kfrm_storage.back();

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Matrix4d dG = kfrm0.opt_G.inverse()*(cur_tf.inverse()*kfrm1.G);
        Eigen::Vector3d dxi = TF_to_se2(dG);
        printf("[LAST_LC] dxi: %f, %f, %f\n", dxi[0], dxi[1], dxi[2]*R2D);

        double err = kfrm_icp(kfrm0, kfrm1, dG);
        if(err < config->SLAM_ICP_ERROR_THRESHOLD)
        {
            pgo.add_lc(kfrm0.id, kfrm1.id, dG, err);
            printf("[LAST_LC] pose graph optimization, id:%d-%d, err:%f\n", kfrm0.id, kfrm1.id, err);

            // optimal pose update
            std::vector<Eigen::Matrix4d> opt_poses = pgo.get_optimal_poses();
            for(size_t p = 0; p < kfrm_storage.size(); p++)
            {
                kfrm_storage[p].opt_G = opt_poses[p];
                kfrm_update_que.push(p);

                printf("[LAST_LC] optimal pose update, id:%d\n", (int)p);
            }

            printf("[LAST_LC] success\n");
        }
        else
        {
            printf("[LAST_LC] failed\n");
        }
    }
}
