#include "slam_2d.h"

SLAM_2D::SLAM_2D(QObject *parent)
    : QObject{parent}
{
    cur_tf.setIdentity();
}

SLAM_2D::~SLAM_2D()
{
    // stop slam loops
    if(map_a_thread != NULL)
    {
        map_a_flag = false;
        map_a_thread->join();
        map_a_thread = NULL;
    }

    if(map_b_thread != NULL)
    {
        map_b_flag = false;
        map_b_thread->join();
        map_b_thread = NULL;
    }

    is_slam = false;

    // stop obs loops
    if(obs_thread != NULL)
    {
        obs_flag = false;
        obs_thread->join();
        obs_thread = NULL;
    }

    // stop loc loops
    if(loc_a_thread != NULL)
    {
        loc_a_flag = false;
        loc_a_thread->join();
        loc_a_thread = NULL;
    }

    if(loc_b_thread != NULL)
    {
        loc_b_flag = false;
        loc_b_thread->join();
        loc_b_thread = NULL;
    }

    is_loc = false;
}

void SLAM_2D::mapping_start()
{
    if(is_slam == false)
    {
        printf("[SLAM] mapping start\n");

        // clear objects        
        mtx.lock();
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
        cur_tf = Eigen::Matrix4d::Identity();
        mtx.unlock();

        // start loop
        if(map_a_thread == NULL)
        {
            map_a_flag = true;
            map_a_thread = new std::thread(&SLAM_2D::map_a_loop, this);
        }

        if(map_b_thread == NULL)
        {
            map_b_flag = true;
            map_b_thread = new std::thread(&SLAM_2D::map_b_loop, this);
        }

        // set flag
        is_slam = true;
    }
    else
    {
        printf("[SLAM] mapping already running\n");
    }
}

void SLAM_2D::mapping_stop()
{
    // stop slam loops
    if(map_a_thread != NULL)
    {
        map_a_flag = false;
        map_a_thread->join();
        map_a_thread = NULL;
    }

    if(map_b_thread != NULL)
    {
        map_b_flag = false;
        map_b_thread->join();
        map_b_thread = NULL;
    }

    is_slam = false;
}

void SLAM_2D::localization_start()
{
    if(is_loc == false)
    {
        // set loc start time
        loc_st_time = get_time();

        // start loop
        if(loc_a_thread == NULL)
        {
            loc_a_flag = true;
            loc_a_thread = new std::thread(&SLAM_2D::loc_a_loop, this);
        }

        if(loc_b_thread == NULL)
        {
            loc_b_flag = true;
            loc_b_thread = new std::thread(&SLAM_2D::loc_b_loop, this);
        }

        if(obs_thread == NULL)
        {
            obs_flag = true;
            obs_thread = new std::thread(&SLAM_2D::obs_loop, this);
        }

        // set flag
        is_loc = true;
    }
    else
    {
        printf("[SLAM] localization already running\n");
    }
}

void SLAM_2D::localization_stop()
{
    is_loc = false;

    // stop loc loops
    if(loc_a_thread != NULL)
    {
        loc_a_flag = false;
        loc_a_thread->join();
        loc_a_thread = NULL;
    }

    if(loc_b_thread != NULL)
    {
        loc_b_flag = false;
        loc_b_thread->join();
        loc_b_thread = NULL;
    }

    if(obs_thread != NULL)
    {
        obs_flag = false;
        obs_thread->join();
        obs_thread = NULL;
    }    
}

Eigen::Matrix4d SLAM_2D::get_cur_tf()
{
    mtx.lock();
    Eigen::Matrix4d res = cur_tf;
    mtx.unlock();

    return res;
}

Eigen::Matrix4d SLAM_2D::get_best_tf(double t)
{
    mtx.lock();
    std::vector<TIME_POSE> _tp_storage = tp_storage;
    Eigen::Matrix4d _cur_tf = cur_tf;
    mtx.unlock();

    if(_tp_storage.size() == 0)
    {
        return _cur_tf;
    }

    int min_idx = 0;
    double min_dt = 99999999;
    for(size_t p = 0; p < _tp_storage.size(); p++)
    {
        double dt = std::abs(_tp_storage[p].t - t);
        if(dt < min_dt)
        {
            min_dt = dt;
            min_idx = p;
        }
    }

    if(min_dt > 1.0)
    {
        return _cur_tf;
    }
    else
    {
        return _tp_storage[min_idx].tf;
    }
}

TIME_POSE_PTS SLAM_2D::get_cur_tpp()
{
    mtx.lock();
    TIME_POSE_PTS res = cur_tpp;
    mtx.unlock();

    return res;
}

Eigen::Vector2d SLAM_2D::get_cur_ieir()
{
    mtx.lock();
    Eigen::Vector2d res = cur_ieir;
    mtx.unlock();

    return res;
}

QString SLAM_2D::get_cur_loc_state()
{
    mtx.lock();
    QString res = cur_loc_state; // none, fail, good
    mtx.unlock();

    return res;
}

void SLAM_2D::set_cur_loc_state(QString str)
{
    mtx.lock();
    cur_loc_state = str; // none, fail, good
    mtx.unlock();
}

QString SLAM_2D::get_info_text()
{
    Eigen::Vector2d cur_ieir = get_cur_ieir();
    QString loc_state = get_cur_loc_state();

    QString str;
    str.sprintf("[SLAM_INFO]\nmap_t: %.3f, %.3f\nloc_t: %.3f, %.3f\nkfrm_num: %d\nie: %.3f, ir: %.3f, loc_state: %s",
                          (double)proc_time_map_a, (double)proc_time_map_b,
                          (double)proc_time_loc_a, (double)proc_time_loc_b,
                          (int)kfrm_storage.size(),
                          cur_ieir[0], cur_ieir[1], loc_state.toLocal8Bit().data());
    return str;
}

void SLAM_2D::map_a_loop()
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

    printf("[SLAM] map_a_loop start\n");
    while(map_a_flag)
    {
        FRAME frm;
        if(lidar->scan_que.try_pop(frm))
        {
            double st_time = get_time();

            if(frm_cnt == 0)
            {
                // get last global tf
                mtx.lock();
                Eigen::Matrix4d _cur_tf = cur_tf;
                mtx.unlock();

                // local to global
                std::vector<PT_XYZR> dsk;
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Vector3d _P = frm.pts[p];

                    #ifdef USE_SRV
                    double dx = 1.0;
                    if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
                       _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }
                    #endif

                    Eigen::Vector3d P = _cur_tf.block(0,0,3,3)*_P + _cur_tf.block(0,3,3,1);
                    Eigen::Vector3d V = (P - _cur_tf.block(0,3,3,1)).normalized();

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
                mtx.lock();
                cur_tf = _cur_tf;
                mtx.unlock();

                // update previous frame
                frm0 = frm;

                // update tree add count
                add_cnt++;
            }
            else
            {
                // get last global tf
                mtx.lock();
                Eigen::Matrix4d _cur_tf = cur_tf;
                mtx.unlock();

                // initial guess
                Eigen::Matrix4d delta_tf = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm.mo.pose);
                Eigen::Matrix4d G = _cur_tf * delta_tf;

                // check moving
                double moving_d = calc_dist_2d(delta_tf.block(0,3,3,1));
                double moving_th = std::abs(TF_to_se2(delta_tf)[2]);
                if(moving_d > 0.05 || moving_th > 2.5*D2R)
                {
                    // pose estimation
                    double err = frm_icp(*live_tree, live_cloud, frm, G);
                    Eigen::Vector2d ieir = calc_ie_ir(*live_tree, live_cloud, frm, G);
                    if(err < config->SLAM_ICP_ERROR_THRESHOLD)
                    {
                        // local to global
                        std::vector<PT_XYZR> dsk;
                        for(size_t p = 0; p < frm.pts.size(); p++)
                        {
                            Eigen::Vector3d _P = frm.pts[p];

                            #ifdef USE_SRV
                            double dx = 1.0;
                            if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
                               _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                            {
                                continue;
                            }
                            #endif

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
                        mtx.lock();
                        cur_tf = G;
                        mtx.unlock();

                        // update previous frame
                        frm0 = frm;

                        // update tree add count
                        add_cnt++;

                        // make keyframe
                        if(add_cnt % config->SLAM_KFRM_UPDATE_NUM == 0)
                        {
                            // set keyframe
                            KFRAME kfrm;
                            kfrm.id = kfrm_id++;
                            kfrm.pts = live_cloud.pts;
                            kfrm.G = G;
                            kfrm.opt_G = G;
                            kfrm_que.push(kfrm);
                            printf("keyframe created, id:%d\n", kfrm.id);
                        }

                        // update ieir
                        mtx.lock();
                        cur_ieir = ieir;
                        mtx.unlock();
                    }
                    else
                    {
                        mtx.lock();
                        cur_ieir[0] = err;
                        mtx.unlock();
                    }
                }
            }

            // update frame count
            frm_cnt++;

            // update processing time
            proc_time_map_a = get_time() - st_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[SLAM] map_a_loop stop\n");
}

void SLAM_2D::map_b_loop()
{
    PGO pgo;

    printf("[SLAM] map_b_loop start\n");
    while(map_b_flag)
    {
        KFRAME kfrm;
        if(kfrm_que.try_pop(kfrm))
        {
            double st_time = get_time();

            printf("[SLAM] kfrm:%d processing start\n", kfrm.id);

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
                            printf("[SLAM] less overlap, id:%d-%d, d:%f, overlap:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, d, overlap_ratio);
                            continue;
                        }

                        printf("[SLAM] try loop closing, id:%d-%d, d:%f, overlap:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, d, overlap_ratio);

                        Eigen::Matrix4d dG = kfrm_storage[p0].opt_G.inverse()*kfrm_storage[p1].opt_G;
                        double err = kfrm_icp(kfrm_storage[p0], kfrm_storage[p1], dG);
                        if(err < config->SLAM_ICP_ERROR_THRESHOLD)
                        {
                            pgo.add_lc(kfrm_storage[p0].id, kfrm_storage[p1].id, dG, err);
                            printf("[SLAM] pose graph optimization, id:%d-%d, err:%f\n", kfrm_storage[p0].id, kfrm_storage[p1].id, err);

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
                            printf("[SLAM] loop closing failed\n");
                        }
                    }
                }
            }

            printf("[SLAM] kfrm:%d processing complete\n", kfrm.id);

            // update processing time
            proc_time_map_b = get_time() - st_time;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[SLAM] map_b_loop stop\n");
}

void SLAM_2D::loc_a_loop()
{
    lidar->scan_que.clear();
    Eigen::Vector3d pre_mo(0,0,0);

    printf("[SLAM] loc_a_loop start\n");
    while(loc_a_flag)
    {
        bool is_new = false;
        FRAME frm;
        while(lidar->scan_que.try_pop(frm) && loc_a_flag)
        {
            is_new = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if(is_new)
        {
            if(unimap->is_loaded == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            double st_time = get_time();

            mtx.lock();
            Eigen::Matrix4d _cur_tf = cur_tf;            
            mtx.unlock();

            /*
            // check moving
            if(get_time() > loc_st_time + 10.0)
            {
                Eigen::Vector3d cur_mo = frm.mo.pose;
                double dx = pre_mo[0] - cur_mo[0];
                double dy = pre_mo[1] - cur_mo[1];
                double d = std::sqrt(dx*dx + dy*dy);
                double dth = std::abs(deltaRad(cur_mo[2], pre_mo[2]));
                if(d < 0.05 && dth < 2.0*D2R)
                {
                    continue;
                }
                pre_mo = cur_mo;
            }
            */

            // pose estimation            
            double err = map_icp(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _cur_tf);
            if(err < config->SLAM_ICP_ERROR_THRESHOLD)
            {
                // for loc b loop
                TIME_POSE tp;
                tp.t = frm.t;
                tp.tf = _cur_tf;
                tp.tf2 = se2_to_TF(frm.mo.pose);
                tp_que.push(tp);

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = _cur_tf;
                tpp.pts = frm.pts;
                tpp_que.push(tpp);

                // update
                mtx.lock();
                cur_tpp = tpp;
                mtx.unlock();

                // check inlier error, inlier ratio
                Eigen::Vector2d ieir = calc_ie_ir(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _cur_tf);

                // update ieir
                mtx.lock();
                cur_ieir = ieir;
                mtx.unlock();
            }
            else
            {
                // update ieir
                mtx.lock();
                cur_ieir[0] = err;
                mtx.unlock();
            }

            //lidar->scan_que.clear();

            // update processing time
            proc_time_loc_a = get_time() - st_time;
        }        

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[SLAM] loc_a_loop stop\n");
}

void SLAM_2D::loc_b_loop()
{
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    mtx.lock();
    tp_storage.clear();
    mtx.unlock();

    MOBILE_POSE mo0 = mobile->get_pose();

    printf("[SLAM] loc_b_loop start\n");
    while(loc_b_flag)
    {
        MOBILE_POSE mo = mobile->get_pose();
        if(mo.t > mo0.t)
        {
            double st_time = get_time();

            Eigen::Matrix4d pre_mo_tf = se2_to_TF(mo0.pose);
            Eigen::Matrix4d cur_mo_tf = se2_to_TF(mo.pose);

            // get current tf
            mtx.lock();
            Eigen::Matrix4d _cur_tf = cur_tf;
            mtx.unlock();

            // update odo_tf
            Eigen::Matrix4d delta_tf = pre_mo_tf.inverse()*cur_mo_tf;
            pre_mo_tf = cur_mo_tf;
            Eigen::Matrix4d odo_tf = _cur_tf*delta_tf;

            // icp-odometry fusion
            TIME_POSE tp;
            if(tp_que.try_pop(tp))
            {
                if(std::abs(mo.t - tp.t) < 0.3)
                {
                    double alpha = config->LOC_FUSION_RATIO; // 1.0 means odo_tf 100%
                    Eigen::Matrix4d icp_tf = tp.tf;

                    // for odometry slip
                    Eigen::Vector2d dtdr = dTdR(odo_tf, icp_tf);
                    if(std::abs(dtdr[1]) > 10.0*D2R)
                    {
                        alpha = 0;
                        printf("[LOC] slip detection, dth: %f\n", dtdr[1]*R2D);
                    }

                    // interpolation
                    Eigen::Matrix4d dtf = icp_tf.inverse()*odo_tf;
                    Eigen::Matrix4d fused_tf = icp_tf*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                    // update
                    _cur_tf = fused_tf;
                }
            }
            else
            {
                // update
                _cur_tf = odo_tf;
            }


            // aruco fusion
            TIME_POSE_ID aruco_tpi = aruco->get_cur_tpi();

            // calc tf
            NODE* node = unimap->get_node_by_name(QString::number(aruco_tpi.id, 10));
            if(node != NULL)
            {
                // if(std::abs(mo.t - aruco_tpi.t) < 0.3)
                {
                    Eigen::Matrix4d T_g_m = node->tf;
                    Eigen::Matrix4d T_m_r = aruco_tpi.tf.inverse();
                    Eigen::Matrix4d T_g_r = T_g_m*T_m_r;

                    // interpolation
                    // double alpha = 0.1; // 0.1 means 90% aruco_tf, 10% cur_tf
                    // Eigen::Matrix4d dtf = T_g_r.inverse()*_cur_tf;
                    // Eigen::Matrix4d fused_tf = T_g_r*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                    // update
                    _cur_tf = T_g_r;
                }
            }


            // update
            mtx.lock();
            cur_tf = _cur_tf;

            TIME_POSE fused_tp;
            fused_tp.t = mo.t;
            fused_tp.tf = _cur_tf;
            tp_storage.push_back(fused_tp);
            if(tp_storage.size() > 300)
            {
                tp_storage.erase(tp_storage.begin());
            }
            mtx.unlock();

            // for next operation
            mo0 = mo;

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
            printf("[SLAM] loc_b_loop loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    printf("[SLAM] loc_b_loop stop\n");
}

void SLAM_2D::obs_loop()
{
    printf("[SLAM] obs_loop start\n");
    while(obs_flag)
    {
        TIME_POSE_PTS tpp;
        if(tpp_que.try_pop(tpp))
        {            
            // add blidar pts
            {
                TIME_PTS blidar_scan = blidar->get_cur_tp();
                Eigen::Matrix4d tf0 = tpp.tf.inverse()*get_best_tf(blidar_scan.t);

                for(size_t p = 0; p < blidar_scan.pts.size(); p++)
                {
                    Eigen::Vector3d P = blidar_scan.pts[p];
                    Eigen::Vector3d _P = tf0.block(0,0,3,3)*P + tf0.block(0,3,3,1);
                    tpp.pts.push_back(_P);
                }
            }

            // add cam pts
            {
                TIME_PTS cam_scan0 = cam->get_scan0();
                Eigen::Matrix4d tf0 = tpp.tf.inverse()*get_best_tf(cam_scan0.t);

                for(size_t p = 0; p < cam_scan0.pts.size(); p+=4)
                {
                    Eigen::Vector3d P = cam_scan0.pts[p];
                    Eigen::Vector3d _P = tf0.block(0,0,3,3)*P + tf0.block(0,3,3,1);
                    tpp.pts.push_back(_P);
                }

                TIME_PTS cam_scan1 = cam->get_scan1();
                Eigen::Matrix4d tf1 = tpp.tf.inverse()*get_best_tf(cam_scan1.t);
                for(size_t p = 0; p < cam_scan1.pts.size(); p+=4)
                {
                    Eigen::Vector3d P = cam_scan1.pts[p];
                    Eigen::Vector3d _P = tf1.block(0,0,3,3)*P + tf1.block(0,3,3,1);
                    tpp.pts.push_back(_P);
                }
            }

            obsmap->update_obs_map(tpp);
            tpp_que.clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[SLAM] obs_loop stop\n");
}

// algorithms
double SLAM_2D::map_icp0(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
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

    const double cost_threshold = config->LOC_ICP_COST_THRESHOLD_0*config->LOC_ICP_COST_THRESHOLD_0;
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
            //printf("[map_icp0] not enough correspondences, iter: %d, num: %d->%d!!\n", iter, (int)frm.pts.size(), num_correspondence);
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
        //printf("[map_icp0] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);
        return 9999;
    }

    return last_err;
}

double SLAM_2D::map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
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
            if(dist < config->LIDAR_MIN_RANGE)
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
        return 9999;
    }

    return last_err;
}

double SLAM_2D::frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        Eigen::Vector3d _P = frm.pts[p];

        #ifdef USE_SRV
        double dx = 1.0;
        if(_P[0] > config->ROBOT_SIZE_X[0] - dx && _P[0] < config->ROBOT_SIZE_X[1] + dx &&
           _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
        {
            continue;
        }
        #endif

        idx_list.push_back(p);
        Eigen::Vector3d P = G.block(0,0,3,3)*_P + G.block(0,3,3,1);
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

    if(last_err > first_err+0.01 || last_err > config->SLAM_ICP_ERROR_THRESHOLD)
    {
        printf("[frm_icp0] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence,
               first_err, last_err, convergence, get_time()-t_st);

        return 9999;
    }

    return last_err;
}

double SLAM_2D::kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
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
    const int max_iter = max_iter0;
    double lambda = lambda0;
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
            if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + rmt_sigma)
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
    if(last_err > first_err+0.01 || last_err > config->SLAM_ICP_ERROR_THRESHOLD)
    {
        return 9999;
    }
    return last_err;
}

void SLAM_2D::semi_auto_init_start()
{
    is_busy = true;

    if(unimap->kdtree_cloud.pts.size() == 0)
    {
        is_busy = false;
        Q_EMIT signal_localization_semiautoinit_failed("kdtree_cloud size 0");
        return;
    }

    int wait_cnt = 0;
    std::vector<Eigen::Vector3d> _cur_scan = lidar->get_cur_scan();
    while(_cur_scan.size() == 0)
    {
        wait_cnt++;
        if(wait_cnt > 30)
        {
            is_busy = false;
            Q_EMIT signal_localization_semiautoinit_failed("cur scan size 0");
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    FRAME frm;
    frm.pts = _cur_scan;

    // candidates
    std::vector<QString> ids = unimap->get_nodes("INIT");
    if(ids.size() == 0)
    {
        is_busy = false;
        Q_EMIT signal_localization_semiautoinit_failed("no INIT nodes");
        return;
    }

    // find best match
    Eigen::Vector2d min_ieir(1.0, 0.0);
    Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
    double min_cost = std::numeric_limits<double>::max();
    for(size_t p = 0; p < ids.size(); p++)
    {
        NODE* node = unimap->get_node_by_id(ids[p]);
        if(node == NULL)
        {
            continue;
        }

        Eigen::Matrix4d tf = node->tf;
        for(double th = -180.0; th < 180.0; th += 10.0)
        {
            Eigen::Matrix4d rot = se2_to_TF(Eigen::Vector3d(0, 0, th*D2R));
            Eigen::Matrix4d _tf = tf*rot;

            double err = map_icp0(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _tf);
            if(err < config->LOC_ICP_ERROR_THRESHOLD)
            {
                Eigen::Vector2d ieir = calc_ie_ir(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _tf);
                double cost = (ieir[0]/config->LOC_ICP_ERROR_THRESHOLD) + (1.0 - ieir[1]);
                if(cost < min_cost)
                {
                    min_ieir = ieir;
                    min_cost = cost;
                    min_tf = _tf;

                    //Eigen::Vector3d min_pose = TF_to_se2(min_tf);
                    //printf("[AUTOINIT] x:%f, y:%f, th:%f, cost:%f\n", min_pose[0], min_pose[1], min_pose[2]*R2D, cost);
                }
            }
        }
    }

    if(min_ieir[0] < config->LOC_CHECK_IE && min_ieir[1] > config->LOC_CHECK_IR)
    {
        Eigen::Vector3d min_pose = TF_to_se2(min_tf);
        printf("[AUTOINIT] success auto init. x:%f, y:%f, th:%f, cost:%f\n", min_pose[0], min_pose[1], min_pose[2]*R2D, min_cost);

        mtx.lock();
        cur_tf = min_tf;
        mtx.unlock();

        localization_start();
        Q_EMIT signal_localization_semiautoinit_succeed("success");
    }
    else
    {
        Q_EMIT signal_localization_semiautoinit_failed("failed semi-auto init");
        printf("[AUTOINIT] failed auto init.\n");
    }

    is_busy = false;
}

double SLAM_2D::calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1)
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

Eigen::Vector2d SLAM_2D::calc_ie_ir(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    int err_num = 0;
    double err_sum = 0;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        Eigen::Vector3d P = frm.pts[p];
        Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

        // find nn
        std::vector<unsigned int> ret_near_idxs(1);
        std::vector<double> ret_near_sq_dists(1);

        double near_query_pt[3] = {_P[0], _P[1], _P[2]};
        tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

        // point to point distance
        double err = std::sqrt(ret_near_sq_dists[0]);
        if(err > config->LOC_CHECK_DIST)
        {
            continue;
        }

        err_sum += err;
        err_num++;
    }

    if(err_num == 0)
    {
        Eigen::Vector2d res;
        res[0] = 1.0;
        res[1] = 0.0;
        return res;
    }
    else
    {
        Eigen::Vector2d res;
        res[0] = err_sum/err_num; // inlier error
        res[1] = (double)err_num/frm.pts.size(); // inlier ratio
        return res;
    }
}
