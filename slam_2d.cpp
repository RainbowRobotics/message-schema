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

    is_loc = false;
}

void SLAM_2D::mapping_start()
{
    if(is_slam == false)
    {
        printf("[SLAM] mapping start\n");

        // clear object
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

    is_loc = false;
}

Eigen::Matrix4d SLAM_2D::get_cur_tf()
{
    mtx.lock();
    Eigen::Matrix4d res = cur_tf;
    mtx.unlock();

    return res;
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
    QString res = cur_loc_state;
    mtx.unlock();

    return res;
}

void SLAM_2D::set_cur_loc_state(QString str)
{
    mtx.lock();
    cur_loc_state = str;
    mtx.unlock();
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
                //Eigen::Vector3d input = mobile->get_control_input();
                //Eigen::Vector3d feedback = mobile->get_pose().vel;
                //double diff = (feedback - input).norm();
                //if((std::abs(input[0]) > 0 || std::abs(input[1]) > 0 || std::abs(input[2]) > 0) && diff < 0.05)
                //if(std::abs(input[0]) > 0 || std::abs(input[1]) > 0 || std::abs(input[2]) > 0)

                Eigen::Matrix4d dtf = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm.mo.pose);
                Eigen::Vector3d dxi = TF_to_se2(dtf);
                if(std::abs(dxi[0]) > 0.05 || std::abs(dxi[1]) > 0.05 || std::abs(dxi[2]) > 3.0*D2R)
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
                    }

                    // update ieir
                    mtx.lock();
                    cur_ieir = ieir;
                    mtx.unlock();
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

    printf("[SLAM] loc_a_loop start\n");
    while(loc_a_flag)
    {
        FRAME frm;
        if(lidar->scan_que.try_pop(frm))
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

            // pose estimation
            map_icp(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _cur_tf);

            // check inlier error, inlier ratio
            Eigen::Vector2d ieir = calc_ie_ir(*unimap->kdtree_index, unimap->kdtree_cloud, frm, _cur_tf);
            if(ieir[0] < config->LOC_CHECK_IE && ieir[1] > config->LOC_CHECK_IR)
            {
                // for loc b loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = _cur_tf;
                tpp.tf2 = se2_to_TF(frm.mo.pose);
                tpp.pts = frm.pts;
                tpp_que.push(tpp);

                // for obs loop
                tpp_que2.push(tpp);

                // update
                mtx.lock();
                cur_tpp = tpp;
                mtx.unlock();
            }

            // update ieir
            mtx.lock();
            cur_ieir = ieir;            
            mtx.unlock();

            lidar->scan_que.clear();

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

    bool is_first = true;
    Eigen::Matrix4d pre_mo_tf;

    printf("[SLAM] loc_b_loop start\n");
    while(loc_b_flag)
    {
        MOBILE_POSE mo = mobile->get_pose();
        if(mo.t > 0)
        {
            double st_time = get_time();

            Eigen::Matrix4d cur_mo_tf = se2_to_TF(mo.pose);

            // check first
            if(is_first)
            {
                is_first = false;
                pre_mo_tf = cur_mo_tf;

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            mtx.lock();
            Eigen::Matrix4d _cur_tf = cur_tf;
            mtx.unlock();

            Eigen::Matrix4d delta_tf = pre_mo_tf.inverse()*cur_mo_tf;
            _cur_tf = _cur_tf*delta_tf;

            TIME_POSE_PTS tpp;
            if(tpp_que.try_pop(tpp))
            {

                /*
                Eigen::Matrix4d fused_tf = intp_tf(config->LOC_FUSION_RATIO, tpp.tf, _cur_tf); // 1.0 mean odometry 100%
                _cur_tf = fused_tf;
                */

                Eigen::Matrix4d delta_tf = tpp.tf2.inverse()*cur_mo_tf;
                Eigen::Matrix4d icp_tf = tpp.tf*delta_tf;

                Eigen::Matrix4d fused_tf = intp_tf(config->LOC_FUSION_RATIO, icp_tf, _cur_tf); // 1.0 mean odometry 100%
                _cur_tf = fused_tf;                
            }

            /*
            TIME_POSE_PTS tpp;
            if(tpp_que.try_pop(tpp))
            {
                Eigen::Matrix4d delta_tf = tpp.tf2.inverse()*cur_mo_tf;
                Eigen::Matrix4d icp_tf = tpp.tf*delta_tf;

                Eigen::Matrix4d fused_tf = intp_tf(config->LOC_FUSION_RATIO, icp_tf, _cur_tf); // 1.0 mean odometry 100%
                _cur_tf = fused_tf;
            }
            else
            {
                // just add
                Eigen::Matrix4d delta_tf = pre_mo_tf.inverse()*cur_mo_tf;
                _cur_tf = _cur_tf*delta_tf;
            }
            */

            // update
            mtx.lock();
            cur_tf = _cur_tf;            
            mtx.unlock();

            // update pre mobile tf
            pre_mo_tf = cur_mo_tf;

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
        if(tpp_que2.try_pop(tpp))
        {
            obsmap->update_obs_map(tpp);
            tpp_que2.clear();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[SLAM] obs_loop stop\n");
}

// algorithms
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
    const int max_iter = 100;
    double lambda = 0.01;
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

            // local to global
            Eigen::Vector3d P1 = pts[i];
            Eigen::Vector3d _P1 = _G.block(0,0,3,3)*P1 + _G.block(0,3,3,1);

            // find nn            
            std::vector<unsigned int> ret_near_idxs(1);
            std::vector<double> ret_near_sq_dists(1);

            double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
            tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

            int nn_idx = ret_near_idxs[0];            
            Eigen::Vector3d P0(cloud.pts[nn_idx].x, cloud.pts[nn_idx].y, cloud.pts[nn_idx].z);

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
            double dist = frm.pts[i].norm();
            double weight = 1.0 + 0.1*dist;

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
        if(sigma < 0.001)
        {
            sigma = 0.001;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = 30;
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
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
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

    if(last_err > first_err+0.01 || first_err > config->LOC_ICP_ERROR_THRESHOLD || last_err > config->LOC_ICP_ERROR_THRESHOLD)
    {
        printf("[map_icp] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence,
               first_err, last_err, convergence, get_time()-t_st);
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
        idx_list.push_back(p);

        Eigen::Vector3d P = G.block(0,0,3,3)*frm.pts[p] + G.block(0,3,3,1);
        pts.push_back(P);
    }

    // solution
    Eigen::Matrix4d _G = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = 100;
    double lambda = 0.01;
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

            /*
            // find nn            
            std::vector<unsigned int> ret_near_idxs(1);
            std::vector<double> ret_near_sq_dists(1);

            double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
            tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

            int nn_idx = ret_near_idxs[0];
            Eigen::Vector3d P0(cloud.pts[nn_idx].x, cloud.pts[nn_idx].y, cloud.pts[nn_idx].z);
            Eigen::Vector3d V0(cloud.pts[nn_idx].vx, cloud.pts[nn_idx].vy, cloud.pts[nn_idx].vz);
            */

            int nn_idx = 0;
            Eigen::Vector3d V0;
            Eigen::Vector3d P0(0, 0, 0);
            {
                std::vector<unsigned int> ret_near_idxs(3);
                std::vector<double> ret_near_sq_dists(3);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], 3, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                V0 = Eigen::Vector3d(cloud.pts[nn_idx].vx, cloud.pts[nn_idx].vy, cloud.pts[nn_idx].vz);

                for(size_t q = 0; q < 3; q++)
                {
                    int idx = ret_near_idxs[q];
                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                }
                P0 /= 3;
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
            double dist = frm.pts[i].norm();
            double weight = 1.0 + 0.1*dist;

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
        if(sigma < 0.001)
        {
            sigma = 0.001;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = 30;
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
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
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

    if(last_err > first_err+0.01 || first_err > config->SLAM_ICP_ERROR_THRESHOLD || last_err > config->SLAM_ICP_ERROR_THRESHOLD)
    {
        printf("[frm_icp0] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence,
               first_err, last_err, convergence, get_time()-t_st);
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
    const int max_iter = 100;
    double lambda = 0.01;
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

            /*
            // find nn
            std::vector<unsigned int> ret_near_idxs(1);
            std::vector<double> ret_near_sq_dists(1);

            double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
            tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

            int nn_idx = ret_near_idxs[0];
            Eigen::Vector3d P0(cloud.pts[nn_idx].x, cloud.pts[nn_idx].y, cloud.pts[nn_idx].z);
            */

            int nn_idx = 0;
            Eigen::Vector3d P0(0, 0, 0);
            {
                std::vector<unsigned int> ret_near_idxs(3);
                std::vector<double> ret_near_sq_dists(3);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], 3, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                for(size_t q = 0; q < 3; q++)
                {
                    int idx = ret_near_idxs[q];
                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                }
                P0 /= 3;
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
        if(sigma < 0.001)
        {
            sigma = 0.001;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = 30;
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
            lambda *= 0.1;
        }
        else
        {
            lambda *= std::pow(2, iter);
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

    Eigen::Vector2d res;
    res[0] = err_sum/err_num; // inlier error
    res[1] = (double)err_num/frm.pts.size(); // inlier ratio
    return res;
}
