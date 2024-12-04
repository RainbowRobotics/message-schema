#include "docking.h"

DOCKING::DOCKING(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
}

DOCKING::~DOCKING()
{
    // control loop stop
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
    mobile->move(0, 0, 0);
}

void DOCKING::init()
{
    fsm_state = DOCKING_FSM_OFF;
    Vfrm = generateVKframe();
    frm1_center = calculateCenter(Vfrm);
}

void DOCKING::stop()
{
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
    
    fsm_state = DOCKING_FSM_OFF;
    mobile->move(0, 0, 0);
}

void DOCKING::move()
{
    // stop first
    stop();
    // obs clear
    obsmap->clear();
    // start control loop
    a_flag = true;
    dock = false ; //dock flag init

    //generate V
    Vfrm = generateVKframe();
    frm1_center = calculateCenter(Vfrm);

    // start docking control loop
    fsm_state = DOCKING_FSM_POINTDOCK;
    a_thread = new std::thread(&DOCKING::a_loop, this);
}

bool DOCKING::find_Vmark()
{
    std::vector<std::vector<Eigen::Vector3d>>filtered_clusters;
    std::vector<Eigen::Vector3d> c_lidar = lidar->get_cur_scan();
    std::vector<Eigen::Vector3d> filtered_lidar;

    //ROI
    for(unsigned int i=0; i<c_lidar.size(); i++)
    {
        double x = static_cast<double>(c_lidar[i].x());
        double y = static_cast<double>(c_lidar[i].y());

        double angle = std::atan2(y,x);

        if(std::abs(angle) < config->DOCKING_CLUST_ANGLE_THRESHOLD)
        {
            filtered_lidar.push_back(c_lidar[i]);
        }
    }

    const int c_points = filtered_lidar.size();
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2));

    for(unsigned int i=0; i <c_points; ++i)
    {
        float x = static_cast<float>(filtered_lidar[i].x());
        float y = static_cast<float>(filtered_lidar[i].y());
        polar[i][0] = x;
        polar[i][1] = y;
    }

    polar[c_points] = polar[0];
    std::vector<bool> clustered1(c_points+1 ,false);
    std::vector<bool> clustered2(c_points+1 ,false);

    float d =0.0;
    for (unsigned int i = 0; i < c_points; ++i)
    {
        d = sqrt(pow(polar[i][0] - polar[i + 1][0], 2) + pow(polar[i][1] - polar[i + 1][1], 2));

        if (d < clust_d_threshold)
        {
            clustered1[i] = true;
            clustered2[i + 1] = true;
        }
    }

    clustered2[0] = clustered2[c_points];
    std::vector<int> begin;
    std::vector<int> nclus;
    int i =0;
    bool flag = true;

    while(i<c_points && flag==true)
    {
        if (clustered1[i] == true && clustered2[i] == false && flag == true)
        {
            begin.push_back(i);
            nclus.push_back(1);
            while(clustered2[i+1] == true && clustered1[i+1] == true)
            {
                i++;
                ++nclus.back();

                if(i==c_points-1 && flag == true)
                {
                    i = -1;
                    flag = false;
                }
            }
        ++nclus.back();
        }
    i++;
    }

    if(clustered1[c_points-1]== true and clustered2[c_points-1] == false)
    {
        begin.push_back(c_points-1);
        nclus.push_back(1);

        i = 0;
        while(clustered2[i] == true && clustered1[i] == true )
        {
            i++;
            ++nclus.back();
        }
    }

    polar.pop_back(); 
    int len = polar.size();
    std::vector<std::vector<Eigen::Vector3d>> g_clusters;

    for(unsigned int i=0; i<begin.size(); ++i)
    {
        std::vector<Eigen::Vector3d> cluster;
        double x,y;
        int j =begin[i];
        bool fl = true;

        while (j<nclus[i]+begin[i])
        {
            if(j== len && fl == true) fl = false;
            if (fl == true)
            {
                x = polar[j][0];
                y = polar[j][1];
            }
            else
            {
                x = polar[j-len][0];
                y = polar[j-len][1];
            }

            Eigen::Vector3d point(x, y, 0.0);
            cluster.push_back(point);
            ++j;
        }
        int num = cluster.size();

        if(num > clust_size_min && num < clust_size_max) g_clusters.push_back(cluster);
    }

    for(auto& clust : g_clusters)
    {
        float mean_x = 0.0; float mean_y = 0.0;
        float sum_x= 0.0; float sum_y= 0.0;
        float d = 0.0; float find_angle =0.0;

        for(auto& point : clust)
        {
            sum_x += point.x();
            sum_y += point.y();
        }

        mean_x = sum_x / clust.size();
        mean_y = sum_y / clust.size();

        find_angle = std::atan2(mean_y, mean_x);
        d = sqrt(pow(mean_x, 2) + pow(mean_y, 2));

        //<Seokgyun.kim>
        //clust_dist_threshold_min and max filter clusters based on the distance from the robot.
        //clust_angle_threshold filters clusters based on the angle relative to the robot's coordinate system.
        if( d > config->DOCKING_CLUST_DIST_THRESHOLD_MIN && d < config->DOCKING_CLUST_DIST_THRESHOLD_MAX )
        {
            filtered_clusters.push_back(clust);
        }
    }

    std::vector<Eigen::Vector3d> docking_clust; // finally docking cluster

    if(!sizefilter(filtered_clusters,docking_clust))
    {
        return false;
    }
    clusters_queue.push(docking_clust);

    if(clusters_queue.size() > 10)
    {
        // KFRAME kfrm = generateVKframe();
        KFRAME cur_frm;

        while (!clusters_queue.empty())
        {
            const auto& cluster = clusters_queue.front();

            for (const auto& point : cluster)
            {
                PT_XYZR pt;
                pt.x = point.x();
                pt.y = point.y();
                pt.z = 0.0;
                pt.vx = 0.0;
                pt.vy = 0.0;
                pt.vz = 1.0;
                pt.r = 0.0;
                pt.k0 = 0;
                pt.k = 0;
                pt.do_cnt = 0;
                cur_frm.pts.push_back(pt);
            }
            clusters_queue.pop();
        }

        Eigen::Vector3d frm0_center0 = calculateCenter(cur_frm);
        Eigen::Matrix4d dock_tf = Eigen::Matrix4d::Identity();

        dock_tf = calculateTranslationMatrix(frm1_center, frm0_center0);

        double err = Vfrm_icp(cur_frm, Vfrm, dock_tf);

        if(err >0.003) //0.001
        {
            return false;
        }

        docking_station = dock_tf;

        Eigen::Matrix4d cur_pos = slam->get_cur_tf();
        docking_station_m = cur_pos * docking_station;

        path_flag = true;
        return true;
    }

    return false;
}

void DOCKING::a_loop()
{
    const double dt = 0.02; // 50hz
    double obs_wait_st_time = 0.0;
    double wait_start_time = 0.0;
    double vmark_start_time = get_time();

    while(a_flag){

        double pre_loop_time = get_time();
        Eigen::Matrix4d cur_pos = slam->get_cur_tf();

        int is_good_everything = is_everything_fine();

        if(is_good_everything == DRIVING_FAILED)
        {
            failed_reason = "FIND CHECK WRONG";
            fsm_state = DOCKING_FSM_FAILED;
        }
        else if(is_good_everything == DRIVING_NOT_READY)
        {
            failed_reason = "FIND CHECK WRONG";
            fsm_state = DOCKING_FSM_FAILED;
        }


        if(fsm_state == DOCKING_FSM_POINTDOCK)
        {

            double cmd_v =0.0;
            double cmd_w =0.0;
            find_Vmark();
            if(path_flag)
            {
                Eigen::Vector2d dtdr = dTdR(cur_pos, docking_station_m);

                if(std::abs(dtdr(0)) < config->DOCKING_GOAL_D)
                {
                    mobile->move(0,0,0);
                    fsm_state = DOCKING_FSM_COMPENSATE;
                    continue;
                }
                dockControl(cur_pos,cmd_v,cmd_w);

//                if(obsmap->is_tf_collision(cur_pos, 0.1, 0.1))
//                {
//                    obs_wait_st_time = get_time();
//                    mobile->move(0,0,0);
//                    fsm_state = DOCKING_FSM_OBS;

//                    printf("[DOCKING] DRIVING -> OBS_WAIT\n");
//                    continue;
//                }
                mobile->move(cmd_v,0,cmd_w);
            }

            else
            {
                if(get_time() - vmark_start_time > 10.0)
                {
                    Q_EMIT signal_dock_failed(failed_reason);
                }
            }
        }

        else if(fsm_state == DOCKING_FSM_COMPENSATE)
        {
            double cmd_v =0.0;
            double cmd_w =0.0;
            find_Vmark();

            if(path_flag)
            {
                Eigen::Vector2d dtdr = dTdR(cur_pos, docking_station_m);

                if(std::abs(dtdr(1)) < config->DOCKING_GOAL_TH)
                {
                    mobile->move(0,0,0);
                    fsm_state = DOCKING_FSM_DOCK;
                    continue;
                }
                dockControl(cur_pos,cmd_v,cmd_w);
                mobile->move(cmd_v,0.0,cmd_w);
            }
        }

        else if(fsm_state == DOCKING_FSM_DOCK)
        {
            if(dock)
            {
                failed_reason = "LOGIC WRONG";
                fsm_state = DOCKING_FSM_FAILED;
            }

            else
            {
                mobile->move_linear(config->DOCKING_POINTDOCK_MARGIN +0.025, 0.1);
                dock = true;
                fsm_state = DOCKING_FSM_WAIT;
                wait_start_time = get_time();
            }

        }

        else if (fsm_state == DOCKING_FSM_WAIT)
        {
            MOBILE_STATUS ms = mobile->get_status();

            mobile->move(0, 0, 0); 
            if (get_time() - wait_start_time > 20.0)
            {
                failed_reason = "NOT CONNECTED";
                Q_EMIT signal_dock_failed(failed_reason);
                fsm_state = DOCKING_FSM_FAILED;
            }

            if (ms.charge_state == 3)
            {
                fsm_state = DOCKING_FSM_COMPLETE;
                Q_EMIT signal_dock_succeed("success");
                // if(!dock_first)
                // {
                //     fsm_state = DOCKING_FSM_COMPLETE;
                //     Q_EMIT signal_dock_succeed("success");
                //     dock_first = true;
                // }
            }
        }

        else if (fsm_state == DOCKING_FSM_OBS)
        {
            mobile->move(0,0,0);
            if(get_time() - obs_wait_st_time > 1.0)
            {
                fsm_state = DOCKING_FSM_OBS;
                printf("[DOCKING] OBS_WAIT -> DRIVING\n");
                continue;
            }
        }
        
        else if (fsm_state == DOCKING_FSM_FAILED)
        {
            //TODO what?
        }

        else if (fsm_state == DOCKING_FSM_COMPLETE)
        {
            //TODO what?
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
}

// check condition
int DOCKING::is_everything_fine()
{

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.connection_m0 != 1 || ms.connection_m1 != 1)
    {
        logger->write_log("[DOCKING] failed (motor not connected)", "Red", true, false);
        return DRIVING_FAILED;
    }

    if(ms.status_m0 > 1 || ms.status_m1 > 1)
    {
        int motor_err_code = ms.status_m0 > 1 ? ms.status_m0 : ms.status_m1;
        if(motor_err_code == MOTOR_ERR_MOD)
        {
            logger->write_log("[DOCKING] failed (motor error MOD, 2)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_JAM)
        {
            logger->write_log("[DOCKING] failed (motor error JAM, 4)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_CUR)
        {
            logger->write_log("[DOCKING] failed (motor error CUR, 8)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_BIG)
        {
            logger->write_log("[DOCKING] failed (motor error BIG, 16)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_IN)
        {
            logger->write_log("[DOCKING] failed (motor error IN, 32)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_PSI)
        {
            logger->write_log("[DOCKING] failed (motor error:PS1|2, 64)", "Red", true, false);
        }
        else if(motor_err_code == MOTOR_ERR_NON)
        {
            logger->write_log("[DOCKING] failed (motor error NON, 128)", "Red", true, false);
        }
        return DRIVING_FAILED;
    }

    // if(ms.charge_state == 1)
    // {
    //     logger->write_log("[DOCKING] failed (robot charging)", "Red", true, false);
    //     return DRIVING_FAILED;
    // }

    if(ms.emo_state == 0)
    {
        logger->write_log("[DOCKING] not ready (emo pushed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    if(ms.status_m0 == 0 && ms.status_m1 == 0)
    {
        logger->write_log("[DOCKING] not ready (motor lock offed)", "Orange", true, false);
        return DRIVING_NOT_READY;
    }

    return DRIVING_FINE;
}

void DOCKING::dockControl( const Eigen::Matrix4d& cur_pose, double& linear_vel, double& angular_vel)
{

    Eigen::Vector3d cur_vel(mobile->vx0, mobile->vy0, mobile->wz0);
    const float dt = 0.02;

    double v0 = cur_vel(0);
    double y0 = cur_vel(1);
    double w0 = cur_vel(2);
    double v =0.0;
    double w =0.0;

    Eigen::Matrix4d local = Eigen::Matrix4d::Identity();

    //PID
    Eigen::Matrix4d dist = cur_pose.inverse()*docking_station_m;
    Eigen::Vector2d dtdr = dTdR(local, dist);


    double dist_x = dist(0,3);
    double dist_y = dist(1,3);
    double err_d = std::sqrt(dist_x*dist_x + dist_y*dist_y);
    double err_th = std::atan2(dist_y,dist_x);
    double d_err_th = 0.0;
    double d_err_v = 0.0;

    if(err_v_old)
    {
        d_err_th = (err_th - err_th_old) * dt;
        d_err_v = (err_d - err_v_old) * dt;
    }

    v = config->DOCKING_KP_d * err_d + config->DOCKING_KD_d * d_err_v;
    w = config->DOCKING_KP_th * err_th + config->DOCKING_KD_th * d_err_th;

    if(fsm_state == DOCKING_FSM_COMPENSATE)
    {
        err_th = std::atan2(dist(1,0),dist(0,0));
        w = config->DOCKING_KP_th * err_th + config->DOCKING_KD_th * d_err_th;
        v = 0.0;
    }

    v = saturation(v, v0- (limit_accel*dt) , v0 +(limit_accel*dt));
    w = saturation(w, w0-(limit_th_acc*D2R*dt), w0 +(limit_th_acc*D2R*dt));
    v = saturation(v, -limit_vel , limit_vel);
    w = saturation(w, -limit_th, limit_th);

    linear_vel = v;
    angular_vel = w;
}

KFRAME DOCKING::generateVKframe()
{
    KFRAME frame;
    XYZR_CLOUD res1;
    XYZR_CLOUD res2;
    XYZR_CLOUD res3;
    XYZR_CLOUD res4;

    //vdock
//    Eigen::Vector3d p1(config->ROBOT_SIZE_X[1] + config->DOCKING_DOCK_SIZE_X[1] + config->DOCKING_POINTDOCK_MARGIN, 0.0, 0.0);
//    Eigen::Vector3d p2(p1.x() - 0.05, p1.y() + 0.16248, 0.0);
//    Eigen::Vector3d p4(p2.x(), p2.y() + 0.03, 0.0);
//    Eigen::Vector3d p6(p4.x() + 0.18, p4.y(), 0.0);
//    Eigen::Vector3d p3(p1.x() - 0.05, p1.y() - 0.16248, 0.0);
//    Eigen::Vector3d p5(p3.x(), p3.y() - 0.03, 0.0);
//    Eigen::Vector3d p7(p5.x() + 0.18, p5.y(), 0.0);

//    res1 = generateSamplePoints(p4,p2,40);
//    res2 = generateSamplePoints(p2,p1,80);
//    res3 = generateSamplePoints(p1,p3,80);
//    res4 = generateSamplePoints(p3,p5,40);

    Eigen::Vector3d p1(config->ROBOT_SIZE_X[1]+config->DOCKING_POINTDOCK_MARGIN, 0.225, 0.0);
    Eigen::Vector3d p2(p1.x(), p1.y() - 0.125 ,0.0);
    Eigen::Vector3d p3(p2.x() + 0.07, p2.y() - 0.1 , 0.0);
    Eigen::Vector3d p4(p3.x() , p3.y() - 0.22 , 0.0);

    res1 = generateSamplePoints(p1,p2,40);
    res2 = generateSamplePoints(p2,p3,40);
    res3 = generateSamplePoints(p3,p4,40);

    for (const auto& pt : res1.pts)
    {
        frame.pts.push_back(pt);
        Eigen::Vector3d debug_pt(pt.x, pt.y, pt.z);
        debug_frame.push_back(debug_pt);
    }
    for (const auto& pt : res2.pts)
    {
        frame.pts.push_back(pt);
        Eigen::Vector3d debug_pt(pt.x, pt.y, pt.z);
        debug_frame.push_back(debug_pt);
    }
    for (const auto& pt : res3.pts)
    {
        frame.pts.push_back(pt);
        Eigen::Vector3d debug_pt(pt.x, pt.y, pt.z);
        debug_frame.push_back(debug_pt);
    }
//    for (const auto& pt : res4.pts)
//    {
//        frame.pts.push_back(pt);
//        Eigen::Vector3d debug_pt(pt.x, pt.y, pt.z);
//        debug_frame.push_back(debug_pt);
//    }

    return frame;
}

Eigen::Vector3d DOCKING::calculateCenter(const KFRAME& kframe)
{
    Eigen::Vector3d center(0.0, 0.0, 0.0);
    for (const auto& pt : kframe.pts)
    {
        center(0) += pt.x;
        center(1) += pt.y;
    }
    center /= static_cast<double>(kframe.pts.size());
    return center;
}

XYZR_CLOUD DOCKING::generateSamplePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n)
{
    // To do -- XYZR_CLOUD is heavy to use

    XYZR_CLOUD cloud;
    if (n < 2)
    {
        return cloud;
    }
    float dx = (p2.x() - p1.x()) / (n - 1);
    float dy = (p2.y() - p1.y()) / (n - 1);

    for (int i = 0; i < n; ++i)
    {
        PT_XYZR pt;
        pt.x = p1.x() + i * dx;
        pt.y = p1.y() + i * dy;
        pt.z = 0.0;
        pt.vx = 0.0;
        pt.vy = 0.0;
        pt.vz = 1.0;
        pt.r = 0.0;
        pt.k0 = 0;
        pt.k = 0;
        pt.do_cnt = 0;
        cloud.pts.push_back(pt);
    }
    return cloud;
}


Eigen::Matrix4d DOCKING::calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = to[0] - from[0];
    translationMatrix(1, 3) = to[1] - from[1];
    translationMatrix(2, 3) = to[2] - from[2];

    return translationMatrix;
}

double DOCKING::Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
{

    std::random_device rd;
    std::default_random_engine engine(rd());

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
    const int max_iter = 200;
    double lambda = 0.05; //0.01;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double cost_threshold = ICP_COST_THRESHOLD*ICP_COST_THRESHOLD;
    const int num_feature = std::min<int>(idx_list.size(),ICP_MAX_FEATURE_NUM);

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        std::shuffle(idx_list.begin(), idx_list.end(), engine);
        // std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());

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
                const int pt_num = 5;
                std::vector<unsigned int> ret_near_idxs(pt_num);
                std::vector<double> ret_near_sq_dists(pt_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                nn_idx = ret_near_idxs[0];
                for(int q = 0; q < pt_num; q++)
                {
                    int idx = ret_near_idxs[q];
                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                }
                P0 /= pt_num;
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

            double cost = (_P1 - P0).squaredNorm();
            if(cost > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + 0.01)
            {
                continue;
            }

            // jacobian
            Eigen::Vector3d xi = TF_to_se2(_dG);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!std::isfinite(J[0]) || !std::isfinite(J[1]) || !std::isfinite(J[2]))
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
        if(num_correspondence < 10)
        {
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
    dG = _dG*dG;

    if(last_err > first_err+0.01 || last_err > 0.2)
    {
        return 9999;
    }
    return last_err;
}


bool DOCKING::sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out)
{
    std::vector<std::vector<Eigen::Vector3d>> candidate;

    if(in.size() ==0) return false;

    //dist
    float min_dist = 9999.;
    int min_idx = 0;
    int idx =0;

    for(auto& clusters : in)
    {
        float sum_x=0.0;
        float sum_y=0.0;

        for(auto& point : clusters)
        {
            sum_x += point.x();
            sum_y += point.y();
        }
        float c_x = sum_x / clusters.size();
        float c_y = sum_y / clusters.size();
        float dist =std::sqrt(c_x*c_x + c_y*c_y);

        if(dist < min_dist)
        {
            min_dist = dist ;
            min_idx = idx;
        }

        idx++;
    }

    out = in[min_idx];

    return true;

    if(candidate.size() == 0) return false;
    if(candidate.size() == 1)
    {
        out = candidate[0];
        return true;
    }

    else
    {
        double error_min =999.;
        int idx =0;
        int min_idx=0;
        for(auto& clust : candidate)
        {
            KFRAME cur_frm;
            for (const auto& point : clust)
            {
                PT_XYZR pt;
                pt.x = point.x();
                pt.y = point.y();
                pt.z = 0.0;
                pt.vx = 0.0;
                pt.vy = 0.0;
                pt.vz = 1.0;
                pt.r = 0.0;
                pt.k0 = 0;
                pt.k = 0;
                pt.do_cnt = 0;
                cur_frm.pts.push_back(pt);
            }

            Eigen::Vector3d frm0_center0 = calculateCenter(cur_frm);
            Eigen::Matrix4d temp_tf = Eigen::Matrix4d::Identity();
            temp_tf = calculateTranslationMatrix(frm1_center, frm0_center0);
            double err = Vfrm_icp(cur_frm, Vfrm, temp_tf);
            if(err < error_min)
            {
                error_min = err;
                min_idx = idx;
            }
            idx++;
        }

        out = candidate[min_idx];
        return true;

    }

    return false;
}

std::vector<Eigen::Vector3d> DOCKING::get_cur_clust()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> debug_p = debug_frame;
    mtx.unlock();

    return debug_p;
}


void DOCKING::b_loop()
{
    const double dt = 0.02; // 50hz
    double undock_odom = 0.0;

    while(b_flag)
    {
        double pre_loop_time = get_time();
        pre_loop_time = get_time();


        if(fsm_state == DOCKING_FSM_UNDOCK)
        {
            double t = std::abs(config->DOCKING_POINTDOCK_MARGIN /0.1) + 0.5;

            if(get_time() - undock_time > t )
            {
                printf("[DOCKING] UNDOCK SUCCESS\n");
                Q_EMIT signal_undock_succeed("");
                fsm_state == DOCKING_FSM_OFF;
            }
        }

        else if (fsm_state ==DOCKING_FSM_OFF)
        {


        }
        else
        {
            printf("[DOCKING] UNDOCK FAILED\n");
            Q_EMIT signal_undock_failed(failed_reason);
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
    }

}
bool DOCKING::undock()
{
        int is_good_everything = is_everything_fine();

        if(is_good_everything == DRIVING_FAILED)
        {
            mobile->move(0, 0, 0);
            printf("[DOCKING] something wrong (failed)\n");
            return false;
        }

        else if(is_good_everything == DRIVING_NOT_READY)
        {
            mobile->move(0, 0, 0);
            printf("[DOCKING] something wrong (not ready)\n");
            return false;
        }

        else
        {    
            // obs clear
            obsmap->clear();
            // start control loop
            b_flag = true;
            // start docking control loop
            fsm_state = DOCKING_FSM_UNDOCK;
            b_thread = new std::thread(&DOCKING::b_loop, this);
            mobile->move_linear(-1*config->DOCKING_POINTDOCK_MARGIN, 0.1);
            undock_time = get_time();
            return true;
        }


}
