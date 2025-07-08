#include "dockcontrol.h"

DOCKCONTROL* DOCKCONTROL::instance(QObject *parent)
{
    static DOCKCONTROL* inst = nullptr;
    if(!inst && parent)
    {
        inst = new DOCKCONTROL(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }

    return inst;
}

DOCKCONTROL::DOCKCONTROL(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    lidar_2d(nullptr),
    qr_sensor(nullptr),
    obsmap(nullptr)
{

}

DOCKCONTROL::~DOCKCONTROL()
{

    if(a_thread && a_thread->joinable())
    {
        a_flag = false;
        a_thread->join();
    }
    a_thread.reset();

    if(b_thread && b_thread->joinable())
    {
        b_flag = false;
        b_thread->join();
    }
    b_thread.reset();
}

void DOCKCONTROL::init()
{
    fsm_state = DOCKING_FSM_OFF;
}

void DOCKCONTROL::move()
{

    mobile->set_detect_mode(2);

    if(config->get_docking_type() == 0) // 0 - lidar docking
    {
        //stop first
        stop();

        //obs clear -- delete renew
        
        //set flags
        a_flag = true;
        undock_flag = false;
        path_flag = false; // for pointdock pathflag
        dock = false; // for pointdock pdu linear move flag

        vfrm = generate_vkframe(1); // generate aline vmark frame
        frm1_center = calculate_center(vfrm);

        oneque_vfrm = generate_vkframe(0); // generate oneque vmark frame
        oneque_frm1_center = calculate_center(oneque_vfrm);

        //odometry coordinate set
        Eigen::Vector3d odom_pose = mobile->get_pose().pose;
        odom_start_tf = se2_to_TF(odom_pose);

        // start docking control loop
        fsm_state = DOCKING_FSM_CHKCHARGE;
        a_thread = std::make_unique<std::thread>(&DOCKCONTROL::a_loop, this);
    }
}

void DOCKCONTROL::a_loop()
{
    const double dt = 0.02; // 50hz
    double obs_wait_st_time = 0.0;
    double wait_start_time = 0.0;
    double vmark_start_time = get_time();

    unsigned int normal_dock_cnt =0;
    unsigned int oneque_dock_cnt =0;
    unsigned int notfind_dock_cnt =0;

    bool middle_set = false;
    Eigen::Matrix4d middle_point_odom;
    bool final_dock =false;

    while(a_flag)
    {

        double pre_loop_time = get_time();

        //use odom frame
        Eigen::Vector3d odom_pose = mobile->get_pose().pose;
        Eigen::Matrix4d cur_pos_odom = se2_to_TF(odom_pose);

        int is_good_everything = is_everything_fine();

        if(is_good_everything == DRIVING_FAILED)
        {
            failed_reason = "FIND CHECK WRONG";
            fsm_state = DOCKING_FSM_FAILED;
        }

        if(fsm_state == DOCKING_FSM_CHKCHARGE)
        {
            int l_dock_type = check_oneque();

            if(l_dock_type == 0) // NORMAL
            {
                normal_dock_cnt++;
                oneque_dock_cnt =0;
                notfind_dock_cnt =0;
            }
            else if(l_dock_type == 1) //ONE QUE
            {
                normal_dock_cnt =0;
                oneque_dock_cnt ++;
                notfind_dock_cnt =0;
            }
            else if (l_dock_type == -1)
            {
                normal_dock_cnt =0;
                oneque_dock_cnt =0;
                notfind_dock_cnt ++;
            }

            if(normal_dock_cnt > 10)
            {
                fsm_state = DOCKING_FSM_POINTDOCK;
                oneque_dock = false;
            }
            if(oneque_dock_cnt > 10)
            {
                fsm_state = DOCKING_FSM_POINTDOCK;
                oneque_dock = true;
            }
            if(notfind_dock_cnt > 10)
            {
                failed_reason = "FIND CHECK WRONG";
                fsm_state = DOCKING_FSM_FAILED;
            }
        }

        else if(fsm_state == DOCKING_FSM_POINTDOCK)
        {
            double cmd_v =0.0;
            double cmd_w =0.0;

            if(oneque_dock)
            {
                Eigen::Matrix4d target_tf;

                Eigen::Matrix4d vmark_tf;
                Eigen::Matrix4d middle_point;

                vmark_tf = find_vmark();

                double dist = std::sqrt(std::pow(vmark_tf(0, 3), 2) + std::pow(vmark_tf(1, 3), 2));
                double theta = std::atan2(vmark_tf(1, 0), vmark_tf(0, 0));


                //dock check
                if(dist < config->get_docking_goal_dist())  //&& theta < config->DOCKING_GOAL_TH)
                {
                    mobile->move(0,0,0);
                    fsm_state = DOCKING_FSM_WAIT;
                    wait_start_time = get_time();
                    continue;

                }


                //first middle point set
                if(!middle_set)
                {
                    middle_set = true;
                    middle_point = straight_path(vmark_tf);
                    middle_point_odom = se2_to_TF(mobile->get_pose().pose) * middle_point;
                }


                // target check logic/////////////////////////////////////////////////////
                if(!final_dock)
                {
                    Eigen::Vector3d odom_pose = mobile->get_pose().pose;
                    Eigen::Matrix4d cur_pos_odom = se2_to_TF(odom_pose);

                    //middle point - from odom to base frame
                    target_tf = cur_pos_odom * middle_point;
                    
                    //midlle point arrive check
                    double mid_dist = std::sqrt(std::pow(target_tf(0, 3), 2) + std::pow(target_tf(1, 3), 2));
                    double mid_theta = std::atan2(target_tf(1, 0), target_tf(0, 0));

                    if(mid_dist < config->get_docking_goal_dist() && mid_theta < config->get_docking_goal_th())
                    {
                        final_dock = true;
                    }
                }

                else
                {
                    target_tf = vmark_tf;

                }
                //////////////////////////////////////////////////////////////////////////
    
                pointdockControl(final_dock,target_tf,cmd_v,cmd_w);
                mobile->move(cmd_v,0,cmd_w);
                
            }

            else
            {
                find_vmark();
                if(path_flag)
                {
                    // use odom_frame
                    Eigen::Vector2d dtdr = dTdR(cur_pos_odom,docking_station_o);

                    if(std::abs(dtdr(0)) < config->get_docking_goal_dist())
                    {
                        mobile->move(0,0,0);
                        fsm_state = DOCKING_FSM_COMPENSATE;
                        continue;
                    }
                    dockControl(cur_pos_odom,cmd_v,cmd_w);
                    mobile->move(cmd_v,0,cmd_w);
                }

                else
                {
                    if(get_time() - vmark_start_time > 10.0)
                    {
                        DATA_DOCK ddock;
                        ddock.command = "dock";
                        ddock.result = "fail";
                        ddock.message = failed_reason;
                        ddock.time = get_time();
                        // Q_EMIT signal_dock_response(ddock);
                    }
                }
            }

        }

        else if(fsm_state == DOCKING_FSM_COMPENSATE)
        {
            double cmd_v =0.0;
            double cmd_w =0.0;

            find_vmark();

            if(path_flag)
            {
                // use map_frame
                //Eigen::Vector2d dtdr = dTdR(cur_pos, docking_station_m);

                // use odom_frame
                Eigen::Vector2d dtdr = dTdR(cur_pos_odom,docking_station_o);

                if(std::abs(dtdr(1)) < config->get_docking_goal_th())
                {
                    mobile->move(0,0,0);
                    fsm_state = DOCKING_FSM_DOCK;
                    continue;
                }
                dockControl(cur_pos_odom,cmd_v,cmd_w);
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
                mobile->move_linear_x(config->get_docking_pointdock_margin() + 0.028 , 0.05);
                dock = true;
                fsm_state = DOCKING_FSM_WAIT;
                wait_start_time = get_time();
            }

        }

        else if (fsm_state == DOCKING_FSM_WAIT)
        {
            MOBILE_STATUS ms = mobile->get_status();

            mobile->move(0, 0, 0);

            if (ms.charge_state == 3 && ms.cur_m0 < 60 && ms.cur_m1 < 60)
            {
                fsm_state = DOCKING_FSM_COMPLETE;

                DATA_DOCK ddock;
                ddock.command = "dock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                // Q_EMIT signal_dock_response(ddock);
            }

            else if (get_time() - wait_start_time > 8.0)
            {
                if(!undock_flag)
                {
                    undock_flag = true;
                    undock_waiting_time = get_time();
                    mobile->move_linear_x(-1*config->get_robot_size_x_max(), 0.05);
                }
                else
                {
                    double t = std::abs(config->get_robot_size_x_max() /0.05) + 0.5;
                    if(get_time() - undock_waiting_time > t)
                    {
                        qDebug() << "no charge[charge_state,cur_m0,cur_m1]" << ms.charge_state << ms.cur_m0 << ms.cur_m1;
                        failed_reason = "NOT CONNECTED";

                        DATA_DOCK ddock;
                        ddock.command = "dock";
                        ddock.result = "fail";
                        ddock.message = failed_reason;
                        ddock.time = get_time();

                        // Q_EMIT signal_dock_response(ddock);
                        fsm_state = DOCKING_FSM_FAILED;
                    }
                }
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
            //printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }

    
}

void DOCKCONTROL::b_loop()
{
    const double dt = 0.02; // 50hz
    double undock_odom = 0.0;

    while(b_flag)
    {
        double pre_loop_time = get_time();
        pre_loop_time = get_time();


        if(fsm_state == DOCKING_FSM_UNDOCK)
        {
            double t = std::abs((config->get_robot_size_x_max() +0.3) / 0.1) + 0.5;

            if(get_time() - undock_time > t )
            {
                printf("[DOCKING] UNDOCK SUCCESS\n");

                DATA_DOCK ddock;
                ddock.command = "undock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                // Q_EMIT signal_dock_response(ddock);
                fsm_state = DOCKING_FSM_OFF;
            }
        }

        else if (fsm_state ==DOCKING_FSM_OFF)
        {


        }
        else
        {
            printf("[DOCKING] UNDOCK FAILED\n");

            DATA_DOCK ddock;
            ddock.command = "undock";
            ddock.result = "fail";
            ddock.message = failed_reason;
            ddock.time = get_time();

            // Q_EMIT signal_dock_response(ddock);
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
            //printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
    }
}

void DOCKCONTROL::stop()
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

    if(config->get_docking_type() == 0)
    {
        undock_flag = false;
        path_flag = false;
    }

    mobile->move(0, 0, 0);
    is_moving = false;
    is_pause = false;

}

void DOCKCONTROL::set_config_module(CONFIG* _config)
{
    config = _config;
}

void DOCKCONTROL::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void DOCKCONTROL::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void DOCKCONTROL::set_lidar_2d_module(LIDAR_2D* _lidar_2d)
{
    lidar_2d = _lidar_2d;
}

void DOCKCONTROL::set_qr_sensor_module(QR_SENSOR* _qr_sensor)
{
    qr_sensor = _qr_sensor;
}

void DOCKCONTROL::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}


Eigen::Matrix4d DOCKCONTROL::find_vmark()
{
    Eigen::Matrix4d out_;

    std::vector<std::vector<Eigen::Vector3d>>filtered_clusters;
    std::vector<Eigen::Vector3d> c_lidar;

    if(oneque_dock)
    {
        c_lidar = lidar_2d->get_cur_raw(0).pts;
    }
    else
    {
        //front_back merge
       c_lidar = lidar_2d->get_cur_raw(0).pts;

       const auto& back_pts = lidar_2d->get_cur_raw(1).pts;
       c_lidar.insert(c_lidar.end(), back_pts.begin(), back_pts.end());
    }

    std::vector<Eigen::Vector3d> filtered_lidar;

    //ROI
    for(unsigned int i=0; i<c_lidar.size(); i++)
    {
        double x = static_cast<double>(c_lidar[i].x());
        double y = static_cast<double>(c_lidar[i].y());

        double angle = std::atan2(y,x);
        double dist = std::sqrt(pow(x, 2) + pow(y, 2));

        if(std::abs(angle) < config->get_docking_clust_angle_threshold())
        {
            if(dist < config->get_docking_clust_dist_threshold_max() )
            {
                filtered_lidar.push_back(c_lidar[i]);
            }
        }
    }

    const int c_points = filtered_lidar.size();
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2));

    for(unsigned int i=0; i < c_points; ++i)
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

    while(i < c_points && flag == true)
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

        // TODO -- CLUSTER SIZE LOGIC
        // if(num > clust_size_min && num < clust_size_max) g_clusters.push_back(cluster);
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

        if( d > config->get_robot_size_x_max() && d < config->get_docking_find_vmark_dist_threshold_max())
        {
            filtered_clusters.push_back(clust);
        }
    }

    std::vector<Eigen::Vector3d> docking_clust; // finally docking cluster

    // Todo - Test check candidate->sizefilter
    if(!sizefilter(filtered_clusters,docking_clust))
    {
        return out_;
    }

    if(oneque_dock)
    {

        if(clusters_queue.size() >=10)
        {
            clusters_queue.pop();
        }


        clusters_queue.push(docking_clust);

        if(clusters_queue.size() == 10)
        {
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

            Eigen::Vector3d frm0_center0 = calculate_center(cur_frm);
            Eigen::Matrix4d dock_tf = Eigen::Matrix4d::Identity();
            double err = 0.0;

            dock_tf = calculate_translation_matrix(oneque_frm1_center, frm0_center0);
            err = vfrm_icp(cur_frm, oneque_vfrm, dock_tf);

            if(err > config->get_docking_icp_err_threshold()) //CHANGE
            {
                return out_;
            }

            out_ = dock_tf;

            return out_;
        }
    
    }
    else
    {
        clusters_queue.push(docking_clust);

        if(clusters_queue.size() > 10)
        {
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

            Eigen::Vector3d frm0_center0 = calculate_center(cur_frm);
            Eigen::Matrix4d dock_tf = Eigen::Matrix4d::Identity();
            double err = 0.0;


            dock_tf = calculate_translation_matrix(frm1_center, frm0_center0);
            err = vfrm_icp(cur_frm, vfrm, dock_tf);

            if(err > config->get_docking_icp_err_threshold()) //CHANGE
            {
                return out_;
            }


            docking_station = dock_tf;

//            Eigen::Matrix4d cur_pos = slam->get_cur_tf();
//            docking_station_m = cur_pos * docking_station;

            //odom_start_tf : odom->base_link transformation
            docking_station_o = se2_to_TF(mobile->get_pose().pose) * docking_station;

            path_flag = true;

            return out_;
        }
    }

    return out_;
}

KFRAME DOCKCONTROL::generate_vkframe(int type)
{
    KFRAME frame;

    XYZR_CLOUD res1;
    XYZR_CLOUD res2;
    XYZR_CLOUD res3;
    XYZR_CLOUD res4;

    if(type == 0) //oneque vframe
    {
        Eigen::Vector3d p1(config->get_robot_size_x_max(), 0.225, 0.0);
        Eigen::Vector3d p2(p1.x(), p1.y() - 0.125 ,0.0);
        Eigen::Vector3d p3(p2.x() + 0.07, p2.y() - 0.1 , 0.0);
        Eigen::Vector3d p4(p3.x() , p3.y() - 0.22 , 0.0);

        res1 = generate_sample_points(p1,p2,40);
        res2 = generate_sample_points(p2,p3,40);
        res3 = generate_sample_points(p3,p4,40);

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
    }

    else if (type == 1) //aline vfrmae
    {
        Eigen::Vector3d p1(config->get_robot_size_x_max() + config->get_docking_pointdock_margin(), 0.225, 0.0);
        Eigen::Vector3d p2(p1.x(), p1.y() - 0.125 ,0.0);
        Eigen::Vector3d p3(p2.x() + 0.07, p2.y() - 0.1 , 0.0);
        Eigen::Vector3d p4(p3.x() , p3.y() - 0.22 , 0.0);

        res1 = generate_sample_points(p1,p2,40);
        res2 = generate_sample_points(p2,p3,40);
        res3 = generate_sample_points(p3,p4,40);

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
    }

    return frame;
}

Eigen::Vector3d DOCKCONTROL::calculate_center(const KFRAME& kframe)
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

double DOCKCONTROL::vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
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
    double lambda = 0.01; //0.05 -- original version
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


bool DOCKCONTROL::is_everything_fine()
{
    // ToDo : safety function check
    MOBILE_STATUS mobile_status = mobile->get_status();

    if(mobile_status.charge_state == 1)
    {
        logger->write_log("[DOCK] dock failed (charging)", "Red", true, false);
        return false;
    }

    if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1)
    {
        logger->write_log("[DOCK] dock failed (motor not connected)", "Red", true, false);
        return false;
    }

    if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1)
    {
        logger->write_log("[DOCK] dock failed (emo pushed)", "Red", true, false);
        return false;
    }

    if(mobile_status.status_m0 == 0 || mobile_status.status_m1 == 0)
    {
        logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
        return false;
    }
 
    return true;
}


int DOCKCONTROL::check_oneque()
{
    std::vector<std::vector<Eigen::Vector3d>>filtered_clusters;
    std::vector<Eigen::Vector3d> c_lidar;

    //front_back merge
    c_lidar = lidar_2d->get_cur_raw(0).pts;

    const auto& back_pts = lidar_2d->get_cur_raw(1).pts;
    c_lidar.insert(c_lidar.end(), back_pts.begin(), back_pts.end());
    

    std::vector<Eigen::Vector3d> filtered_lidar;

    //ROI
    for(unsigned int i=0; i<c_lidar.size(); i++)
    {
        double x = static_cast<double>(c_lidar[i].x());
        double y = static_cast<double>(c_lidar[i].y());

        double angle = std::atan2(y,x);
        double dist = std::sqrt(pow(x, 2) + pow(y, 2));
        if(std::abs(angle) < config->get_docking_clust_angle_threshold())
        {
            if(dist < config->get_docking_clust_dist_threshold_max())
            {
                
                filtered_lidar.push_back(c_lidar[i]);
            }
        }
    }

    const int c_points = filtered_lidar.size();
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2));

    for(unsigned int i=0; i < c_points; ++i)
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

    while(i < c_points && flag == true)
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

        // change size filter
        // if(num > clust_size_min && num < clust_size_max) g_clusters.push_back(cluster);
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

        // threshold min have to be bigger than robot size x
        if( d > config->get_robot_size_x_max() + 0.03 && d < config->get_docking_find_vmark_dist_threshold_max())
        {
            filtered_clusters.push_back(clust);
        }
    }

    std::vector<Eigen::Vector3d> docking_clust;

    if(!sizefilter(filtered_clusters,docking_clust))
    {
        return 3;
    }

    clusters_queue.push(docking_clust);

    if(clusters_queue.size() > 20) // first hard check
    {
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

        Eigen::Vector3d frm0_center0 = calculate_center(cur_frm);
        Eigen::Matrix4d dock_tf = Eigen::Matrix4d::Identity();
        double err = 0.0;

        dock_tf = calculate_translation_matrix(frm1_center, frm0_center0);
        err = vfrm_icp(cur_frm, vfrm, dock_tf);

        if(err > config->get_docking_icp_err_threshold()) //0.002
        {
            return -1; // NOT FIND
        }

        double x = dock_tf(0, 3);
        double y = dock_tf(1, 3);
        double head_th = std::atan2(y,x);
        double theta = std::atan2(dock_tf(1, 0), dock_tf(0, 0));

        double dist = sqrt( x*x + y*y );
        double head_err_th = fabs(head_th*180 / M_PI);
        double orein_err_th = fabs(theta*180.0 /M_PI);

        if(orein_err_th <=10.0 && head_err_th <=10.0)
        {
            return 1; // ONEQUE DOCK CNT
        }

        else
        {
            return 0; // NORMAL DOCK CNT
        }

    }

    return 3; // NO VMARK
}

void DOCKCONTROL::pointdockControl(bool final_dock, const Eigen::Matrix4d& err_tf , double& cmd_v, double& cmd_w)
{
    //last control input -- Todo maybe use observation output?
    Eigen::Vector3d cur_vel = mobile->get_control_input();

    const float dt =0.02;

    double v0 = cur_vel(0);
    double y0 = cur_vel(1);
    double w0 = cur_vel(2);
    double v =0.0;
    double w =0.0;


//------------------point--------------------------------------------------------------
    double dx = err_tf(0,3);
    double dy = err_tf(1,3);
    double dist = std::sqrt(dx*dx + dy*dy);
    double heading_err = std::atan2(dy,dx);
    double orien_err = std::atan2(err_tf(1,0),err_tf(0,0));


    double dist_min = 0.05;
    double dist_max = 1.5;
    double clamped_dist = std::min(std::max(dist, dist_min), dist_max);

    double alpha = (clamped_dist - dist_min) / (dist_max - dist_min);
    double beta = 1.0 - alpha;



    v = config->get_docking_k_rho()*dist;
    w = config->get_docking_k_alpha()*alpha*heading_err + config->get_docking_k_beta()*beta*orien_err;



    v = saturation(v, v0- (limit_accel*dt) , v0 +(limit_accel*dt));
    w = saturation(w, w0-(limit_th_acc*D2R*dt), w0 +(limit_th_acc*D2R*dt));
    v = saturation(v, -oneque_limit_vel , oneque_limit_vel);
    w = saturation(w, -limit_th, limit_th);
    
    if(final_dock)
    {
        if(dist <= config->get_robot_size_x_max() +0.05)
        {
            v = 0.05;
            w = 0.0;
        }
    }

    cmd_v = v;
    cmd_w = w;
}


void DOCKCONTROL::dockControl(const Eigen::Matrix4d& cur_pose, double& linear_vel, double& angular_vel)
{

    Eigen::Vector3d cur_vel = mobile->get_control_input();
    
    const float dt = 0.02;

    double v0 = cur_vel(0);
    double y0 = cur_vel(1);
    double w0 = cur_vel(2);
    double v =0.0;
    double w =0.0;

    Eigen::Matrix4d local = Eigen::Matrix4d::Identity();

    //use odom frame
    Eigen::Matrix4d dist = cur_pose.inverse()*docking_station_o;
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

    v = config->get_docking_kp_dist() * err_d + config->get_docking_kd_dist() * d_err_v;
    w = config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th;

    if(fsm_state == DOCKING_FSM_COMPENSATE)
    {
        err_th = std::atan2(dist(1,0),dist(0,0));
        w = config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th;
        v = 0.0;
    }

    v = saturation(v, v0- (limit_accel*dt) , v0 +(limit_accel*dt));
    w = saturation(w, w0-(limit_th_acc*D2R*dt), w0 +(limit_th_acc*D2R*dt));
    v = saturation(v, -limit_vel , limit_vel);
    w = saturation(w, -limit_th, limit_th);

    linear_vel = v;
    angular_vel = w;
}

Eigen::Matrix4d DOCKCONTROL::calculate_translation_matrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = to[0] - from[0];
    translationMatrix(1, 3) = to[1] - from[1];
    translationMatrix(2, 3) = to[2] - from[2];

    return translationMatrix;
}

XYZR_CLOUD DOCKCONTROL::generate_sample_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n)
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

bool DOCKCONTROL::sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out)
{
    const float len_error_tolerance = 0.1; // 10cm

    std::vector<std::vector<Eigen::Vector3d>> candidate;

    if(in.size() ==0) return false;

    //dist
    std::vector<float> lengths;

    for(auto& clusters : in)
    {
        float total_length = 0.0;

        for(int i =1; i< clusters.size(); i++)
        {
            float dx = clusters[i].x() - clusters[i - 1].x();
            float dy = clusters[i].y() - clusters[i - 1].y();

            total_length += std::sqrt(dx *dx + dy* dy);
        }
        lengths.push_back(total_length);
    }

    int best_idx = -1;
    float min_error = std::numeric_limits<float>::max();

    double _charge_length = config->get_docking_chg_length();

    for (int i = 0; i < lengths.size(); i++)
    {
        float error = std::abs(lengths[i] - _charge_length);

        if (error < min_error)
        {
            min_error = error;
            best_idx = i;
        }
    }

    if(min_error > len_error_tolerance || best_idx <0)
    {
        return false;
    }

    out = in[best_idx];

    return true;
}


Eigen::Matrix4d DOCKCONTROL::straight_path(const Eigen::Matrix4d& end_pose)
{
    Eigen::Matrix4d out_;

    double offset = 0.5;
    Eigen::Vector3d back_dir = -end_pose.block<3,1>(0,0);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3,3>(0,0) = end_pose.block<3,3>(0,0);
    pose.block<3,1>(0,3) = end_pose.block<3,1>(0,3) + back_dir * offset;

    out_ = pose;

    return out_;
}

bool DOCKCONTROL::undock()
{

    int is_good_everything = is_everything_fine();

    if(is_good_everything == DRIVING_FAILED)
    {
        mobile->move(0, 0, 0);
        printf("[DOCKING] something wrong (failed)\n");
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
        b_thread = std::make_unique<std::thread>(&DOCKCONTROL::b_loop, this);

        mobile->stop_charge();
        mobile->move_linear_x(-1*(config->get_robot_size_x_max()), 0.05);
        undock_time = get_time();
        return true;
    }
}
