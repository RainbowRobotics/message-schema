#include "dockcontrol.h"

namespace
{
    const char* MODULE_NAME = "DOCK";
}

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
    if(config->get_docking_type() == 0) // 0 - lidar docking
    {
        reverse_mode = config->get_docking_reverse_mode();
        DWA_TABLE.clear();

        if(reverse_mode)
        {
            DWA_TABLE = generate_dwa_traj_table(-0.1, -0.05, 0.01, -10.0, 10.0, 1.0, 0.02, 500); // 0.02 - 500
        }
        else
        {
            DWA_TABLE = generate_dwa_traj_table(0.05 ,0.1, 0.01, -10.0, 10.0, 1.0, 0.02, 500); // 0.02 - 500
        }

        //stop first
        stop();

        //obs clear -- delete renew

        //set flags
        a_flag = true;
        undock_flag = false;
        path_flag = false; // for pointdock pathflag
        dock = false; // for pointdock pdu linear move flag
        failed_flag = false;
        dock_retry_flag = false;
        find_check = false;

        first_aline = Eigen::Matrix4d::Identity();

        bool reverse_flag = false;
        reverse_flag = config->get_docking_reverse_mode();

        vfrm = generate_vkframe(1, reverse_flag); // generate aline vmark frame
        frm1_center = calculate_center(vfrm);

        oneque_vfrm = generate_vkframe(0, reverse_flag); // generate oneque vmark frame
        oneque_frm1_center = calculate_center(oneque_vfrm);

        // start docking control loop
        fsm_state = DOCKING_FSM_CHKCHARGE;
        a_thread = std::make_unique<std::thread>(&DOCKCONTROL::a_loop, this);
    }

    return;
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

    bool final_dock =false;
    bool findvmark_toggle = false;

    bool ycompensate_aline = false;
    std::vector<double> dtdr_mean;

    //control parameter setting
    err_th_old = 0.0;
    err_v_old = 0.0;

    log_info("docking_loop start");
    while(a_flag)
    {
        double pre_loop_time = get_time();

        //use odom frame
        Eigen::Vector3d odom_pose = mobile->get_pose().pose;
        Eigen::Matrix4d cur_pos_odom = se2_to_TF(odom_pose);

        int is_good_everything = is_everything_fine();

        if(is_good_everything == DRIVING_FAILED)
        {
            failed_flag = true;
            failed_reason = "IS_GOOD_ERROR";
            fsm_state = DOCKING_FSM_FAILED;
        }

        if(fsm_state == DOCKING_FSM_CHKCHARGE)
        {
            double check_time = get_time();

            int l_dock_type;
            find_vmark(l_dock_type);

            if(l_dock_type == 0) // NOT FOUND
            {
                normal_dock_cnt =0;
                oneque_dock_cnt =0;
                notfind_dock_cnt ++;
            }
            else if(l_dock_type == 1) //ONE QUE
            {
                oneque_dock_cnt ++;
                normal_dock_cnt =0;
                notfind_dock_cnt =0;
            }
            else if (l_dock_type == 3)
            {
                normal_dock_cnt =0;
                oneque_dock_cnt =0;
                notfind_dock_cnt ++; // 0.2s count
            }

            if(normal_dock_cnt > 5)
            {
                fsm_state = DOCKING_FSM_POINTDOCK;
                oneque_dock = false;
            }
            if(oneque_dock_cnt > 5)
            {
                fsm_state = DOCKING_FSM_POINTDOCK;
                oneque_dock = true;
            }
            if(notfind_dock_cnt * 0.2 > config->get_docking_waiting_time())
            {
                failed_flag = true;
                failed_reason = "FIND CHECK WRONG";
                fsm_state = DOCKING_FSM_FAILED;
            }
        }

        else if(fsm_state == DOCKING_FSM_POINTDOCK)
        {
            double cmd_v = 0.0;
            double cmd_w = 0.0;
            double cmd_v_y = 0.0;

            if(oneque_dock)
            {
                Eigen::Matrix4d target_tf;
                Eigen::Matrix4d vmark_tf;

                int t;
                vmark_tf = find_vmark(t); // vamrk_tf -> base_frame

                if(path_flag)
                {

                    Eigen::Vector2d dtdr = dTdR(cur_pos_odom,docking_station_o);

                    Eigen::Matrix4d rel_tf = cur_pos_odom.inverse() * docking_station_o;

                    double dist_x = rel_tf(0, 3);  // 로봇 기준 x 방향 거리
                    double dist_y = rel_tf(1, 3);  // 로봇 기준 y 방향 거리
                    Eigen::Matrix4d station_rel_tf = docking_station_o.inverse() * cur_pos_odom;

                    double s_d_x = station_rel_tf(0, 3);  // 도킹스테이션 기준 x 위치
                    double s_d_y = station_rel_tf(1, 3);  // 도킹스테이션 기준 y 위치


                    if(config->get_robot_wheel_type() == "DD")
                    {
                        if(dtdr(0) < 0.35)
                        {
                            final_dock = true;
                            first_aline = cur_pos_odom;
                        }

                        if(dtdr(0) < config->get_docking_goal_dist())
                        {
                            mobile->move(0,0,0);

                            Eigen::Matrix4d target_tf = cur_pos_odom.inverse()*docking_station_o;

                            //calculate dist_y in base_frame
                            double dist_x = target_tf(0,3);
                            double dist_y = target_tf(1,3);

                            fsm_state = DOCKING_FSM_COMPENSATE;
                            wait_start_time = get_time();
                            continue;
                        }
                    }

                    else if(config->get_robot_wheel_type() == "MECANUM")
                    {
                        final_dock = true;

                        if(dtdr(0) < config->get_docking_goal_dist())
                        {
                            mobile->move(0,0,0);

                            Eigen::Matrix4d target_tf = cur_pos_odom.inverse()*docking_station_o;

                            //calculate dist_y in base_frame
                            double dist_x = target_tf(0,3);
                            double dist_y = target_tf(1,3);

                            fsm_state = DOCKING_FSM_COMPENSATE;
                            wait_start_time = get_time();
                            continue;
                        }
                    }

                    dockControl(final_dock,cur_pos_odom,cmd_v,cmd_w,cmd_v_y);
                    mobile->move(cmd_v,cmd_v_y,cmd_w);
                }
            }

        }

        else if(fsm_state == DOCKING_FSM_YCOMPENSATE)
        {
            Eigen::Vector2d dtdr = dTdR(cur_pos_odom,first_aline);

            //not used
        }

        else if(fsm_state == DOCKING_FSM_COMPENSATE)
        {
            double cmd_v =0.0;
            double cmd_w =0.0;
            double cmd_y =0.0;

            Eigen::Matrix4d target_tf;

            int t;
            find_vmark(t);


            if(path_flag)
            {
                // use odom_frame
                Eigen::Vector2d dtdr = dTdR(cur_pos_odom,docking_station_o);

                if(std::abs(dtdr(1)) < config->get_docking_goal_th())
                {
                    mobile->move(0,0,0);
                    fsm_state = DOCKING_FSM_DOCK;
                    path_flag = false;
                    find_check = false;
                    continue;
                }
                dockControl(0,cur_pos_odom,cmd_v,cmd_w,cmd_y);
                mobile->move(cmd_v,0.0,cmd_w);
            }
        }

        else if(fsm_state == DOCKING_FSM_DOCK)
        {
            int t;
            find_vmark(t);
            Eigen::Vector2d dtdr = dTdR(cur_pos_odom,docking_station_o);

            if(path_flag)
            {
                if(find_check)
                {
                    dtdr_mean.push_back(dtdr(0));
                    find_check = false;
                }


                if (dtdr_mean.size() >= 10)
                {
                    double sum = 0.0;

                    for (double v : dtdr_mean)
                    {
                        sum += v;
                    }
                    double avg = sum / dtdr_mean.size();

                    if (dock)
                    {
                        if(get_time() - wait_start_time > dock_success_wait_time + 1.5)
                        {
                            wait_start_time = get_time();
                            fsm_state = DOCKING_FSM_WAIT;

                            if(config->get_charge_type() == "XNERGY")
                            {
                                int non_used_int = 0;
                                mobile->xnergy_command(0, non_used_int);

                                spdlog::info("[DOCKING] xnergy command charge start");
                            }
                        }
                    }
                    else
                    {
                        dock = true;
                        double linear_offset = config->get_docking_linear_x_offset();
                        if(reverse_mode)
                        {
                            mobile->move_linear_x(-1*(avg + linear_offset), 0.05);
                        }
                        else
                        {
                            mobile->move_linear_x(avg + linear_offset, 0.05);
                        }

                        dock_success_wait_time = std::abs((avg + linear_offset) / 0.05) + 0.5;
                        wait_start_time = get_time();
                    }
                }
            }
        }

        else if (fsm_state == DOCKING_FSM_WAIT)
        {
            MOBILE_STATUS ms = mobile->get_status();
            mobile->move(0, 0, 0);
            double check_motor_a = config->get_docking_check_motor_a();

            if (ms.charge_state == 3 && ms.cur_m0 < check_motor_a && ms.cur_m1 < check_motor_a)
            {
                fsm_state = DOCKING_FSM_COMPLETE;

                DATA_DOCK ddock;
                ddock.id = cmd_id;
                ddock.command = "dock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                 Q_EMIT signal_dock_response(ddock);
            }

            else if (get_time() - wait_start_time > 15.0)
            {
                if(!undock_flag)
                {
                    undock_flag = true;
                    undock_waiting_time = get_time();
                    dock = false;

                    if(reverse_mode)
                    {
                        mobile->move_linear_x(config->get_robot_size_x_max() + 0.08, 0.05);
                    }
                    else
                    {
                        mobile->move_linear_x(config->get_robot_size_x_min() - 0.08, 0.05);
                    }


                    if(config->get_charge_type() == "XNERGY")
                    {
                        int non_used_int = 0;
                        mobile->xnergy_command(1, non_used_int); //xnergy stop
                        spdlog::info("[DOCKING] xnergy command charge stop");
                    }
                    spdlog::info("[DOCKING] undock");
                }
                else
                {
                    double t = std::abs(config->get_robot_size_x_max() + 0.15 /0.05) + 0.5;

                    if(get_time() - undock_waiting_time > t)
                    {
                        spdlog::info("[DOCKING] not charge: {}, {}, {}",ms.charge_state, ms.cur_m0, ms.cur_m1 );
                        failed_reason = "NOT CONNECTED";
                        failed_flag =true;

                        //retry logic
                        dock_retry_flag = true;
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
                spdlog::info("[DOCKING] OBS_WAIT -> DRIVING");
                continue;
            }
        }

        else if (fsm_state == DOCKING_FSM_FAILED)
        {
            if(failed_flag)
            {
                failed_flag = false;

                DATA_DOCK ddock;
                ddock.id = cmd_id;
                ddock.command = "dock";
                ddock.result = "fail";
                ddock.message = failed_reason;
                ddock.time = get_time();

                Q_EMIT signal_dock_response(ddock);

                log_info("DOCKING_FSM_FAILED->DOCKING_FSM_OFF");
                fsm_state = DOCKING_FSM_OFF;
            }

            log_info("DOCKING_FSM_FAILED");
            break;
        }

        else if (fsm_state == DOCKING_FSM_COMPLETE)
        {

            
            if(config->get_charge_type() == "XNERGY")
            {
                double xnergy_set_current = config->get_xnergy_set_current();
                mobile->xnergy_command(3, xnergy_set_current);
            }


            fsm_state = DOCKING_FSM_OFF;

            log_info("DOCKING_FSM_COMPLETE->DOCKING_FSM_OFF");
            break;
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
            spdlog::warn("[DOCKING] loop time drift, dt: {}", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
    log_info("docking_loop stop");
}

void DOCKCONTROL::b_loop()
{
    const double dt = 0.02; // 50hz

    spdlog::info("[DOCKING] undocking loop start");

    while(b_flag)
    {
        double pre_loop_time = get_time();
        pre_loop_time = get_time();

        if(fsm_state == DOCKING_FSM_UNDOCK)
        {
            double t = std::abs((config->get_robot_size_x_max()) / 0.05) + 0.5;

            if(get_time() - undock_time > t)
            {
                dock = false; // undock done

                // todo -- have to add undock success request&response check logic

                spdlog::info("[DOCKING] UNDOCK SUCCESS");
                DATA_DOCK ddock;
                ddock.id = cmd_id;
                ddock.command = "undock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                Q_EMIT signal_dock_response(ddock);

                fsm_state = DOCKING_FSM_OFF;
                log_info("DOCKING_FSM_UNDOCK->DOCKING_FSM_OFF");
                break;
            }
        }

        else if (fsm_state == DOCKING_FSM_OFF)
        {
            spdlog::info("[DOCKING] UNDOCK COMPLETE");
        }
        else if (fsm_state == DOCKING_FSM_UNDOCK_FAILED)
        {
            spdlog::info("[DOCKING] UNDOCK FAILED");
            DATA_DOCK ddock;
            ddock.id = cmd_id;
            ddock.command = "undock";
            ddock.result = "fail";
            ddock.message = "IS_GOOD_EVERYTHING CHECK FAILED";
            ddock.time = get_time();

            Q_EMIT signal_dock_response(ddock);
            fsm_state = DOCKING_FSM_OFF;
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
            spdlog::warn("[AUTO] loop time drift, dt: {}", delta_loop_time);
        }
    }
    log_info("undocking_loop stop");
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

    fsm_state = DOCKING_FSM_OFF;
    is_moving = false;
    is_pause = false;
    dock_retry_flag = false;

    if(config->get_charge_type() == "XNERGY")
    {
        int non_used_int = 0;
        mobile->xnergy_command(1, non_used_int); //xnergy stop
        spdlog::info("[DOCKING] xnergy command charge stop");
    }

    // dockstop response
    DATA_DOCK ddock;
    ddock.id = cmd_id;
    ddock.command = "dockstop";
    ddock.result = "done";
    ddock.message = "";
    ddock.time = get_time();
    Q_EMIT signal_dock_response(ddock);

    log_info("dockcontrol stop complete");
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

Eigen::Matrix4d DOCKCONTROL::find_vmark(int& dock_check)
{
    dock_check = - 1 ;

    Eigen::Matrix4d out_;

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

        // for reverse_mode adjusts the comparison
        double target_angle = 0.0 ;

        if(reverse_mode)
        {
            target_angle = M_PI;
        }
        else
        {
            target_angle = 0.0;
        }

        double angle_adjusted = std::abs(std::abs(angle) - target_angle);

        if(std::abs(angle_adjusted) < config->get_docking_clust_angle_threshold())
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

        if (d < 0.01)
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
         g_clusters.push_back(cluster);
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

        if( d > config->get_robot_size_x_max() && d < config->get_docking_find_vmark_dist_threshold_max())
        {
            filtered_clusters.push_back(clust);
        }
    }

    std::vector<Eigen::Vector3d> docking_clust; // finally docking cluster

    // Todo - Test check candidate->sizefilter

    if(!sizefilter(filtered_clusters,docking_clust))
    {
        dock_check = 3;
        return out_;
    }

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

        if(fsm_state == DOCKING_FSM_DOCK)
        {
            dock_tf = calculate_translation_matrix(oneque_frm1_center, frm0_center0);
            err = vfrm_icp(cur_frm, oneque_vfrm, dock_tf);

            if(err > 0.1)
            {
                spdlog::warn("[DOCKING] findvmark err: {}", err);
                return out_;
            }

        }
        else
        {
            dock_tf = calculate_translation_matrix(frm1_center, frm0_center0);
            err = vfrm_icp(cur_frm, vfrm, dock_tf);

            if(err > config->get_docking_icp_err_threshold())
            {
                spdlog::warn("[DOCKING] findvmark err: {}", err);
                dock_check =3;
                return out_;
            }
        }



        out_ = dock_tf; // use base_frame

        if(fsm_state == DOCKING_FSM_CHKCHARGE)
        {
            double x = dock_tf(0, 3);
            double y = dock_tf(1, 3);

            double head_th = std::atan2(y,x);
            double theta = std::atan2(dock_tf(1, 0), dock_tf(0, 0));

            double head_err_th = fabs(head_th*180 / M_PI);
            double orein_err_th = fabs(theta*180.0 /M_PI);

            bool condition;


            if(reverse_mode)
            {
                condition = orein_err_th <= 20.0 && head_err_th >= 160.0;
            }
            else
            {
                condition = orein_err_th <= 20.0 && head_err_th <= 20.0;
            }

            if(condition)
            {
                dock_check = 1; // ONEQUE DOCK CNT
            }

            else
            {
                dock_check = 3; // NO DOCK
            }
        }
        else
        {
            if(find_check)
            {
                Eigen::Matrix4d temp = se2_to_TF(mobile->get_pose().pose) * dock_tf;

                // dist is absolute value
                double dist = (temp.block<2,1>(0,3) - docking_station_o.block<2,1>(0,3)).norm();

                if(dist < 0.2)
                {
                    docking_station = dock_tf;
                    docking_station_o = se2_to_TF(mobile->get_pose().pose) * docking_station;
                    path_flag = true;
                    find_check = true;
                    return out_;
                }

                else
                {
                    return out_;
                }
            }
            //flag toggle
            docking_station = dock_tf;
            docking_station_o = se2_to_TF(mobile->get_pose().pose) * docking_station;
            path_flag = true;
            find_check = true;
        }

        return out_;
    }

    return out_;
}

KFRAME DOCKCONTROL::generate_vkframe(int type, bool reverse_flag)
{
    KFRAME frame;

    XYZR_CLOUD res1;
    XYZR_CLOUD res2;
    XYZR_CLOUD res3;
    XYZR_CLOUD res4;

    double x_offset = config->get_docking_x_offset();
    double y_offset = config->get_docking_y_offset();

    if(type == 0) //oneque vframe
    {
        Eigen::Vector3d p1, p2, p3, p4;

        if(config->get_charge_type() == "RAINBOW")
        {
            if(reverse_flag)
            {
                p1 = Eigen::Vector3d((config->get_robot_size_x_min() - x_offset), -0.225 - y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() + 0.125, 0.0);
                p3 = Eigen::Vector3d(p2.x() - 0.07, p2.y() + 0.1, 0.0);
                p4 = Eigen::Vector3d(p3.x(), p3.y() + 0.22, 0.0);
            }
            else
            {
                p1 = Eigen::Vector3d(config->get_robot_size_x_max() + x_offset, 0.225 + y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() - 0.125 ,0.0);
                p3 = Eigen::Vector3d(p2.x() + 0.07, p2.y() - 0.1 , 0.0);
                p4 = Eigen::Vector3d(p3.x() , p3.y() - 0.22 , 0.0);
            }
        }
        else if(config->get_charge_type() == "XNERGY")
        {
            if(reverse_flag)
            {
                spdlog::info("[DOCKING] XNERGY reverse charge type");
                p1 = Eigen::Vector3d((config->get_robot_size_x_min() - x_offset), -0.275 - y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() + 0.175 ,0.0);
                p3 = Eigen::Vector3d(p2.x() - 0.07, p2.y() + 0.1 , 0.0);
                p4 = Eigen::Vector3d(p3.x() , p3.y() + 0.275 , 0.0);
            }
            else
            {
                spdlog::info("[DOCKING] XNERGY charge type");
                p1 = Eigen::Vector3d(config->get_robot_size_x_max() + x_offset, 0.275 + y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() - 0.175 ,0.0);
                p3 = Eigen::Vector3d(p2.x() + 0.07, p2.y() - 0.1 , 0.0);
                p4 = Eigen::Vector3d(p3.x() , p3.y() - 0.275 , 0.0);
            }
        }
        else
        {
            spdlog::error("[DOCKING] invalid charge type");
            return frame;
        }

        res1 = generate_sample_points(p1,p2,40);
        res2 = generate_sample_points(p2,p3,40);
        res3 = generate_sample_points(p3,p4,40);

        for (const auto& pt : res1.pts)
        {
            frame.pts.push_back(pt);

        }
        for (const auto& pt : res2.pts)
        {
            frame.pts.push_back(pt);

        }
        for (const auto& pt : res3.pts)
        {
            frame.pts.push_back(pt);
        }
    }

    else if (type == 1) //aline vfrmae
    {

        Eigen::Vector3d p1, p2, p3, p4;

        if(config->get_charge_type() == "RAINBOW")
        {
            if(reverse_flag)
            {
                // 후면 라이다 기준으로 좌표 변환 (x축 반대, y축 유지)
                p1 = Eigen::Vector3d((config->get_robot_size_x_min() - config->get_docking_pointdock_margin() - x_offset), -0.225 - y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() + 0.125, 0.0);
                p3 = Eigen::Vector3d(p2.x() - 0.07, p2.y() + 0.1, 0.0);
                p4 = Eigen::Vector3d(p3.x(), p3.y() + 0.22, 0.0);
            }
            else
            {
                p1 = Eigen::Vector3d(config->get_robot_size_x_max() + config->get_docking_pointdock_margin() + x_offset, 0.225 + y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() - 0.125, 0.0);
                p3 = Eigen::Vector3d(p2.x() + 0.07, p2.y() - 0.1, 0.0);
                p4 = Eigen::Vector3d(p3.x(), p3.y() - 0.22, 0.0);
            }
        }

        else if(config->get_charge_type() == "XNERGY")
        {
            if(reverse_flag)
            {
                spdlog::info("[DOCKING] XNERGY reverse charge type");
                p1 = Eigen::Vector3d((config->get_robot_size_x_min() - config->get_docking_pointdock_margin() - x_offset), -0.275 - y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() + 0.175, 0.0);
                p3 = Eigen::Vector3d(p2.x() - 0.07, p2.y() + 0.1, 0.0);
                p4 = Eigen::Vector3d(p3.x(), p3.y() + 0.275, 0.0);
            }
            else
            {
                spdlog::info("[DOCKING] XNERGY charge type");
                p1 = Eigen::Vector3d(config->get_robot_size_x_max() + config->get_docking_pointdock_margin() + x_offset, 0.275 + y_offset, 0.0);
                p2 = Eigen::Vector3d(p1.x(), p1.y() - 0.175, 0.0);
                p3 = Eigen::Vector3d(p2.x() + 0.07, p2.y() - 0.1, 0.0);
                p4 = Eigen::Vector3d(p3.x(), p3.y() - 0.275, 0.0);
            }

        }
        else
        {
            spdlog::error("[DOCKING] invalid charge type");
            return frame;
        }

        res1 = generate_sample_points(p1,p2,50);
        res2 = generate_sample_points(p2,p3,50);
        res3 = generate_sample_points(p3,p4,50);

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

    QString wheel_type = config->get_robot_wheel_type();

//    if(mobile_status.charge_state == 1)
//    {
//        logger->write_log("[DOCK] dock failed (charging)", "Red", true, false);
//        spdlog::info("[DOCK] dock failed (charging)");
//        return false;
//    }

    if(wheel_type == "DD")
    {
        if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1)
        {
            logger->write_log("[DOCK] dock&undock failed (motor status is not 1)", "Red", true, false);
            spdlog::info("[DOCK] dock&undock failed (motor status is not 1)");
            return false;
        }

        if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1)
        {
            logger->write_log("[DOCK] dock&undock failed (motor not connection)", "Red", true, false);
            spdlog::info("[DOCK] dock&undock failed (motor not connection)");
            return false;
        }

        if(mobile_status.status_m0 == 0 || mobile_status.status_m1 == 0)
        {
            logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
            spdlog::info("[DOCK] dock failed (motor error)");
            return false;
        }
    }
    else if(wheel_type == "MECANUM")
    {
        if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1 || mobile_status.status_m2 > 1 || mobile_status.status_m3 > 1 )
        {
            logger->write_log("[DOCK] dock&undock failed (motor status is not 1)", "Red", true, false);
            spdlog::info("[DOCK] dock&undock failed (motor status is not 1)");
            return false;
        }

        if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1 || mobile_status.connection_m2 != 1 || mobile_status.connection_m3 != 1)
        {
            logger->write_log("[DOCK] dock&undock failed (motor not connection)", "Red", true, false);
            spdlog::info("[DOCK] dock&undock failed (motor not connection)");
            return false;
        }

        if(mobile_status.status_m0 == 0 || mobile_status.status_m1 == 0 || mobile_status.status_m2 == 0 || mobile_status.status_m3 == 0)
        {
            logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
            spdlog::info("[DOCK] dock failed (motor error)");
            return false;
        }
    }
    else
    {

        spdlog::info("[DOCK] dock&undock failed (don`t know wheel type)");
    }


    return true;
}


void DOCKCONTROL::dockControl(bool final_dock, const Eigen::Matrix4d& cur_pose, double& linear_vel, double& angular_vel, double& linear_vel_y)
{

    Eigen::Vector3d cur_vel = mobile->get_control_input();

    const float dt = 0.02;

    double v0 = cur_vel(0);
    double v0_y = cur_vel(1);
    double w0 = cur_vel(2);

    double v =0.0;
    double v_y =0.0;
    double w =0.0;

    //use odom frame
    Eigen::Matrix4d target_tf = cur_pose.inverse()*docking_station_o;

    double dist_x = target_tf(0,3);
    double dist_y = target_tf(1,3);

    double err_d = std::sqrt(dist_x*dist_x + dist_y*dist_y);

    //add --0715-2247
    if (dist_x < 0)
    {
        err_d *= -1;
    }


    double err_th = std::atan2(dist_y,dist_x);
    double d_err_th = 0.0;
    double d_err_v = 0.0;
    double d_dist_x = 0.0;
    double d_dist_y = 0.0;

    err_th = wrapToPi(err_th);


    if(err_v_old)
    {
        d_err_th = (err_th - err_th_old) * dt;
        d_err_v = (err_d - err_v_old) * dt;

        d_dist_x = (dist_x - dist_x_old) * dt;
        d_dist_y = (dist_y - dist_y_old) * dt;
    }


    if (fsm_state == DOCKING_FSM_POINTDOCK)
    {

        if (final_dock)
        {
            //final PID Controller
            double direction = 1.0;

            if(reverse_mode)
            {
                direction = -1.0;
            }
            else
            {
                direction = 1.0;
            }

            if (std::abs(err_th) > M_PI_2)
            {
                direction = direction * -1.0;
                err_th = wrapToPi(err_th - M_PI);
                if (err_th > M_PI) err_th -= 2 * M_PI;
            }

            if(config->get_robot_wheel_type() == "MECANUM")
            {
                v =  (config->get_docking_kp_dist() * dist_x + config->get_docking_kd_dist() * d_dist_x);
                v_y = (config->get_docking_kp_dist() * dist_y + config->get_docking_kd_dist() * d_dist_y);
                // w = (config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th);
            }
            else
            {
                v = direction * (config->get_docking_kp_dist() * err_d + config->get_docking_kd_dist() * d_err_v);
                w = direction * (config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th);
            }
        }

        else
        {
            //DWA Controller

            Eigen::Vector3d target_pos = target_tf.block<3,1>(0,3);
            double target_yaw = atan2(target_tf(1,0), target_tf(0,0));

            double min_cost = std::numeric_limits<double>::max();
            std::pair<double, double> best_vw = {0.0, 0.0};
            double yaw_weight = config->get_docking_dwa_yaw_weight();

            for (const auto& [vw, traj] : DWA_TABLE)
            {
                for (const auto& pose : traj)
                {
                    Eigen::Vector3d traj_pos = pose.block<3,1>(0,3);
                    double traj_yaw = atan2(pose(1,0), pose(0,0));

                    double pos_err = (traj_pos - target_pos).head<2>().norm();
                    double yaw_err = std::abs(wrapToPi(traj_yaw - target_yaw));
                    double cost = pos_err + yaw_weight * yaw_err;

                    if (cost < min_cost)
                    {
                        min_cost = cost;
                        best_vw = vw;
                    }
                }
            }

            v = best_vw.first;
            w = best_vw.second * D2R;
        }

    }


    else if(fsm_state == DOCKING_FSM_COMPENSATE)
    {
        err_th = std::atan2(target_tf(1,0),target_tf(0,0));
        w = config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th;
        v = 0.0;
    }

    else if(fsm_state == DOCKING_FSM_YCOMPENSATE)
    {
        spdlog::info("[DOCKING] ycompensate");

        //use odom frame
        Eigen::Matrix4d target_tf_ycompen = cur_pose.inverse()*first_aline;

        double direction = 0.0;
        if(reverse_mode)
        {
            direction = -1.0;
        }
        else
        {
            direction = 1.0;
        }

        if (std::abs(err_th) > M_PI_2)
        {
            direction = -1.0* direction;
            err_th = wrapToPi(err_th - M_PI);
            if (err_th > M_PI) err_th -= 2 * M_PI;
        }

        v = direction * (config->get_docking_kp_dist() * err_d + config->get_docking_kd_dist() * d_err_v);
        w = direction * (config->get_docking_kp_th() * err_th + config->get_docking_kd_th() * d_err_th);

    }

    v = saturation(v, v0- (limit_accel*dt) , v0 +(limit_accel*dt));
    v_y = saturation(v_y, v0_y- (limit_accel*dt) , v0_y +(limit_accel*dt));
    w = saturation(w, w0-(limit_th_acc*D2R*dt), w0 +(limit_th_acc*D2R*dt));
    v = saturation(v, -limit_vel , limit_vel);
    w = saturation(w, -limit_th, limit_th);
    v_y = saturation(v_y, -limit_vel, limit_vel);

    linear_vel = v;
    angular_vel = w;
    linear_vel_y = v_y;
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
    const float len_error_tolerance = 0.2; // 15cm
    std::vector<std::vector<Eigen::Vector3d>> candidates;

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

    double _charge_length = config->get_docking_chg_length();

    for (size_t i = 0; i < lengths.size(); ++i)
    {
        float error = std::abs(lengths[i] - _charge_length);
        if (error <= len_error_tolerance)
        {
            candidates.push_back(in[i]);
        }
    }

    if (candidates.empty())
    {
        return false;
    }

    double min_dist = std::numeric_limits<double>::max();
    int best_idx = -1;

    for (size_t i = 0; i < candidates.size(); ++i)
    {
        const auto& cluster = candidates[i];

        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        for (const auto& pt : cluster)
            mean += pt;
        mean /= cluster.size();

        double dist = mean.head<2>().norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            best_idx = i;
        }
    }

    if (best_idx >= 0)
    {
        out = candidates[best_idx];
        return true;
    }

    return false;
}



bool DOCKCONTROL::undock()
{

    int is_good_everything = is_everything_fine();

    if(is_good_everything == DRIVING_FAILED)
    {
        mobile->move(0, 0, 0);
        spdlog::error("[DOCKING] something wrong (failed)");

        //undock something wrong
        DATA_DOCK ddock;
        ddock.id = cmd_id;
        ddock.command = "undock";
        ddock.result = "fail";
        ddock.message = "IS_GOOD_EVERYTHING CHECK FAILED";
        ddock.time = get_time();

        Q_EMIT signal_dock_response(ddock);
        return false;
    }
    else
    {
        //first stop
        stop();
        // obs clear
        obsmap->clear();
        // start control loop
        b_flag = true;
        // start docking control loop
        fsm_state = DOCKING_FSM_UNDOCK;
        
        b_thread = std::make_unique<std::thread>(&DOCKCONTROL::b_loop, this);

        dock = false;
        reverse_mode = config->get_docking_reverse_mode();
        if(reverse_mode)
        {
            mobile->move_linear_x((config->get_robot_size_x_max() + 0.08), 0.05);
        }
        else
        {
            mobile->move_linear_x((config->get_robot_size_x_min() - 0.08), 0.05);
        }
        undock_time = get_time();
        return true;
    }
}

std::vector<Eigen::Vector3d> DOCKCONTROL::get_cur_clust()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> debug_p = debug_frame;
    debug_frame.clear();
    mtx.unlock();

    return debug_p;
}

std::map<std::pair<double, double>, std::vector<Eigen::Matrix4d>>
DOCKCONTROL::generate_dwa_traj_table(double min_v, double max_v, double v_step,
                                     double min_w_deg, double max_w_deg, double w_step_deg,
                                     double dt, int steps)
{
    std::map<std::pair<double, double>, std::vector<Eigen::Matrix4d>> table;

    for (double v = min_v; v <= max_v; v += v_step)
    {
        for (double w_deg = min_w_deg; w_deg <= max_w_deg; w_deg += w_step_deg)
        {
            double w_rad = w_deg * M_PI / 180.0;

            std::vector<Eigen::Matrix4d> poses;
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();

            double yaw = 0.0;

            for (int i = 0; i < steps; ++i)
            {
                poses.push_back(pose);

                double dx = v * std::cos(yaw) * dt;
                double dy = v * std::sin(yaw) * dt;
                double dyaw = w_rad * dt;

                pose(0, 3) += dx;
                pose(1, 3) += dy;
                yaw += dyaw;

                pose.block<3,3>(0,0) = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            }

            table[{v, w_deg}] = poses;
        }
    }

    return table;
}

bool DOCKCONTROL::get_dock_state()
{
    bool out_;

    mtx.lock();
    out_ = dock;
    mtx.unlock();

    return out_;
}

bool DOCKCONTROL::get_dock_retry_flag()
{
    return (bool)dock_retry_flag.load();
}

void DOCKCONTROL::set_dock_retry_flag(bool val)
{
    dock_retry_flag.store(val);
}

int DOCKCONTROL::get_dock_fsm_state()
{
    return (int)fsm_state.load();
}

void DOCKCONTROL::set_cmd_id(QString id)
{
    mtx.lock();
    cmd_id = id;
    mtx.unlock();
}

double DOCKCONTROL::wrapToPi(double angle)
{
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}