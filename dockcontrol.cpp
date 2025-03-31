#include "dockcontrol.h"

DOCKCONTROL::DOCKCONTROL(QObject *parent)
    : QObject{parent}
{
}

DOCKCONTROL::~DOCKCONTROL()
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
    is_moving = false;
}

void DOCKCONTROL::init()
{
    fsm_state = DOCKING_FSM_OFF;

    if(config->DOCKING_TYPE == 0)
    {
        //L_DOCK_INIT
        Vfrm = generateVKframe();
        frm1_center = calculateCenter(Vfrm);

    }
}

void DOCKCONTROL::stop()
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

    if(config->DOCKING_TYPE == 0)
    {
        undock_flag = false;
        path_flag = false;
    }

    mobile->move(0, 0, 0);
    is_moving = false;
    is_pause = false;

}

void DOCKCONTROL::move()
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();

    // start control loop
    a_flag = true;
    undock_flag = false;
    path_flag = false;
    dock = false ;

    //generate V
    Vfrm = generateVKframe();
    frm1_center = calculateCenter(Vfrm);

    //odometry coordinate set
    Eigen::Vector3d odom_pose = mobile->get_pose().pose;
    odom_start_tf = se2_to_TF(odom_pose);

    //debug
    Eigen::Matrix4d start = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d end = Eigen::Matrix4d::Identity();

    start(0,3) = 0.0;
    start(1,3) = 0.0;

    end(0,3) = 2.0;
    end(1,3) = 2.0;

    hybrid_dubins(start,end);

    Eigen::Matrix4d temp_dock_tf = Eigen::Matrix4d::Identity();

    double theta = M_PI / 4.0;
    temp_dock_tf(0, 0) = std::cos(theta);
    temp_dock_tf(0, 1) = -std::sin(theta);
    temp_dock_tf(1, 0) = std::sin(theta);
    temp_dock_tf(1, 1) = std::cos(theta);

    temp_dock_tf(0,3) = 5.0;
    temp_dock_tf(1,3) = 2.0;


    std::vector<Eigen::Matrix4d> final_path = generateStraightPathDock(temp_dock_tf,0.05, 1.0);

    // start docking control loop
    fsm_state = DOCKING_FSM_POINTDOCK;
    a_thread = new std::thread(&DOCKCONTROL::a_loop, this);

}

void DOCKCONTROL::move(double p_gain, double d_gain, double off_x, double off_y, double off_t)
{
    // stop first
    stop();

    // load preset
    params = load_preset(0);

    ox = std::clamp(off_x, -0.03, 0.03);
    oy = std::clamp(off_y, -0.03, 0.03);
    ot = off_t*D2R;

    bqr->offset_x = ox;
    bqr->offset_y = oy;
    bqr->offset_th = ot;

    p_gain_ratio = std::clamp(p_gain, 0.0, 2.0);
    d_gain_ratio = std::clamp(d_gain, 0.0, 2.0);

    // start control loop
    a_flag = true;
    a_thread = new std::thread(&DOCKCONTROL::a_loop, this);
}

#if defined(USE_SRV) || defined(USE_AMR) || defined(USE_AMR_400_LAKI)
void DOCKCONTROL::a_loop()
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

                    Q_EMIT signal_dock_response(ddock);
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
                mobile->move_linear_x(config->DOCKING_POINTDOCK_MARGIN+0.028 , 0.05);
                dock = true;
                fsm_state = DOCKING_FSM_WAIT;
                wait_start_time = get_time();
            }

        }

        else if (fsm_state == DOCKING_FSM_WAIT)
        {
            MOBILE_STATUS ms = mobile->get_status();

            mobile->move(0, 0, 0);
            qDebug() << "wait start time" << wait_start_time;
            qDebug() << "motor cur" << ms.cur_m0 << ms.cur_m1;
            if (ms.charge_state == 3 && ms.cur_m0 < 60 && ms.cur_m1 < 60)
            {
                fsm_state = DOCKING_FSM_COMPLETE;

                DATA_DOCK ddock;
                ddock.command = "dock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                Q_EMIT signal_dock_response(ddock);
            }
            else if (get_time() - wait_start_time > 8.0)
            {
                if(!undock_flag)
                {
                    qDebug() << "slamnav undock";
                    undock_flag = true;
                    undock_waiting_time = get_time();
                    mobile->move_linear_x(-1*config->DOCKING_POINTDOCK_MARGIN - 0.3, 0.05);
                }
                else
                {
                    double t = std::abs(config->DOCKING_POINTDOCK_MARGIN+ 0.3 /0.05) + 0.5;
                    if(get_time() - undock_waiting_time > t)
                    {
                        qDebug() << "slamnav2 notconeected";
                        failed_reason = "NOT CONNECTED";

                        DATA_DOCK ddock;
                        ddock.command = "dock";
                        ddock.result = "fail";
                        ddock.message = failed_reason;
                        ddock.time = get_time();

                        Q_EMIT signal_dock_response(ddock);
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
            double t = std::abs((config->DOCKING_POINTDOCK_MARGIN +0.3) /0.1) + 0.5;

            if(get_time() - undock_time > t )
            {
                printf("[DOCKING] UNDOCK SUCCESS\n");

                DATA_DOCK ddock;
                ddock.command = "undock";
                ddock.result = "success";
                ddock.message = "";
                ddock.time = get_time();

                Q_EMIT signal_dock_response(ddock);

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

            Q_EMIT signal_dock_response(ddock);
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

#endif

#if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
void DOCKCONTROL::a_loop()
{
    // set flag
    if(bqr->is_connected == false)
    {
        is_moving = false;
        logger->write_log("[DOCKING] code reader not connected.", "Red", true, false);
        return;
    }

    if(bqr->is_recv_data == false)
    {
        is_moving = false;
        logger->write_log("[DOCKING] code not detected.", "Red", true, false);
        return;
    }

    is_moving = true;

    // loop params
    const double dt = 0.05; // 20hz
    double pre_loop_time = get_time();

    // control params
    double extend_dt = 0;

    double pre_err_th = 0;
    double pre_goal_err_d = 0;
    fsm_state = DOCKING_FSM_POINTDOCK;

    logger->write_log("[DOCK] start a_loop", "Green");
    while(a_flag)
    {
        if(is_pause)
        {
            logger->write_log("[DOCK] pause", "Green");
            mobile->move(0, 0, 0);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        bool is_station_docking = (fsm_state != DOCKING_FSM_POINTDOCK_FOR_CHRGE) && (fsm_state != DOCKING_FSM_WAIT);
        if(is_everything_fine() == false && (is_station_docking == true))
        {
            mobile->move(0, 0, 0);

            is_moving = false;
            fsm_state = DOCKING_FSM_FAILED;
            logger->write_log("[DOCK] something wrong. docking fail", "Red");
            return;
        }

        // code reader error
        double code_err_x = bqr->err_x;
        double code_err_y = bqr->err_y;
        double code_err_th = bqr->err_th;

        if(fsm_state == DOCKING_FSM_POINTDOCK)
        {
            // gain ratio
            double kp = p_gain_ratio;
            double kd = d_gain_ratio;

            // calc error and ref vel
            double goal_err_d = std::sqrt(code_err_x*code_err_x + code_err_y*code_err_y) + 0.000001;
            double goal_err_th = code_err_th;

            double dir_x = code_err_x/goal_err_d;
            double dir_y = code_err_y/goal_err_d;
            double ref_v = kp*params.GAIN_P*goal_err_d + kd*params.GAIN_D*(goal_err_d - pre_goal_err_d)/dt;
            ref_v = saturation(ref_v, -params.LIMIT_V, params.LIMIT_V);

            double vx = dir_x*ref_v;
            double vy = dir_y*ref_v;

            double wz = kp*params.GAIN_P*goal_err_th + kd*params.GAIN_D*(deltaRad(goal_err_th, pre_err_th))/dt;
            wz = saturation(wz, -params.LIMIT_W*D2R, params.LIMIT_W*D2R);

            pre_err_th = goal_err_th;

            // goal check
            if(std::abs(goal_err_d) < config->DOCKING_GOAL_D && std::abs(goal_err_th) < config->DOCKING_GOAL_TH*D2R)
            {
                extend_dt += dt;
                if(extend_dt > config->DOCKING_EXTENDED_CONTROL_TIME)
                {
                    mobile->move(0, 0, 0);
                    if(docking_node_type == "GOAL")
                    {
                        is_moving = false;
                        fsm_state = DOCKING_FSM_COMPLETE;

                        QString str;
                        str.sprintf("[DOCK] COMPLETE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                        logger->write_log(str, "Green");
                        return;
                    }
                    else if(docking_node_type == "STATION")
                    {
                        fsm_state = DOCKING_FSM_POINTDOCK_FOR_CHRGE;

                        QString str;
                        str.sprintf("[DOCK] COMPLETE -> MOVE FOR CHRGE, goal_err: %.3f, %.3f\n", goal_err_d, goal_err_th*R2D);
                        logger->write_log(str, "Green");
                        continue;
                    }
                }
            }

            mobile->move(vx, vy, wz);
            pre_goal_err_d = goal_err_d;
        }
        else if(fsm_state == DOCKING_FSM_POINTDOCK_FOR_CHRGE)
        {
            double t = station_linear_move_d / (station_linear_move_v + 1e-6);
            mobile->move_linear_x(station_linear_move_d, station_linear_move_v);

            fsm_state = DOCKING_FSM_WAIT;
            int wait_t = int((t+1.5)*1000);
            QString str;
            str.sprintf("[DOCK] DRIVING_FOR_CHRGE -> WAIT_FOR_CHRGE, wait_time: %.3f (s)", wait_t);
            logger->write_log(str, "Green");

            std::this_thread::sleep_for(std::chrono::milliseconds(wait_t));
            continue;
        }
        else if(fsm_state == DOCKING_FSM_WAIT)
        {
            MOBILE_STATUS ms = mobile->get_status();
            if((int)ms.charge_state == CHARGING_STATION_CHARGING)
            {
                is_moving = false;
                fsm_state = DOCKING_FSM_COMPLETE;
                logger->write_log("[DOCK] DOCKING & CHARGING COMPLETE", "Green");
                return;
            }
            else
            {
                is_moving = false;
                fsm_state = DOCKING_FSM_FAILED;
                logger->write_log("[DOCK] CHARGING FAILED. check this", "Red");
                return;
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
            printf("[DOCK] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }

    mobile->move(0, 0, 0);
    is_moving = false;
    fsm_state = DOCKING_FSM_COMPLETE;

    logger->write_log("[DOCK] stop a_loop", "Green");
}
#endif


void DOCKCONTROL::dockControl( const Eigen::Matrix4d& cur_pose, double& linear_vel, double& angular_vel)
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

bool DOCKCONTROL::find_Vmark()
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
        qDebug() << "err" << err;
        if(err >0.002) //0.001
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


bool DOCKCONTROL::is_everything_fine()
{

    MOBILE_STATUS mobile_status = mobile->get_status();
    if(mobile_status.charge_state == 1)
    {
        logger->write_log("[DOCK] dock failed (charging)", "Red", true, false);
        return false;
    }

    #if defined(USE_SRV) || defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
    if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1)
    {
        logger->write_log("[DOCK] dock failed (motor not connected)", "Red", true, false);
        return false;
    }
    #endif

    #if defined(USE_SRV) || defined(USE_AMR) || defined(USE_AMR_400_LAKI)
    if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1)
    {
        logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
        return false;
    }
    #endif

    if(mobile_status.motor_stop_state == 0)
    {
        logger->write_log("[DOCK] dock failed (emo pushed)", "Red", true, false);
        return false;
    }

    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(bqr->is_recv_data == false)
    {
        return false;
    }
    #endif

    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(mobile_status.status_m0 > 1 || mobile_status.status_m1 > 1 || mobile_status.status_m2 > 1 || mobile_status.status_m3 > 1)
    {
        logger->write_log("[DOCK] dock failed (motor error)", "Red", true, false);
        return false;
    }
    #endif


    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(mobile_status.connection_m0 != 1 || mobile_status.connection_m1 != 1 || mobile_status.connection_m2 != 1 || mobile_status.connection_m3 != 1)
    {
        logger->write_log("[DOCK] dock failed (motor not connected)", "Red", true, false);
        return false;
    }
    #endif

    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    if(mobile_status.status_m0 == 0 || mobile_status.status_m1 == 0 || mobile_status.status_m2 == 0 || mobile_status.status_m3 == 0)
    {
        logger->write_log("[DOCK] dock failed (motor lock offed)", "Red", true, false);
        return false;
    }
    #endif

    return true;
}

void DOCKCONTROL::slot_check_docking()
{
    printf("[DOCKING] slot check docking\n");

    if(docking_node_type == "GOAL" || docking_node_type == "INIT")
    {
        docking_node_type = "GOAL";
    }
    else if(docking_node_type == "STATION")
    {
        docking_node_type = "STATION";
    }
    else
    {
        printf("[DOCKING] not support node type.\n");
        return;
    }

    double offset_x = ox;
    double offset_y = oy;
    double offset_th = ot;

    move(1, 0, offset_x, offset_y, offset_th);
}

DCTRL_PARAM DOCKCONTROL::load_preset(int preset)
{
    DCTRL_PARAM res;

    // read
    QString preset_path = "";

    // config module init
    #ifdef USE_SRV
    preset_path = QCoreApplication::applicationDirPath() + "/config/SRV/" + "docking_preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400/" + "docking_preset_" + QString::number(preset) + ".json";
    #endif

    #ifdef USE_AMR_400_LAKI
    preset_path = QCoreApplication::applicationDirPath() + "/config/AMR_400_LAKI/" + "docking_preset_" + QString::number(preset) + ".json";
    #endif

    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    preset_path = QCoreApplication::applicationDirPath() + "/config/MECANUM/" + "docking_preset_" + QString::number(preset) + ".json";
    #endif

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

            res.GAIN_P = obj["GAIN_P"].toString().toDouble();
            printf("[PRESET] GAIN_P :%f\n", res.GAIN_P);

            res.GAIN_D = obj["GAIN_D"].toString().toDouble();
            printf("[PRESET] GAIN_D :%f\n", res.GAIN_D);

            res.LIMIT_V = obj["LIMIT_V"].toString().toDouble();
            printf("[PRESET] LIMIT_V :%f\n", res.LIMIT_V);

            res.LIMIT_W = obj["LIMIT_W"].toString().toDouble();
            printf("[PRESET] LIMIT_W :%f\n", res.LIMIT_W);

            res.LIMIT_V_ACC = obj["LIMIT_V_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_V_ACC :%f\n", res.LIMIT_V_ACC);

            res.LIMIT_W_ACC = obj["LIMIT_W_ACC"].toString().toDouble();
            printf("[PRESET] LIMIT_W_ACC :%f\n", res.LIMIT_W_ACC);

            file.close();
        }
    }
    else
    {
        printf("[DOCK] invalid preset path\n");
    }

    return res;
}

void DOCKCONTROL::set_node_type(QString type)
{
    mtx.lock();
    docking_node_type = type;
    mtx.unlock();
}

void DOCKCONTROL::set_dock_offset(Eigen::Vector3d offset_val)
{
    mtx.lock();
    ox = offset_val[0];
    oy = offset_val[1];
    ot = offset_val[2];
    mtx.unlock();
}


Eigen::Vector3d DOCKCONTROL::calculateCenter(const KFRAME& kframe)
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

XYZR_CLOUD DOCKCONTROL::generateSamplePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n)
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


Eigen::Matrix4d DOCKCONTROL::calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = to[0] - from[0];
    translationMatrix(1, 3) = to[1] - from[1];
    translationMatrix(2, 3) = to[2] - from[2];

    return translationMatrix;
}

double DOCKCONTROL::Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
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


bool DOCKCONTROL::sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out)
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

std::vector<Eigen::Vector3d> DOCKCONTROL::get_cur_clust()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> debug_p = debug_frame;
    mtx.unlock();

    return debug_p;
}
KFRAME DOCKCONTROL::generateVKframe()
{
    KFRAME frame;
    XYZR_CLOUD res1;
    XYZR_CLOUD res2;
    XYZR_CLOUD res3;
    XYZR_CLOUD res4;

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

bool DOCKCONTROL::undock()
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
        b_thread = new std::thread(&DOCKCONTROL::b_loop, this);
        mobile->stop_charge();
        mobile->move_linear_x(-1*(config->DOCKING_POINTDOCK_MARGIN), 0.05);
        undock_time = get_time();
        return true;
    }
}


std::vector<Eigen::Matrix4d> DOCKCONTROL::hybrid_dubins(const Eigen::Matrix4d& st_tf, const Eigen::Matrix4d& ed_tf)
{
    const double chk_dt = 0.03;
    const double chk_dr = 3.0*D2R;

    const double sampling_dt = 0.05;
    const double sampling_dr = 5.0*D2R;

    const double penaltyTurning = 1.05;
    const double penaltyCOD = 2.0;
    const double penaltyReversing = 2.0;

    gs = config->DOCKING_GRID_SIZE;
    dock_mapsize = config->DOCKING_MAP_SIZE;
    w = dock_mapsize/gs;
    h = dock_mapsize/gs;


    cv::Vec2i st_uv = xy_uv(st_tf(0,3), st_tf(1,3));
    cv::Vec2i ed_uv = xy_uv(ed_tf(0,3), ed_tf(1,3));

    //bfs
    cv::Mat bfs_map;
    bfs_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    cv::Mat flow_field = calc_flowfield(bfs_map, ed_uv);
    double cost0 = flow_field.ptr<float>(st_uv[1])[st_uv[0]];

    // search algorithm
    std::vector<HASTAR_NODE*> open_set;
    std::vector<std::vector<HASTAR_NODE*>> close_set(w*h); // close set maybe too big

    // set st, ed
    // update h
    HASTAR_NODE *ed = new HASTAR_NODE();
    ed->tf = ed_tf;

    HASTAR_NODE *st = new HASTAR_NODE();
    st->tf = st_tf;

    double dx = ed->tf(0,3) - st->tf(0,3);
    double dy = ed->tf(1,3) - st->tf(1,3);
    st->g = std::sqrt(dx*dx + dy*dy);
    st->h = updateH(st->tf,ed->tf);
    st->f = st->g + st->h;
    qDebug() <<" first h" << st->h;
    open_set.push_back(st);


    // search loop
    int iter = 0;
    double st_time = get_time();
    while(open_set.size() > 0)
    {
        // get best node
        int cur_idx = 0;
        HASTAR_NODE *cur = open_set.front();
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
        cv::Vec2i uv0 = xy_uv(cur->tf(0,3), cur->tf(1,3));
        if(uv0[0] < 0 || uv0[0] >= w || uv0[1] < 0 || uv0[1] >= h)
        {
            continue;
        }

        close_set[uv0[1]*w + uv0[0]].push_back(cur);

        // for debug
//        if(config->USE_SIM == 1)
//        {
//            debug_img.ptr<cv::Vec3b>(uv0[1])[uv0[0]] = cv::Vec3b(0,255,0);

//            cv::circle(debug_img, cv::Point(st_uv[0], st_uv[1]), 3, cv::Scalar(0,0,255), 2);
//            cv::circle(debug_img, cv::Point(ed_uv[0], ed_uv[1]), 3, cv::Scalar(0,255,255), 2);

//            cv::Mat debug_map2;
//            cv::resize(debug_img, debug_map2, cv::Size(debug_img.cols*2, debug_img.rows*2));
//            cv::imshow("calc_path_debug", debug_map2);
//        }

        // found goal
        if(cur->h < st->h*0.5)
        {
            qDebug() << "find goal";
            Eigen::Matrix4d goal_tf0 = calc_tf(cur->tf.block(0,3,3,1), ed->tf.block(0,3,3,1));
            Eigen::Matrix4d goal_tf1 = goal_tf0;
            goal_tf0.block(0,3,3,1) = cur->tf.block(0,3,3,1);
            goal_tf1.block(0,3,3,1) = ed->tf.block(0,3,3,1);

            std::vector<Eigen::Matrix4d> traj_goal0 = intp_tf(cur->tf, goal_tf0, sampling_dt, sampling_dr);
            std::vector<Eigen::Matrix4d> traj_goal1 = intp_tf(goal_tf0, goal_tf1, sampling_dt, sampling_dr);

            std::vector<Eigen::Matrix4d> traj_goal = traj_goal0;
            traj_goal.insert(traj_goal.end(), traj_goal1.begin(), traj_goal1.end());

            std::vector<Eigen::Matrix4d> res;

            HASTAR_NODE* _cur = cur;
            while(_cur != NULL)
            {
                qDebug() <<"r";
                res.push_back(_cur->tf);
                _cur = _cur->parent;
                if(_cur != NULL)
                {
                    Eigen::Vector3d debug_pt(_cur->tf(0,3), _cur->tf(1,3), 0.0);
                    debug_frame.push_back(debug_pt);
                }
            }
            qDebug() << "when die2";
            std::reverse(res.begin(), res.end());

            // set final pose
            for(size_t p = 0; p < traj_goal.size(); p++)
            {
                res.push_back(traj_goal[p]);

                Eigen::Vector3d debug_pt(traj_goal[p](0,3), traj_goal[p](1,3), 0.0);
                debug_frame.push_back(debug_pt);
            }

            qDebug() << "when die3";

            printf("[DOCKING] path_finding complete, num:%d, iter:%d\n", (int)res.size(), iter);
            return res;

        }

        // expand nodes for differential drive
        double step = gs;
        std::vector<Eigen::Matrix4d> around;
        for(int i = -2; i <= 2; i++)
        {
            for(int j = -2; j <= 2; j++)
            {
                if(j == 0 && i == 0)
                {
                    continue;
                }

                double offset_x = j*step;
                double offset_y = i*step;
                double offset_th = std::atan2(offset_y, offset_x);

                Eigen::Vector3d offset_xi(offset_x, offset_y, offset_th);
                Eigen::Matrix4d offset_tf = se2_to_TF(offset_xi);

                Eigen::Matrix4d tf0 = cur->tf;
                Eigen::Matrix4d tf1 = offset_tf;

                tf1(0,3) += tf0(0,3);
                tf1(1,3) += tf0(1,3);
                tf1(2,3) += tf0(2,3);

                // check range
                if(calc_dist_2d(tf1.block(0,3,3,1)) > dock_mapsize)
                {
                    continue;
                }

                // check collision
                cv::Vec2i uv1 = xy_uv(tf1(0,3), tf1(1,3));
                if(uv1[0] < 0 || uv1[0] >= w || uv1[1] < 0 || uv1[1] >= h)
                {
                    continue;
                }

//                if(flow_field.ptr<float>(uv1[1])[uv1[0]] < 0)
//                {
//                    continue;
//                }

                // check close set
                bool is_close_set = false;
                for(size_t p = 0; p < close_set[uv1[1]*w + uv1[0]].size(); p++)
                {
                    double dth = calc_dth(close_set[uv1[1]*w + uv1[0]][p]->tf, tf1);
                    if(dth < chk_dr) // ?
                    {
                        is_close_set = true;
                        break;
                    }
                }
                if(is_close_set)
                {
                    continue;
                }

                // check collision
                std::vector<Eigen::Matrix4d> traj = intp_tf(tf0, tf1, sampling_dt, sampling_dr);
//                if(is_collision(obs_map, traj, margin_x, margin_y))
//                {
//                    continue;
//                }
                around.push_back(tf1);
            }
        }

        // calc child node
        for(size_t p = 0; p < around.size(); p++)
        {
            // 탐색노드의 픽셀좌표확인
            cv::Vec2i uv = xy_uv(around[p](0,3), around[p](1,3));
            if(uv[0] < 0 || uv[0] >= w || uv[1] < 0 || uv[1] >= h)
            {
                continue;
            }

            //거리 cost 확인
            double cost1 = flow_field.ptr<float>(uv[1])[uv[0]];

            if(cost1 < 0)
            {
                continue;
            }

            // calc heuristics
            // update h
            Eigen::Vector2d dtdr1 = dTdR(cur->tf, around[p]);   
            double dx = ed->tf(0,3) - st->tf(0,3);
            double dy = ed->tf(1,3) - st->tf(1,3);

            int prim = 0;
            // prim 0 :d / 1: d-l / 2: d-r / 3: b / 4: b-r / 5: b-l

            double g =0;
            double h =0;
            double f =0;
            //front move
            if(dtdr1[0] > 0)
            {
                if(dtdr1[1] > 0)
                {
                    prim = 1;
                }
                else if (dtdr1[1] < 0)
                {
                    prim = 2;
                }
                else
                {
                    prim = 0;
                }
            }

            //back move
            else
            {
                if(dtdr1[1] > 0)
                {
                    prim = 4;
                }
                else if(dtdr1[1] < 0)
                {
                    prim = 5;
                }
                else
                {
                    prim = 3;
                }
            }

            if(prim < 3) //forward driving
            {
                if(cur->prim != prim)
                {
                    if(cur->prim > 2)
                    {
                        //change direction
                        g += dtdr1[0] * penaltyTurning * penaltyCOD;
                    }
                    else
                    {
                        //just turning
                        g += dtdr1[0] * penaltyTurning;
                    }
                }
                else
                {
                    g+= dtdr1[0];
                }
            }

            else //reversiong driving
            {
                if(cur->prim != prim)
                {
                    if(cur->prim <3)
                    {
                        g+= dtdr1[0] * penaltyTurning * penaltyReversing * penaltyCOD;
                    }
                    else
                    {
                        g+= dtdr1[0] * penaltyTurning * penaltyReversing;
                    }
                }
                else
                {
                    g+= dtdr1[0] * penaltyReversing;
                }
            }
                    //cost1*gs; //std::sqrt(dx*dx+dy*dy);
                                /*cur->g + dtdr1[0]
                              + dtdr1[1] * 0.1
                              + 1.0/calc_clearance(obs_map, around[p], 2.0) * 0.1;*/

            h = updateH(cur->tf,around[p]);//cost1*gs; // m단위로 변환하기위한 gs
            f = g + h;

            qDebug() << "chiled prim" << prim;

            // check open set
            bool is_open_set = false;
            HASTAR_NODE* open_node = NULL;
            for(size_t q = 0; q < open_set.size(); q++)
            {
                Eigen::Vector2d dtdr = dTdR(open_set[q]->tf, around[p]);
                if(dtdr[0] < chk_dt && dtdr[1] < chk_dr)
                {
                    is_open_set = true;
                    open_node = open_set[q];
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
                }
                continue;
            }

            // add new child to open set
            HASTAR_NODE *child = new HASTAR_NODE();
            child->parent = cur;
            child->tf = around[p];
            child->g = g;
            child->h = h;
            child->f = f;
            child->prim = prim;
            open_set.push_back(child);
        }

        double timeout = get_time() - st_time;
        if(timeout > 1.5)
        {
            printf("[OBSMAP] timeout, iter:%d\n", iter);
            break;
        }

        iter++;
    }
    std::vector<Eigen::Matrix4d> res;
    return res;
}


std::vector<Eigen::Matrix4d> DOCKCONTROL::runAstar(const Eigen::Matrix4d &start, const Eigen::Matrix4d &end)
{
    const double resolution = 0.05; //m
    const double turn_weight = 1.5;

    const int grid_size = 100;
    const int ns = grid_size * grid_size;

    int grid_center = grid_size / 2;

    Eigen::Vector2i start_grid(grid_center + round(start(0, 3) / resolution),
                            grid_center - round(start(1, 3) / resolution));

    Eigen::Vector2i end_grid(grid_center + round(end(0, 3) / resolution),
                             grid_center - round(end(1, 3) / resolution));

    std::vector<float> potarr(ns, FLT_MAX);
    std::vector<bool> visited(ns, false);
    std::vector<int> came_from(ns, -1);
    std::vector<int> curP, nextP, overP;

    //just array
    int startCell = start_grid.x() * grid_size + start_grid.y();
    int goalCell = end_grid.x() * grid_size + end_grid.y();

    potarr[startCell] = 0.0;
    curP.push_back(startCell);

    std::vector<Eigen::Vector2i> directions =
    {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    float curT = std::hypot(end_grid.x() - start_grid.x(), end_grid.y() - start_grid.y());

    float priInc = 2.0f;
    int cycles = 10000;

    const double w_d = 1.0;
    const double w_theta = 5.0;
    const double k = 0.1;

    bool goal_found = false;

    for (int cycle = 0; cycle < cycles; cycle++)
    {
        if (curP.empty() && nextP.empty())
            break;

        std::vector<int> tempP;

        for (int idx : curP)
        {
            int x = idx / grid_size;
            int y = idx % grid_size;

            if (idx == goalCell) {
                goal_found = true;
                break;
            }

            for (auto &dir : directions)
            {
                int new_x = x + dir.x();
                int new_y = y + dir.y();
                int new_idx = new_x * grid_size + new_y;

                if (new_x < 0 || new_x >= grid_size || new_y < 0 || new_y >= grid_size || visited[new_idx])
                {

                    continue;
                }

                double g = potarr[idx] + std::hypot(dir.x(), dir.y());

                double distance = std::hypot(end_grid.x() - new_x, end_grid.y() - new_y);
                double theta_new = std::atan2(end_grid.y() - new_y, end_grid.x() - new_x);
                double theta_old = std::atan2(end_grid.y() - y, end_grid.x() - x);
                double angle_diff = std::abs(std::fmod(theta_new - theta_old + M_PI, 2 * M_PI) - M_PI);

                double angle_weight = w_theta * angle_diff * std::exp(-k * distance);

                double h = w_d * distance + angle_weight;

                float f = g + h;

                if (f < potarr[new_idx])
                {
                    potarr[new_idx] = f;
                    came_from[new_idx] = idx;
                    if (f < curT)
                        nextP.push_back(new_idx);
                    else
                        tempP.push_back(new_idx);
                }
            }
        }

        if (goal_found)
        {
            qDebug() << "goal found";
            break;
        }
        curP = nextP;
        nextP = tempP;

        if (curP.empty())
        {
            curT += priInc;
            curP = overP;
            overP.clear();
        }
    }

    if (!goal_found) return {};

    std::vector<Eigen::Matrix4d> path;
    for (int idx = goalCell; idx != startCell && idx != -1; idx = came_from[idx])
    {
        int x = idx / grid_size;
        int y = idx % grid_size;

        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = (x - grid_center) * resolution;
        pose(1, 3) = (y - grid_center) * resolution;
        path.push_back(pose);


        Eigen::Vector3d debug_pt(pose(0,3), pose(1,3), 0.0);
        debug_frame.push_back(debug_pt);
    }

    std::reverse(path.begin(), path.end());
    return path;
}

cv::Vec2i DOCKCONTROL::xy_uv(double x, double y)
{
    // y axis flip
    int u = std::floor(x/gs) + cx;
    int v = std::floor(-y/gs) + cy;
    return cv::Vec2i(u, v);
}

cv::Vec2d DOCKCONTROL::uv_xy(int u, int v)
{
    double x = (u-cx)*gs;
    double y = -(v-cy)*gs;
    return cv::Vec2d(x, y);
}

cv::Mat DOCKCONTROL::calc_flowfield(const cv::Mat& map, cv::Vec2i ed)
{
    ed[0] = saturation(ed[0], 0, w-1);
    ed[1] = saturation(ed[1], 0, h-1);



    cv::Mat res(h, w, CV_32F, cv::Scalar(-1));

    std::vector<cv::Vec2i> directions = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };

    std::queue<cv::Vec2i> q;
    q.push(ed);
    res.ptr<float>(ed[1])[ed[0]] = 0;

    while(!q.empty())
    {

        cv::Vec2i current = q.front();
        q.pop();

        float current_cost = res.ptr<float>(current[1])[current[0]];

        for(const auto& dir : directions)
        {

            cv::Vec2i neighbor = current + dir;


            if (neighbor[0] < 0 || neighbor[0] >= w || neighbor[1] < 0 || neighbor[1] >= h)
            {

                continue;
            }

            if (map.ptr<uchar>(neighbor[1])[neighbor[0]] == 255)
            {

                continue;
            }

            if (res.ptr<float>(neighbor[1])[neighbor[0]] == -1)
            {
                res.ptr<float>(neighbor[1])[neighbor[0]] = current_cost + 1;
                q.push(neighbor);
            }
        }
    }

    return res;
}

std::vector<Eigen::Matrix4d> DOCKCONTROL::generateStraightPathDock(const Eigen::Matrix4d& dock_tf, double step, double length)
{
    std::vector<Eigen::Matrix4d> path;

    int num_steps = static_cast<int>(length / step);
    Eigen::Vector3d dir = dock_tf.block<3,1>(0, 0).normalized();
    Eigen::Vector3d end_pos = dock_tf.block<3,1>(0, 3);

    for (int i = num_steps; i >= 0; --i)
    {
        Eigen::Vector3d offset = -dir * (i * step);
        Eigen::Matrix4d tf = dock_tf;
        tf.block<3,1>(0, 3) = end_pos + offset;

        Eigen::Vector3d debug_pt(tf(0,3), tf(1,3), 0.0);
        debug_frame.push_back(debug_pt);
        path.push_back(tf);
    }
    return path;
}

double DOCKCONTROL::updateH(const Eigen::Matrix4d st, const Eigen::Matrix4d ed)
{

    double set_h = 0;
    double dubinsCost =0;
    double reedsSheppCost =0;
    ompl::base::DubinsStateSpace dubinsPath(1);

    State* dbst = (State*)dubinsPath.allocState();
    State* dbend = (State*)dubinsPath.allocState();

    dbst->setXY(st(0,3),st(1,3));
    dbst->setYaw(std::atan2(st(1,0),st(0,0)));

    dbend->setXY(ed(0,3),ed(1,3));
    dbend->setYaw(std::atan2(ed(1,0),ed(0,0)));

    dubinsCost = dubinsPath.distance(dbst,dbend);


    //if use reverse path mode
    //
    double dx = ed(0,3) - st(0,3);
    double dy = ed(1,3) - st(1,3);

    double twoDcost = std::sqrt(dx*dx + dy*dy);
    set_h = std::max(reedsSheppCost, std::max(dubinsCost, twoDcost));

    return set_h;
}
