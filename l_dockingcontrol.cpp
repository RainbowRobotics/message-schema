#include "l_dockingcontrol.h"


L_DOCKINGCONTROL::L_DOCKINGCONTROL(QObject *parent)
    : QObject{parent}
{
    last_cur_pos.setZero();
}

L_DOCKINGCONTROL::~L_DOCKINGCONTROL()
{
    // control loop stop
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }


    mobile->move(0, 0, 0);
    is_moving = false;
}

void L_DOCKINGCONTROL::init()
{

}


void L_DOCKINGCONTROL::stop()
{
    // control loop stop
    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
    mobile->move(0, 0, 0);
    is_pause = false;
}



void L_DOCKINGCONTROL::move()
{
    // stop first
    stop();

    // obs clear
    obsmap->clear();
    is_moving = true;
    // start control loop
    a_flag = true;
    // start docking control loop           
    process = DockingState::PointDock; 
    a_thread = new std::thread(&L_DOCKINGCONTROL::a_loop, this);
}


bool L_DOCKINGCONTROL::find_vmark()
{
    std::vector<c_pointList> filtered_clusters;
    std::vector<Eigen::Vector3d> c_lidar = lidar->get_cur_scan();
    const int c_points = c_lidar.size();
    std::vector< std::vector<float> > polar(c_points +1 ,std::vector<float>(2));

    for(unsigned int i=0; i <c_points; ++i)
    {
        float x = static_cast<float>(c_lidar[i].x());
        float y = static_cast<float>(c_lidar[i].y());
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

    polar.pop_back(); //remove the wrapping element
    int len = polar.size();
    std::vector<c_pointList> g_clusters;

    for(unsigned int i=0; i<begin.size(); ++i)
    {
        c_pointList cluster;
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
            cluster.push_back(c_point(x, y));
            ++j;
        }
        int num = cluster.size();
        if(num > clust_size_min && num < clust_size_max) g_clusters.push_back(cluster);
    }

    for(auto& c_pointList : g_clusters)
    {
        float mean_x = 0.0; float mean_y = 0.0;
        float sum_x= 0.0; float sum_y= 0.0;
        float d = 0.0; float find_angle =0.0;

        for(auto& point : c_pointList)
        {
            sum_x += point.first;
            sum_y += point.second;
        }

        mean_x = sum_x / c_pointList.size();
        mean_y = sum_y / c_pointList.size();

        find_angle = std::atan2(mean_y, mean_x);
        d = sqrt(pow(mean_x, 2) + pow(mean_y, 2));

        //<Seokgyun.kim> 
        //clust_dist_threshold_min and max filter clusters based on the distance from the robot.
        //clust_angle_threshold filters clusters based on the angle relative to the robot's coordinate system.
        if( d > clust_dist_threshold_min && d < clust_dist_threshold_max && std::abs(find_angle) < clust_angle_threshold)
        {
            filtered_clusters.push_back(c_pointList);
        }
    }

    c_pointList docking_clust ;

    if(!mapfixclust(filtered_clusters, docking_clust)) return false;
    
    clusters_queue.push(docking_clust);

    if(clusters_queue.size() > 10)
    {
        qDebug() << "queue size 10";
        KFRAME kfrm = generateVKframe();
        KFRAME cur_frm;

        while (!clusters_queue.empty()) 
        {
            const auto& cluster = clusters_queue.front();

            for (const auto& point : cluster)
            {
                PT_XYZR pt;
                pt.x = point.first;
                pt.y = point.second;
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
        Eigen::Vector3d frm1_center1 = calculateCenter(kfrm);
        Eigen::Matrix4d dock_tf = Eigen::Matrix4d::Identity();
        dock_tf = calculateTranslationMatrix(frm1_center1, frm0_center0);

        double err = Vfrm_icp(cur_frm, kfrm, dock_tf);
        qDebug() << "error is" << err;
        if(err >0.001)
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



void L_DOCKINGCONTROL::a_loop()
{
    const double dt = 0.02; // 50hz
    float odom_v_dock =0.0;
    is_moving = true;
    while(a_flag){

        double pre_loop_time = get_time();
        Eigen::Matrix4d cur_pos = slam->get_cur_tf();
        switch(process)
        {
            case DockingState::PointDock:
            {
                double cmd_v =0.0;
                double cmd_w =0.0;
                find_vmark();
                if(path_flag)
                {
                    Eigen::Vector2d dtdr = dTdR(cur_pos, docking_station_m);

                    if(std::abs(dtdr(0)) < 0.05)
                    {
                        mobile->move(0,0,0);
                        path_flag = true;
                        process = DockingState::CompensationYaw;
                        break;
                    }
                    dockControl(cur_pos,cmd_v,cmd_w);
                    mobile->move(cmd_v,0,cmd_w);
                }
                break;
            }
            case DockingState::CompensationYaw:
            {
                double cmd_v =0.0;
                double cmd_w =0.0;

                if(path_flag)
                {
                    std::vector<Eigen::Matrix4d> path_pos = get_path();
                    Eigen::Matrix4d dock_station_local = cur_pos.inverse()*docking_station_m;
                    Eigen::Vector2d dtdr = dTdR(cur_pos, docking_station_m);

                     if(std::abs(dtdr(1)) < 0.1*M_PI / 180.0){
                         mobile->move(0,0,0);
                         process = DockingState::Dock;
                         path_flag = false;
                         break;
                     }
                     dockControl(cur_pos,cmd_v,cmd_w);
                     mobile->move(cmd_v,0.0,cmd_w);
                }

                else
                {
                    find_vmark();
                    mobile->move(0,0,0);
                }
                break;
            }

            case DockingState::Dock:
            {
               double cmd_v =0.0;
               double cmd_w =0.0;

               if(odom_v_dock < POINTDOCK_MARGIN)
               { 
                   cmd_v = 0.03;
                   odom_v_dock += cmd_v*dt;
                   mobile->move(cmd_v,0,0);
               }
               else
               {
                   mobile->move(0,0,0);
                   process = DockingState::None;
               }
            break;
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
            printf("[AUTO] loop time drift, dt:%f\n", delta_loop_time);
        }
        pre_loop_time = get_time();
    }
}

std::vector<Eigen::Matrix4d> L_DOCKINGCONTROL::get_path()
{
    mtx.lock();
    std::vector<Eigen::Matrix4d> out = dock_path_;
    mtx.unlock();
    return out;
}


std::vector<Eigen::Vector3d> L_DOCKINGCONTROL::get_cur_clust()
{
    mtx.lock();
    std::vector<c_pointList> o_cluster = line_points;
    mtx.unlock();
    std::vector<Eigen::Vector3d> res;

    for(const auto& pointList : o_cluster)
    {
        for (const auto& point : pointList) 
        {
            Eigen::Vector3d eigen_point(point.first, point.second, 0.0);
            res.push_back(eigen_point);
        }
    }
    return res;
}

std::vector<Eigen::Matrix4d> L_DOCKINGCONTROL::generateSmoothPath(const Eigen::Matrix4d& start, const Eigen::Matrix4d& goal, const double& control_dist) {

    std::vector<Eigen::Matrix4d> path;

    double start_x = start(0, 3);
    double start_y = start(1, 3);
    double start_theta = extractTheta(start);
    double goal_x = goal(0, 3);
    double goal_y = goal(1, 3);
    double goal_theta = extractTheta(goal);
    std::vector<Eigen::Matrix4d> sampled_points;

    double x_min = -0.5;
    double x_max = 0.0;
    int num_samples = 25;
    double step = (x_max - x_min) / (num_samples - 1);

    for (int i = 0; i < num_samples; ++i) 
    {
        double x = x_min + i * step;
        Eigen::Matrix4d local_point = Eigen::Matrix4d::Identity();
        local_point (0, 3) = x;
        Eigen::Matrix4d point_in_goal = goal * local_point;

        sampled_points.push_back(point_in_goal);
    }
    sampled_points.push_back(goal);
    path = sampled_points;



    #ifdef lattice_planner
    std::vector<Eigen::Matrix4d> combined_path;
    std::vector<Eigen::Matrix4d> straight_path;
    std::vector<Eigen::Matrix4d> curve_path;
    Eigen::Matrix4d cur_pose = slam->get_cur_tf();

    for(auto& point : path)
    {
        point = cur_pose.inverse()*point; // convert to local frame
        straight_path.push_back(point);
    }

    lattice_primitives best_cost_primitive;
    best_cost_primitive = critic_primitives(make_primitives(straight_path[0]));

    if(best_cost_primitive.trajectory.size() == 0)
    {
        return path;
    }

    for(auto& vec : best_cost_primitive.trajectory)
    {
        Eigen::Matrix4d point = cur_pose * se2_to_TF(vec);  // convert to global frame
        combined_path.push_back(point);
    }

    Eigen::Matrix4d last_point = getLastPathPos(combined_path);

    int temp_idx =0;
    for(auto& point : path)
    {
        if(temp_idx==0)
        {
            last_point = point;
        }
        combined_path.push_back(point);
        temp_idx ++;
    }

    return combined_path;
    #endif


    return path;
}

double L_DOCKINGCONTROL::distance(const Eigen::Matrix4d& m1, const Eigen::Matrix4d& m2) 
{
    double dx = m2(0, 3) - m1(0, 3);
    double dy = m2(1, 3) - m1(1, 3);
    return std::sqrt(dx * dx + dy * dy);
}

double L_DOCKINGCONTROL::extractTheta(const Eigen::Matrix4d& matrix) 
{
    return std::atan2(matrix(1, 0), matrix(0, 0));
}


Eigen::Matrix4d L_DOCKINGCONTROL::findLookaheadPointReverse(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& cur_pose, const double& lookahead_dist) {

    int closest_idx = findClosetPointIndex(path, cur_pose);

    for (int i = closest_idx; i < path.size(); ++i) 
    {
        if (distance(path[i], cur_pose) >= lookahead_dist) 
        {
            return path[i];
        }
    }
    return path.back();
}

int L_DOCKINGCONTROL::findClosetPointIndex(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& cur_pose) 
{

    double min_dist = std::numeric_limits<double>::max();
    int closest_idx = -1;

    for (int i = 0; i < path.size(); ++i) 
    {
        double dist = distance(path[i], cur_pose);

        if (dist < min_dist) 
        {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

Eigen::Matrix4d L_DOCKINGCONTROL::getLastPathPos(const std::vector<Eigen::Matrix4d>& path_pos) {
    if (!path_pos.empty()) {
        return path_pos.back();
    } else {
        throw std::runtime_error("Path is empty!");
    }
}

void L_DOCKINGCONTROL::dockControl( const Eigen::Matrix4d& cur_pose, double& linear_vel, double& angular_vel) {

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

    v = Kp_d * err_d + Kd_d * d_err_v;
    w = Kp_th * err_th + Kd_th * d_err_th;

    if(process == DockingState::CompensationYaw)
    {
        err_th = std::atan2(dist(1,0),dist(0,0));
        w = Kp_th * err_th + Kd_th * d_err_th;
        v = 0.0;
    }

    v = saturation(v, v0- (limit_accel*dt) , v0 +(limit_accel*dt));
    w = saturation(w, w0-(limit_th_acc*D2R*dt), w0 +(limit_th_acc*D2R*dt));
    v = saturation(v, -limit_vel , limit_vel);
    w = saturation(w, -limit_th, limit_th);

    //set cmd
    linear_vel = v;
    angular_vel = w;

}

std::vector<pcl::PointXYZRGB> L_DOCKINGCONTROL::get_pcl(int r, int g, int b)
{
    std::vector<pcl::PointXYZRGB> pcl_points;

    mtx.lock();
    c_pointList cp1 = debug_clust_l;
    c_pointList cp2 = debug_clust_r;
    mtx.unlock();

    for(auto& point : cp1)
    {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = point.first;
        pcl_point.y = point.second;
        pcl_point.z = 0.0;
        pcl_point.r = 255;
        pcl_point.g = 0;
        pcl_point.b = 0;
        pcl_points.push_back(pcl_point);
    }
    for(auto& point : cp2)
    {
        pcl::PointXYZRGB pcl_point;
        pcl_point.x = point.first;
        pcl_point.y = point.second;
        pcl_point.z = 0.0;
        pcl_point.r = 0;
        pcl_point.g = 0;
        pcl_point.b = 255;
        pcl_points.push_back(pcl_point);
    }
    std::vector<pcl::PointXYZRGB> out = pcl_points;

    return out;
}


bool L_DOCKINGCONTROL::mapfixclust(std::vector<c_pointList>& filtered_c, c_pointList& dock_clusts)
{

    Eigen::Matrix4d map_tf = slam->get_cur_tf();
    std::vector<double> temp;

    for(auto& c_pointList : filtered_c)
    {
        float sum_x = 0.0;
        float sum_y = 0.0;

        for(auto& point : c_pointList)
        {
            sum_x += point.first;
            sum_y += point.second;
        }
        
        float mean_x = sum_x / c_pointList.size();
        float mean_y = sum_y / c_pointList.size();
        Eigen::Vector4d temp_p(mean_x,mean_y, 0.0, 1.0);
        Eigen::Vector4d clust_mapf = map_tf * temp_p;
        double dist = std::sqrt(std::pow(clust_mapf(0) - DOCKING_STATION_G[0],2) + std::pow(clust_mapf(1) - DOCKING_STATION_G[1],2));
        temp.push_back(dist);
    }

    if(temp.size()==0)
    {
        return false;
    }

    if(temp.size()==1)
    {
        dock_clusts = filtered_c[0];
        return true;
    }

    else
    {
        double min_d = 9999.;
        int idx=0;
        int min_idx=0;

        for(auto& d : temp)
        {
            if(min_d > d) 
            {
                min_d = d;
                min_idx =idx;
            }
            idx++;
        }

        qDebug() << "min idx is " << min_idx;
        dock_clusts = filtered_c[min_idx];
        qDebug() << "dock_clusts size" << dock_clusts.size();
        return true;

    }
    return false;
}


KFRAME L_DOCKINGCONTROL::generateVKframe()
{
    KFRAME frame;
    XYZR_CLOUD res2;
    XYZR_CLOUD res3;
    XYZR_CLOUD res4;
    XYZR_CLOUD res5;

    c_point p1 = c_point(config->ROBOT_SIZE_X[1] + DOCK_SIZE_X[1] + POINTDOCK_MARGIN, 0.0);
    qDebug() << "config robot size is " << config->ROBOT_SIZE_X[1];
    c_point p2 = c_point(p1.first - 0.05, p1.second + 0.16248);
    c_point p4 = c_point(p2.first,  p2.second+0.03);
    c_point p6 = c_point(p4.first +0.18, p4.second);
    c_point p3 = c_point(p1.first -0.05, p1.second- 0.16248);
    c_point p5 = c_point(p3.first, p3.second-0.03);
    c_point p7 = c_point(p5.first+ 0.18, p5.second);

    res2 = generateSamplePoints(p4,p2,40);
    res3 = generateSamplePoints(p2,p1,80);
    res4 = generateSamplePoints(p1,p3,80);
    res5 = generateSamplePoints(p3,p5,40);

    c_pointList temp;

    for (const auto& pt : res2.pts) 
    {
        frame.pts.push_back(pt);
        temp.push_back(c_point(pt.x,pt.y));
    }
    for (const auto& pt : res3.pts) 
    {
        frame.pts.push_back(pt);
        temp.push_back(c_point(pt.x,pt.y));
    }
    for (const auto& pt : res4.pts) 
    {
        frame.pts.push_back(pt);
        temp.push_back(c_point(pt.x,pt.y));
    }
    for (const auto& pt : res5.pts) 
    {
        frame.pts.push_back(pt);
        temp.push_back(c_point(pt.x,pt.y));
    }
    line_points.push_back(temp);

    return frame;
}


XYZR_CLOUD L_DOCKINGCONTROL::generateSamplePoints(const c_point& p1, const c_point& p2, int n)
{
    XYZR_CLOUD cloud;
    if (n < 2) 
    {
        return cloud;
    }
    float dx = (p2.first - p1.first) / (n - 1);
    float dy = (p2.second - p1.second) / (n - 1);

    for (int i = 0; i < n; ++i) 
    {
        PT_XYZR pt;
        pt.x = p1.first + i * dx;
        pt.y = p1.second + i * dy;
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

Eigen::Vector3d L_DOCKINGCONTROL::calculateCenter(const KFRAME& kframe) 
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

Eigen::Matrix4d L_DOCKINGCONTROL::calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to) 
{
    Eigen::Matrix4d translationMatrix = Eigen::Matrix4d::Identity();
    translationMatrix(0, 3) = to[0] - from[0];
    translationMatrix(1, 3) = to[1] - from[1];
    translationMatrix(2, 3) = to[2] - from[2];
    return translationMatrix;
}



lattice_primitives L_DOCKINGCONTROL::critic_primitives(const std::vector<lattice_primitives>& lattice_primitives)
{

    struct lattice_primitives out;
    double max_cost = -999.;
    int max_index =0;
    int index =0;

    if(lattice_primitives.size() == 0)
    {
        return out;
    }

    for(const auto& lattice_primitive : lattice_primitives)
    {
        double cost =0.0;
        double angle_offset = lattice_primitive.angle_offset;
        double dist = lattice_primitive.pos_offset;
        double accel_w = std::abs(lattice_primitive.accel_w);
        double max_w = std::abs(lattice_primitive.max_w);

        cost = L_CRITICS_ANGLE_SCALE* angle_offset + L_CRITICS_DIST_SCALE * dist + L_CRITICS_MAX_W_SCALE * max_w + L_CRITICS_ACCEL_SCALE* accel_w ;

        if(max_cost< cost)
        {
            max_cost = cost;
            max_index = index;
        }

        index++;
    }
    out = lattice_primitives[max_index];

    return out;
}

std::vector<lattice_primitives> L_DOCKINGCONTROL::make_primitives(const Eigen::Matrix4d& goal_pos)
{
        double direction = 0.0;

        std::vector<lattice_primitives> primitives;
        Eigen::Vector3d end_pose = Eigen::Vector3d(goal_pos(0,3), goal_pos(1,3), std::atan2(goal_pos(1,0), goal_pos(0,0)));
        direction = (end_pose[1] >= 0.0) ? 1.0 : -1.0;
        std::vector<double> angle_sample;

        if(direction > 0)
        {
            for(double angle =0; angle <= 90*M_PI / 180; angle += 5*M_PI / 180)
            {
                angle_sample.push_back(angle);
            }
        }
        else
        {
            for(double angle =0; angle >= -90*M_PI / 180; angle -= 5*M_PI / 180)
            {
                angle_sample.push_back(angle);
            }
        }

        for(auto angle_deg : angle_sample)
        {
            double start_angle_rad = angle_deg;

            lattice_primitives sample_primitve;
            Eigen::Vector3d start_pose = Eigen::Vector3d(0.0, 0.0, start_angle_rad);
            std::vector<double> w_accel_sample;

            if(direction > 0)
            {
                for(double w_accel = 0.0; w_accel > -sampling_w_accel_max; w_accel -= sampling_w_accel_step)
                {
                    w_accel_sample.push_back(w_accel);
                }
            }
            else{
                for(double w_accel = 0.0; w_accel < sampling_w_accel_max; w_accel += sampling_w_accel_step)
                {
                    w_accel_sample.push_back(w_accel);
                }
            }


            for (const auto& w_accel : w_accel_sample) 
            {
                std::vector<double> w_max_sample;

                if(direction > 0)
                {
                    for(double w_max = 0.0; w_max > -sampling_w_max; w_max -= sampling_w_max_step)
                    {
                        w_max_sample.push_back(w_max);
                    }
                }
                else{
                    for(double w_max = 0.0; w_max < sampling_w_max; w_max += sampling_w_max_step)
                    {
                        w_max_sample.push_back(w_max);
                    }
                }

                for(const auto& w_max_ : w_max_sample)
                {
                    double trajectory_radius = 0.0;
                    double max_sample_time = 10.0;
                    double v = 0.1;
                    double dt = 0.05;
                    bool sampling_done = false;
                    double sampling_time = 0.0;
                    double w_old =0.0;
                    double angle_offset_= 0.0;
                    double pos_offset_= 0.0;

                    std::vector<Eigen::Vector3d> trajectory;
                    Eigen::Vector3d pose = Eigen::Vector3d(0.0, 0.0, 0.0);
                    trajectory_radius = (w_max_ == 0.0) ? 0.0 : (v / w_max_);
                    pose = start_pose;

                    for(double t = 0.0; t < max_sample_time; t += dt)
                    {
                        double w_ = w_old + w_accel *dt;
                        w_ = std::clamp(w_,-w_max_ ,w_max_);
                        double dx = v * cos(pose[2]) * dt;
                        double dy = v * sin(pose[2]) * dt;
                        double dtheta = w_ * dt;

                        pose[0] += dx;
                        pose[1] += dy;
                        pose[2] += dtheta;
                        pose[2] = normalizeAngle(pose[2]);

                        double dist_to_goal = sqrt(pow(pose[0] - end_pose[0], 2) + pow(pose[1] - end_pose[1], 2));
                        double dist_to_yaw = abs(pose[2] - end_pose[2]);

                        w_old = w_;

                        if(dist_to_goal > sampling_break_dist_scale*sqrt(end_pose[0]*end_pose[0] + end_pose[1]*end_pose[1]))
                        {
                            break;
                        }

                        if(dist_to_goal < sampling_dist_threshold && dist_to_yaw < sampling_angle_threshold)
                        {
                            trajectory.push_back(pose);
                            sampling_done = true;
                            sampling_time = t;
                            angle_offset_ = dist_to_yaw;
                            pos_offset_ = dist_to_goal;
                            break;
                        }
                        trajectory.push_back(pose);
                    }

                    if(sampling_done)
                    {
                        sample_primitve.start_pose = se2_to_TF(start_pose);
                        sample_primitve.end_pose = se2_to_TF(trajectory.back());
                        sample_primitve.trajectory_radius = trajectory_radius;
                        sample_primitve.v = v;
                        sample_primitve.max_w = w_max_;
                        sample_primitve.sample_time = sampling_time;
                        sample_primitve.turn_left = (direction != 1.0);
                        sample_primitve.trajectory = trajectory;
                        sample_primitve.angle_offset = angle_offset_;
                        sample_primitve.pos_offset = pos_offset_;
                        primitives.push_back(sample_primitve);
                        }
                    }
                }
            }

    return primitives;
}


// check condition
int L_DOCKINGCONTROL::is_everything_fine()
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



double L_DOCKINGCONTROL::Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG)
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

    const double cost_threshold = ICP_COST_THRESHOLD*ICP_COST_THRESHOLD;//0.5 * 0.5;//00000.0;
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
        if(num_correspondence < ICP_CORRESPONDENCE_THRESHOLD)
        {
            //printf("[frm_icp] not enough correspondences, %d!!\n", num_correspondence);
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


