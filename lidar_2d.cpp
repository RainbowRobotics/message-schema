#include "lidar_2d.h"

LIDAR_2D::LIDAR_2D(QObject *parent) : QObject{parent}
{
}

LIDAR_2D::~LIDAR_2D()
{
    if(grab_thread_f != NULL)
    {
        grab_flag_f = false;
        grab_thread_f->join();
        grab_thread_f = NULL;
    }

    #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
    if(grab_thread_b != NULL)
    {
        grab_flag_b = false;
        grab_thread_b->join();
        grab_thread_b = NULL;
    }
    #endif

    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

void LIDAR_2D::open()
{
    // check simulation mode
    if(config->USE_SIM == 1)
    {
        printf("[LIDAR] simulation mode\n");
        return;
    }

    // start grab loop
    if(grab_thread_f == NULL)
    {
        grab_flag_f = true;
        grab_thread_f = new std::thread(&LIDAR_2D::grab_loop_f, this);
    }

    #if defined(USE_AMR_400) || defined(USE_AMR_400_LAKI)
    if(grab_thread_b == NULL)
    {
        grab_flag_b = true;
        grab_thread_b = new std::thread(&LIDAR_2D::grab_loop_b, this);
    }
    #endif

    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LIDAR_2D::a_loop, this);
    }
}

void LIDAR_2D::sync_f()
{
    is_sync_f = true;
    logger->write_log("[LIDAR] front time sync");
}

void LIDAR_2D::sync_b()
{
    is_sync_b = true;
    logger->write_log("[LIDAR] back time sync");
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan_f()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan_f;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan_b()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan_b;
    mtx.unlock();

    return res;
}

std::vector<Eigen::Vector3d> LIDAR_2D::get_cur_scan()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan;
    mtx.unlock();

    return res;
}

bool LIDAR_2D::is_shadow(const double r1, const double r2, const double included_angle, const double min_angle_tan, const double max_angle_tan)
{
    const double perpendicular_y = r2 * std::sin(included_angle);
    const double perpendicular_x = r1 - r2 * std::cos(included_angle);
    const double perpendicular_tan = std::abs(perpendicular_y) / perpendicular_x;

    if(perpendicular_tan > 0)
    {
        if(perpendicular_tan < min_angle_tan)
        {
            return true;
        }
    }
    else
    {
        if(perpendicular_tan > max_angle_tan)
        {
            return true;
        }
    }
    return false;
}

std::vector<Eigen::Vector3d> LIDAR_2D::scan_shadow_filter(std::vector<Eigen::Vector3d>& pts, int shadow_window)
{    
    const double min_angle_tan = std::tan(5.0*D2R);
    const double max_angle_tan = std::tan(175.0*D2R);

    std::vector<Eigen::Vector3d> filtered_pts;
    for(size_t p = 0; p < pts.size(); p++)
    {
        Eigen::Vector3d P1 = pts[p];
        double r1 = std::sqrt(P1(0)*P1(0) + P1(1)*P1(1));

        bool is_good = true;
        for(int k = -shadow_window; k < shadow_window; k++)
        {
            int neighbor_idx = p + k;
            if(neighbor_idx >= 0 && neighbor_idx < (int)pts.size())
            {
                Eigen::Vector3d P2 = pts[neighbor_idx];
                double r2 = std::sqrt(P2(0)*P2(0) + P2(1)*P2(1));

                // calculate included angle
                double angle_1 = std::atan2(P1.y(), P1.x());
                double angle_2 = std::atan2(P2.y(), P2.x());
                double included_angle = std::abs(deltaRad(angle_1, angle_2));
                if(is_shadow(r1, r2, included_angle, min_angle_tan, max_angle_tan))                
                {
                    is_good = false;
                    break;
                }
            }
        }

        if(is_good)
        {
            filtered_pts.push_back(P1);
        }
    }

    return filtered_pts;
}


#if defined(USE_SRV)
void LIDAR_2D::grab_loop_f()
{
    //sudo adduser $USER dialout
    //return;

    logger->write_log("[LIDAR] start grab loop", "Green", true, false);

    sl::ILidarDriver* drv = *sl::createLidarDriver();
    if(!drv)
    {
        logger->write_log("[LIDAR] driver init failed", "Red", true, false);
        return;
    }
    logger->write_log("[LIDAR] driver init success", "Green", true, false);

    sl::IChannel* channel = (*sl::createSerialPortChannel("/dev/ttyRP0", 256000));
    logger->write_log("[LIDAR] channel init success", "Green", true, false);

    if(!channel->open())
    {
        logger->write_log("[LIDAR] port open failed", "Red", true, false);
        return;
    }
    else
    {
        // close for next operation
        channel->close();
    }

    if(drv->connect(channel) != SL_RESULT_OK)
    {
        logger->write_log("[LIDAR] connection failed", "Red", true, false);
        return;
    }

    logger->write_log("[LIDAR] connect success", "Green", true, false);

    std::vector<sl::LidarScanMode> modes;
    drv->getAllSupportedScanModes(modes);
    if(modes.size() == 0)
    {
        logger->write_log("[LIDAR] no mode failed", "Red", true, false);
        return;
    }

    sl::LidarScanMode mode;
    double per_sample = modes[0].us_per_sample;
    for(size_t p = 0; p < modes.size(); p++)
    {
        printf("%s[%d] us_per_sample:%f\n", modes[p].scan_mode, modes[p].id, modes[p].us_per_sample);
        if(modes[p].us_per_sample < per_sample)
        {
            per_sample = modes[p].us_per_sample;
            mode = modes[p];
        }
    }
    per_sample *= U2S;

    if(drv->setMotorSpeed(DEFAULT_MOTOR_SPEED) != SL_RESULT_OK)
    {
        logger->write_log("[LIDAR] lidar set motor speed failed", "Red", true, false);
        return;
    }

    if(drv->startScanExpress(0, mode.id, 0, &mode) != SL_RESULT_OK)
    {
        logger->write_log("[LIDAR] start scan failed", "Red", true, false);
        return;
    }
    logger->write_log(QString("[LIDAR] lidar scan start, MODE :%1").arg(mode.scan_mode), "Green", true, false);

    is_connected_f = true;

    int drop_cnt = 10;
    while(grab_flag_f)
    {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        sl_u64 timeStamp = 0;
        if(drv->grabScanDataHqWithTimeStamp(nodes, count, timeStamp) == SL_RESULT_OK)
        {
            drv->ascendScanData(nodes, count);

            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // time sync
            double pc_t = get_time();
            double lidar_t = (double)timeStamp*U2S;

            if(is_sync_f)
            {
                is_sync_f = false;
                offset_t_f = pc_t - lidar_t;

                is_synced_f = true;

                logger->write_log(QString("[LIDAR] sync, offset_t_f: %1").arg(offset_t_f));
            }

            // check lidar, mobile sync
            if(is_synced_f == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            double t0 = lidar_t + offset_t_f;
            double t1 = lidar_t + (count*per_sample) + offset_t_f;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;

            const int step = 1;
            for(size_t p = 0; p < count; p+=step)
            {
                if(nodes[p].dist_mm_q2 == 0)
                {
                    continue;
                }

                double t = t0 + p*per_sample;
                double deg = (nodes[p].angle_z_q14 * 90.0)/16384.0;
                double dist = (nodes[p].dist_mm_q2/4.0)/1000.0;
                double rssi = (double)nodes[p].quality;

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // check invalid lidar frame
            if(raw_pts.size() < 100)
            {
                // drop
                logger->write_log("[LIDAR] not enough points, drop");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            Eigen::Vector3d min_pose = pose_storage.back().pose;
            double min_dt = 99999999;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_pose = pose_storage[p].pose;
                }
            }

            MOBILE_POSE mo;
            mo.t = t1;
            mo.pose = min_pose;

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = raw_pts;
            frm.mo = mo;
            raw_que_f.push(frm);

            last_t_f = t1;

            // que overflow control
            if(raw_que_f.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_f.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    logger->write_log("[LIDAR] stop, front", "Green", true, false);
}

void LIDAR_2D::a_loop()
{
    printf("[LIDAR] start a loop\n");
    while(a_flag)
    {
        RAW_FRAME frm0;
        if(raw_que_f.try_pop(frm0))
        {
            std::vector<Eigen::Vector3d> filtered_pts_f = scan_shadow_filter(frm0.dsk, 5);

            Eigen::Matrix4d tf_f = ZYX_to_TF(config->LIDAR_TF_F);
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> pts;
            std::vector<Eigen::Vector3d> pts_f;
            for(size_t p = 0; p < filtered_pts_f.size(); p++)
            {
                Eigen::Vector3d P = tf_f.block(0,0,3,3)*filtered_pts_f[p] + tf_f.block(0,3,3,1);

                if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                   P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                {
                    continue;
                }

                pts.push_back(P);
                pts_f.push_back(P);
                reflects.push_back(frm0.reflects[p]);
            }

            // update
            mtx.lock();
            cur_scan = pts;
            cur_scan_f = pts_f;
            mtx.unlock();

            FRAME merge_frm;
            merge_frm.t = frm0.t1;
            merge_frm.mo = frm0.mo;
            merge_frm.reflects = reflects;
            merge_frm.pts = pts;
            scan_que.push(merge_frm);

            // for que overflow
            if(scan_que.unsafe_size() > 50)
            {
                FRAME temp;
                scan_que.try_pop(temp);
            }
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop a loop\n");
}
#endif

#if defined(USE_AMR_400)
void LIDAR_2D::grab_loop_f()
{
    printf("[LIDAR] start grab loop, front\n");

    // sensors
    std::string ip_str = "192.168.2.10"; // front right lidar ip
    sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(ip_str);
    sick::types::port_t tcp_port{2122};

    // host pc
    std::string host_ip_str = "192.168.2.2";

    sick::datastructure::CommSettings comm_settings;
    comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str); // laptop(temporal)
    comm_settings.host_udp_port = 0;
    comm_settings.e_interface_type = 4;
    comm_settings.publishing_frequency = 2; // 1:25 hz, 2:12.5 hz

    // create instance
    auto safety_scanner = std::make_unique<sick::SyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings);

    is_connected_f = true;

    int drop_cnt = 10;
    while(grab_flag_f)
    {
        if(safety_scanner->isDataAvailable())
        {
            auto timeout = boost::posix_time::seconds(1);
            sick::datastructure::Data data = safety_scanner->receive(timeout);

            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // time sync
            double pc_t = get_time();
            double lidar_t = data.getDataHeaderPtr()->getTimestampTime()*M2S;

            if(is_sync_f)
            {
                is_sync_f = false;
                offset_t_f = pc_t - lidar_t;

                is_synced_f = true;
                printf("[LIDAR] sync, lidar_t_f: %f, offset_t_f: %f\n", lidar_t, (double)offset_t_f);
            }

            // check lidar, mobile sync
            if(is_synced_f == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            std::vector<sick::ScanPoint> sp = data.getMeasurementDataPtr()->getScanPointsVector();
            int num_scan_point = sp.size();
            double scan_time = data.getDerivedValuesPtr()->getScanTime()*M2S;
            //double time_increment = data.getDerivedValuesPtr()->getInterbeamPeriod()*U2S;
            double time_increment = scan_time/num_scan_point;

            double t0 = lidar_t + offset_t_f;
            double t1 = t0 + scan_time;

            //double t1 = lidar_t + offset_t_f;
            //double t0 = t1 - scan_time;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;

            // parsing            
            for(size_t p = 0; p < sp.size(); p++)
            {
                double t = t0 + time_increment*p;
                double deg = sp[p].getAngle();
                double dist = (double)sp[p].getDistance()/1000.0; // mm to meter
                double rssi = sp[p].getReflectivity();                               

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                if((deg < -47.5 + angle_offset) || (deg > 227.5 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1)
            {
                // drop
                printf("[LIDAR] front lidar, invalid mobile poses\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // precise deskewing
            Eigen::Matrix4d tf0 = se2_to_TF(pose_storage[idx0].pose);
            Eigen::Matrix4d tf1 = se2_to_TF(pose_storage[idx1].pose);
            //Eigen::Matrix4d dtf = tf0.inverse()*tf1;
            Eigen::Matrix4d dtf = Eigen::Matrix4d::Identity();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];
                double alpha = (t-t0)/(t1-t0);
                Eigen::Matrix4d tf = intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);
                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = t0;
            mo.pose = TF_to_se2(tf0);
            //printf("[LIDAR_F] t:%f, pose:%.3f, %.3f, %.3f\n", mo.t, mo.pose[0], mo.pose[1], mo.pose[2]*R2D);

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = dsk_pts;
            frm.mo = mo;
            raw_que_f.push(frm);

            last_t_f = t1;

            // que overflow control
            if(raw_que_f.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_f.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, front\n");
}

void LIDAR_2D::grab_loop_b()
{
    printf("[LIDAR] start grab loop, back\n");

    // sensors
    std::string ip_str = "192.168.2.11"; // back left lidar ip
    sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(ip_str);
    sick::types::port_t tcp_port{2122};

    // host pc
    std::string host_ip_str = "192.168.2.2";

    sick::datastructure::CommSettings comm_settings;
    comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str); // laptop(temporal)
    comm_settings.host_udp_port = 0;
    comm_settings.e_interface_type = 4;
    comm_settings.publishing_frequency = 2; // 1:25 hz, 2:12.5 hz

    // create instance
    auto safety_scanner = std::make_unique<sick::SyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings);

    is_connected_b = true;

    int drop_cnt = 10;
    while(grab_flag_b)
    {
        if(safety_scanner->isDataAvailable())
        {
            auto timeout = boost::posix_time::seconds(1);
            sick::datastructure::Data data = safety_scanner->receive(timeout);

            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // time sync
            double pc_t = get_time();
            double lidar_t = data.getDataHeaderPtr()->getTimestampTime()*M2S;

            if(is_sync_b)
            {
                is_sync_b = false;
                offset_t_b = pc_t - lidar_t;

                is_synced_b = true;
                printf("[LIDAR] sync, lidar_t_b: %f, offset_t_b: %f\n", lidar_t, (double)offset_t_b);
            }

            // check lidar, mobile sync
            if(is_synced_b == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            std::vector<sick::ScanPoint> sp = data.getMeasurementDataPtr()->getScanPointsVector();
            int num_scan_point = sp.size();
            double scan_time = data.getDerivedValuesPtr()->getScanTime()*M2S;
            //double time_increment = data.getDerivedValuesPtr()->getInterbeamPeriod()*U2S;
            double time_increment = scan_time/num_scan_point;

            double t0 = lidar_t + offset_t_b;
            double t1 = t0 + scan_time;

            //double t1 = lidar_t + offset_t_b;
            //double t0 = t1 - scan_time;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_b)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;

            // parsing            
            for(size_t p = 0; p < sp.size(); p++)
            {
                double t = t0 + time_increment*p;
                double deg = sp[p].getAngle();
                double dist = (double)sp[p].getDistance()/1000.0; // mm to meter
                double rssi = sp[p].getReflectivity();

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                if((deg < -47.5 + angle_offset) || (deg > 227.5 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1)
            {
                // drop
                printf("[LIDAR] back lidar, invalid mobile poses\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // precise deskewing
            Eigen::Matrix4d tf0 = se2_to_TF(pose_storage[idx0].pose);
            Eigen::Matrix4d tf1 = se2_to_TF(pose_storage[idx1].pose);
            //Eigen::Matrix4d dtf = tf0.inverse()*tf1;
            Eigen::Matrix4d dtf = Eigen::Matrix4d::Identity();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];
                double alpha = (t-t0)/(t1-t0);
                Eigen::Matrix4d tf = intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);
                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = t0;
            mo.pose = TF_to_se2(tf0);
            //printf("[LIDAR_B] t:%f, pose:%.3f, %.3f, %.3f\n", mo.t, mo.pose[0], mo.pose[1], mo.pose[2]*R2D);

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = dsk_pts;
            frm.mo = mo;
            raw_que_b.push(frm);

            last_t_b = t1;

            // que overflow control
            if(raw_que_b.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_b.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, back\n");
}

void LIDAR_2D::a_loop()
{
    std::vector<RAW_FRAME> storage;

    printf("[LIDAR] start a loop\n");
    while(a_flag)
    {
        RAW_FRAME frm0;
        if(raw_que_f.try_pop(frm0))
        {
            // wait
            while(frm0.t1 > last_t_b && a_flag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // find best pair
            RAW_FRAME frm;
            while(raw_que_b.try_pop(frm) && a_flag)
            {
                storage.push_back(frm);
            }

            int min_idx = -1;
            double min_dt = 99999999;
            for(size_t p = 0; p < storage.size(); p++)
            {
                double dt = std::abs(storage[p].t0 - frm0.t0);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_idx = p;
                }
            }

            // merge two lidar frame
            Eigen::Matrix4d tf_f = flip_lidar_tf(ZYX_to_TF(config->LIDAR_TF_F));
            Eigen::Matrix4d tf_b = flip_lidar_tf(ZYX_to_TF(config->LIDAR_TF_B));

            if(min_idx >= 0)
            {
                RAW_FRAME frm1 = storage[min_idx];

                // apply shadow filter for frm0 and frm1
                std::vector<Eigen::Vector3d> filtered_pts_f = scan_shadow_filter(frm0.dsk, 3);
                std::vector<Eigen::Vector3d> filtered_pts_b = scan_shadow_filter(frm1.dsk, 3);

                // lidar frame to robot frame
                std::vector<double> reflects_f;
                std::vector<Eigen::Vector3d> pts_f;

                for(size_t p = 0; p < filtered_pts_f.size(); p++)
                {
                    Eigen::Vector3d P = tf_f.block(0,0,3,3)*filtered_pts_f[p] + tf_f.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }
                    pts_f.push_back(P);
                    reflects_f.push_back(frm0.reflects[p]);
                }

                // lidar frame to robot frame
                std::vector<double> reflects_b;
                std::vector<Eigen::Vector3d> pts_b;
                for(size_t p = 0; p < filtered_pts_b.size(); p++)
                {
                    Eigen::Vector3d P = tf_b.block(0,0,3,3)*filtered_pts_b[p] + tf_b.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }
                    pts_b.push_back(P);
                    reflects_b.push_back(frm1.reflects[p]);
                }

                // icp for merge
                Eigen::Matrix4d dG = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm1.mo.pose);

                // merge
                std::vector<double> reflects = reflects_f;
                std::vector<Eigen::Vector3d> pts = pts_f;
                for(size_t p = 0; p < pts_b.size(); p++)
                {
                    Eigen::Vector3d P = pts_b[p];
                    Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);
                    pts.push_back(_P);
                    reflects.push_back(reflects_b[p]);

                    pts_b[p] = _P;
                }

                // roll pitch compensation
                Eigen::Vector3d imu = mobile->get_imu();
                Eigen::Matrix3d rot = Sophus::SO3d::exp(imu).matrix();
                rot = remove_rz(rot);

                std::vector<Eigen::Vector3d> pts_outlier;
                std::vector<Eigen::Vector3d> pts_inlier;
                std::vector<double> reflects_inlier;
                for(size_t p = 0; p < pts.size(); p++)
                {
                    Eigen::Vector3d P = pts[p];
                    Eigen::Vector3d _P = rot*P;
                    if(_P[2] < -0.15)
                    {
                        //pts_outlier.push_back(Eigen::Vector3d(_P[0], _P[1], 0));
                        pts_outlier.push_back(Eigen::Vector3d(P[0], P[1], 0));
                    }
                    else
                    {
                        //pts_inlier.push_back(Eigen::Vector3d(_P[0], _P[1], 0));
                        pts_inlier.push_back(Eigen::Vector3d(P[0], P[1], 0));
                        reflects_inlier.push_back(reflects[p]);
                    }
                }

                // update result
                if(config->USE_IMU)
                {
                    pts = pts_inlier;
                    reflects = reflects_inlier;
                }

                // update
                mtx.lock();
                cur_scan_outlier = pts_outlier;
                cur_scan = pts;
                cur_scan_f = pts_f;
                cur_scan_b = pts_b;
                mtx.unlock();

                FRAME merge_frm;
                merge_frm.t = frm0.mo.t;
                merge_frm.mo = frm0.mo;
                merge_frm.reflects = reflects;
                merge_frm.pts = pts;
                scan_que.push(merge_frm);

                // for que overflow
                if(scan_que.unsafe_size() > 50)
                {
                    FRAME temp;
                    scan_que.try_pop(temp);
                }

                storage.erase(storage.begin(), storage.begin()+min_idx);
            }
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop a loop\n");
}
#endif

#if defined(USE_AMR_400_LAKI)
void LIDAR_2D::grab_loop_f()
{
    printf("[LIDAR] start grab loop, front\n");

    // init lidar (host pc should be set ip 192.168.2.2)
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2367", "192.168.2.10", "8888"));
    is_connected_f = true;
    is_synced_f = true;

    // gap time each points
    const double point_interval = 32*U2S;

    int drop_cnt = 10;
    while(grab_flag_f)
    {
        repark_t temp_pack;
        if(lidar->get_repackedpack(temp_pack))
        {
            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            if(temp_pack.maxdots == 0 || temp_pack.maxdots > 3599)
            {
                logger->write_log("[LIDAR] invalid data from front lidar\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            double lidar_t = get_time();

            // lidar frame synced ref time
            double t0 = lidar_t - (temp_pack.maxdots * point_interval);
            double t1 = lidar_t;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_f)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;
            for (int p = 0; p < temp_pack.maxdots; p++)
            {
                double t = t0 + (p*point_interval);
                double deg = (double)temp_pack.dotcloud[p].angle/100; // deg, 0~360
                double dist = ((double)temp_pack.dotcloud[p].distance)/1000; // meter
                double rssi = (double)temp_pack.dotcloud[p].rssi;

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                if((deg < 45.0 + angle_offset) || (deg > 315.0 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(int p = (int)pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1 || idx0 == idx1)
            {
                // drop                
                printf("[LIDAR] front sync drop, t_first:%f, t_last:%f, t0:%f, t1:%f\n", pose_storage.front().t, pose_storage.back().t, t0, t1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            Eigen::Vector3d min_pose = pose_storage[idx1].pose;
            double min_dt = 99999999;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_pose = pose_storage[p].pose;
                }
            }

            MOBILE_POSE mo;
            mo.t = t1;
            mo.pose = min_pose;

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = raw_pts;
            frm.mo = mo;
            raw_que_f.push(frm);

            last_t_f = t1;

            // que overflow control
            if(raw_que_f.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_f.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, front\n");
}

void LIDAR_2D::grab_loop_b()
{
    printf("[LIDAR] start grab loop, back\n");

    // init lidar
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2368", "192.168.2.11", "8888"));
    is_connected_b = true;
    is_synced_b = true;

    // gap time each points
    const double point_interval = 32*U2S;

    int drop_cnt = 10;
    while(grab_flag_b)
    {
        repark_t temp_pack;
        if(lidar->get_repackedpack(temp_pack))
        {
            // initial drop
            if(drop_cnt > 0)
            {
                drop_cnt--;
                continue;
            }

            if(temp_pack.maxdots == 0 || temp_pack.maxdots > 3599)
            {
                logger->write_log("[LIDAR] no data from back lidar\n");
                continue;
            }

            double lidar_t = get_time();

            // lidar frame synced ref time
            double t0 = lidar_t - (temp_pack.maxdots * point_interval);
            double t1 = lidar_t;

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                // drop
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // wait
            while(t1 > mobile->last_pose_t && grab_flag_b)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> raw_pts;
            for (int p = 0; p < temp_pack.maxdots; p++)
            {
                double t = t0 + (p*point_interval);
                double deg = (double)temp_pack.dotcloud[p].angle/100; // deg, 0~360
                double dist = ((double)temp_pack.dotcloud[p].distance)/1000; // meter
                double rssi = (double)temp_pack.dotcloud[p].rssi;

                // dist filter
                if(dist < 0.05 || dist > config->LIDAR_MAX_RANGE)
                {
                    continue;
                }

                // angle filterping
                if((deg < 45.0 + angle_offset) || (deg > 315.0 - angle_offset))
                {
                    continue;
                }

                double x = dist * std::cos(deg*D2R);
                double y = dist * std::sin(deg*D2R);
                if(!isfinite(x) || !isfinite(y))
                {
                    continue;
                }

                times.push_back(t);
                reflects.push_back(rssi);
                raw_pts.push_back(Eigen::Vector3d(x, y, 0));
            }

            // get boundary mobile pose
            int idx0 = -1;
            for(int p = (int)pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }

            // check
            if(idx0 == -1 || idx1 == -1 || idx0 == idx1)
            {
                // drop
                printf("[LIDAR] back sync drop, t_first:%f, t_last:%f, t0:%f, t1:%f\n", pose_storage.front().t, pose_storage.back().t, t0, t1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            Eigen::Vector3d min_pose = pose_storage[idx1].pose;
            double min_dt = 99999999;
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_pose = pose_storage[p].pose;
                }
            }

            MOBILE_POSE mo;
            mo.t = t1;
            mo.pose = min_pose;

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.times = times;
            frm.reflects = reflects;
            frm.dsk = raw_pts;
            frm.mo = mo;
            raw_que_b.push(frm);

            last_t_b = t1;

            // que overflow control
            if(raw_que_b.unsafe_size() > 50)
            {
                RAW_FRAME temp;
                raw_que_b.try_pop(temp);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop, back\n");
}

void LIDAR_2D::a_loop()
{
    std::vector<RAW_FRAME> storage;

    printf("[LIDAR] start a loop\n");
    while(a_flag)
    {
        RAW_FRAME frm0;
        if(raw_que_f.try_pop(frm0))
        {
            // wait
            while(frm0.t1 > last_t_b && a_flag)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // find best pair
            RAW_FRAME frm;
            while(raw_que_b.try_pop(frm) && a_flag)
            {
                storage.push_back(frm);
            }

            int min_idx = -1;
            double min_dt = 99999999;
            for(size_t p = 0; p < storage.size(); p++)
            {
                double dt = std::abs(storage[p].t1 - frm0.t1);
                if(dt < min_dt)
                {
                    min_dt = dt;
                    min_idx = p;
                }
            }

            // merge two lidar frame
            Eigen::Matrix4d tf_f = flip_lidar_tf(ZYX_to_TF(config->LIDAR_TF_F));
            Eigen::Matrix4d tf_b = flip_lidar_tf(ZYX_to_TF(config->LIDAR_TF_B));

            if(min_idx >= 0)
            {
                RAW_FRAME frm1 = storage[min_idx];

                // apply shadow filter for frm0 and frm1
                std::vector<Eigen::Vector3d> filtered_pts_f = scan_shadow_filter(frm0.dsk, 3);
                std::vector<Eigen::Vector3d> filtered_pts_b = scan_shadow_filter(frm1.dsk, 3);

                // lidar frame to robot frame
                std::vector<double> reflects_f;
                std::vector<Eigen::Vector3d> pts_f;

                for(size_t p = 0; p < filtered_pts_f.size(); p++)
                {
                    Eigen::Vector3d P = tf_f.block(0,0,3,3)*filtered_pts_f[p] + tf_f.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }
                    pts_f.push_back(P);
                    reflects_f.push_back(frm0.reflects[p]);
                }

                // lidar frame to robot frame
                std::vector<double> reflects_b;
                std::vector<Eigen::Vector3d> pts_b;
                for(size_t p = 0; p < filtered_pts_b.size(); p++)
                {
                    Eigen::Vector3d P = tf_b.block(0,0,3,3)*filtered_pts_b[p] + tf_b.block(0,3,3,1);

                    if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                       P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                    {
                        continue;
                    }
                    pts_b.push_back(P);
                    reflects_b.push_back(frm1.reflects[p]);
                }

                // icp for merge
                Eigen::Matrix4d dG = se2_to_TF(frm0.mo.pose).inverse()*se2_to_TF(frm1.mo.pose);

                // merge
                std::vector<double> reflects = reflects_f;
                std::vector<Eigen::Vector3d> pts = pts_f;
                for(size_t p = 0; p < pts_b.size(); p++)
                {
                    Eigen::Vector3d P = pts_b[p];
                    Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);
                    pts.push_back(_P);
                    reflects.push_back(reflects_b[p]);

                    pts_b[p] = _P;
                }

                // roll pitch compensation
                Eigen::Vector3d imu = mobile->get_imu();
                Eigen::Matrix3d rot = Sophus::SO3d::exp(imu).matrix();
                rot = remove_rz(rot);

                std::vector<Eigen::Vector3d> pts_outlier;
                std::vector<Eigen::Vector3d> pts_inlier;
                std::vector<double> reflects_inlier;
                for(size_t p = 0; p < pts.size(); p++)
                {
                    Eigen::Vector3d P = pts[p];
                    Eigen::Vector3d _P = rot*P;
                    if(_P[2] < -0.15)
                    {
                        //pts_outlier.push_back(Eigen::Vector3d(_P[0], _P[1], 0));
                        pts_outlier.push_back(Eigen::Vector3d(P[0], P[1], 0));
                    }
                    else
                    {
                        //pts_inlier.push_back(Eigen::Vector3d(_P[0], _P[1], 0));
                        pts_inlier.push_back(Eigen::Vector3d(P[0], P[1], 0));
                        reflects_inlier.push_back(reflects[p]);
                    }
                }

                // update result
                if(config->USE_IMU)
                {
                    pts = pts_inlier;
                    reflects = reflects_inlier;
                }

                // update
                mtx.lock();
                cur_scan_outlier = pts_outlier;
                cur_scan = pts;
                cur_scan_f = pts_f;
                cur_scan_b = pts_b;
                mtx.unlock();

                FRAME merge_frm;
                merge_frm.t = frm0.mo.t;
                merge_frm.mo = frm0.mo;
                merge_frm.reflects = reflects;
                merge_frm.pts = pts;
                scan_que.push(merge_frm);

                // for que overflow
                if(scan_que.unsafe_size() > 50)
                {
                    FRAME temp;
                    scan_que.try_pop(temp);
                }

                storage.erase(storage.begin(), storage.begin()+min_idx);
            }
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LIDAR] stop a loop\n");
}
#endif

