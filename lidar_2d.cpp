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

    if(grab_thread_b != NULL)
    {
        grab_flag_b = false;
        grab_thread_b->join();
        grab_thread_b = NULL;
    }

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
    if(config->SIM_MODE == 1)
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

    if(grab_thread_b == NULL)
    {
        grab_flag_b = true;
        grab_thread_b = new std::thread(&LIDAR_2D::grab_loop_b, this);
    }

    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LIDAR_2D::a_loop, this);
    }
}

void LIDAR_2D::sync_f()
{
    is_sync_f = true;
    printf("[LIDAR] front time sync\n");
}

void LIDAR_2D::sync_b()
{
    is_sync_b = true;
    printf("[LIDAR] back time sync\n");
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

#ifdef USE_LIDAR_SICK
void LIDAR_2D::grab_loop_f()
{
    printf("[LIDAR] start grab loop, front\n");

    // sensors
    std::string ip_str = "192.168.2.11";
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

    int drop_cnt = 100;
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
                raw_que_f.clear();

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
    std::string ip_str = "192.168.2.10";
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

    int drop_cnt = 100;
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
                raw_que_b.clear();

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
#endif

#ifdef USE_LIDAR_LAKI
void LIDAR_2D::grab_loop_f()
{
    printf("[LIDAR] start grab loop, front\n");

    // init lidar (host pc should be set ip 192.168.2.2)
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2367", "192.168.2.10", "8888"));
    is_connected_f = true;

    int drop_cnt = 100;
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

            // time sync
            double pc_t = get_time();
            double lidar_t = temp_pack.dotcloud[0].timestamp * U2S;

            if(is_sync_f)
            {
                is_sync_f = false;
                offset_t_f = pc_t - lidar_t;
                raw_que_f.clear();

                is_synced_f = true;
                printf("[LIDAR] sync, offset_t_f: %f\n", (double)offset_t_f);
            }

            // check lidar, mobile sync
            if(is_synced_f == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            double t0 = lidar_t + offset_t_f;
            double t1 = (temp_pack.dotcloud[temp_pack.maxdots-1].timestamp * U2S) + offset_t_f;

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
                double t = temp_pack.dotcloud[p].timestamp*U2S + offset_t_f;
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
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_tf = se2_to_TF(pose_storage[p].pose);
                }
            }
            Eigen::Matrix4d min_tf_inv = min_tf.inverse();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];

                int i1 = 0;
                for(int i = idx0; i <= idx1; i++)
                {
                    if(pose_storage[i].t > t)
                    {
                        i1 = i;
                        break;
                    }
                }

                int i0 = i1 - 1;
                if(i0 < 0)
                {
                    i0 = idx0;
                    i1 = idx1;
                }

                // get bound_t
                double _mo_t0 = pose_storage[i0].t;
                double _mo_t1 = pose_storage[i1].t;

                Eigen::Matrix4d _mo_tf0 = se2_to_TF(pose_storage[i0].pose);
                Eigen::Matrix4d _mo_tf1 = se2_to_TF(pose_storage[i1].pose);

                double alpha = (t-_mo_t0)/(_mo_t1-_mo_t0);
                Eigen::Matrix4d tf = intp_tf(alpha, _mo_tf0, _mo_tf1);

                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
                dsk_pts[p] = min_tf_inv.block(0,0,3,3)*dsk_pts[p] + min_tf_inv.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = min_t;
            mo.pose = TF_to_se2(min_tf);

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

    // init lidar
    std::unique_ptr<LakiBeamUDP> lidar(new LakiBeamUDP("192.168.2.2", "2368", "192.168.2.11", "8888"));
    is_connected_b = true;

    int drop_cnt = 100;
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

            // time sync
            double pc_t = get_time();
            double lidar_t = temp_pack.dotcloud[0].timestamp * U2S;

            if(is_sync_b)
            {
                is_sync_b = false;
                offset_t_b = pc_t - lidar_t;
                raw_que_b.clear();

                is_synced_b = true;
                printf("[LIDAR] sync, offset_t_b: %f\n", (double)offset_t_b);
            }

            // check lidar, mobile sync
            if(is_synced_b == false || mobile->is_synced == false)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            double t0 = lidar_t + offset_t_b;
            double t1 = (temp_pack.dotcloud[temp_pack.maxdots-1].timestamp * U2S) + offset_t_b;

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
                double t = temp_pack.dotcloud[p].timestamp*U2S + offset_t_b;
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
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t1);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_tf = se2_to_TF(pose_storage[p].pose);
                }
            }
            Eigen::Matrix4d min_tf_inv = min_tf.inverse();

            std::vector<Eigen::Vector3d> dsk_pts(raw_pts.size());
            for(size_t p = 0; p < raw_pts.size(); p++)
            {
                double t = times[p];

                int i1 = 0;
                for(int i = idx0; i <= idx1; i++)
                {
                    if(pose_storage[i].t > t)
                    {
                        i1 = i;
                        break;
                    }
                }

                int i0 = i1 - 1;
                if(i0 < 0)
                {
                    i0 = idx0;
                    i1 = idx1;
                }

                // get bound_t
                double _mo_t0 = pose_storage[i0].t;
                double _mo_t1 = pose_storage[i1].t;

                Eigen::Matrix4d _mo_tf0 = se2_to_TF(pose_storage[i0].pose);
                Eigen::Matrix4d _mo_tf1 = se2_to_TF(pose_storage[i1].pose);

                double alpha = (t-_mo_t0)/(_mo_t1-_mo_t0);
                Eigen::Matrix4d tf = intp_tf(alpha, _mo_tf0, _mo_tf1);

                dsk_pts[p] = tf.block(0,0,3,3)*raw_pts[p] + tf.block(0,3,3,1);
                dsk_pts[p] = min_tf_inv.block(0,0,3,3)*dsk_pts[p] + min_tf_inv.block(0,3,3,1);
            }

            MOBILE_POSE mo;
            mo.t = min_t;
            mo.pose = TF_to_se2(min_tf);

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
#endif

void LIDAR_2D::align(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1, Eigen::Matrix4d& dG)
{
    double t_st = get_time();

    // build kd_tree
    XYZR_CLOUD cloud;
    for(size_t p = 0; p < pts0.size(); p++)
    {
        PT_XYZR pt;
        pt.x = pts0[p][0];
        pt.y = pts0[p][1];
        pt.z = pts0[p][2];
        cloud.pts.push_back(pt);
    }

    KD_TREE_XYZR tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    double ref_th = TF_to_se2(dG)[2];

    double min_d = 99999999;
    double min_th = ref_th;
    for(double dth = -10.0*D2R; dth <= 10.0*D2R; dth += 0.5*D2R)
    {
        double th = ref_th + dth;
        Eigen::Matrix4d G = se2_to_TF(Eigen::Vector3d(0, 0, th));

        int cnt = 0;
        double sum_d = 0;
        for(size_t p = 0; p < pts1.size(); p++)
        {
            Eigen::Vector3d P1 = pts1[p];
            Eigen::Vector3d _P1 = G.block(0,0,3,3)*P1 + G.block(0,3,3,1);

            // find nn
            std::vector<unsigned int> ret_near_idxs(1);
            std::vector<double> ret_near_sq_dists(1);

            double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
            tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

            int nn_idx = ret_near_idxs[0];
            Eigen::Vector3d P0(cloud.pts[nn_idx].x, cloud.pts[nn_idx].y, cloud.pts[nn_idx].z);

            double d = (_P1-P0).norm();
            if(d > 1.0)
            {
                continue;
            }

            sum_d += d;
            cnt++;
        }

        double mean_d = sum_d/cnt;
        if(mean_d < min_d)
        {
            min_d = mean_d;
            min_th = th;
        }
    }

    // update
    dG = se2_to_TF(Eigen::Vector3d(0, 0, min_th));
    printf("[LIDAR] align, min_th:%.3f, dt:%.3f\n", min_th*R2D, get_time()-t_st);
}

void LIDAR_2D::icp(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1, Eigen::Matrix4d& dG)
{
    double t_st = get_time();

    // build kd_tree
    XYZR_CLOUD cloud;
    for(size_t p = 0; p < pts0.size(); p++)
    {
        PT_XYZR pt;
        pt.x = pts0[p][0];
        pt.y = pts0[p][1];
        pt.z = pts0[p][2];
        cloud.pts.push_back(pt);
    }

    KD_TREE_XYZR tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // points random selection
    std::vector<int> idx_list;
    for(size_t p = 0; p < pts1.size(); p++)
    {
        idx_list.push_back(p);
    }

    // solution
    Eigen::Matrix4d _dG = dG;

    // optimization param
    const int max_iter = 100;
    double lambda = 0.01;
    double first_err = 9999;
    double last_err = 9999;
    double convergence = 9999;
    int num_correspondence = 0;

    const double rmt_sigma = 0.01;
    const double cost_threshold = 0.05*0.05;
    const int num_feature = std::min<int>(idx_list.size(), 1000);

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
            Eigen::Vector3d P1 = pts1[i];
            Eigen::Vector3d _P1 = _dG.block(0,0,3,3)*P1 + _dG.block(0,3,3,1);

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
            Eigen::Vector3d xi = TF_to_se2(_dG);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // additional weight
            double dist = pts1[i].norm();
            double weight = 1.0 + 0.1*dist;
            //double weight = 1.0;

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
            return;
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
        xi[0] = X(0, 0)*0;
        xi[1] = X(1, 0)*0;
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

    //printf("[LIDAR] icp, i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);

    // check
    if(last_err > 0.1 || last_err > first_err)
    {
        return;
    }

    // update
    dG = _dG;
    return;
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
            Eigen::Matrix4d tf_f = reversed_Lidar(ZYX_to_TF(config->LIDAR_TF_F));
            Eigen::Matrix4d tf_b = reversed_Lidar(ZYX_to_TF(config->LIDAR_TF_B));

            if(min_idx >= 0)
            {
                RAW_FRAME frm1 = storage[min_idx];

                // lidar frame to robot frame
                std::vector<double> reflects_f;
                std::vector<Eigen::Vector3d> pts_f;
                for(size_t p = 0; p < frm0.dsk.size(); p++)
                {
                    Eigen::Vector3d P = tf_f.block(0,0,3,3)*frm0.dsk[p] + tf_f.block(0,3,3,1);

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
                for(size_t p = 0; p < frm1.dsk.size(); p++)
                {
                    Eigen::Vector3d P = tf_b.block(0,0,3,3)*frm1.dsk[p] + tf_b.block(0,3,3,1);

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
                //icp(pts_f, pts_b, dG);

                //Eigen::Vector3d dxi = TF_to_se2(dG);
                //printf("dxi:%f, %f, %f\n", dxi[0], dxi[1], dxi[2]*R2D);
                //align(pts_f, pts_b, dG);

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

                // update
                mtx.lock();
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
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    printf("[LIDAR] stop a loop\n");
}

