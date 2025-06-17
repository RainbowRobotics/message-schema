#include "sick.h"

SICK::SICK(QObject *parent) : QObject{parent}
{
}

SICK::~SICK()
{
    for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
    {
        is_connected[idx] = false;
    }

    close();
}

void SICK::init()
{
    for(int i = 0; i < config->LIDAR_2D_NUM; i++)
    {
        pts_tf[i] = string_to_TF(config->LIDAR_2D_TF[i]);
    }

    printf("[LIVOX] init\n");
}

void SICK::open()
{
    printf("[SICK] open\n");

    // stop first
    close();

    // loop start
    for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
    {
        if(grab_thread[idx] == nullptr)
        {
            grab_flag[idx] = true;
            grab_thread[idx] = new std::thread(&SICK::grab_loop, this, idx);
        }
    }
}

void SICK::close()
{
    for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
    {
        is_connected[idx] = false;
        if(grab_thread[idx] != nullptr)
        {
            grab_flag[idx] = false;
            grab_thread[idx]->join();
            grab_thread[idx] = nullptr;
        }
    }
}

QString SICK::get_info_text(int idx)
{
    QString res;
    res += QString().sprintf("[SICK %d]\npts_t: %.3f (%d)\n", idx, cur_raw_t[idx].load(), cur_pts_num[idx].load());
    res += QString().sprintf("fq: %d,", (int)raw_que[idx].unsafe_size());

    return res;
}

void SICK::sync(int idx)
{
    is_sync[idx] = true;
    printf("[SICK] front time sync\n");
}

void SICK::grab_loop(int idx)
{
    /* set host ip */
    std::string host_ip_str = "192.168.2.2"; // mobile (pdu) ip
    sick::datastructure::CommSettings comm_settings;
    comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str);
    comm_settings.host_udp_port = 0;
    comm_settings.e_interface_type = 4;
    comm_settings.publishing_frequency = 2; // 1:25 hz, 2:12.5 hz;

    /* set sensor ip & port */
    std::string ip_str = config->LIDAR_2D_IP[idx].toStdString();
    sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(ip_str);
    sick::types::port_t tcp_port{2122};

    std::unique_ptr<sick::SyncSickSafetyScanner> safety_scanner;
    try
    {
        safety_scanner = std::make_unique<sick::SyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings);
    }
    catch(...)
    {
        is_connected[idx] = false;
        printf("[SICK] failed to connect lidar %d\n", idx);
        return;
    }

    is_connected[idx] = true;
    printf("[SICK] start grab loop, %d\n", idx);
    while(grab_flag[idx])
    {
        if(safety_scanner->isDataAvailable())
        {
            auto timeout = boost::posix_time::seconds(1);
            sick::datastructure::Data data = safety_scanner->receive(timeout);

            // check mobile pose
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // time sync
            double pc_t = get_time();
            double lidar_t = data.getDataHeaderPtr()->getTimestampTime()*M2S;

            if(is_sync[idx])
            {
                is_sync[idx] = false;
                offset_t[idx] = pc_t - lidar_t;

                is_synced[idx] = true;
                QString str = QString("[SICK] sync. lidar :%1, lidar_t_f: %2, offset_t_f: %3").arg(idx).arg(lidar_t).arg((double)offset_t[idx]);
                printf("%s\n", str.toLocal8Bit().data());
            }

            // check lidar, mobile sync
            if(!is_synced[idx].load() || !mobile->is_synced.load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // lidar frame synced ref time
            std::vector<sick::ScanPoint> sp = data.getMeasurementDataPtr()->getScanPointsVector();
            int num_scan_point = sp.size();
            double scan_time = data.getDerivedValuesPtr()->getScanTime()*M2S;
            double time_increment = scan_time/num_scan_point;

            double t0 = lidar_t + offset_t[idx];
            double t1 = t0 + scan_time;

            // wait until t1 <= mobile->last_pose_t
            while(t1 > mobile->last_pose_t.load() && grab_flag[idx].load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

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
            for(int p = 0; p < (int)pose_storage.size(); p++)
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
                printf("[SICK] lidar: %d, invalid mobile poses\n", idx);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> pts;

            // parsing            
            for(size_t p = 0; p < sp.size(); p++)
            {
                // check sick invaild pts
                bool is_infinite = sp[p].getInfiniteBit();
                bool is_glare = sp[p].getGlareBit();
                bool is_reflect = sp[p].getReflectorBit();
                bool is_contamination = (sp[p].getContaminationBit() || sp[p].getContaminationWarningBit());
                if(is_infinite || is_glare || is_reflect || is_contamination)
                {
                    continue;
                }

                // dist filter
                double dist = (double)sp[p].getDistance()/1000.0; // mm to meter
                if(dist < config->LIDAR_2D_MIN_RANGE || dist > config->LIDAR_2D_MAX_RANGE)
                {
                    continue;
                }

                // angle filter
                double deg = sp[p].getAngle();
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

                Eigen::Vector3d _P = Eigen::Vector3d(x, y, 0);
                Eigen::Vector3d P = pts_tf[idx].block(0,0,3,3)*_P + pts_tf[idx].block(0,3,3,1);
                if(P[0] > config->ROBOT_SIZE_X[0] && P[0] < config->ROBOT_SIZE_X[1] &&
                   P[1] > config->ROBOT_SIZE_Y[0] && P[1] < config->ROBOT_SIZE_Y[1])
                {
                    continue;
                }

                double t = t0 + time_increment*p;
                double rssi = sp[p].getReflectivity();

                times.push_back(t);
                reflects.push_back(rssi);
                pts.push_back(P);
            }

            cur_pts_num[idx] = (int)pts.size();
            cur_raw_t[idx] = t0;

            MOBILE_POSE mo;
            mo.t = t0;
            mo.pose = pose_storage[idx0].pose;
            //printf("[LIDAR] front lidar t:%f, pose:%.3f, %.3f, %.3f\n", mo.t, mo.pose[0], mo.pose[1], mo.pose[2]*R2D);

            RAW_FRAME frm;
            frm.t0 = t0;
            frm.t1 = t1;
            frm.pose0 = pose_storage[idx0].pose;
            frm.pose1 = pose_storage[idx1].pose;
            frm.times = times;
            frm.reflects = reflects;
            frm.pts = pts;
            frm.mo = mo;
            raw_que[idx].push(frm);

            mtx.lock();
            cur_raw[idx] = frm;
            mtx.unlock();

            // que overflow control
            if(raw_que[idx].unsafe_size() > 50)
            {
                RAW_FRAME tmp;
                raw_que[idx].try_pop(tmp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[SICK] stop grab loop: %d\n", idx);
}
