#include "sick.h"

namespace 
{
    const char* MODULE_NAME = "SICK";
}

SICK* SICK::instance(QObject* parent)
{
    static SICK* inst = nullptr;
    if(!inst && parent)
    {
        inst = new SICK(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

SICK::SICK(QObject *parent) : QObject{parent}
{
}

SICK::~SICK()
{
    for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
    {
        is_connected[idx] = false;
    }

    close();
}

void SICK::open()
{
    if (!config || !config->get_use_lidar_2d())
    {
        //printf("[SICK] disabled -> skip open\n");
        log_warn("disabled -> skip open");
        return;
    }

    //printf("[SICK] open\n");
    //spdlog::info("[SICK] open");
    log_info("open");

    // stop first
    close();

    for(int i = 0; i < config->get_lidar_2d_num(); i++)
    {
        pts_tf[i] = string_to_TF(config->get_lidar_2d_tf(i));
    }

    //printf("[LIVOX] init\n");
    //spdlog::info("[SICK] init");
    log_info("init");
    

    // loop start
    for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
    {
        if(grab_thread[idx] == nullptr)
        {
            grab_flag[idx] = true;
            grab_thread[idx] = make_unique<std::thread>(&SICK::grab_loop, this, idx);
        }
    }
}

void SICK::close()
{
    log_info("close");
    for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
    {
        is_connected[idx] = false;

        grab_flag[idx] = false;
        if(grab_thread[idx] && grab_thread[idx]->joinable())
        {
            grab_thread[idx]->join();
        }
        grab_thread[idx].reset();
    }
}

QString SICK::get_info_text(int idx)
{
    QString res;
    res += QString().sprintf("[SICK %d]\npts_t: %.3f (%d)\n", idx, cur_raw_t[idx].load(), cur_pts_num[idx].load());
    res += QString().sprintf("fq: %d,", (int)raw_que[idx].unsafe_size());

    return res;
}

RAW_FRAME SICK::get_cur_raw(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    RAW_FRAME res = cur_raw[idx];
    return res;
}

bool SICK::get_is_connected(int idx)
{
    return (bool)is_connected[idx].load();
}

bool SICK::get_is_sync(int idx)
{
    return (bool)is_sync[idx].load();
}

void SICK::set_is_sync(int idx, bool val)
{
    is_sync[idx].store(val);
}

bool SICK::try_pop_raw_que(int idx, RAW_FRAME& frm)
{
    if(raw_que[idx].try_pop(frm))
    {
        return true;
    }
    return false;
}

void SICK::sync(int idx)
{
    is_sync[idx] = true;
    //printf("[SICK] front time sync\n");
    log_info("front time sync");
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
    std::string ip_str = config->get_lidar_2d_ip(idx).toStdString();
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
        //printf("[SICK] failed to connect lidar %d\n", idx);
        log_error("failed to connect lidar {}", idx);
        return;
    }

    Eigen::Matrix3d R_ = pts_tf[idx].block(0,0,3,3);
    Eigen::Vector3d t_ = pts_tf[idx].block(0,3,3,1);

    is_connected[idx] = true;
    //printf("[SICK] start grab loop, %d\n", idx);
    log_info("start grab loop, {}", idx);
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
            log_debug("lidar: {}, pc_t: {}, lidar_t: {}", idx, pc_t, lidar_t);

            if(is_sync[idx])
            {
                is_sync[idx] = false;
                offset_t[idx] = pc_t - lidar_t;

                is_synced[idx] = true;
                QString str = QString("[SICK] sync. lidar :%1, lidar_t_f: %2, offset_t_f: %3").arg(idx).arg(lidar_t).arg((double)offset_t[idx]);
                //printf("%s\n", str.toLocal8Bit().data());
                log_info("sync. lidar :{}, lidar_t_f: {}, offset_t_f: {}", idx, lidar_t, (double)offset_t[idx]);
            }

            // check lidar, mobile sync
            if(!is_synced[idx].load() || !mobile->get_is_synced())
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
            while(t1 > mobile->get_last_pose_t() && grab_flag[idx].load())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get boundary mobile pose
            int idx0 = -1;
            for(int p = (int)pose_storage.size()-1; p >= 0; p--)
            {
//                qDebug()<<"pose_storage[p].t : "<<pose_storage[p].t;
//                qDebug()<<"t0 : "<<t0;
                if(pose_storage[p].t < t0)
                {
                    idx0 = p;
                    break;
                }
            }

            int idx1 = -1;
            for(int p = 0; p < (int)pose_storage.size(); p++)
            {
//                qDebug()<<"pose_storage[p].t : "<<pose_storage[p].t;
//                qDebug()<<"t1 : "<<t1;
                if(pose_storage[p].t > t1)
                {
                    idx1 = p;
                    break;
                }
            }
//            qDebug()<<"idx0 : "<<idx0<<", idx1 :"<<idx1;

            // check
            if(idx0 == -1 || idx1 == -1 || idx0 == idx1)
            {
                // drop
                //printf("[SICK] lidar: %d, invalid mobile poses\n", idx);
                log_warn("lidar: {}, invalid mobile poses, pose_storage.size(): {}", idx, pose_storage.size());
                if (!pose_storage.empty()) 
                {
                    log_warn("  pose_storage time range: [{}, {}]", pose_storage.front().t, pose_storage.back().t);
                } 
                else 
                {
                    log_warn("  pose_storage is empty");
                }
                log_warn("  t0: {}, t1: {}, idx0: {}, idx1: {}", t0, t1, idx0, idx1);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // get lidar raw data
            std::vector<double> times;
            std::vector<double> reflects;
            std::vector<Eigen::Vector3d> pts;

            double x_min = config->get_robot_size_x_min(); double x_max = config->get_robot_size_x_max();
            double y_min = config->get_robot_size_y_min(); double y_max = config->get_robot_size_y_max();

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
                if(dist < config->get_lidar_2d_min_range() || dist > config->get_lidar_2d_max_range())
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
                Eigen::Vector3d P = R_ * _P + t_;
                if(P[0] > x_min && P[0] < x_max &&
                   P[1] > y_min && P[1] < y_max)
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
            log_debug("front lidar t:{}, pose:{:.3f}, {:.3f}, {:.3f}", mo.t, mo.pose[0], mo.pose[1], mo.pose[2]*R2D);

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

            {
                std::unique_lock<std::shared_mutex> lock(mtx);
                cur_raw[idx] = frm;
            }

            // que overflow control
            if(raw_que[idx].unsafe_size() > 50)
            {
                RAW_FRAME tmp;
                raw_que[idx].try_pop(tmp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    //printf("[SICK] stop grab loop: %d\n", idx);
    log_info("stop grab loop: {}", idx);
}

void SICK::set_config_module(CONFIG* _config)
{
    config = _config;
}

void SICK::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void SICK::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}
