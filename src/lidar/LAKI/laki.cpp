#include "laki.h"

namespace 
{
    const char* MODULE_NAME = "LAKI";
}

LAKI* LAKI::instance(QObject* parent)
{
    static LAKI* inst = nullptr;
    if(!inst && parent)
    {
        inst = new LAKI(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

LAKI::LAKI(QObject *parent) : QObject{parent}
{
}

LAKI::~LAKI()
{
    for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
    {
        is_connected[idx] = false;
    }

    close();
}

void LAKI::open()
{
    if (!config || !config->get_use_lidar_2d())
    {
        log_warn("disabled -> skip open");
        return;
    }

    log_info("open");

    // stop first
    close();

    for(int i = 0; i < config->get_lidar_2d_num(); i++)
    {
        pts_tf[i] = string_to_TF(config->get_lidar_2d_tf(i));
    }

    log_info("init");

    // loop start
    for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
    {
        if(grab_thread[idx] == nullptr)
        {
            grab_flag[idx] = true;
            grab_thread[idx] = make_unique<std::thread>(&LAKI::grab_loop, this, idx);
        }
    }
}

void LAKI::close()
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

QString LAKI::get_info_text(int idx)
{
    QString res;
    res += QString().sprintf("[LAKI %d]\npts_t: %.3f (%d)\n", idx, cur_raw_t[idx].load(), cur_pts_num[idx].load());
    res += QString().sprintf("fq: %d,", (int)raw_que[idx].unsafe_size());

    return res;
}

RAW_FRAME LAKI::get_cur_raw(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    RAW_FRAME res = cur_raw[idx];
    return res;
}

bool LAKI::get_is_connected(int idx)
{
    return (bool)is_connected[idx].load();
}

bool LAKI::get_is_sync(int idx)
{
    return (bool)is_sync[idx].load();
}

void LAKI::set_is_sync(int idx, bool val)
{
    is_sync[idx].store(val);
}

bool LAKI::try_pop_raw_que(int idx, RAW_FRAME& frm)
{
    if(raw_que[idx].try_pop(frm))
    {
        return true;
    }
    return false;
}

void LAKI::sync(int idx)
{
    is_sync[idx] = true;
    log_info("front time sync");
}

void LAKI::grab_loop(int idx)
{
    /* set local ip & port */
    std::string local_ip = "192.168.2.2"; // mobile (pdu) ip
    // Use different local ports for each lidar to avoid port conflicts
    std::string local_port = std::to_string(2368 + idx);

    /* set sensor ip & port */
    std::string sensor_ip_str = config->get_lidar_2d_ip(idx).toStdString();
    std::string sensor_port = "2368";  // sensor always sends to 2368
    std::string http_port = "80";

    try
    {
        laki_http[idx] = std::make_unique<LakiBeamHTTP>(local_ip, local_port, sensor_ip_str, http_port);
        laki_udp[idx] = std::make_unique<LakiBeamUDP>(local_ip, local_port, sensor_ip_str, sensor_port);
        
        // Configure LAKI lidar to send data to the correct port
        std::string result;
        std::string target_port = std::to_string(2368 + idx);
        if(!laki_http[idx]->put_host_port(target_port))
        {
            log_warn("Failed to set LAKI lidar {} host port to {}", idx, target_port);
        }
        else
        {
            log_info("LAKI lidar {} configured to send to port {}", idx, target_port);
        }
    }
    catch(...)
    {
        is_connected[idx] = false;
        log_error("failed to connect lidar {}", idx);
        return;
    }

    Eigen::Matrix3d R_ = pts_tf[idx].block(0,0,3,3);
    Eigen::Vector3d t_ = pts_tf[idx].block(0,3,3,1);

    is_connected[idx] = true;
    printf("[LAKI] start grab loop, %d\n", idx);

    while(grab_flag[idx])
    {
        repark_t pack;
        if(!laki_udp[idx]->get_repackedpack(pack))
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // check mobile pose
        if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // time sync
        double pc_t = get_time();
        // LAKI timestamp is in microseconds (NOT milliseconds like SICK)
        double lidar_t = pack.dotcloud[0].timestamp * 1e-6; // microseconds to seconds

        if(is_sync[idx])
        {
            is_sync[idx] = false;
            offset_t[idx] = pc_t - lidar_t;

            is_synced[idx] = true;
            log_info("sync. lidar :{}, lidar_t_f: {}, offset_t_f: {}", idx, lidar_t, (double)offset_t[idx]);
        }

        // check lidar, mobile sync
        if(!is_synced[idx].load() || !mobile->get_is_synced())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // lidar frame synced ref time
        int num_scan_point = pack.maxdots;
        if(num_scan_point <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Calculate scan time from interval
        double scan_time = pack.interval * 1e-3; // interval is in milliseconds
        double time_increment = scan_time / num_scan_point;

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

            if(idx0 == -1)
            {
                idx0 = 0;
            }
            if(idx1 == -1)
            {
                idx1 = (int)pose_storage.size() - 1;  // use last pose
            }
            if(idx0 == idx1 && idx0 < (int)pose_storage.size() - 1)
            {
                idx1 = idx0 + 1;  // use next pose
            }
        }


        // get lidar raw data
        std::vector<double> times;
        std::vector<double> reflects;
        std::vector<Eigen::Vector3d> pts;

        double x_min = config->get_robot_size_x_min(); double x_max = config->get_robot_size_x_max();
        double y_min = config->get_robot_size_y_min(); double y_max = config->get_robot_size_y_max();

        // parsing LAKI point cloud data
        for(int p = 0; p < num_scan_point; p++)
        {
            // dist filter (LAKI distance is in mm)
            double dist = pack.dotcloud[p].distance * 0.001; // mm to meter
            if(dist < config->get_lidar_2d_min_range() || dist > config->get_lidar_2d_max_range())
            {
                continue;
            }

            // angle filter (LAKI angle is in 0.01 degree units, 0-36000)
            double deg = pack.dotcloud[p].angle * 0.01; // convert to degrees
            
            // LAKI coordinate system: 0 degree is forward, increases counter-clockwise
            // Convert to standard coordinate: -180 to 180 range
            if(deg > 180.0)
            {
                deg -= 360.0;
            }

            double x = dist * std::cos(deg * D2R);
            double y = dist * std::sin(deg * D2R);
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

            double t = t0 + time_increment * p;
            double rssi = pack.dotcloud[p].rssi;

            times.push_back(t);
            reflects.push_back(rssi);
            pts.push_back(P);
        }

        cur_pts_num[idx] = (int)pts.size();
        cur_raw_t[idx] = t0;

        MOBILE_POSE mo;
        mo.t = t0;
        mo.pose = pose_storage[idx0].pose;
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

    log_info("stop grab loop: {}", idx);
}

void LAKI::set_config_module(CONFIG* _config)
{
    config = _config;
}

void LAKI::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void LAKI::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}
