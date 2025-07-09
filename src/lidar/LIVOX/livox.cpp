#include "livox.h"

LIVOX* LIVOX::instance(QObject* parent)
{
    static LIVOX* inst = nullptr;
    if(!inst && parent)
    {
        inst = new LIVOX(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

LIVOX::LIVOX(QObject *parent) : QObject{parent}
{

}

LIVOX::~LIVOX()
{
    is_connected = false;

    close();
}

void LIVOX::open()
{
    printf("[LIVOX] open\n");

    // stop first
    close();

    for(int i = 0; i < config->get_lidar_3d_num(); i++)
    {
        QString tf_str = config->get_lidar_3d_tf(i);
        pts_tf[i] = string_to_TF(tf_str);
        pts_R[i] = pts_tf[i].block(0,0,3,3);
        pts_t[i] = pts_tf[i].block(0,3,3,1);
        // imu_tf[i] = string_to_TF(config->LIDAR_IMU_TF[i]);
        imu_tf[i] = string_to_TF(tf_str);
        imu_R[i] = imu_tf[i].block(0,0,3,3);
        imu_t[i] = imu_tf[i].block(0,3,3,1);
    }

    printf("[LIVOX] init\n");

    // loop start
    grab_flag = true;
    grab_thread = std::make_unique<std::thread>(&LIVOX::grab_loop, this);
}

void LIVOX::close()
{
    is_connected = false;

    grab_flag = false;
    if(grab_thread && grab_thread->joinable())
    {
        grab_thread->join();
    }
    grab_thread.reset();
}

QString LIVOX::get_info_text(int idx)
{
    QString res;

    IMU imu;
    {
        std::shared_lock<std::shared_mutex> lock(lidar_mtx);
        imu = cur_imu[idx];
    }

    res += QString().sprintf("[LIDAR %d]\nimu_t: %.3f, pts_t: %.3f (%d)\n", idx, cur_imu_t[idx].load(), cur_frm_t[idx].load(), cur_pts_num[idx].load());
    res += QString().sprintf("acc: %.2f, %.2f, %.2f\n", imu.acc_x, imu.acc_y, imu.acc_z);
    res += QString().sprintf("gyr: %.2f, %.2f, %.2f\n", imu.gyr_x * R2D, imu.gyr_y * R2D, imu.gyr_z * R2D);
    res += QString().sprintf("so3: %.1f, %.1f, %.1f\n", imu.rx * R2D, imu.ry * R2D, imu.rz * R2D);
    res += QString().sprintf("time_sync_type: %d\n fq: %d,", time_type[idx].load(), (int)frm_que[idx].unsafe_size());

    return res;
}

LVX_FRM LIVOX::get_cur_raw(int idx)
{
    std::shared_lock<std::shared_mutex> lock(lidar_mtx);
    LVX_FRM res = cur_raw[idx];
    return res;
}

IMU LIVOX::get_cur_imu(int idx)
{
    std::shared_lock<std::shared_mutex> lock(imu_mtx);
    IMU res = cur_imu[idx];
    return res;
}

std::vector<IMU> LIVOX::get_imu_storage(int idx)
{
    std::shared_lock<std::shared_mutex> lock(imu_mtx);
    std::vector<IMU> res = imu_storage[idx];
    return res;
}

int LIVOX::get_livox_idx(uint32_t handle)
{
    for(size_t i = 0; i < livox_handles.size(); i++)
    {
        if(livox_handles[i] == handle)
        {
            return i;
        }
    }
    return -1;
}

bool LIVOX::get_is_connected(int idx)
{
    return true;//(bool)is_connected[idx].load();
}

void LIVOX::set_is_connected(int idx, bool val)
{
    //is_connected[idx].store(val);
}

bool LIVOX::get_is_sync(int idx)
{
    return (bool)is_sync[idx].load();
}

void LIVOX::set_is_sync(int idx, bool val)
{
    is_sync[idx].store(val);
}

int LIVOX::get_time_type(int idx)
{
    return (int)time_type[idx].load();
}

bool LIVOX::try_pop_frm_que(int idx, LVX_FRM& frm)
{
    if(frm_que[idx].try_pop(frm))
    {
        return true;
    }
    return false;
}

void LIVOX::grab_loop()
{
    // Disable logger
    DisableLivoxSdkConsoleLogger();
    printf("[LIVOX] Disable debug message\n");

    // Init Livox SDK2
    QString path = QCoreApplication::applicationDirPath() + "/configs/" + config->get_platform_name() + "/mid360_config.json";
    printf("[LIVOX] load, %s\n", path.toLocal8Bit().data());

    FILE* fp = fopen(path.toLocal8Bit().data(), "r");
    if(fp == nullptr)
    {
        printf("[LIVOX] config file not found or unreadable: %s\n", path.toLocal8Bit().data());
        return;
    }
    fclose(fp);

    if(!LivoxLidarSdkInit(path.toLocal8Bit().data()))
    {
        printf("[LIVOX] livox Init Failed\n");
        LivoxLidarSdkUninit();
        return;
    }

    printf("[LIVOX] livox initialized\n");

    // Register handle callback
    SetLivoxLidarInfoChangeCallback([](const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
    {
        LIVOX* livox = static_cast<LIVOX*>(client_data);
        if(livox && info)
        {
            std::string ip(info->lidar_ip);
            int idx = -1;

            if(ip == livox->config->get_lidar_3d_ip(0).toStdString())
            {
                idx = 0;
            }
            else if(ip == livox->config->get_lidar_3d_ip(1).toStdString())
            {
                idx = 1;
            }

            if(idx >= 0)
            {
                livox->livox_handles[idx] = handle;
                printf("[LIVOX] handle registered [%d]: %u, ip: %s, sn: %s\n", idx, handle, info->lidar_ip, info->sn);

                SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
                livox->is_connected = true;
            }
            else
            {
                printf("[LIVOX] unknown IP: %s (sn: %s)\n", info->lidar_ip, info->sn);
            }
        }
    }, this);


    // set callbacks
    auto point_cloud_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LIVOX* livox = static_cast<LIVOX*>(client_data);
        if(livox == nullptr || data == nullptr)
        {
            return;
        }

        // get livox index
        int idx = livox->get_livox_idx(handle);
        if(idx < 0 || !livox->is_connected)
        {
            // printf("[LIDAR] handle: %u -> idx: %d\n", handle, idx);
            return;
        }

        // get timestamp type
        livox->time_type[idx] = data->time_type;

        // parsing metadata
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec
        uint16_t time_interval = data->time_interval/10; // 0.1us -> nanosec
        // double livox_time = time_base * N2S;
        // double pc_time = get_time()+st_time_for_get_time; // PC 시간
        // printf("[PTP-CHECK] livox_time: %.9f, pc_time: %.9f, diff: %.6f sec\n", livox_time, pc_time, livox_time - pc_time);

        // parsing point cloud
        if(data->data_type == kLivoxLidarCartesianCoordinateHighData)
        {
            LivoxLidarCartesianHighRawPoint *p_point_data = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data);
            if(livox->offset_t[idx] == 0)
            {
                return;
            }

            std::vector<LVX_PT> pts;
            for(uint32_t i = 0; i < data->dot_num; i++)
            {
                // filtering
                if(p_point_data[i].reflectivity == 0)
                {
                    continue;
                }

                // other, energy, spatial all 0
                uint8_t tag = p_point_data[i].tag;
                if((tag & 0b00111111) != 0)
                {
                    continue;
                }

                Eigen::Vector3d P;
                P[0] = p_point_data[i].x/1000.0;
                P[1] = p_point_data[i].y/1000.0;
                P[2] = p_point_data[i].z/1000.0;

                Eigen::Vector3d _P = livox->pts_R[idx]*P + livox->pts_t[idx];
                double d = _P.norm();

                // double d = P.norm();
                if(d < livox->config->get_lidar_3d_min_range() || d > livox->config->get_lidar_3d_max_range())
                {
                    continue;
                }

                // self collision filter
                if(check_self_collision(_P(0), _P(1), _P(2),
                                        livox->config->get_robot_size_x_min(), livox->config->get_robot_size_x_max(),
                                        livox->config->get_robot_size_y_min(), livox->config->get_robot_size_y_max(),
                                        livox->config->get_robot_size_z_min(), livox->config->get_robot_size_z_max()))
                {
                    continue;
                }

                LVX_PT pt;
                pt.t = (time_base + i * time_interval)*N2S + livox->offset_t[idx]; // nanosec to sec
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                pt.reflect = p_point_data[i].reflectivity;
                pt.tag = p_point_data[i].tag;

                pts.push_back(pt);
            }

            // update
            livox->pts_storage[idx].insert(livox->pts_storage[idx].end(), pts.begin(), pts.end());
            if(livox->pts_storage[idx].size() >= 2 && (livox->pts_storage[idx].back().t - livox->pts_storage[idx].front().t) > livox->lvx_frm_dt)
            {
                // calc alpha
                const double t0 = livox->pts_storage[idx].front().t;
                const double t1 = livox->pts_storage[idx].back().t;

                for(size_t p = 0; p < livox->pts_storage[idx].size(); p++)
                {
                    double t = livox->pts_storage[idx][p].t;
                    double alpha = (t-t0)/(t1-t0);
                    livox->pts_storage[idx][p].alpha = alpha;
                }

                // make frame
                LVX_FRM frm;
                frm.t = t0;
                // frm.pts = std::move(livox->pts_storage[idx]);
                frm.pts = livox->pts_storage[idx];

                // set queue
                livox->frm_que[idx].push(frm);

                // for queue overflow
                if(livox->frm_que[idx].unsafe_size() > 10)
                {
                    LVX_FRM tmp;
                    livox->frm_que[idx].try_pop(tmp);
                }

                // set raw pts
                {
                    std::unique_lock<std::shared_mutex> lock(livox->lidar_mtx);
                    livox->cur_raw[idx] = frm;
                    livox->cur_frm_t[idx] = frm.t;
                    livox->cur_pts_num[idx] = frm.pts.size();
                }

                // clear
                livox->pts_storage[idx].clear();
                livox->pts_storage[idx].push_back(frm.pts.back());
            }
        }
    };

    auto imu_data_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LIVOX* livox = static_cast<LIVOX*>(client_data);
        if(livox == nullptr)
        {
            return;
        }

        if(data == nullptr)
        {
            return;
        }

        // get lidar index
        int idx = livox->get_livox_idx(handle);
        if(idx < 0 || !livox->is_connected)
        {
            return;
        }

        // parsing
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec

        // time sync
        if(livox->offset_t[idx] == 0 || livox->is_sync[idx])
        {
            livox->is_sync[idx] = false;
            livox->offset_t[idx] = get_time() - (time_base * N2S);
        }

        LivoxLidarImuRawPoint *p_point_data = reinterpret_cast<LivoxLidarImuRawPoint *>(data->data);

        Eigen::Vector3d acc_vec;
        acc_vec[0] = p_point_data->acc_x; // normalized gravity vector
        acc_vec[1] = p_point_data->acc_y;
        acc_vec[2] = p_point_data->acc_z;

        Eigen::Vector3d gyr_vec;
        gyr_vec[0] = p_point_data->gyro_x; // rad/s
        gyr_vec[1] = p_point_data->gyro_y; // rad/s
        gyr_vec[2] = p_point_data->gyro_z; // rad/s

        Eigen::Vector3d _acc_vec = livox->imu_tf[idx].block(0,0,3,3)*acc_vec;
        Eigen::Vector3d _gyr_vec = livox->imu_tf[idx].block(0,0,3,3)*gyr_vec;

        // considering centripetal acceleration
        Eigen::Vector3d imu_pos = livox->imu_tf[idx].block(0,3,3,1);
        _acc_vec = _acc_vec - _gyr_vec.cross(_gyr_vec.cross(imu_pos));

        IMU imu;
        // imu.t = time_base * N2S; //+ livox->offset_t[idx];
        imu.t = time_base * N2S + livox->offset_t[idx];
        imu.acc_x = _acc_vec[0]*ACC_G; // m/s^2
        imu.acc_y = _acc_vec[1]*ACC_G;
        imu.acc_z = _acc_vec[2]*ACC_G;
        imu.gyr_x = _gyr_vec[0]; // rad/s
        imu.gyr_y = _gyr_vec[1];
        imu.gyr_z = _gyr_vec[2];

        //printf("[LIDAR] imu received, t: %f\n", imu.t);

        // get orientation using complementary filter
        double q0, q1, q2, q3;
        livox->imu_filter[idx].update(_acc_vec[0], _acc_vec[1], _acc_vec[2], _gyr_vec[0], _gyr_vec[1], _gyr_vec[2], 0.005); // 200hz
        livox->imu_filter[idx].getOrientation(q0, q1, q2, q3);
        Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
        Eigen::Vector3d r = Sophus::SO3d::fitToSO3(R).log();

        imu.rx = r[0];
        imu.ry = r[1];
        imu.rz = r[2];

        // update storage
        if(livox->offset_t[idx] != 0)
        {
            std::unique_lock<std::shared_mutex> lock(livox->imu_mtx);
            {
                livox->cur_imu[idx] = imu;
                livox->cur_imu_t[idx] = imu.t;

                livox->imu_storage[idx].push_back(imu);
            }

            if(livox->imu_storage[idx].size() > 200)
            {
                livox->imu_storage[idx].erase(livox->imu_storage[idx].begin());
            }
        }
    };

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(point_cloud_callback, this); // client_data is unused
    SetLivoxLidarImuDataCallback(imu_data_callback, this); // client_data is unused
    printf("[LIVOX] callback registered\n");

    printf("[LIVOX] grab_loop start\n");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
    }

    // uninit
    LivoxLidarSdkUninit();

    printf("[LIVOX] grab_loop stop\n");
}

void LIVOX::set_config_module(CONFIG* _config)
{
    config = _config;
}

void LIVOX::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}
