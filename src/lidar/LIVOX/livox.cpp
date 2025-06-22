#include "livox.h"

LIVOX::LIVOX(QObject *parent) : QObject{parent}
{

}

LIVOX::~LIVOX()
{
    is_connected = false;

    close();
}

void LIVOX::init()
{
    for(int i = 0; i < config->LIDAR_3D_NUM; i++)
    {
        pts_tf[i] = string_to_TF(config->LIDAR_3D_TF[i]);
        // imu_tf[i] = string_to_TF(config->LIDAR_IMU_TF[i]);
        imu_tf[i] = string_to_TF(config->LIDAR_3D_TF[i]);
    }

    printf("[LIVOX] init\n");
}

void LIVOX::open()
{
    printf("[LIVOX] open\n");

    // stop first
    close();

    // loop start
    if(grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&LIVOX::grab_loop, this);
    }
}

void LIVOX::close()
{
    is_connected = false;

    if(grab_thread != NULL)
    {
        grab_flag = false;
        grab_thread->join();
        grab_thread = NULL;
    }
}

QString LIVOX::get_info_text(int idx)
{
    QString res;

    mtx.lock();
    IMU imu =  cur_imu[idx];
    mtx.unlock();

    res += QString().sprintf("[LIDAR %d]\nimu_t: %.3f, pts_t: %.3f (%d)\n", idx, cur_imu_t[idx].load(), cur_frm_t[idx].load(), cur_pts_num[idx].load());
    res += QString().sprintf("acc: %.2f, %.2f, %.2f\n", imu.acc_x, imu.acc_y, imu.acc_z);
    res += QString().sprintf("gyr: %.2f, %.2f, %.2f\n", imu.gyr_x * R2D, imu.gyr_y * R2D, imu.gyr_z * R2D);
    res += QString().sprintf("so3: %.1f, %.1f, %.1f\n", imu.rx * R2D, imu.ry * R2D, imu.rz * R2D);
    res += QString().sprintf("time_sync_type: %d\n fq: %d,", time_type[idx].load(), (int)frm_que[idx].unsafe_size());

    return res;
}

LVX_FRM LIVOX::get_cur_raw(int idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    LVX_FRM res = cur_raw[idx];
    return res;
}

IMU LIVOX::get_cur_imu(int idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    IMU res = cur_imu[idx];
    return res;
}

std::vector<IMU> LIVOX::get_imu_storage(int idx)
{
    std::lock_guard<std::mutex> lock(mtx);
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

void LIVOX::grab_loop()
{
    // Disable logger
    DisableLivoxSdkConsoleLogger();
    printf("[LIVOX] Disable debug message\n");

    // Init Livox SDK2
    QString path = QCoreApplication::applicationDirPath() + "/configs/" + config->PLATFORM_NAME + "/mid360_config.json";
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

            if(ip == livox->config->LIDAR_3D_IP[0].toStdString())
            {
                idx = 0;
            }
            else if(ip == livox->config->LIDAR_3D_IP[1].toStdString())
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

                Eigen::Vector3d _P = livox->pts_tf[idx].block(0,0,3,3)*P + livox->pts_tf[idx].block(0,3,3,1);
                double d = _P.norm();

                // double d = P.norm();
                if(d < livox->config->LIDAR_3D_MIN_RANGE || d > livox->config->LIDAR_3D_MAX_RANGE)
                {
                    continue;
                }

                // self collision filter
                // if(check_self_collision(_P(0), _P(1), _P(2),
                //                         livox->config->ROBOT_SIZE_X[0], livox->config->ROBOT_SIZE_X[1],
                //                         livox->config->ROBOT_SIZE_Y[0], livox->config->ROBOT_SIZE_Y[1],
                //                         livox->config->ROBOT_SIZE_Z[0], livox->config->ROBOT_SIZE_Z[1]))
                // {
                //     continue;
                // }

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
                livox->mtx.lock();
                livox->cur_raw[idx] = frm;
                livox->cur_frm_t[idx] = frm.t;
                livox->cur_pts_num[idx] = frm.pts.size();
                livox->mtx.unlock();

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
        livox->mtx.lock();

        if(livox->offset_t[idx] != 0)
        {
            livox->cur_imu[idx] = imu;
            livox->cur_imu_t[idx] = imu.t;

            livox->imu_storage[idx].push_back(imu);
            // if(livox->imu_storage.front().t < imu.t - 3.0)
            // {
            //     livox->imu_storage.erase(livox->imu_storage.begin());
            // }

            if(livox->imu_storage[idx].size() > 200)
            {
                livox->imu_storage[idx].erase(livox->imu_storage[idx].begin());
            }
        }

        livox->mtx.unlock();
    };

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(point_cloud_callback, this); // client_data is unused
    SetLivoxLidarImuDataCallback(imu_data_callback, this); // client_data is unused
    printf("[LIVOX] callback registered\n");


    printf("[LIVOX] grab_loop start\n");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // uninit
    LivoxLidarSdkUninit();

    printf("[LIVOX] grab_loop stop\n");
}
