#include "lidar_3d.h"

LIDAR_3D::LIDAR_3D(QObject *parent) : QObject(parent)
{
    for(int i = 0; i < config->LIDAR_3D_NUM; i++)
    {
        pts_tf[i].setIdentity();
        imu_tf[i].setIdentity();
    }
}

LIDAR_3D::~LIDAR_3D()
{
    is_connected = false;

    close();
}

void LIDAR_3D::init()
{
    for(int i = 0; i < config->LIDAR_3D_NUM; i++)
    {
        pts_tf[i] = string_to_TF(config->LIDAR_3D_TF[i]);
        // imu_tf[i] = string_to_TF(config->LIDAR_IMU_TF[i]);
        imu_tf[i] = string_to_TF(config->LIDAR_3D_TF[i]);
    }

    printf("[LIDAR] init\n");
}

void LIDAR_3D::open()
{
    printf("[LIDAR] open\n");

    // stop first
    close();

    // loop start
    if(grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&LIDAR_3D::grab_loop, this);
    }

    // dsk loop start
    for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
    {
        if(dsk_thread[idx] == NULL)
        {
            dsk_flag[idx] = true;
            dsk_thread[idx] = new std::thread(&LIDAR_3D::dsk_loop, this, idx);
        }
    }

    // a loop start
    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LIDAR_3D::a_loop, this);
    }
}

void LIDAR_3D::close()
{
    is_connected = false;

    if(grab_thread != NULL)
    {
        grab_flag = false;
        grab_thread->join();
        grab_thread = NULL;
    }

    for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
    {
        if(dsk_thread[idx] != NULL)
        {
            dsk_flag[idx] = false;
            dsk_thread[idx]->join();
            dsk_thread[idx] = NULL;
        }
    }

    if(a_thread != NULL)
    {
        a_flag = false;
        a_thread->join();
        a_thread = NULL;
    }
}

IMU LIDAR_3D::get_cur_imu(int idx)
{
    mtx.lock();
    IMU res = cur_imu[idx];
    mtx.unlock();

    return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t)
{
    mtx.lock();
    double min_dt = 99999999;
    IMU res;
    for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
    {
        for(size_t p = 0; p < imu_storage[idx].size(); p++)
        {
            double dt = std::abs(imu_storage[idx][p].t - ref_t);
            if(dt < min_dt)
            {
                min_dt = dt;
                res = imu_storage[idx][p];
            }
        }
    }
    mtx.unlock();

    return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t, int idx)
{
    mtx.lock();
    double min_dt = 99999999;
    IMU res = imu_storage[idx].back();
    for(size_t p = 0; p < imu_storage[idx].size(); p++)
    {
        double dt = std::abs(imu_storage[idx][p].t - ref_t);
        if(dt < min_dt)
        {
            min_dt = dt;
            res = imu_storage[idx][p];
        }
    }
    mtx.unlock();

    return res;
}

LVX_FRM LIDAR_3D::get_cur_frm(int idx)
{
    mtx.lock();
    LVX_FRM res = cur_frm[idx];
    mtx.unlock();

    return res;
}

QString LIDAR_3D::get_cur_state()
{
    mtx.lock();
    QString res = cur_state;
    mtx.unlock();

    return res;
}

void LIDAR_3D::set_cur_state(QString str)
{
    mtx.lock();
    cur_state = str;
    mtx.unlock();
}

QString LIDAR_3D::get_info_text()
{
    IMU imu0 = get_cur_imu(0);
    IMU imu1 = get_cur_imu(1);

    QString res;
    res += QString().sprintf("[LIDAR 0]\nimu_t: %.3f, pts_t: %.3f (%d)\nacc: %.2f, %.2f, %.2f\ngyr: %.2f, %.2f, %.2f\nso3: %.1f, %.1f, %.1f\n",
                             cur_imu_t[0].load(), cur_frm_t[0].load(), cur_pts_num[0].load(),
                             imu0.acc_x, imu0.acc_y, imu0.acc_z,
                             imu0.gyr_x * R2D, imu0.gyr_y * R2D, imu0.gyr_z * R2D,
                             imu0.rx * R2D, imu0.ry * R2D, imu0.rz * R2D);

    res += QString().sprintf("[LIDAR 1]\nimu_t: %.3f, pts_t: %.3f (%d)\nacc: %.2f, %.2f, %.2f\ngyr: %.2f, %.2f, %.2f\nso3: %.1f, %.1f, %.1f\n",
                             cur_imu_t[1].load(), cur_frm_t[1].load(), cur_pts_num[1].load(),
                             imu1.acc_x, imu1.acc_y, imu1.acc_z,
                             imu1.gyr_x * R2D, imu1.gyr_y * R2D, imu1.gyr_z * R2D,
                             imu1.rx * R2D, imu1.ry * R2D, imu1.rz * R2D);

    res += QString().sprintf("fq: (%d, %d), dq: (%d, %d)\nmq: %d ,pts_t: %.3f (%d)\ntime_sync_type: %d, %d\n",
                             (int)frm_que[0].unsafe_size(), (int)frm_que[1].unsafe_size(),
                             (int)dsk_que[0].unsafe_size(), (int)dsk_que[1].unsafe_size(),
                             (int)merged_que.unsafe_size(), cur_merged_frm_t.load(), cur_merged_num.load(),
                             time_type[0].load(), time_type[1].load());

    return res;
}

int LIDAR_3D::get_lidar_idx(uint32_t handle)
{
    for(size_t i = 0; i < lidar_handles.size(); i++)
    {
        if(lidar_handles[i] == handle)
        {
            return i;
        }
    }
    return -1;
}

void LIDAR_3D::grab_loop()
{
    // Disable logger
    DisableLivoxSdkConsoleLogger();
    printf("[LIDAR] Disable debug message\n");

    // Init Livox SDK2
    QString path = QDir::currentPath() + "/../configs/slamnav_3d/mid360_config.json";
    printf("[LIDAR] load, %s\n", path.toLocal8Bit().data());

    FILE* fp = fopen(path.toLocal8Bit().data(), "r");
    if(fp == nullptr)
    {
        printf("[LIDAR] config file not found or unreadable: %s\n", path.toLocal8Bit().data());
        return;
    }
    fclose(fp);

    if(!LivoxLidarSdkInit(path.toLocal8Bit().data()))
    {
        printf("[LIDAR] livox Init Failed\n");
        LivoxLidarSdkUninit();
        return;
    }

    printf("[LIDAR] livox initialized\n");

    // Register handle callback
    SetLivoxLidarInfoChangeCallback([](const uint32_t handle, const LivoxLidarInfo* info, void* client_data)
    {
        LIDAR_3D* lidar = static_cast<LIDAR_3D*>(client_data);

        if(lidar && info)
        {
            std::string ip(info->lidar_ip);
            int idx = -1;

            if(ip == "192.168.1.101")
            {
                idx = 0;
            }
            else if(ip == "192.168.1.102")
            {
                idx = 1;
            }

            if(idx >= 0)
            {
                lidar->lidar_handles[idx] = handle;
                printf("[LIDAR] handle registered [%d]: %u, ip: %s, sn: %s\n", idx, handle, info->lidar_ip, info->sn);

                SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
                lidar->is_connected = true;
            }
            else
            {
                printf("[LIDAR] unknown IP: %s (sn: %s)\n", info->lidar_ip, info->sn);
            }
        }
    }, this);


    // set callbacks
    auto point_cloud_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LIDAR_3D* lidar = static_cast<LIDAR_3D*>(client_data);
        if(lidar == nullptr || data == nullptr)
        {
            return;
        }

        // get lidar index
        int idx = lidar->get_lidar_idx(handle);
        if(idx < 0 || !lidar->is_connected)
        {
            // printf("[LIDAR] handle: %u -> idx: %d\n", handle, idx);
            return;
        }

        // get timestamp type
        lidar->time_type[idx] = data->time_type;

        // parsing metadata
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec
        uint16_t time_interval = data->time_interval/10; // 0.1us -> nanosec
        // double lidar_time = time_base * N2S;
        // double pc_time = get_time()+st_time_for_get_time; // PC 시간
        // printf("[PTP-CHECK] lidar_time: %.9f, pc_time: %.9f, diff: %.6f sec\n", lidar_time, pc_time, lidar_time - pc_time);

        // parsing point cloud
        if(data->data_type == kLivoxLidarCartesianCoordinateHighData)
        {
            LivoxLidarCartesianHighRawPoint *p_point_data = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(data->data);
            if(lidar->offset_t[idx] == 0)
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

                Eigen::Vector3d _P = lidar->pts_tf[idx].block(0,0,3,3)*P + lidar->pts_tf[idx].block(0,3,3,1);
                double d = _P.norm();

                // double d = P.norm();
                if(d < lidar->config->LIDAR_3D_MIN_RANGE || d > lidar->config->LIDAR_3D_MAX_RANGE)
                {
                    continue;
                }

                // self collision filter
                // if(check_self_collision(_P(0), _P(1), _P(2),
                //                         lidar->config->ROBOT_SIZE_X[0], lidar->config->ROBOT_SIZE_X[1],
                //                         lidar->config->ROBOT_SIZE_Y[0], lidar->config->ROBOT_SIZE_Y[1],
                //                         lidar->config->ROBOT_SIZE_Z[0], lidar->config->ROBOT_SIZE_Z[1]))
                // {
                //     continue;
                // }

                LVX_PT pt;
                pt.t = (time_base + i * time_interval)*N2S + lidar->offset_t[idx]; // nanosec to sec
                pt.x = _P[0];
                pt.y = _P[1];
                pt.z = _P[2];
                pt.reflect = p_point_data[i].reflectivity;
                pt.tag = p_point_data[i].tag;

                pts.push_back(pt);
            }

            // update
            lidar->pts_storage[idx].insert(lidar->pts_storage[idx].end(), pts.begin(), pts.end());
            if(lidar->pts_storage[idx].size() >= 2 && (lidar->pts_storage[idx].back().t - lidar->pts_storage[idx].front().t) > 0.1)
            {
                // calc alpha
                const double t0 = lidar->pts_storage[idx].front().t;
                const double t1 = lidar->pts_storage[idx].back().t;

                for(size_t p = 0; p < lidar->pts_storage[idx].size(); p++)
                {
                    double t = lidar->pts_storage[idx][p].t;
                    double alpha = (t-t0)/(t1-t0);
                    lidar->pts_storage[idx][p].alpha = alpha;
                }

                // make frame
                LVX_FRM frm;
                frm.t = t0;
                // frm.pts = std::move(lidar->pts_storage[idx]);
                frm.pts = lidar->pts_storage[idx];

                // set queue
                lidar->frm_que[idx].push(frm);

                // for queue overflow
                if(lidar->frm_que[idx].unsafe_size() > 10)
                {
                    LVX_FRM tmp;
                    lidar->frm_que[idx].try_pop(tmp);
                }

                // set raw pts
                lidar->mtx.lock();
                lidar->cur_frm[idx] = frm;
                lidar->cur_frm_t[idx] = frm.t;
                lidar->cur_pts_num[idx] = frm.pts.size();
                lidar->mtx.unlock();

                // clear
                lidar->pts_storage[idx].clear();
                lidar->pts_storage[idx].push_back(frm.pts.back());
            }
        }
    };

    auto imu_data_callback = [](const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data)
    {
        // check
        LIDAR_3D* lidar = static_cast<LIDAR_3D*>(client_data);
        if(lidar == nullptr)
        {
            return;
        }

        if(data == nullptr)
        {
            return;
        }

        // get lidar index
        int idx = lidar->get_lidar_idx(handle);
        if(idx < 0 || !lidar->is_connected)
        {
            return;
        }

        // parsing
        uint64_t time_base = *reinterpret_cast<const uint64_t*>(data->timestamp); // nanosec

        // time sync
        if(lidar->offset_t[idx] == 0 || lidar->is_sync)
        {
            lidar->is_sync = false;
            lidar->offset_t[idx] = get_time() - (time_base * N2S);
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

        Eigen::Vector3d _acc_vec = lidar->imu_tf[idx].block(0,0,3,3)*acc_vec;
        Eigen::Vector3d _gyr_vec = lidar->imu_tf[idx].block(0,0,3,3)*gyr_vec;

        // considering centripetal acceleration
        Eigen::Vector3d imu_pos = lidar->imu_tf[idx].block(0,3,3,1);
        _acc_vec = _acc_vec - _gyr_vec.cross(_gyr_vec.cross(imu_pos));

        IMU imu;
        // imu.t = time_base * N2S; //+ lidar->offset_t[idx];
        imu.t = time_base * N2S + lidar->offset_t[idx];
        imu.acc_x = _acc_vec[0]*ACC_G; // m/s^2
        imu.acc_y = _acc_vec[1]*ACC_G;
        imu.acc_z = _acc_vec[2]*ACC_G;
        imu.gyr_x = _gyr_vec[0]; // rad/s
        imu.gyr_y = _gyr_vec[1];
        imu.gyr_z = _gyr_vec[2];

        //printf("[LIDAR] imu received, t: %f\n", imu.t);

        // get orientation using complementary filter
        double q0, q1, q2, q3;
        lidar->imu_filter[idx].update(_acc_vec[0], _acc_vec[1], _acc_vec[2], _gyr_vec[0], _gyr_vec[1], _gyr_vec[2], 0.005); // 200hz
        lidar->imu_filter[idx].getOrientation(q0, q1, q2, q3);
        Eigen::Matrix3d R = Eigen::Quaterniond(q0, q1, q2, q3).normalized().toRotationMatrix();
        Eigen::Vector3d r = Sophus::SO3d::fitToSO3(R).log();

        imu.rx = r[0];
        imu.ry = r[1];
        imu.rz = r[2];

        // update storage
        lidar->mtx.lock();

        if(lidar->offset_t[idx] != 0)
        {
            lidar->cur_imu[idx] = imu;
            lidar->cur_imu_t[idx] = imu.t;

            lidar->imu_storage[idx].push_back(imu);
            // if(lidar->imu_storage.front().t < imu.t - 3.0)
            // {
            //     lidar->imu_storage.erase(lidar->imu_storage.begin());
            // }

            if(lidar->imu_storage[idx].size() > 200)
            {
                lidar->imu_storage[idx].erase(lidar->imu_storage[idx].begin());
            }
        }

        lidar->mtx.unlock();
    };

    // Register callbacks
    SetLivoxLidarPointCloudCallBack(point_cloud_callback, this); // client_data is unused
    SetLivoxLidarImuDataCallback(imu_data_callback, this); // client_data is unused
    printf("[LIDAR] callback registered\n");


    printf("[LIDAR] grab_loop start\n");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // uninit
    LivoxLidarSdkUninit();

    printf("[LIDAR] grab_loop stop\n");
}

void LIDAR_3D::dsk_loop(int idx)
{
    IMU _pre_imu;

    printf("[LIDAR] dsk_loop[%d] start\n", idx);
    while(dsk_flag[idx])
    {
        LVX_FRM frm;
        if(frm_que[idx].try_pop(frm))
        {
            // skip
            // if(frm.pts.size() < config->LOC_ICP_MAX_FEATURE_NUM*0.5)
            // {
            //     continue;
            // }

            // imu
            IMU _cur_imu = get_best_imu(frm.t, idx);

            // deskewing
            std::vector<Eigen::Vector3d> dsk;
            {
                IMU imu0 = get_best_imu(frm.pts.front().t, idx);
                IMU imu1 = get_best_imu(frm.pts.back().t, idx);

                Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(imu0.rx, imu0.ry, imu0.rz)).matrix();
                Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(imu1.rx, imu1.ry, imu1.rz)).matrix();
                Eigen::Matrix3d dR = R0.inverse()*R1;
                Eigen::Matrix4d dG = Eigen::Matrix4d::Identity();
                dG.block(0,0,3,3) = dR;

                // std::vector<Eigen::Vector3d> _deskewed_pts(frm.pts.size());
                // std::vector<Eigen::Vector6d> model = calc_linear_motion_model(frm);
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Matrix4d G_i = intp_tf(frm.pts[p].alpha, Eigen::Matrix4d::Identity(), dG);
                    Eigen::Vector3d P(frm.pts[p].x, frm.pts[p].y, frm.pts[p].z);
                    Eigen::Vector3d _P = G_i.block(0,0,3,3)*P + G_i.block(0,3,3,1);
                    dsk.push_back(_P);
                }
            }

            // make frame
            TIME_PTS dsk_frm;
            dsk_frm.t = frm.t;
            dsk_frm.pts = dsk;
            // printf("dsk_frm[%d] t:%f, pts:%zu\n", idx, dsk_frm.t, dsk_frm.pts.size());

            // set queue
            dsk_que[idx].push(dsk_frm);

            // for queue overflow
            if(dsk_que[idx].unsafe_size() > 10)
            {
                TIME_PTS tmp;
                dsk_que[idx].try_pop(tmp);
            }

            // for next
            _pre_imu = _cur_imu;

        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LIDAR] dsk_loop[%d] stop\n", idx);
}

void LIDAR_3D::a_loop()
{
    // 10ms
    double timeout = 0.05;
    double wait_timeout = 0.15;
    double last_recv_t[config->LIDAR_3D_NUM];

    std::vector<TIME_PTS> storage[config->LIDAR_3D_NUM];

    printf("[LIDAR] a_loop start\n");
    while(a_flag)
    {
        TIME_PTS merge_frm;
        merge_frm.pts.clear();

        // set storage

        if(time_type[0].load() == 1 && time_type[1].load() == 1)
        {
            for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
            {
                TIME_PTS frm;
                if(dsk_que[idx].try_pop(frm))
                {
                    storage[idx].push_back(frm);
                    last_recv_t[idx] = frm.t;

                    if(storage[idx].size() > 10)
                    {
                        storage[idx].erase(storage[idx].begin());
                    }
                }
            }
        }
        else
        {
            TIME_PTS frm;
            if(dsk_que[0].try_pop(frm))
            {
                storage[0].push_back(frm);
                last_recv_t[0] = frm.t;

                if(storage[0].size() > 10)
                {
                    storage[0].erase(storage[0].begin());
                }
            }
        }

        /*
        // find the closest pair
        bool matched = false;
        TIME_PTS best0, best1;
        double min_dt = 9999;
        size_t best_i = 0, best_j = 0;

        for(size_t i = 0; i < storage[0].size(); i++)
        {
            for(size_t j = 0; j < storage[1].size(); j++)
            {
                double dt = std::abs(storage[0][i].t - storage[1][j].t);
                if(dt < timeout && dt < min_dt)
                {
                    matched = true;
                    min_dt = dt;
                    best0 = storage[0][i];
                    best1 = storage[1][j];
                    best_i = i;
                    best_j = j;
                }
            }
        }
        */


        bool matched = false;
        TIME_PTS best0, best1;
        double min_dt = 9999;
        size_t ref_i = 0;
        size_t best_j = 0;

        if(!storage[0].empty() && !storage[1].empty())
        {
            ref_i = storage[0].size() - 1;

            for(size_t j = 0; j < storage[1].size(); j++)
            {
                double dt = std::abs(storage[0][ref_i].t - storage[1][j].t);
                if(dt < timeout && dt < min_dt)
                {
                    min_dt = dt;
                    best0 = storage[0][ref_i];
                    best1 = storage[1][j];
                    best_j = j;
                    matched = true;
                }
            }
        }

        if(matched)
        {
            // merge
            merge_frm.t = std::min(best0.t, best1.t);
            merge_frm.pts.insert(merge_frm.pts.end(), best0.pts.begin(), best0.pts.end());
            merge_frm.pts.insert(merge_frm.pts.end(), best1.pts.begin(), best1.pts.end());

            // erase
            // storage[0].erase(storage[0].begin(), storage[0].begin() + best_i + 1);
            storage[0].erase(storage[0].begin(), storage[0].begin() + ref_i + 1);
            storage[1].erase(storage[1].begin(), storage[1].begin() + best_j + 1);
            // printf("[LIDAR] paired t=%.6f, idx: (%zu, %zu), pts0_t=%.6f, pts1_t=%.6f, total=%zu (storage: %zu, %zu))\n", merge_frm.t, ref_i, best_j, best0.t, best1.t, merge_frm.pts.size(), storage[0].size(), storage[1].size());
        }
        else
        {
            // use single frame only if too old to wait for match
            int use_idx = -1;

            if(!storage[0].empty() && storage[1].empty())
            {
                if(storage[0].back().t - last_recv_t[1] > wait_timeout)
                {
                    use_idx = 0;
                    // printf("[LIDAR] 0-time: %f - %f = %f\n", storage[0].back().t, last_recv_t[1], storage[0].back().t - last_recv_t[1]);
                }
            }
            else if(storage[0].empty() && !storage[1].empty())
            {
                if(storage[1].back().t - last_recv_t[0] > wait_timeout)
                {
                    use_idx = 1;
                    // printf("[LIDAR] 1-time: %f - %f = %f\n", storage[1].back().t, last_recv_t[0], storage[1].back().t - last_recv_t[0]);
                }
            }
            else if(!storage[0].empty() && !storage[1].empty())
            {
                if(std::abs(storage[0].back().t - storage[1].back().t) > wait_timeout)
                {
                    // use_idx = (t0 < t1) ? 0 : 1;
                    use_idx = 0;
                }
            }

            if(use_idx >= 0)
            {
                merge_frm = storage[use_idx].back();
                storage[use_idx].pop_back();

                // printf("[LIDAR] single  t=%.6f, from=%d, pts=%zu (storage: %zu, %zu)\n", merge_frm.t, use_idx, merge_frm.pts.size(), storage[0].size(), storage[1].size());
            }
        }

        // save result
        if((int)merge_frm.pts.size() > 0)
        {
            merged_que.push(merge_frm);

            // watchdog
            cur_merged_frm_t = merge_frm.t;
            cur_merged_num = merge_frm.pts.size();

            if(merged_que.unsafe_size() > 10)
            {
                TIME_PTS tmp;
                merged_que.try_pop(tmp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LIDAR] a_loop stop\n");
}


