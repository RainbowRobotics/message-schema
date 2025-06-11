#include "lidar_3d.h"

LIDAR_3D::LIDAR_3D(QObject *parent) : QObject(parent)
{

}

LIDAR_3D::~LIDAR_3D()
{
    is_connected = false;

    // close lidar
    if(livox != NULL)
    {
        livox->close();
        delete livox;
        livox = NULL;
    }

    close();
}

void LIDAR_3D::init()
{
    if(config->LIDAR_3D_TYPE == "LIVOX")
    {
        if(livox == nullptr)
        {
            livox = new LIVOX();
            livox->config = this->config;
            livox->logger = this->logger;
            livox->init();
            livox->open();
        }
    }
    printf("[LIDAR_3D] init\n");
}

void LIDAR_3D::open()
{
    printf("[LIDAR_3D] open\n");

    // stop first
    close();

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

LVX_FRM LIDAR_3D::get_cur_frm(int idx)
{
    LVX_FRM res;

    if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
    {
        mtx.lock();
        res = livox->cur_frm[idx];
        mtx.unlock();
    }

    return res;
}

IMU LIDAR_3D::get_cur_imu(int idx)
{
    IMU res;

    if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
    {
        mtx.lock();
        res = livox->cur_imu[idx];
        mtx.unlock();
    }

    return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t)
{
    std::vector<std::vector<IMU>> imu_storage(config->LIDAR_3D_NUM);
    if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
    {
        livox->mtx.lock();
        for(int i = 0; i < config->LIDAR_3D_NUM; i++)
        {
            imu_storage[i] = livox->imu_storage[i];
        }
        livox->mtx.unlock();
    }
    else
    {
        printf("[LIDAR_3D] unknown LIDAR type or lidar not initialized\n");
        return IMU();
    }

    double min_dt = 99999999;
    IMU res;
    for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
    {
        const std::vector<IMU>& list = imu_storage[idx];
        for(size_t p = 0; p < list.size(); p++)
        {
            double dt = std::abs(list[p].t - ref_t);
            if(dt < min_dt)
            {
                min_dt = dt;
                res = list[p];
            }
        }
    }

    return res;
}

IMU LIDAR_3D::get_best_imu(double ref_t, int idx)
{
    std::vector<IMU> imu_storage;
    if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
    {
        livox->mtx.lock();
        imu_storage = livox->imu_storage[idx];
        livox->mtx.unlock();
    }
    else
    {
        printf("[LIDAR_3D] unknown LIDAR type or lidar not initialized\n");
        return IMU();
    }

    double min_dt = 99999999;
    IMU res = imu_storage.back();
    for(size_t p = 0; p < imu_storage.size(); p++)
    {
        double dt = std::abs(imu_storage[p].t - ref_t);
        if(dt < min_dt)
        {
            min_dt = dt;
            res = imu_storage[p];
        }
    }

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
    QString res;

    if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
    {
        for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
        {
            res += livox->get_info_text(idx);

            res += QString().sprintf("dq: %d\n\n", (int)dsk_que[idx].unsafe_size());
        }
    }

    res += QString().sprintf("mq: %d (%d)",(int)merged_que.unsafe_size(), cur_merged_num.load());

    return res;
}

void LIDAR_3D::set_sync_flag(bool flag)
{
    is_sync = true;

    for(int idx = 0; idx < config->LIDAR_3D_NUM; idx++)
    {
        if(config->LIDAR_3D_TYPE == "LIVOX" && livox != NULL)
        {
            livox->is_sync[idx].store(flag);
            printf("[LIDAR_3D] set livox->is_sync[%d] = %d\n",idx, flag);
        }
    }
}

void LIDAR_3D::dsk_loop(int idx)
{
    IMU _pre_imu;

    printf("[LIDAR_3D] dsk_loop[%d] start\n", idx);
    while(dsk_flag[idx])
    {
        if(!is_connected && livox->is_connected)
        {
            is_connected = true;
        }

        LVX_FRM frm;
        if(livox->frm_que[idx].try_pop(frm))
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
    printf("[LIDAR_3D] dsk_loop[%d] stop\n", idx);
}

void LIDAR_3D::a_loop()
{
    // 10ms
    double timeout = 0.05;
    double wait_timeout = 0.15;
    double last_recv_t[config->LIDAR_3D_NUM];

    std::vector<TIME_PTS> storage[2];

    printf("[LIDAR_3D] a_loop start\n");
    while(a_flag)
    {
        TIME_PTS merge_frm;
        merge_frm.pts.clear();

        // set storage
        if(config->LIDAR_3D_NUM == 1)
        {
            TIME_PTS frm;
            if(dsk_que[0].try_pop(frm))
            {
                merged_que.push(frm);
                cur_merged_frm_t = frm.t;
                cur_merged_num = frm.pts.size();

                if(merged_que.unsafe_size() > 10)
                {
                    TIME_PTS tmp;
                    merged_que.try_pop(tmp);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        else if((livox->time_type[0].load() == 1 && livox->time_type[1].load() == 1))
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
    printf("[LIDAR_3D] a_loop stop\n");
}
