#include "lidar_2d.h"

LIDAR_2D::LIDAR_2D(QObject *parent)
    : QObject{parent}
{

}

LIDAR_2D::~LIDAR_2D()
{
    is_connected = false;

    // close lidar
    if(sick != NULL)
    {
        sick->close();
        delete sick;
        sick = NULL;
    }

    close();
}

void LIDAR_2D::init()
{
    if(config->LIDAR_2D_TYPE == "SICK")
    {
        if(sick == nullptr)
        {
            sick = new SICK();
            sick->config = this->config;
            sick->logger = this->logger;
            sick->mobile = this->mobile;
            sick->init();
            sick->open();
        }
    }
}

void LIDAR_2D::open()
{
    printf("[LIDAR_2D] open\n");

    // stop first
    close();

    // dsk loop start
    for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
    {
        if(dsk_thread[idx] == NULL)
        {
            dsk_flag[idx] = true;
            dsk_thread[idx] = new std::thread(&LIDAR_2D::dsk_loop, this, idx);
        }
    }

    // a loop start
    if(a_thread == NULL)
    {
        a_flag = true;
        a_thread = new std::thread(&LIDAR_2D::a_loop, this);
    }
}

void LIDAR_2D::close()
{
    is_connected = false;

    for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
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

RAW_FRAME LIDAR_2D::get_cur_frm(int idx)
{
    RAW_FRAME res;

    if(config->LIDAR_2D_TYPE == "SICK" && sick != nullptr)
    {
        mtx.lock();
        res = sick->cur_frm[idx];
        mtx.unlock();
    }

    return res;
}

QString LIDAR_2D::get_info_text()
{
    QString res;

    if(config->LIDAR_2D_TYPE == "SICK" && sick != nullptr)
    {
        for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
        {
            res += sick->get_info_text(idx);
            res += QString().sprintf("dq: %d\n\n", (int)dsk_que[idx].unsafe_size());
        }
    }

    res += QString().sprintf("mq: %d (%d)",(int)merged_que.unsafe_size(), cur_merged_num.load());

    return res;
}

void LIDAR_2D::set_sync_flag(bool flag)
{
    is_sync = flag;

    if(config->LIDAR_2D_TYPE == "SICK" && sick != nullptr)
    {
        for(int p=0; p<config->LIDAR_2D_NUM; p++)
        {
            sick->is_sync[p].store(flag);
            printf("[LIDAR_2D] set sick->is_sync = %d\n", flag);
        }
    }
}

void LIDAR_2D::dsk_loop(int idx)
{
    printf("[LIDAR_2D] dsk_loop[%d] start\n", idx);
    while(dsk_flag[idx])
    {
        if(!is_connected)
        {
            is_connected = true;
        }

        if(is_connected == false)
        {
            // drop
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        RAW_FRAME frm;
        if(sick->raw_que[idx].try_pop(frm))
        {
            double t0 = frm.t0;
            double t1 = frm.t1;

            // deskewing
            std::vector<Eigen::Vector3d> dsk;
            {
                // precise deskewing
                Eigen::Matrix4d tf0 = se2_to_TF(frm.pose0);
                Eigen::Matrix4d tf1 = se2_to_TF(frm.pose1);
                Eigen::Matrix4d dtf = tf0.inverse()*tf1;

                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    double t = frm.times[p];
                    double alpha = (t-t0)/(t1-t0);
                    Eigen::Matrix4d tf = intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);
                    dsk.push_back(tf.block(0,0,3,3)*frm.pts[p] + tf.block(0,3,3,1));
                }
            }

            std::vector<double> filtered_time;
            std::vector<double> filtered_reflects;
            std::vector<Eigen::Vector3d> filetered_dsk0;

            auto filtered_dsk = scan_shadow_filter(dsk, 5);
            for(size_t p = 0; p < filtered_dsk.size(); p++)
            {
                if(filtered_dsk[p].second)
                {
                    Eigen::Vector3d P = filtered_dsk[p].first;
                    filetered_dsk0.push_back(P);

                    filtered_reflects.push_back(frm.reflects[p]);
                    filtered_time.push_back(frm.times[p]);
                }
            }

            // make frame
            RAW_FRAME dsk_frm;
            dsk_frm.t0 = frm.t0;
            dsk_frm.t1 = frm.t1;
            dsk_frm.mo = frm.mo;
            dsk_frm.reflects = filtered_reflects;
            dsk_frm.times = filtered_time;
            dsk_frm.pts = filetered_dsk0;

            printf("dsk_frm[%d] t:%f, filtered_pts:%zu, pts:%zu\n", idx, dsk_frm.mo.t, dsk_frm.pts.size(), dsk.size());

            // set queue
            dsk_que[idx].push(dsk_frm);

            // for queue overflow
            if(dsk_que[idx].unsafe_size() > 10)
            {
                RAW_FRAME tmp;
                dsk_que[idx].try_pop(tmp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LIDAR_2D] dsk_loop[%d] stop\n", idx);
}

void LIDAR_2D::a_loop()
{
    // 10ms
    double timeout = 0.05;
    double wait_timeout = 0.15;
    double last_recv_t[config->LIDAR_2D_NUM];

    std::vector<RAW_FRAME> storage[2];

    printf("[LIDAR_2D] a_loop start\n");
    while(a_flag)
    {
        // if single lidar
        if(config->LIDAR_2D_NUM == 1)
        {
            RAW_FRAME raw_frm;
            if(dsk_que[0].try_pop(raw_frm))
            {
                FRAME frm;
                frm.t = raw_frm.mo.t;
                frm.mo = raw_frm.mo;
                frm.pts = raw_frm.pts;
                frm.reflects = raw_frm.reflects;

                cur_merged_frm_t = frm.t;
                cur_merged_num = frm.pts.size();

                merged_que.push(frm);
                if(merged_que.unsafe_size() > 10)
                {
                    FRAME tmp;
                    merged_que.try_pop(tmp);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // if pair lidar
        if(sick->is_connected[0].load() && sick->is_connected[1].load())
        {
            for(int idx = 0; idx < config->LIDAR_2D_NUM; idx++)
            {
                RAW_FRAME frm;
                if(dsk_que[idx].try_pop(frm))
                {
                    last_recv_t[idx] = frm.mo.t;

                    storage[idx].push_back(frm);
                    if(storage[idx].size() > 10)
                    {
                        storage[idx].erase(storage[idx].begin());
                    }
                }
            }
        }
        else
        {
            RAW_FRAME frm;
            if(dsk_que[0].try_pop(frm))
            {
                last_recv_t[0] = frm.mo.t;

                storage[0].push_back(frm);
                if(storage[0].size() > 10)
                {
                    storage[0].erase(storage[0].begin());
                }
            }
        }

        bool matched = false;
        RAW_FRAME best0, best1;
        double min_dt = std::numeric_limits<double>::max();
        size_t ref_i = 0;
        size_t best_j = 0;
        if(!storage[0].empty() && !storage[1].empty())
        {
            ref_i = 0;
            best0 = storage[0].front();

            for(size_t j = 0; j < storage[1].size(); j++)
            {
                double dt = std::abs(storage[0][ref_i].t0 - storage[1][j].t0);
                if(dt < timeout && dt < min_dt)
                {
                    min_dt = dt;
                    best1 = storage[1][j];
                    best_j = j;
                    matched = true;
                }
            }
        }

        FRAME merge_frm;
        merge_frm.pts.clear();
        if(matched)
        {
            Eigen::Matrix4d dG = se2_to_TF(best0.mo.pose).inverse()*se2_to_TF(best1.mo.pose);

            std::vector<Eigen::Vector3d> best1_pts;
            std::vector<double> best1_reflects;
            for(size_t p = 0; p < best1.pts.size(); p++)
            {
                Eigen::Vector3d P = best1.pts[p];
                Eigen::Vector3d _P = dG.block(0,0,3,3)*P + dG.block(0,3,3,1);
                best1_pts.push_back(_P);
                best1_reflects.push_back(best1.reflects[p]);
            }

            // merge
            merge_frm.t = best0.mo.t;
            merge_frm.mo = best0.mo;

            merge_frm.pts.insert(merge_frm.pts.end(), best0.pts.begin(), best0.pts.end());
            merge_frm.pts.insert(merge_frm.pts.end(), best1_pts.begin(), best1_pts.end());

            merge_frm.reflects.insert(merge_frm.reflects.end(), best0.reflects.begin(), best0.reflects.end());
            merge_frm.reflects.insert(merge_frm.reflects.end(), best1_reflects.begin(), best1_reflects.end());

            storage[0].erase(storage[0].begin(), storage[0].begin() + ref_i + 1);
            storage[1].erase(storage[1].begin(), storage[1].begin() + best_j + 1);

            //printf("[LIDAR] paired t=%.6f, idx: (%zu, %zu), pts0_t=%.6f, pts1_t=%.6f, pts0=%zu, pts1=%zu, total=%zu (storage: %zu, %zu))\n", merge_frm.t, ref_i, best_j, best0.t0, best1.t0, merge_frm.pts.size(), best0.pts.size(), best1_pts.size(), storage[0].size(), storage[1].size());
        }
        else
        {
            // use single frame only if too old to wait for match
            int use_idx = -1;

            if(!storage[0].empty() && storage[1].empty())
            {
                if(storage[0].back().mo.t - last_recv_t[1] > wait_timeout)
                {
                    use_idx = 0;
                    // printf("[LIDAR] 0-time: %f - %f = %f\n", storage[0].back().t, last_recv_t[1], storage[0].back().t - last_recv_t[1]);
                }
            }
            else if(storage[0].empty() && !storage[1].empty())
            {
                if(storage[1].back().mo.t - last_recv_t[0] > wait_timeout)
                {
                    use_idx = 1;
                    // printf("[LIDAR] 1-time: %f - %f = %f\n", storage[1].back().t, last_recv_t[0], storage[1].back().t - last_recv_t[0]);
                }
            }
            else if(!storage[0].empty() && !storage[1].empty())
            {
                if(std::abs(storage[0].back().mo.t - storage[1].back().mo.t) > wait_timeout)
                {
                    // use_idx = (t0 < t1) ? 0 : 1;
                    use_idx = 0;
                }
            }

            if(use_idx >= 0)
            {
                //merge_frm = storage[use_idx].back();
                //storage[use_idx].pop_back();

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
                FRAME tmp;
                merged_que.try_pop(tmp);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    printf("[LIDAR_2D] a_loop stop\n");
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

std::vector<std::pair<Eigen::Vector3d, bool>> LIDAR_2D::scan_shadow_filter(const std::vector<Eigen::Vector3d>& dsk, int shadow_window)
{
    const double min_angle_tan = std::tan(5.0*D2R);
    const double max_angle_tan = std::tan(175.0*D2R);

    std::vector<std::pair<Eigen::Vector3d, bool>> filtered_pts;
    filtered_pts.resize(dsk.size());

    for(size_t p = 0; p < dsk.size(); p++)
    {
        Eigen::Vector3d P1 = dsk[p];
        double r1 = std::sqrt(P1(0)*P1(0) + P1(1)*P1(1));

        bool is_good = true;
        for(int k = -shadow_window; k < shadow_window; k++)
        {
            int neighbor_idx = p + k;
            if(neighbor_idx >= 0 && neighbor_idx < (int)dsk.size())
            {
                Eigen::Vector3d P2 = dsk[neighbor_idx];
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
            filtered_pts[p] = std::make_pair(P1, true);
        }
        else
        {
            filtered_pts[p] = std::make_pair(Eigen::Vector3d(0,0,0), false);
        }
    }

    return filtered_pts;
}
