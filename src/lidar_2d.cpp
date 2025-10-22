#include "lidar_2d.h"
namespace 
{
    const char* MODULE_NAME = "LIDAR_2D";
}

LIDAR_2D* LIDAR_2D::instance(QObject* parent)
{
    static LIDAR_2D* inst = nullptr;
    if(!inst && parent)
    {
        inst = new LIDAR_2D(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

LIDAR_2D::LIDAR_2D(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    sick(nullptr),
    rp(nullptr)
{

}

LIDAR_2D::~LIDAR_2D()
{
    is_connected.store(false);

    // close lidar
    if(sick != nullptr)
    {
        sick->close();
    }

    if(rp != nullptr)
    {
        rp->close();
    }

    close();
}

void LIDAR_2D::init()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        spdlog::info("[LIDAR_2D] simulation mode init");

        return;
    }

    //std::cout << "config->get_lidar_2d_type(): " << config->get_lidar_2d_type().toStdString() << std::endl;
    spdlog::info("[LIDAR_2D]config->get_lidar_2d_type(): {}", config->get_lidar_2d_type().toStdString());

    if(config->get_lidar_2d_type() == "SICK")
    {
        if(!sick)
        {
            SICK::instance(this);

            sick = SICK::instance();
            sick->set_config_module(this->config);
            sick->set_logger_module(this->logger);
            sick->set_mobile_module(this->mobile);
            sick->open();
            //printf("[LIDAR_2D] try to open SICK 2D lidar\n");
            if(config)
            {
                spdlog::info("[LIDAR_2D] try to open SICK 2D lidar");
            }
        }
    }
    else if(config->get_lidar_2d_type() == "RP")
    {
        if(!rp)
        {
            RP_LIDAR::instance(this);

            rp = RP_LIDAR::instance();
            rp->set_config_module(this->config);
            rp->set_logger_module(this->logger);
            rp->set_mobile_module(this->mobile);
            rp->open();
            //printf("[LIDAR_2D] try to open RP 2D lidar\n");
            if(config && config->get_log_level() != "off")
            {
                spdlog::info("[LIDAR_2D] try to open RP 2D lidar");
            }
        }

    }
}

void LIDAR_2D::open()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        //printf("[LIDAR_2D] simulation mode\n");

          spdlog::info("[LIDAR_2D] simulation mode open");
        return;
    }

    //printf("[LIDAR_2D] open\n");
    if(config && config->get_log_level() != "off")
    {
        spdlog::info("[LIDAR_2D] open");
    }

    // stop first
    close();

    // deskewing loop start
    int lidar_num = config->get_lidar_2d_num();
    for(int idx = 0; idx < lidar_num; idx++)
    {
        deskewing_flag[idx]   = true;
        deskewing_thread[idx] = std::make_unique<std::thread>(&LIDAR_2D::deskewing_loop, this, idx);
    }

    // merge loop start
    merge_flag = true;
    merge_thread = std::make_unique<std::thread>(&LIDAR_2D::merge_loop, this);
}

void LIDAR_2D::close()
{
    is_connected = false;

    int lidar_num = config->get_lidar_2d_num();
    for(int idx = 0; idx < lidar_num; idx++)
    {
        deskewing_flag[idx] = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    merge_flag = false;
    if(merge_thread && merge_thread->joinable())
    {
        merge_thread->join();
    }
    merge_thread.reset();

    for(int idx = 0; idx < lidar_num; idx++)
    {
        if(deskewing_thread[idx] && deskewing_thread[idx]->joinable())
        {
            deskewing_thread[idx]->join();
        }
        deskewing_thread[idx].reset();
    }
}

RAW_FRAME LIDAR_2D::get_cur_raw(int idx)
{
    RAW_FRAME res;
    if(config->get_lidar_2d_type() == "SICK" && sick != nullptr)
    {
        res = sick->get_cur_raw(idx);
    }
    else if(config->get_lidar_2d_type() == "RP" && rp != nullptr)
    {
        res = rp->get_cur_raw(idx);
    }
    return res;
}

FRAME LIDAR_2D::get_cur_frm()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return cur_frm;
}

QString LIDAR_2D::get_info_text()
{
    QString res;
    if(config->get_lidar_2d_type() == "SICK" && sick != nullptr)
    {
        for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
        {
            res += sick->get_info_text(idx);
            res += QString("dq: %1\n\n").arg(deskewing_que[idx].unsafe_size());
        }
    }
    else if(config->get_lidar_2d_type() == "RP" && rp != nullptr)
    {
        for(int idx = 0; idx < config->get_lidar_2d_num(); idx++)
        {
            res += rp->get_info_text(idx);
            res += QString("dq: %1\n\n").arg(deskewing_que[idx].unsafe_size());
        }
    }
    res += QString("mq: %1 (%2)").arg((int)merged_que.unsafe_size()).arg(cur_merged_num.load());
    return res;
}

bool LIDAR_2D::get_is_connected()
{
    return (bool)is_connected.load();
}

bool LIDAR_2D::get_is_sync()
{
    return (bool)is_sync.load();
}

double LIDAR_2D::get_process_time_deskewing(int idx)
{
    return (double)process_time_deskewing[idx].load();
}

double LIDAR_2D::get_process_time_merge()
{
    return (double)process_time_merge.load();
}

void LIDAR_2D::set_sync_flag(bool flag)
{
    is_sync = flag;
    int lidar_num = config->get_lidar_2d_num();
    if(config->get_lidar_2d_type() == "SICK" && sick != nullptr)
    {
        for(int p = 0; p < lidar_num; p++)
        {
            sick->set_is_sync(p, flag);
            //printf("[LIDAR_2D] set sick->is_sync = %d\n", flag);
            log_info("[LIDAR_2D] set sick->is_sync = {}", flag);
        }
    }
    else if(config->get_lidar_2d_type() == "RP" && rp != nullptr)
    {
        for(int p = 0; p < lidar_num; p++)
        {
            rp->set_is_sync(p, flag);
            //printf("[LIDAR_2D] set rp->is_sync = %d\n", flag);
            spdlog::info("[LIDAR_2D] set rp->is_sync = %d", flag);
        }
    }
}

void LIDAR_2D::set_is_connected(bool val)
{
    is_connected.store(val);
}

void LIDAR_2D::clear_merged_queue()
{
    merged_que.clear();
}

bool LIDAR_2D::try_pop_merged_queue(FRAME& frm)
{
    if(merged_que.try_pop(frm))
    {
        return true;
    }

    return false;
}

bool LIDAR_2D::get_deskewing_frm(FRAME &frm, int idx)
{
    if(idx < 0 || idx >= 2)
    {
        return false;
    }

    double served_t = last_served_dsk_t[idx].load();

    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        if(last_dsk_frm[idx].mo.t < 0.0 || last_dsk_frm[idx].mo.t <= served_t)
        {
            return false;
        }

        // raw frame -> frame
        frm.t = last_dsk_frm[idx].mo.t;
        frm.mo = last_dsk_frm[idx].mo;
        frm.pts = last_dsk_frm[idx].pts;
        frm.reflects = last_dsk_frm[idx].reflects;
    }

    last_served_dsk_t[idx] = frm.mo.t;
    return true;
}

void LIDAR_2D::deskewing_loop(int idx)
{
    double pre_loop_time = get_time();

    //printf("[LIDAR_2D] dsk_loop[%d] start\n", idx);
    spdlog::info("[LIDAR_2D] dsk_loop[%d] start", idx);

    while(deskewing_flag[idx])
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
        bool is_try_pop = false;
        if(config->get_lidar_2d_type() == "SICK")
        {
            is_try_pop = sick->try_pop_raw_que(idx, frm);
        }
        else if(config->get_lidar_2d_type() == "RP")
        {
            is_try_pop = rp->try_pop_raw_que(idx, frm);
        }

        if(is_try_pop)
        {
            double t0 = frm.t0;
            double t1 = frm.t1;

            // deskewing
            std::vector<Eigen::Vector3d> deskewing_pts;
            {
                // precise deskewing
                Eigen::Matrix4d tf0 = se2_to_TF(frm.pose0);
                Eigen::Matrix4d tf1 = se2_to_TF(frm.pose1);
                Eigen::Matrix4d dtf = tf0.inverse() * tf1;

                deskewing_pts.reserve(frm.pts.size());
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    double t = frm.times[p];
                    double alpha = (t-t0)/((t1-t0) + 1e-06);

                    Eigen::Matrix4d tf = intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);
                    deskewing_pts.push_back(tf.block(0,0,3,3)*frm.pts[p] + tf.block(0,3,3,1));
                }
            }

            std::vector<double> filtered_time;
            std::vector<double> filtered_reflects;
            std::vector<Eigen::Vector3d> filetered_deskewing;

            auto filtered_dsk = scan_shadow_filter(deskewing_pts, 5);
            for(size_t p = 0; p < filtered_dsk.size(); p++)
            {
                if(filtered_dsk[p].second)
                {
                    Eigen::Vector3d P = filtered_dsk[p].first;
                    filetered_deskewing.push_back(P);

                    filtered_reflects.push_back(frm.reflects[p]);
                    filtered_time.push_back(frm.times[p]);
                }
            }

            // make frame
            RAW_FRAME deskewing_frm;
            deskewing_frm.t0 = frm.t0;
            deskewing_frm.t1 = frm.t1;
            deskewing_frm.mo = frm.mo;
            deskewing_frm.reflects = std::move(filtered_reflects);
            deskewing_frm.times    = std::move(filtered_time);
            deskewing_frm.pts      = std::move(filetered_deskewing);

            //       printf("dsk_frm[%d] t:%f, filtered_pts:%zu, pts:%zu\n", idx, dsk_frm.mo.t, dsk_frm.pts.size(), dsk.size());

            // for mapping
            {
                std::shared_lock<std::shared_mutex> lock(mtx);
                last_dsk_frm[idx] = deskewing_frm;
            }

            // set queue
            deskewing_que[idx].push(deskewing_frm);

            // for queue overflow
            if(deskewing_que[idx].unsafe_size() > LIDAR_2D_INFO::deskewing_que_max_size)
            {
                RAW_FRAME tmp;
                deskewing_que[idx].try_pop(tmp);
            }

            process_time_deskewing[idx] = (double)(get_time() - pre_loop_time);
            pre_loop_time = get_time();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    spdlog::info("[LIDAR_2D] deskewing_loop[%d] stop", idx);
}

void LIDAR_2D::merge_loop()
{
    const int lidar_num = config->get_lidar_2d_num();

    // 10ms
    const double timeout = 0.05;
    const double wait_timeout = 0.15;
    double last_recv_t[lidar_num];

    std::vector<RAW_FRAME> storage[lidar_num];

    double pre_loop_time = get_time();

    //printf("[LIDAR_2D] merge_loop start\n");
    spdlog::info("[LIDAR_2D] merge_loop start");

    while(merge_flag)
    {
        // if single lidar
        if(lidar_num == 1)
        {
            RAW_FRAME raw_frm;
            if(deskewing_que[0].try_pop(raw_frm))
            {
                FRAME frm;
                frm.t   = raw_frm.mo.t;
                frm.mo  = raw_frm.mo;
                frm.pts = raw_frm.pts;
                frm.reflects = raw_frm.reflects;

                cur_merged_frm_t = frm.t;
                cur_merged_num   = frm.pts.size();

                // update
                merged_que.push(frm);
                if(merged_que.unsafe_size() > LIDAR_2D_INFO::merge_que_max_size)
                {
                    FRAME tmp;
                    merged_que.try_pop(tmp);
                }

                std::unique_lock<std::shared_mutex> lock(mtx);
                cur_frm = frm;
            }

            process_time_merge = (double)(get_time() - pre_loop_time);
            pre_loop_time = get_time();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // if pair lidar
        int lidar_num = config->get_lidar_2d_num();
        if(lidar_num == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        bool is_ok = false;
        if(lidar_num == 2)
        {
            if(config->get_lidar_2d_type() == "SICK")
            {
                is_ok = (sick->get_is_connected(0) && sick->get_is_connected(1));
            }
            else if(config->get_lidar_2d_type() == "RP")
            {
                is_ok = (rp->get_is_connected(0) && rp->get_is_connected(1));
            }
        }

        if(is_ok)
        {
            for(int idx = 0; idx < lidar_num; idx++)
            {
                RAW_FRAME frm;
                if(deskewing_que[idx].try_pop(frm))
                {
                    last_recv_t[idx] = frm.mo.t;

                    storage[idx].push_back(frm);
                    if(storage[idx].size() > LIDAR_2D_INFO::deskewing_storage_max_size)
                    {
                        storage[idx].erase(storage[idx].begin());
                    }
                }
            }
        }
        else
        {
            RAW_FRAME frm;
            if(deskewing_que[0].try_pop(frm))
            {
                last_recv_t[0] = frm.mo.t;

                storage[0].push_back(frm);
                if(storage[0].size() > LIDAR_2D_INFO::deskewing_storage_max_size)
                {
                    storage[0].erase(storage[0].begin());
                }
            }
        }

        bool matched  = false;
        double min_dt = std::numeric_limits<double>::max();
        size_t ref_i  = 0;
        size_t best_j = 0;
        RAW_FRAME best0, best1;
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
            if(config->set_debug_lidar_2d())
            {
                spdlog::debug("[LIDAR] paired t={:.6f}, idx: ({}, {}), pts0_t={:.6f}, pts1_t={:.6f},pts0={}, pts1={}, total={} (storage: {}, {})",merge_frm.t,ref_i, best_j,best0.t0, best1.t0,merge_frm.pts.size(),best0.pts.size(), best1_pts.size(),storage[0].size(),storage[1].size());
            }
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

                    //printf("[LIDAR] 0-time: %f - %f = %f\n", storage[0].back().t, last_recv_t[1], storage[0].back().t - last_recv_t[1]);
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
                spdlog::debug("[LIDAR] single  t={:.6f}, from={}, pts={} (storage: {}, {})",merge_frm.t,use_idx, merge_frm.pts.size(),storage[0].size(),storage[1].size());
            }
        }

        // save result
        if((int)merge_frm.pts.size() > 0)
        {
            merged_que.push(merge_frm);

            // watchdog
            cur_merged_frm_t = merge_frm.t;
            cur_merged_num = merge_frm.pts.size();

            if(merged_que.unsafe_size() > LIDAR_2D_INFO::merge_que_max_size)
            {
                FRAME tmp;
                merged_que.try_pop(tmp);
            }

            std::unique_lock<std::shared_mutex> lock(mtx);
            cur_frm = merge_frm;
        }

        process_time_merge = (double)(get_time() - pre_loop_time);
        pre_loop_time = get_time();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    spdlog::info("[LIDAR_2D] merge_loop stop");
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
    const double min_angle_tan = std::tan(LIDAR_2D_INFO::scan_shadow_min_angle*D2R);
    const double max_angle_tan = std::tan(LIDAR_2D_INFO::scan_shadow_max_angle*D2R);

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

void LIDAR_2D::set_config_module(CONFIG* _config)
{
    config = _config;
}

void LIDAR_2D::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void LIDAR_2D::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}
