#include "localization.h"

#include <algorithm>
namespace 
{
    const char* MODULE_NAME = "LOC";
}

LOCALIZATION* LOCALIZATION::instance(QObject* parent)
{
    static LOCALIZATION* inst = nullptr;
    if(!inst && parent)
    {
        inst = new LOCALIZATION(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

LOCALIZATION::LOCALIZATION(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    lidar_2d(nullptr),
    lidar_3d(nullptr),
    cam(nullptr),
    unimap(nullptr),
    obsmap(nullptr)
{
    cur_tf.setIdentity();
}

LOCALIZATION::~LOCALIZATION()
{
    stop();
}

void LOCALIZATION::start()
{
    if(is_loc)
    {
        //printf("[LOCALIZATION] already running\n");
        spdlog::warn("[LOCALIZATION] already running");
        return;
    }

    // set flag
    is_loc = true;

    QString loc_mode = config->get_loc_mode();

    if(config->get_use_ekf() && !config->get_use_sim())
    {
        if(loc_mode == "2D")
        {
            ekf_flag = true;
            ekf_thread = std::make_unique<std::thread>(&LOCALIZATION::ekf_loop, this);
        }
        else if(loc_mode == "3D")
        {
            ekf_flag = true;
            ekf_thread = std::make_unique<std::thread>(&LOCALIZATION::ekf_loop_3d, this);
        }
    }
    else
    {
        if(loc_mode == "2D")
        {
            localization_flag = true;
            localization_thread = std::make_unique<std::thread>(&LOCALIZATION::localization_loop_2d, this);
        }
        else if(loc_mode == "3D")
        {
            localization_flag = true;
            localization_thread = std::make_unique<std::thread>(&LOCALIZATION::localization_loop_3d, this);
        }
        else
        {
            //printf("[LOCALIZATION] unknown LOC_MODE: %s\n", loc_mode.toStdString().c_str());
            spdlog::error("[LOCALIZATION] unknown LOC_MODE: {}", loc_mode.toStdString().c_str());
            is_loc = false;
        }

        odometry_flag = true;
        odometry_thread = std::make_unique<std::thread>(&LOCALIZATION::odometry_loop, this);
    }

    obs_flag = true;
    obs_thread = std::make_unique<std::thread>(&LOCALIZATION::obs_loop, this);

    //printf("[LOCALIZATION(%s)] start\n", loc_mode.toStdString().c_str());
    spdlog::info("[LOCALIZATION({})] start", loc_mode.toStdString().c_str());
}

void LOCALIZATION::stop()
{
    is_loc = false;

    localization_flag = false;
    if(localization_thread && localization_thread->joinable())
    {
        localization_thread->join();
    }
    localization_thread.reset();

    ekf_flag = false;
    if(ekf_thread && ekf_thread->joinable())
    {
        ekf_thread->join();
    }
    ekf_thread.reset();

    odometry_flag = false;
    if(odometry_thread && odometry_thread->joinable())
    {
        odometry_thread->join();
    }
    odometry_thread.reset();

    obs_flag = false;
    if(obs_thread && obs_thread->joinable())
    {
        obs_thread->join();
    }
    obs_thread.reset();

    ekf.reset();
    ekf_3d.reset();

    QString loc_mode = config->get_loc_mode();
    //printf("[LOCALIZATION(%s)] stop\n", loc_mode.toStdString().c_str());
    spdlog::info("[LOCALIZATION({})] stop", loc_mode.toStdString().c_str());
}

void LOCALIZATION::set_cur_tf(Eigen::Matrix4d tf)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_tf = tf;
}

Eigen::Matrix4d LOCALIZATION::get_cur_tf()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Matrix4d res = cur_tf;
    return res;
}

Eigen::Matrix4d LOCALIZATION::get_best_tf(double t)
{
    std::vector<TIME_POSE> _tp_storage;
    Eigen::Matrix4d _cur_tf;
    {
        std::lock_guard<std::mutex> lock(mtx);
        _tp_storage = tp_storage;
        _cur_tf = cur_tf;
    }

    if(_tp_storage.size() == 0)
    {
        return _cur_tf;
    }

    int min_idx = 0;
    double min_dt = std::numeric_limits<double>::max();
    for(size_t p = 0; p < _tp_storage.size(); p++)
    {
        double dt = std::abs(_tp_storage[p].t - t);
        if(dt < min_dt)
        {
            min_dt = dt;
            min_idx = p;
        }
    }

    if(min_dt > 1.0)
    {
        return _cur_tf;
    }
    else
    {
        return _tp_storage[min_idx].tf;
    }
}

std::vector<Eigen::Vector3d> LOCALIZATION::get_cur_global_scan()
{
    std::lock_guard<std::mutex> lock(mtx);
    std::vector<Eigen::Vector3d> res = cur_global_scan;
    return res;
}

Eigen::Vector2d LOCALIZATION::get_cur_ieir()
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Vector2d res = cur_ieir;
    return res;
}

QString LOCALIZATION::get_cur_loc_state()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_loc_state;
    return res;
}

void LOCALIZATION::set_cur_loc_state(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_loc_state = str;
}

void LOCALIZATION::set_cur_ieir(Eigen::Vector2d ieir)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_ieir = ieir;
}

QString LOCALIZATION::get_info_text()
{
    Eigen::Vector3d cur_xi = TF_to_se2(get_cur_tf());
    Eigen::Vector2d cur_ieir = get_cur_ieir();
    Eigen::Matrix4d  cur_tf = get_cur_tf();

    Eigen::Vector3d translation = cur_tf.block<3,1>(0,3);  // translation
    double yaw_deg = atan2(cur_tf(1,0), cur_tf(0,0)) * 180.0 / M_PI;  // rotation

    QString loc_state = get_cur_loc_state();
    //    QString str = QString("[LOC]\npos:%1,%2,%3\nerr:%4 %\nloc_t:%5,%6,%7,%8\nieir:(%9,%10)\nloc_state:%11")

    QString str = QString("[LOC]\npos:%1,%2,%3\nloc_t:%4,%5,%6,%7\nieir:(%8,%9)\nloc_state:%10")
            .arg(cur_xi[0], 0, 'f', 2)            // %1
            .arg(cur_xi[1], 0, 'f', 2)            // %2
            .arg(cur_xi[2], 0, 'f', 2)            // %3
            //            .arg(cur_tf_err * 100, 0, 'f', 2)     // %4
            .arg(translation[0], 0, 'f', 2)       // %5
            .arg(translation[1], 0, 'f', 2)       // %6
            .arg(translation[2], 0, 'f', 2)       // %7
            .arg(yaw_deg, 0, 'f', 2)              // %8
            .arg(cur_ieir[0], 0, 'f', 2)          // %9
            .arg(cur_ieir[1], 0, 'f', 2)          // %10
            .arg(loc_state);                      // %11
    return str;
}

bool LOCALIZATION::get_is_loc()
{
    return (bool)is_loc.load();
}

bool LOCALIZATION::get_is_busy()
{
    return (bool)is_busy.load();
}

double LOCALIZATION::get_process_time_localization()
{
    return (double)process_time_localization.load();
}

double LOCALIZATION::get_process_time_odometry()
{
    return (double)process_time_odometry.load();
}

double LOCALIZATION::get_process_time_obs()
{
    return (double)process_time_obs.load();
}

void LOCALIZATION::start_semiauto_init()
{
    if(is_busy)
    {
        spdlog::warn("[SEMIAUTO INIT] busy");
        return;
    }

    is_busy = true;

    QString loc_mode = config->get_loc_mode();
    if(loc_mode != "2D" && loc_mode != "3D")
    {
        is_busy = false;
        spdlog::error("[SEMIAUTO INIT] unknown LOC_MODE: {}", loc_mode.toStdString().c_str());
        return;
    }

    if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
    {
        is_busy = false;
        spdlog::error("[SEMIAUTO INIT] map not loaded");
        return;
    }

    const int timeout_cnt = 30;
    int wait_cnt = 0;

    if(loc_mode == "2D")
    {
        auto kdtree_index = unimap->get_kdtree_cloud_index();
        auto kdtree_cloud = unimap->get_kdtree_cloud();
        if(!kdtree_cloud || !kdtree_index || kdtree_cloud->pts.empty())
        {
            is_busy = false;
            spdlog::error("[SEMIAUTO INIT] map kdtree not ready");
            return;
        }

        FRAME frm = lidar_2d->get_cur_frm();
        while(frm.pts.empty() && ++wait_cnt <= timeout_cnt)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            frm = lidar_2d->get_cur_frm();
        }

        if(frm.pts.empty())
        {
            is_busy = false;
            spdlog::warn("[SEMIAUTO INIT] no frame data");
            return;
        }

        std::vector<QString> prefix_list = {"CHARGING"};
        std::vector<int> idxs = unimap->get_init_candidates(prefix_list);
        if(idxs.empty())
        {
            is_busy = false;
            spdlog::warn("[SEMIAUTO INIT] no init candidates");
            return;
        }

        Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
        double min_cost = std::numeric_limits<double>::max();
        Eigen::Vector2d min_ieir(1.0, 0.0);
        double min_err_valid = std::numeric_limits<double>::max();
        Eigen::Matrix4d best_candidate_tf = Eigen::Matrix4d::Identity();
        Eigen::Vector2d best_candidate_ieir(1.0, 0.0);
        double best_candidate_err = std::numeric_limits<double>::max();

        auto nodes = unimap->get_nodes_origin();
        for(int i : idxs)
        {
            Eigen::Matrix4d tf = (*nodes)[i].tf;
            for(double th = -180.0; th < 180.0; th += 20.0)
            {
                Eigen::Matrix4d _tf = tf * se2_to_TF({0, 0, th*D2R});
                double err = map_icp(*kdtree_index, *kdtree_cloud, frm, _tf);
                Eigen::Vector2d ieir = calc_ieir(*kdtree_index, frm, _tf);

                if(err < best_candidate_err)
                {
                    best_candidate_err = err;
                    best_candidate_tf = _tf;
                    best_candidate_ieir = ieir;
                }

                if(err < config->get_loc_2d_icp_error_threshold())
                {
                    double cost = (ieir[0]/config->get_loc_2d_icp_error_threshold()) + (1.0 - ieir[1]);
                    if(cost < min_cost)
                    {
                        min_cost = cost;
                        min_ieir = ieir;
                        min_tf = _tf;
                        min_err_valid = err;
                        auto pose = TF_to_se2(min_tf);
                        //printf("[AUTOINIT] x:%f, y:%f, th:%f, cost:%f\n", pose[0], pose[1], pose[2]*R2D, cost);
                        spdlog::info("[AUTOINIT] x:{}, y:{}, th:{}, cost:{}", pose[0], pose[1], pose[2]*R2D, cost);
                    }
                }
            }
        }

        if(min_cost == std::numeric_limits<double>::max())
        {
            min_tf = best_candidate_tf;
            min_ieir = best_candidate_ieir;
            min_err_valid = best_candidate_err;
        }

        const double check_ie = config->get_loc_2d_check_inlier_error();
        const double check_ir = config->get_loc_2d_check_inlier_ratio();
        const bool enable_rotation_fallback = config->get_loc_2d_use_rotation_fallback();

        auto run_rotation_fallback = [&](const Eigen::Matrix4d& base_tf,
                                         Eigen::Matrix4d& out_tf,
                                         Eigen::Vector2d& out_ieir,
                                         double& out_err)->bool
        {
            double step_deg = config->get_loc_2d_rotation_fallback_step();
            if(step_deg <= 0.0)
            {
                step_deg = 10.0;
            }
            double range_deg = config->get_loc_2d_rotation_fallback_range();
            if(range_deg <= 0.0)
            {
                range_deg = 180.0;
            }
            range_deg = std::min(range_deg, 180.0);

            const double err_threshold = config->get_loc_2d_icp_error_threshold();

            bool found = false;
            double best_cost = std::numeric_limits<double>::max();
            Eigen::Vector3d base_xi = TF_to_se2(base_tf);

            for(double deg = -range_deg; deg <= range_deg; deg += step_deg)
            {
                Eigen::Vector3d xi = base_xi;
                double yaw = xi[2] + deg * D2R;
                yaw = deltaRad(yaw, 0.0);
                xi[2] = yaw;
                Eigen::Matrix4d candidate_tf = se2_to_TF(xi);

                double err = map_icp(*kdtree_index, *kdtree_cloud, frm, candidate_tf);
                if(err >= err_threshold)
                {
                    continue;
                }

                Eigen::Vector2d ieir_val = calc_ieir(*kdtree_index, frm, candidate_tf);
                if(ieir_val[0] >= check_ie || ieir_val[1] <= check_ir)
                {
                    continue;
                }

                double cost = (ieir_val[0]/err_threshold) + (1.0 - ieir_val[1]);
                if(cost < best_cost)
                {
                    best_cost = cost;
                    out_tf = candidate_tf;
                    out_ieir = ieir_val;
                    out_err = err;
                    found = true;
                }
            }
            return found;
        };

        bool success = false;
        Eigen::Matrix4d selected_tf = min_tf;
        Eigen::Vector2d selected_ieir = calc_ieir(frm.pts, min_tf);
        double selected_err = min_err_valid;

        if(min_cost < std::numeric_limits<double>::max())
        {
            if(selected_ieir[0] < check_ie && selected_ieir[1] > check_ir)
            {
                success = true;
            }
        }
        else
        {
            selected_ieir = best_candidate_ieir;
            selected_err = best_candidate_err;
        }

        if(!success && enable_rotation_fallback)
        {
            Eigen::Matrix4d fallback_tf;
            Eigen::Vector2d fallback_ieir(1.0, 0.0);
            double fallback_err = std::numeric_limits<double>::max();

            auto attempt_rotation = [&](const Eigen::Matrix4d& base_tf)->bool
            {
                Eigen::Matrix4d tmp_tf;
                Eigen::Vector2d tmp_ieir(1.0, 0.0);
                double tmp_err = std::numeric_limits<double>::max();
                bool ok = run_rotation_fallback(base_tf, tmp_tf, tmp_ieir, tmp_err);
                if(ok)
                {
                    fallback_tf = tmp_tf;
                    fallback_ieir = tmp_ieir;
                    fallback_err = tmp_err;
                }
                return ok;
            };

            Eigen::Matrix4d base_tf = (min_cost < std::numeric_limits<double>::max()) ? min_tf : best_candidate_tf;
            bool fallback_found = attempt_rotation(base_tf);

            if(!fallback_found)
            {
                for(int idx : idxs)
                {
                    if(attempt_rotation((*nodes)[idx].tf))
                    {
                        fallback_found = true;
                        break;
                    }
                }
            }

            if(fallback_found)
            {
                selected_tf = fallback_tf;
                selected_ieir = calc_ieir(frm.pts, fallback_tf);
                selected_err = fallback_err;
                success = true;
                spdlog::info("[AUTOINIT] rotation fallback selected. err:{}, ie:{}, ir:{}",
                             selected_err, fallback_ieir[0], fallback_ieir[1]);
            }
            else
            {
                spdlog::warn("[SEMIAUTO INIT] rotation fallback could not find valid pose");
            }
        }

        if(success)
        {
            set_cur_tf(selected_tf);
            start();
            spdlog::info("[AUTOINIT] success auto init. ieir: {}, {}", selected_ieir[0], selected_ieir[1]);
        }
    }
    else // "3D"
    {
        TIME_PTS cur_frm = lidar_3d->get_cur_frm();
        while(cur_frm.pts.empty() && ++wait_cnt <= timeout_cnt)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            cur_frm = lidar_3d->get_cur_frm();
        }
        if(cur_frm.pts.empty())
        {
            is_busy = false;
            spdlog::warn("[SEMIAUTO INIT] no frame data");
            return;
        }

        std::vector<QString> prefix_list = {config->get_robot_serial_number(), "CHARGING"};
        std::vector<int> idxs = unimap->get_init_candidates(prefix_list);
        if(idxs.empty())
        {
            is_busy = false;
            spdlog::warn("[SEMIAUTO INIT] no init candidates");
            return;
        }

        auto nodes = unimap->get_nodes_origin();
        Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
        double min_err = std::numeric_limits<double>::max();
        for(int i : idxs)
        {
            Eigen::Matrix4d tf = (*nodes)[i].tf;
            for(double th = -180.0; th < 180.0; th += 90.0)
            {
                Eigen::Matrix4d _tf = tf * se2_to_TF({0, 0, th*D2R});
                double err = map_icp(cur_frm.pts, _tf);
                if(err < min_err)
                {
                    min_err = err;
                    min_tf = _tf;
                    auto xi = TF_to_se2(min_tf);
                    //printf("[AUTOINIT] x:%f, y:%f, th:%f, cost:%f\n", xi[0], xi[1], xi[2]*R2D, err);
                    spdlog::info("[AUTOINIT] x:{}, y:{}, th:{}, cost:{}", xi[0], xi[1], xi[2]*R2D, err);
                }
            }
        }

        auto ieir = calc_ieir(cur_frm.pts, min_tf);
        if(ieir[0] < config->get_loc_2d_check_inlier_error() && ieir[1] > config->get_loc_2d_check_inlier_ratio())
        {
            set_cur_tf(min_tf);
            start();
            //printf("[AUTOINIT] success auto init. ieir: %f, %f\n", ieir[0], ieir[1]);
            spdlog::info("[AUTOINIT] success auto init. ieir: {}, {}", ieir[0], ieir[1]);
        }
    }

    is_busy = false;
    spdlog::info("[SEMIAUTO INIT] finished");
}

Eigen::Vector2d LOCALIZATION::calc_ieir(const std::vector<Eigen::Vector3d>& pts, const Eigen::Matrix4d& G)
{
    if(pts.size() < 100)
    {
        return Eigen::Vector2d(1.0, 0);
    }

    const double surfel_range = config->get_loc_2d_surfel_range();
    const double inlier_check_dist = config->get_loc_2d_check_dist();

    Eigen::Matrix3d cur_R = G.block(0,0,3,3);
    Eigen::Vector3d cur_t = G.block(0,3,3,1);

    std::shared_ptr<vector<Eigen::Vector3d>> nn_pts = unimap->get_map_3d_pts();

    double err_sum = 0;
    int cnt = 0;
    int cnt_all = 0;
    for(size_t p = 0; p < pts.size(); p += 8)
    {
        cnt_all++;

        Eigen::Vector3d P = pts[p];
        Eigen::Vector3d _P = cur_R*P + cur_t;
        std::vector<int> nn_idxs = unimap->knn_search_idx(_P, 1, surfel_range);
        if(nn_idxs.size() == 0)
        {
            continue;
        }

        Eigen::Vector3d nn_pt = (*nn_pts)[nn_idxs[0]];
        double d = (nn_pt - _P).norm();
        if(d < inlier_check_dist)
        {
            err_sum += d;
            cnt++;
        }
    }

    if(cnt == 0 || cnt_all == 0)
    {
        return Eigen::Vector2d(1.0, 0);
    }

    Eigen::Vector2d res;
    res[0] = err_sum/cnt;
    res[1] = (double)cnt/cnt_all;
    return res;
}

Eigen::Vector2d LOCALIZATION::calc_ieir(KD_TREE_XYZR& tree, FRAME& frm, Eigen::Matrix4d& G)
{
    const double check_dist = config->get_loc_2d_check_dist();

    int err_num = 0;
    double err_sum = 0;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        Eigen::Vector3d P = frm.pts[p];
        Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);

        // find nn
        std::vector<unsigned int> ret_near_idxs(1);
        std::vector<double> ret_near_sq_dists(1);

        double near_query_pt[3] = {_P[0], _P[1], _P[2]};
        tree.knnSearch(&near_query_pt[0], 1, &ret_near_idxs[0], &ret_near_sq_dists[0]);

        // point to point distance
        double err = std::sqrt(ret_near_sq_dists[0]);
        if(err > check_dist)
        {
            continue;
        }

        err_sum += err;
        err_num++;
    }

    if(err_num == 0)
    {
        Eigen::Vector2d res;
        res[0] = 1.0;
        res[1] = 0.0;
        return res;
    }
    else
    {
        Eigen::Vector2d res;
        res[0] = err_sum/err_num; // inlier error
        res[1] = (double)err_num/frm.pts.size(); // inlier ratio
        return res;
    }
}

void LOCALIZATION::localization_loop_2d()
{
    lidar_2d->clear_merged_queue();

    //printf("[LOCALIZATION(2D)] a_loop start\n");
    spdlog::info("[LOCALIZATION(2D)] a_loop start");
    while(localization_flag)
    {
        FRAME frm;
        if(lidar_2d->try_pop_merged_queue(frm))
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            double st_time = get_time();

            // initial guess
            Eigen::Matrix4d G = get_cur_tf();

            auto kdtree_index = unimap->get_kdtree_cloud_index();
            auto kdtree_cloud = unimap->get_kdtree_cloud();

            // icp
            double err = map_icp((*kdtree_index), (*kdtree_cloud), frm, G);

            // check ieir
            cur_ieir = calc_ieir((*kdtree_index), frm, G);

            // check error
            if(err < config->get_loc_2d_icp_error_threshold())
            {
                // for loc b loop
                TIME_POSE tp;
                tp.t = frm.t;
                tp.tf = G;
                tp_que.push(tp);

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = G;
                tpp.pts = frm.pts;
                tpp_que.push(tpp);

                // update
                set_cur_tf(G);

                // local to global deskewed point
                std::vector<Eigen::Vector3d> pts(frm.pts.size());
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Vector3d P(frm.pts[p][0], frm.pts[p][1], frm.pts[p][2]);
                    Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                    pts[p] = _P;
                }

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_global_scan = pts;
                }

                // for next
                // _cur_tf = G;
            }

            // set localization info for plot
            cur_tf_err = err;
            process_time_localization = get_time() - st_time;

            // for speed
            lidar_2d->clear_merged_queue();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //printf("[LOCALIZATION(2D)] a_loop stop\n");
    spdlog::info("[LOCALIZATION(2D)] a_loop stop");
}

void LOCALIZATION::localization_loop_3d()
{
    IMU pre_imu;
    Eigen::Matrix4d _cur_tf = get_cur_tf();
    // Eigen::Matrix4d _pre_tf = _cur_tf;

    double pre_loop_time = get_time();

    // lidar localization (10hz)
    lidar_3d->clear_merged_queue();

    //printf("[LOCALIZATION(3D)] a_loop start\n");
    spdlog::info("[LOCALIZATION(3D)] a_loop start");
    while(localization_flag)
    {
        TIME_PTS frm;
        if(lidar_3d->try_pop_merged_queue(frm))
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            // initial guess
            // Eigen::Vector3d delta_xi = TF_to_se2(_pre_tf.inverse()*_cur_tf);
            // Eigen::Matrix4d G = _cur_tf*ZYX_to_TF(delta_xi[0], delta_xi[1], 0, 0, 0, delta_xi[2]);
            // G = _cur_tf;
            Eigen::Matrix4d G = _cur_tf;

            // imu
            IMU cur_imu = lidar_3d->get_best_imu(frm.t, 0);
            if(pre_imu.t != 0)
            {
                Eigen::Matrix3d R0 = Sophus::SO3d::exp(Sophus::Vector3d(pre_imu.rx, pre_imu.ry, pre_imu.rz)).matrix();
                Eigen::Matrix3d R1 = Sophus::SO3d::exp(Sophus::Vector3d(cur_imu.rx, cur_imu.ry, cur_imu.rz)).matrix();
                G.block(0,0,3,3) = G.block(0,0,3,3)*(R0.inverse()*R1);
            }

            // icp
            std::vector<Eigen::Vector3d> dsk = frm.pts;
            double err = map_icp(dsk, G);

            // check ieir
            cur_ieir = calc_ieir(dsk, G);

            // check error
            if(err < config->get_loc_2d_icp_error_threshold())
            {
                // for loc b loop
                TIME_POSE tp;
                tp.t = frm.t;
                tp.tf = G;
                tp_que.push(tp);

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = G;
                tpp.pts = dsk;
                //tpp_que.push(tpp);

                // update
                set_cur_tf(G);

                // local to global deskewed point
                std::vector<Eigen::Vector3d> pts(dsk.size());
                for(size_t p = 0; p < dsk.size(); p++)
                {
                    Eigen::Vector3d P(dsk[p][0], dsk[p][1], dsk[p][2]);
                    Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                    pts[p] = _P;
                }

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_global_scan = pts;
                }

                // for next
                // _pre_tf = _cur_tf;
                _cur_tf = G;
                pre_imu = cur_imu;
            }

            // set localization info for plot
            cur_tf_err = err;

            double cur_loop_time = get_time();
            process_time_localization = cur_loop_time - pre_loop_time;
            pre_loop_time = cur_loop_time;

            // for speed
            lidar_3d->clear_merged_queue();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    //printf("[LOCALIZATION(3D)] a_loop stop\n");
    spdlog::info("[LOCALIZATION(3D)] a_loop stop");
}

void LOCALIZATION::odometry_loop()
{
    // loop params
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    // clear storage
    {
        std::lock_guard<std::mutex> lock(mtx);
        tp_storage.clear();
    }

    // params
    MOBILE_POSE pre_mo = mobile->get_pose();


    //printf("[LOCALIZATION] odometry loop start\n");
    //qDebug()<<"odometry_flag : "<<odometry_flag;

    spdlog::info("[LOCALTIZATION] odometry loop start, odometryty_flag: {}", odometry_flag.load());

    while(odometry_flag)
    {
        MOBILE_POSE cur_mo = mobile->get_pose();

        if(cur_mo.t > pre_mo.t && config->get_loc_2d_icp_odometry_fusion_ratio() != 0.0)
        {
            Eigen::Matrix4d pre_mo_tf = se2_to_TF(pre_mo.pose);
            Eigen::Matrix4d cur_mo_tf = se2_to_TF(cur_mo.pose);

            // get current tf
            Eigen::Matrix4d _cur_tf = get_cur_tf();

            // update odo_tf
            Eigen::Matrix4d delta_tf = pre_mo_tf.inverse()*cur_mo_tf;
            Eigen::Matrix4d odo_tf = _cur_tf*delta_tf;
            pre_mo_tf = cur_mo_tf;

            // icp-odometry fusion
            TIME_POSE tp;
            if(tp_que.try_pop(tp) && !config->get_use_sim())
            {
                // time delay compensation
                Eigen::Matrix4d tf0 = se2_to_TF(mobile->get_best_mo(tp.t).pose);
                Eigen::Matrix4d tf1 = se2_to_TF(cur_mo.pose);
                Eigen::Matrix4d mo_dtf = tf0.inverse()*tf1;
                Eigen::Matrix4d icp_tf = tp.tf * mo_dtf;

                double alpha = config->get_loc_2d_icp_odometry_fusion_ratio(); // 1.0 means odo_tf 100%

                // for odometry slip
                Eigen::Vector2d dtdr = dTdR(odo_tf, icp_tf);
                if(std::abs(dtdr[1]) > 10.0*D2R)
                {
                    alpha = 0;
                    //printf("[LOCALIZATION] slip detection, alpha set 0, dth: %f\n", dtdr[1]*R2D);
                    spdlog::warn("[LOCALIZATION] slip detection, alpha set 0, dth: {}", dtdr[1]*R2D);
                }

                // interpolation
                Eigen::Matrix4d dtf = icp_tf.inverse()*odo_tf;
                Eigen::Matrix4d fused_tf = icp_tf*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                // update
                _cur_tf = fused_tf;
            }
            else
            {
                // update
                _cur_tf = odo_tf;
            }

            /*
            // aruco fusion
            TIME_POSE_ID aruco_tpi = aruco->get_cur_tpi();
            if(aruco_tpi.t > pre_aruco_t && config->USE_SIM == 0)
            {
                double d = calc_dist_2d(aruco_tpi.tf.block(0,3,3,1));
                if(d < config->LOC_ARUCO_ODO_FUSION_DIST)
                {
                    NODE* node = unimap->get_node_by_name(QString::number(aruco_tpi.id, 10));
                    if(node != NULL)
                    {
                        // parse tf
                        Eigen::Matrix4d T_g_m0 = node->tf; // stored global marker tf
                        Eigen::Matrix4d T_m_r = aruco_tpi.tf.inverse();

                        // time delay compensation
                        Eigen::Matrix4d tf0 = se2_to_TF(mobile->get_best_mo(aruco_tpi.t).pose);
                        Eigen::Matrix4d tf1 = se2_to_TF(cur_mo.pose);
                        Eigen::Matrix4d mo_dtf = tf0.inverse()*tf1;

                        Eigen::Matrix4d T_g_r = T_g_m0*T_m_r*mo_dtf;
                        Eigen::Matrix4d aruco_tf = se2_to_TF(TF_to_se2(T_g_r));

                        // interpolation
                        double alpha = config->LOC_ARUCO_ODO_FUSION_RATIO; // 1.0 mean odometry only
                        if(std::abs(cur_mo.vel[2]) > 10.0*D2R)
                        {
                            alpha = 1.0; // odometry only
                        }

                        Eigen::Matrix4d dtf = aruco_tf.inverse()*_cur_tf;
                        Eigen::Matrix4d fused_tf = aruco_tf*intp_tf(alpha, Eigen::Matrix4d::Identity(), dtf);

                        // update
                        _cur_tf = fused_tf;
                        pre_aruco_t = aruco_tpi.t;
                    }
                }
            }
            */

            // update
            set_cur_tf(_cur_tf);
            {
                std::lock_guard<std::mutex> lock(mtx);
                TIME_POSE fused_tp;
                fused_tp.t = cur_mo.t;
                fused_tp.tf = _cur_tf;
                tp_storage.push_back(fused_tp);
                if(tp_storage.size() > 300)
                {
                    tp_storage.erase(tp_storage.begin());
                }
            }

            // for next operation
            pre_mo = cur_mo;
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            //printf("[LOCALIZATION] odometry loop time drift, dt:%f\n", delta_loop_time);
        }

        // update processing time
        process_time_odometry = delta_loop_time;
        pre_loop_time = cur_loop_time;
    }
    //printf("[LOCALIZATION] odometry loop stop\n");
    spdlog::info("[LOCALIZATION] odometry loop stop");
}

void LOCALIZATION::ekf_loop()
{
    // loop params
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    lidar_2d->clear_merged_queue();

    //printf("[LOCALIZATION] ekf_loop start\n");
    spdlog::info("[LOCALIZATION] ekf_loop start");
    while(ekf_flag)
    {
        MOBILE_POSE cur_mo = mobile->get_pose();

        if(ekf.initialized.load())
        {
            ekf.predict(se2_to_TF(cur_mo.pose));
        }

        Eigen::Matrix4d G = ekf.initialized.load() ? ekf.get_cur_tf() : get_cur_tf();

        FRAME frm;
        if(lidar_2d->try_pop_merged_queue(frm))
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            double st_time = get_time();

            auto kdtree_index = unimap->get_kdtree_cloud_index();
            auto kdtree_cloud = unimap->get_kdtree_cloud();

            double err = map_icp((*kdtree_index), (*kdtree_cloud), frm, G);

            // check ieir
            cur_ieir = calc_ieir((*kdtree_index), frm, G);

            if(err < config->get_loc_2d_icp_error_threshold())
            {
                if(!ekf.initialized.load())
                {
                    ekf.init(G);
                }
                else
                {
                    ekf.estimate(G, cur_ieir);
                }
                G = ekf.get_cur_tf();

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = G;
                tpp.pts = frm.pts;
                tpp_que.push(tpp);

                // local to global deskewed point
                std::vector<Eigen::Vector3d> pts(frm.pts.size());
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Vector3d P(frm.pts[p][0], frm.pts[p][1], frm.pts[p][2]);
                    Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                    pts[p] = _P;
                }

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_global_scan = pts;
                }
            }

            // set localization info for plot
            cur_tf_err = err;

            // for speed
            lidar_2d->clear_merged_queue();
        }

        // update
        set_cur_tf(G);

        // set localization info for plot
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            //printf("[LOCALIZATION] odometry loop time drift, dt:%f\n", delta_loop_time);
        }

        process_time_localization = cur_loop_time - pre_loop_time;
        pre_loop_time = cur_loop_time;
    }
    //printf("[LOCALIZATION] ekf loop stop\n");
    spdlog::info("[LOCALIZATION] ekf loop stop");
}

void LOCALIZATION::ekf_loop_3d()
{
    lidar_3d->clear_merged_queue();

    // loop params
    const double dt = 0.02; // 50hz
    double pre_loop_time = get_time();

    //printf("[LOCALIZATION] ekf_loop_3d start\n");
    spdlog::info("[LOCALIZATION] ekf_loop_3d start");
    while(ekf_flag)
    {
        MOBILE_POSE cur_mo = mobile->get_pose();

        if(ekf_3d.initialized.load())
        {
            ekf_3d.predict(se2_to_TF(cur_mo.pose));
        }

        Eigen::Matrix4d G = ekf_3d.initialized.load() ? ekf_3d.get_cur_tf() : get_cur_tf();

        TIME_PTS frm;
        if(lidar_3d->try_pop_merged_queue(frm))
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }

            auto kdtree_index = unimap->get_kdtree_cloud_index();
            auto kdtree_cloud = unimap->get_kdtree_cloud();

            // icp
            std::vector<Eigen::Vector3d> dsk = frm.pts;
            double err = map_icp(dsk, G);

            // check ieir
            cur_ieir = calc_ieir(dsk, G);

            if(err < config->get_loc_2d_icp_error_threshold())
            {
                if(!ekf_3d.initialized.load())
                {
                    ekf_3d.init(G);
                }
                else
                {
                    ekf_3d.estimate(G, cur_ieir);
                }
                G = ekf_3d.get_cur_tf();

                // for obs loop
                TIME_POSE_PTS tpp;
                tpp.t = frm.t;
                tpp.tf = G;
                tpp.pts = frm.pts;
                tpp_que.push(tpp);

                // local to global deskewed point
                std::vector<Eigen::Vector3d> pts(frm.pts.size());
                for(size_t p = 0; p < frm.pts.size(); p++)
                {
                    Eigen::Vector3d P(frm.pts[p][0], frm.pts[p][1], frm.pts[p][2]);
                    Eigen::Vector3d _P = G.block(0,0,3,3)*P + G.block(0,3,3,1);
                    pts[p] = _P;
                }

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_global_scan = pts;
                }
            }

            // for speed
            lidar_3d->clear_merged_queue();
        }

        // update
        set_cur_tf(G);

        // set localization info for plot
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            //printf("[LOCALIZATION] odometry loop time drift, dt:%f\n", delta_loop_time);
        }

        process_time_localization = cur_loop_time - pre_loop_time;
        pre_loop_time = cur_loop_time;
    }
    //printf("[LOCALIZATION] ekf_loop_3d stop\n");
    spdlog::info("[LOCALIZATION] ekf_loop_3d stop");
}

void LOCALIZATION::obs_loop()
{
    //printf("[LOCALIZATION] obs_loop start\n");
    spdlog::info("[LOCALIZATION] obs_loop start");
    double pre_loop_time = get_time();

    //printf("[LOCALIZATION] get_obs_map_max_z: %.3f, get_obs_map_min_z: %.3f, get_obs_map_range: %.3f\n", config->get_obs_map_max_z(),config->get_obs_map_min_z(),config->get_obs_map_range());
    spdlog::info("[LOCALIZATION] get_obs_map_max_z: {}, get_obs_map_min_z: {}, get_obs_map_range: {}", config->get_obs_map_max_z(),config->get_obs_map_min_z(),config->get_obs_map_range());

    if(obsmap)
    {
        //printf("[LOCALIZATION] total_get_obs_box_max_z: %.3f, total_get_obs_box_map_min_z: %.3f, total_get_obs_box_map_range: %.3f\n", obsmap->get_obs_box_map_max_z(),obsmap->get_obs_box_map_min_z(),obsmap->get_obs_box_map_range());
        spdlog::info("[LOCALIZATION] total_get_obs_box_max_z: {}, total_get_obs_box_map_min_z: {}, total_get_obs_box_map_range: {}", obsmap->get_obs_box_map_max_z(),obsmap->get_obs_box_map_min_z(),obsmap->get_obs_box_map_range());
    }

    std::vector<Eigen::Vector3d> temp_pts;
    temp_pts.reserve(10000);

    while(obs_flag)
    {
        if(config->get_use_sim())
        {
            if(unimap->get_is_loaded() == MAP_LOADED)
            {
                Eigen::Matrix4d _cur_tf = get_cur_tf();
                obsmap->update_obs_map_sim(_cur_tf);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        Eigen::Matrix4d _cur_tf = get_cur_tf();
        if(config->get_use_lidar_2d())
        {
            FRAME scan = lidar_2d->get_cur_frm();
            if(!scan.pts.empty())
            {
                Eigen::Matrix4d tf0 = _cur_tf.inverse() * get_best_tf(scan.t);
                Eigen::Matrix3d R0 = tf0.block(0,0,3,3);
                Eigen::Vector3d t0 = tf0.block(0,3,3,1);

                std::vector<Eigen::Vector3d> transformed_pts(scan.pts.size());
                #pragma omp parallel for
                for(size_t i = 0; i < scan.pts.size(); i++)
                {
                    transformed_pts[i] = R0 * scan.pts[i] + t0;
                }

                temp_pts.insert(temp_pts.end(), transformed_pts.begin(), transformed_pts.end());
            }
        }

        if(config->get_use_lidar_3d())
        {
            TIME_PTS scan = lidar_3d->get_cur_frm();
            if(!scan.pts.empty())
            {
                Eigen::Matrix4d tf0 = _cur_tf.inverse() * get_best_tf(scan.t);
                Eigen::Matrix3d R0 = tf0.block(0,0,3,3);
                Eigen::Vector3d t0 = tf0.block(0,3,3,1);

                std::vector<Eigen::Vector3d> transformed_pts(scan.pts.size());
                #pragma omp parallel for
                for(size_t i = 0; i < scan.pts.size(); i++)
                {
                    transformed_pts[i] = R0 * scan.pts[i] + t0;
                }

                temp_pts.insert(temp_pts.end(), transformed_pts.begin(), transformed_pts.end());
            }
        }

        if(config->get_use_cam())
        {
            for(int i = 0; i < config->get_cam_num(); ++i)
            {
                TIME_PTS scan = cam->get_scan(i);
                if(!scan.pts.empty())
                {
                    Eigen::Matrix4d tf0 = _cur_tf.inverse() * get_best_tf(scan.t);
                    Eigen::Matrix3d R0 = tf0.block(0,0,3,3);
                    Eigen::Vector3d t0 = tf0.block(0,3,3,1);

                    std::vector<Eigen::Vector3d> transformed_pts(scan.pts.size());
                    #pragma omp parallel for
                    for(size_t i = 0; i < scan.pts.size(); i++)
                    {
                        transformed_pts[i] = R0 * scan.pts[i] + t0;
                    }

                    temp_pts.insert(temp_pts.end(), transformed_pts.begin(), transformed_pts.end());
                }
            }
        }

        // std::cout << "temp_pts size: " << temp_pts.size() << std::endl;

        TIME_POSE_PTS tpp;
        tpp.tf = _cur_tf;
        tpp.pts = std::move(temp_pts);

        // update obstacle map
        obsmap->update_obs_map(tpp);

        // update processing time
        process_time_obs = get_time() - pre_loop_time;
        pre_loop_time = get_time();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    //printf("[LOCALIZATION] obs_loop stop\n");
    spdlog::info("[LOCALIZATION] obs_loop stop");
}

// 2D
double LOCALIZATION::map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    std::vector<Eigen::Vector3d> pts;
    for(size_t p = 0; p < frm.pts.size(); p++)
    {
        idx_list.push_back(p);

        Eigen::Vector3d P = G.block(0,0,3,3)*frm.pts[p] + G.block(0,3,3,1);
        pts.push_back(P);
    }

    // solution
    Eigen::Matrix4d _G = Eigen::Matrix4d::Identity();

    // optimization param
    const int max_iter = max_iter0;
    double lambda = lambda0;
    double first_err   = std::numeric_limits<double>::max();
    double last_err    = std::numeric_limits<double>::max();
    double convergence = std::numeric_limits<double>::max();
    int num_correspondence = 0;

    const double cost_threshold0 = config->get_loc_2d_icp_cost_threshold();
    const double cost_threshold = cost_threshold0 * cost_threshold0;
    const int num_feature = std::min<int>(idx_list.size(), config->get_loc_2d_icp_max_feature_num());
    const int pt_surfel_num = config->get_loc_2d_surfel_num();
    const double sq_d_max0 = config->get_loc_2d_surfel_range();

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        // shuffle
        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());

        Eigen::Matrix3d _R0 = _G.block(0,0,3,3);
        Eigen::Vector3d _t0 = _G.block(0,3,3,1);

        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;
        for(size_t p = 0; p < idx_list.size(); p++)
        {
            // get index
            int i = idx_list[p];
            double dist = calc_dist_2d(frm.pts[i]);
            if(dist < config->get_lidar_2d_min_range())
            {
                continue;
            }

            // local to global
            Eigen::Vector3d P1 = pts[i];
            Eigen::Vector3d _P1 = _R0*P1 + _t0;

            // knn points
            Eigen::Vector3d P0(0, 0, 0);
            {
                std::vector<unsigned int> ret_near_idxs(pt_surfel_num);
                std::vector<double> ret_near_sq_dists(pt_surfel_num);

                double near_query_pt[3] = {_P1[0], _P1[1], _P1[2]};
                tree.knnSearch(&near_query_pt[0], pt_surfel_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

                int sq_threshold_cnt = 0;
                int cnt = 0;
                int idx0 = ret_near_idxs[0];
                double sq_d_max = sq_d_max0 * sq_d_max0;
                for(int q = 0; q < pt_surfel_num; q++)
                {
                    int idx = ret_near_idxs[q];
                    double dx = cloud.pts[idx].x - cloud.pts[idx0].x;
                    double dy = cloud.pts[idx].y - cloud.pts[idx0].y;
                    double sq_d = dx*dx + dy*dy;
                    if(sq_d > sq_d_max)
                    {
                        continue;
                    }

                    P0 += Eigen::Vector3d(cloud.pts[idx].x, cloud.pts[idx].y, cloud.pts[idx].z);
                    cnt++;
                }
                P0 /= cnt;
            }

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
            Eigen::Vector3d xi = TF_to_se2(_G);

            double J[3] = {0,};
            J[0] = 2.0 * (_P1[0] - P0[0]);
            J[1] = 2.0 * (_P1[1] - P0[1]);
            J[2] = 2.0 * ((_P1[0] - P0[0]) * (-std::sin(xi[2]) * P1[0] - std::cos(xi[2]) * P1[1]) + (_P1[1] - P0[1]) * (std::cos(xi[2]) * P1[0] - std::sin(xi[2]) * P1[1]));
            if(!isfinite(J[0]) || !isfinite(J[1]) || !isfinite(J[2]))
            {
                continue;
            }

            // additional weight
            double weight = 1.0 + 0.05*dist;

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
            //printf("[map_icp(2D)] not enough correspondences, iter: %d, num: %d->%d!!\n", iter, (int)frm.pts.size(), num_correspondence);
            spdlog::warn("[map_icp(2D)] not enough correspondences, iter: {}, num: {}->{}!!", iter, (int)frm.pts.size(), num_correspondence);
            return std::numeric_limits<double>::max();
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
        if(sigma < sigma_eps)
        {
            sigma = sigma_eps;
        }

        // calc weight
        for (size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = t_dist_v0;
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
            err += std::abs(w*c);
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
            lambda *= lambda_dec;
        }
        else
        {
            lambda *= lambda_inc;
        }
        last_err = err;

        // pose update
        Eigen::Vector3d xi;
        xi[0] = X(0, 0);
        xi[1] = X(1, 0);
        xi[2] = X(2, 0);

        _G = se2_to_TF(xi)*_G;
        refine_pose(_G);

        // for rmt
        tm0 = tm1;
        tm1 = _G.block(0,3,3,1).norm();

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    G = _G*G;

    if(last_err > first_err+0.01 || last_err > config->get_loc_2d_icp_error_threshold())
    {
        //printf("[map_icp] i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);
        spdlog::warn("[map_icp] i:{}, n:{}, e:{}->{}, c:{}, dt:{}", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);

        //return 9999;
    }

    return last_err;
}

// 3D
double LOCALIZATION::map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G)
{
    // for processing time
    double t_st = get_time();

    // for random selection
    std::vector<int> idx_list;
    for(size_t p = 0; p < pts.size(); p++)
    {
        // set idxs
        idx_list.push_back(p);
    }

    // solution
    Eigen::Matrix4d _G = G;

    // optimization param
    const int max_iter = max_iter0;
    double lambda = lambda0;
    double first_err = std::numeric_limits<double>::max();
    double last_err = std::numeric_limits<double>::max();
    double convergence = std::numeric_limits<double>::max();
    int num_correspondence = 0;

    // for rmt
    double tm0 = 0;
    double tm1 = 0;

    const double surfel_range = config->get_loc_2d_surfel_range();
    const double surfel_balance = config->get_loc_3d_surfel_balance();
    const double max_feature_num = config->get_loc_2d_icp_max_feature_num();
    const double cost_threshold = config->get_loc_2d_icp_cost_threshold();

    // loop
    int iter = 0;
    for(iter = 0; iter < max_iter; iter++)
    {
        // shuffle
        std::shuffle(idx_list.begin(), idx_list.end(), std::default_random_engine());
        const int num_feature = std::min<int>(idx_list.size(), max_feature_num);
        std::vector<int> surfel_cnt(3,0);

        // calc cost jacobian
        std::vector<double> costs;
        std::vector<COST_JACOBIAN> cj_set;
        for(size_t k = 0; k < idx_list.size(); k++)
        {
            // get index
            int i = idx_list[k];

            // local to global
            Eigen::Vector3d P = pts[i];
            Eigen::Vector3d P1 = _G.block(0,0,3,3)*P + _G.block(0,3,3,1);

            // calc correspondence
            const int _nn_num = 1; // config->LVX_SURFEL_NN_NUM;
            std::vector<int> nn_idxs = unimap->knn_search_idx(P1, _nn_num, surfel_range);
            if((int)nn_idxs.size() != _nn_num)
            {
                continue;
            }

            // calc surfel
            Eigen::Vector3d N0(0,0,0);
            Eigen::Vector3d P0(0,0,0);
            for(size_t p = 0; p < nn_idxs.size(); p++)
            {
                int idx = nn_idxs[p];
                P0 += unimap->get_pts_3d(idx);
                N0 += unimap->get_normal_3d(idx);
            }

            P0 /= nn_idxs.size();
            N0 /= nn_idxs.size();
            N0.normalize();

            // balance filter
            int axis_idx = get_major_axis(N0);
            if(surfel_cnt[axis_idx] > num_feature*surfel_balance)
            {
                continue;
            }

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

            // calc cost
            double cost = N0.dot(P1-P0);
            if(std::abs(cost) > cost_threshold || std::abs(cost) > rmt*std::abs(cost) + rmt_sigma)
            {
                continue;
            }

            // Compute Jacobian using Lie Algebra
            Eigen::Matrix<double, 3, 6> dP1_dxi;                        // Jacobian of SE(3) transformation
            dP1_dxi.block<3, 3>(0, 0) =  Eigen::Matrix3d::Identity();   // Translation part
            dP1_dxi.block<3, 3>(0, 3) = -Sophus::SO3d::hat(P1);         // Rotation part

            // Cost Jacobian: N0^T * dP1/dxi
            Eigen::Matrix<double, 1, 6> jacobian = N0.transpose() * dP1_dxi;
            if(!jacobian.allFinite())
            {
                continue;
            }

            double J[6];
            J[0]  = jacobian(0,0);
            J[1]  = jacobian(0,1);
            J[2]  = jacobian(0,2);
            J[3]  = jacobian(0,3);
            J[4]  = jacobian(0,4);
            J[5]  = jacobian(0,5);

            // storing cost jacobian
            COST_JACOBIAN cj;
            cj.c = cost;
            cj.w = 1.0;
            memcpy(cj.J, J, sizeof(double)*12);

            cj_set.push_back(cj);
            costs.push_back(cost);

            // update surfel count
            surfel_cnt[axis_idx]++;

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
            //printf("[map_icp(3D)] not enough correspondences, iter: %d, num: %d!!\n", iter, num_correspondence);
            spdlog::warn("[map_icp(3D)] not enough correspondences, iter: {}, num: {}!!", iter, num_correspondence);
            return 9999;
        }

        // calc mu
        std::nth_element(costs.begin(), costs.begin() + costs.size()/2, costs.end());
        double mu = costs[costs.size() / 2];

        // calc sigma
        std::vector<double> vars(costs.size());
        for(size_t p = 0; p < costs.size(); p++)
        {
            vars[p] = std::abs(costs[p] - mu);
        }

        std::nth_element(vars.begin(), vars.begin() + vars.size()/2, vars.end());
        double sigma = std::max(1.4826 * vars[vars.size()/2], sigma_eps);

        // calc weight
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double V0 = t_dist_v0;
            double w = (V0 + 1.0) / (V0 + ((c - mu) / sigma) * ((c - mu) / sigma));
            cj_set[p].w *= std::clamp(w, 0.01, 10.0);
        }

        // make matrix
        double _A[6*6] = { 0, };
        double _b[6] = { 0, };
        double err = 0;
        double err_cnt = 0;
        for(size_t p = 0; p < cj_set.size(); p++)
        {
            double c = cj_set[p].c;
            double* J = cj_set[p].J;
            double w = cj_set[p].w;

            // set tempolar matrix(col major)
            for(int y = 0; y < 6; y++)
            {
                for(int x = 0; x < 6; x++)
                {
                    _A[y * 6 + x] += w * J[y] * J[x];
                }
                _b[y] += w * c * J[y];
            }

            // error
            err += std::abs(c);
            err_cnt += w;
        }
        err /= err_cnt;

        // set first error
        if(iter == 0)
        {
            first_err = err;
        }

        // solve
        Eigen::Matrix<double, 6, 6> A(_A);
        A += 1e-6*Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::Matrix<double, 6, 1> b(_b);
        Eigen::Matrix<double, 6, 6> diag_A = A.diagonal().asDiagonal();
        Eigen::Matrix<double, 6, 1> X = (-(A + lambda * diag_A)).ldlt().solve(b);

        // lambda update
        double err_ratio = (last_err-err)/last_err;
        if(err_ratio > 0.1)
        {
            //lambda *= lambda_dec;
            lambda = std::max(lambda * lambda_dec, 1e-6);
        }
        else if(err_ratio < 0.01)
        {
            //lambda *= lambda_inc;
            lambda = std::min(lambda * lambda_inc, 1e6);
        }
        last_err = err;

        // pose update
        Sophus::Vector6d xi;
        xi[0] = X(0, 0); // tx
        xi[1] = X(1, 0); // ty
        xi[2] = X(2, 0); // tz
        xi[3] = X(3, 0); // rx
        xi[4] = X(4, 0); // ry
        xi[5] = X(5, 0); // rz

        _G = Sophus::SE3d::exp(xi).matrix()*_G;

        // for rmt
        tm0 = tm1;
        tm1 = (_G.block(0,3,3,1).norm() + _G.block(0,3,3,1).norm())/2;

        // convergence check
        convergence = X.cwiseAbs().maxCoeff();
        if(convergence < 1e-7)
        {
            break;
        }
    }

    // update
    G = _G;

    // for debug
    //printf("[map_icp(3D)] map_icp, i:%d, n:%d, e:%f->%f, c:%e, dt:%.3f\n", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);
    spdlog::debug("[map_icp(3D)] map_icp, i:{}, n:{}, e:{}->{}, c:{}, dt:{}", iter, num_correspondence, first_err, last_err, convergence, get_time()-t_st);

    return last_err;
}

void LOCALIZATION::set_config_module(CONFIG* _config)
{
    config = _config;
}

void LOCALIZATION::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void LOCALIZATION::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}

void LOCALIZATION::set_lidar_2d_module(LIDAR_2D *_lidar_2d)
{
    lidar_2d = _lidar_2d;
}

void LOCALIZATION::set_lidar_3d_module(LIDAR_3D *_lidar_3d)
{
    lidar_3d = _lidar_3d;
}

void LOCALIZATION::set_cam_module(CAM *_cam)
{
    cam = _cam;
}

void LOCALIZATION::set_unimap_module(UNIMAP *_unimap)
{
    unimap = _unimap;
}

void LOCALIZATION::set_obsmap_module(OBSMAP *_obsmap)
{
    obsmap = _obsmap;
}
