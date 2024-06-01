#ifndef SLAM_2D_H
#define SLAM_2D_H

// defines
#include "global_defines.h"
#include "utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "unimap.h"

// third party
#include "pgo.h"

// qt
#include <QObject>

class SLAM_2D : public QObject
{
    Q_OBJECT
public:
    explicit SLAM_2D(QObject *parent = nullptr);
    ~SLAM_2D();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    UNIMAP *unimap = NULL;

    // interface
    void mapping_start();
    void mapping_stop();

    void localization_start();
    void localization_stop();

    Eigen::Matrix4d get_cur_tf();
    Eigen::Vector2d get_cur_ieir();
    TIME_POSE_PTS get_cur_tpp();

    // algorithms
    double frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    double calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1);
    Eigen::Vector2d calc_ie_ir(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);

    // flag
    std::atomic<bool> is_slam = {false};
    std::atomic<bool> is_loc = {false};

    // live kd_tree
    XYZR_CLOUD live_cloud;
    KD_TREE_XYZR *live_tree = NULL;

    // result        
    Eigen::Matrix4d cur_tf;
    Eigen::Vector2d cur_ieir;
    TIME_POSE_PTS cur_tpp;

    // keyframe que
    tbb::concurrent_queue<KFRAME> kfrm_que;
    tbb::concurrent_queue<int> kfrm_update_que;
    std::vector<KFRAME> kfrm_storage;

    // for loc
    tbb::concurrent_queue<TIME_POSE_PTS> tpp_que;

    // for plot
    std::atomic<double> proc_time_map_a = {0};
    std::atomic<double> proc_time_map_b = {0};

    std::atomic<double> proc_time_loc_a = {0};
    std::atomic<double> proc_time_loc_b = {0};

    // algorithm params
    const double rmt_sigma = 0.01;

public:
    std::atomic<bool> map_a_flag = {false};
    std::thread* map_a_thread = NULL;
    void map_a_loop();

    std::atomic<bool> map_b_flag = {false};
    std::thread* map_b_thread = NULL;
    void map_b_loop();

    std::atomic<bool> loc_a_flag = {false};
    std::thread* loc_a_thread = NULL;
    void loc_a_loop();

    std::atomic<bool> loc_b_flag = {false};
    std::thread* loc_b_thread = NULL;
    void loc_b_loop();

Q_SIGNALS:

};

#endif // SLAM_2D_H
