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
#include "lidar_bottom.h"
#include "cam.h"
#include "unimap.h"
#include "obsmap.h"

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
    LIDAR_BOTTOM *blidar = NULL;
    CAM *cam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;

    // interface
    void mapping_start();
    void mapping_stop();

    void localization_start();
    void localization_stop();

    Eigen::Matrix4d get_cur_tf();
    Eigen::Vector2d get_cur_ieir();
    TIME_POSE_PTS get_cur_tpp();
    Eigen::Matrix4d get_best_tf(double t);

    QString get_info_text();
    QString get_cur_loc_state();
    void set_cur_loc_state(QString str);    

    // auto init
    void semi_auto_init_start();

    // algorithms
    double frm_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);
    double kfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    double calc_overlap_ratio(std::vector<Eigen::Vector3d>& pts0, std::vector<Eigen::Vector3d>& pts1);
    Eigen::Vector2d calc_ie_ir(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);

    // flag
    std::atomic<bool> is_slam = {false};
    std::atomic<bool> is_loc = {false};
    std::atomic<bool> is_busy = {false};
    std::atomic<bool> is_init = {false};
    std::atomic<bool> is_qa = {false};

    // live kd_tree
    XYZR_CLOUD live_cloud;
    KD_TREE_XYZR *live_tree = NULL;

    // result        
    Eigen::Matrix4d cur_tf;
    Eigen::Vector2d cur_ieir;
    TIME_POSE_PTS cur_tpp;
    QString cur_loc_state = "none";

    // keyframe que
    tbb::concurrent_queue<KFRAME> kfrm_que;
    tbb::concurrent_queue<int> kfrm_update_que;
    std::vector<KFRAME> kfrm_storage;

    // for loc
    std::atomic<double> loc_st_time = {0};
    std::vector<TIME_POSE> tp_storage;
    tbb::concurrent_queue<TIME_POSE> tp_que;
    tbb::concurrent_queue<TIME_POSE_PTS> tpp_que;

    // for plot
    std::atomic<double> proc_time_map_a = {0};
    std::atomic<double> proc_time_map_b = {0};

    std::atomic<double> proc_time_loc_a = {0};
    std::atomic<double> proc_time_loc_b = {0};

    // algorithm params
    const int max_iter0 = 50;
    const double lambda0 = 0.1; // 0.1
    const double lambda_dec = 0.1; // 0.01 ~ 0.1
    const double lambda_inc = 150; // 100 ~ 300
    const double t_dist_v0 = 15; // 5~30
    const int near_pt_num = 5; // 5~10
    const double rmt_sigma = 0.01; // 0.01 good
    const double sigma_eps = 1e-6;

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

    std::atomic<bool> obs_flag = {false};
    std::thread* obs_thread = NULL;
    void obs_loop();

Q_SIGNALS:
    void signal_localization_semiautoinit_succeed(QString message);
    void signal_localization_semiautoinit_failed(QString message);
};

#endif // SLAM_2D_H
