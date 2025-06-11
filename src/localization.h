#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_3d.h"
// #include "cam.h"
#include "unimap.h"
#include "obsmap.h"

#include <QObject>

class LOCALIZATION : public QObject
{
    Q_OBJECT
public:
    explicit LOCALIZATION(QObject *parent = nullptr);
    ~LOCALIZATION();

    // mutex
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar_2d = NULL;
    LIDAR_3D *lidar_3d = NULL;
    // CAM *cam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;

    // interface functions
    void start();
    void stop();

    void set_cur_tf(Eigen::Matrix4d tf);
    Eigen::Matrix4d get_cur_tf();
    Eigen::Vector2d get_cur_ieir();
    QString get_cur_loc_state();
    void set_cur_loc_state(QString str);
    QString get_info_text();

    Eigen::Vector2d calc_ieir(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G);
    Eigen::Vector2d calc_ieir(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);

    // algorithm for localization
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G); //2D
    double map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G); // 3D

    // loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop_2d();
    void a_loop_3d();

    std::atomic<bool> b_flag = {false};
    std::thread* b_thread = NULL;
    void b_loop();

    std::atomic<bool> obs_flag = {false};
    std::thread* obs_thread = NULL;
    void obs_loop();

    // for plot
    tbb::concurrent_queue<std::vector<Eigen::Vector3d>> plot_cur_pts_que;

    // flag
    std::atomic<bool> is_loc = {false};

    // result
    Eigen::Matrix4d cur_tf;
    Eigen::Vector2d cur_ieir;
    std::atomic<double> cur_tf_err = {0};
    QString cur_loc_state = "none";

    // for loc
    std::vector<TIME_POSE> tp_storage;
    tbb::concurrent_queue<TIME_POSE> tp_que;
    tbb::concurrent_queue<TIME_POSE_PTS> tpp_que;

    // loop processing time
    std::atomic<double> proc_time_loc_a = {0};
    std::atomic<double> proc_time_loc_b = {0};
    std::atomic<double> proc_time_obs = {0};

private:
    const int max_iter0 = 50;
    const double lambda0 = 0.1; // 0.1
    const double lambda_dec = 0.1; // 0.01 ~ 0.1
    const double lambda_inc = 150; // 100 ~ 300
    const double t_dist_v0 = 15; // 5~30
    const int near_pt_num = 10; // 5~10
    const int maginal_cnt = 300; // 100~300
    const double rmt_sigma = 0.01; // 0.01 good
    const double sigma_eps = 1e-6;

};

#endif // LOCALIZATION_H
