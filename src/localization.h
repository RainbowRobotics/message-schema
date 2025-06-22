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

constexpr double lambda0    = 0.1;   // 0.1
constexpr double lambda_dec = 0.1;   // 0.01 ~ 0.1
constexpr double lambda_inc = 150;   // 100 ~ 300
constexpr double t_dist_v0  = 15;    // 5~30
constexpr double rmt_sigma  = 0.01;  // 0.01 good
constexpr double sigma_eps  = 1e-6;
constexpr int near_pt_num   = 10;    // 5~10
constexpr int maginal_cnt   = 300;   // 100~300
constexpr int max_iter0     = 50;

class LOCALIZATION : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LOCALIZATION)
public:
    // make singleton
    static LOCALIZATION* instance(QObject* parent = nullptr);

    // start localization module
    void start();

    // stop localization module
    void stop();

    /***********************
     * interface funcs
     ***********************/
    std::vector<Eigen::Vector3d> get_cur_global_scan();
    Eigen::Matrix4d get_cur_tf();
    Eigen::Matrix4d get_best_tf(double t);
    Eigen::Vector2d get_cur_ieir();
    QString get_cur_loc_state();
    QString get_info_text();
    bool get_is_loc();
    bool get_is_busy();

    void set_cur_tf(Eigen::Matrix4d tf);
    void set_cur_loc_state(QString str);
    void set_cur_ieir(Eigen::Vector2d ieir);

    Eigen::Vector2d calc_ieir(const std::vector<Eigen::Vector3d>& pts, const  Eigen::Matrix4d& G);
    Eigen::Vector2d calc_ieir(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);

    void semi_auto_init_start();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);

private:
    explicit LOCALIZATION(QObject *parent = nullptr);
    ~LOCALIZATION();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    LIDAR_2D* lidar_2d;
    LIDAR_3D* lidar_3d;
    // CAM *cam = NULL;
    UNIMAP* unimap;
    OBSMAP* obsmap;

    // algorithm for localization
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G); //2D
    double map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G); // 3D

    // loop
    std::atomic<bool> localization_flag = {false};
    std::unique_ptr<std::thread> localization_thread;
    void localization_loop_2d();
    void localization_loop_3d();

    std::atomic<bool> odometry_flag = {false};
    std::unique_ptr<std::thread> odometry_thread;
    void odometry_loop();

    std::atomic<bool> obs_flag = {false};
    std::unique_ptr<std::thread> obs_thread;
    void obs_loop();

    // for plot
    std::vector<Eigen::Vector3d> cur_global_scan;

    // flag
    std::atomic<bool> is_loc = {false};
    std::atomic<bool> is_busy = {false};

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

};

#endif // LOCALIZATION_H
