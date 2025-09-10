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
#include "cam.h"
#include "unimap.h"
#include "obsmap.h"

#include "ekf.h"

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
    bool get_is_loc();                                  // check if connected lidar
    bool get_is_busy();                                 // check if semiauto init busy
    QString get_cur_loc_state();                        // get localization state (none, fail, good)
    QString get_info_text();                            // get all localization info
    Eigen::Matrix4d get_cur_tf();                       // get current tf (4x4 matrix)
    Eigen::Matrix4d get_best_tf(double t);              // get nearest time tf (4x4 matrix)
    Eigen::Vector2d get_cur_ieir();                     // get cur inlier error, inlier ratio
    std::vector<Eigen::Vector3d> get_cur_global_scan(); // get cur global scan

    double get_process_time_localization();
    double get_process_time_odometry();
    double get_process_time_obs();
    double get_process_time_node();

    void set_cur_loc_state(QString str);                // set loc state
    void set_cur_tf(Eigen::Matrix4d tf);                // set current tf
    void set_cur_ieir(Eigen::Vector2d ieir);            // set current inlier error, inlier ratio (only use simulation)

    Eigen::Vector2d calc_ieir(KD_TREE_XYZR& tree, FRAME& frm, Eigen::Matrix4d& G);   // 2D calc ieir
    Eigen::Vector2d calc_ieir(const std::vector<Eigen::Vector3d>& pts, const Eigen::Matrix4d& G);      // 3D calc ieir

    void start_semiauto_init();     // start semi-auto init

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_cam_module(CAM *_cam);
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
    CAM* cam;
    UNIMAP* unimap;
    OBSMAP* obsmap;

    // algorithm for localization
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G);  // 2D icp
    double map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G);                  // 3D icp

    // loop
    std::atomic<bool> localization_flag = {false};      // localization thread flag (lidar localization)
    std::unique_ptr<std::thread> localization_thread;   // localization thread
    void localization_loop_2d();                        // 2D localization loop
    void localization_loop_3d();                        // 3D localization loop

    std::atomic<bool> odometry_flag = {false};          // odometry thread flag (wheel odometry)
    std::unique_ptr<std::thread> odometry_thread;       // odometry thread
    void odometry_loop();                               // odometry loop        

    std::atomic<bool> obs_flag = {false};               // obstacle thread flag (obstacle map update)
    std::unique_ptr<std::thread> obs_thread;            // obstacle thread
    void obs_loop();                                    // obstacle loop

    // for plot
    std::vector<Eigen::Vector3d> cur_global_scan;       // cur global scan

    // flag
    std::atomic<bool> is_loc = {false};
    std::atomic<bool> is_busy = {false};

    // result
    Eigen::Matrix4d cur_tf;
    Eigen::Vector2d cur_ieir;
    std::atomic<double> cur_tf_err = {0.0};
    QString cur_loc_state = "none";

    // for loc
    std::vector<TIME_POSE> tp_storage;
    tbb::concurrent_queue<TIME_POSE> tp_que;
    tbb::concurrent_queue<TIME_POSE_PTS> tpp_que;

    // loop processing time
    std::atomic<double> process_time_localization = {0.0};
    std::atomic<double> process_time_odometry = {0.0};
    std::atomic<double> process_time_obs = {0.0};

    // extended kalman filter
    EKF ekf;
};

#endif // LOCALIZATION_H
