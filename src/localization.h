#ifndef LOCALIZATION_H
#define LOCALIZATION_H

// defines
#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"
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
    LIDAR_3D *lidar_3d = NULL;
    // CAM *cam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;

    // interface functions
    void start();
    void stop();

    void set_cur_tf(Eigen::Matrix4d tf);
    Eigen::Matrix4d get_cur_tf();

    Eigen::Vector2d calc_ieir(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G);

    // algorithm for localization
    double map_icp(KD_TREE_XYZR& tree, XYZR_CLOUD& cloud, FRAME& frm, Eigen::Matrix4d& G); //2D
    double map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G); // 3D

    // loc loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop_2d();
    void a_loop_3d();

    // current pose
    Eigen::Matrix4d cur_tf;

    // for plot
    tbb::concurrent_queue<std::vector<Eigen::Vector3d>> plot_cur_pts_que;

    // flag
    std::atomic<bool> is_loc = {false};

    std::atomic<double> cur_tf_t = {0};
    std::atomic<double> cur_tf_err = {0};
    std::atomic<double> cur_tf_ie = {0};
    std::atomic<double> cur_tf_ir = {0};

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
