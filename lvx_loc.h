#ifndef LVX_LOC_H
#define LVX_LOC_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// imu filter
#include "imu_filter.h"

// modules
#include "config.h"

// livox
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <QObject>

class LVX_LOC : public QObject
{
    Q_OBJECT
public:
    explicit LVX_LOC(QObject *parent = nullptr);
    ~LVX_LOC();
    std::mutex mtx;

    // imu filter
    imu_tools::ComplementaryFilter imu_filter;

    // modules
    CONFIG *config = NULL;

    // interface
    void init();
    void open();
    void map_load(QString file_path);    
    void loc_start();
    void loc_stop();

    void set_cur_tf(Eigen::Matrix4d tf);
    Eigen::Matrix4d get_cur_tf();
    IMU get_best_imu(double ref_t);    
    QString get_info_text();

    // map load
    QString last_map_file_path = "";
    std::thread* load_thread = NULL;
    void load_func(QString file_path);

    // grab loop
    std::atomic<bool> grab_flag = {false};
    std::thread* grab_thread = NULL;
    void grab_loop();

    // a loop
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop();

    // flag
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_loaded = {false};
    std::atomic<bool> is_sync = {false};

    // params
    std::atomic<double> offset_t = {0};
    std::atomic<double> cur_time = {0};
    std::atomic<double> cur_err = {0};
    Eigen::Matrix4d lvx_tf;
    Eigen::Matrix4d cur_tf;

    // storage    
    std::vector<LVX_PT> pts_storage;
    tbb::concurrent_queue<LVX_FRM> frm_que;
    std::vector<IMU> imu_storage;

    // for localization
    CLOUD cloud;
    KD_TREE *tree = NULL;

    std::vector<Eigen::Vector3d> map_pts;
    std::vector<Eigen::Vector3d> map_nor;

    std::vector<int> knn_search_idx(Eigen::Vector3d center, int k, double radius);
    double map_icp(std::vector<Eigen::Vector3d>& pts, Eigen::Matrix4d& G);

private:
    const int max_iter0 = 30; // maximum iteration
    const double lambda0 = 0.01; // 0.1
    const double lambda_dec = 0.1; // 0.01 ~ 0.1
    const double lambda_inc = 150; // 100 ~ 300
    const double t_dist_v0 = 15; // 5~30
    const double rmt_sigma = 0.01; // 0.01 good
    const double sigma_eps = 1e-6; // eps sigma
    const int nn_num = 3; // 5 ~ 10

Q_SIGNALS:

};

#endif // LVX_LOC_H
