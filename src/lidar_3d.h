#ifndef LIDAR_3D_H
#define LIDAR_3D_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
// #include "mobile.h"

// imu filter
#include "imu_filter.h"

// livox
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <QObject>

class LIDAR_3D : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_3D(QObject *parent = nullptr);
    ~LIDAR_3D();

    // mutex
    std::mutex mtx;

    // imu filter
    imu_tools::ComplementaryFilter imu_filter[2];

    // other modules
    CONFIG* config = NULL;
    LOGGER *logger = NULL;
    // MOBILE* mobile = NULL;

    // extrinsics
    Eigen::Matrix4d pts_tf[2];
    Eigen::Matrix4d imu_tf[2];
    double frm_dt = 0.1;

    // interface functions
    void init();
    void open();
    void close();
    IMU get_cur_imu(int idx);
    IMU get_best_imu(double ref_t);
    IMU get_best_imu(double ref_t, int idx);
    LVX_FRM get_cur_frm(int idx);
    std::vector<IMU> get_imu_storage();
    std::vector<IMU> get_imu_segment(double t0, double t1);

    QString get_cur_state();
    void set_cur_state(QString str);
    QString get_info_text();

    // for multi lidar
    std::array<uint32_t, 2> lidar_handles = {0, 0};
    int get_lidar_idx(uint32_t handle);

    // grab loop
    std::atomic<bool> grab_flag = {false};
    std::thread* grab_thread = NULL;
    void grab_loop();

    // dsk loop
    std::atomic<bool> dsk_flag[2] = {false, false};
    std::thread* dsk_thread[2] = {NULL, NULL};
    void dsk_loop(int idx);

    // a loop (merge)
    std::atomic<bool> a_flag = {false};
    std::thread* a_thread = NULL;
    void a_loop();

    // watchdog
    QString cur_state = "none";

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};

    std::atomic<double> offset_t[2] = {0, 0};
    std::atomic<double> cur_frm_t[2] = {0, 0};
    std::atomic<double> cur_imu_t[2] = {0, 0};
    std::atomic<int> cur_pts_num[2] = {0, 0};

    std::atomic<int> time_type[2] = {-1, -1};
    std::atomic<double> cur_merged_frm_t = 0;
    std::atomic<int> cur_merged_num = 0;

    // storage
    // std::atomic<double> pts_storing_st_time = {0};
    std::vector<LVX_PT> pts_storage[2];
    tbb::concurrent_queue<LVX_FRM> frm_que[2];
    tbb::concurrent_queue<TIME_PTS> dsk_que[2];
    tbb::concurrent_queue<TIME_PTS> merged_que;
    std::vector<IMU> imu_storage[2];

    LVX_FRM cur_frm[2];
    IMU cur_imu[2];

};

#endif // LIDAR_3D_H
