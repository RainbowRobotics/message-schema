#ifndef LIDAR_3D_H
#define LIDAR_3D_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
// #include "mobile.h"
#include "lidar/LIVOX/livox.h"

// imu filter
#include "imu_filter.h"

#include <QObject>

class LIDAR_3D : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_3D(QObject *parent = nullptr);
    ~LIDAR_3D();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config = NULL;
    LOGGER *logger = NULL;
    // MOBILE* mobile = NULL;

    // interface functions
    void init();
    void open();
    void close();

    LVX_FRM get_cur_frm(int idx);
    IMU get_cur_imu(int idx);

    IMU get_best_imu(double ref_t);
    IMU get_best_imu(double ref_t, int idx);

    QString get_cur_state();
    void set_cur_state(QString str);
    QString get_info_text();
    void set_sync_flag(bool flag);

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

    // std::atomic<double> offset_t[2] = {0, 0};
    // std::atomic<double> cur_frm_t[2] = {0, 0};
    // std::atomic<double> cur_imu_t[2] = {0, 0};
    // std::atomic<int> cur_pts_num[2] = {0, 0};

    // std::atomic<int> time_type[2] = {-1, -1};
    std::atomic<double> cur_merged_frm_t = 0;
    std::atomic<int> cur_merged_num = 0;

    // storage
    // std::atomic<double> pts_storing_st_time = {0};
    // std::vector<LVX_PT> pts_storage[2];
    tbb::concurrent_queue<TIME_PTS> dsk_que[2];
    tbb::concurrent_queue<TIME_PTS> merged_que;
    // std::vector<IMU> imu_storage[2];

    // tbb::concurrent_queue<LVX_FRM> frm_que[2];

    // LVX_FRM cur_frm[2];
    // IMU cur_imu[2];

private:
    LIVOX *livox = NULL;

};

#endif // LIDAR_3D_H
