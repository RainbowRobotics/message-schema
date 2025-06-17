#ifndef LIDAR_2D_H
#define LIDAR_2D_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include "lidar/SICK/sick.h"

#include <QObject>

class LIDAR_2D : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_2D(QObject *parent = nullptr);
    ~LIDAR_2D();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config = NULL;
    LOGGER *logger = NULL;
    MOBILE* mobile = NULL;

    // interface functions
    void init();
    void open();
    void close();

    RAW_FRAME get_cur_raw(int idx);

    QString get_info_text();
    QString get_cur_state();
    void set_cur_state(QString str);
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
    tbb::concurrent_queue<RAW_FRAME> dsk_que[2];
    tbb::concurrent_queue<FRAME> merged_que;

    std::vector<std::pair<Eigen::Vector3d, bool>> scan_shadow_filter(const std::vector<Eigen::Vector3d>& dsk, int shadow_window);
    bool is_shadow(const double r1, const double r2, const double included_angle, const double min_angle_tan, const double max_angle_tan);


private:
    SICK *sick = nullptr;
};

#endif // LIDAR_2D_H
