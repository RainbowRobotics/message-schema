#ifndef LIVOX_H
#define LIVOX_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"

// imu filter
#include "imu_filter.h"

// livox
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#include <QObject>

class LIVOX : public QObject
{
    Q_OBJECT
public:
    explicit LIVOX(QObject *parent = nullptr);
    ~LIVOX();

    // mutex
    std::mutex mtx;

    // imu filter
    imu_tools::ComplementaryFilter imu_filter[2];

    // other modules
    CONFIG* config = NULL;
    LOGGER *logger = NULL;

    // extrinsics
    Eigen::Matrix4d pts_tf[2];
    Eigen::Matrix4d imu_tf[2];

    // interface functions
    void init();
    void open();
    void close();

    IMU get_cur_imu(int idx);
    LVX_FRM get_cur_frm(int idx);
    QString get_info_text(int idx);

    // for multi lidar
    std::array<uint32_t, 2> livox_handles = {0, 0};
    int get_livox_idx(uint32_t handle);

    // grab loop
    std::atomic<bool> grab_flag = {false};
    std::thread* grab_thread = NULL;
    void grab_loop();

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};

    std::atomic<double> offset_t[2] = {0, 0};
    std::atomic<double> cur_frm_t[2] = {0, 0};
    std::atomic<double> cur_imu_t[2] = {0, 0};
    std::atomic<int> cur_pts_num[2] = {0, 0};

    std::atomic<int> time_type[2] = {-1, -1};

    // storage
    std::vector<LVX_PT> pts_storage[2];
    std::vector<IMU> imu_storage[2];
    tbb::concurrent_queue<LVX_FRM> frm_que[2];

    LVX_FRM cur_frm[2];
    IMU cur_imu[2];

private:
    const double lvx_frm_dt = 0.1;
};

#endif // LIVOX_H
