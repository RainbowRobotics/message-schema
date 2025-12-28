#ifndef LIVOX_H
#define LIVOX_H

// global defines
#include "slamnav_sensor_types.h"
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
    Q_DISABLE_COPY(LIVOX)

public:
    // make singleton
    static LIVOX* instance(QObject* parent = nullptr);

    // start livox module
    void open();

    // stop livox module
    void close();

    /***********************
     * interface funcs
     ***********************/
    IMU get_cur_imu(int idx);
    LVX_FRM get_cur_raw(int idx);
    QString get_info_text(int idx);
    std::vector<IMU> get_imu_storage(int idx);

    bool get_is_connected(int idx);
    void set_is_connected(int idx, bool val);
    bool get_is_sync(int idx);
    void set_is_sync(int idx, bool val);

    int get_time_type(int idx);

    bool try_pop_frm_que(int idx, LVX_FRM& frm);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);

private:
    explicit LIVOX(QObject *parent = nullptr);
    ~LIVOX();

    // mutex
    std::shared_mutex lidar_mtx;
    std::shared_mutex imu_mtx;

    // imu filter (complementary Filter)
    imu_tools::ComplementaryFilter imu_filter[2];

    // other modules
    CONFIG* config;
    LOGGER* logger;

    // extrinsics
    Eigen::Matrix4d pts_tf[2];
    Eigen::Matrix3d pts_R[2];
    Eigen::Vector3d pts_t[2];
    Eigen::Matrix4d imu_tf[2];
    Eigen::Matrix3d imu_R[2];
    Eigen::Vector3d imu_t[2];

    // for multi lidar
    std::array<uint32_t, 2> livox_handles = {0, 0};
    int get_livox_idx(uint32_t handle);

    // grab loop
    std::atomic<bool> grab_flag = {false};
    std::unique_ptr<std::thread> grab_thread;
    void grab_loop();

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync[2] = {false, false};

    std::atomic<double> offset_t[2] = {0, 0};
    std::atomic<double> cur_frm_t[2] = {0, 0};
    std::atomic<double> cur_imu_t[2] = {0, 0};
    std::atomic<int> cur_pts_num[2] = {0, 0};

    std::atomic<int> time_type[2] = {-1, -1};

    // storage
    std::vector<LVX_PT> pts_storage[2];
    std::vector<IMU> imu_storage[2];
    tbb::concurrent_queue<LVX_FRM> frm_que[2];

    LVX_FRM cur_raw[2];
    IMU cur_imu[2];

private:
    const double lvx_frm_dt = 0.1;
};

#endif // LIVOX_H
