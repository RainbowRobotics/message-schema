#ifndef RP_LIDAR_H
#define RP_LIDAR_H

// defines
#include "slamnav_sensor_types.h"
#include "my_utils.h"

// S1 Lidar
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#ifndef _countof
    #define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

// qt
#include <QObject>

struct RP_LIDAR_INFO
{
    static constexpr int minimum_lidar_pts = 100;
    static constexpr double deg_resolution = 16384.0;
    static constexpr double dist_resolution = 1000.0;
};

class RP_LIDAR : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(RP_LIDAR)
public:
    // make singleton
    static RP_LIDAR* instance(QObject* parent = nullptr);

    void open();

    // stop RP lidar module
    void close();

    // sync between mobile, RP lidar
    void sync(int idx);

    /***********************
     * interface funcs
     ***********************/
    void set_is_sync(int idx, bool val);
    bool get_is_sync(int idx);
    bool get_is_connected(int idx);
    QString get_info_text(int idx);
    RAW_FRAME get_cur_raw(int idx);

    bool try_pop_raw_que(int idx, RAW_FRAME& frm);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE *_mobile);

private:
    explicit RP_LIDAR(QObject *parent = nullptr);
    ~RP_LIDAR();

    std::mutex mtx;

    // other modules
    CONFIG *config;
    LOGGER *logger;
    MOBILE *mobile;

    // extrinsics
    Eigen::Matrix4d pts_tf[2];

    // grab loop
    std::atomic<bool> grab_flag[2] = {false, false};
    std::array<std::unique_ptr<std::thread>, 2> grab_thread;
    void grab_loop(int idx);

    // flags
    std::atomic<bool> is_sync[2]      = {false, false};
    std::atomic<bool> is_synced[2]    = {false, false};
    std::atomic<bool> is_connected[2] = {false, false};

    // params
    std::atomic<int> cur_pts_num[2]  = {0, 0};
    std::atomic<double> offset_t[2]  = {0, 0};
    std::atomic<double> cur_raw_t[2] = {0, 0};

    // storage
    tbb::concurrent_queue<RAW_FRAME> raw_que[2];

    // vars
    const double angle_offset = 10.0; // LAKI:8.0, SICK:10.0
    RAW_FRAME cur_raw[2];
};

#endif // RP_LIDAR_H
