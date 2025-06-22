#ifndef LIDAR_3D_H
#define LIDAR_3D_H

// global defines
#include "global_defines.h"
#include "my_utils.h"

// modules
#include "config.h"
#include "logger.h"
 #include "mobile.h"
#include "lidar/LIVOX/livox.h"

// imu filter
#include "imu_filter.h"

#include <QObject>

class LIDAR_3D : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(LIDAR_3D)
public:
    // make singleton
    static LIDAR_3D* instance(QObject* parent = nullptr);

    // initialization lidar_3d module
    void init();

    // open lidar_3d module as written in config.json
    void open();

    // close lidar_3d module
    void close();

    /***********************
     * interface funcs
     ***********************/
    QString get_info_text();
    QString get_cur_state();
    LVX_FRM get_cur_raw(int idx);
    bool get_is_connected();
    bool get_is_sync();
    IMU get_cur_imu(int idx);
    IMU get_best_imu(double ref_t);
    IMU get_best_imu(double ref_t, int idx);
    void set_cur_state(QString str);
    void set_sync_flag(bool flag);

    void clear_merged_queue();
    bool try_pop_merged_queue(TIME_PTS& frm);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);

private:
    explicit LIDAR_3D(QObject *parent = nullptr);
    ~LIDAR_3D();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    LIVOX* livox;

    // deskewing loop
    std::atomic<bool> deskewing_flag[2] = {false, false};
    std::array<std::unique_ptr<std::thread>, 2> deskewing_thread;
    void deskewing_loop(int idx);

    // merge loop
    std::atomic<bool> merge_flag = {false};
    std::unique_ptr<std::thread> merge_thread;
    void merge_loop();

    // watchdog
    QString cur_state = "none";

    // params
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_sync = {false};

    std::atomic<double> cur_merged_frm_t = 0;
    std::atomic<int> cur_merged_num = 0;

    // storage
    tbb::concurrent_queue<TIME_PTS> deskewing_que[2];
    tbb::concurrent_queue<TIME_PTS> merged_que;
};

#endif // LIDAR_3D_H
