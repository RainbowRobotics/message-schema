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
#include "lidar/AIRY/airy.h"

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
    IMU get_cur_imu(int idx);                   // get cur imu info (built-in lidar)
    IMU get_best_imu(double ref_t);             // get nearest time(ref_t) imu info (Compare values ​​in all storages)
    IMU get_best_imu(double ref_t, int idx);    // get nearest time(ref_t) imu info (Compare values ​​in specific storage)
    bool get_is_sync();                         // check if synced lidar to mobile
    bool get_is_connected();                    // check if connected lidar
    double get_process_time_merge();            // merge loop processing time
    double get_process_time_deskewing(int idx); // deskewing loop processing time
    LVX_FRM get_cur_raw(int idx);               // get cur raw frame (LVX_FRM, global pts, not deskewing)
    QString get_info_text();                    // get all lidar info
    QString get_cur_state();                    // get lidar state
    TIME_PTS get_cur_frm();

    // setter func
    void set_is_sync(bool flag);                // set sync
    void set_cur_state(QString str);            // set cur state

    // related queue func
    void clear_merged_queue();                  // clear merge queue (for access from outside)
    bool try_pop_merged_queue(TIME_PTS& frm);   // try_pop merge queue (for access from outside)

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
    AIRY* airy;

    // deskewing loop
    std::atomic<bool> deskewing_flag[2] = {false, false};           // deskewing thread flag
    std::array<std::unique_ptr<std::thread>, 2> deskewing_thread;   // deskewing thread
    void deskewing_loop(int idx);                                   // deskewing loop (raw_frame to deskewing frame)

    // merge loop
    std::atomic<bool> merge_flag = {false};                         // merge thread flag
    std::unique_ptr<std::thread> merge_thread;                      // merge thread
    void merge_loop();                                              // merge loop (raw_frame array cnt is 2, merge)

    // flags
    std::atomic<bool> is_connected = {false};                       // is connected lidar
    std::atomic<bool> is_sync = {false};                            // is synced lidar

    // process time
    std::atomic<double> process_time_deskewing[2] = {0.0, 0.0};
    std::atomic<double> process_time_merge        = {0.0};

    // current state
    QString cur_state = "none";
    TIME_PTS cur_frm;
    std::atomic<double> cur_merged_frm_t = 0;                       // last merge_que pop time
    std::atomic<int> cur_merged_num = 0;                            // last merge_que point size

    // storage
    tbb::concurrent_queue<TIME_PTS> deskewing_que[2];               // deskewing queue (used in merge loop)
    tbb::concurrent_queue<TIME_PTS> merged_que;                     // merged queue (used in other modules)
};

#endif // LIDAR_3D_H
