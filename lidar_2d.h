#ifndef LIDAR_2D_H
#define LIDAR_2D_H

// defines
#include "global_defines.h"
#include "utils.h"

// LakiBeam
#include "LakiBeamHTTP.h"
#include "LakiBeamUDP.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

// qt
#include <QObject>

class LIDAR_2D : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_2D(QObject *parent = nullptr);
    ~LIDAR_2D();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;

    // interface funcs
    void open();
    void sync_f();
    void sync_b();

    std::vector<Eigen::Vector3d> get_cur_scan_f();
    std::vector<Eigen::Vector3d> get_cur_scan_b();
    std::vector<Eigen::Vector3d> get_cur_scan();

    // flags
    std::atomic<bool> is_connected_f = {false};
    std::atomic<bool> is_connected_b = {false};

    std::atomic<bool> is_sync_f = {false};
    std::atomic<bool> is_sync_b = {false};

    std::atomic<bool> is_synced_f = {false};
    std::atomic<bool> is_synced_b = {false};

    std::atomic<double> offset_t_f = {0};
    std::atomic<double> offset_t_b = {0};

    std::atomic<double> last_t_f = {0};
    std::atomic<double> last_t_b = {0};

    // storage
    tbb::concurrent_queue<RAW_FRAME> raw_que_f;
    tbb::concurrent_queue<RAW_FRAME> raw_que_b;
    tbb::concurrent_queue<FRAME> scan_que;

    // vars
    const double angle_offset = 8.0;
    std::vector<Eigen::Vector3d> cur_scan_f;
    std::vector<Eigen::Vector3d> cur_scan_b;
    std::vector<Eigen::Vector3d> cur_scan;

    // loops
    std::atomic<bool> grab_flag_f;
    std::thread* grab_thread_f = NULL;
    void grab_loop_f();

    std::atomic<bool> a_flag;
    std::thread* a_thread = NULL;
    void a_loop();

    std::atomic<bool> grab_flag_b;
    std::thread* grab_thread_b = NULL;
    void grab_loop_b();
};

#endif // LIDAR_2D_H
