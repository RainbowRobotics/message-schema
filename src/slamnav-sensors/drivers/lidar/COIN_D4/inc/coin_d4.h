#ifndef COIN_D4_H
#define COIN_D4_H

#include "slamnav_sensor_types.h"
#include "my_utils.h"

#include "C_CSPC_Lidar.h"
#include "pthread.h"

// module
#include "config.h"
#include "logger.h"

// qt
#include <QObject>

class COIN_D4 : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COIN_D4)
public:
    static COIN_D4 *instance(QObject *parent = 0);

    // start coin d4 module
    void open();

    // sync between mobile, sick
    void sync();


    std::vector<Eigen::Vector3d> get_cur_scan();
    TIME_PTS get_cur_tp();


private:
    explicit COIN_D4(QObject *parent = nullptr);
    ~COIN_D4();
    std::shared_mutex mtx;

    void set_raw_scan(const LaserScan& scan);
    LaserScan get_raw_scan();

    // other modules
    CONFIG* config = nullptr;
    LOGGER* logger = nullptr;

    std::vector<Eigen::Vector3d> cur_scan_blidar;
    TIME_PTS cur_tp;

    std::atomic<bool> is_connected = {false};
    std::atomic<double> max_dist_blidar = {0.0};

    LaserScan raw_scan;
    C_CSPC_Lidar laser;
    LidarVersion version;
    device_health LidarHealthInfo;

    std::string port = "/dev/ttyBL0";
    int baudrate = 230400;

    int failure_count = 0;
    const int max_failures = 5;

    float set_min_angle = 135;
    float set_max_angle = 225;

    float set_min_dist = 0.05;
    float set_max_dist = 12.0;

    std::atomic<float> last_time = 0;

private:
    std::atomic<bool> recv_flag = {false};
    std::unique_ptr<std::thread> recv_thread;
    void recv_loop();

    std::atomic<bool> grab_flag = {false};
    std::unique_ptr<std::thread> grab_thread;
    void grab_loop();

    std::atomic<int> scan_fail_count = {0};
    size_t FORCE_SCAN_TRIGGER = 900;

};

#endif // COIN_D4_H
