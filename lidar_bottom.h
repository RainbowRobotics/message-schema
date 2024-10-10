#ifndef LIDAR_BOTTOM_H
#define LIDAR_BOTTOM_H

#include "global_defines.h"

// lidar sdk
#include "node_lidar.h"
#include "msg_recept.h"
#include "serial_port.h"
#include "lidar_data_processing.h"
#include "point_cloud_optimize.h"
#include "lidar_information.h"
#include "mtime.h"
#include "calibration.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

// qt
#include <QObject>
#include <QProcess>
#include <QTimer>

class LIDAR_BOTTOM : public QObject
{
    Q_OBJECT
public:
    explicit LIDAR_BOTTOM(QObject *parent = nullptr);
    ~LIDAR_BOTTOM();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;

    void open();
    void run();
    void stop();

    TIME_PTS get_cur_scan();
    cv::Vec2d transform(cv::Vec3d xi, cv::Vec2d pt);

    std::atomic<bool> is_connected = {false};

    TIME_PTS cur_scan;
    double time = 0;

    std::atomic<int> grab_fail_cnt = {0};
    std::atomic<int> reconnection_fail_cnt = {0};

private:
    std::atomic<bool> recv_flag = {false};
    std::thread* recv_thread = NULL;
    void recv_loop();

    std::atomic<bool> grab_flag = {false};
    std::thread* grab_thread = NULL;
    void grab_loop();

};

#endif // LIDAR_BOTTOM_H
