#ifndef CAM_H
#define CAM_H

#include "global_defines.h"
#include "utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include <libobsensor/ObSensor.hpp>

// qt
#include <QObject>

class CAM : public QObject
{
    Q_OBJECT
public:
    explicit CAM(QObject *parent = nullptr);
    ~CAM();

    // interface func
    void init();

    //void sync();

    QString get_sn();

    cv::Mat get_cur_img();
    cv::Mat get_cur_img_color();

    std::vector<Eigen::Vector3d> get_cur_scan();

    // util func
    Eigen::Matrix4d zyx_tranformation(double x, double y, double z, double rx, double ry, double rz);

    // loop
    std::atomic<bool> grab_flag;
    std::thread* grab_thread = NULL;
    void grab_loop();

    // value
    std::atomic<bool> is_connected = {false};
    std::atomic<bool> is_synced_d = {false};
    std::atomic<bool> is_sync_d = {false};
    std::atomic<bool> is_synced_c = {false};
    std::atomic<bool> is_sync_c = {false};

    std::atomic<double> offset_t_d;
    std::atomic<double> offset_t_c;

    // storage    
    std::mutex mtx;
    QString serial_number;
    cv::Mat cur_img;
    cv::Mat cur_img_color;
    double cur_time = 0;
    std::vector<Eigen::Vector3d> cur_scan;

    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
private:


};

#endif // CAM_H
