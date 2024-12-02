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

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;

    // interface func
    void open();

    cv::Mat get_img0();
    cv::Mat get_img1();

    TIME_IMG get_time_img0();
    TIME_IMG get_time_img1();

    TIME_PTS get_scan0();
    TIME_PTS get_scan1();

    CAM_INTRINSIC get_rgb_intrinsic0();
    CAM_INTRINSIC get_rgb_intrinsic1();

    // loop
    std::atomic<bool> grab_flag;
    std::thread* grab_thread = NULL;
    void grab_loop();

    // value
    std::atomic<bool> is_connected0 = {false};
    std::atomic<bool> is_connected1 = {false};

    std::atomic<bool> is_param_loaded = {false};

    // storage    
    std::mutex mtx;

    cv::Mat cur_img0;
    cv::Mat cur_img1;

    TIME_IMG cur_time_img0;
    TIME_IMG cur_time_img1;

    TIME_PTS cur_scan0;
    TIME_PTS cur_scan1;

    CAM_INTRINSIC rgb_intrinsic0; // color camera intrinsic
    CAM_INTRINSIC rgb_intrinsic1;

    Eigen::Matrix4d rgb_extrinsic0;
    Eigen::Matrix4d rgb_extrinsic1;

private:


};

#endif // CAM_H
