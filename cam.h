#ifndef CAM_H
#define CAM_H

#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include <libobsensor/ObSensor.hpp>

#ifdef USE_OCAM
#include "ocam/format_converter/format_converter.hpp"
#include "ocam/format_converter/ConvertColor.h"
#include "ocam/withrobot_camera.hpp"
#include "ocam/withrobot_debug_print.h"
#include "ocam/camera_thread.h"
//#include "ocam/image_funcs.hpp"

enum OCAM_CONTROL
{
    OCAM_EXPOSURE_TIME_ABS=0,
    OCAM_EXPOSURE_TIME_AUTO,
    OCAM_GAIN,
    OCAM_BLUE_BALANCE,
    OCAM_RED_BALANCE
};
#endif

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

    cv::Mat get_img(int cam_idx);
    TIME_IMG get_time_img(int cam_idx);
    CAM_INTRINSIC get_intrinsic(int cam_idx);
    Eigen::Matrix4d get_extrinsic(int cam_idx);

    TIME_PTS get_scan0();
    TIME_PTS get_scan1();

    // loop
    std::atomic<bool> grab_flag;
    std::thread* grab_thread = NULL;
    void grab_loop();

    // value
    std::atomic<bool> is_connected[4] = {false, false, false, false};
    std::atomic<bool> is_param_loaded = {false};

    // storage    
    std::mutex mtx;

    cv::Mat cur_img[4];
    TIME_IMG cur_time_img[4];

    TIME_PTS cur_scan0;
    TIME_PTS cur_scan1;

    CAM_INTRINSIC intrinsic[4]; // color camera intrinsic
    Eigen::Matrix4d cam_tf[4];

private:


};

#endif // CAM_H
