#ifndef CAM_H
#define CAM_H

#include "global_defines.h"
#include "my_utils.h"

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

    cv::Mat get_img(int cam_idx);
    TIME_IMG get_time_img(int cam_idx);
    CAM_INTRINSIC get_intrinsic(int cam_idx);
    Eigen::Matrix4d get_extrinsic(int cam_idx);
    TIME_PTS get_scan(int cam_idx);

    // loop
    std::atomic<bool> grab_flag;
    std::thread* grab_thread = NULL;
    void grab_loop();

    // value
    std::atomic<bool> is_connected[2] = {false, false};
    std::atomic<bool> is_param_loaded = {false};

    // storage    
    std::recursive_mutex mtx;

    cv::Mat cur_img[2];
    TIME_IMG cur_time_img[2];
    TIME_PTS cur_scan[2];

    CAM_INTRINSIC intrinsic[2];
    Eigen::Matrix4d extrinsic[2];

    //SRV

    const int depth_profile_idx = 25; // depth_profile(25), w:320, h:180, fps:5, format:11
    //const int color_profile_idx = 88; // color_profile(88), w:1920, h:1080, fps:5, format:22
    const int color_profile_idx = 124; // color_profile(124), w:320, h:180, fps:5, format:22


    //D400
    //const int depth_profile_idx = 20; // depth_profile(20), w:640, h:400, fps:5, format:21
    //const int color_profile_idx = 43; // color_profile(43), w:640, h:400, fps:5, format:22

private:


};

#endif // CAM_H
