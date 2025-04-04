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
    std::recursive_mutex mtx;

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
    QString get_cam_info_str();

    // loop
    std::atomic<bool> grab_flag;
    std::thread* grab_thread = NULL;
    void grab_loop();

    // value
    std::atomic<bool> is_connected[2] = {false, false};
    std::atomic<bool> is_param_loaded = {false};

    cv::Mat cur_img[2];
    TIME_IMG cur_time_img[2];
    TIME_PTS cur_scan[2];

    std::atomic<int> cur_w_color = {0};
    std::atomic<int> cur_h_color = {0};
    std::atomic<int> cur_w_depth = {0};
    std::atomic<int> cur_h_depth = {0};

    std::atomic<double> cur_pts_size0 = {0.};
    std::atomic<double> cur_pts_size1 = {0.};

    CAM_INTRINSIC intrinsic[2];
    Eigen::Matrix4d extrinsic[2];
    
    #if defined (USE_S100)
    const int depth_profile_idx = 25; // depth_profile(25), w:320, h:180, fps:5, format:11
    const int color_profile_idx = 124; // color_profile(124), w:320, h:180, fps:5, format:22
    #endif

    #if defined (USE_D400) || defined (USE_D400_LAKI)
    const int depth_profile_idx = 20; // depth_profile(20), w:640, h:400, fps:5, format:21
    const int color_profile_idx = 43; // color_profile(43), w:640, h:400, fps:5, format:22
    #endif

    #if defined (USE_MECANUM_OLD) || defined(USE_MECANUM)
    const int depth_profile_idx = 0;
    const int color_profile_idx = 0;
    #endif

private:


};

#endif // CAM_H
