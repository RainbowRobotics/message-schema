#ifndef ORBBEC_H
#define ORBBEC_H

#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "mobile.h"

#include <libobsensor/ObSensor.hpp>

// qt
#include <QObject>

constexpr int depth_profile_idx = 55; // depth_profile(55), w:640, h:480, fps:30, format:11
constexpr int color_profile_idx = 124; // color_profile(124), w:320, h:180, fps:5, format:22

class ORBBEC : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(ORBBEC)

public:
    // make singleton
    static ORBBEC* instance(QObject* parent = nullptr);

    void init();

    // start ORBBEC module
    void open();

    // stop ORBBEC module
    void close();

    /***********************
     * interface funcs
     ***********************/
    cv::Mat get_img(int cam_idx);
    TIME_IMG get_time_img(int cam_idx);
    CAM_INTRINSIC get_intrinsic(int cam_idx);
    Eigen::Matrix4d get_extrinsic(int cam_idx);
    TIME_PTS get_scan(int cam_idx);
    QString get_cam_info_str();

    bool try_pop_depth_que(int idx, TIME_PTS& tp);
    bool try_pop_img_que(int idx, TIME_IMG& ti);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);

private:
    explicit ORBBEC(QObject *parent = nullptr);
    ~ORBBEC();
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;

    // loop
    std::atomic<bool> grab_flag;
    std::unique_ptr<std::thread> grab_thread;
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

    tbb::concurrent_queue<TIME_PTS> depth_que[2];
    tbb::concurrent_queue<TIME_IMG> img_que[2];

private:


};

#endif // ORBBEC_H
