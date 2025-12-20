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


class ORBBEC : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(ORBBEC)

public:
    // make singleton
    static ORBBEC* instance(QObject* parent = nullptr);

    void init();

    // to check cam is exist?
    void get_cam_exist_check();

    // start ORBBEC module
    void open();

    // stop ORBBEC module
    void close();

    // restart ORBBEC module
    void restart(int idx);

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

    std::atomic<bool> is_connected[max_cam_cnt];
    std::atomic<double> cam_t[max_cam_cnt];

private:
    explicit ORBBEC(QObject *parent = nullptr);
    ~ORBBEC();
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;

    // loop
    std::atomic<bool> grab_flag[max_cam_cnt];
    std::array<std::unique_ptr<std::thread>, max_cam_cnt> grab_thread;
    void grab_loop(int idx);

    // value
    // std::atomic<bool> is_connected[max_cam_cnt];
    std::atomic<bool> is_param_loaded[max_cam_cnt];

    cv::Mat cur_img[max_cam_cnt];
    TIME_IMG cur_time_img[max_cam_cnt];
    TIME_PTS cur_scan[max_cam_cnt];

    std::atomic<int> cur_w_color = {0};
    std::atomic<int> cur_h_color = {0};
    std::atomic<int> cur_w_depth = {0};
    std::atomic<int> cur_h_depth = {0};

    std::atomic<int> already_updated = {0};

    std::atomic<double> cur_pts_size[max_cam_cnt];

    CAM_INTRINSIC intrinsic[max_cam_cnt];
    Eigen::Matrix4d extrinsic[max_cam_cnt];

    tbb::concurrent_queue<TIME_PTS> depth_que[max_cam_cnt];
    tbb::concurrent_queue<TIME_IMG> img_que[max_cam_cnt];

Q_SIGNALS:
    void signal_restart(int idx);

private Q_SLOTS:
    void slot_restart(int idx);
};

#endif // ORBBEC_H
