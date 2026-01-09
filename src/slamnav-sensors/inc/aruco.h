#ifndef ARUCO_H
#define ARUCO_H

#include "slamnav_sensor_types.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "cam.h"

#include <vector>

#include <QObject>

#include <opencv2/aruco.hpp>

class ARUCO : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(ARUCO)

public:
    // make singleton
    static ARUCO* instance(QObject* parent = nullptr);

    // initialization aruco module
    void init();

    TIME_POSE_ID get_cur_tpi();
    cv::Mat get_plot_img(int cam_idx);

    bool get_detect_loop_alive();

    void start_detect_loop();
    void stop_detect_loop();

    bool get_is_thread_alive();

    bool get_is_found(int idx);

    void set_is_pause(bool flag);
    bool get_is_pause();

    void detect(int cam_idx);

    void clear_is_found();

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_cam_module(CAM* _cam);

private:
    explicit ARUCO(QObject *parent = nullptr);
    ~ARUCO();
    std::recursive_mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    CAM* cam;

    std::vector<cv::Point3f> make_obj_pts();
    Eigen::Matrix4d se3_exp(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Matrix4d optional_tf);

    std::atomic<bool> is_found[max_cam_cnt] = {false};
    std::atomic<bool> is_pause = {false};

    std::unique_ptr<std::thread> detect_thread = nullptr;
    std::atomic<bool> detect_flag = {false};
    void detect_loop();

    //const double marker_size = 0.18;
    const double marker_size = 0.1; // 10cm

    // storage
    TIME_POSE_ID cur_tpi;
    cv::Mat cur_plot_img[4];
    double last_t[4] = {0,};
};

#endif // ARUCO_H
