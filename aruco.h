#ifndef ARUCO_H
#define ARUCO_H

#include "global_defines.h"
#include "my_utils.h"

// module
#include "config.h"
#include "logger.h"
#include "cam.h"

#include <QObject>

#include <opencv2/aruco.hpp>

class ARUCO : public QObject
{
    Q_OBJECT
public:
    explicit ARUCO(QObject *parent = nullptr);
    ~ARUCO();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    CAM *cam = NULL;

    void init();

    TIME_POSE_ID get_cur_tpi();
    cv::Mat get_plot_img0();
    cv::Mat get_plot_img1();

    std::vector<cv::Point3f> make_obj_pts();
    Eigen::Matrix4d se3_exp(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Matrix4d optional_tf);

    std::thread* detect_thread0 = NULL;
    std::atomic<bool> detect_flag0 = {false};

    std::thread* detect_thread1 = NULL;
    std::atomic<bool> detect_flag1 = {false};
    void detect_loop(int cam_idx);

    const double marker_size = 0.18;

    // storage
    TIME_POSE_ID cur_tpi;

    cv::Mat cur_aruco_img0;
    cv::Mat cur_aruco_img1;

Q_SIGNALS:


private Q_SLOTS:


};

#endif // ARUCO_H
