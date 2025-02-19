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
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    CAM *cam = NULL;

    void init();

    TIME_POSE_ID get_cur_tpi();
    cv::Mat get_plot_img(int cam_idx);

    std::vector<cv::Point3f> make_obj_pts();
    Eigen::Matrix4d se3_exp(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Matrix4d optional_tf);

    std::thread* a_thread = NULL;
    std::atomic<bool> a_flag = {false};
    void a_loop();

    void detect(int cam_idx);

    //const double marker_size = 0.18;
    const double marker_size = 0.1; // 10cm

    // storage
    TIME_POSE_ID cur_tpi;
    cv::Mat cur_plot_img[4];
    double last_t[4] = {0,};

Q_SIGNALS:


private Q_SLOTS:


};

#endif // ARUCO_H
