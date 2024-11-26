#ifndef ARUCO_H
#define ARUCO_H

#include "global_defines.h"
#include "utils.h"

// module
#include "config.h"
#include "logger.h"
#include "cam.h"
#include "slam_2d.h"
#include "unimap.h"

#include <QObject>
#include <QTimer>

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
    SLAM_2D *slam = NULL;
    UNIMAP* unimap = NULL;

    void init();

    TIME_POSE_ID get_cur_tpi();
    cv::Mat get_plot_img0();
    cv::Mat get_plot_img1();

    TIME_IMG load_sim_image(int cam_idx);
    std::vector<cv::Point3f> make_obj_pts();
    bool check_regist_aruco(QString id, std::vector<cv::Point3d>& corners);
    Eigen::Matrix4d se3_exp(cv::Mat rvec, cv::Mat tvec);


    std::thread* detect_thread0 = NULL;
    std::atomic<bool> detect_flag0 = {false};
    // void detect_loop0();

    std::thread* detect_thread1 = NULL;
    std::atomic<bool> detect_flag1 = {false};
    // void detect_loop1();
    void detect_loop(int cam_idx);


    double marker_size = 0.18;
    std::map<QString, std::vector<cv::Point3d>> aruco_metric;


    // storage
    TIME_POSE_ID cur_tpi;

    cv::Mat cur_aruco_img0;
    cv::Mat cur_aruco_img1;



    // flag
    // std::atomic<bool> is_detect0 = {false};
    // std::atomic<bool> is_detect1 = {false};


Q_SIGNALS:


private Q_SLOTS:


};

#endif // ARUCO_H
