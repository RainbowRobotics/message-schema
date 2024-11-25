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

    cv::Mat get_aruco0();
    cv::Mat get_aruco1();

    // TIME_POSE_ID get_cur_tpi();
    // cv::Mat get_plot_img();

    bool check_regist_aruco(QString id, std::vector<cv::Point3d>& corners);
    Eigen::Matrix4d m2e(cv::Mat rvec, cv::Mat tvec);


    std::thread* detect_thread0 = NULL;
    std::atomic<bool> detect_flag0 = {false};
    void detect_loop0();

    std::thread* detect_thread1 = NULL;
    std::atomic<bool> detect_flag1 = {false};
    void detect_loop1();



    std::map<QString, std::vector<cv::Point3d>> aruco_metric;


    // storage
    cv::Mat cur_aruco0;
    cv::Mat cur_aruco1;

    // flag
    std::atomic<bool> is_detect = {false};


Q_SIGNALS:


private Q_SLOTS:


};

#endif // ARUCO_H
