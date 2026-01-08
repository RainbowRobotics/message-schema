#include "aruco.h"
namespace
{
    const char* MODULE_NAME = "ARUCO";
}

ARUCO* ARUCO::instance(QObject *parent)
{
    static ARUCO* inst = nullptr;
    if(!inst && parent)
    {
        inst = new ARUCO(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }

    return inst;
}

ARUCO::ARUCO(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    cam(nullptr)
{

}

ARUCO::~ARUCO()
{
    stop_detect_loop();
}

void ARUCO::init()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        printf("[ARUCO] simulation mode\n");
        return;
    }

    start_detect_loop();
}

void ARUCO::set_is_pause(bool flag)
{
    is_pause.store(flag);
}

bool ARUCO::get_is_pause()
{
    return (bool)is_pause.load();
}

bool ARUCO::get_detect_loop_alive()
{
    return (bool)detect_flag.load();
}

void ARUCO::start_detect_loop()
{
    if(detect_thread == nullptr)
    {
        detect_flag = true;
        detect_thread = new std::thread(&ARUCO::detect_loop, this);
    }
}

void ARUCO::stop_detect_loop()
{
    if(detect_thread != nullptr)
    {
        detect_flag = false;
        detect_thread->join();
        detect_thread = nullptr;
    }
}

TIME_POSE_ID ARUCO::get_cur_tpi()
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_tpi;
}

cv::Mat ARUCO::get_plot_img(int cam_idx)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    return cur_plot_img[cam_idx].clone();
}

std::vector<cv::Point3f> ARUCO::make_obj_pts()
{
    std::vector<cv::Point3f> obj_pts(4);

    obj_pts[0] = cv::Point3d(-marker_size/2,  marker_size/2, 0.0);
    obj_pts[1] = cv::Point3d( marker_size/2,  marker_size/2, 0.0);
    obj_pts[2] = cv::Point3d( marker_size/2, -marker_size/2, 0.0);
    obj_pts[3] = cv::Point3d(-marker_size/2, -marker_size/2, 0.0);

    return obj_pts;
}

Eigen::Matrix4d ARUCO::se3_exp(cv::Vec3d rvec, cv::Vec3d tvec, Eigen::Matrix4d optional_tf)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d eR;
    eR << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);

    Eigen::Vector3d et;
    et << tvec[0], tvec[1], tvec[2];

    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = eR;
    T.block<3,1>(0,3) = et;

    T = T*optional_tf;
    return T;
}

void ARUCO::detect_loop()
{
    log_info("start a_loop");
    while(detect_flag)
    {
        if(is_pause.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(config->get_use_cam_depth() == false)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        for(int cam_idx = 0; cam_idx < 2; cam_idx++)
        {
            detect(cam_idx);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    log_info("stop a_loop");
}

void ARUCO::detect(int cam_idx)
{
    // get params
    if(!cam->get_is_param_loaded(cam_idx))
    {
        log_warn("cam param not loaded");
        return;
    }

    Eigen::Matrix4d extrinsic = cam->get_extrinsic(cam_idx);
    CAM_INTRINSIC intrinsic = cam->get_intrinsic(cam_idx);
    TIME_IMG t_img = cam->get_time_img(cam_idx);
    if(t_img.t == 0 || t_img.img.empty() || t_img.t == last_t[cam_idx])
    {
        log_warn("invalid cam img");
        return;
    }

    last_t[cam_idx] = t_img.t;

    cv::Matx33d cm(intrinsic.fx,            0, intrinsic.cx,
                              0, intrinsic.fy, intrinsic.cy,
                              0,            0,            1);

    cv::Mat dc(intrinsic.coef_num, 1, CV_64F, cv::Scalar(0));
    if(intrinsic.coef_num == 4)
    {
        dc.ptr<double>(0)[0] = intrinsic.k1;
        dc.ptr<double>(1)[0] = intrinsic.k2;
        dc.ptr<double>(2)[0] = intrinsic.p1;
        dc.ptr<double>(3)[0] = intrinsic.p2;
    }
    else if(intrinsic.coef_num == 8)
    {
        dc.ptr<double>(0)[0] = intrinsic.k1;
        dc.ptr<double>(1)[0] = intrinsic.k2;
        dc.ptr<double>(2)[0] = intrinsic.p1;
        dc.ptr<double>(3)[0] = intrinsic.p2;
        dc.ptr<double>(4)[0] = intrinsic.k3;
        dc.ptr<double>(5)[0] = intrinsic.k4;
        dc.ptr<double>(6)[0] = intrinsic.k5;
        dc.ptr<double>(7)[0] = intrinsic.k6;
    }

    // pre processing
    cv::Mat img;
    cv::cvtColor(t_img.img, img, cv::COLOR_BGR2GRAY);
    //cv::equalizeHist(img, img);

    // marker detection
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    std::vector<int> detected_id;
    std::vector<std::vector<cv::Point2f>> detected_uv;
    cv::aruco::detectMarkers(img, dictionary, detected_uv, detected_id, parameters);
    if(detected_id.empty())
    {
        log_info("no marker detected");
        return;
    }

    // make plot img
    cv::Mat plot_aruco = t_img.img.clone();

    // find nn tf
    bool _is_found = false;
    int min_id = 0;
    double min_d = 99999999;
    Eigen::Matrix4d min_tf = Eigen::Matrix4d::Identity();
    for(size_t id = 0; id < detected_id.size(); id++)
    {
        cv::circle(plot_aruco, detected_uv[id][0], 30, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[id][1], 30, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[id][2], 30, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[id][3], 30, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        // extract 3d-2d pairs
        std::vector<cv::Point3f> matched_xyzs = make_obj_pts();
        std::vector<cv::Point2f> matched_uvs;
        for(int p = 0; p < 4; p++)
        {
            double u = (double)detected_uv[id][p].x;
            double v = (double)detected_uv[id][p].y;
            matched_uvs.push_back(cv::Point2f(u,v));
        }

        // add dummy pts
        for(int i = -5; i <= 5; i++)
        {
            for(int j = -5; j <= 5; j++)
            {
                if(i == 0 && j == 0)
                {
                    continue;
                }

                for(int p = 0; p < 4; p++)
                {
                    double u = (double)detected_uv[id][p].x + (double)j*0.2;
                    double v = (double)detected_uv[id][p].y + (double)i*0.2;
                    matched_xyzs.push_back(matched_xyzs[p]);
                    matched_uvs.push_back(cv::Point2f(u,v));
                }
            }
        }

        // estimate extrinsic
        cv::Mat rvec, tvec;
        if(cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec, false, cv::SOLVEPNP_SQPNP))
        {
            cv::solvePnPRefineLM(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);
            cv::drawFrameAxes(plot_aruco, cm, dc, rvec, tvec, 0.1, 30);

            Eigen::Matrix4d optional_tf = ZYX_to_TF(0,0,0,0,-90.0*D2R,-90.0*D2R);

            Eigen::Matrix4d cam_to_marker = se3_exp(rvec, tvec, optional_tf);
            refine_pose(cam_to_marker);

            Eigen::Matrix4d robot_to_marker = extrinsic * cam_to_marker;
            double d = calc_dist_2d(robot_to_marker.block(0,3,3,1));
            if(d < min_d && d < 1.0)
            {
                min_d = d;
                min_tf = robot_to_marker;
                min_id = detected_id[id];
                _is_found = true;
            }
        }
    }

    if(_is_found)
    {
        TIME_POSE_ID aruco_tpi;
        aruco_tpi.t = t_img.t;
        aruco_tpi.id = min_id;
        aruco_tpi.tf = min_tf;

        // update result
        {
            std::lock_guard<std::recursive_mutex> lock(mtx);
            if(aruco_tpi.t > cur_tpi.t)
            {
                cur_tpi = aruco_tpi;
            }

            cv::resize(plot_aruco, plot_aruco, cv::Size(160, 90));
            cur_plot_img[cam_idx]= plot_aruco.clone();
        }
    }

    is_found.store(_is_found);
}

void ARUCO::set_config_module(CONFIG* _config)
{
    if(_config)
    {
        config = _config;
    }
}

void ARUCO::set_logger_module(LOGGER* _logger)
{
    if(_logger)
    {
        logger = _logger;
    }
}

void ARUCO::set_cam_module(CAM* _cam)
{
    if(_cam)
    {
        cam = _cam;
    }
}