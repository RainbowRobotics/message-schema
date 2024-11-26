#include "aruco.h"

ARUCO::ARUCO(QObject *parent)
    : QObject{parent}
{
}

ARUCO::~ARUCO()
{
    if(detect_thread0 != NULL)
    {
        detect_flag0 = false;
        detect_thread0->join();
        detect_thread0 = NULL;
    }

    if(detect_thread1 != NULL)
    {
        detect_flag1 = false;
        detect_thread1->join();
        detect_thread1 = NULL;
    }
}

void ARUCO::init()
{
    // if(config->SIM_MODE == 1)
    // {
    //     printf("[ARUCO] simulation mode\n");
    //     return;
    // }
    if(detect_thread0 == NULL)
    {
        detect_flag0 = true;
        detect_thread0 = new std::thread(&ARUCO::detect_loop0, this);
    }

    if(detect_thread1 == NULL)
    {
        detect_flag1 = true;
        detect_thread1 = new std::thread(&ARUCO::detect_loop1, this);
    }

}

TIME_POSE_ID ARUCO::get_cur_tpi()
{
    mtx.lock();
    TIME_POSE_ID _cur_tpi = cur_tpi;
    mtx.unlock();

    return _cur_tpi;
}

cv::Mat ARUCO::get_plot_img0()
{
    mtx.lock();
    cv::Mat _cur_aruco_img0 = cur_aruco_img0.clone();
    mtx.unlock();

    return _cur_aruco_img0;
}

cv::Mat ARUCO::get_plot_img1()
{
    mtx.lock();
    cv::Mat _cur_aruco_img1 = cur_aruco_img1.clone();
    mtx.unlock();

    return _cur_aruco_img1;
}

std::vector<cv::Point3d> ARUCO::make_obj_pts()
{
    std::vector<cv::Point3d> obj_pts(4);

    obj_pts[0] = cv::Point3d(-marker_size/2,  marker_size/2, 0.0);
    obj_pts[1] = cv::Point3d( marker_size/2,  marker_size/2, 0.0);
    obj_pts[2] = cv::Point3d( marker_size/2, -marker_size/2, 0.0);
    obj_pts[3] = cv::Point3d(-marker_size/2, -marker_size/2, 0.0);

    return obj_pts;
}

bool ARUCO::check_regist_aruco(QString id, std::vector<cv::Point3d>& corners)
{
    if(aruco_metric.find(id) == aruco_metric.end())
    {
        printf("[ARUCO] no found\n");
        return false;
    }

    corners = aruco_metric[id];
    return true;
}

Eigen::Matrix4d ARUCO::se3_exp(cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    Eigen::Matrix3d eR;
    eR << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
          R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
          R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);

    Eigen::Vector3d et;
    et << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);

    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = eR;
    T.block<3,1>(0,3) = et;

    T = T*ZYX_to_TF(0,0,0,0,-90*D2R,-90*D2R);

    return T;
}

void ARUCO::detect_loop0()
{
    // set robot to camera tf
    Eigen::Matrix4d cam_tf = string_to_TF(config->CAM_TF_0);

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    logger->write_log("[ARUCO] start detection loop0.", "Green");
    while(detect_flag0)
    {
        // get intrinic
        CAM_INTRINSIC intrinsic = cam->get_rgb_intrinsic0();
        if(cam->is_param_loaded == false && config->SIM_MODE == 0)
        {
            printf("[ARUCO_0] failed to load cam params(cam0)\n");

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(config->SIM_MODE)
        {
            intrinsic.fx = 721.142;
            intrinsic.fy = 721.142;
            intrinsic.cx = 636.912;
            intrinsic.cy = 359.406;
        }
        // std::cout << "[ARUCO] intrinsic0(fx,fy,cx,cy):"<< intrinsic.fx << "," << intrinsic.fy << "," << intrinsic.cx << "," << intrinsic.cy << std::endl;

        cv::Matx33d cm(intrinsic.fx, 0, intrinsic.cx,
                       0, intrinsic.fy, intrinsic.cy,
                       0,  0,  1);

        double d[8] = {intrinsic.k1, intrinsic.k2, intrinsic.p1,intrinsic.p2, intrinsic.k3, intrinsic.k4, intrinsic.k5, intrinsic.k6};
        cv::Mat dc(8, 1, cv::DataType<double>::type, d);

        // cv::Mat cur_img;
        TIME_IMG time_img;
        if(config->SIM_MODE == 1)
        {
            QString img_path = QDir::homePath() + "/Pictures/img0.png";
            cv::Mat loaded_img = cv::imread(img_path.toStdString(), cv::IMREAD_COLOR);
            if(loaded_img.empty())
            {
                printf("[ARUCO_0] failed to load img.png\n");
            }
            else
            {
                // printf("[INFO] Loaded img.png\n");
                time_img.img = loaded_img;
            }
        }
        else
        {
            time_img = cam->get_time_img0();
        }

        if(time_img.img.empty())
        {
            printf("[ARUCO_0] time_img empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }


        cv::Mat plot_aruco = time_img.img.clone();
        std::vector<std::vector<cv::Point2f>> detected_uv, rejectedCandidates;
        std::vector<int> detected_id;
        cv::aruco::detectMarkers(time_img.img, dictionary, detected_uv, detected_id, parameters, rejectedCandidates, cm, dc);

        if(detected_id.size() != 1)
        {
            // printf("[ARUCO_0] one more marker\n");
            mtx.lock();
            cur_aruco_img0 = plot_aruco.clone();
            mtx.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::circle(plot_aruco, detected_uv[0][0], 10, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][1], 10, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][2], 10, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][3], 10, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        // std::vector<cv::Point3d> matched_xyzs;
        std::vector<cv::Point3d> matched_xyzs = make_obj_pts();
        std::vector<cv::Point2d> matched_uvs;
        // if(check_regist_aruco(QString::number(detected_id[0]), matched_xyzs))
        {
            // extract information from array
            std::vector<cv::Point2i> uvs;
            for(int p = 0; p < 4; p++)
            {
                double u = (double)detected_uv[0][p].x;
                double v = (double)detected_uv[0][p].y;

                uvs.push_back(cv::Point2d(u,v));
            }

            matched_uvs.insert(matched_uvs.end(), uvs.begin(), uvs.end());
        }

        // not detected uv or no matching points
        if(matched_xyzs.size() == 0 || matched_uvs.size() == 0 || matched_xyzs.size() != matched_uvs.size())
        {
            printf("[ARUCO_0] not detected uv or no matching points\n");
            printf("[ARUCO_0] matched_xyzs.size(): %d, matched_uvs.size(): %d\n)", (int)matched_xyzs.size(), (int)matched_uvs.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // estimate extrinsic
        cv::Mat rvec, tvec;
        cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);

        // if not solve pnp
        if(rvec.empty() || tvec.empty())
        {
            printf("[ARUCO_0] rvec or tvec empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::drawFrameAxes(plot_aruco, cm, dc, rvec, tvec, 0.1, 30);

        mtx.lock();
        cur_aruco_img0 = plot_aruco.clone();
        mtx.unlock();

        // cam to marker
        Eigen::Matrix4d cam_to_marker = se3_exp(rvec, tvec);
        // refine_pose(cam_to_marker);

        // std::cout << "[ARUCO] cam_to_marker:\n" << cam_to_marker << std::endl;
        Eigen::Matrix4d robot_to_marker = cam_tf * cam_to_marker;

        TIME_POSE_ID aruco_tpi;
        aruco_tpi.id = detected_id[0];
        aruco_tpi.t = time_img.t;
        aruco_tpi.tf = robot_to_marker;

        mtx.lock();
        if(aruco_tpi.t > cur_tpi.t)
        {
            cur_tpi = aruco_tpi;
        }
        mtx.unlock();

        // result
        std::cout << "[ARUCO_0] ID:" << detected_id[0] << "\n" << "[ARUCO_0] robot_to_marker:\n" << robot_to_marker << std::endl;

        // is_detect0 = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void ARUCO::detect_loop1()
{
    // set robot to camera tf
    Eigen::Matrix4d cam_tf = string_to_TF(config->CAM_TF_1);

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    logger->write_log("[ARUCO] start detection loop1.", "Green");
    while(detect_flag1)
    {
        // get intrinic
        CAM_INTRINSIC intrinsic = cam->get_rgb_intrinsic1();

        if(cam->is_param_loaded == false && config->SIM_MODE == 0)
        {
            printf("[ARUCO_1] failed to load cam params\n");

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(config->SIM_MODE)
        {
            intrinsic.fx = 721.142;
            intrinsic.fy = 721.142;
            intrinsic.cx = 636.912;
            intrinsic.cy = 359.406;
        }
        // std::cout << "[ARUCO] intrinsic0(fx,fy,cx,cy):"<< intrinsic.fx << "," << intrinsic.fy << "," << intrinsic.cx << "," << intrinsic.cy << std::endl;

        cv::Matx33d cm(intrinsic.fx, 0, intrinsic.cx,
                       0, intrinsic.fy, intrinsic.cy,
                       0,  0,  1);

        double d[8] = {intrinsic.k1, intrinsic.k2, intrinsic.p1,intrinsic.p2, intrinsic.k3, intrinsic.k4, intrinsic.k5, intrinsic.k6};
        cv::Mat dc(8, 1, cv::DataType<double>::type, d);

        // cv::Mat cur_img;
        TIME_IMG time_img;
        if(config->SIM_MODE == 1)
        {
            QString img_path = QDir::homePath() + "/Pictures/img1.png";
            cv::Mat loaded_img = cv::imread(img_path.toStdString(), cv::IMREAD_COLOR);
            if(loaded_img.empty())
            {
                printf("[ARUCO_1] failed to load img.png\n");
            }
            else
            {
                time_img.img = loaded_img;
            }
        }
        else
        {
            time_img = cam->get_time_img1();
        }

        if(time_img.img.empty())
        {
            printf("[ARUCO_1] time_img empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }


        cv::Mat plot_aruco = time_img.img.clone();
        std::vector<std::vector<cv::Point2f>> detected_uv, rejectedCandidates;
        std::vector<int> detected_id;
        cv::aruco::detectMarkers(time_img.img, dictionary, detected_uv, detected_id, parameters, rejectedCandidates, cm, dc);

        if(detected_id.size() != 1)
        {
            // printf("[ARUCO_1] one more marker\n");
            mtx.lock();
            cur_aruco_img1 = plot_aruco.clone();
            mtx.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::circle(plot_aruco, detected_uv[0][0], 10, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][1], 10, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][2], 10, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][3], 10, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        std::vector<cv::Point3d> matched_xyzs = make_obj_pts();
        std::vector<cv::Point2d> matched_uvs;
        // if(check_regist_aruco(QString::number(detected_id[0]), matched_xyzs))
        {
            // extract information from array
            std::vector<cv::Point2i> uvs;
            for(int p = 0; p < 4; ++p)
            {
                double u = (double)detected_uv[0][p].x;
                double v = (double)detected_uv[0][p].y;

                uvs.push_back(cv::Point2d(u,v));
            }

            matched_uvs.insert(matched_uvs.end(), uvs.begin(), uvs.end());
        }

        // not detected uv or no matching points
        if(matched_xyzs.size() == 0 || matched_uvs.size() == 0 || matched_xyzs.size() != matched_uvs.size())
        {
            printf("[ARUCO_1] not detected uv or no matching points\n");
            printf("[ARUCO_1] matched_xyzs.size(): %d, matched_uvs.size(): %d\n)", (int)matched_xyzs.size(), (int)matched_uvs.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // estimate extrinsic
        cv::Mat rvec, tvec;
        cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);

        // if not solve pnp
        if(rvec.empty() || tvec.empty())
        {
            printf("[ARUCO_1] rvec or tvec empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::drawFrameAxes(plot_aruco, cm, dc, rvec, tvec, 0.1, 30);
        mtx.lock();
        cur_aruco_img1 = plot_aruco.clone();
        mtx.unlock();

        // cam to marker
        Eigen::Matrix4d cam_to_marker = se3_exp(rvec, tvec);
        // refine_pose(cam_to_marker);

        // std::cout << "[ARUCO] cam_to_marker:\n" << cam_to_marker << std::endl;
        Eigen::Matrix4d robot_to_marker = cam_tf * cam_to_marker;

        TIME_POSE_ID aruco_tpi;
        aruco_tpi.id = detected_id[0];
        aruco_tpi.t = time_img.t;
        aruco_tpi.tf = robot_to_marker;

        mtx.lock();
        if(aruco_tpi.t > cur_tpi.t)
        {
            cur_tpi = aruco_tpi;
        }
        mtx.unlock();

        // result
        std::cout << "[ARUCO_1] ID:" << detected_id[0] << "\n" << "[ARUCO_1] robot_to_marker:\n" << robot_to_marker << std::endl;

        // is_detect1 = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}




