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

cv::Mat ARUCO::get_aruco0()
{
    mtx.lock();
    cv::Mat _cur_aruco0 = cur_aruco0.clone();
    mtx.unlock();

    return _cur_aruco0;
}

cv::Mat ARUCO::get_aruco1()
{
    mtx.lock();
    cv::Mat _cur_aruco1 = cur_aruco1.clone();
    mtx.unlock();

    return _cur_aruco1;
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

Eigen::Matrix4d ARUCO::m2e(cv::Mat rvec, cv::Mat tvec)
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

    return T;
}

void ARUCO::detect_loop0()
{
    // set robot to camera tf
    Eigen::Matrix4d cam_tf = string_to_TF(config->CAM_TF_0);

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    logger->write_log("[ARUCO] start detection loop.", "Green");
    while(detect_flag0)
    {
        // get intrinic
        CAM_INTRINSIC intrinsic = cam->get_rgb_intrinsic0();

        if(intrinsic.fx == -1)
        {
            logger->write_log("[ARUCO] no intrinsic info.", "Red");
            return;
        }
        // std::cout << "[ARUCO] intrinsic0(fx,fy,cx,cy):"<< intrinsic.fx << "," << intrinsic.fy << "," << intrinsic.cx << "," << intrinsic.cy << std::endl;

        double fx = intrinsic.fx;
        double fy = intrinsic.fy;
        double cx = intrinsic.cx;
        double cy = intrinsic.cy;

        if(config->SIM_MODE)
        {
            fx = 721.142;
            fy = 721.142;
            cx = 636.912;
            cy = 359.406;
            // printf("intrinsic: %f, %f, %f, %f\n",fx, fy, cx, cy);
        }

        cv::Matx33d cm(fx, 0, cx,
                       0, fy, cy,
                       0,  0,  1);

        cv::Mat dc(8,1,cv::DataType<double>::type);
        dc.at<double>(0) = intrinsic.k1; dc.at<double>(1) = intrinsic.k2;
        dc.at<double>(2) = intrinsic.p1; dc.at<double>(3) = intrinsic.p2;
        dc.at<double>(4) = intrinsic.k3; dc.at<double>(5) = intrinsic.k4;
        dc.at<double>(6) = intrinsic.k5; dc.at<double>(7) = intrinsic.k6;

        cv::Mat cur_img;
        if(config->SIM_MODE == 1)
        {
            QString img_path = QDir::homePath() + "/Pictures/img0.png";
            cv::Mat loaded_img = cv::imread(img_path.toStdString(), cv::IMREAD_COLOR);
            if(loaded_img.empty())
            {
                printf("[ERROR] Failed to load img.png\n");
            }
            else
            {
                // printf("[INFO] Loaded img.png\n");
                cur_img = loaded_img;
            }
        }
        else
        {
            cur_img = cam->get_img0();
        }

        if(cur_img.empty())
        {
            printf("[ARUCO] cur_img0 empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }


        cv::Mat plot_aruco = cur_img.clone();
        std::vector<std::vector<cv::Point2f>> detected_uv, rejectedCandidates;
        std::vector<int> detected_id;
        cv::aruco::detectMarkers(cur_img, dictionary, detected_uv, detected_id, parameters, rejectedCandidates, cm, dc);

        if(detected_id.size() != 1)
        {
            //printf("[ARUCO] one more marker\n");

            mtx.lock();
            cur_aruco0 = plot_aruco.clone();
            mtx.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::circle(plot_aruco, detected_uv[0][0], 10, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][1], 10, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][2], 10, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][3], 10, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        // std::vector<cv::Point3d> matched_xyzs;
        std::vector<cv::Point3d> matched_xyzs = {
            cv::Point3d(-0.09, 0.09, 0.0),
            cv::Point3d(0.09, 0.09, 0.0),
            cv::Point3d(0.09, -0.09, 0.0),
            cv::Point3d(-0.09, -0.09, 0.0)
        };

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
            std::cout << "not detected uv or no matching points" << std::endl;
            std::cout << "matched_xyzs.size():" << matched_xyzs.size() << ", matched_uvs.size():" << matched_uvs.size() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // estimate extrinsic
        cv::Mat rvec, tvec;
        cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);

        // if not solve pnp
        if(rvec.empty() || tvec.empty())
        {
            std::cout << "rvec or tvec empty" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        cv::drawFrameAxes(plot_aruco, cm, dc, rvec, tvec, 0.1, 30);
        mtx.lock();
        cur_aruco0 = plot_aruco.clone();
        mtx.unlock();

        // cam to marker
        Eigen::Matrix4d cam_to_marker = m2e(rvec, tvec);
        refine_pose(cam_to_marker);

        // std::cout << "[ARUCO] cam_to_marker:\n" << cam_to_marker << std::endl;
        Eigen::Matrix4d robot_to_marker = cam_tf * cam_to_marker;

        // result
        std::cout << "[ARUCO] robot_to_marker:\n" << robot_to_marker << std::endl;
        is_detect = true;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


void ARUCO::detect_loop1()
{
    /*
    // get intrinic
    CAM_INTRINSIC intrinsic = cam->get_rgb_intrinsic1();
    if(intrinsic.fx == -1)
    {
        logger->write_log("[ARUCO] no intrinsic info.", "Red");
        return;
    }
    std::cout << "[ARUCO] intrinsic0(fx,fy,cx,cy):"<< intrinsic.fx << "," << intrinsic.fy << "," << intrinsic.cx << "," << intrinsic.cy << std::endl;

    double fx = intrinsic.fx;
    double fy = intrinsic.fy;
    double cx = intrinsic.cx;
    double cy = intrinsic.cy;

    if(config->SIM_MODE)
    {
        fx = 721.142;
        fy = 721.142;
        cx = 636.912;
        cy = 359.406;
        printf("intrinsic: %f, %f, %f, %f\n",fx, fy, cx, cy);
    }


    cv::Matx33d cm(fx, 0, cx,
                   0, fy, cy,
                   0,  0,  1);

    cv::Mat dc(8,1,cv::DataType<double>::type);
    dc.at<double>(0) = intrinsic.k1; dc.at<double>(1) = intrinsic.k2;
    dc.at<double>(2) = intrinsic.p1; dc.at<double>(3) = intrinsic.p2;
    dc.at<double>(4) = intrinsic.k3; dc.at<double>(5) = intrinsic.k4;
    dc.at<double>(6) = intrinsic.k5; dc.at<double>(7) = intrinsic.k6;

    // set robot to camera tf
    Eigen::Matrix4d cam_tf = string_to_TF(config->CAM_TF_1);
    Eigen::Matrix3d R_cam = cam_tf.block<3,3>(0,0);
    Eigen::Vector3d t_cam = cam_tf.block<3,1>(0,3);

    // set inverse matrix
    Eigen::Matrix3d R_cam_inv = R_cam.transpose();
    Eigen::Vector3d t_cam_inv = -R_cam_inv * t_cam;

    // set camera to robot tf
    Eigen::Matrix4d cam_tf1;
    cam_tf1.setIdentity();
    cam_tf1.block<3,3>(0,0) = R_cam_inv;
    cam_tf1.block<3,1>(0,3) = t_cam_inv;

    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    logger->write_log("[ARUCO] start detection loop.", "Green");
    while(detect_flag0)
    {
        cv::Mat cur_img;
        if(config->SIM_MODE == 1)
        {
            QString img_path = QDir::homePath() + "/Pictures/img1.png";
            cv::Mat loaded_img = cv::imread(img_path.toStdString(), cv::IMREAD_COLOR);
            if(loaded_img.empty())
            {
                printf("[ERROR] Failed to load img.png\n");
            }
            else
            {
                // printf("[INFO] Loaded img.png\n");
                cur_img = loaded_img;
            }
        }
        else
        {
            cur_img = cam->get_img1();
        }

        if(cur_img.empty())
        {
            printf("[ARUCO] cur_img1 empty\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }


        cv::Mat plot_aruco = cur_img.clone();
        std::vector<std::vector<cv::Point2f>> detected_uv, rejectedCandidates;
        std::vector<int> detected_id;
        cv::aruco::detectMarkers(cur_img, dictionary, detected_uv, detected_id, parameters, rejectedCandidates, cm, dc);

        if(detected_id.size() != 1)
        {
            //printf("[ARUCO] one more marker\n");

            mtx.lock();
            cur_aruco1 = plot_aruco.clone();
            mtx.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::circle(plot_aruco, detected_uv[0][0], 10, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][1], 10, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][2], 10, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][3], 10, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        mtx.lock();
        cur_aruco1 = plot_aruco.clone();
        mtx.unlock();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    */
}


