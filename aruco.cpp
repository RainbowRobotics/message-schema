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
    if(detect_thread0 == NULL)
    {
        detect_flag0 = true;
        detect_thread0 = new std::thread(&ARUCO::detect_loop, this, 0);
    }

    if (detect_thread1 == NULL)
    {
        detect_flag1 = true;
        detect_thread1 = new std::thread(&ARUCO::detect_loop, this, 1);
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

TIME_IMG ARUCO::load_sim_image(int cam_idx)
{
    QString img_path = QDir::homePath() + QString("/Pictures/img%1.png").arg(cam_idx);
    cv::Mat loaded_img = cv::imread(img_path.toStdString(), cv::IMREAD_COLOR);

    TIME_IMG time_img;
    if(!loaded_img.empty())
    {
        time_img.img = loaded_img;
    }
    else
    {
        printf("[ARUCO_%d] failed to load simulation image\n", cam_idx);
    }
    return time_img;
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

Eigen::Matrix4d ARUCO::se3_exp(cv::Vec3d rvec, cv::Vec3d tvec)
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

    T = T*ZYX_to_TF(0,0,0,0,-90*D2R,-90*D2R);

    // cam to marker
    return T;
}

void ARUCO::detect_loop(int cam_idx)
{
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);

    logger->write_log(QString("[ARUCO] start detection loop%1.").arg(cam_idx), "Green");

    Eigen::Matrix4d cam_tf = (cam_idx == 0) ? string_to_TF(config->CAM_TF_0) : string_to_TF(config->CAM_TF_1);

    while((cam_idx == 0) ? detect_flag0 : detect_flag1)
    {
        CAM_INTRINSIC intrinsic = (cam_idx == 0) ? cam->get_rgb_intrinsic0() : cam->get_rgb_intrinsic1();
        if(cam->is_param_loaded == false && config->SIM_MODE == 0)
        {
            printf("[ARUCO_%d] failed to load cam params\n", cam_idx);

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
        cv::Mat dc(8, 1, CV_64F, d);

        TIME_IMG time_img = (config->SIM_MODE == 1) ? load_sim_image(cam_idx)
                                                    : (cam_idx == 0) ? cam->get_time_img0() : cam->get_time_img1();


        if(time_img.img.empty())
        {
            printf("[ARUCO_%d] time_img empty\n", cam_idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::Mat cur_img;
        cv::Mat gray_img;
        cv::cvtColor(time_img.img, gray_img, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(gray_img, cur_img);

        cv::Mat plot_aruco = time_img.img.clone();
        std::vector<std::vector<cv::Point2f>> detected_uv;
        std::vector<int> detected_id;
        cv::aruco::detectMarkers(cur_img, dictionary, detected_uv, detected_id, parameters);

        if(detected_id.size() != 1)
        {
            // printf("[ARUCO_0] one more marker\n");
            mtx.lock();
            if(cam_idx == 0)
            {
                cur_aruco_img0 = plot_aruco.clone();
            }
            else if(cam_idx == 1)
            {
                cur_aruco_img1 = plot_aruco.clone();
            }
            mtx.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        cv::circle(plot_aruco, detected_uv[0][0], 10, cv::Scalar(0,0,255), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][1], 10, cv::Scalar(0,255,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][2], 10, cv::Scalar(255,0,0), -1, cv::LINE_AA);
        cv::circle(plot_aruco, detected_uv[0][3], 10, cv::Scalar(0,255,255), -1, cv::LINE_AA);

        std::vector<cv::Point3f> matched_xyzs = make_obj_pts();
        std::vector<cv::Point2f> matched_uvs;
        // if(check_regist_aruco(QString::number(detected_id[0]), matched_xyzs))
        {
            // extract information from array
            std::vector<cv::Point2f> uvs;
            for(int p = 0; p < 4; p++)
            {
                double u = (double)detected_uv[0][p].x;
                double v = (double)detected_uv[0][p].y;

                uvs.push_back(cv::Point2f(u,v));
            }

            matched_uvs.insert(matched_uvs.end(), uvs.begin(), uvs.end());
        }

        // debug
        for(int i = 0; i < 4; i++)
        {
            printf("uv:(%f, %f) , xyz:(%f, %f, %f)\n", matched_uvs[i].x, matched_uvs[i].y, matched_xyzs[i].x, matched_xyzs[i].y, matched_xyzs[i].z);
        }
        for(int i = 0; i < (int)detected_id.size(); i++)
        {
            printf("detected id: %d, size:%d\n", detected_id[i], (int)detected_id.size());
        }
        printf("img size(%d, %d)\n", time_img.img.cols, time_img.img.rows);

        // not detected uv or no matching points
        if(matched_xyzs.size() == 0 || matched_uvs.size() == 0 || matched_xyzs.size() != matched_uvs.size())
        {
            printf("[ARUCO_%d] not detected uv or no matching points\n", cam_idx);
            printf("[ARUCO_%d] matched_xyzs.size(): %d, matched_uvs.size(): %d\n)", cam_idx, (int)matched_xyzs.size(), (int)matched_uvs.size());
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        // estimate extrinsic
        cv::Mat rvec, tvec;
        // cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);
        if(cv::solvePnP(matched_xyzs, matched_uvs, cm, dc, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
        {
            cv::solvePnPRefineVVS(matched_xyzs, matched_uvs, cm, dc, rvec, tvec);
            cv::drawFrameAxes(plot_aruco, cm, dc, rvec, tvec, 0.1, 30);

            std::cout << "tvec\n" << tvec << std::endl;
        }
        else
        {
            printf("[ARUCO_%d] solve pnp failed\n", cam_idx);
        }

        // if not solve pnp
        if(rvec.empty() || tvec.empty())
        {
            printf("[ARUCO_%d] rvec or tvec empty\n", cam_idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        Eigen::Matrix4d cam_to_marker = se3_exp(rvec, tvec);
        refine_pose(cam_to_marker);

        Eigen::Matrix4d robot_to_marker = cam_tf * cam_to_marker;

        TIME_POSE_ID aruco_tpi;
        aruco_tpi.id = detected_id[0];
        aruco_tpi.t = time_img.t;
        aruco_tpi.tf = robot_to_marker;

        mtx.lock();
        if(aruco_tpi.t > cur_tpi.t || config->SIM_MODE == 1)
        {
            cur_tpi = aruco_tpi;
        }

        // img
        if(cam_idx == 0)
        {
            cur_aruco_img0 = plot_aruco.clone();
        }
        else if(cam_idx == 1)
        {
            cur_aruco_img1 = plot_aruco.clone();
        }
        mtx.unlock();

        // result
        std::cout << "[ARUCO_" << cam_idx << "] aruco id:" << detected_id[0] << "\n"
                  << "robot_to_marker:\n" << robot_to_marker << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}




