 #include "cam.h"

CAM::CAM(QObject *parent) : QObject(parent)
{
    for(int p = 0; p < 4; p++)
    {
        cam_tf[p].setIdentity();
    }
}

CAM::~CAM()
{
    if(grab_thread != NULL)
    {
        grab_flag = false;
        grab_thread->join();
        grab_thread = NULL;
    }
}

void CAM::open()
{
    // check simulation mode
    if(config->SIM_MODE == 1)
    {
        printf("[CAM] simulation mode\n");
        return;
    }

    // start grab loop
    if(grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&CAM::grab_loop, this);
    }    
}

cv::Mat CAM::get_img(int cam_idx)
{
    mtx.lock();
    cv::Mat res = cur_img[cam_idx].clone();
    mtx.unlock();

    return res;
}

TIME_IMG CAM::get_time_img(int cam_idx)
{
    mtx.lock();
    TIME_IMG res = cur_time_img[cam_idx];
    mtx.unlock();

    return res;
}

TIME_PTS CAM::get_scan0()
{
    mtx.lock();
    TIME_PTS res = cur_scan0;
    mtx.unlock();

    return res;
}

TIME_PTS CAM::get_scan1()
{
    mtx.lock();
    TIME_PTS res = cur_scan1;
    mtx.unlock();

    return res;
}

CAM_INTRINSIC CAM::get_intrinsic(int cam_idx)
{
    mtx.lock();
    CAM_INTRINSIC res = intrinsic[cam_idx];
    mtx.unlock();

    return res;
}

Eigen::Matrix4d CAM::get_extrinsic(int cam_idx)
{
    mtx.lock();
    Eigen::Matrix4d res = cam_tf[cam_idx];
    mtx.unlock();

    return res;
}

#if defined(USE_SRV)
void CAM::grab_loop()
{
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);

    ob::Context ctx;
    auto dev_list = ctx.queryDeviceList();
    int dev_count = dev_list->deviceCount();
    if(dev_count != 2)
    {
        printf("[CAM] USE_SRV, need two camera, dev_count :%d\n", dev_count);
        return;
    }

    // set property
    int cam_idx0 = 0;
    int cam_idx1 = 1;

    int serial_valid_cnt = 0;
    for(int p = 0; p < dev_count; p++)
    {
        QString sn = dev_list->getDevice(p)->getDeviceInfo()->serialNumber();
        printf("[CAM] sn : %s\n", sn.toLocal8Bit().data());

        if(config->CAM_SERIAL_NUMBER_0 == sn)
        {
            // left camera
            cam_idx0 = p;
            serial_valid_cnt++;
            printf("[CAM] camera serial0 valid\n");
        }
        else if(config->CAM_SERIAL_NUMBER_1 == sn)
        {
            cam_idx1 = p;
            serial_valid_cnt++;
            printf("[CAM] camera serial1 valid\n");
        }
    }

    if(serial_valid_cnt != 2)
    {
        printf("[CAM] camera serials invalid\n");
        return;
    }

    // set cam0
    auto dev0 = dev_list->getDevice(cam_idx0);
    dev0->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev0->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

    std::shared_ptr<ob::Pipeline> pipe0 = std::make_shared<ob::Pipeline>(dev0);
    Eigen::Matrix4d TF0 = string_to_TF(config->CAM_TF_0);

    auto depth_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_DEPTH);

    /*
    for(size_t p = 0; p < depth_profile_list0->count(); p++)
    {
        auto profile = depth_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
        printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    }
    */


    auto color_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_COLOR);

    /*
    for(size_t p = 0; p < color_profile_list0->count(); p++)
    {
        auto profile = color_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
        printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    }
    */

    auto depth_profile0 = depth_profile_list0->getProfile(31)->as<ob::VideoStreamProfile>();
    printf("[CAM] depth_profile(31), w:%d, h:%d, fps:%d, format:%d\n", depth_profile0->width(), depth_profile0->height(), depth_profile0->fps(), depth_profile0->format());

    //auto color_profile0 = color_profile_list0->getProfile(123)->as<ob::VideoStreamProfile>();
    //printf("[CAM] color_profile(119), w:%d, h:%d, fps:%d, format:%d\n", color_profile0->width(), color_profile0->height(), color_profile0->fps(), color_profile0->format());

    auto color_profile0 = color_profile_list0->getProfile(83)->as<ob::VideoStreamProfile>();
    printf("[CAM] color_profile(83), w:%d, h:%d, fps:%d, format:%d\n", color_profile0->width(), color_profile0->height(), color_profile0->fps(), color_profile0->format());

    std::shared_ptr<ob::Config> config0 = std::make_shared<ob::Config>();
    config0->disableAllStream();
    config0->enableStream(depth_profile0);
    config0->enableStream(color_profile0);
    config0->setAlignMode(ALIGN_DISABLE);

    pipe0->start(config0, [&](std::shared_ptr<ob::FrameSet> fs)
    {
        try
        {
            if(fs->depthFrame() != nullptr)
            {
                uint64_t ts = fs->depthFrame()->systemTimeStamp();
                double t = (double)ts/1000.0 - st_time_for_get_time;

                // get point cloud
                ob::PointCloudFilter point_cloud;
                auto camera_param = pipe0->getCameraParam();

                point_cloud.setCameraParam(camera_param);
                point_cloud.setCreatePointFormat(OB_FORMAT_POINT);

                auto scale = fs->depthFrame()->getValueScale();
                point_cloud.setPositionDataScaled(scale);

                std::shared_ptr<ob::Frame> frame = point_cloud.process(fs);
                OBPoint *point = (OBPoint *)frame->data();

                int w = fs->depthFrame()->width();
                int h = fs->depthFrame()->height();

                std::vector<Eigen::Vector3d> pts;
                for(int i = 0; i < h; i++)
                {
                    for(int j = 0; j < w; j++)
                    {
                        double x = point[i*w+j].x/1000.0;
                        double y = point[i*w+j].y/1000.0;
                        double z = point[i*w+j].z/1000.0;
                        if(x != 0 || y != 0 || z != 0)
                        {
                            Eigen::Vector3d P(x,y,z);
                            Eigen::Vector3d _P = TF0.block(0,0,3,3)*P + TF0.block(0,3,3,1);

                            if(_P[0] > config->ROBOT_SIZE_X[0] && _P[0] < config->ROBOT_SIZE_X[1] &&
                               _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                            {
                                continue;
                            }

                            if(_P[2] < config->CAM_HEIGHT_MIN || _P[2] > config->CAM_HEIGHT_MAX)
                            {
                                continue;
                            }

                            // orthogonal projection
                            _P[2] = 0;
                            pts.push_back(_P);
                        }
                    }
                }

                // voxel filtering
                pts = voxel_filtering(pts, config->SLAM_VOXEL_SIZE);

                // update scan
                TIME_PTS scan;
                scan.t = t;
                scan.pts = pts;

                mtx.lock();
                cur_scan0 = scan;
                mtx.unlock();
            }

            if(fs->colorFrame() != nullptr)
            {
                if(is_connected[0] == false)
                {
                    is_connected[0] = true;
                }

                uint64_t ts = fs->colorFrame()->systemTimeStamp();
                double t = (double)ts/1000.0 - st_time_for_get_time;

                // get color image
                std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();

                // convert opencv image
                cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

                cv::Mat img;
                cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);

                if(!img.empty())
                {
                    TIME_IMG time_img;
                    time_img.t = t;
                    time_img.img = img.clone();

                    // flip for plot
                    cv::Mat plot_img;
                    cv::resize(img, plot_img, cv::Size(160, 90));
                    cv::flip(plot_img, plot_img, -1);

                    mtx.lock();
                    cur_img[0] = plot_img.clone();
                    cur_time_img[0] = time_img;
                    mtx.unlock();
                }
            }
        }
        catch(const ob::Error &e)
        {
            QString str = QString::fromLocal8Bit(e.getMessage());
            logger->write_log("[CAM] " + str, "Red");
        }
    });

    // set cam1
    auto dev1 = dev_list->getDevice(cam_idx1);
    dev1->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev1->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

    std::shared_ptr<ob::Pipeline> pipe1 = std::make_shared<ob::Pipeline>(dev1);
    Eigen::Matrix4d TF1 = string_to_TF(config->CAM_TF_1);

    auto depth_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_DEPTH);
    auto color_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_COLOR);

    auto depth_profile1 = depth_profile_list1->getProfile(31)->as<ob::VideoStreamProfile>();
    //auto color_profile1 = color_profile_list1->getProfile(123)->as<ob::VideoStreamProfile>();
    auto color_profile1 = color_profile_list1->getProfile(83)->as<ob::VideoStreamProfile>();

    std::shared_ptr<ob::Config> config1 = std::make_shared<ob::Config>();
    config1->disableAllStream();
    config1->enableStream(depth_profile1);
    config1->enableStream(color_profile1);
    config1->setAlignMode(ALIGN_DISABLE);

    pipe1->start(config1, [&](std::shared_ptr<ob::FrameSet> fs)
    {
        try
        {
            if(fs->depthFrame() != nullptr)
            {
                uint64_t ts = fs->depthFrame()->systemTimeStamp();
                double t = (double)ts/1000.0 - st_time_for_get_time;

                // get point cloud
                ob::PointCloudFilter point_cloud;
                auto camera_param = pipe1->getCameraParam();

                point_cloud.setCameraParam(camera_param);
                point_cloud.setCreatePointFormat(OB_FORMAT_POINT);

                auto scale = fs->depthFrame()->getValueScale();
                point_cloud.setPositionDataScaled(scale);

                std::shared_ptr<ob::Frame> frame = point_cloud.process(fs);
                OBPoint *point = (OBPoint *)frame->data();

                int w = fs->depthFrame()->width();
                int h = fs->depthFrame()->height();

                std::vector<Eigen::Vector3d> pts;
                for(int i = 0; i < h; i++)
                {
                    for(int j = 0; j < w; j++)
                    {
                        double x = point[i*w+j].x/1000.0;
                        double y = point[i*w+j].y/1000.0;
                        double z = point[i*w+j].z/1000.0;
                        if(x != 0 || y != 0 || z != 0)
                        {
                            Eigen::Vector3d P(x,y,z);
                            Eigen::Vector3d _P = TF1.block(0,0,3,3)*P + TF1.block(0,3,3,1);

                            if(_P[0] > config->ROBOT_SIZE_X[0] && _P[0] < config->ROBOT_SIZE_X[1] &&
                               _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                            {
                                continue;
                            }

                            if(_P[2] < config->CAM_HEIGHT_MIN || _P[2] > config->CAM_HEIGHT_MAX)
                            {
                                continue;
                            }

                            // orthogonal projection
                            _P[2] = 0;
                            pts.push_back(_P);
                        }
                    }
                }

                // voxel filtering
                pts = voxel_filtering(pts, config->SLAM_VOXEL_SIZE);

                // update scan
                TIME_PTS scan;
                scan.t = t;
                scan.pts = pts;

                mtx.lock();
                cur_scan1 = scan;
                mtx.unlock();
            }

            if(fs->colorFrame() != nullptr)
            {
                if(is_connected[1] == false)
                {
                    is_connected[1] = true;
                }

                uint64_t ts = fs->colorFrame()->systemTimeStamp();
                double t = (double)ts/1000.0 - st_time_for_get_time;

                // get color image
                std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();

                // convert opencv image
                cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

                cv::Mat img;
                cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);
                //cv::flip(img, img, -1);

                if(!img.empty())
                {
                    TIME_IMG time_img;
                    time_img.t = t;
                    time_img.img = img.clone();

                    cv::Mat plot_img;
                    cv::resize(img, plot_img, cv::Size(160, 90));

                    mtx.lock();
                    cur_img[1] = plot_img.clone();
                    cur_time_img[1] = time_img;
                    mtx.unlock();
                }
            }
        }
        catch(const ob::Error &e)
        {
            QString str = QString::fromLocal8Bit(e.getMessage());
            logger->write_log("[CAM] " + str, "Red");
        }
    });

    // get intrinsic cam 0
    {
        mtx.lock();
        auto camera_param = pipe0->getCameraParam();
        intrinsic[0].fx = camera_param.rgbIntrinsic.fx;
        intrinsic[0].fy = camera_param.rgbIntrinsic.fy;
        intrinsic[0].cx = camera_param.rgbIntrinsic.cx;
        intrinsic[0].cy = camera_param.rgbIntrinsic.cy;

        intrinsic[0].k1 = camera_param.rgbDistortion.k1;
        intrinsic[0].k2 = camera_param.rgbDistortion.k2;
        intrinsic[0].k3 = camera_param.rgbDistortion.k3;
        intrinsic[0].k4 = camera_param.rgbDistortion.k4;
        intrinsic[0].k5 = camera_param.rgbDistortion.k5;
        intrinsic[0].k6 = camera_param.rgbDistortion.k6;
        intrinsic[0].p1 = camera_param.rgbDistortion.p1;
        intrinsic[0].p2 = camera_param.rgbDistortion.p2;
        intrinsic[0].coef_num = 8;

        cam_tf[0] = string_to_TF(config->CAM_TF_0);
        mtx.unlock();
    }
    printf("[CAM] intrinsic0, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[0].fx, intrinsic[0].fy, intrinsic[0].cx, intrinsic[0].cy);

    // get intrinsic cam 1
    {
        mtx.lock();
        auto camera_param = pipe1->getCameraParam();
        intrinsic[1].fx = camera_param.rgbIntrinsic.fx;
        intrinsic[1].fy = camera_param.rgbIntrinsic.fy;
        intrinsic[1].cx = camera_param.rgbIntrinsic.cx;
        intrinsic[1].cy = camera_param.rgbIntrinsic.cy;

        intrinsic[1].k1 = camera_param.rgbDistortion.k1;
        intrinsic[1].k2 = camera_param.rgbDistortion.k2;
        intrinsic[1].k3 = camera_param.rgbDistortion.k3;
        intrinsic[1].k4 = camera_param.rgbDistortion.k4;
        intrinsic[1].k5 = camera_param.rgbDistortion.k5;
        intrinsic[1].k6 = camera_param.rgbDistortion.k6;
        intrinsic[1].p1 = camera_param.rgbDistortion.p1;
        intrinsic[1].p2 = camera_param.rgbDistortion.p2;
        intrinsic[1].coef_num = 8;

        cam_tf[1] = string_to_TF(config->CAM_TF_1);
        mtx.unlock();
    }
    printf("[CAM] intrinsic1, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[1].fx, intrinsic[1].fy, intrinsic[1].cx, intrinsic[1].cy);

    #ifdef USE_OCAM

    std::vector<Withrobot::usb_device_info> ocam_dev_list;
    Withrobot::get_usb_device_info_list(ocam_dev_list);
    Withrobot::Camera* ocam_pipe0 = NULL; Withrobot::Camera* ocam_pipe1 = NULL;
    Withrobot::camera_format ocam_format;
    ocam_format.width = 1280;
    ocam_format.height = 720;
    ocam_format.pixformat = Withrobot::fourcc_to_pixformat('G','B','G','R');
    ocam_format.frame_rate = 15;
    QString ocam_format_name = "8-bit Bayer GRGR/BGBG 1280 x 720 15 fps";

    if(ocam_dev_list.size() >= 1)
    {
        for(size_t p=0; p<ocam_dev_list.size(); p++)
        {
            if(config->CAM_SERIAL_NUMBER_2 != "")
            {
                if(ocam_dev_list[p].dev_node == config->CAM_SERIAL_NUMBER_2.toStdString())
                {
                    ocam_pipe0 = new Withrobot::Camera(ocam_dev_list[p].dev_node.c_str(), &ocam_format, ocam_format_name.toStdString().c_str());
                    ocam_pipe0->product_name = ocam_dev_list[p].product;
                    ocam_pipe0->set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 15);

                    // get intrinsic cam 2
                    mtx.unlock();
                    intrinsic[2] = string_to_intrinsic(config->CAM_INTRINSIC_2);
                    intrinsic[2].coef_num = 4;

                    cam_tf[2] = string_to_TF(config->CAM_TF_2);
                    mtx.unlock();
                    printf("[CAM] intrinsic2, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[2].fx, intrinsic[2].fy, intrinsic[2].cx, intrinsic[2].cy);

                    /*std::vector<const char*> list;
                    int cnt = ocam_pipe0->get_valid_image_format_list(list);
                    for(int r=0; r<cnt; r++)
                    {
                        std::cout << list[r] << std::endl;
                        std::vector<const char*> list2;
                        int cnt2 = ocam_pipe0->get_valid_resolution_list(list[r], list2);
                        for(int r2=0; r2<cnt2; r2++)
                        {
                            std::cout << list2[r2] << std::endl;
                            std::vector<const char*> list3;
                            int cnt3 = ocam_pipe0->get_valid_ratio_list(list2[r2], list3);
                            for(int r3=0; r3<cnt3; r3++)
                            {
                                std::cout << list3[r3] << std::endl;
                            }
                        }
                    }*/

                    ocam_pipe0->start();
                }
            }

            if(config->CAM_SERIAL_NUMBER_3 != "")
            {
                if(ocam_dev_list[p].dev_node == config->CAM_SERIAL_NUMBER_3.toStdString())
                {
                    ocam_pipe1 = new Withrobot::Camera(ocam_dev_list[p].dev_node.c_str(), &ocam_format, ocam_format_name.toStdString().c_str());
                    ocam_pipe1->product_name = ocam_dev_list[p].product;
                    ocam_pipe1->set_format(1280, 720, Withrobot::fourcc_to_pixformat('G','B','G','R'), 1, 15);

                    // get intrinsic cam 3
                    mtx.unlock();
                    intrinsic[3] = string_to_intrinsic(config->CAM_INTRINSIC_3);
                    intrinsic[3].coef_num = 4;

                    cam_tf[3] = string_to_TF(config->CAM_TF_3);
                    mtx.unlock();
                    printf("[CAM] intrinsic3, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[3].fx, intrinsic[3].fy, intrinsic[3].cx, intrinsic[3].cy);

                    /*std::vector<const char*> list;
                    int cnt = ocam_pipe1->get_valid_image_format_list(list);
                    for(int r=0; r<cnt; r++)
                    {
                        std::cout << list[r] << std::endl;
                        std::vector<const char*> list2;
                        int cnt2 = ocam_pipe1->get_valid_resolution_list(list[r], list2);
                        for(int r2=0; r2<cnt2; r2++)
                        {
                            std::cout << list2[r2] << std::endl;
                            std::vector<const char*> list3;
                            int cnt3 = ocam_pipe1->get_valid_ratio_list(list2[r2], list3);
                            for(int r3=0; r3<cnt3; r3++)
                            {
                                std::cout << list3[r3] << std::endl;
                            }
                        }
                    }*/

                    ocam_pipe1->start();
                }
            }
        }
    }

    is_param_loaded = true;
    printf("[CAM] grab loop started\n");
    while(grab_flag)
    {
        if(ocam_pipe0 != NULL)
        {
            cv::Mat img(cv::Size(ocam_format.width, ocam_format.height), CV_8UC1);
            if(ocam_pipe0->get_frame(img.data, ocam_format.image_size, 1) != 1)
            {
                cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

                TIME_IMG time_img;
                time_img.t = get_time();
                time_img.img = img.clone();

                cv::Mat plot_img;
                cv::resize(img, plot_img, cv::Size(160, 90));
                //cv::flip(plot_img, plot_img, -1);

                mtx.lock();
                cur_img[2] = plot_img.clone();
                cur_time_img[2] = time_img;
                mtx.unlock();

                if(is_connected[2] == false)
                {
                    is_connected[2] = true;
                }
            }
        }

        if(ocam_pipe1 != NULL)
        {
            cv::Mat img(cv::Size(ocam_format.width, ocam_format.height), CV_8UC1);
            if(ocam_pipe1->get_frame(img.data, ocam_format.image_size, 1) != 1)
            {
                cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);

                TIME_IMG time_img;
                time_img.t = get_time();
                time_img.img = img.clone();

                cv::Mat plot_img;
                cv::resize(img, plot_img, cv::Size(160, 90));
                //cv::flip(plot_img, plot_img, -1);

                mtx.lock();
                cur_img[3] = plot_img.clone();
                cur_time_img[3] = time_img;
                mtx.unlock();

                if(is_connected[3] == false)
                {
                    is_connected[3] = true;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if(is_connected[2])
    {
        ocam_pipe0->stop();
        is_connected[2] = false;
    }

    if(is_connected[3])
    {
        ocam_pipe1->stop();
        is_connected[3] = false;
    }
    #endif

    #ifndef USE_OCAM
    // for usb cam2
    cv::VideoCapture cap2;
    QString pipeline_cam2 = "";
    if(config->CAM_SERIAL_NUMBER_2 != "")
    {
        pipeline_cam2.sprintf("v4l2src device=%s ! image/jpeg,framerate=60/1,width=1280,height=720 ! jpegdec ! videoconvert ! appsink",
                               config->CAM_SERIAL_NUMBER_2.toLocal8Bit().data());

        // get intrinsic cam 2
        mtx.unlock();
        intrinsic[2] = string_to_intrinsic(config->CAM_INTRINSIC_2);
        intrinsic[2].coef_num = 4;

        cam_tf[2] = string_to_TF(config->CAM_TF_2);
        mtx.unlock();
        printf("[CAM] intrinsic2, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[2].fx, intrinsic[2].fy, intrinsic[2].cx, intrinsic[2].cy);

        cap2.open(pipeline_cam2.toStdString(), cv::CAP_GSTREAMER);
    }

    // for usb cam3
    cv::VideoCapture cap3;
    QString pipeline_cam3 = "";
    if(config->CAM_SERIAL_NUMBER_3 != "")
    {
        pipeline_cam3.sprintf("v4l2src device=%s ! image/jpeg,framerate=60/1,width=1280,height=720 ! jpegdec ! videoconvert ! appsink",
                               config->CAM_SERIAL_NUMBER_3.toLocal8Bit().data());

        // get intrinsic cam 3
        mtx.unlock();
        intrinsic[3] = string_to_intrinsic(config->CAM_INTRINSIC_3);
        intrinsic[3].coef_num = 4;

        cam_tf[3] = string_to_TF(config->CAM_TF_3);
        mtx.unlock();
        printf("[CAM] intrinsic3, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[3].fx, intrinsic[3].fy, intrinsic[3].cx, intrinsic[3].cy);

        cap3.open(pipeline_cam3.toStdString(), cv::CAP_GSTREAMER);
    }

    is_param_loaded = true;

    printf("[CAM] grab loop started\n");
    while(grab_flag)
    {
        if(cap2.isOpened())
        {
            cv::Mat img;
            cap2 >> img;
            if(!img.empty())
            {
                TIME_IMG time_img;
                time_img.t = get_time();
                time_img.img = img.clone();

                cv::Mat plot_img;
                cv::resize(img, plot_img, cv::Size(160, 90));
                cv::flip(plot_img, plot_img, -1);

                mtx.lock();
                cur_img[2] = plot_img.clone();
                cur_time_img[2] = time_img;
                mtx.unlock();

                if(is_connected[2] == false)
                {
                    is_connected[2] = true;
                }
            }
        }

        if(cap3.isOpened())
        {
            cv::Mat img;
            cap3 >> img;
            if(!img.empty())
            {
                TIME_IMG time_img;
                time_img.t = get_time();
                time_img.img = img.clone();

                cv::Mat plot_img;
                cv::resize(img, plot_img, cv::Size(160, 90));

                mtx.lock();
                cur_img[3] = plot_img.clone();
                cur_time_img[3] = time_img;
                mtx.unlock();

                if(is_connected[3] == false)
                {
                    is_connected[3] = true;
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if(is_connected[2])
    {
        cap2.release();
        is_connected[2] = false;
    }

    if(is_connected[3])
    {
        cap3.release();
        is_connected[3] = false;
    }
    #endif

    pipe0->stop();
    pipe1->stop();

    printf("[CAM] grab loop stopped\n");
}
#endif

#if defined(USE_AMR_400_LAKI) || defined(USE_AMR_400)
void CAM::grab_loop()
{
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_ERROR);

    ob::Context ctx;
    auto dev_list = ctx.queryDeviceList();
    int dev_count = dev_list->deviceCount();
    if(dev_count != 1)
    {
        logger->write_log("[CAM] invalid number of cams", "Red");
        return;
    }

    // set property
    QString sn = dev_list->getDevice(0)->getDeviceInfo()->serialNumber();
    logger->write_log(QString("[CAM] cam serial num:") + sn, "Green");
    printf("[CAM] cam serial num: %s\n", sn.toLocal8Bit().data());

    if((sn != config->CAM_SERIAL_NUMBER_0) && (sn != config->CAM_SERIAL_NUMBER_1))
    {
        logger->write_log("[CAM] invalid cam serial num", "Red");
        return;
    }

    // set cam
    auto dev = dev_list->getDevice(0);
    dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);

    std::shared_ptr<ob::Pipeline> cam_pipe = std::make_shared<ob::Pipeline>(dev);
    Eigen::Matrix4d TF = string_to_TF(config->CAM_TF_0);

    auto depth_profile_list = cam_pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    /*for(size_t p = 0; p < depth_profile_list->count(); p++)
    {
        auto profile = depth_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
        printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    }*/

    auto color_profile_list = cam_pipe->getStreamProfileList(OB_SENSOR_COLOR);
    /*for(size_t p = 0; p < color_profile_list->count(); p++)
    {
        auto profile = color_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
        printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    }*/

    auto depth_profile = depth_profile_list->getProfile(24)->as<ob::VideoStreamProfile>();
    QString str_depth;
    str_depth.sprintf("[CAM] depth_profile(24), w:%d, h:%d, fps:%d, format:%d",
                      depth_profile->width(),
                      depth_profile->height(),
                      depth_profile->fps(),
                      depth_profile->format());
    logger->write_log(str_depth, "Green");

    auto color_profile = color_profile_list->getProfile(40)->as<ob::VideoStreamProfile>();
    QString str_color;
    str_color.sprintf("[CAM] color_profile(40), w:%d, h:%d, fps:%d, format:%d",
                      color_profile->width(),
                      color_profile->height(),
                      color_profile->fps(),
                      color_profile->format());
    logger->write_log(str_color, "Green");

    std::shared_ptr<ob::Config> cam_config = std::make_shared<ob::Config>();
    cam_config->disableAllStream();
    cam_config->enableStream(depth_profile);
    cam_config->enableStream(color_profile);
    cam_config->setAlignMode(ALIGN_DISABLE);

    cam_pipe->start(cam_config, [&](std::shared_ptr<ob::FrameSet> fs)
    {
        try
        {
            if(fs->depthFrame() != nullptr)
            {
                uint64_t ts = fs->depthFrame()->systemTimeStamp();
                double t = (double)ts/1000.0 - st_time_for_get_time;

                // get point cloud
                ob::PointCloudFilter point_cloud;
                auto camera_param = cam_pipe->getCameraParam();

                point_cloud.setCameraParam(camera_param);
                point_cloud.setCreatePointFormat(OB_FORMAT_POINT);

                auto scale = fs->depthFrame()->getValueScale();
                point_cloud.setPositionDataScaled(scale);

                std::shared_ptr<ob::Frame> frame = point_cloud.process(fs);
                OBPoint *point = (OBPoint *)frame->data();

                int w = fs->depthFrame()->width();
                int h = fs->depthFrame()->height();

                std::vector<Eigen::Vector3d> pts;
                for(int i = 0; i < h; i++)
                {
                    for(int j = 0; j < w; j++)
                    {
                        double x = -point[i*w+j].x/1000.0;
                        double y = point[i*w+j].y/1000.0;
                        double z = point[i*w+j].z/1000.0;
                        if(x != 0 || y != 0 || z != 0)
                        {
                            Eigen::Vector3d P(x,y,z);
                            Eigen::Vector3d _P = TF.block(0,0,3,3)*P + TF.block(0,3,3,1);

                            if(_P[0] > config->ROBOT_SIZE_X[0] && _P[0] < config->ROBOT_SIZE_X[1] &&
                               _P[1] > config->ROBOT_SIZE_Y[0] && _P[1] < config->ROBOT_SIZE_Y[1])
                            {
                                continue;
                            }

                            if(_P[2] < config->CAM_HEIGHT_MIN || _P[2] > config->CAM_HEIGHT_MAX)
                            {
                                continue;
                            }

                            // orthogonal projection
                            _P[2] = 0;
                            pts.push_back(_P);
                        }
                    }
                }

                // voxel filtering
                pts = voxel_filtering(pts, config->SLAM_VOXEL_SIZE);

                // update scan
                TIME_PTS scan;
                scan.t = t;
                scan.pts = pts;

                mtx.lock();
                cur_scan0 = scan;
                mtx.unlock();
            }

            if(fs->colorFrame() != nullptr)
            {
                if(is_connected0 == false)
                {
                    is_connected0 = true;
                }

                // get color image
                std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();

                // convert opencv image
                cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

                cv::Mat img;
                cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);
                if(!img.empty())
                {
                    mtx.lock();
                    cur_img0 = img.clone();
                    mtx.unlock();
                }
            }
        }
        catch(const ob::Error &e)
        {
            QString str = QString::fromLocal8Bit(e.getMessage());
            logger->write_log("[CAM] " + str, "Red");
        }
    });

    logger->write_log("[CAM] start grab loop", "Green");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    cam_pipe->stop();
    logger->write_log("[CAM] stop grab loop", "Green");
}
#endif
