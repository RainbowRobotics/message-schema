#include "cam.h"

CAM::CAM(QObject *parent) : QObject(parent)
{
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

cv::Mat CAM::get_img0()
{
    mtx.lock();
    cv::Mat res = cur_img0.clone();
    mtx.unlock();

    return res;
}

cv::Mat CAM::get_img1()
{
    mtx.lock();
    cv::Mat res = cur_img1.clone();
    mtx.unlock();

    return res;
}

TIME_IMG CAM::get_time_img0()
{
    mtx.lock();
    TIME_IMG res = cur_time_img0;
    mtx.unlock();

    return res;
}

TIME_IMG CAM::get_time_img1()
{
    mtx.lock();
    TIME_IMG res = cur_time_img1;
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

CAM_INTRINSIC CAM::get_rgb_intrinsic0()
{
    mtx.lock();
    CAM_INTRINSIC res = rgb_intrinsic0;
    mtx.unlock();

    return res;
}

CAM_INTRINSIC CAM::get_rgb_intrinsic1()
{
    mtx.lock();
    CAM_INTRINSIC res = rgb_intrinsic1;
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
                if(is_connected0 == false)
                {
                    is_connected0 = true;
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
                    cur_img0 = plot_img.clone();
                    cur_time_img0 = time_img;
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
                if(is_connected1 == false)
                {
                    is_connected1 = true;
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
                    cur_img1 = plot_img.clone();
                    cur_time_img1 = time_img;
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

    // get intrinsics
    auto camera_param0 = pipe0->getCameraParam();
    mtx.lock();
    rgb_intrinsic0.fx = camera_param0.rgbIntrinsic.fx; rgb_intrinsic0.fy = camera_param0.rgbIntrinsic.fy;
    rgb_intrinsic0.cx = camera_param0.rgbIntrinsic.cx; rgb_intrinsic0.cy = camera_param0.rgbIntrinsic.cy;

    rgb_intrinsic0.k1 = camera_param0.rgbDistortion.k1; rgb_intrinsic0.k2 = camera_param0.rgbDistortion.k2;
    rgb_intrinsic0.k3 = camera_param0.rgbDistortion.k3; rgb_intrinsic0.k4 = camera_param0.rgbDistortion.k4;
    rgb_intrinsic0.k5 = camera_param0.rgbDistortion.k5; rgb_intrinsic0.k6 = camera_param0.rgbDistortion.k6;
    rgb_intrinsic0.p1 = camera_param0.rgbDistortion.p1; rgb_intrinsic0.p2 = camera_param0.rgbDistortion.p2;

    rgb_extrinsic0.setIdentity();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rgb_extrinsic0(i,j) = camera_param0.transform.rot[i*3+j];
        }
        rgb_extrinsic0(i,3) = camera_param0.transform.trans[i]/1000.0;
    }
    mtx.unlock();

    printf("[CAM] rgb_intrinsic0, fx:%f, fy:%f, cx:%f, cy:%f\n", rgb_intrinsic0.fx, rgb_intrinsic0.fy, rgb_intrinsic0.cx, rgb_intrinsic0.cy);
    printf("[CAM] rgb_extrinsic0, x:%f, y:%f, z:%f\n", camera_param0.transform.trans[0], camera_param0.transform.trans[1], camera_param0.transform.trans[2]);

    auto camera_param1 = pipe1->getCameraParam();
    mtx.lock();
    rgb_intrinsic1.fx = camera_param1.rgbIntrinsic.fx; rgb_intrinsic1.fy = camera_param1.rgbIntrinsic.fy;
    rgb_intrinsic1.cx = camera_param1.rgbIntrinsic.cx; rgb_intrinsic1.cy = camera_param1.rgbIntrinsic.cy;

    rgb_intrinsic1.k1 = camera_param1.rgbDistortion.k1; rgb_intrinsic1.k2 = camera_param1.rgbDistortion.k2;
    rgb_intrinsic1.k3 = camera_param1.rgbDistortion.k3; rgb_intrinsic1.k4 = camera_param1.rgbDistortion.k4;
    rgb_intrinsic1.k5 = camera_param1.rgbDistortion.k5; rgb_intrinsic1.k6 = camera_param1.rgbDistortion.k6;
    rgb_intrinsic1.p1 = camera_param1.rgbDistortion.p1; rgb_intrinsic1.p2 = camera_param1.rgbDistortion.p2;

    rgb_extrinsic1.setIdentity();
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            rgb_extrinsic1(i,j) = camera_param1.transform.rot[i*3+j];
        }
        rgb_extrinsic1(i,3) = camera_param1.transform.trans[i]/1000.0;
    }
    mtx.unlock();

    printf("[CAM] rgb_intrinsic1, fx:%f, fy:%f, cx:%f, cy:%f\n", rgb_intrinsic1.fx, rgb_intrinsic1.fy, rgb_intrinsic1.cx, rgb_intrinsic1.cy);
    printf("[CAM] rgb_extrinsic1, x:%f, y:%f, z:%f\n", camera_param1.transform.trans[0], camera_param1.transform.trans[1], camera_param1.transform.trans[2]);

    is_param_loaded = true;

    printf("[CAM] grab loop started\n");
    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

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
    int pn = 0;
    for(pn=0; pn<depth_profile_list->count(); ++pn){
        auto profile = depth_profile_list->getProfile(pn)->as<ob::VideoStreamProfile>();
        printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", pn, profile->width(), profile->height(), profile->fps(), profile->format());
        if(profile->width() == 320 && profile->height() == 200 && profile->fps() == 5 && profile->format() == 24){
            break;
        }
    }


    auto depth_profile = depth_profile_list->getProfile(pn)->as<ob::VideoStreamProfile>();
    QString str_depth;
    str_depth.sprintf("[CAM] depth_profile(%d), w:%d, h:%d, fps:%d, format:%d",
                      pn,
                      depth_profile->width(),
                      depth_profile->height(),
                      depth_profile->fps(),
                      depth_profile->format());
    logger->write_log(str_depth, "Green");


    auto color_profile_list = cam_pipe->getStreamProfileList(OB_SENSOR_COLOR);
    for(pn=0; pn<color_profile_list->count(); ++pn){
        auto profile = color_profile_list->getProfile(pn)->as<ob::VideoStreamProfile>();
        printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", pn, profile->width(), profile->height(), profile->fps(), profile->format());
        if(profile->width() == 640 && profile->height() == 400 && profile->fps() == 5 && profile->format() == 22){
            break;
        }
    }



    auto color_profile = color_profile_list->getProfile(pn)->as<ob::VideoStreamProfile>();
    QString str_color;
    str_color.sprintf("[CAM] color_profile(%d), w:%d, h:%d, fps:%d, format:%d",
                      pn,
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
