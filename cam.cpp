 #include "cam.h"

CAM::CAM(QObject *parent) : QObject(parent)
{
    for(int p = 0; p < 2; p++)
    {
        extrinsic[p].setIdentity();
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

TIME_PTS CAM::get_scan(int cam_idx)
{
    mtx.lock();
    TIME_PTS res = cur_scan[cam_idx];
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
    Eigen::Matrix4d res = extrinsic[cam_idx];
    mtx.unlock();

    return res;
}

void CAM::grab_loop()
{
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);

    ob::Context ctx;
    auto dev_list = ctx.queryDeviceList();
    int dev_count = dev_list->deviceCount();
    if(dev_count == 0)
    {
        logger->write_log("[CAM] no camera", "Red");
        return;
    }
    else if(dev_count == 1)
    {
        // set property
        int cam_idx0 = 0;
        QString sn = dev_list->getDevice(0)->getDeviceInfo()->serialNumber();
        logger->write_log(QString("[CAM] detected serial number, sn:%1").arg(sn));

        if(sn != config->CAM_SERIAL_NUMBER_0)
        {
            logger->write_log("[CAM] invalid serial number");
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
                    cur_scan[0] = scan;
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

            extrinsic[0] = string_to_TF(config->CAM_TF_0);
            mtx.unlock();
        }
        printf("[CAM] intrinsic0, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[0].fx, intrinsic[0].fy, intrinsic[0].cx, intrinsic[0].cy);

        is_param_loaded = true;

        printf("[CAM] grab loop started\n");
        while(grab_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        pipe0->stop();
        printf("[CAM] grab loop stopped\n");
    }
    else if(dev_count == 2)
    {
        // set property
        int cam_idx0 = 0;
        int cam_idx1 = 1;

        int serial_valid_cnt = 0;
        for(int p = 0; p < dev_count; p++)
        {
            QString sn = dev_list->getDevice(p)->getDeviceInfo()->serialNumber();
            logger->write_log(QString("[CAM] detected serial number, sn:%1").arg(sn));
            if(config->CAM_SERIAL_NUMBER_0 == sn)
            {
                // left camera
                cam_idx0 = p;
                serial_valid_cnt++;
            }
            else if(config->CAM_SERIAL_NUMBER_1 == sn)
            {
                cam_idx1 = p;
                serial_valid_cnt++;
            }
        }

        if(serial_valid_cnt != 2)
        {
            logger->write_log("[CAM] invalid serial number");
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
                    cur_scan[0] = scan;
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
                        cv::flip(plot_img, plot_img, -1); // flip for SRV

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
                    cur_scan[1] = scan;
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

            extrinsic[0] = string_to_TF(config->CAM_TF_0);
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

            extrinsic[1] = string_to_TF(config->CAM_TF_1);
            mtx.unlock();
        }
        printf("[CAM] intrinsic1, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[1].fx, intrinsic[1].fy, intrinsic[1].cx, intrinsic[1].cy);

        is_param_loaded = true;

        printf("[CAM] grab loop started\n");
        while(grab_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        pipe0->stop();
        pipe1->stop();

        printf("[CAM] grab loop stopped\n");
    }
}
