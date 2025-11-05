#include "ORBBEC.h"
namespace
{
    const char* MODULE_NAME = "ORBBEC";
}

ORBBEC* ORBBEC::instance(QObject* parent)
{
    static ORBBEC* inst = nullptr;
    if(!inst && parent)
    {
        inst = new ORBBEC(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

ORBBEC::ORBBEC(QObject *parent) : QObject(parent)
{
    for(int p = 0; p < max_cam_cnt; p++)
    {
        is_connected[p] = false;
        is_param_loaded[p] = false;
        cur_pts_size[p] = 0.0;
        extrinsic[p].setIdentity();
    }
}

ORBBEC::~ORBBEC()
{
    for(int p = 0; p < max_cam_cnt; p++)
    {
        grab_flag[p] = false;

        if(grab_thread[p] && grab_thread[p]->joinable())
        {
            grab_thread[p]->join();
        }
        grab_thread[p].reset();
    }
}

void ORBBEC::init()
{
    for(int i = 0; i < config->get_cam_num(); i++)
    {
        extrinsic[i] = string_to_TF(config->get_cam_tf(i));
    }

    //printf("[ORBBEC] init\n");
    log_info("init");
}

void ORBBEC::open()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        //printf("[ORBBEC] simulation mode\n");
        log_info("simulation mode");
        return;
    }

    // start grab loop
    for(int p = 0; p < config->get_cam_num(); p++)
    {
        grab_flag[p] = true;
        grab_thread[p] = std::make_unique<std::thread>(&ORBBEC::grab_loop, this, p);
    }
}

void ORBBEC::close()
{
    for(int p = 0; p < config->get_cam_num(); p++)
    {
        grab_flag[p] = false;
        if(grab_thread[p] && grab_thread[p]->joinable())
        {
            grab_thread[p]->join();
        }
        grab_thread[p].reset();
    }
}

QString ORBBEC::get_cam_info_str()
{
    QString connection_str = "connection:";
    for(int p = 0; p < config->get_cam_num(); p++)
    {
        QString connection = QString("%1,").arg(is_connected[p] ? "1" : "0");
        connection_str += connection;
    }

    QString pts_str = "pts:";
    for(int p = 0; p < config->get_cam_num(); p++)
    {
        QString pts = QString("%1,").arg(cur_pts_size[p]);
        pts_str += pts;
    }

    QString str = QString("[ORBBEC]\n%1, color(w,h):%2,%3\ndepth(w,h):%4,%5, %6").arg(connection_str)
                                                                                 .arg((int)cur_w_color).arg((int)cur_h_color)
                                                                                 .arg((int)cur_w_depth).arg((int)cur_h_depth)
                                                                                 .arg(pts_str);
    return str;
}

cv::Mat ORBBEC::get_img(int cam_idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    cv::Mat res = cur_img[cam_idx].clone();
    return res;
}

TIME_IMG ORBBEC::get_time_img(int cam_idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    TIME_IMG res = cur_time_img[cam_idx];
    return res;
}

TIME_PTS ORBBEC::get_scan(int cam_idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    TIME_PTS res = cur_scan[cam_idx];
    return res;
}

CAM_INTRINSIC ORBBEC::get_intrinsic(int cam_idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    CAM_INTRINSIC res = intrinsic[cam_idx];
    return res;
}

Eigen::Matrix4d ORBBEC::get_extrinsic(int cam_idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    Eigen::Matrix4d res = extrinsic[cam_idx];
    return res;
}

bool ORBBEC::try_pop_depth_que(int idx, TIME_PTS &tp)
{
    if(depth_que[idx].try_pop(tp))
    {
        return true;
    }

    return false;
}

bool ORBBEC::try_pop_img_que(int idx, TIME_IMG &ti)
{
    if(img_que[idx].try_pop(ti))
    {
        return true;
    }

    return false;
}

void ORBBEC::grab_loop(int idx)
{
    // check device
    ob::Context ctx;
    ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
    ctx.setLoggerToConsole(OB_LOG_SEVERITY_OFF);

    auto dev_list = ctx.queryDeviceList();
    int dev_count = dev_list->deviceCount();
    if(dev_count == 0)
    {
        grab_flag[idx] = false;

        logger->write_log("[ORBBEC] no camera", "Red");
        //spdlog::error("[ORBBEC] no camera");
        log_error("no camera");
        return;
    }

    QString sn = dev_list->getDevice(idx)->getDeviceInfo()->serialNumber();
    //if(sn != config->get_cam_serial_number(idx))
    //{
    //    grab_flag[idx] = false;

    //    logger->write_log("[ORBBEC] no camera", "Red");
    //    return;
    //}
    logger->write_log(QString("[ORBBEC] detected serial number, sn:%1").arg(sn));
    log_info("detected serial number, sn:{}", sn.toStdString());

    double x_min = config->get_robot_size_x_min(), x_max = config->get_robot_size_x_max();
    double y_min = config->get_robot_size_y_min(), y_max = config->get_robot_size_y_max();
    double z_min = config->get_cam_height_min(),  z_max = config->get_cam_height_max();
    double voxel_size = config->get_mapping_voxel_size();

    // set cam
    auto dev = dev_list->getDevice(idx);
    dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    Eigen::Matrix4d TF = string_to_TF(config->get_cam_tf(idx));
    Eigen::Matrix3d TF_R = TF.block(0,0,3,3);
    Eigen::Vector3d TF_t = TF.block(0,3,3,1);

    auto depth_profile_list = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    auto depth_profile = depth_profile_list->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
    //printf("[ORBBEC] depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", depth_profile_idx, depth_profile->width(), depth_profile->height(), depth_profile->fps(), depth_profile->format());
    log_info("Current depth_profile({}, w:{}, h:{}, fps:{}, format:{})", depth_profile_idx, depth_profile->width(), depth_profile->height(), depth_profile->fps(), static_cast<int>(depth_profile->format()));
    cur_w_depth = depth_profile->width();
    cur_h_depth = depth_profile->height();

    //log_info("Available Depth profiles:");
    //for(size_t p = 0; p < depth_profile_list->count(); p++)
    //{
    //    auto profile = depth_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
    //    //printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    //    log_info("depth_profile({}, w:{}, h:{}, fps:{}, format:{})", p, profile->width(), profile->height(), profile->fps(), static_cast<int>(profile->format()));
    //}

    auto color_profile_list = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profile_list->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();
    //printf("[ORBBEC] color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", color_profile_idx, color_profile->width(), color_profile->height(), color_profile->fps(), color_profile->format());
    log_info("Current color_profile({}, w:{}, h:{}, fps:{}, format:{})", color_profile_idx, color_profile->width(), color_profile->height(), color_profile->fps(), static_cast<int>(color_profile->format()));
    cur_w_color = color_profile->width();
    cur_h_color = color_profile->height();

    //log_info("Available Color profiles:");
    //for(size_t p = 0; p < color_profile_list->count(); p++)
    //{
    //    auto profile = color_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
    //    //printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    //    log_info("color_profile({}, w:{}, h:{}, fps:{}, format:{})", p, profile->width(), profile->height(), profile->fps(), static_cast<int>(profile->format()));
    //}

    std::shared_ptr<ob::Config> cam_config = std::make_shared<ob::Config>();
    //cam_config->disableAllStream();
    cam_config->enableStream(depth_profile);
    cam_config->enableStream(color_profile);
    cam_config->setAlignMode(ALIGN_DISABLE);

    ob::PointCloudFilter point_cloud;
    bool filter_param_set = false;
    pipe->start(cam_config, [&](std::shared_ptr<ob::FrameSet> fs)
    {
        try
        {
            if(fs->depthFrame() != nullptr)
            {
                uint64_t ts = fs->depthFrame()->systemTimeStamp();
                double t = static_cast<double>(ts) / 1000.0;

                if(!filter_param_set)
                {
                    auto camera_param = pipe->getCameraParam();
                    point_cloud.setCameraParam(camera_param);
                    point_cloud.setCreatePointFormat(OB_FORMAT_POINT);
                    filter_param_set = true;
                }
                auto scale = fs->depthFrame()->getValueScale();
                point_cloud.setPositionDataScaled(scale);

                std::shared_ptr<ob::Frame> frame = point_cloud.process(fs);
                OBPoint *point = (OBPoint *)frame->data();

                OBColorPoint *cPoint = (OBColorPoint *)frame->data();

                int w = fs->depthFrame()->width();
                int h = fs->depthFrame()->height();

                // pts reserve
                std::vector<Eigen::Vector3d> pts;
                {
                    for(int p = 0; p < w * h; p++)
                    {
                        double x = point[p].x / 1000.0;
                        double y = point[p].y / 1000.0;
                        double z = point[p].z / 1000.0;


                        if(x == 0 && y == 0 && z == 0)
                        {
                            continue;
                        }

                        Eigen::Vector3d P(x, y, z);
                        Eigen::Vector3d _P = TF_R * P + TF_t;

                        if(_P[0] > x_min && _P[0] < x_max && _P[1] > y_min && _P[1] < y_max)
                        {
                            continue;
                        }

                        if(_P[2] < z_min || _P[2] > z_max)
                        {
                            continue;
                        }

//                        _P[2] = 0;
                        pts.push_back(_P);
                    }
                }

                pts = voxel_filtering(pts, voxel_size);
                cur_pts_size[idx] = pts.size();

                TIME_PTS scan;
                scan.t = t;
                scan.pts = std::move(pts);

                depth_que[idx].push(scan);
                if(depth_que[idx].unsafe_size() > 10)
                {
                    TIME_PTS dummy;
                    depth_que[idx].try_pop(dummy);
                }

                {
                    std::lock_guard<std::mutex> lock(mtx);
                    cur_scan[idx] = std::move(scan);
                }
            }

            if(fs->colorFrame() != nullptr)
            {
                if(!is_connected[idx])
                {
                    is_connected[idx] = true;
                }

                uint64_t ts = fs->colorFrame()->systemTimeStamp();
                double t = static_cast<double>(ts) / 1000.0;


                std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();
                cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

                cv::Mat img;
                cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);

                if(!img.empty())
                {
                    TIME_IMG time_img;
                    time_img.t = t;
                    time_img.img = img.clone();

                    img_que[idx].push(time_img);

                    if(img_que[idx].unsafe_size() > 10)
                    {
                        TIME_IMG dummy;
                        img_que[idx].try_pop(dummy);
                    }

                    cv::Mat plot_img;
                    cv::resize(img, plot_img, cv::Size(160, 90));
                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_img[idx]      = plot_img.clone();
                        cur_time_img[idx] = time_img;
                    }
                }
            }
        }
        catch(const ob::Error &e)
        {
            QString str = QString::fromLocal8Bit(e.getMessage());
            logger->write_log("[ORBBEC] " + str, "Red");
            log_error("ORBBEC error: {}", str.toStdString());
        }

        // get intrinsic cam 0
        if(!is_param_loaded[idx])
        {
            is_param_loaded[idx] = true;
            auto camera_param = pipe->getCameraParam();

            std::lock_guard<std::mutex> lock(mtx);
            intrinsic[idx].fx = camera_param.rgbIntrinsic.fx;
            intrinsic[idx].fy = camera_param.rgbIntrinsic.fy;
            intrinsic[idx].cx = camera_param.rgbIntrinsic.cx;
            intrinsic[idx].cy = camera_param.rgbIntrinsic.cy;
            intrinsic[idx].k1 = camera_param.rgbDistortion.k1;
            intrinsic[idx].k2 = camera_param.rgbDistortion.k2;
            intrinsic[idx].k3 = camera_param.rgbDistortion.k3;
            intrinsic[idx].k4 = camera_param.rgbDistortion.k4;
            intrinsic[idx].k5 = camera_param.rgbDistortion.k5;
            intrinsic[idx].k6 = camera_param.rgbDistortion.k6;
            intrinsic[idx].p1 = camera_param.rgbDistortion.p1;
            intrinsic[idx].p2 = camera_param.rgbDistortion.p2;
            intrinsic[idx].coef_num = 8;

            extrinsic[idx] = string_to_TF(config->get_cam_tf(idx));
            //printf("[ORBBEC] intrinsic%d, fx:%f, fy:%f, cx:%f, cy:%f\n", idx, intrinsic[idx].fx, intrinsic[idx].fy, intrinsic[idx].cx, intrinsic[idx].cy);
            log_info("intrinsic{}, fx:{}, fy:{}, cx:{}, cy:{}", idx, intrinsic[idx].fx, intrinsic[idx].fy, intrinsic[idx].cx, intrinsic[idx].cy);
        }
    });

    //printf("[ORBBEC] grab loop started\n");
    log_info("grab loop started");
    std::this_thread::sleep_for(std::chrono::milliseconds(idx * 500));
    while(grab_flag[idx])
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    pipe->stop();
    //printf("[ORBBEC] grab loop stopped\n");
    log_info("grab loop stopped");
}

/*void ORBBEC::grab_loop()
{
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);

    ob::Context ctx;
    auto dev_list = ctx.queryDeviceList();
    int dev_count = dev_list->deviceCount();
    if(dev_count == 0)
    {
        logger->write_log("[ORBBEC] no camera", "Red");
        return;
    }
    else if(dev_count == 1)
    {
        // set property
        int cam_idx0 = 0;
        QString sn = dev_list->getDevice(0)->getDeviceInfo()->serialNumber();
        logger->write_log(QString("[ORBBEC] detected serial number, sn:%1").arg(sn));

        if(sn != config->get_cam_serial_number(0))
        {
            logger->write_log("[ORBBEC] invalid serial number");
            return;
        }

        // set cam0
        auto dev0 = dev_list->getDevice(cam_idx0);
        dev0->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
        dev0->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

        std::shared_ptr<ob::Pipeline> pipe0 = std::make_shared<ob::Pipeline>(dev0);
        Eigen::Matrix4d TF0 = string_to_TF(config->get_cam_tf(0));

        auto depth_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_DEPTH);

        for(size_t p = 0; p < depth_profile_list0->count(); p++)
        {
            auto profile = depth_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
            printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
        }


        auto color_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_COLOR);

        for(size_t p = 0; p < color_profile_list0->count(); p++)
        {
            auto profile = color_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
            printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
        }


        auto depth_profile0 = depth_profile_list0->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
        printf("[ORBBEC] depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", depth_profile_idx, depth_profile0->width(), depth_profile0->height(), depth_profile0->fps(), depth_profile0->format());

        cur_w_depth = depth_profile0->width();
        cur_h_depth = depth_profile0->height();

        auto color_profile0 = color_profile_list0->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();
        printf("[ORBBEC] color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", color_profile_idx, color_profile0->width(), color_profile0->height(), color_profile0->fps(), color_profile0->format());

        cur_w_color = color_profile0->width();
        cur_h_color = color_profile0->height();

        std::shared_ptr<ob::Config> config0 = std::make_shared<ob::Config>();
        config0->disableAllStream();
        config0->enableStream(depth_profile0);
        config0->enableStream(color_profile0);
        config0->setAlignMode(ALIGN_DISABLE);

        double x_min = config->get_robot_size_x_min(); double x_max = config->get_robot_size_x_max();
        double y_min = config->get_robot_size_y_min(); double y_max = config->get_robot_size_y_max();
        double z_min = config->get_cam_height_min();   double z_max = config->get_cam_height_max();

        pipe0->start(config0, [&](std::shared_ptr<ob::FrameSet> fs)
        {
            try
            {
                if(fs->depthFrame() != nullptr)
                {
                    uint64_t ts = fs->depthFrame()->systemTimeStamp();
                    double t = (double)ts/1000.0;

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

                                if(_P[0] > x_min && _P[0] < x_max &&
                                        _P[1] > y_min && _P[1] < y_max)
                                {
                                    continue;
                                }

                                if(_P[2] < z_min || _P[2] > z_max)
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
                    pts = voxel_filtering(pts, config->get_mapping_voxel_size());
                    cur_pts_size0 = pts.size();

                    // update scan
                    TIME_PTS scan;
                    scan.t = t;
                    scan.pts = pts;

                    depth_que[0].push(scan);
                    if(depth_que[0].unsafe_size() > 10)
                    {
                        TIME_PTS dummy;
                        depth_que[0].try_pop(dummy);
                    }

                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_scan[0] = scan;
                    }
                }

                if(fs->colorFrame() != nullptr)
                {
                    if(is_connected[0] == false)
                    {
                        is_connected[0] = true;
                    }

                    uint64_t ts = fs->colorFrame()->systemTimeStamp();
                    double t = (double)ts/1000.0;

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

                        img_que[0].push(time_img);
                        if(img_que[0].unsafe_size() > 10)
                        {
                            TIME_IMG dummy;
                            img_que[0].try_pop(dummy);
                        }

                        // flip for plot
                        cv::Mat plot_img;
                        cv::resize(img, plot_img, cv::Size(160, 100));
                        {
                            std::lock_guard<std::mutex> lock(mtx);
                            cur_img[0] = plot_img.clone();
                            cur_time_img[0] = time_img;
                        }
                    }
                }
            }
            catch(const ob::Error &e)
            {
                QString str = QString::fromLocal8Bit(e.getMessage());
                logger->write_log("[ORBBEC] " + str, "Red");
            }
        });

        // get intrinsic cam 0
        {
            auto camera_param = pipe0->getCameraParam();

            std::lock_guard<std::mutex> lock(mtx);
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

            extrinsic[0] = string_to_TF(config->get_cam_tf(0));
        }
        printf("[ORBBEC] intrinsic0, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[0].fx, intrinsic[0].fy, intrinsic[0].cx, intrinsic[0].cy);

        is_param_loaded = true;

        printf("[ORBBEC] grab loop started\n");
        while(grab_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }

        pipe0->stop();
        printf("[ORBBEC] grab loop stopped\n");
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
            if(config->get_cam_serial_number(0) == sn)
            {
                cam_idx0 = p;
                serial_valid_cnt++;
            }
            else if(config->get_cam_serial_number(1) == sn)
            {
                cam_idx1 = p;
                serial_valid_cnt++;
            }
            logger->write_log(QString("[ORBBEC] detected serial number, sn:%1").arg(sn));
        }

        if(serial_valid_cnt != 2)
        {
            logger->write_log("[ORBBEC] invalid serial number");
            return;
        }

        double x_min = config->get_robot_size_x_min(), x_max = config->get_robot_size_x_max();
        double y_min = config->get_robot_size_y_min(), y_max = config->get_robot_size_y_max();
        double z_min = config->get_cam_height_min(),  z_max = config->get_cam_height_max();
        double voxel_size = config->get_mapping_voxel_size();

        // set cam0
        auto dev0 = dev_list->getDevice(cam_idx0);
        dev0->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
        dev0->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

        std::shared_ptr<ob::Pipeline> pipe0 = std::make_shared<ob::Pipeline>(dev0);
        Eigen::Matrix4d TF0 = string_to_TF(config->get_cam_tf(0));

        Eigen::Matrix3d TF0_R = TF0.block(0,0,3,3);
        Eigen::Vector3d TF0_t = TF0.block(0,3,3,1);

        auto depth_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_DEPTH);

        for(size_t p = 0; p < depth_profile_list0->count(); p++)
        {
            auto profile = depth_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
            printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
        }

        auto color_profile_list0 = pipe0->getStreamProfileList(OB_SENSOR_COLOR);

        for(size_t p = 0; p < color_profile_list0->count(); p++)
        {
            auto profile = color_profile_list0->getProfile(p)->as<ob::VideoStreamProfile>();
            printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
        }


        auto depth_profile0 = depth_profile_list0->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
        printf("[ORBBEC] depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", depth_profile_idx, depth_profile0->width(), depth_profile0->height(), depth_profile0->fps(), depth_profile0->format());

        cur_w_depth = depth_profile0->width();
        cur_h_depth = depth_profile0->height();

        auto color_profile0 = color_profile_list0->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();
        printf("[ORBBEC] color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", color_profile_idx, color_profile0->width(), color_profile0->height(), color_profile0->fps(), color_profile0->format());

        cur_w_color = color_profile0->width();
        cur_h_color = color_profile0->height();

        std::shared_ptr<ob::Config> config0 = std::make_shared<ob::Config>();
        config0->disableAllStream();
        config0->enableStream(depth_profile0);
        config0->enableStream(color_profile0);
        config0->setAlignMode(ALIGN_DISABLE);

        ob::PointCloudFilter point_cloud0;
        bool filter_param_set0 = false;
        pipe0->start(config0, [&](std::shared_ptr<ob::FrameSet> fs)
        {
            try
            {
                if(fs->depthFrame() != nullptr)
                {
                    uint64_t ts = fs->depthFrame()->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    if(!filter_param_set0)
                    {
                        auto camera_param = pipe0->getCameraParam();
                        point_cloud0.setCameraParam(camera_param);
                        point_cloud0.setCreatePointFormat(OB_FORMAT_POINT);
                        filter_param_set0 = true;
                    }
                    auto scale = fs->depthFrame()->getValueScale();
                    point_cloud0.setPositionDataScaled(scale);

                    std::shared_ptr<ob::Frame> frame = point_cloud0.process(fs);
                    OBPoint *point = reinterpret_cast<OBPoint *>(frame->data());

                    int w = fs->depthFrame()->width();
                    int h = fs->depthFrame()->height();

                    // pts reserve
                    std::vector<Eigen::Vector3d> pts;
                    pts.reserve((w * h) / 10);

                    #pragma omp parallel
                    {
                        std::vector<Eigen::Vector3d> local_pts;
                        local_pts.reserve(w * h / (10 * omp_get_num_threads()));
                        #pragma omp for nowait
                        for(int idx = 0; idx < w * h; ++idx)
                        {
                            double x = point[idx].x / 1000.0;
                            double y = point[idx].y / 1000.0;
                            double z = point[idx].z / 1000.0;
                            if(x == 0 && y == 0 && z == 0)
                            {
                                continue;
                            }

                            Eigen::Vector3d P(x, y, z);
                            Eigen::Vector3d _P = TF0_R * P + TF0_t;

                            if(_P[2] < z_min || _P[2] > z_max)
                            {
                                continue;
                            }
                            if(_P[0] > x_min && _P[0] < x_max && _P[1] > y_min && _P[1] < y_max)
                            {
                                continue;
                            }

                            _P[2] = 0;

                            local_pts.push_back(_P);
                        }

                        #pragma omp critical
                        {
                            pts.insert(pts.end(), local_pts.begin(), local_pts.end());
                        }
                    }

                    pts = voxel_filtering(pts, voxel_size);
                    cur_pts_size0 = pts.size();

                    TIME_PTS scan;
                    scan.t = t;
                    scan.pts = std::move(pts);

                    depth_que[0].push(scan);
                    if(depth_que[0].unsafe_size() > 10)
                    {
                        TIME_PTS dummy;
                        depth_que[0].try_pop(dummy);
                    }

                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_scan[0] = std::move(scan);
                    }
                }

                if(fs->colorFrame() != nullptr)
                {
                    if(!is_connected[0])
                    {
                        is_connected[0] = true;
                    }

                    uint64_t ts = fs->colorFrame()->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();
                    cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
                    cv::Mat img;
                    cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);

                    if(!img.empty())
                    {
                        TIME_IMG time_img;
                        time_img.t = t;
                        time_img.img = img;

                        if(img_que[0].unsafe_size() > 10)
                        {
                            TIME_IMG dummy;
                            img_que[0].try_pop(dummy);
                        }

                        cv::Mat plot_img;
                        cv::resize(img, plot_img, cv::Size(160, 90));
                        {
                            std::lock_guard<std::mutex> lock(mtx);
                            cur_img[0] = plot_img.clone();
                            cur_time_img[0] = std::move(time_img);
                        }
                    }
                }
            }
            catch(const ob::Error &e)
            {
                QString str = QString::fromLocal8Bit(e.getMessage());
                logger->write_log("[ORBBEC] " + str, "Red");
            }
        });

        // set cam1
        auto dev1 = dev_list->getDevice(cam_idx1);
        dev1->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
        dev1->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

        std::shared_ptr<ob::Pipeline> pipe1 = std::make_shared<ob::Pipeline>(dev1);
        Eigen::Matrix4d TF1 = string_to_TF(config->get_cam_tf(1));
        Eigen::Matrix3d TF1_R = TF1.block(0,0,3,3);
        Eigen::Vector3d TF1_t = TF1.block(0,3,3,1);

        auto depth_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_DEPTH);
        auto color_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_COLOR);

        auto depth_profile1 = depth_profile_list1->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
        auto color_profile1 = color_profile_list1->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();

        std::shared_ptr<ob::Config> config1 = std::make_shared<ob::Config>();
        config1->disableAllStream();
        config1->enableStream(depth_profile1);
        config1->enableStream(color_profile1);
        config1->setAlignMode(ALIGN_DISABLE);

        ob::PointCloudFilter point_cloud1;
        bool filter_param_set1 = false;
        pipe1->start(config1, [&](std::shared_ptr<ob::FrameSet> fs)
        {
            try
            {
                if(fs->depthFrame() != nullptr)
                {
                    uint64_t ts = fs->depthFrame()->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    if(!filter_param_set1)
                    {
                        auto camera_param = pipe1->getCameraParam();
                        point_cloud1.setCameraParam(camera_param);
                        point_cloud1.setCreatePointFormat(OB_FORMAT_POINT);
                        filter_param_set1 = true;
                    }

                    auto scale = fs->depthFrame()->getValueScale();
                    point_cloud1.setPositionDataScaled(scale);

                    std::shared_ptr<ob::Frame> frame = point_cloud1.process(fs);
                    OBPoint *point = reinterpret_cast<OBPoint *>(frame->data());

                    int w = fs->depthFrame()->width();
                    int h = fs->depthFrame()->height();

                    // pts reserve
                    std::vector<Eigen::Vector3d> pts;
                    pts.reserve((w * h) / 10);

                    #pragma omp parallel
                    {
                        std::vector<Eigen::Vector3d> local_pts;
                        local_pts.reserve(w * h / (10 * omp_get_num_threads()));
                        #pragma omp for nowait
                        for(int idx = 0; idx < w * h; ++idx)
                        {
                            double x = point[idx].x / 1000.0;
                            double y = point[idx].y / 1000.0;
                            double z = point[idx].z / 1000.0;
                            if(x == 0 && y == 0 && z == 0)
                            {
                                continue;
                            }

                            Eigen::Vector3d P(x, y, z);
                            Eigen::Vector3d _P = TF1_R * P + TF1_t;

                            if(_P[2] < z_min || _P[2] > z_max)
                            {
                                continue;
                            }
                            if(_P[0] > x_min && _P[0] < x_max && _P[1] > y_min && _P[1] < y_max)
                            {
                                continue;
                            }

                            _P[2] = 0;

                            local_pts.push_back(_P);
                        }

                        #pragma omp critical
                        {
                            pts.insert(pts.end(), local_pts.begin(), local_pts.end());
                        }
                    }

                    pts = voxel_filtering(pts, voxel_size);
                    cur_pts_size1 = pts.size();

                    TIME_PTS scan;
                    scan.t = t;
                    scan.pts = std::move(pts);

                    depth_que[1].push(scan);
                    if(depth_que[1].unsafe_size() > 10)
                    {
                        TIME_PTS dummy;
                        depth_que[1].try_pop(dummy);
                    }

                    {
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_scan[1] = std::move(scan);
                    }
                }

                if(fs->colorFrame() != nullptr)
                {
                    if(!is_connected[1])
                    {
                        is_connected[1] = true;
                    }

                    uint64_t ts = fs->colorFrame()->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();
                    cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
                    cv::Mat img;
                    cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);

                    if(!img.empty())
                    {
                        TIME_IMG time_img;
                        time_img.t = t;
                        time_img.img = img;

                        if(img_que[1].unsafe_size() > 10)
                        {
                            TIME_IMG dummy;
                            img_que[1].try_pop(dummy);
                        }

                        cv::Mat plot_img;
                        cv::resize(img, plot_img, cv::Size(160, 90));
                        {
                            std::lock_guard<std::mutex> lock(mtx);
                            cur_img[1] = plot_img.clone();
                            cur_time_img[1] = std::move(time_img);
                        }
                    }
                }
            }
            catch(const ob::Error &e)
            {
                QString str = QString::fromLocal8Bit(e.getMessage());
                logger->write_log("[ORBBEC] " + str, "Red");
            }
        });

        // get intrinsic cam 0
        {
            auto camera_param = pipe0->getCameraParam();

            std::lock_guard<std::mutex> lock(mtx);
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

            extrinsic[0] = string_to_TF(config->get_cam_tf(0));
        }
        printf("[ORBBEC] intrinsic0, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[0].fx, intrinsic[0].fy, intrinsic[0].cx, intrinsic[0].cy);

        // get intrinsic cam 1
        {
            auto camera_param = pipe1->getCameraParam();

            std::lock_guard<std::mutex> lock(mtx);
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

            extrinsic[1] = string_to_TF(config->get_cam_tf(1));
        }
        printf("[ORBBEC] intrinsic1, fx:%f, fy:%f, cx:%f, cy:%f\n", intrinsic[1].fx, intrinsic[1].fy, intrinsic[1].cx, intrinsic[1].cy);

        is_param_loaded = true;

        printf("[ORBBEC] grab loop started\n");
        while(grab_flag)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }

        pipe0->stop();
        pipe1->stop();

        printf("[ORBBEC] grab loop stopped\n");
    }
}*/

void ORBBEC::set_config_module(CONFIG* _config)
{
    config = _config;
}

void ORBBEC::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void ORBBEC::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}
