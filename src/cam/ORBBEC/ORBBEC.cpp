#include "ORBBEC.h"

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
    for(int p=0; p<max_cam_cnt; p++)
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

    printf("[ORBBEC] init\n");
}

void ORBBEC::open()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        printf("[ORBBEC] simulation mode\n");
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

void ORBBEC::get_cam_exist_check()
{
//    // check device
//    ob::Context ctx;
//    ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
//    ctx.setLoggerToConsole(OB_LOG_SEVERITY_OFF);

//    auto dev_list = ctx.queryDeviceList();
//    int dev_count = dev_list->deviceCount();

//    if(dev_count == 0)
//    {
//        grab_flag[idx] = false;

//        logger->write_log("[ORBBEC] no camera", "Red");
//        return;
//    }

////    QString sn = dev_list->getDevice(idx)->getDeviceInfo()->serialNumber();
//    int dev_match_success = false;
//    int dev_idx = 0;
//    QString sn_connected = "";
//    for(int i=0; i<dev_count; i++)
//    {
//        QString sn = dev_list->getDevice(i)->getDeviceInfo()->serialNumber();
//        if(sn == config->get_cam_serial_number(idx))
//        {
//            dev_match_success = true;
//            dev_idx = i;
//            sn_connected = sn;
//            break;
//        }
//    }
//    if(dev_match_success == false)
//    {
//        static QAtomicInt already_updated = 0; // 전역 또는 static 변수

//        if(dev_match_success == false)
//        {
//            grab_flag[idx] = false;

//            if(already_updated.testAndSetRelaxed(0, 1)) // 처음 호출한 스레드만 true
//            {
//                QString serial_number_str[max_cam_cnt];
//                for (int i = 0 ; i < dev_count; i++)
//                {
//                    serial_number_str[i] = dev_list->getDevice(i)->getDeviceInfo()->serialNumber();
//                }
//                config->set_cam_order(serial_number_str);
//                logger->write_log("[ORBBEC] no camera match -> saved serial numbers", "Yellow");
//            }

//            return;
//        }
//    }
}

QString ORBBEC::get_cam_info_str()
{
    QString connection_str = "connection:";
    int camNum = config->get_cam_num();

    for(int p = 0; p < camNum; p++)
    {
        QString connection = QString("%1,").arg(is_connected[p] ? "1" : "0");
        connection_str += connection;
    }

    QString pts_str = "pts:";
    for(int p = 0; p < camNum; p++)
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
        return;
    }

//    QString sn = dev_list->getDevice(idx)->getDeviceInfo()->serialNumber();
    int dev_match_success = false;
    int dev_idx = 0;
    QString sn_connected = "";
    for(int i=0; i<dev_count; i++)
    {
        QString sn = dev_list->getDevice(i)->getDeviceInfo()->serialNumber();
        if(sn == config->get_cam_serial_number(idx))
        {
            dev_match_success = true;
            dev_idx = i;
            sn_connected = sn;
            break;
        }
    }
    if(dev_match_success == false)
    {
        grab_flag[idx] = false;
        dev_match_success = true;
        int expected = 0;

        if (already_updated.compare_exchange_strong(expected, 1))
        {
            QString serial_number_str[max_cam_cnt];
            for (int i = 0 ; i < dev_count; i++)
            {
                serial_number_str[i] = dev_list->getDevice(i)->getDeviceInfo()->serialNumber();
            }
            config->set_cam_order(serial_number_str);
            logger->write_log("[ORBBEC] no camera match -> saved serial numbers", "Yellow");
        }

        return;
    }
    //if(sn != config->get_cam_serial_number(idx))
    //{
    //    grab_flag[idx] = false;

    //    logger->write_log("[ORBBEC] no camera", "Red");
    //    return;
    //}
//    get_cam_exist_check();

    logger->write_log(QString("[ORBBEC] connected serial number, sn:%1").arg(sn_connected));

    double x_min = config->get_robot_size_x_min(), x_max = config->get_robot_size_x_max();
    double y_min = config->get_robot_size_y_min(), y_max = config->get_robot_size_y_max();
    double z_min = config->get_cam_height_min(),  z_max = config->get_cam_height_max();
    double voxel_size = config->get_mapping_voxel_size();

    // set cam
    auto dev = dev_list->getDevice(dev_idx);
    dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);

    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    Eigen::Matrix4d TF = string_to_TF(config->get_cam_tf(idx));
    Eigen::Matrix3d TF_R = TF.block(0,0,3,3);
    Eigen::Vector3d TF_t = TF.block(0,3,3,1);

    auto depth_profile_list = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    auto depth_profile = depth_profile_list->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
    printf("[ORBBEC] depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", depth_profile_idx, depth_profile->width(), depth_profile->height(), depth_profile->fps(), depth_profile->format());
    cur_w_depth = depth_profile->width();
    cur_h_depth = depth_profile->height();

    //for(size_t p = 0; p < depth_profile_list->count(); p++)
    //{
    //    auto profile = depth_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
    //    printf("depth_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
    //}

    auto color_profile_list = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    auto color_profile = color_profile_list->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();
    printf("[ORBBEC] color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", color_profile_idx, color_profile->width(), color_profile->height(), color_profile->fps(), color_profile->format());
    cur_w_color = color_profile->width();
    cur_h_color = color_profile->height();

    //for(size_t p = 0; p < color_profile_list->count(); p++)
    //{
    //    auto profile = color_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
    //    printf("color_profile(%d), w:%d, h:%d, fps:%d, format:%d\n", p, profile->width(), profile->height(), profile->fps(), profile->format());
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
                OBPoint *point = reinterpret_cast<OBPoint *>(frame->data());

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

                        _P[2] = 0;
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
//                cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());
//                cv::Mat img;
//                cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);
                const uchar* dataPtr = static_cast<const uchar*>(colorFrame->data());
                std::vector<uchar> buf(dataPtr, dataPtr + colorFrame->dataSize());
                cv::Mat encoded(1, colorFrame->dataSize(), CV_8UC1, buf.data());
                cv::Mat img = cv::imdecode(encoded, cv::IMREAD_COLOR);

                if(!img.empty())
                {
                    TIME_IMG time_img;
                    time_img.t = t;
                    time_img.img = img;

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
                        cur_img[idx] = plot_img.clone();
                        cur_time_img[idx] = std::move(time_img);
                    }
                }
            }


        }
        catch(const ob::Error &e)
        {
            QString str = QString::fromLocal8Bit(e.getMessage());
            logger->write_log("[ORBBEC] " + str, "Red");
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
            printf("[ORBBEC] intrinsic%d, fx:%f, fy:%f, cx:%f, cy:%f\n", idx, intrinsic[idx].fx, intrinsic[idx].fy, intrinsic[idx].cx, intrinsic[idx].cy);
        }
    });

    printf("[ORBBEC] grab loop started\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(idx * 500));
    while(grab_flag[idx])
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    pipe->stop();
    printf("[ORBBEC] grab loop stopped\n");
}

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
