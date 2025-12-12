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
        cam_t[p] = 0.0;
        is_param_loaded[p] = false;
        cur_pts_size[p] = 0.0;
        extrinsic[p].setIdentity();
    }

    connect(this, SIGNAL(signal_restart(int)), this, SLOT(slot_restart(int)));
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
        is_connected[p] = false;

        grab_flag[p] = false;
        if(grab_thread[p] && grab_thread[p]->joinable())
        {
            grab_thread[p]->join();
        }
        grab_thread[p].reset();
    }
}

void ORBBEC::restart(int idx)
{
    is_connected[idx] = false;

    // close
    grab_flag[idx] = false;
    if(grab_thread[idx] && grab_thread[idx]->joinable())
    {
        grab_thread[idx]->join();
    }
    grab_thread[idx].reset();

    // open
    grab_flag[idx] = true;
    grab_thread[idx] = std::make_unique<std::thread>(&ORBBEC::grab_loop, this, idx);
}



QString ORBBEC::get_cam_info_str()
{
    QString connection_str = "connection:";
    for(int p = 0; p < config->get_cam_num(); p++)
    {
        QString connection = QString("%1,").arg(is_connected[p].load() ? "1" : "0");
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
    try
    {
        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_DEBUG);

        // check device
        ob::Context ctx;
        ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
        ctx.setLoggerToConsole(OB_LOG_SEVERITY_OFF);

        auto dev_list = ctx.queryDeviceList();
        int dev_count = dev_list->deviceCount();
        if(dev_count == 0)
        {
            grab_flag[idx] = false;
            log_error("no camera");
            return;
        }

        QString target_sn = config->get_cam_serial_number(idx);
        if(target_sn.isEmpty())
        {
            grab_flag[idx] = false;
            log_error("camera {} has empty serial number in config", idx);
            return;
        }

        int dev_idx = -1;
        for(int d = 0; d < dev_count; d++)
        {
            QString sn = dev_list->getDevice(d)->getDeviceInfo()->serialNumber();
            if(sn == target_sn)
            {
                dev_idx = d;
                break;
            }
        }

        if(dev_idx < 0)
        {
            grab_flag[idx] = false;
            log_error("camera not found, idx:{}, sn:{}", idx, target_sn.toStdString());
            return;
        }

        double x_min = config->get_robot_size_x_min(), x_max = config->get_robot_size_x_max();
        double y_min = config->get_robot_size_y_min(), y_max = config->get_robot_size_y_max();
        double z_min = config->get_cam_height_min(),  z_max = config->get_cam_height_max();
        double voxel_size = config->get_mapping_voxel_size();

        // set cam
        auto dev = dev_list->getDevice(dev_idx);
        dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
        dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);
        log_info("detected camera, idx:{}, dev_idx:{}, sn:{}, tf:{}", idx, dev_idx, target_sn.toStdString(), config->get_cam_tf(idx).toStdString());

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

        if(idx == 0)
        {
            cam_config->disableAllStream();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        cam_config->enableStream(depth_profile);
        cam_config->enableStream(color_profile);
        cam_config->setAlignMode(ALIGN_DISABLE);

        pipe->start(cam_config);

        log_info("grab loop started, idx:{}", idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(idx * 500));

        int cnt = 0;

        ob::PointCloudFilter point_cloud;
        bool filter_param_set = false;
        while(grab_flag[idx])
        {
            // double pre_loop_time = get_time();

            auto fs = pipe->waitForFrames(100);
            if(!fs)
            {
                continue;
            }

            try
            {
                if(auto df = fs->depthFrame())
                {
                    uint64_t ts = df->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    cam_t[idx].store(get_time());

                    if(!filter_param_set)
                    {
                        auto camera_param = pipe->getCameraParam();
                        point_cloud.setCameraParam(camera_param);
                        point_cloud.setCreatePointFormat(OB_FORMAT_POINT);
                        filter_param_set = true;
                    }

                    float scale = df->getValueScale();
                    point_cloud.setPositionDataScaled(scale);

                    std::shared_ptr<ob::Frame> frame = point_cloud.process(fs);
                    OBPoint *point = (OBPoint *)frame->data();
                    int w = df->width();
                    int h = df->height();

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

                            //_P[2] = 0;
                            pts.push_back(_P);
                        }
                    }

                    //std::cout << idx << ", pts.size(): " << pts.size() << std::endl;

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
                else
                {
                    cnt ++;
                    if(cnt > 100)
                    {
                        log_info("signal_restart requested, idx:{}, cnt:{}", idx, cnt);
                        cnt = 0;
                        Q_EMIT signal_restart(idx);
                        break;
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
                    log_info("intrinsic{}, fx:{}, fy:{}, cx:{}, cy:{}", idx, intrinsic[idx].fx, intrinsic[idx].fy, intrinsic[idx].cx, intrinsic[idx].cy);

                    extrinsic[idx] = TF;
                }
            }
            catch(ob::Error &e)
            {
                log_info("inner ob::Error idx:{}, name:{}, args:{}, msg:{}, type:{}", idx, e.getName(), e.getArgs(), e.getMessage(), static_cast<int>(e.getExceptionType()));
                Q_EMIT signal_restart(idx);
                break;
            }
            catch(const std::exception& e)
            {
                log_info("inner std::exception idx:{}, what:{}", idx, e.what());
                Q_EMIT signal_restart(idx);
                break;
            }

            // =======================
            // loop time check
            // =======================

            // double cur_loop_time = get_time();
            // double delta_loop_time = cur_loop_time - pre_loop_time;
            // log_warn("cam_{} loop time, dt:{}", idx, delta_loop_time);
        }

        is_connected[idx] = false;

        try
        {
            pipe->stop();
        }
        catch (const ob::Error& e)
        {
            log_info("stop() ob::Error idx:{}, name:{}, args:{}, msg:{}, type:{}", idx, e.getName(), e.getArgs(), e.getMessage(), static_cast<int>(e.getExceptionType()));
        }

        pipe.reset();
        log_info("grab loop stopped, idx:{}", idx);
    }
    catch(ob::Error &e)
    {
        log_info("ob::Error idx:{}, name:{}, args:{}, msg:{}, type:{}", idx, e.getName(), e.getArgs(), e.getMessage(), static_cast<int>(e.getExceptionType()));
    }
}

void ORBBEC::slot_restart(int idx)
{
    log_info("slot_restart, idx:{}", idx);
    restart(idx);
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
