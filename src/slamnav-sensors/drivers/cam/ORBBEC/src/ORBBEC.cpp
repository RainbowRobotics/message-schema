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
    persistent_ctx = std::make_shared<ob::Context>();
    persistent_ctx->setLoggerSeverity(OB_LOG_SEVERITY_OFF);


    persistent_ctx->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed,
                                                     std::shared_ptr<ob::DeviceList> added) {
        if(removed && removed->deviceCount() > 0)
        {
            for(uint32_t i = 0; i < removed->deviceCount(); i++)
            {
                auto info = removed->getDevice(i)->getDeviceInfo();
                QString sn = info->serialNumber();
                log_warn("Device removed: {}", sn.toStdString());

                for(int idx = 0; idx < config->get_cam_num(); idx++)
                {
                    if(config->get_cam_serial_number(idx) == sn)
                    {
                        device_ready[idx] = false;
                        is_connected[idx] = false;
                    }
                }
            }
        }

        if(added && added->deviceCount() > 0)
        {
            for(uint32_t i = 0; i < added->deviceCount(); i++)
            {
                auto info = added->getDevice(i)->getDeviceInfo();
                QString sn = info->serialNumber();
                log_info("Device added: {}", sn.toStdString());

                for(int idx = 0; idx < config->get_cam_num(); idx++)
                {
                    if(config->get_cam_serial_number(idx) == sn)
                    {
                        device_ready[idx] = true;
                    }
                }
            }
        }
    });

    for(int i = 0; i < config->get_cam_num(); i++)
    {
        extrinsic[i] = string_to_TF(config->get_cam_tf(i));
        device_ready[i] = false;
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
    log_info("restart requested, idx:{}", idx);

    grab_flag[idx] = false;
    if(grab_thread[idx] && grab_thread[idx]->joinable())
    {
        grab_thread[idx]->join();
    }
    grab_thread[idx].reset();

    bool reboot_success = false;
    try
    {
        QString target_sn = config->get_cam_serial_number(idx);
        auto dev_list = persistent_ctx->queryDeviceList();

        for(uint32_t d = 0; d < dev_list->deviceCount(); d++)
        {
            auto dev = dev_list->getDevice(d);
            QString sn = dev->getDeviceInfo()->serialNumber();

            if(sn == target_sn)
            {
                log_info("Rebooting device, idx:{}, sn:{}", idx, sn.toStdString());

                device_ready[idx] = false;
                dev->reboot();
                reboot_success = true;
                break;
            }
        }
    }
    catch(const ob::Error& e)
    {
        log_warn("device->reboot() failed, idx:{}, msg:{}", idx, e.getMessage());
    }
    catch(const std::exception& e)
    {
        log_warn("device->reboot() exception, idx:{}, what:{}", idx, e.what());
    }

    if(reboot_success)
    {
        const int MAX_WAIT_SEC = 10;
        int waited = 0;

        log_info("Waiting for device to reconnect, idx:{}", idx);

        while(!device_ready[idx] && waited < MAX_WAIT_SEC)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            waited++;
            log_debug("Waiting... {}/{}s, idx:{}", waited, MAX_WAIT_SEC, idx);
        }

        if(device_ready[idx])
        {
            log_info("Device reconnected after reboot, idx:{}, waited:{}s", idx, waited);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            log_warn("Device not reconnected after {}s, idx:{}", MAX_WAIT_SEC, idx);
        }
    }
    else
    {
        log_info("Reboot not performed, waiting 2s before retry, idx:{}", idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }

    grab_flag[idx] = true;
    grab_thread[idx] = std::make_unique<std::thread>(&ORBBEC::grab_loop, this, idx);

    log_info("restart complete, idx:{}", idx);
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
                                                                                 .arg((int)cur_w_color[0]).arg((int)cur_h_color[0])
                                                                                 .arg((int)cur_w_depth[0]).arg((int)cur_h_depth[0])
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
    std::shared_ptr<ob::Device> dev;
    std::shared_ptr<ob::Pipeline> pipe;

    try
    {
        if(idx == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        }

        ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);

        // check device
        ob::Context ctx;
        ctx.setLoggerSeverity(OB_LOG_SEVERITY_OFF);
        ctx.setLoggerToConsole(OB_LOG_SEVERITY_OFF);

        QString target_sn = config->get_cam_serial_number(idx);
        if(target_sn.isEmpty())
        {
            grab_flag[idx] = false;
            log_error("camera {} has empty serial number in config", idx);
            return;
        }

        int retry = 0;
        int dev_idx = -1;
        constexpr int reconnection_max_try = 5;
        while(retry < reconnection_max_try && dev_idx < 0)
        {
            auto dev_list = ctx.queryDeviceList();
            int dev_count = dev_list->deviceCount();

            if(dev_count == 0)
            {
                retry++;
                log_warn("no camera found, retry {}/{}, idx:{}", retry, reconnection_max_try, idx);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                continue;
            }

            for(int d = 0; d < dev_count; d++)
            {
                try
                {
                    QString sn = dev_list->getDevice(d)->getDeviceInfo()->serialNumber();
                    if(sn == target_sn)
                    {
                        dev_idx = d;
                        dev = dev_list->getDevice(d);
                        break;
                    }
                }
                catch(const ob::Error& e)
                {
                    log_error("failed to get device info: idx:{}, msg:{}", d, e.getMessage());
                    continue;
                }
            }

            if(dev_idx < 0)
            {
                retry++;
                log_warn("camera not found in list, retry {}/{}, idx:{}, sn:{}",
                         retry, reconnection_max_try, idx, target_sn.toStdString());
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }


        if(dev_idx < 0 || !dev)
        {
            grab_flag[idx] = false;
            log_error("camera not found after {} retries, idx:{}, sn:{}", reconnection_max_try, idx, target_sn.toStdString());
            return;
        }

        log_info("detected camera, idx:{}, dev_idx:{}, sn:{}, tf:{}", idx, dev_idx, target_sn.toStdString(), config->get_cam_tf(idx).toStdString());

        double x_min = config->get_robot_size_x_min(), x_max = config->get_robot_size_x_max();
        double y_min = config->get_robot_size_y_min(), y_max = config->get_robot_size_y_max();
        double z_min = config->get_cam_height_min(),  z_max = config->get_cam_height_max();
        double voxel_size = config->get_mapping_voxel_size();

        // set cam
        try
        {
            dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
            dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, false);
        }
        catch(const ob::Error& e)
        {
            log_warn("failed to set mirror property, idx:{}, msg:{}", idx, e.getMessage());
        }

        pipe = std::make_shared<ob::Pipeline>(dev);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        Eigen::Matrix4d TF = string_to_TF(config->get_cam_tf(idx));
        Eigen::Matrix3d TF_R = TF.block(0,0,3,3);
        Eigen::Vector3d TF_t = TF.block(0,3,3,1);

        int depth_profile_idx = config->get_cam_depth_profile(idx);
        int color_profile_idx = config->get_cam_color_profile(idx);
        if(depth_profile_idx == -1 || color_profile_idx == -1)
        {
            grab_flag[idx] = false;
            log_error("invalid camera {} profile idx", idx);
            return;
        }

        auto depth_profile_list = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
        if(depth_profile_idx >= depth_profile_list->count())
        {
            log_error("Invalid depth_profile_idx: {} (max {})", depth_profile_idx, depth_profile_list->count() - 1);
            for(size_t p = 0; p < depth_profile_list->count(); p++)
            {
                auto profile = depth_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
                log_info("depth_profile({}, w:{}, h:{}, fps:{}, format:{})", p, profile->width(), profile->height(), profile->fps(), static_cast<int>(profile->format()));
            }
            return;
        }

        auto depth_profile = depth_profile_list->getProfile(depth_profile_idx)->as<ob::VideoStreamProfile>();
        cur_w_depth[idx] = depth_profile->width();
        cur_h_depth[idx] = depth_profile->height();
        log_info("Current depth_profile({}, w:{}, h:{}, fps:{}, format:{})", depth_profile_idx, depth_profile->width(), depth_profile->height(), depth_profile->fps(), static_cast<int>(depth_profile->format()));

        auto color_profile_list = pipe->getStreamProfileList(OB_SENSOR_COLOR);
        if(color_profile_idx >= color_profile_list->count())
        {
            log_error("Invalid color_profile_idx: {} (max {})", color_profile_idx, color_profile_list->count() - 1);
            for(size_t p = 0; p < color_profile_list->count(); p++)
            {
                auto profile = color_profile_list->getProfile(p)->as<ob::VideoStreamProfile>();
                log_info("color_profile({}, w:{}, h:{}, fps:{}, format:{})", p, profile->width(), profile->height(), profile->fps(), static_cast<int>(profile->format()));
            }
            return;
        }

        auto color_profile = color_profile_list->getProfile(color_profile_idx)->as<ob::VideoStreamProfile>();
        cur_w_color[idx] = color_profile->width();
        cur_h_color[idx] = color_profile->height();
        log_info("Current color_profile({}, w:{}, h:{}, fps:{}, format:{})", color_profile_idx, color_profile->width(), color_profile->height(), color_profile->fps(), static_cast<int>(color_profile->format()));

        double depth_aspect = static_cast<double>(depth_profile->width()) / depth_profile->height();
        double color_aspect = static_cast<double>(color_profile->width()) / color_profile->height();
        if(std::abs(depth_aspect - color_aspect) > 0.1)
        {
            log_warn("Aspect ratio mismatch: depth={:.2f}, color={:.2f}", depth_aspect, color_aspect);
        }

        std::shared_ptr<ob::Config> cam_config = std::make_shared<ob::Config>();
        cam_config->enableStream(depth_profile);
        cam_config->enableStream(color_profile);
        cam_config->setAlignMode(ALIGN_DISABLE);

        pipe->start(cam_config);
        log_info("grab loop started, idx:{}", idx);

        std::this_thread::sleep_for(std::chrono::milliseconds(1500 + idx * 500));

        int no_depth_cnt = 0, no_color_cnt = 0;
        int timeout_cnt = 0;
        constexpr int MAX_NO_DEPTH_CNT = 50;
        constexpr int MAX_NO_COLOR_CNT = 50;
        constexpr int MAX_TIMEOUT_CNT = 20;

        ob::PointCloudFilter point_cloud;
        bool filter_param_set = false;
        while(grab_flag[idx])
        {
            try
            {
                auto fs = pipe->waitForFrames(150);
                if(!fs)
                {
                    timeout_cnt++;
                    if(timeout_cnt > MAX_TIMEOUT_CNT)
                    {
                        log_error("frame timeout exceeded, idx:{}, restarting", idx);
                        Q_EMIT signal_restart(idx);
                        break;
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(30));
                    continue;
                }

                timeout_cnt = 0;

                if(auto df = fs->depthFrame())
                {
                    no_depth_cnt = 0;

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
                else
                {
                    no_depth_cnt++;
                    if(no_depth_cnt > MAX_NO_DEPTH_CNT)
                    {
                        log_error("no depth frame for too long, idx:{}, restarting", idx);
                        Q_EMIT signal_restart(idx);
                        break;
                    }
                }

                if(auto cf = fs->colorFrame())
                {
                    if(!is_connected[idx])
                    {
                        is_connected[idx] = true;
                    }

                    uint64_t ts = cf->systemTimeStamp();
                    double t = static_cast<double>(ts) / 1000.0;

                    cv::Mat raw(cf->height(), cf->width(), CV_8UC3, cf->data());
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
                else
                {
                    no_color_cnt++;
                    if(no_color_cnt > MAX_NO_COLOR_CNT)
                    {
                        log_error("no color frame for too long, idx:{}, restarting", idx);
                        Q_EMIT signal_restart(idx);
                        break;
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
                log_error("ob::Error in loop, idx:{}, msg:{}", idx, e.getMessage());
                Q_EMIT signal_restart(idx);
                break;
            }
            catch(const std::exception& e)
            {
                log_error("std::exception in loop, idx:{}, what:{}", idx, e.what());
                Q_EMIT signal_restart(idx);
                break;
            }
        }
        is_connected[idx] = false;
    }
    catch(ob::Error &e)
    {
        is_connected[idx] = false;
        log_error("ob::Error in grab_loop, idx:{}, msg:{}", idx, e.getMessage());
    }
    catch(const std::exception& e)
    {
        is_connected[idx] = false;
        log_error("std::exception in grab_loop, idx:{}, what:{}", idx, e.what());
    }

    if(pipe)
    {
        try
        {
            pipe->stop();
        }
        catch (const ob::Error& e)
        {
            log_warn("pipe->stop() error, idx:{}, msg:{}", idx, e.getMessage());
        }

        pipe.reset();
    }
    dev.reset();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    log_info("grab loop stopped, idx:{}", idx);
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