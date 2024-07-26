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

void CAM::init()
{
    // check simulation mode
    if(config->SIM_MODE == 1)
    {
        printf("[CAM] simulation mode\n");
        return;
    }

    // start grab loop
    if (grab_thread == NULL)
    {
        grab_flag = true;
        grab_thread = new std::thread(&CAM::grab_loop, this);
    }
}

void CAM::grab_loop()
{
#ifndef USE_SIM
    // check device
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_FATAL);
    ob::Context ctx;
    auto devList = ctx.queryDeviceList();
    int devCount = devList->deviceCount();
    if(devCount != 1)
    {
        logger->PrintLog("[CAM] only support one device, failed", "Red", true, false);
        return;
    }

    auto dev = devList->getDevice(0);
    dev->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);

    // get camera
    std::shared_ptr<ob::Pipeline> pipe = std::make_shared<ob::Pipeline>(dev);
    auto dev_info = dev->getDeviceInfo();
    QString sn = dev_info->serialNumber();

    serial_number = sn;
    QString serial_str = "[CAM] camera serial : " + sn;
    logger->PrintLog(serial_str, "Green", true, false);

    // params
    QStringList cam_tf = config->CAM_TF.split(",");
    if(cam_tf.size() != 6)
    {
        logger->PrintLog("[CAM] cam tf invalid", "Red", true, false);
        return;
    }
    is_connected = true;

    Eigen::Matrix4d T_cam = zyx_tranformation(cam_tf[0].toDouble(), cam_tf[1].toDouble(), cam_tf[2].toDouble(), cam_tf[3].toDouble(), cam_tf[4].toDouble(), cam_tf[5].toDouble());

    // setting
    std::shared_ptr<ob::Config> cam_config = std::make_shared<ob::Config>();
    cam_config->disableAllStream();

    auto depthProfileList = pipe->getStreamProfileList(OB_SENSOR_DEPTH);
    std::shared_ptr<ob::VideoStreamProfile> depthProfile = depthProfileList->getVideoStreamProfile(640, 400, OB_FORMAT_Y16, 30);
    {
        depthProfile = std::const_pointer_cast<ob::StreamProfile>(depthProfileList->getProfile(0))->as<ob::VideoStreamProfile>();

        int w = depthProfile->width();
        int h = depthProfile->height();
        int fps = depthProfile->fps();
        printf("[CAM] depth w:%d, h:%d, fps:%d\n", w, h, fps);
    }
    cam_config->enableStream(depthProfile);

    auto colorProfileList = pipe->getStreamProfileList(OB_SENSOR_COLOR);
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = colorProfileList->getVideoStreamProfile(640, 400, OB_FORMAT_RGB, 30);
    {
        colorProfile = std::const_pointer_cast<ob::StreamProfile>(colorProfileList->getProfile(0))->as<ob::VideoStreamProfile>();

        int w = colorProfile->width();
        int h = colorProfile->height();
        int fps = colorProfile->fps();
        printf("[CAM] color, w:%d, h:%d, fps:%d\n", w, h, fps);
    }
    cam_config->enableStream(colorProfile);
    cam_config->setAlignMode(ALIGN_DISABLE);

    int drop_cnt_d = 100;
    int drop_cnt_c = 100;

    // frame callback left
    pipe->start(cam_config, [&](std::shared_ptr<ob::FrameSet> frameset)
    {
        bool is_grab_depth = true;
        if(frameset->depthFrame() != nullptr)
        {
            // initial drop
            if(drop_cnt_d > 0)
            {
                drop_cnt_d--;
                is_grab_depth = false;
            }

            double pc_t = get_time();

            uint64_t ts = frameset->depthFrame()->systemTimeStamp();
            double depth_t = (double)ts/1000.0;

            if(is_sync_d)
            {
                is_sync_d = false;
                offset_t_d = pc_t - depth_t;

                is_synced_d = true;
                printf("[CAM] sync, offset_t_d: %f\n", (double)offset_t_d);
            }

            // check cam, mobile sync
            if(is_synced_d == false || mobile->is_synced == false)
            {
                is_grab_depth = false;
            }

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                is_grab_depth = false;
            }

            // wait
            double t = depth_t + offset_t_d;
            if(t > mobile->last_pose_t && grab_flag)
            {
                is_grab_depth = false;
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get boundary mobile pose
            int idx = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t)
                {
                    idx = p;
                    break;
                }
            }

            // check
            if(idx == -1)
            {
                // drop
                printf("[CAM] depth, invalid mobile poses\n");
                is_grab_depth = false;
            }

            // precise deskewing
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Vector3d min_pose = Eigen::Vector3d(0,0,0);
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_pose = pose_storage[p].pose;
                }
            }

            if(is_grab_depth)
            {
                // get point cloud
                ob::PointCloudFilter pointCloud;
                auto cameraParam = pipe->getCameraParam();

                pointCloud.setCameraParam(cameraParam);
                pointCloud.setCreatePointFormat(OB_FORMAT_POINT);

                auto depthValueScale = frameset->depthFrame()->getValueScale();
                pointCloud.setPositionDataScaled(depthValueScale);

                std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);
                OBPoint *point = (OBPoint *)frame->data();

                int w = 640;
                int h = 400;
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
                            Eigen::Vector3d P = T_cam.block(0,0,3,3)*Eigen::Vector3d(x,y,z)+T_cam.block(0,3,3,1);
                            if(P[2] < config->CAM_HEIGHT_MIN || P[2] > config->CAM_HEIGHT_MAX)
                            {
                                continue;
                            }

                            pts.push_back(P);
                        }
                    }
                }

                MOBILE_POSE mo;
                mo.t= min_t;
                mo.pose = min_pose;

                CAM_DEPTH_FRAME frm;
                frm.t = t;
                frm.pts = pts;
                frm.mo = mo;
            }
        }

        bool is_grab_color = true;
        if(frameset->colorFrame() != nullptr)
        {
            // initial drop
            if(drop_cnt_c > 0)
            {
                drop_cnt_c--;
                is_grab_color = false;
            }

            double pc_t = get_time();

            uint64_t ts = frameset->colorFrame()->systemTimeStamp();
            double color_t = (double)ts/1000.0;

            if(is_sync_c)
            {
                is_sync_c = false;
                offset_t_c = pc_t - color_t;

                is_synced_c = true;
                printf("[CAM] sync, offset_t_c: %f\n", (double)offset_t_c);
            }

            // check cam, mobile sync
            if(is_synced_c == false || mobile->is_synced == false)
            {
                is_grab_color = false;
            }

            // check
            if(mobile->get_pose_storage_size() != MO_STORAGE_NUM)
            {
                is_grab_color = false;
            }

            // wait
            double t = color_t + offset_t_d;
            if(t > mobile->last_pose_t && grab_flag)
            {
                is_grab_color = false;
            }

            // get mobile poses
            std::vector<MOBILE_POSE> pose_storage = mobile->get_pose_storage();

            // get boundary mobile pose
            int idx = -1;
            for(size_t p = pose_storage.size()-1; p >= 0; p--)
            {
                if(pose_storage[p].t < t)
                {
                    idx = p;
                    break;
                }
            }

            // check
            if(idx == -1)
            {
                // drop
                printf("[CAM] color, invalid mobile poses\n");
                is_grab_color = false;
            }

            // precise deskewing
            double min_t = 0;
            double min_dt = 99999999;
            Eigen::Vector3d min_pose = Eigen::Vector3d(0,0,0);
            for(size_t p = 0; p < pose_storage.size(); p++)
            {
                double dt = std::abs(pose_storage[p].t - t);
                if(dt < min_dt)
                {
                    min_t = pose_storage[p].t;
                    min_dt = dt;
                    min_pose = pose_storage[p].pose;
                }
            }

            if(is_grab_color)
            {
                // get color image
                std::shared_ptr<ob::ColorFrame> colorFrame = frameset->colorFrame();
                cv::Mat img_c_raw(1, colorFrame->dataSize(), CV_8UC1, colorFrame->data());
                cv::Mat img_c = cv::imdecode(img_c_raw, cv::IMREAD_COLOR);
                cv::flip(img_c, img_c, -1);

                if(!img_c.empty())
                {
                    // get gray image
                    cv::Mat img;
                    cv::cvtColor(img_c, img, cv::COLOR_BGR2GRAY);

                    MOBILE_POSE mo;
                    mo.t= min_t;
                    mo.pose = min_pose;

                    CAM_IMG_FRAME frm;
                    frm.t = t;
                    frm.img = img;
                    frm.mo = mo;
                }
            }
        }
    });

    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    pipe->stop();

    #endif
}

QString CAM::get_sn()
{
    mtx.lock();
    QString _sn = serial_number;
    mtx.unlock();

    return _sn;
}

cv::Mat CAM::get_cur_img()
{
    mtx.lock();
    cv::Mat _cur_img = cur_img.clone();
    mtx.unlock();

    return _cur_img;
}

cv::Mat CAM::get_cur_img_color()
{
    mtx.lock();
    cv::Mat _cur_img_color = cur_img_color.clone();
    mtx.unlock();

    return _cur_img_color;
}

std::vector<Eigen::Vector3d> CAM::get_cur_scan()
{
    mtx.lock();
    std::vector<Eigen::Vector3d> res = cur_scan;
    mtx.unlock();

    return res;
}

Eigen::Matrix4d CAM::zyx_tranformation(double x, double y, double z, double rx, double ry, double rz)
{
    Eigen::Matrix4d res;
    res.setIdentity();

    Eigen::AngleAxisd Rz(rz*D2R, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd Ry(ry*D2R, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rx(rx*D2R, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = Rz * Ry * Rx;

    res.block(0,0,3,3) = q.matrix();
    res(0,3) = x;
    res(1,3) = y;
    res(2,3) = z;

    return res;
}

