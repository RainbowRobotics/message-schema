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
    dev0->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);

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

    auto depth_profile0 = depth_profile_list0->getProfile(27)->as<ob::VideoStreamProfile>();
    printf("[CAM] depth_profile(31), w:%d, h:%d, fps:%d, format:%d\n", depth_profile0->width(), depth_profile0->height(), depth_profile0->fps(), depth_profile0->format());

    auto color_profile0 = color_profile_list0->getProfile(123)->as<ob::VideoStreamProfile>();
    printf("[CAM] color_profile(119), w:%d, h:%d, fps:%d, format:%d\n", color_profile0->width(), color_profile0->height(), color_profile0->fps(), color_profile0->format());

    std::shared_ptr<ob::Config> config0 = std::make_shared<ob::Config>();
    config0->disableAllStream();
    config0->enableStream(depth_profile0);
    config0->enableStream(color_profile0);
    config0->setAlignMode(ALIGN_DISABLE);

    pipe0->start(config0, [&](std::shared_ptr<ob::FrameSet> fs)
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
                    double x = -point[i*w+j].x/1000.0;
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
            // get color image
            std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();

            // convert opencv image
            cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

            cv::Mat img;
            cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);
            cv::flip(img, img, -1);

            if(!img.empty())
            {
                mtx.lock();
                cur_img0 = img.clone();
                mtx.unlock();
            }
        }
    });

    // set cam1
    auto dev1 = dev_list->getDevice(cam_idx1);
    dev1->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, false);
    dev1->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, true);

    std::shared_ptr<ob::Pipeline> pipe1 = std::make_shared<ob::Pipeline>(dev1);
    Eigen::Matrix4d TF1 = string_to_TF(config->CAM_TF_1);

    auto depth_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_DEPTH);
    auto color_profile_list1 = pipe1->getStreamProfileList(OB_SENSOR_COLOR);

    auto depth_profile1 = depth_profile_list1->getProfile(27)->as<ob::VideoStreamProfile>();
    auto color_profile1 = color_profile_list1->getProfile(123)->as<ob::VideoStreamProfile>();

    std::shared_ptr<ob::Config> config1 = std::make_shared<ob::Config>();
    config1->disableAllStream();
    config1->enableStream(depth_profile1);
    config1->enableStream(color_profile1);
    config1->setAlignMode(ALIGN_DISABLE);

    pipe1->start(config1, [&](std::shared_ptr<ob::FrameSet> fs)
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
                    double x = -point[i*w+j].x/1000.0;
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
            // get color image
            std::shared_ptr<ob::ColorFrame> colorFrame = fs->colorFrame();

            // convert opencv image
            cv::Mat raw(colorFrame->height(), colorFrame->width(), CV_8UC3, colorFrame->data());

            cv::Mat img;
            cv::cvtColor(raw, img, cv::COLOR_RGB2BGR);
            //cv::flip(img, img, -1);

            if(!img.empty())
            {
                mtx.lock();
                cur_img1 = img.clone();
                mtx.unlock();
            }
        }
    });

    // set flag
    is_connected0 = true;
    is_connected1 = true;

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

#if defined(USE_AMR_400) || defined(USE_AMR_400_PROTO) || defined(USE_AMR_400_LAKI) || defined(USE_AMR_KAI)
void CAM::grab_loop()
{
    // set flag
    is_connected0 = false;
    is_connected1 = false;

    printf("[CAM] grab loop started\n");

    while(grab_flag)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("[CAM] grab loop stopped\n");
}
#endif
