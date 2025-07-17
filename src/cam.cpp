#include "cam.h"

CAM* CAM::instance(QObject* parent)
{
    static CAM* inst = nullptr;
    if(!inst && parent)
    {
        inst = new CAM(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

CAM::CAM(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    orbbec(nullptr)
{
    // init
    for(int p = 0; p < max_cam_cnt; p++)
    {
        is_connected[p] = false;
        post_process_flag[p] = false;
        process_time_post[p] = 0.0;
    }
}

void CAM::close()
{
    int cam_num = config->get_cam_num();
    for(int p = 0; p < cam_num; p++)
    {
        is_connected[p] = false;
    }

    if(config->get_cam_type() == "ORBBEC" && orbbec)
    {
        int cam_num = config->get_cam_num();
        for(int p = 0; p < cam_num; p++)
        {
            post_process_flag[p] = false;
            if(post_process_thread[p] && post_process_thread[p]->joinable())
            {
                post_process_thread[p]->join();
            }
            post_process_thread[p].reset();
        }
    }
}

CAM::~CAM()
{
    int cam_num = config->get_cam_num();
    for(int p = 0; p < cam_num; p++)
    {
        is_connected[p] = false;
    }

    // close cam
    if(orbbec != nullptr)
    {
        orbbec->close();
    }

    close();
}

void CAM::init()
{
    // check simulation mode
    if(config->get_use_sim())
    {
        printf("[CAM] simulation mode\n");
        return;
    }

    if(config->get_cam_type() == "ORBBEC")
    {
        if(!orbbec)
        {
            orbbec = ORBBEC::instance(this);
            orbbec->set_config_module(this->config);
            orbbec->set_logger_module(this->logger);
            orbbec->set_mobile_module(this->mobile);
            orbbec->init();
            orbbec->open();
        }
    }
}

void CAM::open()
{
    if(config->get_cam_type() == "ORBBEC" && orbbec)
    {
        int cam_num = config->get_cam_num();
        for(int p = 0; p < cam_num; p++)
        {
            post_process_flag[p] = true;
            post_process_thread[p] = std::make_unique<std::thread>(&CAM::post_process_loop, this, p);
        }
    }

    if(config->get_use_rtsp())
    {
        rtsp_flag = true;
        rtsp_thread = std::make_unique<std::thread>(&CAM::rtsp_loop, this);
    }
}

double CAM::get_process_time_post(int idx)
{
    return (double)process_time_post[idx].load();
}

TIME_IMG CAM::get_time_img(int idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    return cur_time_img[idx];
}

TIME_PTS CAM::get_scan(int idx)
{
    std::lock_guard<std::mutex> lock(mtx);
    return cur_scan[idx];
}

void CAM::post_process_loop(int idx)
{
    const double dt = 0.025; // 40hz
    double pre_loop_time = get_time();

    while(post_process_flag)
    {
        if(config->get_cam_type() == "ORBBEC" && orbbec)
        {
            TIME_PTS tp;
            if(orbbec->try_pop_depth_que(idx, tp))
            {
                if(is_connected[idx] == false)
                {
                    is_connected[idx] = true;
                }

                std::lock_guard<std::mutex> lock(mtx);
                cur_scan[idx] = tp;
            }

            TIME_IMG ti;
            if(orbbec->try_pop_img_que(idx, ti))
            {
                if(is_connected[idx] == false)
                {
                    is_connected[idx] = true;
                }

                std::lock_guard<std::mutex> lock(mtx);
                cur_time_img[idx] = ti;
            }
        }

        // for real time loop
        double cur_loop_time = get_time();
        double delta_loop_time = cur_loop_time - pre_loop_time;
        if(delta_loop_time < dt)
        {
            int sleep_ms = (dt-delta_loop_time)*1000;
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
        }
        else
        {
            logger->write_log(QString("[AUTO] loop time drift, dt:%1").arg(delta_loop_time));
        }
        process_time_post[idx] = delta_loop_time;
        pre_loop_time = get_time();
    }
}

void CAM::rtsp_loop()
{
    const int send_w = 320;
    const int send_h = 160;

    cv::VideoWriter writer[2];
    std::string pipeline[2];
    pipeline[0] = "appsrc ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=600 key-int-max=30 bframes=0 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://localhost:8554/cam0 protocols=udp latency=0";
    pipeline[1] = "appsrc ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=600 key-int-max=30 bframes=0 ! video/x-h264,profile=baseline ! rtspclientsink location=rtsp://localhost:8554/cam1 protocols=udp latency=0";

    for(int p=0; p<config->get_cam_num(); p++)
    {
        bool is_try_open = false;
        if(!writer[p].isOpened())
        {
            is_try_open = writer[p].open(pipeline[p], 0, (double)10, cv::Size(send_w, send_h), true);
        }

        if(!is_try_open)
        {
            logger->write_log(QString("[RTSP] cam%1 rtsp writer open failed").arg(p));
            rtsp_flag = false;
            return;
        }
    }

    while(rtsp_flag)
    {
        for(int p=0; p<config->get_cam_num(); p++)
        {
            if(!is_connected[p])
            {
                continue;
            }

            if(!writer[p].isOpened())
            {
                continue;
            }

            TIME_IMG timg = get_time_img(p);
            cv::Mat img = timg.img.clone();
            if(!img.empty())
            {
                if(img.cols != send_w || img.rows != send_h)
                {
                    cv::Mat _img;
                    cv::resize(img, _img, cv::Size(send_w, send_h));
                    writer[p].write(_img);
                }
                else
                {
                    writer[p].write(img);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void CAM::set_config_module(CONFIG* _config)
{
    config = _config;
}

void CAM::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void CAM::set_mobile_module(MOBILE *_mobile)
{
    mobile = _mobile;
}
