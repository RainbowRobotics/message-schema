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

}

void CAM::close()
{
    is_connected = false;

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
    is_connected.store(false);

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
    while(post_process_flag)
    {
        if(config->get_cam_type() == "ORBBEC" && orbbec)
        {
            TIME_PTS tp;
            if(orbbec->try_pop_depth_que(idx, tp))
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_scan[idx] = tp;
            }

            TIME_IMG ti;
            if(orbbec->try_pop_img_que(idx, ti))
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_time_img[idx] = ti;
            }
        }
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
