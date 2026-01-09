#include "lidar_bottom.h"
namespace
{
    const char* MODULE_NAME = "LIDAR_BOTTOM";
}

LIDAR_BOTTOM* LIDAR_BOTTOM::instance(QObject* parent)
{
    static LIDAR_BOTTOM* inst = nullptr;
    if(!inst && parent)
    {
        inst = new LIDAR_BOTTOM(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

LIDAR_BOTTOM::LIDAR_BOTTOM(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    coin_d4(nullptr)
{

}

LIDAR_BOTTOM::~LIDAR_BOTTOM()
{
    is_connected.store(false);

    // close lidar
    if(coin_d4 != nullptr)
    {
        coin_d4->close();
    }
}

void LIDAR_BOTTOM::init()
{
    if(config->get_use_sim())
    {
        log_info("simulation mode init");
        return;
    }

    COIN_D4::instance(this);

    coin_d4 = COIN_D4::instance();
    coin_d4->set_config_module(this->config);
    coin_d4->set_logger_module(this->logger);
    coin_d4->open();
}

void LIDAR_BOTTOM::open()
{
    if(config->get_use_sim())
    {
        log_info("simulation mode open");
        return;
    }

    if(deskewing_thread == nullptr)
    {
        deskewing_flag = true;
        deskewing_thread = make_unique<std::thread>(&LIDAR_BOTTOM::deskewing_thread, this);
    }
}

void LIDAR_BOTTOM::close()
{
    is_connected.store(false);

    deskewing_flag = false;
    if(deskewing_thread && deskewing_thread->joinable())
    {
        deskewing_thread->join();
    }
    deskewing_thread.reset();
    log_info("close");
}

bool LIDAR_BOTTOM::get_is_connected()
{
    return (bool)coin_d4->is_connected.load();
}

std::vector<Eigen::Vector3d> LIDAR_BOTTOM::get_cur_pts()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return coin_d4->get_cur_pts();
}

void LIDAR_BOTTOM::deskewing_loop()
{
    log_info("deskewing_loop start");
    while(deskewing_flag)
    {
        // todo

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    log_info("deskewing_loop stop");
}

void LIDAR_BOTTOM::set_config_module(CONFIG* _config)
{
    config = _config;
}

void LIDAR_BOTTOM::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void LIDAR_BOTTOM::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}


