#include "safety.h"


SAFETY* SAFETY::instance(QObject* parent)
{
    static SAFETY* inst = nullptr;
    if(!inst && parent)
    {
        inst = new SAFETY(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

SAFETY::SAFETY(QObject *parent) : QObject{parent},
    config(nullptr),
    logger(nullptr),
    mobile(nullptr),
    unimap(nullptr),
    obsmap(nullptr),
    loc(nullptr)
{

}

SAFETY::~SAFETY()
{
    safety_flag = false;
    if(safety_thread && safety_thread->joinable())
    {
        safety_thread->join();
    }
    safety_thread.reset();
}

void SAFETY::init()
{
    if(config)
    {
        safety_fields = config->get_monitoring_field();
        for(int i = 0; i < safety_fields.size(); i++)
        {
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].monitor_id: " + QString::number(safety_fields[i].monitor_id), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].min_x: " + QString::number(safety_fields[i].min_x), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].max_x: " + QString::number(safety_fields[i].max_x), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].min_y: " + QString::number(safety_fields[i].min_y), "Green");
            logger->write_log("[SAFETY] safety_fields[" + QString::number(i) + "].max_y: " + QString::number(safety_fields[i].max_y), "Green");
        }
        logger->write_log("[SAFETY] safety_fields size: " + QString::number(safety_fields.size()), "Green");

    }
    else
    {
        //To do: log error message
    }
}

void SAFETY::start()
{
    safety_flag = true;
    safety_thread = std::make_unique<std::thread>(&SAFETY::safety_loop, this);
}



void SAFETY::stop()
{
    safety_flag = false;
    if(safety_thread && safety_thread->joinable())
    {
        safety_thread->join();
    }
    safety_thread.reset();
}


void SAFETY::set_obsmap_module(OBSMAP* _obsmap)
{
    obsmap = _obsmap;
}

void SAFETY::set_localization_module(LOCALIZATION* _loc)
{
    loc = _loc;
}

void SAFETY::set_config_module(CONFIG* _config)
{
    config = _config;
}

void SAFETY::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void SAFETY::set_mobile_module(MOBILE* _mobile)
{
    mobile = _mobile;
}

void SAFETY::set_unimap_module(UNIMAP* _unimap)
{
    unimap = _unimap;
}


void SAFETY::safety_loop()
{
    while(safety_flag)
    {


         
        // todo
    }
}