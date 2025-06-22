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
    gemini_2e(nullptr)
{

}

void CAM::close()
{

}

CAM::~CAM()
{
    is_connected.store(false);

    // close cam
    if(gemini_2e != nullptr)
    {
        gemini_2e->close();
    }

    close();
}

void CAM::init()
{
    if(config->CAM_TYPE == "gemini_2e")
    {
        if(gemini_2e == nullptr)
        {
            gemini_2e = GEMINI2E::instance(this);
            gemini_2e->set_config_module(this->config);
            gemini_2e->set_logger_module(this->logger);
            gemini_2e->set_mobile_module(this->mobile);
            gemini_2e->init();
            gemini_2e->open();
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
