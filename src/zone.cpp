#include "zone.h"

ZONE* ZONE::instance(QObject* parent)
{
    static ZONE* inst = nullptr;
    if(!inst && parent)
    {
        inst = new ZONE(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

ZONE::ZONE(QObject *parent) : QObject{parent}
{

}

ZONE::~ZONE()
{
    close();
}

void ZONE::init()
{
    printf("[ZONE] init\n");
}

void ZONE::open()
{
    zone_flag = true;
    zone_thread = std::make_unique<std::thread>(&ZONE::zone_loop, this);

    printf("[ZONE] open\n");
}

void ZONE::close()
{
    zone_flag = false;
    if(zone_thread && zone_thread->joinable())
    {
        zone_thread->join();
    }
    zone_thread.reset();
    printf("[ZONE] close\n");
}

QString ZONE::get_cur_zone()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_zone;
    return res;
}

void ZONE::set_cur_zone(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_zone = str;
}

void ZONE::set_config_module(CONFIG* _config)
{
    config = _config;
}

void ZONE::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void ZONE::set_unimap_module(UNIMAP *_unimap)
{
    unimap = _unimap;
}

void ZONE::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}

void ZONE::zone_loop()
{
    printf("[ZONE] zone_loop start\n");
    while(zone_flag)
    {
        // plot text
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        pcl::PointXYZ position;
        position.x = cur_tf(0,3);
        position.y = cur_tf(1,3);
        position.z = cur_tf(2,3) + 1.5;

        // check zone
        QString zone = "";
        std::vector<QString> zones = unimap->get_nodes("ZONE");
        for(size_t p = 0; p < zones.size(); p++)
        {
            NODE* node = unimap->get_node_by_id(zones[p]);
            if(node != NULL)
            {
                QString name = node->name;
                QString info = node->info;

                NODE_INFO res;
                if(parse_info(info, "SIZE", res))
                {
                    Eigen::Matrix4d tf = node->tf.inverse()*cur_tf;

                    double x = tf(0,3);
                    double y = tf(1,3);
                    double z = tf(2,3);

                    if(x > -res.sz[0]/2 && x < res.sz[0]/2 &&
                            y > -res.sz[1]/2 && y < res.sz[1]/2 &&
                            z > -res.sz[2]/2 && z < res.sz[2]/2)
                    {
                        // current zone
                        zone = name;
                        break;
                    }
                }
            }
        }

        // update
        set_cur_zone(zone);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
