#include "policy.h"

POLICY* POLICY::instance(QObject* parent)
{
    static POLICY* inst = nullptr;
    if(!inst && parent)
    {
        inst = new POLICY(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

POLICY::POLICY(QObject *parent) : QObject{parent}
{

}

POLICY::~POLICY()
{
    close();
}

void POLICY::init()
{
    printf("[POLICY] init\n");
}

void POLICY::open()
{
    node_flag = true;
    node_thread = std::make_unique<std::thread>(&POLICY::node_loop, this);

    link_flag = true;
    link_thread = std::make_unique<std::thread>(&POLICY::link_loop, this);

    zone_flag = true;
    zone_thread = std::make_unique<std::thread>(&POLICY::zone_loop, this);

    printf("[POLICY] open\n");
}

void POLICY::close()
{
    node_flag = false;
    if(node_thread && node_thread->joinable())
    {
        node_thread->join();
    }
    node_thread.reset();

    link_flag = false;
    if(link_thread && link_thread->joinable())
    {
        link_thread->join();
    }
    link_thread.reset();

    zone_flag = false;
    if(zone_thread && zone_thread->joinable())
    {
        zone_thread->join();
    }
    zone_thread.reset();
    printf("[POLICY] close\n");
}

QString POLICY::get_cur_node()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_node;
    return res;
}

QString POLICY::get_cur_link()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_link;
    return res;
}

QString POLICY::get_cur_info()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_info;
    return res;
}


void POLICY::set_cur_info(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_info = str;
}

QString POLICY::get_cur_zone()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_zone;
    return res;
}

void POLICY::set_cur_zone(QString str)
{
    std::lock_guard<std::mutex> lock(mtx);
    cur_zone = str;
}

void POLICY::set_config_module(CONFIG* _config)
{
    config = _config;
}

void POLICY::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}

void POLICY::set_unimap_module(UNIMAP *_unimap)
{
    unimap = _unimap;
}

void POLICY::set_localization_module(LOCALIZATION *_loc)
{
    loc = _loc;
}

void POLICY::node_loop()
{
    QString pre_node_id = "";

    printf("[POLICY] node_loop start\n");
    while(node_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        QString _cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
        if(pre_node_id == "")
        {
            pre_node_id = _cur_node_id;

            // update
            std::lock_guard<std::mutex> lock(mtx);
            cur_node = pre_node_id;
        }
        else
        {
            // calc pre node id
            NODE *node = unimap->get_node_by_id(_cur_node_id);
            if(node != nullptr)
            {
                double d = calc_dist_2d(node->tf.block(0,3,3,1) - cur_tf.block(0,3,3,1));
                if(d < config->get_robot_radius())
                {
                    if(pre_node_id != _cur_node_id)
                    {
                        pre_node_id = _cur_node_id;

                        // update
                        std::lock_guard<std::mutex> lock(mtx);
                        cur_node = pre_node_id;
                    }
                }
            }
        }

        // printf("[NODE] node=%s\n", _cur_node_id.toStdString().c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] node_loop stop\n");
}

void POLICY::link_loop()
{
    printf("[POLICY] link_loop start\n");
    while(link_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }



        // printf("[NODE] node=%s\n", _cur_node_id.toStdString().c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] link_loop stop\n");
}


void POLICY::zone_loop()
{
    bool is_ready = false;

    printf("[POLICY] zone_loop start\n");
    while(zone_flag)
    {
        if(!is_ready)
        {
            if(unimap->get_is_loaded() == MAP_LOADED)
            {
                is_ready = true;
                if(unimap->get_nodes("ZONE").empty())
                {
                    break;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // plot text
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        pcl::PointXYZ position;
        position.x = cur_tf(0,3);
        position.y = cur_tf(1,3);
        position.z = cur_tf(2,3) + 1.5;

        // check zone
        QString zone = "";
        QString info = "";

        std::vector<QString> zones = unimap->get_nodes("ZONE");
        for(size_t p = 0; p < zones.size(); p++)
        {
            NODE* node = unimap->get_node_by_id(zones[p]);
            if(node != NULL)
            {
                QString name = node->name;
                // QString info = node->info;

                NODE_INFO res;
                if(parse_info(node->info, "", res))
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

                        QStringList parts;

                        // speed
                        if(res.speed == "SLOW")      parts << "SPEED_SLOW";
                        else if(res.speed == "FAST") parts << "SPEED_FAST";

                        // flags
                        if(res.warning_beep)   parts << "WARNING_BEEP";
                        if(res.ignore_2d)      parts << "IGNORE_2D";
                        if(res.ignore_3d)      parts << "IGNORE_3D";
                        if(res.ignore_cam)     parts << "IGNORE_CAM";
                        if(res.ignore_obs_2d)  parts << "IGNORE_OBS_2D";
                        if(res.ignore_obs_3d)  parts << "IGNORE_OBS_3D";
                        if(res.ignore_obs_cam) parts << "IGNORE_OBS_CAM";

                        info = parts.join(", ");

                        // printf("[ZONE] active zone=%s, info=%s\n",
                        //        zone.toStdString().c_str(),
                        //        info.toStdString().c_str());

                        break;
                    }
                }
            }
        }

        // update
        set_cur_zone(zone);
        set_cur_info(info);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] zone_loop stop\n");
}
