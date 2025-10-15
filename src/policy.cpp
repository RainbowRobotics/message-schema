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

    policy_flag = true;
    policy_thread = std::make_unique<std::thread>(&POLICY::policy_loop, this);

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

    policy_flag = false;
    if(policy_thread && policy_thread->joinable())
    {
        policy_thread->join();
    }
    policy_thread.reset();

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

QString POLICY::get_cur_zone()
{
    std::lock_guard<std::mutex> lock(mtx);
    QString res = cur_zone;
    return res;
}

NODE_INFO POLICY::get_node_info()
{
    std::lock_guard<std::mutex> lock(mtx);
    NODE_INFO res = node_info;
    return res;
}

NODE_INFO POLICY::get_link_info()
{
    std::lock_guard<std::mutex> lock(mtx);
    NODE_INFO res = link_info;
    return res;
}

NODE_INFO POLICY::get_zone_info()
{
    std::lock_guard<std::mutex> lock(mtx);
    NODE_INFO res = zone_info;
    return res;
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

void POLICY::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    ctrl = _ctrl;
}

void POLICY::apply_reverse(bool on)
{
    // prevent duplication
    static bool prev = false;
    if(on == prev)
    {
        return;
    }



    prev = on;

    printf("[POLICY] REVERSE %s\n", on ? "ON" : "OFF");
}

void POLICY::node_loop()
{
    printf("[POLICY] node_loop start\n");

    while(node_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector2d pos(cur_tf(0,3), cur_tf(1,3));

        NODE_INFO info;
        QString cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
        if(!cur_node_id.isEmpty())
        {
            NODE* node = unimap->get_node_by_id(cur_node_id);
            if(node != nullptr)
            {
                double d = calc_dist_2d(node->tf.block(0,3,3,1) - cur_tf.block(0,3,3,1));
                if(d <= config->get_robot_radius())
                {
                    parse_info(node->info, "", info);
                }
                else
                {
                    cur_node_id.clear();
                }
            }
        }

        // update
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_node = cur_node_id;
            node_info = info;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("[POLICY] node_loop stop\n");
}

void POLICY::link_loop()
{
    QString last_link_str = "";

    printf("[POLICY] link_loop start\n");
    while(link_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector2d pos(cur_tf(0,3), cur_tf(1,3));

        QString cur_node_id = get_cur_node();
        std::vector<QString> global_path = ctrl->get_cur_global_path().node;

        QString cur_link_str = "";
        NODE_INFO cur_link_info;
        bool info_updated = false;

        if(global_path.size() < 2)
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link.clear();
                link_info = NODE_INFO();
            }
            last_link_str.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(cur_node_id.isEmpty())
        {
            if(!last_link_str.isEmpty())
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link = last_link_str;
                link_info = NODE_INFO();
            }
            else
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link.clear();
                link_info = NODE_INFO();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // find the idx of the cur_node in path
        int idx = -1;
        for(size_t i = 0; i < global_path.size(); i++)
        {
            if(global_path[i] == cur_node_id)
            {
                idx = i;
                break;
            }
        }

        // if there is no cur_node on the path, the link is terminated
        if(idx < 0)
        {
            // update
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link.clear();
                link_info = NODE_INFO();
            }
            last_link_str.clear();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        if(idx + 1 < (int)global_path.size())
        {
            // if there is a next node -> activate link (st=cur_node, ed=next_node)
            QString st_id = global_path[idx];
            QString ed_id = global_path[idx + 1];

            cur_link_str = st_id + "->" + ed_id;

            if(cur_link_str != last_link_str)
            {
                std::vector<LINK_INFO> link = unimap->get_special_links();
                for(const auto& lk : link)
                {
                    if(lk.st_id == st_id && lk.ed_id == ed_id)
                    {
                        NODE_INFO res;
                        if(parse_info(lk.info, "", res))
                        {
                            cur_link_info = res;
                            info_updated  = true;
                        }
                        break;
                    }
                }
                last_link_str = cur_link_str;
            }

            // update
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link = cur_link_str;
                if(info_updated)
                {
                    link_info = cur_link_info;
                }
            }
        }
        else
        {
            // if there is no next node -> end of link
            {
                std::lock_guard<std::mutex> lock(mtx);
                cur_link = cur_link_str;
                link_info = cur_link_info;
            }
            last_link_str.clear();
        }

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
        NODE_INFO info;

        std::vector<QString> zones = unimap->get_nodes("ZONE");
        for(size_t p = 0; p < zones.size(); p++)
        {
            NODE* node = unimap->get_node_by_id(zones[p]);
            if(node != NULL)
            {
                QString name = node->name;

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
                        info = res;
                        break;
                    }
                }
            }
        }

        // update
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_zone = zone;
            zone_info = info;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] zone_loop stop\n");
}

void POLICY::policy_loop()
{
    QString last_info = "";

    printf("[POLICY] policy_loop start\n");

    while(policy_flag)
    {
        NODE_INFO z = get_zone_info();
        NODE_INFO l = get_link_info();
        NODE_INFO n = get_node_info();

        bool slow            = (z.slow           || l.slow           || n.slow);
        bool fast            = (z.fast           || l.fast           || n.fast);
        bool reverse         = (z.reverse        || l.reverse        || n.reverse);
        bool warning_beep    = (z.warning_beep   || l.warning_beep   || n.warning_beep);
        bool ignore_2d       = (z.ignore_2d      || l.ignore_2d      || n.ignore_2d);
        bool ignore_3d       = (z.ignore_3d      || l.ignore_3d      || n.ignore_3d);
        bool ignore_cam      = (z.ignore_cam     || l.ignore_cam     || n.ignore_cam);
        bool ignore_obs_2d   = (z.ignore_obs_2d  || l.ignore_obs_2d  || n.ignore_obs_2d);
        bool ignore_obs_3d   = (z.ignore_obs_3d  || l.ignore_obs_3d  || n.ignore_obs_3d);
        bool ignore_obs_cam  = (z.ignore_obs_cam || l.ignore_obs_cam || n.ignore_obs_cam);

        if(slow && fast)
        {
            fast = false;
        }

        QString info = QString("S%1 F%2 R%3 W%4 I2%5 I3%6 IC%7 O2%8 O3%9 OC%10")
                .arg(slow ? "1" : "0")
                .arg(fast ? "1" : "0")
                .arg(reverse ? "1" : "0")
                .arg(warning_beep ? "1" : "0")
                .arg(ignore_2d    ? "1" : "0")
                .arg(ignore_3d    ? "1" : "0")
                .arg(ignore_cam   ? "1" : "0")
                .arg(ignore_obs_2d? "1" : "0")
                .arg(ignore_obs_3d? "1" : "0")
                .arg(ignore_obs_cam? "1" : "0");

        if(info != last_info)
        {
            // apply_slow(slow);
            // apply_fast(fast);
            apply_reverse(reverse);
            // apply_warning_beep(warning_beep);
            // apply_ignore_2d(ignore_2d);
            // apply_ignore_3d(ignore_3d);
            // apply_ignore_cam(ignore_cam);
            // apply_ignore_odbs_2d(ignore_obs_2d);
            // apply_ignore_obs_3d(ignore_obs_3d);
            // apply_ignore_obs_cam(ignore_obs_cam);

            last_info = info;
            printf("[POLICY] applied flags: %s\n", info.toStdString().c_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] policy_loop stop\n");
}
