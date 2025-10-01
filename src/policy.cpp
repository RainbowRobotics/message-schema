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

void POLICY::apply_slow(bool on)
{
    // prevent duplication
    static bool prev = false;
    if(on == prev)
    {
        return;
    }



    prev = on;

    printf("[POLICY] SLOW %s\n", on ? "ON" : "OFF");
}

void POLICY::apply_fast(bool on)
{
    // prevent duplication
    static bool prev = false;
    if(on == prev)
    {
        return;
    }

    prev = on;

    printf("[POLICY] FAST %s\n", on ? "ON" : "OFF");
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
        QString cur_node_id = unimap->get_node_id_edge(cur_tf.block(0,3,3,1));
        NODE *node = unimap->get_node_by_id(cur_node_id);

        QString inside_node = "";
        NODE_INFO inside_info;

        if(node != nullptr)
        {
            double d = calc_dist_2d(node->tf.block(0,3,3,1) - cur_tf.block(0,3,3,1));
            if(d < config->get_robot_radius())
            {
                inside_node = cur_node_id;
                NODE_INFO res;
                if(parse_info(node->info, "", res))
                {
                    inside_info = res;
                }

            }
        }

        // update
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_node = inside_node;
            node_info = inside_info;
        }

        // printf("[NODE] node=%s\n", _cur_node_id.toStdString().c_str());

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] node_loop stop\n");
}

void POLICY::link_loop()
{
    printf("[POLICY] link_loop start\n");

    QString last_undir = "";
    QString last_oriented = "";
    double  last_t = 0.0;
    bool    has_last_t = false;

    int outside_cnt = 0;

    while(link_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        QString cur_node_id = get_cur_node();
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector2d pos(cur_tf(0,3), cur_tf(1,3));

        QString inside_link = "";
        NODE_INFO inside_info;

        double half_width = 0.40;

        int inside_idx = -1;
        double best_across = 1e9;

        std::vector<LINK_INFO> links = unimap->get_special_links();
        for(size_t i = 0; i < links.size(); i++)
        {
            LINK_INFO& link = links[i];

            double L = link.length;
            if(L <= 1e-9)
            {
                continue;
            }

            double ux = link.ed.x() - link.st.x();
            double uy = link.ed.y() - link.st.y();

            double vx = pos.x() - link.mid.x();
            double vy = pos.y() - link.mid.y();

            double along  = std::abs((vx*ux + vy*uy) / L);
            double across = std::abs((vx*uy - vy*ux) / L);

            if(along <= 0.5*L && across <= half_width)
            {
                if(across < best_across)
                {
                    best_across = across;
                    inside_idx = (int)i;
                }
            }
        }

        if(inside_idx >= 0)
        {
            LINK_INFO& link = links[(size_t)inside_idx];

            QString cur_undir = (link.st_id < link.ed_id) ? (link.st_id + "|" + link.ed_id) : (link.ed_id + "|" + link.st_id);

            if(last_undir != cur_undir)
            {
                last_undir = cur_undir;
                has_last_t = false;
                last_oriented.clear();
            }

            Eigen::Vector2d st2(link.st.x(), link.st.y());
            Eigen::Vector2d u(link.ed.x() - link.st.x(), link.ed.y() - link.st.y());
            double L = u.norm();
            double t = 0.0;
            if(L > 1e-12)
            {
                Eigen::Vector2d w = pos - st2;
                t = w.dot(u) / L;
            }

            if(has_last_t)
            {
                if(t > last_t +  0.02)
                {
                    last_oriented = link.st_id + "->" + link.ed_id;
                }
                else if(t < last_t -  0.02)
                {
                    last_oriented = link.ed_id + "->" + link.st_id;
                }

                if(last_oriented.isEmpty())
                {
                    double ds = (pos - st2).squaredNorm();
                    double de = (pos - Eigen::Vector2d(link.ed.x(), link.ed.y())).squaredNorm();
                    last_oriented = (ds <= de) ? (link.st_id + "->" + link.ed_id) : (link.ed_id + "->" + link.st_id);
                }
            }
            else
            {
                double ds = (pos - st2).squaredNorm();
                double de = (pos - Eigen::Vector2d(link.ed.x(), link.ed.y())).squaredNorm();
                last_oriented = (ds <= de) ? (link.st_id + "->" + link.ed_id) : (link.ed_id + "->" + link.st_id);
                has_last_t = true;
            }

            last_t = t;
            outside_cnt = 0;

            inside_link = last_oriented;

            NODE_INFO res;
            if(parse_info(link.info, "", res))
            {
                inside_info = res;
            }
        }
        else
        {
            outside_cnt++;
            if(outside_cnt >= 3)
            {
                last_undir.clear();
                last_oriented.clear();
                has_last_t = false;

                inside_link.clear();
                inside_info = NODE_INFO();
            }
            else
            {
                inside_link = last_oriented;
                inside_info = NODE_INFO();
            }
        }

        // update
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_link = inside_link;
            link_info = inside_info;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    printf("[POLICY] link_loop stop\n");
}

/*
void POLICY::link_loop()
{
    QString pre_node_id = "";

    printf("[POLICY] link_loop start\n");
    while(link_flag)
    {
        if(unimap->get_is_loaded() != MAP_LOADED)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        QString cur_node_id = get_cur_node();
        Eigen::Matrix4d cur_tf = loc->get_cur_tf();
        Eigen::Vector2d pos = Eigen::Vector2d(cur_tf(0,3), cur_tf(1,3));

        if(!cur_node_id.isEmpty())
        {
            pre_node_id = cur_node_id;
        }

        QString inside_link = "";
        NODE_INFO inside_info;

        double half_width = 0.40;

        bool is_link = false;

        std::vector<LINK_INFO> links = unimap->get_special_links();
        for(size_t i = 0; i < links.size(); i++)
        {
            LINK_INFO& link = links[i];

            double L = link.length;
            if(L <= 1e-9)
            {
                continue;
            }

            double ux = link.ed.x() - link.st.x();
            double uy = link.ed.y() - link.st.y();

            double vx = pos.x() - link.mid.x();
            double vy = pos.y() - link.mid.y();

            double along  = std::abs((vx*ux + vy*uy) / L);
            double across = std::abs((vx*uy - vy*ux) / L);
            if(along <= 0.5*L && across <= half_width)
            {
                is_link = true;

                if(pre_node_id == link.st_id)
                {
                    inside_link = link.st_id + "->" + link.ed_id;
                }
                else if(pre_node_id == link.ed_id)
                {
                    inside_link = link.ed_id + "->" + link.st_id;
                }
                else
                {
                    inside_link = link.st_id + "->" + link.ed_id;
                }

                NODE_INFO res;
                if(parse_info(link.info, "", res))
                {
                    inside_info = res;
                }
                break;
            }
        }

        if(!is_link && !cur_node_id.isEmpty())
        {
            pre_node_id = cur_node_id;
        }
        printf("[LINK] pre_node_id=%s\n", pre_node_id.toStdString().c_str());

        // update
        {
            std::lock_guard<std::mutex> lock(mtx);
            cur_link = inside_link;
            link_info = inside_info;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[POLICY] link_loop stop\n");
}
*/

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

        QString info = QString("S%1 F%2 W%3 I2%4 I3%5 IC%6 O2%7 O3%8 OC%9")
                .arg(slow ? "1" : "0")
                .arg(fast ? "1" : "0")
                .arg(warning_beep ? "1" : "0")
                .arg(ignore_2d    ? "1" : "0")
                .arg(ignore_3d    ? "1" : "0")
                .arg(ignore_cam   ? "1" : "0")
                .arg(ignore_obs_2d? "1" : "0")
                .arg(ignore_obs_3d? "1" : "0")
                .arg(ignore_obs_cam? "1" : "0");

        if(info != last_info)
        {
            apply_slow(slow);
            apply_fast(fast);
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
