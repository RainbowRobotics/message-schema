#include "unimap.h"

UNIMAP::UNIMAP(QObject *parent) : QObject(parent)
{    

}

UNIMAP::~UNIMAP()
{

}

void UNIMAP::clear()
{
    is_loaded = false;

    kdtree_mask.clear();
    kdtree_cloud.pts.clear();

    if(kdtree_index != NULL)
    {
        delete kdtree_index;
        kdtree_index = NULL;
    }

    nodes.clear();

    map_min_x = 99999999;
    map_max_x = -99999999;
    map_min_y = 99999999;
    map_max_y = -99999999;
    map_min_z = 99999999;
    map_max_z = -99999999;

    map_dir = "";
}

void UNIMAP::load_map(QString path)
{
    // clear flag
    is_loaded = false;

    // set load map dir
    map_dir = path;

    // load cloud map
    {
        QString cloud_csv_path = path + "/cloud.csv";
        QFileInfo cloud_csv_info(cloud_csv_path);
        if(cloud_csv_info.exists() && cloud_csv_info.isFile())
        {
            // clear first
            kdtree_mask.clear();
            kdtree_cloud.pts.clear();

            // csv file format : x,y,z,r
            QFile cloud_csv_file(cloud_csv_path);
            if(cloud_csv_file.open(QIODevice::ReadOnly))
            {
                QTextStream in(&cloud_csv_file);
                while(!in.atEnd())
                {
                    QString line = in.readLine();
                    QStringList str = line.split(",");

                    double x = str[0].toDouble();
                    double y = str[1].toDouble();
                    double z = str[2].toDouble();
                    double r = str[3].toDouble();

                    PT_XYZR pt;
                    pt.x = x;
                    pt.y = y;
                    pt.z = z;
                    pt.r = r;

                    kdtree_cloud.pts.push_back(pt);
                    kdtree_mask.push_back(1);
                }
                cloud_csv_file.close();

                printf("[UNIMAP] %s loaded, map_pts:%d\n", cloud_csv_path.toLocal8Bit().data(), (int)kdtree_cloud.pts.size());                
            }
        }
        else
        {
            printf("[UNIMAP] load failed\n");
            return;
        }
    }

    // set KDTree
    {
        if(kdtree_index != NULL)
        {
            delete kdtree_index;
            kdtree_index = NULL;
        }

        kdtree_index = new KD_TREE_XYZR(3, kdtree_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree_index->buildIndex();

        // set boundary
        KD_TREE_XYZR::BoundingBox bbox;
        kdtree_index->computeBoundingBox(bbox);

        map_min_x = bbox[0].low;
        map_max_x = bbox[0].high;
        map_min_y = bbox[1].low;
        map_max_y = bbox[1].high;
        map_min_z = bbox[2].low;
        map_max_z = bbox[2].high;

        // boundary loosing
        double gap_x = map_max_x - map_min_x;
        if(std::abs(gap_x) < 1.0)
        {
            map_min_x -= 0.5;
            map_max_x += 0.5;
        }

        double gap_y = map_max_y - map_min_y;
        if(std::abs(gap_y) < 1.0)
        {
            map_min_y -= 0.5;
            map_max_y += 0.5;
        }

        double gap_z = map_max_z - map_min_z;
        if(std::abs(gap_z) < 1.0)
        {
            map_min_z -= 0.5;
            map_max_z += 0.5;
        }

        printf("[UNIMAP] build kdtree for map icp\n");
        printf("[UNIMAP] boundary x:(%.2f, %.2f), y:(%.2f, %.2f), z:(%.2f, %.2f)\n",
               map_min_x, map_max_x, map_min_y, map_max_y, map_min_z, map_max_z);
    }

    // load topology file
    {
        nodes.clear();

        QString topo_path = map_dir + "/topo.json";
        QFileInfo topo_info(topo_path);
        if(topo_info.exists() && topo_info.isFile())
        {
            QFile topo_file(topo_path);
            if(topo_file.open(QIODevice::ReadOnly))
            {
                QByteArray data = topo_file.readAll();
                QJsonDocument doc = QJsonDocument::fromJson(data);

                QJsonArray arr = doc.array();
                Q_FOREACH(const QJsonValue &val, arr)
                {
                    QJsonObject obj = val.toObject();

                    NODE node;
                    node.id = obj["id"].toString();
                    node.name = obj["name"].toString();
                    node.type = obj["type"].toString();
                    node.info = obj["info"].toString();
                    node.tf = string_to_TF(obj["pose"].toString());
                    node.linked = array_to_links(obj["links"].toArray());                    
                    nodes.push_back(node);
                }
                topo_file.close();

                QString str;
                str.sprintf("[UNIMAP] %s loaded", topo_path.toStdString().c_str());
                logger->write_log(str, "Green", true, false);
            }
        }
    }

    // set flag
    is_loaded = true;
}

void UNIMAP::save_map()
{
    // get save folder
    QString path = map_dir;

    // save cloud csv
    QString cloud_csv_path = path + "/cloud.csv";
    QFile cloud_csv_file(cloud_csv_path);
    if(cloud_csv_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        for(size_t p = 0; p < kdtree_cloud.pts.size(); p++)
        {
            if(kdtree_mask[p] == 0)
            {
                continue;
            }

            PT_XYZR pt = kdtree_cloud.pts[p];

            double x = pt.x;
            double y = pt.y;
            double z = pt.z;
            double r = pt.r;

            QString str;
            str.sprintf("%f,%f,%f,%f\n", x, y, z, r);
            cloud_csv_file.write(str.toUtf8());
        }

        cloud_csv_file.close();
        printf("[UNIMAP] %s saved\n", cloud_csv_path.toLocal8Bit().data());
    }
}

void UNIMAP::save_annotation()
{
    // get save folder
    QString path = map_dir;

    // save topo.json
    QString topo_path = path + "/topo.json";
    QFile topo_file(topo_path);
    if(topo_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        QJsonArray arr;
        for(size_t p = 0; p < nodes.size(); p++)
        {
            QJsonObject obj;
            obj["id"] = nodes[p].id;
            obj["name"] = nodes[p].name;
            obj["type"] = nodes[p].type;
            obj["info"] = nodes[p].info;
            obj["pose"] = TF_to_string(nodes[p].tf);
            obj["links"] = links_to_array(nodes[p].linked);            
            arr.append(obj);
        }

        QJsonDocument doc(arr);
        topo_file.write(doc.toJson());
        topo_file.close();

        printf("[UNIMAP] %s saved\n", topo_path.toLocal8Bit().data());
    }
}

void UNIMAP::set_cloud_mask(Eigen::Vector3d P, double radius, int val)
{
    if(is_loaded == false || kdtree_mask.size() == 0)
    {
        return;
    }

    double query_pt[3] = {P[0], P[1], P[2]};
    double sq_radius = radius*radius;

    std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
    nanoflann::SearchParameters params;
    kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);
    for(size_t p = 0; p < res_idxs.size(); p++)
    {
        int idx = res_idxs[p].first;
        kdtree_mask[idx] = val;
    }
}

void UNIMAP::add_node(PICKING pick, QString type, QString info)
{
    if(pick.cur_node == "")
    {
        QString name = gen_node_name();
        if(type == "ZONE")
        {
            name = QInputDialog::getText(nullptr, "INPUT", "name:");
        }

        NODE node;
        node.id = gen_node_id();
        node.name = name;
        node.type = type;
        node.info = info;
        node.tf = ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]);        
        nodes.push_back(node);

        printf("[UNIMAP] add node, %s\n", node.id.toLocal8Bit().data());
    }
    else
    {
        NODE* node = get_node_by_id(pick.cur_node);
        if(node != NULL)
        {
            auto it = std::find(nodes.begin(), nodes.end(), *node);
            if(it != nodes.end())
            {
                // disconnect other link
                QString id = it->id;
                for(size_t p = 0; p < nodes.size(); p++)
                {
                    for(size_t q = 0; q < nodes[p].linked.size(); q++)
                    {
                        auto _it = std::find(nodes[p].linked.begin(), nodes[p].linked.end(), id);
                        if(_it != nodes[p].linked.end())
                        {
                            nodes[p].linked.erase(_it);
                        }
                    }
                }

                // erase node
                nodes.erase(it);                
            }
        }
    }
}

QString UNIMAP::add_node(Eigen::Matrix4d tf, QString type)
{
    // for quick annotation
    NODE node;
    node.id = gen_node_id();
    node.name = gen_node_name();
    node.type = type;
    node.info = "";
    node.tf = tf;
    nodes.push_back(node);

    printf("[UNIMAP] add node, %s\n", node.id.toLocal8Bit().data());

    return node.id;
}

void UNIMAP::edit_node_pos(PICKING pick)
{
    if(pick.cur_node == "" || pick.r_pose == Eigen::Vector3d(0, 0, 0))
    {
        printf("cur_node or picking tf empty\n");
        return;
    }

    NODE* node = get_node_by_id(pick.cur_node);
    if(node != NULL)
    {
        node->tf = ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]);
    }
}

void UNIMAP::edit_node_pos(QString id, Eigen::Matrix4d tf)
{
    if(id == "" || tf.block(0,3,3,1) == Eigen::Vector3d(0, 0, 0))
    {
        printf("cur_node or picking tf empty\n");
        return;
    }

    NODE* node = get_node_by_id(id);
    if(node != NULL)
    {
        node->tf = tf;
    }
}

void UNIMAP::edit_node_type(PICKING pick, QString type)
{
    if(pick.cur_node == "")
    {
        printf("cur_node empty\n");
        return;
    }

    NODE* node = get_node_by_id(pick.cur_node);
    if(node != NULL)
    {
        node->type = type;
    }
}

void UNIMAP::edit_node_info(PICKING pick, QString info)
{
    if(pick.cur_node == "")
    {
        printf("cur_node empty\n");
        return;
    }

    NODE* node = get_node_by_id(pick.cur_node);
    if(node != NULL)
    {
        node->info = info;
    }
}

void UNIMAP::edit_node_name(PICKING pick)
{
    if(pick.cur_node == "")
    {
        printf("cur_node empty\n");
        return;
    }

    NODE* node = get_node_by_id(pick.cur_node);
    if(node != NULL)
    {
        QString name = node->name;
        if(name == "")
        {
            name = gen_node_name();
        }

        bool ok = false;
        QString str = QInputDialog::getText(NULL, "Input", "node name:", QLineEdit::Normal, name, &ok);
        if(ok && !str.isEmpty())
        {
            node->name = str;
        }
    }
}

void UNIMAP::add_link1(PICKING pick)
{
    if(pick.pre_node != "" && pick.cur_node != "" && pick.pre_node != pick.cur_node)
    {
        NODE* node = get_node_by_id(pick.pre_node);
        if(node != NULL)
        {
            auto it = std::find(node->linked.begin(), node->linked.end(), pick.cur_node);
            if(it == node->linked.end())
            {
                node->linked.push_back(pick.cur_node);
            }
            else
            {
                node->linked.erase(it);
            }
        }
    }
}

void UNIMAP::add_link2(PICKING pick)
{
    if(pick.pre_node != "" && pick.cur_node != "" && pick.pre_node != pick.cur_node)
    {
        NODE* node0 = get_node_by_id(pick.pre_node);
        NODE* node1 = get_node_by_id(pick.cur_node);
        if(node0 != NULL && node1 != NULL)
        {
            auto it0 = std::find(node0->linked.begin(), node0->linked.end(), pick.cur_node);
            auto it1 = std::find(node1->linked.begin(), node1->linked.end(), pick.pre_node);
            if(it0 == node0->linked.end() && it1 == node1->linked.end())
            {
                node0->linked.push_back(pick.cur_node);
                node1->linked.push_back(pick.pre_node);
            }
            else
            {
                if(it0 != node0->linked.end())
                {
                    node0->linked.erase(it0);
                }

                if(it1 != node1->linked.end())
                {
                    node1->linked.erase(it1);
                }
            }
        }
    }
}

void UNIMAP::add_link2(QString id0, QString id1)
{
    if(id0 != "" && id1 != "" && id0 != id1)
    {
        NODE* node0 = get_node_by_id(id0);
        NODE* node1 = get_node_by_id(id1);
        if(node0 != NULL && node1 != NULL)
        {
            auto it0 = std::find(node0->linked.begin(), node0->linked.end(), id0);
            auto it1 = std::find(node1->linked.begin(), node1->linked.end(), id1);
            if(it0 == node0->linked.end() && it1 == node1->linked.end())
            {
                node0->linked.push_back(id1);
                node1->linked.push_back(id0);

                printf("[UNIMAP] add link, %s -> %s\n", id0.toLocal8Bit().data(), id1.toLocal8Bit().data());
            }
        }
    }
}

void UNIMAP::add_link_auto()
{
    if(nodes.size() == 0)
    {
        return;
    }

    std::vector<QString> nodes_id;
    std::vector<QString> goals_id = get_nodes("GOAL");
    std::vector<QString> inits_id = get_nodes("INIT");
    std::vector<QString> routes_id = get_nodes("ROUTE");

    nodes_id.insert(nodes_id.end(), goals_id.begin(), goals_id.end());
    nodes_id.insert(nodes_id.end(), inits_id.begin(), inits_id.end());
    nodes_id.insert(nodes_id.end(), routes_id.begin(), routes_id.end());

    std::vector<NODE*> _nodes;
    for (size_t p = 0; p < nodes_id.size(); p++)
    {
        NODE* node = get_node_by_id(nodes_id[p]);
        if(node == nullptr)
        {
            continue;
        }

        _nodes.push_back(node);
    }

    if(_nodes.size() == 0)
    {
        return;
    }

    Graph g(_nodes.size());
    boost::property_map<Graph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, g);
    for (size_t p = 0; p < _nodes.size(); p++)
    {
        Eigen::Vector3d pt1 = _nodes[p]->tf.block(0,3,3,1);
        Point p1(pt1[0], pt1[1]);

        for (size_t q = p + 1; q < _nodes.size(); q++)
        {
            Eigen::Vector3d pt2 = _nodes[q]->tf.block(0,3,3,1);
            Point p2(pt2[0], pt2[1]);

            Edge e;
            bool inserted;
            std::tie(e, inserted) = boost::add_edge(p, q, g);
            weightmap[e] = std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
        }
    }

    std::vector<Vertex> p_(num_vertices(g));
    boost::prim_minimum_spanning_tree(g, &p_[0]);
    for (size_t i = 0; i != p_.size(); ++i)
    {
        bool is_alreay_linked = false;
        if (p_[i] != i)
        {
            for(size_t c = 0; c < _nodes[i]->linked.size(); c++)
            {
                if(_nodes[i]->linked[c] == _nodes[p_[i]]->id)
                {
                    is_alreay_linked = true;
                    break;
                }
            }
            if(is_alreay_linked == true)
            {
                continue;
            }

            add_link2(_nodes[i]->id, _nodes[p_[i]]->id);
        }
    }
}

void UNIMAP::clear_nodes()
{
    nodes.clear();

    printf("[UNIMAP] topology cleared\n");
}

std::vector<QString> UNIMAP::get_linked_nodes(QString id)
{
    std::set<QString> visited;
    std::vector<QString> result;

    std::function<void(const QString&)> dfs = [&](const QString& node_id)
    {
        auto it = std::find_if(nodes.begin(), nodes.end(), [&](const NODE& node) { return node.id == node_id; });
        if (it == nodes.end())
        {
            return;
        }

        visited.insert(node_id);
        result.push_back(node_id);

        const NODE& node = *it;
        for (const auto& linked_id : node.linked)
        {
            if (visited.find(linked_id) == visited.end())
            {
                dfs(linked_id);
            }
        }
    };

    dfs(id);

    return result;
}

std::vector<QString> UNIMAP::get_nodes(QString type)
{
    std::vector<QString> res;
    for(auto& it: nodes)
    {
        if(it.type == type)
        {
            res.push_back(it.id);
        }
    }
    return res;
}

std::vector<QString> UNIMAP::get_nodes()
{
    std::vector<QString> res;
    for(auto& it: nodes)
    {
        res.push_back(it.id);
    }
    return res;
}

QString UNIMAP::gen_node_id()
{
    QString res;
    res.sprintf("N_%lld", (long long)(get_time0()*1000));
    return res;
}

QString UNIMAP::gen_node_name()
{
    int max = 0;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        QString str = nodes[p].name;
        QStringList str_list = str.split("_");
        if(str_list.size() != 2)
        {
            continue;
        }

        int n = str_list[1].toInt();
        if(n > max)
        {
            max = n;
        }
    }

    QString res;
    res.sprintf("LOC_%d", max + 1);
    return res;
}

QString UNIMAP::get_node_id(Eigen::Vector3d pos)
{
    if(nodes.size() == 0)
    {
        return "";
    }

    // find node
    int min_idx = -1;
    double min_d = 99999999;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        double d = (nodes[p].tf.block(0,3,3,1) - pos).norm();
        if(d < config->ROBOT_RADIUS*2 && d < min_d)
        {
            min_d = d;
            min_idx = p;
        }
    }

    if(min_idx != -1)
    {
        return nodes[min_idx].id;
    }
    return "";
}

QString UNIMAP::get_goal_id(Eigen::Vector3d pos)
{
    if(nodes.size() == 0)
    {
        return "";
    }

    // find node
    int min_idx = -1;
    double min_d = 99999999;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        if(nodes[p].type == "GOAL" || nodes[p].type == "INIT")
        {
            double d = (nodes[p].tf.block(0,3,3,1) - pos).norm();
            if(d < config->ROBOT_RADIUS*2 && d < min_d)
            {
                min_d = d;
                min_idx = p;
            }
        }
    }

    if(min_idx != -1)
    {
        return nodes[min_idx].id;
    }
    return "";
}

QString UNIMAP::get_node_id_nn(Eigen::Vector3d pos)
{
    if(nodes.size() == 0)
    {
        return "";
    }

    // find node
    int min_idx = -1;
    double min_d = 99999999;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        // check non drivable
        if(nodes[p].type == "ROUTE" || nodes[p].type == "GOAL" || nodes[p].type == "INIT")
        {
            double d = (nodes[p].tf.block(0,3,3,1) - pos).norm();
            if(d < min_d)
            {
                min_d = d;
                min_idx = p;
            }
        }
    }

    if(min_idx != -1)
    {
        return nodes[min_idx].id;
    }
    return "";
}

QString UNIMAP::get_node_id_edge(Eigen::Vector3d pos)
{
    if(nodes.size() == 0)
    {
        return "";
    }

    Eigen::Vector3d _P0(0,0,0);
    Eigen::Vector3d _P1(0,0,0);
    double min_d = 99999999;

    bool is_found = false;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        // check drivable
        if(nodes[p].type == "ROUTE" || nodes[p].type == "GOAL" || nodes[p].type == "INIT")
        {
            QString node_id0 = nodes[p].id;
            Eigen::Matrix4d tf0 = nodes[p].tf;
            Eigen::Vector3d pos0 = tf0.block(0,3,3,1);

            for(size_t q = 0; q < nodes[p].linked.size(); q++)
            {
                QString node_id1 = nodes[p].linked[q];

                NODE* node = get_node_by_id(node_id1);
                if(node == NULL)
                {
                    continue;
                }

                // check drivable
                if(node->type == "ROUTE" || node->type == "GOAL" || node->type == "INIT")
                {
                    Eigen::Matrix4d tf1 = node->tf;
                    Eigen::Vector3d pos1 = tf1.block(0,3,3,1);

                    double d = calc_seg_dist(pos0, pos1, pos);
                    if(d < min_d)
                    {
                        min_d = d;
                        _P0 = pos0;
                        _P1 = pos1;
                        is_found = true;
                    }
                }
            }
        }
    }

    if(is_found)
    {
        double d0 = (_P0-pos).norm();
        double d1 = (_P1-pos).norm();
        if(d0 < d1)
        {
            return get_node_id_nn(_P0);
        }
        else
        {
            return get_node_id_nn(_P1);
        }
    }

    return "";
}

NODE* UNIMAP::get_node_by_id(QString id)
{
    if(id == "")
    {
        return NULL;
    }

    NODE *node = NULL;
    if((int)nodes.size() != 0)
    {
        for(size_t p = 0; p < nodes.size(); p++)
        {
            if(nodes[p].id == id)
            {
                node = &nodes[p];
                break;
            }
        }
    }

    return node;
}

NODE* UNIMAP::get_node_by_name(QString name)
{
    if(name == "")
    {
        return NULL;
    }

    NODE *node = NULL;
    if((int)nodes.size() != 0)
    {
        for(size_t p = 0; p < nodes.size(); p++)
        {
            if(nodes[p].name == name)
            {
                node = &nodes[p];
                break;
            }
        }
    }

    return node;
}
