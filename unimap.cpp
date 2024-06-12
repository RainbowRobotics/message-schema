#include "unimap.h"

UNIMAP::UNIMAP(QObject *parent) : QObject(parent)
{
    is_loaded = false;
}

UNIMAP::~UNIMAP()
{

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

            map_min_x = 99999999;
            map_max_x = -99999999;
            map_min_y = 99999999;
            map_max_y = -99999999;
            map_min_z = 99999999;
            map_max_z = -99999999;

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

                    // for bounding box
                    if(x < map_min_x)
                    {
                        map_min_x = x;
                    }

                    if(x > map_max_x)
                    {
                        map_max_x = x;
                    }

                    if(y < map_min_y)
                    {
                        map_min_y = y;
                    }

                    if(y > map_max_y)
                    {
                        map_max_y = y;
                    }

                    if(z < map_min_z)
                    {
                        map_min_z = z;
                    }

                    if(z > map_max_z)
                    {
                        map_max_z = z;
                    }
                }
                cloud_csv_file.close();

                printf("[UNIMAP] %s loaded, map_pts:%d\n", cloud_csv_path.toLocal8Bit().data(), (int)kdtree_cloud.pts.size());
                printf("[UNIMAP] boundary x:(%.2f, %.2f), y:(%.2f, %.2f), z:(%.2f, %.2f)\n", map_min_x, map_max_x,
                       map_min_y, map_max_y, map_min_z, map_max_z);
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

        printf("[UNIMAP] build kd tree for map icp\n");
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
                    node.tf = string_to_TF(obj["pose"].toString());
                    node.linked = array_to_links(obj["links"].toArray());
                    node.info = obj["info"].toString();

                    nodes.push_back(node);
                }
                topo_file.close();

                QString str;
                str.sprintf("[UNIMAP] %s loaded", topo_path.toStdString().c_str());
                logger->PrintLog(str, "Green", true, false);
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
            obj["pose"] = TF_to_string(nodes[p].tf);
            obj["links"] = links_to_array(nodes[p].linked);
            obj["info"] = nodes[p].info;

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

bool UNIMAP::is_los(Eigen::Vector3d P0, Eigen::Vector3d P1, double radius)
{
    if(is_loaded == false)
    {
        return true;
    }

    double sq_radius = radius*radius;
    double length = (P1-P0).norm();
    Eigen::Vector3d dir = (P1-P0).normalized();

    for(double l = 0; l < length; l += radius)
    {
        Eigen::Vector3d P = P0 + l*dir;

        // radius search
        double query_pt[3] = {P[0], P[1], P[2]};

        std::vector<nanoflann::ResultItem<unsigned int, double>> res_idxs;
        nanoflann::SearchParameters params;
        kdtree_index->radiusSearch(&query_pt[0], sq_radius, res_idxs, params);
        if(res_idxs.size() > 0)
        {
            return false;
        }
    }
    return true;
}

void UNIMAP::add_node(PICKING pick, QString type, QString info)
{
    if(pick.cur_node == "")
    {
        NODE node;
        node.id = gen_node_id();
        node.type = type;
        node.tf = ZYX_to_TF(pick.r_pose[0], pick.r_pose[1], 0, 0, 0, pick.r_pose[2]);
        node.info = info;
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

QString UNIMAP::add_node(Eigen::Matrix4d tf, QString type, QString info)
{
    NODE node;
    node.id = gen_node_id();
    node.type = type;
    node.tf = tf;
    node.info = info;
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

void UNIMAP::save_node_pos(PICKING pick)
{
    if(pick.cur_node == "" || pick.r_pose == Eigen::Vector3d(0, 0, 0))
    {
        printf("cur_node or picking tf empty\n");
        return;
    }

    NODE* node = get_node_by_id(pick.cur_node);
    if(node != NULL)
    {
        double x = pick.r_pose[0];
        double y = pick.r_pose[1];
        double th = pick.r_pose[2];

        node->tf = ZYX_to_TF(x, y, 0, 0, 0, th);
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

void UNIMAP::clear_nodes()
{
    mtx.lock();
    nodes.clear();
    mtx.unlock();

    printf("[UNIMAP] topology cleared\n");
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

QString UNIMAP::gen_node_id()
{
    QString res;
    if(nodes.size() == 0)
    {
        res.sprintf("N_%04d", 0);
    }
    else
    {
        int max_num = 0;
        for(size_t p = 0; p < nodes.size(); p++)
        {
            QStringList list = nodes[p].id.split("_");
            int num = list[1].toInt();
            if(num > max_num)
            {
                max_num = num;
            }
        }
        res.sprintf("N_%04d", max_num+1);
    }
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
        if(nodes[p].type == "ROUTE" || nodes[p].type == "GOAL")
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

QString UNIMAP::get_node_id_edge_nn(Eigen::Vector3d pos)
{
    double min_d = 99999999;
    NODE* min_node0 = NULL;
    NODE* min_node1 = NULL;
    Eigen::Vector3d min_pos0;
    Eigen::Vector3d min_pos1;
    for(auto& it: nodes)
    {
        if(it.type == "ROUTE" || it.type == "GOAL")
        {
            for(size_t p = 0; p < it.linked.size(); p++)
            {
                QString id0 = it.id;
                QString id1 = it.linked[p];

                NODE* node0 = get_node_by_id(id0);
                NODE* node1 = get_node_by_id(id1);

                Eigen::Vector3d P0 = node0->tf.block(0,3,3,1);
                Eigen::Vector3d P1 = node1->tf.block(0,3,3,1);

                double d = calc_seg_dist(P0, P1, pos);
                if(d < min_d)
                {
                    min_d = d;
                    min_node0 = node0;
                    min_node1 = node1;
                    min_pos0 = P0;
                    min_pos1 = P1;
                }
            }
        }
    }

    double d0 = (min_pos0 - pos).norm();
    double d1 = (min_pos1 - pos).norm();

    if(d0 < d1)
    {
        return min_node0->id;
    }
    else
    {
        return min_node1->id;
    }
}

QString UNIMAP::get_node_id_los(Eigen::Vector3d pos)
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
        if(nodes[p].type == "ROUTE" || nodes[p].type == "GOAL")
        {
            double d = (nodes[p].tf.block(0,3,3,1) - pos).norm();
            if(d < min_d)
            {
                if(is_los(nodes[p].tf.block(0,3,3,1), pos, config->ROBOT_SIZE_Y[1]))
                {
                    min_d = d;
                    min_idx = p;
                }
            }
        }
    }

    if(min_idx != -1)
    {
        return nodes[min_idx].id;
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
    if(nodes.size() != 0)
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

NODE* UNIMAP::get_node_nn(Eigen::Vector3d pos)
{
    if(nodes.size() == 0)
    {
        return NULL;
    }

    // find node
    int min_idx = -1;
    double min_d = 99999999;
    for(size_t p = 0; p < nodes.size(); p++)
    {
        if(nodes[p].type == "ROUTE" || nodes[p].type == "GOAL")
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
        return &nodes[min_idx];
    }

    return NULL;
}

NODE* UNIMAP::get_edge_nn(Eigen::Vector3d pos)
{
    double min_d = 99999999;
    NODE* min_node0 = NULL;
    NODE* min_node1 = NULL;
    Eigen::Vector3d min_pos0;
    Eigen::Vector3d min_pos1;
    for(auto& it: nodes)
    {
        if(it.type == "ROUTE" || it.type == "GOAL")
        {
            for(size_t p = 0; p < it.linked.size(); p++)
            {
                QString id0 = it.id;
                QString id1 = it.linked[p];

                NODE* node0 = get_node_by_id(id0);
                NODE* node1 = get_node_by_id(id1);

                Eigen::Vector3d P0 = node0->tf.block(0,3,3,1);
                Eigen::Vector3d P1 = node1->tf.block(0,3,3,1);

                double d = calc_seg_dist(P0, P1, pos);
                if(d < min_d)
                {
                    min_d = d;
                    min_node0 = node0;
                    min_node1 = node1;
                    min_pos0 = P0;
                    min_pos1 = P1;
                }
            }
        }
    }

    double d0 = (min_pos0 - pos).norm();
    double d1 = (min_pos1 - pos).norm();

    if(d0 < d1)
    {
        return min_node0;
    }
    else
    {
        return min_node1;
    }
}

QJsonArray UNIMAP::pose_to_array(Eigen::Vector3d pose)
{
    QJsonArray res;
    res.append(pose[0]);
    res.append(pose[1]);
    res.append(pose[2]);
    return res;
}

Eigen::Vector3d UNIMAP::array_to_pose(QJsonArray arr)
{
    Eigen::Vector3d res;
    res[0] = arr[0].toDouble();
    res[1] = arr[1].toDouble();
    res[2] = arr[2].toDouble();
    return res;
}

QJsonArray UNIMAP::links_to_array(std::vector<QString> links)
{
    QJsonArray res;
    for(size_t p = 0; p < links.size(); p++)
    {
        res.append(links[p]);
    }
    return res;
}

std::vector<QString> UNIMAP::array_to_links(QJsonArray arr)
{
    std::vector<QString> res;
    for(int p = 0; p < arr.size(); p++)
    {
        res.push_back(arr[p].toString());
    }
    return res;
}


