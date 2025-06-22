#include "unimap.h"

UNIMAP* UNIMAP::instance(QObject* parent)
{
    static UNIMAP* inst = nullptr;
    if(!inst && parent)
    {
        inst = new UNIMAP(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

UNIMAP::UNIMAP(QObject *parent) : QObject(parent),
    config(nullptr),
    logger(nullptr)
{
    nodes = std::make_shared<std::vector<NODE>>();

    // init related 2d map params
    kdtree_cloud    = std::make_shared<XYZR_CLOUD>();
    kdtree_mask     = std::make_shared<std::vector<int>>();

    // init related 3d map params
    tree_3d         = std::make_unique<KD_TREE>(3, cloud_3d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    map_3d_pts      = std::make_shared<std::vector<Eigen::Vector3d>>();
    map_3d_normal   = std::make_shared<std::vector<Eigen::Vector3d>>();
    map_3d_reflects = std::make_shared<std::vector<double>>();
}

UNIMAP::~UNIMAP()
{
    clear();
}

void UNIMAP::clear()
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    is_loaded.store(MAP_NOT_LOADED);

    // 2d cloud
    if(kdtree_mask)
    {
        kdtree_mask->clear();
    }
    if(kdtree_cloud)
    {
        kdtree_cloud->pts.clear();
    }

    kdtree_node.pos.clear();
    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    if(nodes)
    {
        nodes->clear();
    }
    nodes_id_map.clear();

    map_min_x =  std::numeric_limits<double>::max();
    map_max_x = -std::numeric_limits<double>::max();
    map_min_y =  std::numeric_limits<double>::max();
    map_max_y = -std::numeric_limits<double>::max();
    map_min_z =  std::numeric_limits<double>::max();
    map_max_z = -std::numeric_limits<double>::max();

    // 3d
    if(tree_3d)
    {
        tree_3d.reset();
    }

    map_path = "";
}

void UNIMAP::load_map(QString path)
{
    // clear flag
    is_loaded.store(MAP_LOADING);

    // set load map dir
    map_path = path;

    bool loaded_2d = load_2d();
    bool loaded_3d = load_3d();
    load_topo();

    QString loc_mode = config->get_loc_mode();
    if(loc_mode == "2D" && loaded_2d)
    {
        is_loaded.store(MAP_LOADED);
    }
    else if(loc_mode == "3D" && loaded_3d)
    {
        is_loaded.store(MAP_LOADED);
    }
    else
    {
        is_loaded.store(MAP_NOT_LOADED);
    }
}

bool UNIMAP::load_2d()
{
    QString path = map_path;

    // load cloud map
    {
        QString cloud_csv_path = path + "/cloud.csv";
        QFileInfo cloud_csv_info(cloud_csv_path);
        if(cloud_csv_info.exists() && cloud_csv_info.isFile())
        {
            // clear first
            {
                std::unique_lock<std::shared_mutex> lock(mtx);
                if(kdtree_mask)
                {
                    kdtree_mask->clear();
                }
                if(kdtree_cloud)
                {
                    kdtree_cloud->pts.clear();
                }
            }

            // clear additional cloud
            additional_cloud.clear();

            // csv file format : x,y,z,r
            std::vector<PT_XYZR> pts;

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

                    pts.push_back(pt);
                }
                cloud_csv_file.close();

                {
                    std::unique_lock<std::shared_mutex> lock(mtx);
                    if(kdtree_cloud)
                    {
                        kdtree_cloud->pts = voxel_filtering(pts, 0.05);
                    }
                    if(kdtree_mask)
                    {
                        kdtree_mask->resize(kdtree_cloud->pts.size(), 1);
                    }
                }

                printf("[UNIMAP(2D)] %s loaded, map_pts:%d\n", cloud_csv_path.toLocal8Bit().data(), (int)kdtree_cloud->pts.size());
            }
        }
        else
        {
            printf("[UNIMAP(2D)] load failed\n");
            return false;
        }
    }

    // set KDTree
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if(kdtree_cloud)
        {
            kdtree_index = std::make_shared<KD_TREE_XYZR>(3, *kdtree_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
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

            printf("[UNIMAP(2D)] build kdtree for map icp\n");
            printf("[UNIMAP(2D)] boundary x:(%.2f, %.2f), y:(%.2f, %.2f), z:(%.2f, %.2f)\n",
                   map_min_x, map_max_x, map_min_y, map_max_y, map_min_z, map_max_z);
        }
    }

    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    kdtree_node_index = std::make_unique<KD_TREE_NODE>(3, kdtree_node, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree_node_index->buildIndex();

    return true;
}

bool UNIMAP::load_3d()
{
    if(map_path.isEmpty())
    {
        printf("[UNIMAP(3D)] map_dir is empty!\n");
        return false;
    }

    QString file_path = map_path + "/map.las";

    if(!QFile::exists(file_path))
    {
        return false;
    }

    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createStage("readers.las");
    if(reader == nullptr)
    {
        printf("[UNIMAP(3D)] map read failed\n");
        return false;
    }

    pdal::Options options;
    options.add("filename", file_path.toStdString());
    options.add("extra_dims", "NormalX=double,NormalY=double,NormalZ=double");
    reader->setOptions(options);

    pdal::PointTable point_table;
    reader->prepare(point_table);

    pdal::PointViewSet point_views = reader->execute(point_table);
    pdal::PointViewPtr point_view = *point_views.begin();

    uint64_t point_size = point_view->size();

    std::vector<Eigen::Vector3d> pts; pts.resize(point_size);
    std::vector<Eigen::Vector3d> nor; nor.resize(point_size);
    std::vector<double> reflects; reflects.resize(point_size);
    std::vector<cv::Vec3b> colors; colors.resize(point_size);
    for(pdal::PointId id = 0; id < point_view->size(); id++)
    {
        double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

        double nx = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalX, id);
        double ny = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalY, id);
        double nz = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalZ, id);

        double intensity = point_view->getFieldAs<double>(pdal::Dimension::Id::Intensity, id);

        uint8_t red = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Red, id);
        uint8_t green = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Green, id);
        uint8_t blue = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Blue, id);

        pts[id]      = Eigen::Vector3d(x, y, z);
        nor[id]      = Eigen::Vector3d(nx, ny, nz);
        reflects[id] = intensity;
        colors[id]   = cv::Vec3b(blue, green, red);
    }

    // update
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        cloud_3d.pts = pts;
        if(tree_3d)
        {
            tree_3d->buildIndex();
        }

        // set plot storage
        map_3d_pts      = std::make_shared<std::vector<Eigen::Vector3d>>(pts);
        map_3d_normal   = std::make_shared<std::vector<Eigen::Vector3d>>(nor);
        map_3d_reflects = std::make_shared<std::vector<double>>(reflects);
    }

    std::cout << "[UNIMAP(3D)] Loaded " << pts.size() << " points from " << file_path.toStdString() << std::endl;
    return true;
}

bool UNIMAP::load_topo()
{
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if(nodes)
        {
            nodes->clear();
        }
    }
    
    QString topo_path = map_path + "/topo.json";
    QFileInfo topo_info(topo_path);
    if(!topo_info.exists() || !topo_info.isFile())
    {
        return false;
    }

    QFile topo_file(topo_path);
    if(topo_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = topo_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);

        QJsonArray arr = doc.array();
        std::vector<NODE> temp_nodes;
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
            temp_nodes.push_back(node);
        }
        topo_file.close();
        
        {
            std::unique_lock<std::shared_mutex> lock(mtx);
            if(nodes)
            {
                *nodes = temp_nodes;
            }
            
            std::vector<Eigen::Vector3d> node_pos; node_pos.resize(nodes->size());
            std::vector<QString> node_id; node_id.resize(nodes->size());
            for(size_t p=0; p<nodes->size(); p++)
            {
                NODE node = (*nodes)[p];
                if(node.type == "ROUTE" || node.type == "GOAL" || node.type == "INIT" || node.type == "STATION")
                {
                    node_pos[p] = node.tf.block(0,3,3,1);
                    node_id[p] = node.id;
                }

                NODE* ptr = &(*nodes)[p];
                nodes_id_map[ptr->id] = ptr;
                nodes_name_map[ptr->name] = ptr;
            }

            kdtree_node.pos = node_pos;
            kdtree_node.id = node_id;
        }
        
        logger->write_log(QString("[UNIMAP] %1 loaded").arg(topo_path), "Green", true, false);
    }
    return true;
}

void UNIMAP::save_annotation()
{
    // get save folder
    QString path = map_path;

    // save topo.json
    QString topo_path = path + "/topo.json";
    QFile topo_file(topo_path);
    if(topo_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        QJsonArray arr;
        for(size_t p = 0; p < nodes->size(); p++)
        {
            QJsonObject obj;
            obj["id"]    = (*nodes)[p].id;
            obj["name"]  = (*nodes)[p].name;
            obj["type"]  = (*nodes)[p].type;
            obj["info"]  = (*nodes)[p].info;
            obj["pose"]  = TF_to_string((*nodes)[p].tf);
            obj["links"] = links_to_array((*nodes)[p].linked);
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
    if(is_loaded != MAP_LOADED)
    {
        return;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!kdtree_mask || kdtree_mask->size() == 0 || !kdtree_index)
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
        if(idx < (int)kdtree_mask->size())
        {
            (*kdtree_mask)[idx] = val;
        }
    }
}

void UNIMAP::clear_nodes()
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    if(nodes)
    {
        nodes->clear();
    }
    nodes_id_map.clear();
    nodes_name_map.clear();

    printf("[UNIMAP] topology cleared\n");
}

std::vector<QString> UNIMAP::get_linked_nodes(QString id)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!nodes)
    {
        return std::vector<QString>();
    }

    std::set<QString> visited;
    std::vector<QString> result;

    std::function<void(const QString&)> dfs = [&](const QString& node_id)
    {
        auto it = std::find_if(nodes->begin(), nodes->end(), [&](const NODE& node)
        {
            return node.id == node_id;
        });

        if (it == nodes->end())
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
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!nodes)
    {
        return std::vector<QString>();
    }

    std::vector<QString> res;
    for(const auto& it: (*nodes))
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
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!nodes)
    {
        return std::vector<QString>();
    }

    std::vector<QString> res;
    res.reserve(nodes->size());
    for(const auto& it: (*nodes))
    {
        res.push_back(it.id);
    }
    return res;
}

std::shared_ptr<std::vector<NODE>> UNIMAP::get_nodes_origin()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return nodes;
}

double UNIMAP::get_map_min_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_min_x;
}

double UNIMAP::get_map_max_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_max_x;
}

double UNIMAP::get_map_min_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_min_y;
}

double UNIMAP::get_map_max_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_max_y;
}

std::vector<int> UNIMAP::get_init_candidates(std::vector<QString> prefix_list)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    std::vector<int> res;
    if(!nodes)
    {
        return res;
    }
    
    for(size_t p = 0; p < nodes->size(); p++)
    {
        if((*nodes)[p].type == "INIT")
        {
            res.push_back(p);
            continue;
        }

        for(size_t q = 0; q < prefix_list.size(); q++)
        {
            if((*nodes)[p].name.contains(prefix_list[q]))
            {
                res.push_back(p);
                break;
            }
        }
    }
    return res;
}

int UNIMAP::get_nodes_size()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(!nodes)
    {
        return 0;
    }
    int res = (int)nodes->size();
    return res;
}

QString UNIMAP::get_map_path()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    QString res = map_path;
    return res;
}

bool UNIMAP::get_is_loaded()
{
    return (bool)is_loaded.load();
}

void UNIMAP::set_map_path(QString path)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    map_path = path;
}

std::shared_ptr<std::vector<int>> UNIMAP::get_kdtree_mask()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return kdtree_mask;
}

std::shared_ptr<XYZR_CLOUD> UNIMAP::get_kdtree_cloud()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return kdtree_cloud;
}

std::shared_ptr<std::vector<Eigen::Vector3d>> UNIMAP::get_map_3d_pts()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_3d_pts;
}

std::shared_ptr<std::vector<Eigen::Vector3d>> UNIMAP::get_map_3d_normal()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_3d_normal;
}

Eigen::Vector3d UNIMAP::get_normal_3d(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(!map_3d_normal)
    {
        return Eigen::Vector3d(0,0,0);
    }

    if(idx < 0 || idx > (int)map_3d_normal->size())
    {
        return Eigen::Vector3d(0,0,0);
    }

    Eigen::Vector3d res = (*map_3d_normal)[idx];
    return res;
}

Eigen::Vector3d UNIMAP::get_pts_3d(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(!map_3d_pts)
    {
        return Eigen::Vector3d(0,0,0);
    }

    if(idx < 0 || idx > (int)map_3d_pts->size())
    {
        return Eigen::Vector3d(0,0,0);
    }

    Eigen::Vector3d res = (*map_3d_pts)[idx];
    return res;
}

std::shared_ptr<std::vector<double>> UNIMAP::get_map_3d_reflects()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_3d_reflects;
}

std::shared_ptr<KD_TREE_XYZR> UNIMAP::get_kdtree_index()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return kdtree_index;
}

void UNIMAP::radius_search_kdtree_idx(double query[], double sq_radius, std::vector<nanoflann::ResultItem<unsigned int, double>>& res_idxs, nanoflann::SearchParameters params)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!kdtree_index)
    {
        return;
    }
    
    kdtree_index->radiusSearch(&query[0], sq_radius, res_idxs, params);
}

QString UNIMAP::get_node_id_edge(Eigen::Vector3d pos)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!nodes || nodes->size() == 0 || !kdtree_node_index)
    {
        return "";
    }

    const int cnt_num = 10;
    std::vector<unsigned int> ret_near_idxs(cnt_num);
    std::vector<double> ret_near_sq_dists(cnt_num);

    double near_query_pt[3] = {pos[0], pos[1], pos[2]};
    kdtree_node_index->knnSearch(&near_query_pt[0], cnt_num, &ret_near_idxs[0], &ret_near_sq_dists[0]);

    Eigen::Vector3d _P0(0,0,0);
    Eigen::Vector3d _P1(0,0,0);
    QString _node_id0 = "";
    QString _node_id1 = "";
    double min_d = std::numeric_limits<double>::max();

    bool is_found = false;
    for(int p = 0; p < cnt_num; p++)
    {
        if(ret_near_idxs[p] >= kdtree_node.id.size())
        {
            continue;
        }
        
        QString id0 = kdtree_node.id[ret_near_idxs[p]];

        NODE* node0 = get_node_by_id(id0);
        if(node0 == nullptr)
        {
            continue;
        }

        if(ret_near_idxs[p] >= kdtree_node.pos.size())
        {
            continue;
        }
        
        Eigen::Vector3d pos0 = kdtree_node.pos[ret_near_idxs[p]];

        for(size_t q=0; q<node0->linked.size(); q++)
        {
            QString id1 = node0->linked[q];

            NODE* node1 = get_node_by_id(id1);
            if(node1 == nullptr)
            {
                continue;
            }

            if(node1->type == "ROUTE" || node1->type == "GOAL" || node1->type == "INIT" || node1->type == "STATION")
            {
                Eigen::Matrix4d tf1 = node1->tf;
                Eigen::Vector3d pos1 = tf1.block(0,3,3,1);

                double d = calc_seg_dist(pos0, pos1, pos);
                if(d < min_d)
                {
                    min_d = d;
                    _P0 = pos0;
                    _P1 = pos1;
                    _node_id0 = id0;
                    _node_id1 = id1;
                    is_found = true;
                }
            }
        }
    }

    if(is_found && min_d < 3.0)
    {
        double d0 = calc_dist_2d(_P0-pos);
        double d1 = calc_dist_2d(_P1-pos);
        if(d0 < d1)
        {
            return _node_id0;
        }
        else
        {
            return _node_id1;
        }
    }
    else
    {
        if(ret_near_idxs[0] < kdtree_node.pos.size())
        {
            Eigen::Vector3d pos0 = kdtree_node.pos[ret_near_idxs[0]];
            if((pos-pos0).norm() < 10.0)
            {
                if(ret_near_idxs[0] < kdtree_node.id.size())
                {
                    return kdtree_node.id[ret_near_idxs[0]];
                }
            }
        }
        return "";
    }

    return "";
}

QString UNIMAP::get_goal_id(Eigen::Vector3d pos)
{
    return "";
    /*if(nodes->size() == 0)
    {
        return "";
    }

    // find node
    int min_idx = -1;
    double min_d = std::numeric_limits<double>::max();
    for(size_t p = 0; p < nodes->size(); p++)
    {
        if(nodes[p].type == "GOAL" || nodes[p].type == "INIT" || nodes[p].type == "STATION")
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
    return "";*/
}

NODE* UNIMAP::get_node_by_id(QString id)
{
    if(id == "")
    {
        return nullptr;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    NODE *node = nullptr;

    auto it = nodes_id_map.find(id);
    if(it != nodes_id_map.end())
    {
        node = it->second;
    }

    return node;
}

NODE* UNIMAP::get_node_by_name(QString name)
{
    if(name == "")
    {
        return nullptr;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    NODE *node = nullptr;

    auto it = nodes_name_map.find(name);
    if(it != nodes_name_map.end())
    {
        node = it->second;
    }

    return node;
}

QString UNIMAP::get_cur_zone(Eigen::Matrix4d tf)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    QString zone_type = "";

    if(!nodes)
    {
        return zone_type;
    }

    std::vector<QString> zones = get_nodes("ZONE");
    for(size_t p=0; p<zones.size(); p++)
    {
        NODE* node = get_node_by_id(zones[p]);
        if(node == nullptr)
        {
            continue;
        }

        QString info = node->info;
        NODE_INFO res;
        if(parse_info(info, "SIZE", res))
        {
            Eigen::Matrix4d res_tf = node->tf.inverse()*tf;

            double x = res_tf(0,3);
            double y = res_tf(1,3);
            double z = res_tf(2,3);

            if(x > -res.sz[0]/2 && x < res.sz[0]/ 2 &&
               y > -res.sz[1]/2 && y < res.sz[1]/ 2 &&
               z > -res.sz[2]/2 && z < res.sz[2]/ 2)
            {
                zone_type = node->type;
                break;
            }
        }
    }

    return zone_type;
}

std::vector<int> UNIMAP::knn_search_idx(Eigen::Vector3d center, int k, double radius)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!tree_3d)
    {
        return std::vector<int>();
    }
    
    const double sq_radius = radius*radius;

    std::vector<unsigned int> res_idxs(k);
    std::vector<double> res_sq_dists(k);
    double query_pt[3] = {center[0], center[1], center[2]};

    tree_3d->knnSearch(&query_pt[0], k, &res_idxs[0], &res_sq_dists[0]);

    std::vector<int> res;
    for(size_t p = 0; p < res_idxs.size(); p++)
    {
        if(res_idxs[p] < cloud_3d.pts.size())
        {
            double dx = cloud_3d.pts[res_idxs[p]][0] - cloud_3d.pts[res_idxs[0]][0];
            double dy = cloud_3d.pts[res_idxs[p]][1] - cloud_3d.pts[res_idxs[0]][1];
            double dz = cloud_3d.pts[res_idxs[p]][2] - cloud_3d.pts[res_idxs[0]][2];
            double sq_d = dx*dx + dy*dy + dz*dz;

            if(sq_d < sq_radius)
            {
                res.push_back(res_idxs[p]);
            }
        }
    }
    return res;
}

void UNIMAP::set_config_module(CONFIG* _config)
{
    config = _config;
}

void UNIMAP::set_logger_module(LOGGER* _logger)
{
    logger = _logger;
}
