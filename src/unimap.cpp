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
    // init related 2d map params
    kdtree_cloud_2d       = std::make_shared<XYZR_CLOUD>();
    kdtree_cloud_2d_index = std::make_shared<KD_TREE_XYZR>(3, *kdtree_cloud_2d, nanoflann::KDTreeSingleIndexAdaptorParams(10));

    // init related 3d map params
    map_3d_pts            = std::make_shared<std::vector<Eigen::Vector3d>>();
    map_3d_normal         = std::make_shared<std::vector<Eigen::Vector3d>>();
    map_3d_reflects       = std::make_shared<std::vector<double>>();
}

UNIMAP::~UNIMAP()
{
    clear();
}

void UNIMAP::clear()
{
    is_loaded.store(MAP_NOT_LOADED);

    std::unique_lock<std::shared_mutex> lock(mtx);
    
    // clear 2d cloud
    {
        if(kdtree_cloud_2d_index)
        {
            kdtree_cloud_2d_index.reset();
        }

        if(kdtree_cloud_2d)
        {
            kdtree_cloud_2d->pts.clear();
        }
    }

    // clear 3d cloud
    {
        if(kdtree_cloud_3d_index)
        {
            kdtree_cloud_3d_index.reset();
        }

        kdtree_cloud_3d.pts.clear();
    }

    // clear fast find node kdtree
    kdtree_node.pos.clear();
    kdtree_node.id.clear();
    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    // clear nodes
    multi_index_nodes.clear();

    // clear bounding box
    map_min_x =  std::numeric_limits<double>::max();
    map_max_x = -std::numeric_limits<double>::max();
    map_min_y =  std::numeric_limits<double>::max();
    map_max_y = -std::numeric_limits<double>::max();
    map_min_z =  std::numeric_limits<double>::max();
    map_max_z = -std::numeric_limits<double>::max();

    // clear map path
    map_path = "";
}

void UNIMAP::load_map(QString path)
{
    // clear flag
    is_loaded.store(MAP_LOADING);

    // set load map dir
    map_path = path;

    bool loaded_2d = load_2d();
    auto future = std::async(std::launch::async, [this]()
    {
        return this->load_3d();
    });
    bool loaded_3d = future.get();

    load_node();

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
    QString path;
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        path = map_path;
    }

    // load cloud map
    {
        QString cloud_csv_path = path + "/cloud.csv";
        QFileInfo cloud_csv_info(cloud_csv_path);
        if(cloud_csv_info.exists() && cloud_csv_info.isFile())
        {
            // clear 2d cloud info
            {
                std::unique_lock<std::shared_mutex> lock(mtx);
                if(kdtree_cloud_2d)
                {
                    kdtree_cloud_2d->pts.clear();
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

                    if(str.size() != 4)
                    {
                        if(logger)
                        {
                            logger->write_log("[UNIAMP] critical error loading 2d map.", "Red");
                        }
                        return false;
                    }

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

                // set 2d cloud info
                {
                    std::unique_lock<std::shared_mutex> lock(mtx);
                    if(kdtree_cloud_2d)
                    {
                        kdtree_cloud_2d->pts = voxel_filtering(pts, 0.05);
                    }
                }

                printf("[UNIMAP(2D)] %s loaded, map_pts:%d\n", cloud_csv_path.toLocal8Bit().data(), (int)kdtree_cloud_2d->pts.size());
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
        if(kdtree_cloud_2d)
        {
            kdtree_cloud_2d_index = std::make_shared<KD_TREE_XYZR>(3, *kdtree_cloud_2d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kdtree_cloud_2d_index->buildIndex();

            // set boundary
            KD_TREE_XYZR::BoundingBox bbox;
            kdtree_cloud_2d_index->computeBoundingBox(bbox);

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

    return true;
}

bool UNIMAP::load_3d()
{
    QString path;
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        path = map_path;
    }

    if(path.isEmpty())
    {
        printf("[UNIMAP(3D)] map_dir is empty!\n");
        return false;
    }

    QString file_path = path + "/map.las";
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

    std::vector<Eigen::Vector3d> pts(point_size);
    std::vector<Eigen::Vector3d> nor(point_size);
    std::vector<double> reflects(point_size);
    std::vector<cv::Vec3b> colors(point_size);

    #pragma omp parallel for
    for(pdal::PointId id = 0; id < point_size; id++)
    {
        // get xyz
        double x = point_view->getFieldAs<double>(pdal::Dimension::Id::X, id);
        double y = point_view->getFieldAs<double>(pdal::Dimension::Id::Y, id);
        double z = point_view->getFieldAs<double>(pdal::Dimension::Id::Z, id);

        // get normal vector (xyz)
        double nx = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalX, id);
        double ny = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalY, id);
        double nz = point_view->getFieldAs<double>(pdal::Dimension::Id::NormalZ, id);

        // get intensity
        double intensity = point_view->getFieldAs<double>(pdal::Dimension::Id::Intensity, id);

        // get color
        uint8_t red = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Red, id);
        uint8_t green = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Green, id);
        uint8_t blue = point_view->getFieldAs<uint8_t>(pdal::Dimension::Id::Blue, id);

        pts[id]      = Eigen::Vector3d(x, y, z);
        nor[id]      = Eigen::Vector3d(nx, ny, nz);
        colors[id]   = cv::Vec3b(blue, green, red);
        reflects[id] = intensity;
    }

    // update
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        kdtree_cloud_3d.pts = pts;

        kdtree_cloud_3d_index = std::make_unique<KD_TREE>(3, kdtree_cloud_3d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree_cloud_3d_index->buildIndex();

        // set plot storage
        map_3d_pts      = std::make_shared<std::vector<Eigen::Vector3d>>(pts);
        map_3d_normal   = std::make_shared<std::vector<Eigen::Vector3d>>(nor);
        map_3d_reflects = std::make_shared<std::vector<double>>(reflects);
    }

    std::cout << "[UNIMAP(3D)] Loaded " << pts.size() << " points from " << file_path.toStdString() << std::endl;
    return true;
}

bool UNIMAP::load_node()
{
    std::unique_lock<std::shared_mutex> lock(mtx);

    // first clear node
    if(multi_index_nodes.size() != 0)
    {
        multi_index_nodes.clear();
    }

    QString _map_path = map_path;
    QString node_path = _map_path + "/topo.json";

    QFileInfo node_info(node_path);
    if(!node_info.exists() || !node_info.isFile())
    {
        return false;
    }

    QFile node_file(node_path);
    if(node_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = node_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);

        QJsonArray arr = doc.array();
        Q_FOREACH(const QJsonValue &val, arr)
        {
            QJsonObject obj = val.toObject();
            NODE node;
            node.id     = obj["id"].toString();
            node.name   = obj["name"].toString();
            node.type   = obj["type"].toString();
            node.info   = obj["info"].toString();
            node.tf     = string_to_TF(obj["pose"].toString());
            node.linked = array_to_links(obj["links"].toArray());

            multi_index_nodes.insert(node);
        }
        node_file.close();
        
        // build KD-tree for nodes
        {
            std::vector<Eigen::Vector3d> node_pos;
            std::vector<QString> node_id;

            auto& id_idx = multi_index_nodes.get<by_id>();
            for(const NODE& node : id_idx)
            {
                if(node.type == "ROUTE" || node.type == "GOAL" || node.type == "INIT" || node.type == "STATION")
                {
                    node_pos.push_back(node.tf.block(0,3,3,1));
                    node_id.push_back(node.id);
                }
            }

            // set kdtree info
            kdtree_node.pos = node_pos;
            kdtree_node.id = node_id;

            // build kdtree
            kdtree_node_index = std::make_unique<KD_TREE_NODE>(3, kdtree_node, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kdtree_node_index->buildIndex();
        }
        
        logger->write_log(QString("[UNIMAP] %1 loaded").arg(node_path), "Green", true, false);
    }
    return true;
}

void UNIMAP::save_node()
{
    // get save folder
    QString path;
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        path = map_path;
    }

    // save topo.json
    QString node_path = path + "/topo.json";
    QFile node_file(node_path);
    if(node_file.open(QIODevice::WriteOnly|QFile::Truncate))
    {
        QJsonArray arr;
        auto& id_idx = multi_index_nodes.get<by_id>();
        for(const NODE& node : id_idx)
        {
            QJsonObject obj;
            obj["id"] = node.id;
            obj["name"] = node.name;
            obj["type"] = node.type;
            obj["info"] = node.info;
            obj["pose"] = TF_to_string(node.tf);
            obj["links"] = links_to_array(node.linked);

            arr.append(obj);
        }

        QJsonDocument doc(arr);
        node_file.write(doc.toJson());
        node_file.close();

        printf("[UNIMAP] %s saved\n", node_path.toLocal8Bit().data());
    }
}

void UNIMAP::clear_nodes()
{
    std::unique_lock<std::shared_mutex> lock(mtx);

    multi_index_nodes.clear();
    
    // clear KD-tree node data
    kdtree_node.pos.clear();
    kdtree_node.id.clear();
    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    printf("[UNIMAP] topology cleared\n");
}

std::vector<QString> UNIMAP::get_nodes(QString type)
{
    std::shared_lock<std::shared_mutex> lock(mtx);

    std::vector<QString> res;
    auto& id_idx = multi_index_nodes.get<by_id>();
    for(const NODE& node : id_idx)
    {
        if(node.type == type)
        {
            res.push_back(node.id);
        }
    }

    return res;
}

std::vector<NODE> UNIMAP::get_nodes()
{
    std::shared_lock<std::shared_mutex> lock(mtx);

    std::vector<NODE> res;
    auto& id_idx = multi_index_nodes.get<by_id>();
    for(const NODE& node : id_idx)
    {
        res.push_back(node);
    }

    return res;
}

QString UNIMAP::gen_node_id()
{
    return QString("N_%1").arg(static_cast<long long>((get_time()*1000)));;
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

std::vector<NODE> UNIMAP::get_init_candidates_id(QString type, std::vector<QString> name_list)
{
    std::vector<NODE> res;
    if(is_loaded != MAP_LOADED)
    {
        return res;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    auto& id_idx = multi_index_nodes.get<by_id>();
    for(const NODE& node : id_idx)
    {
        if(node.type == type)
        {
            res.push_back(node);
            continue;
        }

        for(size_t q = 0; q < name_list.size(); q++)
        {
            if(node.name.contains(name_list[q]))
            {
                res.push_back(node);
                continue;
            }
        }
    }
    return res;
}

QString UNIMAP::get_map_path()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return map_path;
}

int UNIMAP::get_is_loaded()
{
    return (int)is_loaded.load();
}

void UNIMAP::set_map_path(QString path)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    map_path = path;
}

std::shared_ptr<XYZR_CLOUD> UNIMAP::get_kdtree_cloud()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return kdtree_cloud_2d;
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
    if(is_loaded != MAP_LOADED || !map_3d_normal || (idx < 0 || idx >= (int)map_3d_normal->size()))
    {
        return Eigen::Vector3d(0,0,0);
    }

    return (*map_3d_normal)[idx];
}

Eigen::Vector3d UNIMAP::get_pts_3d(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED || !map_3d_pts || (idx < 0 || idx >= (int)map_3d_normal->size()))
    {
        return Eigen::Vector3d(0,0,0);
    }

    return (*map_3d_pts)[idx];
}

std::shared_ptr<std::vector<double>> UNIMAP::get_map_3d_reflects()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED)
    {
        return {};
    }

    return map_3d_reflects;
}

std::shared_ptr<KD_TREE_XYZR> UNIMAP::get_kdtree_cloud_index()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED)
    {
        return {};
    }

    return kdtree_cloud_2d_index;
}

void UNIMAP::radius_search_kdtree_idx(double query[], double sq_radius, std::vector<nanoflann::ResultItem<unsigned int, double>>& res_idxs, nanoflann::SearchParameters params)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED || !kdtree_cloud_2d_index)
    {
        return;
    }
    
    kdtree_cloud_2d_index->radiusSearch(&query[0], sq_radius, res_idxs, params);
}

QString UNIMAP::get_node_id_edge(Eigen::Vector3d pos)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!is_loaded != MAP_LOADED || multi_index_nodes.size() != 0 || !kdtree_node_index || kdtree_node.id.empty() || kdtree_node.pos.empty())
    {
        return "";
    }

    std::vector<unsigned int> near_idxs(UNIMAP_INFO::knn_search_num, 0);
    std::vector<double> near_sq_dists(UNIMAP_INFO::knn_search_num, 0.0);

    double query_pt[3] = { pos[0], pos[1], pos[2] };
    kdtree_node_index->knnSearch(query_pt, UNIMAP_INFO::knn_search_num, near_idxs.data(), near_sq_dists.data());

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d best_p0, best_p1;
    QString best_id0, best_id1;

    bool found_edge = false;
    for(int i = 0; i < UNIMAP_INFO::knn_search_num; ++i)
    {
        auto idx = near_idxs[i];
        if(idx >= kdtree_node.id.size() || idx >= kdtree_node.pos.size())
        {
            continue;
        }

        Eigen::Vector3d pos0 = kdtree_node.pos[idx];

        QString id0 = kdtree_node.id[idx];
        NODE node0  = get_node_by_id(id0);
        if(node0.id.isEmpty())
        {
            continue;
        }

        for(const QString& id1 : node0.linked)
        {
            NODE node1 = get_node_by_id(id1);
            if(node1.id.isEmpty())
            {
                continue;
            }

            if(is_node_drivable(node1.type))
            {
                Eigen::Vector3d pos1 = node1.tf.block(0,3,3,1);
                double dist = calc_seg_dist(pos0, pos1, pos);
                if(dist < min_dist)
                {
                    min_dist = dist;
                    best_p0 = pos0;
                    best_p1 = pos1;
                    best_id0 = id0;
                    best_id1 = id1;
                    found_edge = true;
                }
            }
        }
    }

    if(found_edge)
    {
        double d0 = calc_dist_2d(best_p0 - pos);
        double d1 = calc_dist_2d(best_p1 - pos);
        return (d0 < d1) ? best_id0 : best_id1;
    }
    else
    {
        auto idx = near_idxs[0];
        if (idx < kdtree_node.id.size())
        {
            return kdtree_node.id[idx];
        }
        else
        {
            return "";
        }
    }
}

NODE UNIMAP::get_goal_node(Eigen::Vector3d pos)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(multi_index_nodes.size() == 0 || !kdtree_node_index || kdtree_node.pos.empty())
    {
        return {};
    }

    const double search_radius = config->get_robot_radius() * UNIMAP_INFO::search_radius_weight;
    const int max_candidates = std::max<int>(UNIMAP_INFO::max_candidates_size, multi_index_nodes.size()); // Limit search candidates for performance
    std::vector<unsigned int> candidate_indices(max_candidates);
    std::vector<double> candidate_distances(max_candidates);

    // Search for nearest nodes using KD-tree
    double query_point[3] = {pos[0], pos[1], pos[2]};
    const int found_count = kdtree_node_index->knnSearch(query_point, max_candidates, candidate_indices.data(), candidate_distances.data());
    if(found_count == 0)
    {
        return {};
    }

    // Check each candidate node
    NODE nearest_node;
    double min_distance_squared  = std::numeric_limits<double>::max();
    double search_radius_squared = search_radius * search_radius;
    for(int i = 0; i < found_count; ++i)
    {
        const unsigned int node_idx = candidate_indices[i];
        const double distance_squared = candidate_distances[i];

        if(distance_squared > search_radius_squared)
        {
            continue;
        }

        if(node_idx >= kdtree_node.id.size())
        {
            continue;
        }

        QString node_id = kdtree_node.id[node_idx];
        NODE node = get_node_by_id(node_id);
        if(node.id.isEmpty())
        {
            continue;
        }

        if(!is_goal_node_type(node.type))
        {
            continue;
        }

        if(distance_squared < min_distance_squared)
        {
            min_distance_squared = distance_squared;
            nearest_node = node;
        }
    }

    const double actual_distance = std::sqrt(min_distance_squared);
    logger->write_log(QString("[UNIMAP] Found nearest goal node (KD-tree): %1 (distance: %2)")
                     .arg(nearest_node.id)
                     .arg(actual_distance), "Green");

    return nearest_node;
}

double UNIMAP::calculate_distance_squared(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
    const Eigen::Vector3d diff = pos1 - pos2;
    return diff.squaredNorm();
}

NODE UNIMAP::get_node_by_id(QString id)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(id.isEmpty())
    {
        return {};
    }

    auto& id_idx = multi_index_nodes.get<by_id>();
    auto it = id_idx.find(id);
    if(it != id_idx.end())
    {
        return *it;
    }
    else
    {
        return {};
    }
}

NODE UNIMAP::get_node_by_name(QString name)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(name.isEmpty())
    {
        return {};
    }

    auto& name_idx = multi_index_nodes.get<by_name>();
    auto it = name_idx.find(name);
    if(it != name_idx.end())
    {
        return *it;
    }
    else
    {
        return {};
    }
}

std::vector<int> UNIMAP::knn_search_idx(Eigen::Vector3d center, int k, double radius)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!kdtree_cloud_3d_index)
    {
        return std::vector<int>();
    }
    
    const double sq_radius = radius*radius;

    std::vector<unsigned int> res_idxs(k);
    std::vector<double> res_sq_dists(k);
    double query_pt[3] = {center[0], center[1], center[2]};

    kdtree_cloud_3d_index->knnSearch(&query_pt[0], k, &res_idxs[0], &res_sq_dists[0]);

    std::vector<int> res;
    for(size_t p = 0; p < res_idxs.size(); p++)
    {
        if(res_idxs[p] < kdtree_cloud_3d.pts.size())
        {
            double dx = kdtree_cloud_3d.pts[res_idxs[p]][0] - kdtree_cloud_3d.pts[res_idxs[0]][0];
            double dy = kdtree_cloud_3d.pts[res_idxs[p]][1] - kdtree_cloud_3d.pts[res_idxs[0]][1];
            double dz = kdtree_cloud_3d.pts[res_idxs[p]][2] - kdtree_cloud_3d.pts[res_idxs[0]][2];
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

// Public node management interface
bool UNIMAP::add_node(const NODE& node)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED)
    {
        logger->write_log("[UNIMAP] Map not loaded", "Red");
        return false;
    }

    if(node.id.isEmpty())
    {
        logger->write_log("[UNIMAP] Cannot add node with empty ID", "Red");
        return false;
    }    

    NODE node_from_id = get_node_by_id(node.id);
    if(!node_from_id.id.isEmpty())
    {
        logger->write_log(QString("[UNIMAP] Node with ID '%1' already exists").arg(node_from_id.id), "Red");
        return false;
    }

    NODE node_from_name = get_node_by_name(node.name);
    if(!node_from_name.id.isEmpty())
    {
        logger->write_log(QString("[UNIMAP] Node with ID '%1' already exists").arg(node_from_name.id), "Red");
        return false;
    }

    multi_index_nodes.insert(node);
    logger->write_log(QString("[UNIMAP] Added node: %1").arg(node.id), "Green");

    // Update KD-tree if needed
    if(is_node_drivable(node.type))
    {
        kdtree_node.pos.push_back(node.tf.block(0,3,3,1));
        kdtree_node.id.push_back(node.id);
        
        // Rebuild KD-tree
        if(kdtree_node_index)
        {
            kdtree_node_index.reset();
        }
        kdtree_node_index = std::make_unique<KD_TREE_NODE>(3, kdtree_node, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree_node_index->buildIndex();
    }

    return true;
}

bool UNIMAP::add_node(const Eigen::Matrix4d tf, const QString type)
{
    NODE node;
    node.id = gen_node_id();
    node.type = type;
    node.tf = tf;

    return add_node(node);
}

bool UNIMAP::remove_node(const QString& id)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    if(is_loaded != MAP_LOADED)
    {
        logger->write_log("[UNIMAP] Map not loaded", "Red");
        return false;
    }

    if(id.isEmpty())
    {
        logger->write_log("[UNIMAP] Cannot remove node with empty ID", "Red");
        return false;
    }
    
    NODE node_from_id = get_node_by_id(id);
    if(node_from_id.id.isEmpty())
    {
        logger->write_log(QString("[UNIMAP] Node with ID '%1' not found").arg(node_from_id.id), "Red");
        return false;
    }

    auto& id_idx = multi_index_nodes.get<by_id>();
    auto it = id_idx.find(id);
    id_idx.erase(it);
    
    // Rebuild KD-tree
    kdtree_node.pos.clear();
    kdtree_node.id.clear();

    for(const NODE& node : id_idx)
    {
        if(is_node_drivable(node.type))
        {
            kdtree_node.pos.push_back(node.tf.block(0,3,3,1));
            kdtree_node.id.push_back(node.id);
        }
    }
    
    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    if(!kdtree_node.pos.empty())
    {
        kdtree_node_index = std::make_unique<KD_TREE_NODE>(3, kdtree_node, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        kdtree_node_index->buildIndex();
    }

    logger->write_log(QString("[UNIMAP] Removed node: %1").arg(id), "Green");
    return true;
}

bool UNIMAP::is_goal_node_type(const QString& type)
{
    return (type == "GOAL" || type == "INIT" || type == "STATION");
}

bool UNIMAP::is_node_drivable(QString type)
{
    return (type == "ROUTE" || type == "GOAL" || type == "INIT" || type == "STATION");
}
