#include "unimap.h"
namespace 
{
    const char* MODULE_NAME = "UNIMAP";
}

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
    // init nodes
    nodes = std::make_shared<std::vector<NODE>>();

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
    if(nodes)
    {
        nodes->clear();
    }
    nodes_id_map.clear();
    nodes_name_map.clear();

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

QString UNIMAP::is_load_map_check(QString path)
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

    QString msg;

    if(!loaded_2d)
    {
        return msg = "no 2d map!";
    }
    if(!loaded_3d)
    {
        return msg = "no 3d map!";
    }
    return msg = "all map is available!";
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

    // topo or node load
    QFileInfo node_info(QString(map_path + "/node.json"));
    QFileInfo topo_info(QString(map_path + "/topo.json"));
    if(node_info.exists() && node_info.isFile())
    {
        load_node();
    }
    else if(topo_info.exists() && topo_info.isFile())
    {
        load_topo();
    }

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
                            spdlog::error("[UNIMAP(2D)] critical error loading 2d map.");
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
                        // kdtree_cloud_2d->pts = voxel_filtering(pts, 0.05);
                        kdtree_cloud_2d->pts = pts;
                    }
                }

                //printf("[UNIMAP(2D)] %s loaded, map_pts:%d\n", cloud_csv_path.toLocal8Bit().data(), (int)kdtree_cloud_2d->pts.size());
                spdlog::info("[UNIMAP(2D)] {} loaded, map_pts:{}", cloud_csv_path.toStdString(), (int)kdtree_cloud_2d->pts.size());
            }
        }
        else
        {
            //printf("[UNIMAP(2D)] load failed\n");
            spdlog::error("[UNIMAP(2D)] load failed");
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

            //printf("[UNIMAP(2D)] build kdtree for map icp\n");
            //printf("[UNIMAP(2D)] boundary x:(%.2f, %.2f), y:(%.2f, %.2f), z:(%.2f, %.2f)\n",
            //       map_min_x, map_max_x, map_min_y, map_max_y, map_min_z, map_max_z);
            spdlog::info("[UNIMAP(2D)] build kdtree for map icp");
            spdlog::info("[UNIMAP(2D)] boundary x:({:.2f}, {:.2f}), y:({:.2f}, {:.2f}), z:({:.2f}, {:.2f})",
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
        //printf("[UNIMAP(3D)] map_dir is empty!\n");
        spdlog::error("[UNIMAP(3D)] map_dir is empty!");
        return false;
    }

    QString file_path = path + "/map.las";
    if(!QFile::exists(file_path))
    {
        qDebug()<<"no las!!!!!!!!!!!";
        return false;
    }

    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createStage("readers.las");
    if(reader == nullptr)
    {
        //printf("[UNIMAP(3D)] map read failed\n");
        spdlog::error("[UNIMAP(3D)] map read failed");
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
    spdlog::info("[UNIMAP(3D)] Loaded {} points from {}", pts.size(), file_path.toStdString());
    return true;
}

bool UNIMAP::load_topo()
{
    // first clear node
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if(nodes)
        {
            nodes->clear();
        }
        links.clear();
    }
    
    QString node_path = map_path + "/topo.json";
    QFileInfo node_info(node_path);
    if(!node_info.exists() || !node_info.isFile())
    {
        return false;
    }

    std::vector<LINK> temp_links;

    QFile node_file(node_path);
    if(node_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = node_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);

        QJsonArray arr = doc.array();
        std::vector<NODE> temp_nodes;
        Q_FOREACH(const QJsonValue &val, arr)
        {
            QJsonObject obj = val.toObject();
            NODE node;
            node.id     = obj["id"].toString();
            node.name   = obj["name"].toString();
            node.type   = obj["type"].toString();
            node.context    = obj["info"].toString();
            node.tf     = string_to_TF(obj["pose"].toString());

            // link
            {
                QJsonArray link_arr = obj["links"].toArray();
                QString last_link = "";
                for(int i = 0; i < link_arr.size(); i++)
                {
                    const QJsonValue &lv = link_arr.at(i);
                    if(lv.isString())
                    {
                        QString ed = lv.toString().trimmed();
                        if(ed.isEmpty())
                        {
                            continue;
                        }

                        LINK L;
                        L.st_id = node.id;
                        L.ed_id = ed;
                        temp_links.push_back(L);
                    }
                }
            }

            temp_nodes.push_back(node);
        }
        node_file.close();
        
        // set fast find nodes instance
        {
            std::unique_lock<std::shared_mutex> lock(mtx);
            if(nodes)
            {
                *nodes = temp_nodes;
            }
            links = temp_links;

            // rebuild node maps with indices
            rebuild_node_maps();

            // build KD-tree for nodes
            std::vector<Eigen::Vector3d> node_pos; 
            std::vector<QString> node_id; 
            
            for(size_t p = 0; p < nodes->size(); p++)
            {
                const NODE& node = (*nodes)[p];
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
        spdlog::info("[UNIMAP] {} loaded", node_path.toStdString());
    }
    return true;
}

bool UNIMAP::load_node()
{
    // first clear node
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if(nodes)
        {
            nodes->clear();
        }
        links.clear();
    }

    QString node_path = map_path + "/node.json";
    QFileInfo node_info(node_path);
    if(!node_info.exists() || !node_info.isFile())
    {
        return false;
    }

    std::vector<LINK> temp_links;

    QFile node_file(node_path);
    if(!node_file.open(QIODevice::ReadOnly))
    {
        return false;
    }


    QByteArray data = node_file.readAll();
    node_file.close();

    QJsonDocument doc = QJsonDocument::fromJson(data);
    if(!doc.isArray())
    {
        spdlog::error("[UNIMAP] node.json root is not array");
        return false;
    }

    QJsonArray arr = doc.array();
    std::vector<NODE> temp_nodes;
    temp_nodes.reserve(arr.size());

    Q_FOREACH(const QJsonValue &val, arr)
    {
        if(!val.isObject())
        {
            continue;
        }

        QJsonObject obj = val.toObject();
        NODE node;

        // id, name, type, context
        node.id     = obj["id"].toString();
        node.name   = obj["name"].toString();
        node.type   = obj["type"].toString();
        node.context   = obj["context"].toString();

        // pose
        node.tf     = string_to_TF(obj["pose"].toString());

        // size
        node.size.setZero();
        if(obj.contains("size"))
        {
            QString size = obj["size"].toString();
            QStringList str_list = size.split(",", Qt::SkipEmptyParts);
            if(str_list.size() >= 3)
            {
                node.size.x() = str_list[0].toDouble();
                node.size.y() = str_list[1].toDouble();
                node.size.z() = str_list[2].toDouble();
            }
        }

        // role
        node.role = NODE_ROLE();
        if(obj.contains("role"))
        {
            QString role = obj["role"].toString();
            QStringList str_list = role.split(",", Qt::SkipEmptyParts);
            for(int i = 0; i < str_list.size(); i++)
            {
                QString str = str_list[i].trimmed();
                if(str == "CONVEYOR")
                {
                    node.role.conveyor = true;
                }
                else if(str == "WARNING_BEEP")
                {
                     node.role.warning_beep = true;
                }
                else if(str == "IGNORE_2D")
                {
                    node.role.ignore_2d = true;
                }
                else if(str == "IGNORE_3D")
                {
                    node.role.ignore_3d = true;
                }
                else if(str == "IGNORE_CAM")
                {
                    node.role.ignore_cam = true;
                }
                else if(str == "IGNORE_OBS_2D")
                {
                    node.role.ignore_obs_2d = true;
                }
                else if(str == "IGNORE_OBS_3D")
                {
                    node.role.ignore_obs_3d = true;
                }
                else if(str == "IGNORE_OBS_CAM")
                {
                    node.role.ignore_obs_cam = true;
                }
            }
        }

        // links
        if(obj.contains("links") && obj["links"].isArray())
        {
            QJsonArray link_arr = obj["links"].toArray();
            Q_FOREACH(const QJsonValue &lv, link_arr)
            {
                if(!lv.isObject())
                {
                    continue;
                }

                QJsonObject obj_link = lv.toObject();
                QString ed = obj_link["id"].toString();
                if(ed.isEmpty())
                {
                    continue;
                }

                LINK L;
                L.st_id = node.id;
                L.ed_id = ed;

                // dir
                if(obj_link.contains("dir"))
                {
                    L.dir = obj_link["dir"].toString();
                }
                else
                {
                    L.dir = "";
                }

                // method
                if(obj_link.contains("method"))
                {
                    L.method = obj_link["method"].toString();
                }
                else
                {
                    L.method = "";
                }

                // speed
                if(obj_link.contains("speed"))
                {
                    L.speed = obj_link["speed"].toDouble();
                }
                else
                {
                    L.speed = 0.0;
                }

                temp_links.push_back(L);
            }
        }
        temp_nodes.push_back(node);
    }

    // set fast find nodes instance
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        if(nodes)
        {
            *nodes = temp_nodes;
        }

        // rebuild node maps with indices
        rebuild_node_maps();

        links.clear();
        links.reserve(temp_links.size());
        for(size_t i = 0; i < temp_links.size(); i++)
        {
            LINK link = temp_links[i];

            links.push_back(link);

            // debug
            // {
            //     for(size_t i = 0; i < links.size(); i++)
            //     {
            //         const LINK_INFO &s = links[i];
            //         if(s.info != "" || s.method != "" || s.speed > 0.0)
            //         {
            //             printf("[UNIMAP] %s -> %s, info = %s, method = %s",
            //                    s.st_id.toStdString().c_str(),
            //                    s.ed_id.toStdString().c_str(),
            //                    s.info.toStdString().c_str(),
            //                    s.method.toStdString().c_str());

            //             if(s.speed > 0.0)
            //             {
            //                 printf(", speed = %f", s.speed);
            //             }
            //             printf("\n");
            //         }
            //     }
            // }
        }

        // build KD-tree for nodes
        std::vector<Eigen::Vector3d> node_pos;
        std::vector<QString> node_id;
        for(size_t p = 0; p < nodes->size(); p++)
        {
            const NODE& node = (*nodes)[p];
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
    spdlog::info("[UNIMAP] {} loaded", node_path.toStdString());

    return true;
}

void UNIMAP::save_node()
{
    // get save folder
    QString path = map_path;

    // decide target by presence of node.json
    QString node_path = path + "/node.json";
    QFileInfo node_info(node_path);

    // save as node.json
    {
        QFile node_file(node_path);
        if(node_file.open(QIODevice::WriteOnly | QFile::Truncate))
        {
            QJsonArray arr;
            for(size_t p = 0; p < nodes->size(); p++)
            {
                const NODE &n = (*nodes)[p];

                QJsonObject obj;
                obj["id"]   = n.id;
                obj["name"] = n.name;
                obj["type"] = n.type;
                obj["context"] = n.context;
                obj["pose"] = TF_to_string(n.tf);

                // size
                if(n.size.x() != 0.0 || n.size.y() != 0.0 || n.size.z() != 0.0)
                {
                    QString size_str = QString("%1,%2,%3")
                            .arg(n.size.x(), 0, 'f', 6)
                            .arg(n.size.y(), 0, 'f', 6)
                            .arg(n.size.z(), 0, 'f', 6);
                    obj["size"] = size_str;
                }
                else
                {
                    obj["size"] = QString("");
                }

                // role
                QStringList role_list;
                if(n.role.conveyor)
                {
                    role_list.append("CONVEYOR");
                }
                if(n.role.warning_beep)
                {
                    role_list.append("WARNING_BEEP");
                }
                if(n.role.ignore_2d)
                {
                    role_list.append("IGNORE_2D");
                }
                if(n.role.ignore_3d)
                {
                    role_list.append("IGNORE_3D");
                }
                if(n.role.ignore_cam)
                {
                    role_list.append("IGNORE_CAM");
                }
                if(n.role.ignore_obs_2d)
                {
                    role_list.append("IGNORE_OBS_2D");
                }
                if(n.role.ignore_obs_3d)
                {
                    role_list.append("IGNORE_OBS_3D");
                }
                if(n.role.ignore_obs_cam)
                {
                    role_list.append("IGNORE_OBS_CAM");
                }

                obj["role"] = role_list.join(", ");

                // links
                QJsonArray link_arr;
                for(size_t i = 0; i < links.size(); i++)
                {
                    const LINK &link = links[i];

                    if(link.st_id != n.id)
                    {
                        continue;
                    }

                    QJsonObject obj_link;
                    obj_link["id"] = link.ed_id;

                    if(!link.dir.isEmpty())
                    {
                        obj_link["dir"] = link.dir;
                    }
                    if(!link.method.isEmpty())
                    {
                        obj_link["method"] = link.method;
                    }
                    if(link.speed > 0.0)
                    {
                        obj_link["speed"] = link.speed;
                    }

                    link_arr.append(obj_link);
                }

                obj["links"] = link_arr;
                arr.append(obj);
            }

            QJsonDocument doc(arr);
            node_file.write(doc.toJson());
            node_file.close();

            spdlog::info("[UNIMAP] {} saved", node_path.toStdString());
        }
    }

    // save as topo.json
    {
        // save as topo.json
        QString topo_path = path + "/topo.json";
        QFile topo_file(topo_path);
        if(topo_file.open(QIODevice::WriteOnly|QFile::Truncate))
        {
            QJsonArray arr;
            for(size_t p = 0; p < nodes->size(); p++)
            {
                QJsonObject obj;
                obj["id"] = (*nodes)[p].id;
                obj["name"] = (*nodes)[p].name;
                obj["type"] = (*nodes)[p].type;
                obj["info"] = (*nodes)[p].context;
                obj["pose"] = TF_to_string((*nodes)[p].tf);

                QJsonArray link_arr;
                for(size_t k = 0; k < links.size(); k++)
                {
                    const LINK &lk = links[k];

                    if(lk.st_id == (*nodes)[p].id)
                    {
                        link_arr.append(lk.ed_id);
                    }
                }

                if(!link_arr.isEmpty())
                {
                    obj["links"] = link_arr;
                }

                arr.append(obj);
            }

            QJsonDocument doc(arr);
            topo_file.write(doc.toJson());
            topo_file.close();

            //printf("[UNIMAP] %s saved\n", topo_path.toLocal8Bit().data());
            spdlog::info("[UNIMAP] {} saved", topo_path.toStdString());
        }
    }
}

void UNIMAP::save_map()
{
    if(map_path.isEmpty())
    {
        spdlog::error("[UNIMAP] Cannot save map: map_path is empty");
        return;
    }

    spdlog::info("[UNIMAP] Saving map to {}", map_path.toStdString());

    // Merge additional_cloud into kdtree_cloud_2d before saving
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        
        if(!additional_cloud.empty() && kdtree_cloud_2d)
        {
            spdlog::info("[UNIMAP] Merging {} additional points into map", additional_cloud.size());
            
            for(const auto& pt : additional_cloud)
            {
                PT_XYZR map_pt;
                map_pt.x = pt.x();
                map_pt.y = pt.y();
                map_pt.z = pt.z();
                map_pt.r = 128.0;  // default reflectance
                kdtree_cloud_2d->pts.push_back(map_pt);
            }
            
            // Clear additional_cloud after merging
            additional_cloud.clear();
            
            // Rebuild KD-tree with merged points
            if(kdtree_cloud_2d_index)
            {
                kdtree_cloud_2d_index.reset();
            }
            kdtree_cloud_2d_index = std::make_shared<KD_TREE_XYZR>(3, *kdtree_cloud_2d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            kdtree_cloud_2d_index->buildIndex();
            
            spdlog::info("[UNIMAP] Additional points merged. Total map points: {}", kdtree_cloud_2d->pts.size());
        }
        
        // Clear removed_cloud (these points are already removed from map)
        if(!removed_cloud.empty())
        {
            spdlog::info("[UNIMAP] Clearing {} removed points", removed_cloud.size());
            removed_cloud.clear();
        }
    }

    // Save 2D map
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        
        if(kdtree_cloud_2d && !kdtree_cloud_2d->pts.empty())
        {
            QString cloud_csv_path = map_path + "/cloud.csv";
            QFile cloud_csv_file(cloud_csv_path);
            
            if(cloud_csv_file.open(QIODevice::WriteOnly | QFile::Truncate))
            {
                QTextStream out(&cloud_csv_file);
                
                for(const auto& pt : kdtree_cloud_2d->pts)
                {
                    out << QString::number(pt.x, 'f', 6) << ","
                        << QString::number(pt.y, 'f', 6) << ","
                        << QString::number(pt.z, 'f', 6) << ","
                        << QString::number(pt.r, 'f', 6) << "\n";
                }
                
                cloud_csv_file.close();
                spdlog::info("[UNIMAP(2D)] Saved {} points to {}", 
                           kdtree_cloud_2d->pts.size(), 
                           cloud_csv_path.toStdString());
            }
            else
            {
                spdlog::error("[UNIMAP(2D)] Failed to open {} for writing", cloud_csv_path.toStdString());
            }
        }
    }

    spdlog::info("[UNIMAP] Map save completed");
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
    
    // clear KD-tree node data
    kdtree_node.pos.clear();
    kdtree_node.id.clear();
    if(kdtree_node_index)
    {
        kdtree_node_index.reset();
    }

    //printf("[UNIMAP] topology cleared\n");
    spdlog::info("[UNIMAP] topology cleared");
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

std::shared_ptr<std::vector<NODE>> UNIMAP::get_nodes_origin()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return nodes;
}

std::vector<LINK> UNIMAP::get_links()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return links;
}

QString UNIMAP::gen_node_id()
{
    QString res;
    res.sprintf("N_%lld", (long long)(get_time()*1000));
    return res;
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

QString UNIMAP::get_map_path()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    QString res = map_path;
    return res;
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

std::shared_ptr<KD_TREE_XYZR> UNIMAP::get_kdtree_cloud_index()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return kdtree_cloud_2d_index;
}

void UNIMAP::radius_search_kdtree_idx(double query[], double sq_radius, std::vector<nanoflann::ResultItem<unsigned int, double>>& res_idxs, nanoflann::SearchParameters params)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!kdtree_cloud_2d_index)
    {
        return;
    }
    
    kdtree_cloud_2d_index->radiusSearch(&query[0], sq_radius, res_idxs, params);
}

QString UNIMAP::get_node_id_edge(Eigen::Vector3d pos)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    if(!nodes || nodes->empty() || !kdtree_node_index || kdtree_node.id.empty() || kdtree_node.pos.empty())
    {
        return "";
    }

    std::vector<unsigned int> near_idxs(knn_search_num, 0);
    std::vector<double> near_sq_dists(knn_search_num, 0.0);

    double query_pt[3] = { pos[0], pos[1], pos[2] };
    kdtree_node_index->knnSearch(query_pt, knn_search_num, near_idxs.data(), near_sq_dists.data());

    double min_dist = std::numeric_limits<double>::max();
    Eigen::Vector3d best_p0, best_p1;
    QString best_id0, best_id1;

    bool found_edge = false;
    for(int i = 0; i < knn_search_num; ++i)
    {
        auto idx = near_idxs[i];
        if(idx >= kdtree_node.id.size() || idx >= kdtree_node.pos.size())
        {
            continue;
        }

        QString id0 = kdtree_node.id[idx];
        NODE* node0 = get_node_by_id(id0);
        if(!node0)
        {
            continue;
        }

        Eigen::Vector3d pos0 = kdtree_node.pos[idx];
        for(size_t k = 0; k < links.size(); k++)
        {
            const LINK &link = links[k];

            QString id1;
            bool connected = false;

            if(link.st_id == id0)
            {
                id1 = link.ed_id;
                connected = true;
            }
            else if(link.ed_id == id0)
            {
                id1 = link.st_id;
                connected = true;
            }

            if(!connected)
            {
                continue;
            }


            NODE* node1 = get_node_by_id(id1);
            if(!node1)
            {
                continue;
            }

            if(node1->type == "ROUTE" || node1->type == "GOAL" || node1->type == "INIT" || node1->type == "STATION")
            {
                Eigen::Vector3d pos1 = node1->tf.block(0,3,3,1);
                double dist = calc_seg_dist(pos0, pos1, pos);
                if(dist < min_dist)
                {
                    min_dist = dist;
                    best_id0 = id0;
                    best_id1 = id1;
                    found_edge = true;
                }
            }
        }
    }

    if(found_edge)
    {
        NODE* n0 = get_node_by_id(best_id0);
        NODE* n1 = get_node_by_id(best_id1);

        if(n0 != NULL && n1 != NULL)
        {
            Eigen::Vector3d p0 = n0->tf.block(0,3,3,1);
            Eigen::Vector3d p1 = n1->tf.block(0,3,3,1);

            double d0 = calc_dist_2d(p0 - pos);
            double d1 = calc_dist_2d(p1 - pos);

            if(d0 < d1)
            {
                return best_id0;
            }
            else
            {
                return best_id1;
            }
        }
    }

    auto idx = near_idxs[0];
    if(idx < kdtree_node.id.size())
    {
        return kdtree_node.id[idx];
    }
    return "";
}

std::vector<QString> UNIMAP::get_edge_nodes(const Eigen::Vector3d& pos)
{
    std::shared_lock<std::shared_mutex> lock(mtx);

    std::vector<QString> edge_nodes;

    if(!nodes || nodes->empty() || !kdtree_node_index || kdtree_node.id.empty() || kdtree_node.pos.empty())
    {
        return edge_nodes;
    }

    std::vector<unsigned int> near_idxs(knn_search_num, 0);
    std::vector<double> near_sq_dists(knn_search_num, 0.0);

    double query_pt[3] = { pos[0], pos[1], pos[2] };
    kdtree_node_index->knnSearch(query_pt, knn_search_num, near_idxs.data(), near_sq_dists.data());

    double min_dist = std::numeric_limits<double>::max();
    QString best_id0, best_id1;

    for(int i = 0; i < knn_search_num; i++)
    {
        unsigned int idx = near_idxs[i];
        if(idx >= kdtree_node.id.size() || idx >= kdtree_node.pos.size())
        {
            continue;
        }

        QString id0 = kdtree_node.id[idx];
        NODE* node0 = get_node_by_id(id0);
        if(!node0)
        {
            continue;
        }

        Eigen::Vector3d pos0 = kdtree_node.pos[idx];
        for(size_t k = 0; k < links.size(); k++)
        {
            const LINK &link = links[k];
            QString neighbor_id = "";
            bool is_neighbor = false;

            if(link.st_id == id0)
            {
                neighbor_id = link.ed_id;
                is_neighbor = true;
            }
            else if(link.ed_id == id0)
            {
                neighbor_id = link.st_id;
                is_neighbor = true;
            }

            if(!is_neighbor)
            {
                continue;
                }

            NODE* node1 = get_node_by_id(neighbor_id);
            if(!node1)
            {
                continue;
            }

            if(node1->type == "ROUTE" || node1->type == "GOAL" || node1->type == "INIT" || node1->type == "STATION")
            {
                Eigen::Vector3d pos1 = node1->tf.block(0,3,3,1);
                double dist = calc_seg_dist(pos0, pos1, pos);
                if(dist < min_dist)
                {
                    min_dist = dist;
                    best_id0 = id0;
                    best_id1 = neighbor_id;
                }
            }
        }
    }

    if(!best_id0.isEmpty() && !best_id1.isEmpty())
    {
        edge_nodes.push_back(best_id0);
        edge_nodes.push_back(best_id1);
    }

    return edge_nodes;
}

QString UNIMAP::get_goal_id(Eigen::Vector3d pos)
{
    if(!nodes || nodes->empty())
    {
        std::cout << "node empty" << std::endl;
        spdlog::warn("[UNIMAP] node empty");
        return "";
    }

    if(!kdtree_node_index || kdtree_node.pos.empty())
    {
        std::cout << "kdtree empty" << std::endl;
        spdlog::warn("[UNIMAP] kdtree empty");
        return "";
    }

    const double search_radius = config ? config->get_robot_radius() * 5.0 : 2.0;

    std::shared_lock<std::shared_mutex> lock(mtx);

    const int max_candidates = std::max<int>(20, nodes->size()); // Limit search candidates for performance
    std::vector<unsigned int> candidate_indices(max_candidates);
    std::vector<double> candidate_distances(max_candidates);

    // Search for nearest nodes using KD-tree
    double query_point[3] = {pos[0], pos[1], pos[2]};
    const int found_count = kdtree_node_index->knnSearch(query_point, max_candidates, candidate_indices.data(), candidate_distances.data());
    if(found_count == 0)
    {
        std::cout << "found count 0" << std::endl;
        spdlog::warn("[UNIMAP] found count 0");
        return "";
    }

    const double search_radius_squared = search_radius * search_radius;
    NODE* nearest_node = nullptr;
    double min_distance_squared = std::numeric_limits<double>::max();

    // Check each candidate node
    for(int i = 0; i < found_count; ++i)
    {
        const unsigned int node_idx = candidate_indices[i];
        const double distance_squared = candidate_distances[i];

        // Skip if outside search radius
        if(distance_squared > search_radius_squared)
        {
            continue;
        }

        // Get node ID from KD-tree
        if(node_idx >= kdtree_node.id.size())
        {
            continue;
        }

        const QString& node_id = kdtree_node.id[node_idx];

        // Find the actual node object
        auto it = nodes_id_map.find(node_id);
        if(it == nodes_id_map.end())
        {
            continue;
        }

        size_t index = it->second;
        if(!is_valid_node_index(index))
        {
            continue;
        }

        NODE* _node = &(*nodes)[index];
        if(_node)
        {
            QString type = _node->type;
            if(!is_goal_node_type(type))
            {
                continue;
            }
        }

        // Update nearest node if this one is closer
        if(distance_squared < min_distance_squared)
        {
            min_distance_squared = distance_squared;
            nearest_node = _node;
        }
    }

    // Log the result for debugging
    if(logger && nearest_node)
    {
        const double actual_distance = std::sqrt(min_distance_squared);
        logger->write_log(QString("[UNIMAP] Found nearest goal node (KD-tree): %1 (distance: %2)")
                         .arg(nearest_node->id)
                         .arg(actual_distance), "Green");
        spdlog::info("[UNIMAP] Found nearest goal node (KD-tree): {} (distance: {})", nearest_node->id.toStdString(), actual_distance);
    }

    return nearest_node? nearest_node->id : "";
}

bool UNIMAP::is_goal_node_type(const QString& node_type)
{
    return node_type == "GOAL" || node_type == "INIT" || node_type == "STATION";
}

double UNIMAP::calculate_distance_squared(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2)
{
    const Eigen::Vector3d diff = pos1 - pos2;
    return diff.squaredNorm();
}

NODE* UNIMAP::get_node_by_id(QString id)
{
    if(id.isEmpty())
    {
        return nullptr;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    auto it = nodes_id_map.find(id);
    if(it != nodes_id_map.end())
    {
        size_t index = it->second;
        if(is_valid_node_index(index))
        {
            return &(*nodes)[index];
        }
    }

    return nullptr;
}

NODE* UNIMAP::get_node_by_name(QString name)
{
    if(name.isEmpty())
    {
        return nullptr;
    }

    std::shared_lock<std::shared_mutex> lock(mtx);
    
    auto it = nodes_name_map.find(name);
    if(it != nodes_name_map.end())
    {
        size_t index = it->second;
        if(is_valid_node_index(index))
        {
            return &(*nodes)[index];
        }
    }

    return nullptr;
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

        Eigen::Matrix4d res_tf = node->tf.inverse()*tf;

        double x = res_tf(0,3);
        double y = res_tf(1,3);
        double z = res_tf(2,3);

        if(x > -node->size[0]/2 && x < node->size[0]/ 2 &&
           y > -node->size[1]/2 && y < node->size[1]/ 2 &&
           z > -node->size[2]/2 && z < node->size[2]/ 2)
        {
            zone_type = node->type;
            break;
        }

        // QString info = node->info;
        // NODE_INFO res;
        // if(parse_info(info, "SIZE", res))
        // {
        //     Eigen::Matrix4d res_tf = node->tf.inverse()*tf;

        //     double x = res_tf(0,3);
        //     double y = res_tf(1,3);
        //     double z = res_tf(2,3);

        //     if(x > -res.sz[0]/2 && x < res.sz[0]/ 2 &&
        //             y > -res.sz[1]/2 && y < res.sz[1]/ 2 &&
        //             z > -res.sz[2]/2 && z < res.sz[2]/ 2)
        //     {
        //         zone_type = node->type;
        //         break;
        //     }
        // }
    }

    return zone_type;
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

void UNIMAP::rebuild_node_maps()
{
    nodes_id_map.clear();
    nodes_name_map.clear();
    
    if(!nodes)
    {
        return;
    }
    
    for(size_t i = 0; i < nodes->size(); i++)
    {
        const NODE& node = (*nodes)[i];
        add_node_to_maps(node, i);
    }
}

void UNIMAP::add_node_to_maps(const NODE& node, size_t index)
{
    if(!node.id.isEmpty())
    {
        nodes_id_map[node.id] = index;
    }
    if(!node.name.isEmpty())
    {
        nodes_name_map[node.name] = index;
    }
}

void UNIMAP::remove_node_from_maps(const QString& id, const QString& name)
{
    if(!id.isEmpty())
    {
        nodes_id_map.erase(id);
    }
    if(!name.isEmpty())
    {
        nodes_name_map.erase(name);
    }
}

void UNIMAP::add_link1(PICKING pick)
{
    if(pick.pre_node == "" && pick.cur_node == "" && pick.pre_node == pick.cur_node)
    {
        return;
    }

    NODE* node0 = get_node_by_id(pick.pre_node);
    NODE* node1 = get_node_by_id(pick.cur_node);
    if(node0 == NULL || node1 == NULL)
    {
        return;
    }

    int found_idx = -1;
    for(size_t i = 0; i < links.size(); i++)
    {
        if(links[i].st_id == pick.pre_node && links[i].ed_id == pick.cur_node)
        {
            found_idx = (int)i;
            break;
        }
    }

    if(found_idx < 0)
    {
        LINK L;
        L.st_id  = pick.pre_node;
        L.ed_id  = pick.cur_node;

        links.push_back(L);
    }
    else
    {
        links.erase(links.begin() + found_idx);
    }

}

void UNIMAP::add_link2(PICKING pick)
{
    if(pick.pre_node == "" && pick.cur_node == "" && pick.pre_node == pick.cur_node)
    {
        return;
    }

    NODE* node0 = get_node_by_id(pick.pre_node);
    NODE* node1 = get_node_by_id(pick.cur_node);
    if(node0 == NULL || node1 == NULL)
    {
        return;
    }

    int idx_0_to_1 = -1;
    int idx_1_to_0 = -1;

    for(size_t i = 0; i < links.size(); i++)
    {
        if(links[i].st_id == pick.pre_node && links[i].ed_id == pick.cur_node)
        {
            idx_0_to_1 = (int)i;
        }
        else if(links[i].st_id == pick.cur_node && links[i].ed_id == pick.pre_node)
        {
            idx_1_to_0 = (int)i;
        }
    }

    if(idx_0_to_1 < 0 && idx_1_to_0 < 0)
    {
        LINK L01;
        L01.st_id  = pick.pre_node;
        L01.ed_id  = pick.cur_node;

        LINK L10;
        L10.st_id  = pick.cur_node;
        L10.ed_id  = pick.pre_node;

        links.push_back(L01);
        links.push_back(L10);
    }
    else
    {
        if(idx_0_to_1 > idx_1_to_0)
        {
            if(idx_0_to_1 >= 0)
            {
                links.erase(links.begin() + idx_0_to_1);
            }
            if(idx_1_to_0 >= 0)
            {
                links.erase(links.begin() + idx_1_to_0);
            }
        }
        else
        {
            if(idx_1_to_0 >= 0)
            {
                links.erase(links.begin() + idx_1_to_0);
            }
            if(idx_0_to_1 >= 0)
            {
                links.erase(links.begin() + idx_0_to_1);
            }
        }
    }
}

void UNIMAP::update_node_in_maps(const NODE& node, size_t index)
{
    // Remove old entries first
    if(!node.id.isEmpty())
    {
        auto old_it = nodes_id_map.find(node.id);
        if(old_it != nodes_id_map.end() && old_it->second != index)
        {
            // This ID was used by a different node, remove it
            nodes_id_map.erase(old_it);
        }
    }
    if(!node.name.isEmpty())
    {
        auto old_it = nodes_name_map.find(node.name);
        if(old_it != nodes_name_map.end() && old_it->second != index)
        {
            // This name was used by a different node, remove it
            nodes_name_map.erase(old_it);
        }
    }
    
    // Add new entries
    add_node_to_maps(node, index);
}

void UNIMAP::replace_node(PICKING pick)
{
    if(pick.pre_node.isEmpty() || pick.cur_node.isEmpty())
    {
        printf("[UNIMAP] replace_node, pre_node or cur_node empty\n");
        return;
    }

    if(pick.pre_node == pick.cur_node)
    {
        printf("[UNIMAP] replace_node, pre_node == cur_node\n");
        return;
    }

    NODE* pre = get_node_by_id(pick.pre_node);
    NODE* cur = get_node_by_id(pick.cur_node);

    if(pre == NULL || cur == NULL)
    {
        printf("[UNIMAP] replace_node, pre or cur not found\n");
        return;
    }

    pre->tf = cur->tf;

    remove_node(pick.cur_node);

    printf("[UNIMAP] replace_node, pre:%s moved to cur:%s, cur deleted\n",
           pick.pre_node.toLocal8Bit().data(),
           pick.cur_node.toLocal8Bit().data());
}

bool UNIMAP::is_valid_node_index(size_t index)
{
    return nodes && index < nodes->size();
}

// Public node management interface
bool UNIMAP::add_node(const NODE& node)
{
    if(node.id.isEmpty())
    {
        if(logger)
        {
            logger->write_log("[UNIMAP] Cannot add node with empty ID", "Red");
        }
        spdlog::warn("[UNIMAP] Cannot add node with empty ID");
        return false;
    }
    
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    // Check if node already exists
    if(nodes_id_map.find(node.id) != nodes_id_map.end())
    {
        if(logger)
        {
            logger->write_log(QString("[UNIMAP] Node with ID '%1' already exists").arg(node.id), "Red");
        }
        spdlog::warn("[UNIMAP] Node with ID '{}' already exists", node.id.toStdString());
        return false;
    }
    
    if(!node.name.isEmpty() && nodes_name_map.find(node.name) != nodes_name_map.end())
    {
        if(logger)
        {
            logger->write_log(QString("[UNIMAP] Node with name '%1' already exists").arg(node.name), "Red");
        }
        spdlog::warn("[UNIMAP] Node with name '{}' already exists", node.name.toStdString());
        return false;
    }
    
    // Add node to vector
    if(!nodes)
    {
        nodes = std::make_shared<std::vector<NODE>>();
    }
    
    size_t new_index = nodes->size();
    nodes->push_back(node);
    
    // Add to maps
    add_node_to_maps(node, new_index);
    
    // Update KD-tree if needed
    if(node.type == "ROUTE" || node.type == "GOAL" || node.type == "INIT" || node.type == "STATION")
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
    
    if(logger)
    {
        logger->write_log(QString("[UNIMAP] Added node: %1").arg(node.id), "Green");
    }
    spdlog::info("[UNIMAP] Added node: {}", node.id.toStdString());
    
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
    if(id.isEmpty())
    {
        return false;
    }
    
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    auto it = nodes_id_map.find(id);
    if(it == nodes_id_map.end())
    {
        if(logger)
        {
            logger->write_log(QString("[UNIMAP] Node with ID '%1' not found").arg(id), "Red");
        }
        spdlog::warn("[UNIMAP] Node with ID '{}' not found", id.toStdString());
        return false;
    }
    
    size_t index = it->second;
    if(!is_valid_node_index(index))
    {
        nodes_id_map.erase(it);
        return false;
    }
    
    const NODE& node_to_remove = (*nodes)[index];
    
    // Remove from maps
    remove_node_from_maps(node_to_remove.id, node_to_remove.name);
    
    // Remove from vector (swap with last element for O(1) removal)
    if(index < nodes->size() - 1)
    {
        // Update the index of the last element
        const NODE& last_node = nodes->back();
        update_node_in_maps(last_node, index);
        
        // Swap and pop
        (*nodes)[index] = last_node;
    }
    nodes->pop_back();
    
    // Rebuild maps to fix indices
    rebuild_node_maps();
    
    // Rebuild KD-tree
    kdtree_node.pos.clear();
    kdtree_node.id.clear();
    for(const auto& node : *nodes)
    {
        if(node.type == "ROUTE" || node.type == "GOAL" || node.type == "INIT" || node.type == "STATION")
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
    
    if(logger)
    {
        logger->write_log(QString("[UNIMAP] Removed node: %1").arg(id), "Green");
    }
    spdlog::info("[UNIMAP] Removed node: {}", id.toStdString());
    
    return true;
}

bool UNIMAP::update_node(const QString& id, const NODE& new_node)
{
    if(id.isEmpty() || new_node.id.isEmpty())
    {
        return false;
    }
    
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    auto it = nodes_id_map.find(id);
    if(it == nodes_id_map.end())
    {
        if(logger)
        {
            logger->write_log(QString("[UNIMAP] Node with ID '%1' not found for update").arg(id), "Red");
        }
        spdlog::warn("[UNIMAP] Node with ID '{}' not found for update", id.toStdString());
        return false;
    }
    
    size_t index = it->second;
    if(!is_valid_node_index(index))
    {
        nodes_id_map.erase(it);
        return false;
    }
    
    const NODE& old_node = (*nodes)[index];
    
    // Check name conflict if name changed
    if(!new_node.name.isEmpty() && new_node.name != old_node.name)
    {
        if(nodes_name_map.find(new_node.name) != nodes_name_map.end())
        {
            if(logger)
            {
                logger->write_log(QString("[UNIMAP] Node with name '%1' already exists").arg(new_node.name), "Red");
            }
            spdlog::warn("[UNIMAP] Node with name '{}' already exists", new_node.name.toStdString());
            return false;
        }
    }
    
    // Update node
    (*nodes)[index] = new_node;
    
    // Update maps
    update_node_in_maps(new_node, index);
    
    // Rebuild KD-tree if needed
    bool was_kdtree_node = (old_node.type == "ROUTE" || old_node.type == "GOAL" || old_node.type == "INIT" || old_node.type == "STATION");
    bool is_kdtree_node = (new_node.type == "ROUTE" || new_node.type == "GOAL" || new_node.type == "INIT" || new_node.type == "STATION");
    
    if(was_kdtree_node || is_kdtree_node)
    {
        kdtree_node.pos.clear();
        kdtree_node.id.clear();
        for(const auto& node : *nodes)
        {
            if(node.type == "ROUTE" || node.type == "GOAL" || node.type == "INIT" || node.type == "STATION")
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
    }
    
    if(logger)
    {
        logger->write_log(QString("[UNIMAP] Updated node: %1").arg(id), "Green");
    }
    spdlog::info("[UNIMAP] Updated node: {}", id.toStdString());
    
    return true;
}

bool UNIMAP::node_exists(const QString& id)
{
    if(id.isEmpty())
    {
        return false;
    }
    
    std::shared_lock<std::shared_mutex> lock(mtx);
    auto it = nodes_id_map.find(id);
    if(it != nodes_id_map.end())
    {
        return is_valid_node_index(it->second);
    }
    return false;
}

bool UNIMAP::node_exists_by_name(const QString& name)
{
    if(name.isEmpty())
    {
        return false;
    }
    
    std::shared_lock<std::shared_mutex> lock(mtx);
    auto it = nodes_name_map.find(name);
    if(it != nodes_name_map.end())
    {
        return is_valid_node_index(it->second);
    }
    return false;
}

bool UNIMAP::add_additional_cloud(const std::vector<Eigen::Vector3d>& pts)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    try
    {
        additional_cloud.insert(additional_cloud.end(), pts.begin(), pts.end());
        return true;
    }
    catch(const std::exception& e)
    {
        log_error("[UNIMAP] Failed to add additional cloud: {}", e.what());
        return false;
    }
}

std::vector<Eigen::Vector3d> UNIMAP::get_additional_cloud()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return additional_cloud;
}

void UNIMAP::clear_additional_cloud()
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    additional_cloud.clear();
}

std::vector<Eigen::Vector3d> UNIMAP::get_removed_cloud()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return removed_cloud;
}

void UNIMAP::clear_removed_cloud()
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    removed_cloud.clear();
}

bool UNIMAP::remove_unmatched_points(const std::vector<Eigen::Vector3d>& scan_pts, const Eigen::Matrix4d& cur_tf)
{
    std::unique_lock<std::shared_mutex> lock(mtx);
    
    if(!kdtree_cloud_2d || kdtree_cloud_2d->pts.empty())
    {
        spdlog::warn("[UNIMAP] No existing map to process");
        return false;
    }

    if(scan_pts.empty())
    {
        spdlog::warn("[UNIMAP] No scan points provided");
        return false;
    }

    const double match_threshold = 0.15;  // default 0.15m

    // Calculate update radius from actual scan data
    Eigen::Vector3d robot_pos = cur_tf.block(0,3,3,1);
    
    const double robot_radius = CONFIG::instance()->get_robot_radius();
    const double update_radius = robot_radius * 2.5;  

    log_info("[UNIMAP] Update radius set to robot radius x2.5: {:.2f}m", update_radius);

    // Clear previous removed points
    //removed_cloud.clear();
    
    // Build KD-tree for scan points using XYZR_CLOUD
    XYZR_CLOUD scan_cloud;
    scan_cloud.pts.reserve(scan_pts.size());
    for(const auto& pt : scan_pts)
    {
        PT_XYZR scan_pt;
        scan_pt.x = pt.x();
        scan_pt.y = pt.y();
        scan_pt.z = pt.z();
        scan_pt.r = 128.0;  // temp reflectance value
        scan_cloud.pts.push_back(scan_pt);
    }
    
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, XYZR_CLOUD>,
        XYZR_CLOUD,
        3> SCAN_KD_TREE;
    
    SCAN_KD_TREE scan_tree(3, scan_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    scan_tree.buildIndex();
    
    // Process map points
    size_t removed_count = 0;
    size_t kept_count = 0;
    std::vector<PT_XYZR> updated_map_pts;
    
    for(size_t i = 0; i < kdtree_cloud_2d->pts.size(); i++)
    {
        PT_XYZR& map_pt = kdtree_cloud_2d->pts[i];
        Eigen::Vector3d map_pos(map_pt.x, map_pt.y, map_pt.z);
        
        // range limit
        double dist_to_robot = (map_pos - robot_pos).norm();
        if(dist_to_robot > update_radius)
        {
            updated_map_pts.push_back(map_pt);
            kept_count++;
            continue;
        }
        
        // check matching in scan points
        double query_pt[3] = {map_pt.x, map_pt.y, map_pt.z};
        std::vector<unsigned int> ret_index(1);
        std::vector<double> out_dist_sqr(1);
        
        scan_tree.knnSearch(&query_pt[0], 1, &ret_index[0], &out_dist_sqr[0]);
        
        double match_dist = std::sqrt(out_dist_sqr[0]);
        
        // If matched, keep; if not matched, remove
        if(match_dist < match_threshold)
        {
            updated_map_pts.push_back(map_pt);
            kept_count++;
        }
        else
        {
            // If not matched, add to removed_cloud (for visualization in purple)
            removed_cloud.push_back(map_pos);
            removed_count++;
        }
    }
    
    // Replace map with updated points
    kdtree_cloud_2d->pts = updated_map_pts;
    
    // Rebuild KD-tree
    if(kdtree_cloud_2d_index)
    {
        kdtree_cloud_2d_index.reset();
    }
    kdtree_cloud_2d_index = std::make_shared<KD_TREE_XYZR>(3, *kdtree_cloud_2d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kdtree_cloud_2d_index->buildIndex();
    
    spdlog::info("[UNIMAP] Points processed - Kept: {}, Removed: {}, Total remaining: {}", 
                 kept_count, removed_count, kdtree_cloud_2d->pts.size());
    
    return removed_count > 0;  // Return true if any points were removed
}
