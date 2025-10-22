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
    }
    
    QString node_path = map_path + "/topo.json";
    QFileInfo node_info(node_path);
    if(!node_info.exists() || !node_info.isFile())
    {
        return false;
    }

    std::vector<LINK_INFO> links;

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
            node.info   = obj["info"].toString();
            node.tf     = string_to_TF(obj["pose"].toString());
            node.linked = array_to_links(obj["links"].toArray());

            // link info
            {
                QJsonArray link_arr = obj["links"].toArray();
                QString last_link = "";
                for(int i = 0; i < link_arr.size(); i++)
                {
                    const QJsonValue &lv = link_arr.at(i);
                    if(lv.isString())
                    {
                        last_link = lv.toString();
                    }
                    else if(lv.isObject())
                    {
                        QJsonObject lo = lv.toObject();
                        if(lo.contains("info"))
                        {
                            QString ed = last_link;
                            if(lo.contains("id") && lo["id"].isString())
                            {
                                ed = lo["id"].toString();
                            }
                            if(!ed.isEmpty())
                            {
                                LINK_INFO link;
                                link.st_id = node.id;
                                link.ed_id = ed;
                                link.info  = lo["info"].toString();

                                // speed parsing
                                if(lo.contains("speed"))
                                {
                                    link.speed  = lo["speed"].toDouble();
                                }

                                link.st.setZero();
                                link.ed.setZero();
                                link.mid.setZero();
                                link.length = 0.0;
                                links.push_back(link);
                            }
                        }
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
            
            // rebuild node maps with indices
            rebuild_node_maps();

            // link info
            special_links.clear();
            special_links.reserve(links.size());
            for(size_t i = 0; i < links.size(); i++)
            {
                LINK_INFO s = links[i];

                Eigen::Vector3d st_pos(0.0, 0.0, 0.0);
                Eigen::Vector3d ed_pos(0.0, 0.0, 0.0);

                auto it_st = nodes_id_map.find(s.st_id);
                auto it_ed = nodes_id_map.find(s.ed_id);
                if(it_st != nodes_id_map.end() && it_ed != nodes_id_map.end())
                {
                    size_t idx_st = it_st->second;
                    size_t idx_ed = it_ed->second;

                    if(is_valid_node_index(idx_st) && is_valid_node_index(idx_ed))
                    {
                        const NODE &nst = (*nodes)[idx_st];
                        const NODE &ned = (*nodes)[idx_ed];

                        st_pos = nst.tf.block(0, 3, 3, 1);
                        ed_pos = ned.tf.block(0, 3, 3, 1);
                    }
                }

                s.st = st_pos;
                s.ed = ed_pos;
                s.mid = 0.5 * (s.st + s.ed);
                s.length = (s.ed - s.st).norm();

                special_links.push_back(s);

                // debug
                {
                    for(size_t i = 0; i < special_links.size(); i++)
                    {
                        const LINK_INFO &s = special_links[i];
                        printf("[UNIMAP][SPECIAL_LINK][%zu]\n", i);
                        printf("  st_id = %s, ed_id = %s, info = %s\n",
                               s.st_id.toStdString().c_str(),
                               s.ed_id.toStdString().c_str(),
                               s.info.toStdString().c_str());
                        if(s.speed > 0.0)
                        {
                            printf(", speed = %f", s.speed);
                        }
                        printf("\n");
                        printf("  length = %.3f\n", s.length);
                        printf("  st  = (%.3f, %.3f, %.3f)\n", s.st.x(), s.st.y(), s.st.z());
                        printf("  ed  = (%.3f, %.3f, %.3f)\n", s.ed.x(), s.ed.y(), s.ed.z());
                        printf("  mid = (%.3f, %.3f, %.3f)\n", s.mid.x(), s.mid.y(), s.mid.z());
                    }
                }
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
    }

    QString node_path = map_path + "/node.json";
    QFileInfo node_info(node_path);
    if(!node_info.exists() || !node_info.isFile())
    {
        return false;
    }

    QFile node_file(node_path);
    if(!node_file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    QByteArray data = node_file.readAll();
    node_file.close();

    QJsonParseError perr;
    QJsonDocument doc = QJsonDocument::fromJson(data, &perr);
    if(perr.error != QJsonParseError::NoError || !doc.isArray())
    {
        return false;
    }

    QJsonArray arr = doc.array();

    // parse nodes
    std::vector<NODE> temp_nodes;
    for(int i = 0; i < arr.size(); i++)
    {
        if(!arr[i].isObject())
        {
            continue;
        }

        QJsonObject obj = arr[i].toObject();

        NODE node;

        // id, name, type
        if(obj.contains("id") && obj["id"].isString())
        {
            node.id = obj["id"].toString();
        }
        if(obj.contains("name") && obj["name"].isString())
        {
            node.name = obj["name"].toString();
        }
        if(obj.contains("type") && obj["type"].isString())
        {
            node.type = obj["type"].toString();
        }

        // subtype
        if(obj.contains("subtype") && obj["subtype"].isObject())
        {
            QJsonObject st = obj["subtype"].toObject();
            if(st.contains("name") && st["name"].isString())
            {
                node.subtype_name = st["name"].toString();
            }
            if(st.contains("index") && st["index"].isString())
            {
                node.subtype_index = st["index"].toString();
            }
        }

        // size - ignore nulls safely
        node.size.setZero();
        if(obj.contains("size") && obj["size"].isObject())
        {
            QJsonObject sz = obj["size"].toObject();
            if(sz.contains("x") && !sz["x"].isNull())
            {
                node.size.x() = sz["x"].toDouble();
            }
            if(sz.contains("y") && !sz["y"].isNull())
            {
                node.size.y() = sz["y"].toDouble();
            }
            if(sz.contains("z") && !sz["z"].isNull())
            {
                node.size.z() = sz["z"].toDouble();
            }
        }

        // info
        if(obj.contains("info") && obj["info"].isObject())
        {
            QJsonObject info_obj = obj["info"].toObject();
            QJsonDocument info_doc(info_obj);
            node.info = QString::fromUtf8(info_doc.toJson(QJsonDocument::Compact));
        }
        else
        {
            node.info = "";
        }

        // links
        node.linked.clear();
        if(obj.contains("links") && obj["links"].isArray())
        {
            QJsonArray link_arr = obj["links"].toArray();
            for(int li = 0; li < link_arr.size(); li++)
            {
                if(!link_arr[li].isObject())
                {
                    continue;
                }
                QJsonObject lo = link_arr[li].toObject();
                if(lo.contains("id") && lo["id"].isString())
                {
                    QString to_id = lo["id"].toString();
                    if(!to_id.isEmpty())
                    {
                        node.linked.push_back(to_id);
                    }
                }
            }
        }

        // pose
        if(obj.contains("pose") && obj["pose"].isObject())
        {
            QJsonObject p = obj["pose"].toObject();

            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double rx = 0.0;
            double ry = 0.0;
            double rz = 0.0;

            if(p.contains("x"))
            {
                x = p["x"].toDouble();
            }
            if(p.contains("y"))
            {
                y = p["y"].toDouble();
            }
            if(p.contains("z"))
            {
                z = p["z"].toDouble();
            }
            if(p.contains("rx"))
            {
                rx = p["rx"].toDouble() * D2R;
            }
            if(p.contains("ry"))
            {
                ry = p["ry"].toDouble() * D2R;
            }
            if(p.contains("rz"))
            {
                rz = p["rz"].toDouble() * D2R;
            }

            node.tf = ZYX_to_TF(x, y, z, rx, ry, rz);
        }
        else
        {
            node.tf = Eigen::Matrix4d::Identity();
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
    return true;

}

void UNIMAP::save_node()
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
            obj["id"] = (*nodes)[p].id;
            obj["name"] = (*nodes)[p].name;
            obj["type"] = (*nodes)[p].type;
            obj["info"] = (*nodes)[p].info;
            obj["pose"] = TF_to_string((*nodes)[p].tf);
            obj["links"] = links_to_array((*nodes)[p].linked);

            arr.append(obj);
        }

        QJsonDocument doc(arr);
        topo_file.write(doc.toJson());
        topo_file.close();

        //printf("[UNIMAP] %s saved\n", topo_path.toLocal8Bit().data());
        spdlog::info("[UNIMAP] {} saved", topo_path.toStdString());
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

std::vector<LINK_INFO> UNIMAP::get_special_links()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return special_links;
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
        for (const QString& id1 : node0->linked)
        {
            NODE* node1 = get_node_by_id(id1);
            if(!node1)
            {
                continue;
            }

            if(node1->type == "ROUTE" || node1->type == "GOAL" || node1->type == "INIT" || node1->type == "STATION")
            {
                Eigen::Vector3d pos1 = node1->tf.block(0,3,3,1);
                double dist = calc_seg_dist(pos0, pos1, pos);
                if (dist < min_dist)
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
        for(const QString& id1 : node0->linked)
        {
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
