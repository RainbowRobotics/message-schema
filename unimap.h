#ifndef UNIMAP_H
#define UNIMAP_H

// global defines
#include "global_defines.h"

// other modules
#include "config.h"
#include "logger.h"
#include "utils.h"

#include <QObject>
#include <QDir>

class UNIMAP : public QObject
{
    Q_OBJECT
public:
    explicit UNIMAP(QObject *parent = nullptr);
    ~UNIMAP();

    // mutex
    std::mutex mtx;

    // map dir
    QString map_dir;

    // interface func
    void load_map(QString path);
    void save_map();
    void save_annotation();
    void set_cloud_mask(Eigen::Vector3d P, double radius, int val);

    // topology
    std::vector<QString> get_linked_nodes(QString id);
    std::vector<QString> get_nodes(QString type);    
    QString gen_node_id();    
    QString get_node_id(Eigen::Vector3d pos);
    QString get_goal_id(Eigen::Vector3d pos);
    QString get_node_id_nn(Eigen::Vector3d pos);
    NODE* get_node_by_id(QString id);

    QString add_node(Eigen::Matrix4d tf, QString type);
    void add_node(PICKING pick, QString type, QString info="");
    void edit_node_pos(PICKING pick);
    void edit_node_pos(QString id, Eigen::Matrix4d tf);
    void edit_node_type(PICKING pick, QString type);
    void edit_node_info(PICKING pick, QString info);    
    void clear_nodes();
    void add_link1(PICKING pick);
    void add_link2(PICKING pick);
    void add_link2(QString id0, QString id1);

    // annotation
    std::vector<NODE> nodes;

    // code ref
    std::map<QString, cv::Vec2d> ref_codes;

    // tree for map icp
    std::vector<int> kdtree_mask;
    XYZR_CLOUD kdtree_cloud;
    KD_TREE_XYZR* kdtree_index = NULL;

    // bounding box
    double map_min_x = 99999999;
    double map_max_x = -99999999;
    double map_min_y = 99999999;
    double map_max_y = -99999999;
    double map_min_z = 99999999;
    double map_max_z = -99999999;

    // flags
    std::atomic<bool> is_loaded = {false};    

    // other modules
    CONFIG* config = NULL;
    LOGGER* logger = NULL;
};

#endif // UNIMAP_H
