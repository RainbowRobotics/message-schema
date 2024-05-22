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

    // topology
    std::vector<QString> get_nodes(QString type);
    QString gen_node_id();
    QString get_node_id(Eigen::Vector3d pt);
    NODE* get_node_by_id(QString name);
    NODE* get_node_nn(Eigen::Vector3d pose);
    NODE* get_edge_nn(Eigen::Vector3d pose);

    QString add_node(Eigen::Matrix4d tf, QString type, QString info = "");
    void add_node(PICKING pick, QString type, QString info = "");
    void edit_node_pos(PICKING pick);
    void edit_node_pos(QString id, Eigen::Matrix4d tf);
    void edit_node_type(PICKING pick, QString type);
    void edit_node_info(PICKING pick, QString info);
    void save_node_pos(PICKING pick);
    void save_node_pos(QString cur_node);
    void clear_nodes();
    void add_link1(PICKING pick);
    void add_link2(PICKING pick);
    void add_link2(QString id0, QString id1);

    QJsonArray pose_to_array(Eigen::Vector3d pose);
    Eigen::Vector3d array_to_pose(QJsonArray arr);
    QJsonArray links_to_array(std::vector<QString> links);
    std::vector<QString> array_to_links(QJsonArray arr);

    // annotation
    std::vector<NODE> nodes;

    // tree for map icp
    XYZR_CLOUD kdtree_cloud;
    KD_TREE_XYZR* kdtree_index = NULL;

    // flags
    std::atomic<bool> is_loaded = {false};    

    // other modules
    CONFIG* config = NULL;
    LOGGER* logger = NULL;
};

#endif // UNIMAP_H
