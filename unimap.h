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

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, boost::no_property, boost::property<boost::edge_weight_t, double>> Graph;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

struct Point
{
    double x;
    double y;
    Point(double x, double y): x(x), y(y) {}
};

class UNIMAP : public QObject
{
    Q_OBJECT
public:
    explicit UNIMAP(QObject *parent = nullptr);
    ~UNIMAP();
    void clear();

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
    std::vector<QString> get_nodes();

    QString gen_node_id();
    QString gen_node_name();

    QString get_node_id(Eigen::Vector3d pos);
    QString get_goal_id(Eigen::Vector3d pos);
    QString get_node_id_nn(Eigen::Vector3d pos);
    QString get_node_id_edge(Eigen::Vector3d pos);
    NODE* get_node_by_id(QString id);
    NODE* get_node_by_name(QString name);

    QString add_node(Eigen::Matrix4d tf, QString type);
    void add_node(PICKING pick, QString type, QString info="");
    void edit_node_pos(PICKING pick);
    void edit_node_pos(QString id, Eigen::Matrix4d tf);
    void edit_node_type(PICKING pick, QString type);
    void edit_node_info(PICKING pick, QString info);    
    void edit_node_name(PICKING pick);
    void clear_nodes();
    void add_link1(PICKING pick);
    void add_link2(PICKING pick);
    void add_link2(QString id0, QString id1);
    void add_link_auto();

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
