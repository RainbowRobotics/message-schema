#ifndef UNIMAP_H
#define UNIMAP_H

// global defines
#include "global_defines.h"

// other modules
#include "config.h"
#include "logger.h"
#include "my_utils.h"

#include <QObject>
#include <QDir>

constexpr int knn_search_num = 10;

class UNIMAP : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(UNIMAP)
public:
    // make singleton
    static UNIMAP* instance(QObject* parent = nullptr);

    // load map from path
    void load_map(QString path);
    bool load_node(); // load topo.json

    void save_node();

    // clear all map params
    void clear();

    /***********************
     * interface funcs (get)
     ***********************/
    int get_is_loaded();                                                    // check if map loaded
    NODE* get_node_by_id(QString id);                                       // find node by id
    NODE* get_node_by_name(QString name);                                   // find node by name
    double get_map_min_x();                                                 // get map min x (for bounding box)
    double get_map_max_x();                                                 // get map max x (for bounding box)
    double get_map_min_y();                                                 // get map min y (for bounding box)
    double get_map_max_y();                                                 // get map max y (for bounding box)
    QString get_map_path();                                                 // get current map path
    QString get_goal_id(Eigen::Vector3d pos);                               // get goal node(goal,init,station) id nearest input pos(x,y,th)
    QString get_cur_zone(Eigen::Matrix4d tf);                               // the zone your current location belongs to
    QString get_node_id_edge(Eigen::Vector3d pos);                          // get node id nearest input pos(x,y,th) with edge(link)
    Eigen::Vector3d get_pts_3d(int idx);                                    // get 3d map point
    Eigen::Vector3d get_normal_3d(int idx);                                 // get 3d map normal vector
    std::vector<int> get_init_candidates(std::vector<QString> prefix_list); // get init candidates for semiauto-init
    std::vector<QString> get_nodes(QString type);                           // get specific type nodes
    std::shared_ptr<XYZR_CLOUD> get_kdtree_cloud();                         // get 2d kdtree
    std::shared_ptr<KD_TREE_XYZR> get_kdtree_cloud_index();                 // get 2d kdtree cloud index
    std::shared_ptr<std::vector<double>> get_map_3d_reflects();             // get 3d map reflects
    std::shared_ptr<std::vector<Eigen::Vector3d>> get_map_3d_normal();      // get 3d map normal vectors
    std::shared_ptr<std::vector<Eigen::Vector3d>> get_map_3d_pts();         // get 3d map points
    std::shared_ptr<std::vector<NODE>> get_nodes_origin();                  // get nodes (shared ptr)
    std::vector<LINK_INFO> get_special_links();

    // interface funcs (kdtree)
    void radius_search_kdtree_idx(double query[], double sq_radius,
                                  std::vector<nanoflann::ResultItem<unsigned int, double>>& res_idxs, nanoflann::SearchParameters params);    // try radius search kdtree_cloud_index 2D
    std::vector<int> knn_search_idx(Eigen::Vector3d center, int k, double radius);  // try knn search tree_3d 3D

    // interface funcs (set)
    void set_map_path(QString path);

    // interface funcs (node)
    bool add_node(const NODE& node);
    bool add_node(const Eigen::Matrix4d tf, const QString type);
    bool remove_node(const QString& id);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);

private:
    explicit UNIMAP(QObject *parent = nullptr);
    ~UNIMAP();

    // mutex
    std::shared_mutex mtx;

    // map dir
    QString map_path;

    // load 2d map
    bool load_2d();

    // load 3d map
    bool load_3d();

    // all nodes clear
    void clear_nodes();

    // annotation
    std::shared_ptr<std::vector<NODE>> nodes;
    std::vector<LINK_INFO> special_links;

    // kdtree for map icp (2D)
    std::shared_ptr<XYZR_CLOUD> kdtree_cloud_2d;
    std::shared_ptr<KD_TREE_XYZR> kdtree_cloud_2d_index;

    // kdtree for map icp (3D)
    CLOUD kdtree_cloud_3d;
    std::unique_ptr<KD_TREE> kdtree_cloud_3d_index;

    // 3d pts storage
    std::shared_ptr<std::vector<double>> map_3d_reflects;
    std::shared_ptr<std::vector<Eigen::Vector3d>> map_3d_pts;
    std::shared_ptr<std::vector<Eigen::Vector3d>> map_3d_normal;

    // for fast find node
    XYZ_NODE kdtree_node;
    std::unique_ptr<KD_TREE_NODE> kdtree_node_index;
    std::unordered_map<QString, size_t> nodes_id_map;      // id -> index
    std::unordered_map<QString, size_t> nodes_name_map;    // name -> index

    // additional cloud storage
    std::vector<Eigen::Vector3d> additional_cloud;

    // bounding box
    double map_max_x = std::numeric_limits<double>::min();
    double map_max_y = std::numeric_limits<double>::min();
    double map_max_z = std::numeric_limits<double>::min();
    double map_min_x = std::numeric_limits<double>::max();
    double map_min_y = std::numeric_limits<double>::max();
    double map_min_z = std::numeric_limits<double>::max();

    // flags
    std::atomic<int> is_loaded = {MAP_NOT_LOADED};

    // other modules
    CONFIG* config;
    LOGGER* logger;

    // helper functions for goal search
    bool is_goal_node_type(const QString& node_type);
    double calculate_distance_squared(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2);
    
    // node management functions
    void rebuild_node_maps();
    bool is_valid_node_index(size_t index);
    void add_node_to_maps(const NODE& node, size_t index);
    bool node_exists(const QString& id);
    bool node_exists_by_name(const QString& name);
    bool update_node(const QString& id, const NODE& new_node);
    void update_node_in_maps(const NODE& node, size_t index);
    void remove_node_from_maps(const QString& id, const QString& name);

    QString gen_node_id();
};

#endif // UNIMAP_H
