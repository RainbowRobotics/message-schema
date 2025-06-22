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

class UNIMAP : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(UNIMAP)
public:
    // make singleton
    static UNIMAP* instance(QObject* parent = nullptr);

    // load map from path
    void load_map(QString path);

    // clear all map params
    void clear();

    /***********************
     * interface funcs (get)
     ***********************/
    int get_nodes_size();
    bool get_is_loaded();
    NODE* get_node_by_id(QString id);
    NODE* get_node_by_name(QString name);
    QString get_map_path();
    QString get_node_id(Eigen::Vector3d pos);
    QString get_goal_id(Eigen::Vector3d pos);
    QString get_node_id_nn(Eigen::Vector3d pos);
    QString get_node_id_edge(Eigen::Vector3d pos);
    QString get_cur_zone(Eigen::Matrix4d tf);
    std::vector<QString> get_linked_nodes(QString id);
    std::vector<QString> get_nodes(QString type);
    std::vector<QString> get_nodes();
    std::vector<int> get_init_candidates(std::vector<QString> prefix_list);
    std::shared_ptr<XYZR_CLOUD> get_kdtree_cloud();
    std::shared_ptr<std::vector<int>> get_kdtree_mask();
    std::shared_ptr<std::vector<double>> get_map_3d_reflects();
    std::shared_ptr<std::vector<Eigen::Vector3d>> get_map_3d_normal();
    std::shared_ptr<std::vector<Eigen::Vector3d>> get_map_3d_pts();
    Eigen::Vector3d get_normal_3d(int idx);
    Eigen::Vector3d get_pts_3d(int idx);
    std::shared_ptr<std::vector<NODE>> get_nodes_origin();

    std::shared_ptr<KD_TREE_XYZR> get_kdtree_index();

    void radius_search_kdtree_idx(double query[], double sq_radius, std::vector<nanoflann::ResultItem<unsigned int, double>>& res_idxs, nanoflann::SearchParameters params);
    std::vector<int> knn_search_idx(Eigen::Vector3d center, int k, double radius);

    /***********************
     * interface funcs (set)
     ***********************/
    void set_map_path(QString path);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);

private:
    explicit UNIMAP(QObject *parent = nullptr);
    ~UNIMAP();

    // mutex
    std::mutex mtx;

    // map dir
    QString map_path;

    // interface func
    bool load_2d();
    bool load_3d();
    bool load_topo();

    void save_annotation();
    void set_cloud_mask(Eigen::Vector3d P, double radius, int val);
    void clear_nodes();

    // annotation
    std::shared_ptr<std::vector<NODE>> nodes;

    // code ref
    std::map<QString, cv::Vec2d> ref_codes;

    // tree for map icp
    std::shared_ptr<std::vector<int>> kdtree_mask;
    std::shared_ptr<XYZR_CLOUD> kdtree_cloud;
    std::shared_ptr<KD_TREE_XYZR> kdtree_index;

    // 3D
    CLOUD cloud_3d;
    std::unique_ptr<KD_TREE> tree_3d;

    // 3d pts storage
    std::shared_ptr<std::vector<Eigen::Vector3d>> map_3d_pts;
    std::shared_ptr<std::vector<Eigen::Vector3d>> map_3d_normal;
    std::shared_ptr<std::vector<double>> map_3d_reflects;

    // for fast find neareast
    XYZ_NODE kdtree_node;
    std::unique_ptr<KD_TREE_NODE> kdtree_node_index;
    std::unordered_map<QString, NODE*> nodes_id_map;
    std::unordered_map<QString, NODE*> nodes_name_map;

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
};

#endif // UNIMAP_H
