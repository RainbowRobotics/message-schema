#ifndef OBSMAP_H
#define OBSMAP_H

#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "unimap.h"

// qt
#include <QObject>

class OBSMAP : public QObject
{
    Q_OBJECT
public:
    explicit OBSMAP(QObject *parent = nullptr);
    ~OBSMAP();
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP *unimap = NULL;

    void init();    
    void clear();
    void update_obs_map(TIME_POSE_PTS& tpp);
    void update_obs_map_sim(Eigen::Matrix4d tf);
    void update_vobs_map();

    void get_obs_map(cv::Mat& map, Eigen::Matrix4d& tf);
    void get_dyn_map(cv::Mat& map, Eigen::Matrix4d& tf);
    void get_vir_map(cv::Mat& map, Eigen::Matrix4d& tf);    
    std::vector<Eigen::Vector3d> get_obs_pts();
    std::vector<Eigen::Vector3d> get_dyn_pts();
    std::vector<Eigen::Vector3d> get_vir_pts();
    std::vector<Eigen::Vector3d> get_vir_closure_pts();

    // for plot
    void draw_robot(cv::Mat& img);
    std::vector<Eigen::Vector4d> get_plot_pts(); // x, y, z, prob

    // check collision
    int is_tf_collision(const Eigen::Matrix4d& robot_tf, double margin_x = 0, double margin_y = 0);
    int is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x = 0, double margin_y = 0, int st_idx = 0, int idx_step = 1);
    int is_undock_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x, double margin_y, int st_idx, int idx_step);

    int is_tf_collision_dyn(const Eigen::Matrix4d& robot_tf, double margin_x = 0, double margin_y = 0);
    int is_path_collision_dyn(const std::vector<Eigen::Matrix4d>& robot_tfs_dyn, const std::vector<Eigen::Matrix4d>& robot_tfs_vir, double margin_x = 0, double margin_y = 0, int st_idx = 0, int idx_step = 1);
    int is_undock_path_collision_dyn(const std::vector<Eigen::Matrix4d>& robot_tfs_dyn, const std::vector<Eigen::Matrix4d>& robot_tfs_vir, double margin_x = 0, double margin_y = 0, int st_idx = 0, int idx_step = 1);

    // for avoid path
    double calc_clearance(const cv::Mat& map, const Eigen::Matrix4d& robot_tf, double radius);
    bool is_collision(const cv::Mat& map, const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x = 0, double margin_y = 0);
    cv::Mat calc_flowfield(const cv::Mat& map, cv::Vec2i ed);
    std::vector<Eigen::Matrix4d> calc_path(Eigen::Matrix4d st_tf, Eigen::Matrix4d ed_tf);

    // octree for obsmap
    octomap::OcTree* octree = NULL;
    std::vector<Eigen::Vector3d> obs_pts_static;
    std::vector<Eigen::Vector3d> obs_pts_dynamic;
    std::vector<Eigen::Vector3d> obs_pts_virtual;
    std::vector<Eigen::Vector3d> vir_closure_pts;
    std::vector<Eigen::Vector4d> plot_pts;

    // virtual obs for multirobot
    std::vector<Eigen::Vector3d> vobs_list_robots;
    std::vector<Eigen::Vector3d> vobs_list_closures;

    // grid map
    Eigen::Matrix4d map_tf;
    cv::Mat wall_map;
    cv::Mat static_map;
    cv::Mat dynamic_map;
    cv::Mat virtual_map;

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

    const double P_OBS = 0.6;
    const double P_HIT = 0.9;
    const double P_MISS = 0.45;

    cv::Vec2i xy_uv(double x, double y);
    cv::Vec2d uv_xy(int u, int v);

Q_SIGNALS:
    void obs_updated();

};

#endif // OBSMAP_H
