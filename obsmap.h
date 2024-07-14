#ifndef OBSMAP_H
#define OBSMAP_H

#include "global_defines.h"
#include "utils.h"

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
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    UNIMAP *unimap = NULL;

    void init();    
    void clear();
    void update_obs_map(TIME_POSE_PTS& tpp);
    void get_obs_map(cv::Mat& obs_map, Eigen::Matrix4d& obs_tf);
    cv::Mat calc_avoid_area(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& robot_tf0, const Eigen::Matrix4d& robot_tf1, double margin_x = 0, double margin_y = 0);

    // for plot
    void draw_robot(cv::Mat& img, Eigen::Matrix4d robot_tf);

    // check collision
    bool is_pos_collision(const Eigen::Vector3d& pos, double radius);
    bool is_pivot_collision(const Eigen::Matrix4d& robot_tf);
    bool is_tf_collision(const Eigen::Matrix4d& robot_tf, double margin_x = 0, double margin_y = 0);
    bool is_tf_collision_with_area(const Eigen::Matrix4d& robot_tf, const cv::Mat& area, double margin_x = 0, double margin_y = 0);
    bool is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, int st_idx = 0, int idx_step = 1, double margin_x = 0, double margin_y = 0);
    bool get_tf_collision_cnt(const Eigen::Matrix4d& robot_tf, double margin_x, double margin_y, double chk_range, int& cnt0, int& cnt1);

    // calc repulsive force
    Eigen::Vector3d get_obs_force(const Eigen::Vector3d& center, double max_r);

    // octree for obsmap
    octomap::OcTree* octree = NULL;

    // grid map
    Eigen::Matrix4d tf;
    cv::Mat prob_map;
    cv::Mat wall_map;

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

    cv::Vec2i xy_uv(double x, double y);
    cv::Vec2d uv_xy(int u, int v);

Q_SIGNALS:
    void obs_updated();

};

#endif // OBSMAP_H
