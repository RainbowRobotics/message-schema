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

    // for plot
    void draw_robot(cv::Mat& img, Eigen::Matrix4d robot_tf);

    bool is_collision(const Eigen::Matrix4d& robot_tf);
    bool is_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, int st_idx = 0);
    bool is_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const Eigen::Matrix4d& robot_tf, const cv::Mat& avoid_area);
    bool is_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const std::vector<Eigen::Matrix4d>& robot_tfs, const cv::Mat& avoid_area);        
    bool is_pivot_collision(const Eigen::Matrix4d& robot_tf);
    bool is_pivot_collision(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const Eigen::Matrix4d& robot_tf, const cv::Mat& avoid_area);
    bool is_pos_collision(const Eigen::Vector3d& pos, const double& r);

    int get_conflict_idx(const cv::Mat& obs_map, const Eigen::Matrix4d& obs_tf, const std::vector<Eigen::Matrix4d>& robot_tfs, const cv::Mat& avoid_area, const int idx0);
    Eigen::Vector3d get_obs_force(const Eigen::Vector3d& center, const double& max_r);

    // octree for obsmap
    octomap::OcTree* octree = NULL;

    // grid map
    Eigen::Matrix4d tf;
    cv::Mat map;

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

    const double P_hit = 0.7;
    const double P_miss = 0.4;
    const double P_min = 0.2;
    const double P_max = 0.8;
    const double P_wall = 0.6;
    const double hidden_margin = 0.05;

    cv::Vec2i xy_uv(double x, double y);
    cv::Vec2d uv_xy(int u, int v);

    double odds(double p);
    double odds_inv(double odd);
    double clamp(double p, double min, double max);
    double prob(double m_old, double P);

Q_SIGNALS:
    void obs_updated();

};

#endif // OBSMAP_H
