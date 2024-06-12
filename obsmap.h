#ifndef OBSMAP_H
#define OBSMAP_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "logger.h"

#include <QObject>

class OBSMAP : public QObject
{
    Q_OBJECT
public:
    explicit OBSMAP(QObject *parent = nullptr);
    ~OBSMAP();
    std::mutex mtx;

    CONFIG *config = NULL;
    LOGGER *logger = NULL;

    void init();    
    void update_obs_map(TIME_POSE_PTS& tpp);
    void get_obs_map(cv::Mat& obs_map, Eigen::Matrix4d& obs_tf);

    // for plot
    cv::Mat get_plot_map();
    void draw_robot(cv::Mat& img, Eigen::Matrix4d robot_tf);

    bool is_collision(Eigen::Matrix4d robot_tf);
    bool is_collision(std::vector<Eigen::Matrix4d> robot_tfs);
    bool is_collision(cv::Mat obs_map, Eigen::Matrix4d obs_tf, Eigen::Matrix4d robot_tf, cv::Mat avoid_area);
    bool is_pivot_collision(cv::Mat obs_map, Eigen::Matrix4d obs_tf, Eigen::Matrix4d robot_tf, cv::Mat avoid_area);

    Eigen::Matrix4d tf;
    cv::Mat map;

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

    const double P_hit = 0.55;
    const double P_miss = 0.48;
    const double P_min = 0.1;
    const double P_max = 0.9;
    const double P_wall = 0.6;

    cv::Vec2i xy_uv(double x, double y);
    cv::Vec2d uv_xy(int u, int v);

    double odds(double p);
    double odds_inv(double odd);
    double clamp(double p, double min, double max);
    double prob(double m_old, double P);
    void shift_grid_map(cv::Mat& src, double x, double y, double th);

    std::vector<TIME_POSE_PTS> tpp_storage;

Q_SIGNALS:

};

#endif // OBSMAP_H
