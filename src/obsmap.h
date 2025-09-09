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

constexpr double P_OBS = 0.6;
constexpr double P_HIT = 0.9;
constexpr double P_MISS = 0.45;

class OBSMAP : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(OBSMAP)

public:
    // make singleton
    static OBSMAP* instance(QObject* parent = nullptr);

    // start obsmap module
    void init();

    // clear obsmap
    void clear();

    // update obs map (dynamic)
    void update_obs_map(TIME_POSE_PTS& tpp);

    // update obs map (virtual)
    void update_vobs_map();
    void update_vobs_list_robots(const std::vector<Eigen::Vector3d>& vobs_r);
    void update_vobs_list_closures(const std::vector<Eigen::Vector3d>& vobs_c);

    // update obs map (simulation)
    void update_obs_map_sim(Eigen::Matrix4d tf);

    /***********************
     * interface funcs
     ***********************/
    void get_obs_map(cv::Mat& map, Eigen::Matrix4d& tf);
    void get_dyn_map(cv::Mat& map, Eigen::Matrix4d& tf);
    void get_vir_map(cv::Mat& map, Eigen::Matrix4d& tf);
    double get_last_obs_update_time();
    double get_last_vobs_update_time();
    cv::Mat get_obs_map();
    cv::Mat get_dyn_map();
    cv::Mat get_vir_map();
    Eigen::Matrix4d get_map_tf();
    std::vector<Eigen::Vector3d> get_obs_pts();
    std::vector<Eigen::Vector3d> get_dyn_pts();
    std::vector<Eigen::Vector3d> get_vir_pts();
    std::vector<Eigen::Vector3d> get_vir_closure_pts();
    std::vector<Eigen::Vector4d> get_plot_pts(); // x, y, z, prob

    std::vector<Eigen::Vector3d> get_vobs_list_robots();
    std::vector<Eigen::Vector3d> get_vobs_list_closures();

    void set_vobs_list_robots(const std::vector<Eigen::Vector3d>& _vobs_r_list);
    void set_vobs_list_closures(const std::vector<Eigen::Vector3d>& _vobs_c_list);


    std::vector<Eigen::Matrix4d> calc_path(Eigen::Matrix4d st_tf, Eigen::Matrix4d ed_tf);

    // for plot
    void draw_robot(cv::Mat& img);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);

    /***********************
     * check collision
     ***********************/
    int is_tf_collision(const Eigen::Matrix4d& robot_tf, bool is_dyn = false, double margin_x = 0, double margin_y = 0);
    int is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, bool is_dyn = false, double margin_x = 0, double margin_y = 0, int st_idx = 0, int idx_step = 1);

private:
    explicit OBSMAP(QObject *parent = nullptr);
    ~OBSMAP();

    // mutex
    std::shared_mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    UNIMAP* unimap;

    // for avoid path
    double calc_clearance(const cv::Mat& map, const Eigen::Matrix4d& robot_tf, double radius);
    bool is_collision(const cv::Mat& map, const std::vector<Eigen::Matrix4d>& robot_tfs, double margin_x = 0, double margin_y = 0);
    cv::Mat calc_flowfield(const cv::Mat& map, cv::Vec2i ed);

    // octree for obsmap
    std::unique_ptr<octomap::OcTree> octree;
    std::vector<Eigen::Vector3d> obs_pts;
    std::vector<Eigen::Vector3d> dyn_pts;
    std::vector<Eigen::Vector3d> vir_pts;
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

    std::atomic<double> last_obs_update_time = {0.0};
    std::atomic<double> last_vobs_update_time = {0.0};

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

    inline cv::Vec2i xy_uv(double x, double y);
    inline cv::Vec2d uv_xy(int u, int v);

Q_SIGNALS:
    void obs_updated();

};

#endif // OBSMAP_H
