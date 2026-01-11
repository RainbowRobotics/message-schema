#ifndef OBSMAP_H
#define OBSMAP_H

#include "slamnav_slam_types.h"
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

// claude
/**
 * @brief Lock-Free 읽기를 위한 맵 데이터 스냅샷 구조체
 *
 * 모든 맵 관련 데이터를 하나의 구조체로 묶어서 atomic하게 교체 가능
 */
struct ObsMapSnapshot
{
    Eigen::Matrix4d map_tf = Eigen::Matrix4d::Identity();
    cv::Mat wall_map;
    cv::Mat static_map;
    cv::Mat dynamic_map;
    cv::Mat virtual_map;

    std::vector<Eigen::Vector3d> obs_pts;
    std::vector<Eigen::Vector3d> dyn_pts;
    std::vector<Eigen::Vector3d> vir_pts;
    std::vector<Eigen::Vector3d> vir_closure_pts;
    std::vector<Eigen::Vector4d> plot_pts;

    ObsMapSnapshot() = default;

    ObsMapSnapshot(int h, int w)
    {
        wall_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
        static_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
        dynamic_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
        virtual_map = cv::Mat(h, w, CV_8U, cv::Scalar(0));
    }
};

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
    cv::Mat get_static_map();
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
    void draw_robot_outline(cv::Mat& img);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_unimap_module(UNIMAP* _unimap);

    /***********************
     * set obs box values
     ***********************/
    void set_obs_box_xy(double min_x, double max_x, double min_y, double max_y);
    void set_obs_box_z(double min_z, double max_z);
    void set_obs_box_range(double range);
    void reset_obs_box();

    /***********************
     * get obs box values
     ***********************/
    double get_obs_box_min_x();
    double get_obs_box_max_x();
    double get_obs_box_min_y();
    double get_obs_box_max_y();
    double get_obs_box_min_z();
    double get_obs_box_max_z();
    double get_obs_box_range();

    /***********************
     * check collision
     ***********************/
    int is_tf_collision(const Eigen::Matrix4d& robot_tf, bool is_dyn = false, double margin_x = 0, double margin_y = 0);
    int is_path_collision(const std::vector<Eigen::Matrix4d>& robot_tfs, bool is_dyn = false, double margin_x = 0, double margin_y = 0, int st_idx = 0, int idx_step = 1);

    cv::Vec2i xy_uv(double x, double y);
    cv::Vec2d uv_xy(int u, int v);

    // claude
    std::shared_ptr<ObsMapSnapshot> get_snapshot();

private:
    explicit OBSMAP(QObject *parent = nullptr);
    ~OBSMAP();

    // mutex
    std::shared_mutex mtx;

    // claude
    std::mutex octree_mtx;

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
    // claude
    // std::vector<Eigen::Vector3d> obs_pts;
    // std::vector<Eigen::Vector3d> dyn_pts;
    // std::vector<Eigen::Vector3d> vir_pts;
    // std::vector<Eigen::Vector3d> vir_closure_pts;
    // std::vector<Eigen::Vector4d> plot_pts;

    // claude
    std::shared_ptr<ObsMapSnapshot> current_snapshot;

    // virtual obs for multirobot
    std::vector<Eigen::Vector3d> vobs_list_robots;
    std::vector<Eigen::Vector3d> vobs_list_closures;

    // claude
    // // grid map
    // Eigen::Matrix4d map_tf;
    // cv::Mat wall_map;
    // cv::Mat static_map;
    // cv::Mat dynamic_map;
    // cv::Mat virtual_map;

    std::atomic<double> last_obs_update_time = {0.0};
    std::atomic<double> last_vobs_update_time = {0.0};

    // obs box
    std::atomic<double> obs_box_min_x = {0.0};
    std::atomic<double> obs_box_max_x = {0.0};
    std::atomic<double> obs_box_min_y = {0.0};
    std::atomic<double> obs_box_max_y = {0.0};
    std::atomic<double> obs_box_min_z = {0.0};
    std::atomic<double> obs_box_max_z = {0.0};
    std::atomic<double> obs_box_range = {0.0};

    int w = 300;
    int h = 300;
    int cx = 150;
    int cy = 150;
    double gs = 0.05;

Q_SIGNALS:
    void obs_updated();

};

#endif // OBSMAP_H
