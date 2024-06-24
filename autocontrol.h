#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

// global headers
#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

// ompl
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/config.h>
#include <ompl/util/Console.h>

// qt
#include <QObject>

class AUTOCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit AUTOCONTROL(QObject *parent = nullptr);
    ~AUTOCONTROL();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;

    // params
    CTRL_PARAM params;
    CTRL_PARAM load_preset(int preset);

    // interface funcs
    PATH get_cur_global_path();
    PATH get_cur_local_path();
    void clear_path();

    void init();
    void stop();
    void move_pp(Eigen::Matrix4d goal_tf, int preset);
    void move_hpp(Eigen::Matrix4d goal_tf, int preset);
    void move_tng(Eigen::Matrix4d goal_tf, int preset);

    // global path planning (using topo)
    PATH calc_global_path(Eigen::Matrix4d goal);
    std::vector<QString> topo_path_finding(QString st_node_id, QString ed_node_id);
    std::vector<Eigen::Vector3d> path_dividing(const std::vector<Eigen::Vector3d>& src, double step);
    std::vector<Eigen::Matrix4d> path_dividing(const std::vector<Eigen::Matrix4d>& src, double step);
    std::vector<Eigen::Vector3d> sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step);
    std::vector<Eigen::Vector3d> path_ccma(const std::vector<Eigen::Vector3d>& src);
    std::vector<double> calc_ref_v(const std::vector<Eigen::Matrix4d>& src, double st_v);
    std::vector<double> smoothing_v(const std::vector<double>& src, double path_step);
    std::vector<double> gaussian_filter(const std::vector<double>& src, int mask, double sigma);

    // for local path planning (using obs_map)
    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);    
    bool is_state_valid(const ompl::base::State *state) const;
    bool is_path_valid(std::vector<Eigen::Matrix4d>& path, bool first_pivot);
    int get_valid_idx(cv::Mat& _obs_map, Eigen::Matrix4d& _obs_tf, cv::Mat& _avoid_area, std::vector<Eigen::Matrix4d>& path, int st_idx);

    // local path loop
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop();

    // for control
    bool is_everything_fine();    
    int get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos);

    // control loop
    std::atomic<bool> b_flag = {false};
    std::thread *b_thread = NULL;
    void b_loop_pp();
    void b_loop_hpp();
    void b_loop_tng();

    // storage    
    PATH cur_global_path;
    PATH cur_local_path;

    Eigen::Vector3d last_cur_pos;
    Eigen::Vector3d last_tgt_pos;
    Eigen::Vector3d last_replan_st;
    Eigen::Vector3d last_local_goal;

    // flags    
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> use_local_path = {false};

    // for ompl    
    cv::Mat obs_map;
    Eigen::Matrix4d obs_tf;
    cv::Mat avoid_area;

Q_SIGNALS:

};

#endif // AUTOCONTROL_H
