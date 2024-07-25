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
#include "cam.h"
#include "code_reader.h"
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
#include <ompl/geometric/planners/prm/PRMstar.h>
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
    CAM *cam = NULL;
    CODE_READER *code = NULL;
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
    QString get_obs_condition();

    void init();
    void stop();
    void move_pp(Eigen::Matrix4d goal_tf, int preset);
    void move_hpp(Eigen::Matrix4d goal_tf, int preset);
    void move_tng(Eigen::Matrix4d goal_tf, int preset);

    // global path planning (using topo)
    PATH calc_global_path(Eigen::Matrix4d goal);
    std::vector<QString> topo_path_finding(QString st_node_id, QString ed_node_id);
    std::vector<Eigen::Vector3d> path_resampling(const std::vector<Eigen::Vector3d>& src, double step);
    std::vector<Eigen::Vector3d> sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step);
    std::vector<Eigen::Vector3d> path_ccma(const std::vector<Eigen::Vector3d>& src);
    void calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_th, std::vector<double>& ref_v, double st_v, double step);
    std::vector<double> smoothing_v(const std::vector<double>& src, double path_step);
    std::vector<double> gaussian_filter(const std::vector<double>& src, int mask, double sigma);
    int get_tgt_idx(const std::vector<Eigen::Vector3d>& path, Eigen::Vector3d pos);

    // for local path planning (using obs_map)
    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);    
    bool is_state_valid(const ompl::base::State *state) const;
    int get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos);
    int get_valid_idx(std::vector<Eigen::Matrix4d>& path, int st_idx);
    Eigen::Vector3d refine_force(Eigen::Vector3d f, Eigen::Vector3d P0, Eigen::Vector3d P1);
    PATH calc_local_path();
    PATH calc_avoid_path();

    // for control
    bool is_everything_fine();

    // control loop
    std::atomic<bool> b_flag = {false};
    std::thread *b_thread = NULL;
    void b_loop_pp(Eigen::Matrix4d goal_tf);    
    void b_loop_hpp(Eigen::Matrix4d goal_tf);
    void b_loop_tng(Eigen::Matrix4d goal_tf);

    // storage    
    PATH cur_global_path;
    PATH cur_local_path;

    Eigen::Vector3d last_cur_pos;
    Eigen::Vector3d last_tgt_pos;
    Eigen::Vector3d last_local_goal;

    // flags    
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};
    std::atomic<int> fsm_state = {AUTO_FSM_COMPLETE};
    QString obs_condition = "none";

    // for ompl
    cv::Mat avoid_area;

Q_SIGNALS:
    void signal_global_path_updated();
    void signal_local_path_updated();
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

};

#endif // AUTOCONTROL_H
