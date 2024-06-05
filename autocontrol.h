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
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
    GLOBAL_PATH get_cur_global_path();
    std::vector<Eigen::Matrix4d> get_cur_local_path();

    void stop();
    void move_pp(Eigen::Matrix4d goal_tf, int preset);
    void move_hpp(Eigen::Matrix4d goal_tf, int preset);
    void move_tng(Eigen::Matrix4d goal_tf, int preset);

    // global path planning (using topo)
    GLOBAL_PATH calc_global_path(Eigen::Matrix4d goal_tf, int type = 0); // type 0: smooth path, type 1: no divide path, type 2: no topo path
    std::vector<QString> topo_path_finding(QString goal_node_id);
    std::vector<Eigen::Vector3d> path_dividing(std::vector<Eigen::Vector3d> src, double step);
    std::vector<Eigen::Vector3d> path_ccma(std::vector<Eigen::Vector3d> src);
    std::vector<double> calc_ref_v(std::vector<Eigen::Vector3d> src);
    std::vector<double> smoothing_v(std::vector<double> src);

    // local path planning (using obs_map)
    std::vector<Eigen::Matrix4d> calc_tactile(double vx, double vy, double wz, double dt, double predict_t, Eigen::Matrix4d G0);
    std::vector<Eigen::Matrix4d> calc_local_path(Eigen::Vector3d st_pos, Eigen::Vector3d ed_pos);


    // for control
    bool is_everything_fine();
    std::pair<int, Eigen::Vector3d> get_nn_pos(GLOBAL_PATH& path, Eigen::Vector3d cur_pos);
    Eigen::Vector3d get_tgt(GLOBAL_PATH& path, double dist, Eigen::Vector3d nn_pos, int nn_idx);

    // control loop
    std::atomic<bool> fsm_flag = {false};
    std::thread *fsm_thread = NULL;

    void fsm_loop_pp();
    void fsm_loop_hpp();
    void fsm_loop_tng();

    // storage
    GLOBAL_PATH cur_global_path;
    std::vector<Eigen::Matrix4d> cur_local_path;

    Eigen::Vector3d last_nn_pos;
    Eigen::Vector3d last_pp_tgt;
    Eigen::Vector3d last_local_tgt;

    // flags
    std::atomic<bool> is_moving = {false};

    // for ompl    
    cv::Mat obs_map;
    Eigen::Matrix4d obs_tf;
    cv::Mat avoid_area;

public:
    bool isStateValid(const ob::State *state) const;    

Q_SIGNALS:

};

#endif // AUTOCONTROL_H
