#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

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

    // flags
    std::atomic<bool> is_moving = {false};

Q_SIGNALS:

};

#endif // AUTOCONTROL_H
