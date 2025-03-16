#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

// global headers
#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"
#include "bqr_sensor.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "dockcontrol.h"

// qt
#include <QObject>

class AUTOCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit AUTOCONTROL(QObject *parent = nullptr);
    ~AUTOCONTROL();
    std::recursive_mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    CAM *cam = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;
    DOCKCONTROL *dctrl = NULL;

    // params
    CTRL_PARAM params;
    CTRL_PARAM load_preset(int preset);

    // interface funcs
    PATH get_cur_global_path();
    PATH get_cur_local_path();

    QString get_cur_node_id();

    QString get_multi_req();
    QString get_obs_condition();
    QString get_cur_goal_state();

    void set_multi_req(QString str);
    void set_obs_condition(QString str);   
    void set_cur_goal_state(QString str);

    void init();
    void stop();
    void clear_path();

    /* pp */
    void move_pp(Eigen::Matrix4d goal_tf, int preset);
    void move_pp(std::vector<QString> node_path, int preset);

    /* hpp */
    void move_hpp(Eigen::Matrix4d goal_tf, int val);
    void move_hpp(std::vector<QString> node_path, int val);

    // global path planning (using topo)
    std::vector<QString> remove_duplicates(std::vector<QString> node_path);
    std::vector<std::vector<QString>> symmetric_cut(std::vector<QString> node_path);
    std::vector<std::vector<QString>> loop_cut(std::vector<QString> node_path);
    Eigen::Matrix4d get_approach_pose(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, Eigen::Matrix4d cur_tf);
    PATH calc_global_path(Eigen::Matrix4d goal);
    PATH calc_global_path(std::vector<QString> node_path, bool add_cur_tf);
    std::vector<QString> topo_path_finding(QString st_node_id, QString ed_node_id);
    std::vector<Eigen::Vector3d> path_resampling(const std::vector<Eigen::Vector3d>& src, double step);
    std::vector<Eigen::Matrix4d> path_resampling(const std::vector<Eigen::Matrix4d>& src, double step);    
    std::vector<Eigen::Vector3d> sample_and_interpolation(const std::vector<Eigen::Vector3d>& src, double large_step, double small_step, bool use_ccma = true);
    std::vector<Eigen::Vector3d> path_ccma(const std::vector<Eigen::Vector3d>& src);

    void calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step);
    std::vector<double> smoothing_v(const std::vector<double>& src, double path_step);

    // for local path planning (using obs_map)
    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);        
    int get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos);
    PATH calc_local_path(PATH& global_path);
    PATH calc_avoid_path(PATH& global_path);

    // for control
    int is_everything_fine();

    // calc current node
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop();

    // control loop
    std::atomic<bool> b_flag = {false};
    std::thread *b_thread = NULL;
    void b_loop_pp();
    void b_loop_hpp();

    DATA_MOVE move_info;
    QString cur_goal_state = "none"; // "none", "move", "complete", "fail", "obstacle", "cancel"

    // for path plot
    PATH cur_global_path;
    PATH cur_local_path;

    QString last_node_id = "";

    Eigen::Vector3d last_cur_pos;
    Eigen::Vector3d last_tgt_pos;
    Eigen::Vector3d last_local_goal;

    tbb::concurrent_queue<PATH> global_path_que;

    // flags
    std::atomic<bool> is_debug = {false};
    std::atomic<bool> is_path_overlap = {false};
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};    
    std::atomic<bool> is_undock = {false};
    std::atomic<int> fsm_state = {AUTO_FSM_COMPLETE};

    // params for rrs
    std::atomic<bool> is_rrs = {false};
    QString multi_req = "none"; // none, req_path, recv_path
    QString obs_condition = "none";

Q_SIGNALS:
    void signal_move(DATA_MOVE msg);
    void signal_global_path_updated();
    void signal_local_path_updated();
    void signal_move_response(DATA_MOVE msg);
    void signal_check_docking();

public Q_SLOTS:
    void move(DATA_MOVE msg);

};

#endif // AUTOCONTROL_H
