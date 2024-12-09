#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

// global headers
#include "global_defines.h"
#include "my_utils.h"

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
#include "docking.h"

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
    DOCKING *dctrl = NULL;

    // params
    CTRL_PARAM params;
    CTRL_PARAM load_preset(int preset);

    // interface funcs    
    Eigen::Matrix4d get_cur_goal_tf();
    PATH get_cur_global_path();
    PATH get_cur_local_path();
    QString get_obs_condition();    
    QString get_multi_req();    
    void clear_path();

    void init();
    void stop();
    void change();
    void set_goal(QString goal_id);
    void move_pp(Eigen::Matrix4d goal_tf, int preset);
    void move_pp(std::vector<QString> node_path, int preset);
    void clean_up();

    // global path planning (using topo)
    std::vector<QString> remove_duplicates(std::vector<QString> node_path);
    std::vector<std::vector<QString>> symmetric_cut(std::vector<QString> node_path);
    std::vector<std::vector<QString>> loop_cut(std::vector<QString> node_path);
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

    // control loop
    std::atomic<bool> b_flag = {false};
    std::thread *b_thread = NULL;
    void b_loop_pp();

    // storage        
    Eigen::Matrix4d cur_goal_tf;

    PATH cur_global_path;
    PATH cur_local_path;

    Eigen::Vector3d last_cur_pos;
    Eigen::Vector3d last_tgt_pos;
    Eigen::Vector3d last_local_goal;

    tbb::concurrent_queue<PATH> global_path_que;

    // flags
    std::atomic<bool> is_debug = {false};
    std::atomic<bool> is_change = {false};
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};    
    std::atomic<int> fsm_state = {AUTO_FSM_COMPLETE};
    QString obs_condition = "none";

    // flags for multi
    std::atomic<bool> is_multi = {false};
    QString multi_req = "none"; // none, req_path, recv_path

Q_SIGNALS:
    void signal_global_path_updated();
    void signal_local_path_updated();
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

};

#endif // AUTOCONTROL_H
