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
#include "localization.h"
#include "unimap.h"
#include "obsmap.h"

// qt
#include <QObject>

/*
 * @class AUTOCONTORL
 * @brief robot path finding, path following control
 */

class AUTOCONTROL : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(AUTOCONTROL)

public:
    // make singleton
    static AUTOCONTROL* instance(QObject* parent = nullptr);

    // init control module
    void init();

    // stop control loop
    void stop();

    /***********************
     * interface funcs
     ***********************/
    int get_fsm_state();                    // get current Finit State Machine state
    bool get_is_debug();                    // check if debug
    bool get_is_pause();                    // check if paused
    bool get_is_moving();                   // check if moving
    bool get_multi_inter_lock();            // (if emo push->released:[true] new move cmd come:[false]) to keep it in "not ready" state until a new move command comes.
    PATH get_cur_global_path();             // get current global path
    PATH get_cur_local_path();              // get current local path
    double get_process_time_control();      // get control loop processing time
    double get_process_time_obs();          // get obs loop processing time
    QString get_multi_reqest_state();       // get current multi request state (none, req_path, recv_path)
    QString get_obs_condition();            // get current obstacle condition (none, near, far, vir)
    double get_obs_dist();                 // get current obstacle far dist (1m, 2m)
    std::vector<Eigen::Matrix4d> get_obs_traj();
    QString get_cur_move_state();           // get current move state (none, move, complete, fail, obstacle, cancel)
    DATA_MOVE get_cur_move_info();          // get last received move msg
    Eigen::Vector3d get_last_cur_pos();     // get last current pos
    Eigen::Vector3d get_last_tgt_pos();     // get last target pos
    Eigen::Vector3d get_last_local_goal();  // get last local goal

    void set_is_rrs(bool flag);
    void set_is_pause(bool val);
    void set_is_debug(bool val);
    void set_is_moving(bool val);
    void set_multi_req(QString str);
    void set_obs_condition(QString str);
    void set_cur_goal_state(QString str);
    void set_multi_inter_lock(bool val);

    // extract the path from the cur_tf to predict_t seconds with the cur_vel at the resolution of dt.
    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d _cur_tf);

    // [single robot] calc global path
    PATH calc_global_path(Eigen::Matrix4d goal);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_localization_module(LOCALIZATION* _localization);

public Q_SLOTS:
    // slot func move(receive goal) (start control loop)
    void slot_move(DATA_MOVE msg);

    // slot func move(receive path) (start control loop)
    void slot_path(DATA_PATH msg);

private:
    explicit AUTOCONTROL(QObject *parent = nullptr);
    ~AUTOCONTROL();

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    UNIMAP* unimap;
    OBSMAP* obsmap;
    LOCALIZATION* loc;

    // control params
    CTRL_PARAM params;

    // get control params from /data/slamnav2/[robot_type]/config/preset.json
    CTRL_PARAM load_preset(int preset);

    // [single robot] move (input param: goal transform matrix)
    void move(Eigen::Matrix4d goal_tf, int preset);

    // [multi robot] move (input param: node path)
    void move(std::vector<QString> node_path, int preset);

    // global path, local path
    void clear_path();

    /***********************
     * global path planning
     ***********************/
    // split multi robot node path, symmetric (1-2-3-2-1 -> 1-2-3, 3-2-1)
    std::vector<std::vector<QString>> symmetric_cut(std::vector<QString> node_path);

    // split multi robot node path, loop (1-2-3-1-2 -> 1-2-3, 1-2)
    std::vector<std::vector<QString>> loop_cut(std::vector<QString> node_path);

    // path upsampling with step variables (input: pos vector)
    std::vector<Eigen::Vector3d> path_resampling(const std::vector<Eigen::Vector3d>& src, double step);

    // path upsampling with step variables (input: tf vector)
    std::vector<Eigen::Matrix4d> path_resampling(const std::vector<Eigen::Matrix4d>& src, double step);

    // path resampling with ccma algorithm
    std::vector<Eigen::Vector3d> path_ccma(const std::vector<Eigen::Vector3d>& src);

    // calc node path
    std::vector<QString> calc_node_path(QString st_node_id, QString ed_node_id);

    // delete duplicate nodes in the node path
    std::vector<QString> remove_duplicates(std::vector<QString> node_path);

    // find the tf closest to the current position among the line segments formed by nodes and nodes
    Eigen::Matrix4d get_approach_pose(Eigen::Matrix4d tf0, Eigen::Matrix4d tf1, Eigen::Matrix4d cur_tf);

    // [single robot] calc global path
    //PATH calc_global_path(Eigen::Matrix4d goal);

    // [multi robot] calc global path (node path -> global path)
    PATH calc_global_path(std::vector<QString> node_path, bool add_cur_tf);

    void calc_ref_v(const std::vector<Eigen::Matrix4d>& src, std::vector<double>& ref_v, double st_v, double step);
    std::vector<double> smoothing_v(const std::vector<double>& src, double path_step);

    /***********************
     * local path planning
     ***********************/
    // calculate paths with higher resolution (local path)
    PATH calc_local_path(PATH& global_path);
    PATH calc_local_path_with_cur_vel(PATH& global_path);

    // calculate paths with higher resolution (avoid path)
    PATH calc_avoid_path(PATH& global_path);

    // check which index of the path the current location is at
    int get_nn_idx(std::vector<Eigen::Vector3d>& path, Eigen::Vector3d cur_pos);

    // check if controllability
    int is_everything_fine();

    // control loop
    std::atomic<bool> control_flag = {false};
    std::unique_ptr<std::thread> control_thread;
    void control_loop();

    // obs loop
    std::atomic<bool> obs_flag = {false};
    std::unique_ptr<std::thread> obs_thread;
    void obs_loop();

    // for plot
    Eigen::Vector3d last_cur_pos    = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d last_tgt_pos    = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d last_local_goal = Eigen::Vector3d(0,0,0);

    tbb::concurrent_queue<PATH> global_path_que;

    // flags
    std::atomic<bool> is_path_overlap  = {false};
    std::atomic<bool> is_moving        = {false};
    std::atomic<bool> is_debug         = {false};
    std::atomic<bool> is_pause         = {false};
    std::atomic<bool> is_rrs           = {false};
    std::atomic<int>  fsm_state        = {AUTO_FSM_COMPLETE};
    std::atomic<bool> multi_inter_lock = {false};

    std::atomic<double> process_time_control = {0.0};
    std::atomic<double> process_time_obs = {0.0};

    // params for rrs & plot
    DATA_MOVE cur_move_info;
    QString cur_multi_req     = "none"; // none, req_path, recv_path
    QString cur_obs_condition = "none"; // none, near, far, vir
    QString cur_move_state    = "none"; // none, move, complete, fail, obstacle, cancel
    PATH cur_global_path;
    PATH cur_local_path;

    // obs
    double cur_obs_decel_v = 0.0;
    int cur_obs_value = OBS_NONE;
    std::mutex mtx_obs_decel;

    // driving local ref v oscilation prevent
    Eigen::Vector3d cur_pos_at_start_driving = Eigen::Vector3d(0,0,0);
    int prev_local_ref_v_index = 0;
    bool ref_v_oscilation_end_flag = false;

    // debug
    double cur_obs_dist = 9999.0;
    std::vector<Eigen::Matrix4d> obs_traj;

    // for obs distance
    double max_check_range = 2.5;

Q_SIGNALS:
    void signal_move(DATA_MOVE msg);
    void signal_path(DATA_PATH msg);
    void signal_move_response(DATA_MOVE msg);
    void signal_global_path_updated();
    void signal_local_path_updated();

};

#endif // AUTOCONTROL_H
