#ifndef DOCKCONTROL_H
#define DOCKCONTROL_H

#include <QObject>

#include "slamnav_navigation_types.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "obsmap.h"
#include "qr_sensor.h"
#include "comm_data.h"

struct DWA_Trajectory
{
    std::vector<Eigen::Matrix4d> poses;
    double v;
    double w;
};

class DOCKCONTROL : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(DOCKCONTROL)
public:
    // make singleton
    static DOCKCONTROL* instance(QObject* parent = nullptr);

    // start dock control module
    void init();

    /***********************
     * interface funcs
     ***********************/
    void move();
    void stop();
    bool undock();
    KFRAME generate_vkframe(int type, bool reverse_flag); // type 0 -oneque, 1 - aline
    Eigen::Vector3d calculate_center(const KFRAME& frame);
    double vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    bool is_everything_fine(); 
    Eigen::Matrix4d find_vmark(int& dock_check);
    XYZR_CLOUD generate_sample_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n);
    Eigen::Matrix4d calculate_translation_matrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to); 
    bool sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out);
    bool cluster_candidate(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out);
    bool get_largest_cluster(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out);
    //DWA
    std::map<std::pair<double, double>, std::vector<Eigen::Matrix4d>> DWA_TABLE;
    std::map<std::pair<double,double>, std::vector<Eigen::Matrix4d>> generate_dwa_traj_table(double min_v, double max_v, double v_step, double min_w_deg, double max_w_deg ,double w_step_deg,double dt ,int steps);
    double wrapToPi(double angle);

    /***********************
     * set other modules
     ***********************/
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_qr_sensor_module(QR_SENSOR* _qr_sensor);
    void set_obsmap_module(OBSMAP* _obsmap);


    //for debug
    std::vector<Eigen::Vector3d> get_cur_clust();
    std::vector<Eigen::Vector3d> debug_frame;
    std::vector<Eigen::Vector3d> get_debug_frame();
    bool get_dock_state();
    bool get_dock_retry_flag();
    int  get_dock_fsm_state();
    void set_dock_retry_flag(bool val);

    void set_cmd_id(QString id);

private:
    explicit DOCKCONTROL(QObject *parent = nullptr);
    ~DOCKCONTROL();


    // dock control loop
    std::atomic<bool> a_flag = {false};
    void a_loop();
    std::unique_ptr<std::thread> a_thread;

    // undock control loop
    std::atomic<bool> b_flag = {false};
    void b_loop();
    std::unique_ptr<std::thread> b_thread;

    QString cmd_id ="";

    // flags
    std::atomic<bool> is_moving     = {false};
    std::atomic<bool> is_pause      = {false};
    std::atomic<int> fsm_state      = {DOCKING_FSM_OFF};
    bool dock = false;
    bool path_flag = false;
    bool oneque_dock = false;
    bool undock_flag = false;
    double dock_waiting_time = 0.0;
    double dock_success_wait_time = 0.0;
    double undock_waiting_time = 0.0;
    bool xnergy_charge_trig = false;
    bool find_check = false;
    bool failed_flag = false;
    bool reverse_mode = false;
    Eigen::Matrix4d docking_station;
    Eigen::Matrix4d docking_station_m;
    Eigen::Matrix4d docking_station_o;
    Eigen::Matrix4d first_aline;
    std::atomic<bool> dock_retry_flag = {false};

    // for cluster
    int check_oneque();
    std::queue<std::vector<Eigen::Vector3d>> clusters_queue;
    int clust_size_min = 60;
    int clust_size_max = 500;
    float clust_d_threshold = 0.01;

    // for icp
    KFRAME vfrm; // vmark frame for icp
    Eigen::Vector3d frm1_center;
    KFRAME oneque_vfrm; // oneque vmark frame for icp
    Eigen::Vector3d oneque_frm1_center;
    int ICP_MAX_FEATURE_NUM = 1000;
    double ICP_COST_THRESHOLD = 3.0;
    int ICP_CORRESPONDENCE_THRESHOLD = 10;
    
    // for control
    void dockControl(bool final_dock, const Eigen::Matrix4d& cur_pos, double& cmd_v , double& cmd_w, double& cmd_v_y);
    double limit_accel = 0.1;
    double limit_vel = 0.1;
    double oneque_limit_vel = 0.5;
    double limit_th_acc = 25;
    double limit_th = 0.78;
    double err_th_old = 0.0;
    double err_v_old = 0.0;
    double dist_x_old = 0.0;
    double dist_y_old = 0.0;
    double undock_time =0.0;

    // params for rrs & plot
    QString obs_condition       = "none";
    QString failed_reason       = "";

    // mutex
    std::mutex mtx;

    // other modules
    CONFIG* config;
    LOGGER* logger;
    MOBILE* mobile;
    LIDAR_2D* lidar_2d;
    QR_SENSOR* qr_sensor;
    OBSMAP* obsmap;

    // add for docking
    std::atomic<bool> watch_flag;
    std::unique_ptr<std::thread> watch_thread;
    void watch_stop();
    void watch_loop();

Q_SIGNALS:
    void signal_dock_response(DATA_DOCK msg);

    // add for docking
public Q_SLOTS:
    void slot_docking_start();
    void slot_undocking_start();
    void slot_docking_stop();

};

#endif // DOCKCONTROL_H
