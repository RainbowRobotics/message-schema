#ifndef DOCKINGCONTROL_H
#define DOCKINGCONTROL_H

// global headers
#include "global_defines.h"
#include "my_utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "bqr_sensor.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

#include <QObject>

typedef ompl::base::SE2StateSpace::StateType State;

class DOCKCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit DOCKCONTROL(QObject *parent = nullptr);
    ~DOCKCONTROL();
    std::mutex mtx;

    void init();

    // params
    DCTRL_PARAM params;
    DCTRL_PARAM load_preset(int preset);

    void set_node_type(QString type);
    void set_dock_offset(Eigen::Vector3d offset_val);

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    BQR_SENSOR *bqr = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;

//    void move(double p_gain=1.0, double d_gain=1.0, double off_x=0.0, double off_y=0.0, double off_t=0.0);
    void move(double p_gain, double d_gain, double off_x, double off_y, double off_t);
    void move();
    void stop();
    void a_loop();
    void b_loop();
    bool undock();

    // control loop
    std::atomic<bool> a_flag = {false};
    std::atomic<bool> b_flag = {false};

    std::thread *a_thread = NULL;
    std::thread *b_thread = NULL;



    bool is_everything_fine();

    double p_gain_ratio = 1.0;
    double d_gain_ratio = 1.0;

    const double station_linear_move_d = 0.1;
    const double station_linear_move_v = 0.008;

    double ox = 0.;
    double oy = 0.;
    double ot = 0.;

    QString docking_node_type = "GOAL";

    // flags
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};
    std::atomic<int> fsm_state = {DOCKING_FSM_OFF};
    QString obs_condition = "none";
    QString failed_reason = "";

    // for l_cluster
    std::queue<std::vector<Eigen::Vector3d>> clusters_queue;
    int clust_size_min = 60;
    int clust_size_max = 500;
    float clust_d_threshold = 0.05;
    bool sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out);

    // for l_icp
    KFRAME generateVKframe();
    XYZR_CLOUD generateSamplePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n);
    Eigen::Vector3d calculateCenter(const KFRAME& frame);
    Eigen::Matrix4d calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    double Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    int ICP_MAX_FEATURE_NUM = 1000;
    double ICP_COST_THRESHOLD = 3.0;
    int ICP_CORRESPONDENCE_THRESHOLD = 10;

    // for l_dock
    bool path_flag = false;
    bool undock_flag = false;
    double undock_waiting_time = 0.0;
    Eigen::Matrix4d docking_station;
    Eigen::Matrix4d docking_station_m;
    KFRAME Vfrm;
    Eigen::Vector3d frm1_center;
    bool find_Vmark();
    bool dock = false;

    //for l_control
    double limit_accel = 0.1;
    double limit_vel = 0.1;
    double limit_th_acc = 45;
    double limit_th = 0.785;
    double err_th_old = 0.0;
    double err_v_old = 0.0;
    double undock_time =0.0;
    void dockControl(const Eigen::Matrix4d& cur_pos, double& cmd_v , double& cmd_w);
    bool dock_first = false;

    // for debug
    std::vector<Eigen::Vector3d> get_cur_clust();
    std::vector<Eigen::Vector3d> debug_frame;


    // for hybrid a*
    std::vector<Eigen::Matrix4d> generateStraightPathDock(const Eigen::Matrix4d& dock_tf , double step , double length);
    double updateH(const Eigen::Matrix4d st, const Eigen::Matrix4d ed);
    double gs = 0.05;
    std::vector<Eigen::Matrix4d> hybrid_dubins(const Eigen::Matrix4d &, const Eigen::Matrix4d &);
    std::vector<Eigen::Matrix4d> runAstar(const Eigen::Matrix4d & , const Eigen::Matrix4d &);
    cv::Vec2i xy_uv(double,double);
    cv::Vec2d uv_xy(int,int);
    cv::Mat calc_flowfield(const cv::Mat& map, cv::Vec2i ed);
    int dock_mapsize = 10; // m
    int w = 200;
    int h = 200;
    int cx = 100;
    int cy = 100;

    Eigen::Matrix4d odom_start_tf;


Q_SIGNALS:

    void signal_dock_response(DATA_DOCK msg);
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

public Q_SLOTS:
    void slot_check_docking();
};

#endif // DOCKINGCONTROL_H
