#ifndef L_DOCKINGCONTROL_H
#define L_DOCKINGCONTROL_H

// global headers
#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

#include <QObject>

//#define lattice_planner

typedef std::pair<float, float> c_point;
typedef std::vector<c_point> c_pointList;


struct Point {
double x, y, theta; 
Point(double _x, double _y) : x(_x), y(_y), theta(0.0) {}
};

struct lattice_primitives{
    Eigen::Matrix4d start_pose;
    Eigen::Matrix4d end_pose;
    double trajectory_radius;
    double v;
    double max_w;
    double accel_w;
    double sample_time;
    double angle_offset;
    double pos_offset;
    bool turn_left;
    std::vector<Eigen::Vector3d> trajectory;
};



class L_DOCKINGCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit L_DOCKINGCONTROL(QObject *parent = nullptr);
    ~L_DOCKINGCONTROL();
    std::mutex mtx;

    // other modules
    CONFIG *config = NULL;
    LOGGER *logger = NULL;
    MOBILE *mobile = NULL;
    LIDAR_2D *lidar = NULL;
    CAM *cam = NULL;
    // CODE_READER *code = NULL;
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;
    OBSMAP *obsmap = NULL;


    enum class DockingState
    {
        FindVMark,
        PointDock,
        CompensationYaw,
        Dock,
        None
    };

    DockingState process= DockingState::None;

    std::vector<Eigen::Vector3d> get_cur_clust();
    void init();
    void stop();
    void move();
    bool find_vmark();

    // module
    int is_everything_fine();
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop();
    Eigen::Vector3d last_cur_pos; //why?

    // flags
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};
    QString obs_condition = "none";


    // for cluster
    c_pointList g_cluster;
    std::vector<c_pointList> g_clusters;
    std::vector<c_pointList> filtered_clusters;
    std::queue<c_pointList> clusters_queue;
    double clust_d_threshold = 0.05; //5cm
    unsigned int clust_size_min = 60;
    unsigned int clust_size_max = 250;

    double clust_dist_threshold_min =0.45;
    double clust_dist_threshold_max =2.0;
    double clust_angle_threshold = 90.*M_PI/180.;

    double DOCK_SIZE_X[2] = {-0.025, 0.025};
    double DOCK_SIZE_Y[2] = {-0.2, 0.2};
    double ROBOT_SIZE_X[2] = {-0.4,0.4};
    double ROBOT_SIZE_Y[2] = {-0.45,0.45};
    double POINTDOCK_MARGIN = 0.15;

    double MAPFIX_DIST = 0.5;
    double DOCKING_STATION_G[2] ={-0.86,0.97};
    int ICP_MAX_FEATURE_NUM = 1000;
    double ICP_COST_THRESHOLD = 3.0;
    int ICP_CORRESPONDENCE_THRESHOLD = 10;
    bool path_flag = false;

    // for icp
    double Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    bool mapfixclust(std::vector<c_pointList>& filtered_c, c_pointList& dock_clusts);
    KFRAME generateVKframe();
    XYZR_CLOUD generateSamplePoints(const c_point& p1, const c_point& p2, int n);
    Eigen::Vector3d calculateCenter(const KFRAME& frame);
    Eigen::Matrix4d calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    Eigen::Matrix4d docking_station;
    Eigen::Matrix4d docking_station_m;
    double odom_dist = 0.0;
    bool first_aline = false;

    //for state lattice
    std::vector<lattice_primitives> make_primitives(const Eigen::Matrix4d& e);
    lattice_primitives critic_primitives(const std::vector<lattice_primitives>& lattics);
    double sampling_dist_threshold = 0.05;
    double sampling_angle_threshold = 10*M_PI/180;
    double sampling_break_dist_scale = 2.0;
    double sampling_w_max_step = 2.5*M_PI/180;
    double sampling_w_max = 90*M_PI/180;
    double sampling_w_accel_step = 10*M_PI/180;
    double sampling_w_accel_max = 45*M_PI/180;
    double L_CRITICS_ANGLE_SCALE = 1.0;
    double L_CRITICS_DIST_SCALE = 3.0;
    double L_CRITICS_MAX_W_SCALE = 0.6;
    double L_CRITICS_ACCEL_SCALE = 0.4;
    std::vector<Eigen::Matrix4d> get_path();
    std::vector<Eigen::Matrix4d> dock_path_;
    Eigen::Matrix4d getLastPathPos(const std::vector<Eigen::Matrix4d>& path);
    Eigen::Matrix4d findLookaheadPointReverse(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& cur_pose, const double& lookahead_dist);
    int findClosestPoint(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& cur_pose);
    int findClosetPointIndex(const std::vector<Eigen::Matrix4d>&, const Eigen::Matrix4d&);
    std::vector<Eigen::Matrix4d> generateSmoothPath(const Eigen::Matrix4d& start, const Eigen::Matrix4d& goal, const double& control_dist);
    inline double normalizeAngle(double angle) 
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }


    //for controller
    double limit_accel = 0.1;
    double limit_vel = 0.1;
    double limit_th_acc = 45;
    double limit_th = 0.785;

    //PID
    double err_th_old = 0.0;
    double err_v_old = 0.0;
    double Kp_th = 0.35;
    double Kd_th = 0.15;
    double Kp_d = 0.15;
    double Kd_d = 0.1;



    //for utils
    double extractdist(const std::vector<c_pointList>& c);
    double distance(const Eigen::Matrix4d&, const Eigen::Matrix4d&);
    double extractTheta(const Eigen::Matrix4d& matrix);




    //for controller
    void dockControl(const std::vector<Eigen::Matrix4d>& path, const Eigen::Matrix4d& cur_pose, double lookahead_dist, double& cmd_v, double& cmd_th);
    void dockControl(const Eigen::Matrix4d& cur_pos, double& cmd_v , double& cmd_w);


    //for debug test
    c_pointList debug_clust_l;
    c_pointList debug_clust_r;
    std::vector<pcl::PointXYZRGB> get_pcl(int r, int g, int b);
    std::vector<c_pointList> line_points;

Q_SIGNALS:
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

};

#endif // L_DOCKINGCONTROL_H
