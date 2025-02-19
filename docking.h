#ifndef DOCKING_H
#define DOCKING_H

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
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

#include <QObject>


class DOCKING : public QObject
{
    Q_OBJECT
public:
    explicit DOCKING(QObject *parent = nullptr);
    ~DOCKING();

    //mtx
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

    std::atomic<bool> is_pause = {false};
    std::atomic<bool> is_moving = {false};

    // for module
    int is_everything_fine();
    void init();
    void stop();
    void move();
    void a_loop();
    void b_loop();
    bool undock();
    std::atomic<bool> a_flag = {false};
    std::atomic<bool> b_flag = {false};

    std::atomic<int> fsm_state = {DOCKING_FSM_OFF};
    std::thread *a_thread = NULL;
    std::thread *b_thread = NULL;
    Eigen::Vector3d last_cur_pos;

    // for flags
    QString obs_condition = "none";
    QString failed_reason = "";

    // for cluster
    std::queue<std::vector<Eigen::Vector3d>> clusters_queue;
    int clust_size_min = 60;
    int clust_size_max = 500;
    float clust_d_threshold = 0.05;
    bool sizefilter(const std::vector<std::vector<Eigen::Vector3d>>& in, std::vector<Eigen::Vector3d>& out);

    // for icp
    KFRAME generateVKframe();
    XYZR_CLOUD generateSamplePoints(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int n);
    Eigen::Vector3d calculateCenter(const KFRAME& frame);
    Eigen::Matrix4d calculateTranslationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
    double Vfrm_icp(KFRAME& frm0, KFRAME& frm1, Eigen::Matrix4d& dG);
    int ICP_MAX_FEATURE_NUM = 1000;
    double ICP_COST_THRESHOLD = 3.0;
    int ICP_CORRESPONDENCE_THRESHOLD = 10;

    // for dock
    bool path_flag = false;
    bool undock_flag = false;
    double undock_waiting_time = 0.0;
    Eigen::Matrix4d docking_station;
    Eigen::Matrix4d docking_station_m;
    KFRAME Vfrm;
    Eigen::Vector3d frm1_center;
    bool find_Vmark();
    bool dock = false;
    //for control
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

Q_SIGNALS:
    void signal_dock_response(DATA_DOCK msg);
};

#endif
