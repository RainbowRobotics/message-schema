#ifndef DOCKINGCONTROL_H
#define DOCKINGCONTROL_H

// global headers
#include "global_defines.h"
#include "utils.h"

// other modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "code_reader.h"
#include "cam.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"

#include <QObject>

class DOCKINGCONTROL : public QObject
{
    Q_OBJECT
public:
    explicit DOCKINGCONTROL(QObject *parent = nullptr);
    ~DOCKINGCONTROL();
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

    void init();
    void stop();
    void move(Eigen::Matrix4d goal_tf, double p_gain=1.0, double d_gain=1.0, double off_x=0.0, double off_y=0.0, double off_t=0.0);

    // control loop
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop(Eigen::Matrix4d goal_tf);

    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);
    bool is_everything_fine();

    Eigen::Vector3d last_cur_pos;

    double p_gain_ratio = 1.0;
    double d_gain_ratio = 1.0;

    double ox = 0.;
    double oy = 0.;
    double ot = 0.;

    // flags
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};
    std::atomic<int> fsm_state = {DOCKING_FSM_COMPLETE};
    QString obs_condition = "none";

Q_SIGNALS:
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

};

#endif // DOCKINGCONTROL_H
