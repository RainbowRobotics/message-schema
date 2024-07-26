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
#include "cam.h"
#include "code_reader.h"
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
    void move(Eigen::Matrix4d goal_tf);

    // control loop
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop(Eigen::Matrix4d goal_tf);

    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);
    bool is_everything_fine();

    Eigen::Vector3d last_cur_pos;

    // flags
    std::atomic<bool> is_moving = {false};
    std::atomic<bool> is_pause = {false};
    std::atomic<int> fsm_state = {DOCKING_FSM_DRIVING};
    QString obs_condition = "none";

Q_SIGNALS:
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

};

#endif // DOCKINGCONTROL_H
