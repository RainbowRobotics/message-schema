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

    void move(double p_gain=1.0, double d_gain=1.0, double off_x=0.0, double off_y=0.0, double off_t=0.0);
    void stop();

    // control loop
    std::atomic<bool> a_flag = {false};
    std::thread *a_thread = NULL;
    void a_loop();

    std::vector<Eigen::Matrix4d> calc_trajectory(Eigen::Vector3d cur_vel, double dt, double predict_t, Eigen::Matrix4d G0);
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
    std::atomic<int> fsm_state = {DOCK_FSM_COMPLETE};
    QString obs_condition = "none";

Q_SIGNALS:
    void signal_move_succeed(QString message);
    void signal_move_failed(QString message);

public Q_SLOTS:
    void slot_check_docking();
};

#endif // DOCKINGCONTROL_H
