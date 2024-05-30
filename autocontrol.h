#ifndef AUTOCONTROL_H
#define AUTOCONTROL_H

#include "global_defines.h"
#include "utils.h"

#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "slam_2d.h"
#include "unimap.h"

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
    SLAM_2D *slam = NULL;
    UNIMAP *unimap = NULL;

    // interface funcs
    void move_pp(Eigen::Vector3d goal);
    void move_turn_and_go(Eigen::Vector3d goal);
    void move_holonomic_pp(Eigen::Vector3d goal);

    // global path planning (using topo)


    // local path planning (using obs_map)


    // control loop
    std::atomic<bool> fsm_flag = {false};
    std::thread *fsm_thread = NULL;

    void fsm_loop_pp();
    void fsm_loop_turn_and_go();
    void fsm_loop_holonomic_pp();

Q_SIGNALS:

};

#endif // AUTOCONTROL_H
