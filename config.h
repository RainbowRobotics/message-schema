#ifndef CONFIG_H
#define CONFIG_H

#include "global_defines.h"

#include <QObject>

class CONFIG : public QObject
{
    Q_OBJECT
public:
    explicit CONFIG(QObject *parent = nullptr);

public:
    // unit : meter, degree, second
    // params (initial value ref from AMR200)
    double ROBOT_SIZE_X[2] = {-0.5, 0.5}; // min, max
    double ROBOT_SIZE_Y[2] = {-0.4, 0.4};
    double ROBOT_SIZE_Z[2] = {0.0, 0.3};
    double ROBOT_WHEEL_BASE = 0.467;
    double ROBOT_WHEEL_RADIUS = 0.075;
    double ROBOT_RADIUS = 0.3;

    int MOTOR_ID_L = 1;
    int MOTOR_ID_R = 0;
    double MOTOR_DIR = 1.0;
    double MOTOR_GEAR_RATIO = 11.0;
    double MOTOR_LIMIT_V = 2.0;
    double MOTOR_LIMIT_V_ACC = 1.5;
    double MOTOR_LIMIT_W = 180.0;
    double MOTOR_LIMIT_W_ACC = 360.0;
    double MOTOR_GAIN_KP = 4400.0;
    double MOTOR_GAIN_KI = 0.0;
    double MOTOR_GAIN_KD = 100.0;

    double LIDAR_MAX_RANGE = 40.0;
    QString LIDAR_TF_F = "";
    QString LIDAR_TF_B = "";

    int SLAM_WINDOW_SIZE = 100;
    double SLAM_VOXEL_SIZE = 0.05;

    double SLAM_ICP_COST_THRESHOLD = 0.5;
    double SLAM_ICP_ERROR_THRESHOLD = 0.2;
    int SLAM_ICP_MAX_FEATURE_NUM = 1000;
    int SLAM_ICP_DO_ERASE_GAP = 10;
    int SLAM_ICP_DO_ACCUM_NUM = 3;
    double SLAM_ICP_VIEW_THRESHOLD = 150.0;

    int SLAM_KFRM_UPDATE_NUM = 50;
    double SLAM_KFRM_LC_TRY_DIST = 2.0;
    double SLAM_KFRM_LC_TRY_OVERLAP = 0.5;

    double LOC_ICP_COST_THRESHOLD = 2.0;
    double LOC_ICP_ERROR_THRESHOLD = 0.2;
    int LOC_ICP_MAX_FEATURE_NUM = 750;

    double LOC_CHECK_DIST = 0.3;
    double LOC_CHECK_IE = 0.2;
    double LOC_CHECK_IR = 0.5;
    double LOC_FUSION_RATIO = 0.5;

    double ANNOT_QA_STEP = 0.3;
    int SIM_MODE = 0;

    double DRIVE_GOAL_D = 0.05;
    double DRIVE_GOAL_TH = 3.0;
    double DRIVE_EXTENDED_CONTROL_TIME = 2.0;

    double OBS_LOCAL_GOAL_D = 3.0;
    double OBS_MAP_GRID_SIZE = 0.05;
    double OBS_MAP_RANGE = 5.0;
    double OBS_MAP_MIN_Z = -1.0;
    double OBS_MAP_MAX_Z = 1.0;    

    // cam
    QString CAM_SERIAL_NUMBER = "";
    QString CAM_TF = "";
    double CAM_HEIGHT_MIN = 0.1;
    double CAM_HEIGHT_MAX = 1.0;

    std::vector<QString> params;

public:
    // interface
    void load();
    QString config_path = "";
    std::atomic<bool> is_load = {false};

Q_SIGNALS:

};

#endif // CONFIG_H
