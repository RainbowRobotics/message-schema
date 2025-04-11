#ifndef CONFIG_H
#define CONFIG_H

#include "global_defines.h"

#include <QObject>
#include <QMutex>
#include <QMutexLocker>

class CONFIG : public QObject
{
    Q_OBJECT
public:
    explicit CONFIG(QObject *parent = nullptr);

public:
    QMutex mtx;

    // unit : meter, degree, second
    // params (initial value ref from AMR200)
    QString PLATFORM_NAME = "TT";
    QString PLATFORM_TYPE = "AMR400";

    double ROBOT_SIZE_X[2] = {-0.35, 0.35}; // min, max
    double ROBOT_SIZE_Y[2] = {-0.35, 0.35};
    double ROBOT_SIZE_Z[2] = {0.0, 0.22};
    double ROBOT_WHEEL_BASE = 0.387;
    double ROBOT_WHEEL_RADIUS = 0.0635;
    double ROBOT_RADIUS = 0.5;

    int MOTOR_ID_L = 1;
    int MOTOR_ID_R = 0;
    double MOTOR_DIR = 1.0;
    double MOTOR_GEAR_RATIO = 3.0;
    double MOTOR_LIMIT_V = 2.0;
    double MOTOR_LIMIT_V_ACC = 1.0;
    double MOTOR_LIMIT_W = 180.0;
    double MOTOR_LIMIT_W_ACC = 180.0;
    double MOTOR_GAIN_KP = 4400.0;
    double MOTOR_GAIN_KI = 0.0;
    double MOTOR_GAIN_KD = 100.0;

    double LIDAR_MIN_RANGE = 1.0;
    double LIDAR_MAX_RANGE = 40.0;
    QString LIDAR_TF_F = "0,0,0,0,0,0";
    QString LIDAR_TF_B = "0,0,0,0,0,0";

    int SLAM_WINDOW_SIZE = 100;
    double SLAM_VOXEL_SIZE = 0.05;

    double SLAM_ICP_COST_THRESHOLD = 0.5;
    double SLAM_ICP_ERROR_THRESHOLD = 0.2;
    int SLAM_ICP_MAX_FEATURE_NUM = 1000;
    int SLAM_ICP_DO_ERASE_GAP = 10;
    int SLAM_ICP_DO_ACCUM_NUM = 2;
    double SLAM_ICP_VIEW_THRESHOLD = 170.0;

    int SLAM_KFRM_UPDATE_NUM = 50;
    double SLAM_KFRM_LC_TRY_DIST = 3.0;
    double SLAM_KFRM_LC_TRY_OVERLAP = 0.25;

    double LOC_ICP_COST_THRESHOLD_0 = 1.0;
    double LOC_ICP_COST_THRESHOLD = 0.3;
    double LOC_ICP_ERROR_THRESHOLD = 0.2;
    int LOC_ICP_MAX_FEATURE_NUM = 1000;
    int LOC_SURFEL_NUM = 3;
    double LOC_SURFEL_RANGE = 0.15;

    double LOC_CHECK_DIST = 0.3;
    double LOC_CHECK_IE = 0.2;
    double LOC_CHECK_IR = 0.3;

    double LOC_ICP_ODO_FUSION_RATIO = 0.8;
    double LOC_ARUCO_ODO_FUSION_RATIO = 0.8;
    double LOC_ARUCO_ODO_FUSION_DIST = 2.0;

    double ANNOT_QA_STEP = 0.3;

    int USE_SIM = 0; // 0: real mode, 1: simulation mode
    int USE_IMU = 0;
    int USE_BLIDAR = 0;
    int USE_BQR = 0;
    int USE_BEEP = 0;
    int USE_CAM = 0;
    int USE_RTSP = 0;
    int USE_RRS = 0;
    int USE_FMS = 0;
    int USE_QTUI = 0;
    int USE_ARUCO = 0;
    int USE_EARLYSTOP = 0;
    int USE_LVX = 0;
    int USE_COOP = 0;
    int USE_MULTI = 0;

    double DRIVE_GOAL_APPROACH_GAIN = 1.0;
    double DRIVE_GOAL_D = 0.05;    
    double DRIVE_GOAL_TH = 2.0;
    double DRIVE_EXTENDED_CONTROL_TIME = 1.0;
    double DRIVE_V_DEADZONE = 0.02;
    double DRIVE_W_DEADZONE = 0.1;

    double DOCK_GOAL_D = 0.05;
    double DOCK_GOAL_TH = 2.0;
    double DOCK_EXTENDED_CONTROL_TIME = 10.0;
    double DOCK_UNDOCK_REVERSING_DISTANCE = -0.5;

    int OBS_AVOID = 0; // 0: stop, 1: stop + OMPL, 2: eband + OMPL
    double OBS_DEADZONE_DYN = 0.5; // dynamic obs deadzone
    double OBS_DEADZONE_VIR = 0.5; // virtual obs deadzone
    double OBS_LOCAL_GOAL_D = 4.0;
    double OBS_SAFE_MARGIN_X = 0.1;
    double OBS_SAFE_MARGIN_Y = 0.1;
    double OBS_PATH_MARGIN_X = 0.05;
    double OBS_PATH_MARGIN_Y = 0.0;
    double OBS_MAP_GRID_SIZE = 0.05;
    double OBS_MAP_RANGE = 5.0;
    double OBS_MAP_MIN_V = 0.3;
    double OBS_MAP_MIN_Z = -1.0;
    double OBS_MAP_MAX_Z = 1.0;
    double OBS_PREDICT_TIME = 3.0;

    // cam
    QString CAM_SERIAL_NUMBER_0 = "";
    QString CAM_SERIAL_NUMBER_1 = "";

    QString CAM_TF_0 = "0,0,0,0,0,0"; // x, y, z, rx, ry, rz (euler xyz)
    QString CAM_TF_1 = "0,0,0,0,0,0";

    double CAM_HEIGHT_MIN = 0.1; // for rgbd-cam cloud cropping
    double CAM_HEIGHT_MAX = 1.0;

    // fms
    QString SERVER_IP = "127.0.0.1";
    QString SERVER_ID = "rainbow";
    QString SERVER_PW = "rainbow";

    // docking
    double DOCKING_GOAL_D = 0.05;
    double DOCKING_GOAL_TH = 0.01745; //0.1*D2R;
    double DOCKING_KP_d = 0.15;
    double DOCKING_KD_d = 0.1;
    double DOCKING_KP_th = 0.35;
    double DOCKING_KD_th = 0.15;
    double DOCKING_CLUST_D_THRESHOLD = 0.05;
    double DOCKING_CLUST_DIST_THRESHOLD_MIN = 0.45;
    double DOCKING_CLUST_DIST_THRESHOLD_MAX = 2.0;
    double DOCKING_CLUST_ANGLE_THRESHOLD = 45.0*D2R;
    double DOCKING_DOCK_SIZE_X[2] = {-0.025, 0.025};
    double DOCKING_POINTDOCK_MARGIN = 0.18;
    double DOCKING_ICP_COST_THRESHOLD = 0.5; //3.0;
    double DOCKING_ICP_MAX_FEATURE_NUM = 1000;
    int DOCKING_TYPE = 0; // 0: L_dock 1: FQR_dock 2: bqr
    double DOCKING_EXTENDED_CONTROL_TIME = 10.0;
    double DOCKING_GRID_SIZE = 0.05;
    int DOCKING_MAP_SIZE = 10; //m

    // livox
    QString LVX_TF = "0,0,0,0,0,0";    
    double LVX_FRM_DT = 0.1;
    double LVX_MIN_RANGE = 1.0;
    double LVX_MAX_RANGE = 40.0;
    int LVX_MAX_FEATURE_NUM = 500;
    int LVX_SURFEL_NN_NUM = 1;
    double LVX_SURFEL_RANGE = 1.0;
    double LVX_SURFEL_BALANCE = 0.4; // 0.35 ~ 1.0(off)
    double LVX_COST_THRESHOLD = 1.0;
    double LVX_INLIER_CHECK_DIST = 0.3;

    // map
    QString MAP_PATH = "";

    std::vector<QString> params;

    // bqr code info (only use mecanum old)
    double CODE_A1_X = -27.0; double CODE_A1_Y = -27.0;
    double CODE_A2_X = -27.0; double CODE_A2_Y = -9.0;
    double CODE_A3_X = -27.0; double CODE_A3_Y = 9.0;
    double CODE_A4_X = -27.0; double CODE_A4_Y = 27.0;

    double CODE_B1_X = -9.0;  double CODE_B1_Y = -27.0;
    double CODE_B2_X = -9.0;  double CODE_B2_Y = -9.0;
    double CODE_B3_X = -9.0;  double CODE_B3_Y = 9.0;
    double CODE_B4_X = -9.0;  double CODE_B4_Y = 27.0;

    double CODE_C1_X = 9.0;   double CODE_C1_Y = -27.0;
    double CODE_C2_X = 9.0;   double CODE_C2_Y = -9.0;
    double CODE_C3_X = 9.0;   double CODE_C3_Y = 9.0;
    double CODE_C4_X = 9.0;   double CODE_C4_Y = 27.0;

    double CODE_D1_X = 27.0;  double CODE_D1_Y = -27.0;
    double CODE_D2_X = 27.0;  double CODE_D2_Y = -9.0;
    double CODE_D3_X = 27.0;  double CODE_D3_Y = 9.0;
    double CODE_D4_X = 27.0;  double CODE_D4_Y = 27.0;

public:
    // interface
    void load();
    QString config_path = "";
    QString config_sn_path = "";
    std::atomic<bool> is_load = {false};    

    #if defined(USE_MECANUM_OLD)
    void load_code_info(QString path);
    void load_ext(QString path);
    #endif

    void set_map_path(QString path);

Q_SIGNALS:

};

#endif // CONFIG_H
