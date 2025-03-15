#include "config.h"

CONFIG::CONFIG(QObject *parent)
    : QObject{parent}
{
}

void CONFIG::load()
{
    // load params
    QFileInfo config_info(config_path);
    QFileInfo config_sn_info(config_sn_path);
    if(config_info.exists() && config_info.isFile() && config_sn_info.exists() && config_sn_info.isFile())
    {
        // read config
        QFile config_file(config_path);
        if(config_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
            QJsonObject obj_robot = obj["robot"].toObject();
            {
                PLATFORM_NAME = obj_robot["PLATFORM_NAME"].toString();
                printf("[CONFIG] PLATFORM_NAME, %s\n", obj_robot["PLATFORM_NAME"].toString().toLocal8Bit().data());

                PLATFORM_TYPE = obj_robot["PLATFORM_TYPE"].toString();
                printf("[CONFIG] PLATFORM_TYPE, %s\n", obj_robot["PLATFORM_TYPE"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_motor = obj["motor"].toObject();
            {
                MOTOR_ID_L = obj_motor["MOTOR_ID_L"].toString().toInt();
                printf("[CONFIG] MOTOR_ID_L, %s\n", obj_motor["MOTOR_ID_L"].toString().toLocal8Bit().data());

                MOTOR_ID_R = obj_motor["MOTOR_ID_R"].toString().toInt();
                printf("[CONFIG] MOTOR_ID_R, %s\n", obj_motor["MOTOR_ID_R"].toString().toLocal8Bit().data());

                MOTOR_DIR = obj_motor["MOTOR_DIR"].toString().toDouble();
                printf("[CONFIG] MOTOR_DIR, %s\n", obj_motor["MOTOR_DIR"].toString().toLocal8Bit().data());

                MOTOR_GEAR_RATIO = obj_motor["MOTOR_GEAR_RATIO"].toString().toDouble();
                printf("[CONFIG] MOTOR_GEAR_RATIO, %s\n", obj_motor["MOTOR_GEAR_RATIO"].toString().toLocal8Bit().data());

                MOTOR_LIMIT_V = obj_motor["MOTOR_LIMIT_V"].toString().toDouble();
                printf("[CONFIG] MOTOR_LIMIT_V, %s\n", obj_motor["MOTOR_LIMIT_V"].toString().toLocal8Bit().data());

                MOTOR_LIMIT_V_ACC = obj_motor["MOTOR_LIMIT_V_ACC"].toString().toDouble();
                printf("[CONFIG] MOTOR_LIMIT_V_ACC, %s\n", obj_motor["MOTOR_LIMIT_V_ACC"].toString().toLocal8Bit().data());

                MOTOR_LIMIT_W = obj_motor["MOTOR_LIMIT_W"].toString().toDouble();
                printf("[CONFIG] MOTOR_LIMIT_W, %s\n", obj_motor["MOTOR_LIMIT_W"].toString().toLocal8Bit().data());

                MOTOR_LIMIT_W_ACC = obj_motor["MOTOR_LIMIT_W_ACC"].toString().toDouble();
                printf("[CONFIG] MOTOR_LIMIT_W_ACC, %s\n", obj_motor["MOTOR_LIMIT_W_ACC"].toString().toLocal8Bit().data());

                MOTOR_GAIN_KP = obj_motor["MOTOR_GAIN_KP"].toString().toDouble();
                printf("[CONFIG] MOTOR_GAIN_KP, %s\n", obj_motor["MOTOR_GAIN_KP"].toString().toLocal8Bit().data());

                MOTOR_GAIN_KI = obj_motor["MOTOR_GAIN_KI"].toString().toDouble();
                printf("[CONFIG] MOTOR_GAIN_KI, %s\n", obj_motor["MOTOR_GAIN_KI"].toString().toLocal8Bit().data());

                MOTOR_GAIN_KD = obj_motor["MOTOR_GAIN_KD"].toString().toDouble();
                printf("[CONFIG] MOTOR_GAIN_KD, %s\n", obj_motor["MOTOR_GAIN_KD"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_default = obj["default"].toObject();
            {
                ROBOT_SIZE_X[0] = obj_default["ROBOT_SIZE_MIN_X"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MIN_X, %s\n", obj_default["ROBOT_SIZE_MIN_X"].toString().toLocal8Bit().data());

                ROBOT_SIZE_X[1] = obj_default["ROBOT_SIZE_MAX_X"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MAX_X, %s\n", obj_default["ROBOT_SIZE_MAX_X"].toString().toLocal8Bit().data());

                ROBOT_SIZE_Y[0] = obj_default["ROBOT_SIZE_MIN_Y"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MIN_Y, %s\n", obj_default["ROBOT_SIZE_MIN_Y"].toString().toLocal8Bit().data());

                ROBOT_SIZE_Y[1] = obj_default["ROBOT_SIZE_MAX_Y"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MAX_Y, %s\n", obj_default["ROBOT_SIZE_MAX_Y"].toString().toLocal8Bit().data());

                ROBOT_SIZE_Z[0] = obj_default["ROBOT_SIZE_MIN_Z"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MIN_Z, %s\n", obj_default["ROBOT_SIZE_MIN_Z"].toString().toLocal8Bit().data());

                ROBOT_SIZE_Z[1] = obj_default["ROBOT_SIZE_MAX_Z"].toString().toDouble();
                printf("[CONFIG] ROBOT_SIZE_MAX_Z, %s\n", obj_default["ROBOT_SIZE_MAX_Z"].toString().toLocal8Bit().data());

                // robot radius calculation
                double lx = std::max<double>(std::abs(ROBOT_SIZE_X[0]), std::abs(ROBOT_SIZE_X[1]));
                double ly = std::max<double>(std::abs(ROBOT_SIZE_Y[0]), std::abs(ROBOT_SIZE_Y[1]));
                ROBOT_RADIUS = std::sqrt(lx*lx + ly*ly);
                printf("[CONFIG] ROBOT_RADIUS(auto calc), %.3f\n", ROBOT_RADIUS);

                LIDAR_MIN_RANGE = obj_default["LIDAR_MIN_RANGE"].toString().toDouble();
                printf("[CONFIG] LIDAR_MIN_RANGE, %s\n", obj_default["LIDAR_MIN_RANGE"].toString().toLocal8Bit().data());

                LIDAR_MAX_RANGE = obj_default["LIDAR_MAX_RANGE"].toString().toDouble();
                printf("[CONFIG] LIDAR_MAX_RANGE, %s\n", obj_default["LIDAR_MAX_RANGE"].toString().toLocal8Bit().data());

                LIDAR_TF_F = obj_default["LIDAR_TF_F"].toString();
                printf("[CONFIG] LIDAR_TF_F, %s\n", obj_default["LIDAR_TF_F"].toString().toLocal8Bit().data());

                LIDAR_TF_B = obj_default["LIDAR_TF_B"].toString();
                printf("[CONFIG] LIDAR_TF_B, %s\n", obj_default["LIDAR_TF_B"].toString().toLocal8Bit().data());

                ROBOT_WHEEL_RADIUS = obj_default["ROBOT_WHEEL_RADIUS"].toString().toDouble();
                printf("[CONFIG] ROBOT_WHEEL_RADIUS, %s\n", obj_default["ROBOT_WHEEL_RADIUS"].toString().toLocal8Bit().data());

                ROBOT_WHEEL_BASE = obj_default["ROBOT_WHEEL_BASE"].toString().toDouble();
                printf("[CONFIG] ROBOT_WHEEL_BASE, %s\n", obj_default["ROBOT_WHEEL_BASE"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_mapping = obj["mapping"].toObject();
            {
                SLAM_WINDOW_SIZE = obj_mapping["SLAM_WINDOW_SIZE"].toString().toInt();
                printf("[CONFIG] SLAM_WINDOW_SIZE, %s\n", obj_mapping["SLAM_WINDOW_SIZE"].toString().toLocal8Bit().data());

                SLAM_VOXEL_SIZE = obj_mapping["SLAM_VOXEL_SIZE"].toString().toDouble();
                printf("[CONFIG] SLAM_VOXEL_SIZE, %s\n", obj_mapping["SLAM_VOXEL_SIZE"].toString().toLocal8Bit().data());

                SLAM_ICP_COST_THRESHOLD = obj_mapping["SLAM_ICP_COST_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] SLAM_ICP_COST_THRESHOLD, %s\n", obj_mapping["SLAM_ICP_COST_THRESHOLD"].toString().toLocal8Bit().data());

                SLAM_ICP_ERROR_THRESHOLD = obj_mapping["SLAM_ICP_ERROR_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] SLAM_ICP_ERROR_THRESHOLD, %s\n", obj_mapping["SLAM_ICP_ERROR_THRESHOLD"].toString().toLocal8Bit().data());

                SLAM_ICP_MAX_FEATURE_NUM = obj_mapping["SLAM_ICP_MAX_FEATURE_NUM"].toString().toInt();
                printf("[CONFIG] SLAM_ICP_MAX_FEATURE_NUM, %s\n", obj_mapping["SLAM_ICP_MAX_FEATURE_NUM"].toString().toLocal8Bit().data());

                SLAM_ICP_DO_ERASE_GAP = obj_mapping["SLAM_ICP_DO_ERASE_GAP"].toString().toInt();
                printf("[CONFIG] SLAM_ICP_DO_ERASE_GAP, %s\n", obj_mapping["SLAM_ICP_DO_ERASE_GAP"].toString().toLocal8Bit().data());

                SLAM_ICP_DO_ACCUM_NUM = obj_mapping["SLAM_ICP_DO_ACCUM_NUM"].toString().toInt();
                printf("[CONFIG] SLAM_ICP_DO_ACCUM_NUM, %s\n", obj_mapping["SLAM_ICP_DO_ACCUM_NUM"].toString().toLocal8Bit().data());

                SLAM_ICP_VIEW_THRESHOLD = obj_mapping["SLAM_ICP_VIEW_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] SLAM_ICP_VIEW_THRESHOLD, %s\n", obj_mapping["SLAM_ICP_VIEW_THRESHOLD"].toString().toLocal8Bit().data());

                SLAM_KFRM_UPDATE_NUM = obj_mapping["SLAM_KFRM_UPDATE_NUM"].toString().toInt();
                printf("[CONFIG] SLAM_KFRM_UPDATE_NUM, %s\n", obj_mapping["SLAM_KFRM_UPDATE_NUM"].toString().toLocal8Bit().data());

                SLAM_KFRM_LC_TRY_DIST = obj_mapping["SLAM_KFRM_LC_TRY_DIST"].toString().toDouble();
                printf("[CONFIG] SLAM_KFRM_LC_TRY_DIST, %s\n", obj_mapping["SLAM_KFRM_LC_TRY_DIST"].toString().toLocal8Bit().data());

                SLAM_KFRM_LC_TRY_OVERLAP = obj_mapping["SLAM_KFRM_LC_TRY_OVERLAP"].toString().toDouble();
                printf("[CONFIG] SLAM_KFRM_LC_TRY_OVERLAP, %s\n", obj_mapping["SLAM_KFRM_LC_TRY_OVERLAP"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_loc = obj["loc"].toObject();
            {
                LOC_SURFEL_NUM = obj_loc["LOC_SURFEL_NUM"].toString().toInt();
                printf("[CONFIG] LOC_SURFEL_NUM, %s\n", obj_loc["LOC_SURFEL_NUM"].toString().toLocal8Bit().data());

                LOC_SURFEL_RANGE = obj_loc["LOC_SURFEL_RANGE"].toString().toDouble();
                printf("[CONFIG] LOC_SURFEL_RANGE, %s\n", obj_loc["LOC_SURFEL_RANGE"].toString().toLocal8Bit().data());

                LOC_ICP_COST_THRESHOLD_0 = obj_loc["LOC_ICP_COST_THRESHOLD_0"].toString().toDouble();
                printf("[CONFIG] LOC_ICP_COST_THRESHOLD_0, %s\n", obj_loc["LOC_ICP_COST_THRESHOLD_0"].toString().toLocal8Bit().data());

                LOC_ICP_COST_THRESHOLD = obj_loc["LOC_ICP_COST_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] LOC_ICP_COST_THRESHOLD, %s\n", obj_loc["LOC_ICP_COST_THRESHOLD"].toString().toLocal8Bit().data());

                LOC_ICP_ERROR_THRESHOLD = obj_loc["LOC_ICP_ERROR_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] LOC_ICP_ERROR_THRESHOLD, %s\n", obj_loc["LOC_ICP_ERROR_THRESHOLD"].toString().toLocal8Bit().data());

                LOC_ICP_MAX_FEATURE_NUM = obj_loc["LOC_ICP_MAX_FEATURE_NUM"].toString().toInt();
                printf("[CONFIG] LOC_ICP_MAX_FEATURE_NUM, %s\n", obj_loc["LOC_ICP_MAX_FEATURE_NUM"].toString().toLocal8Bit().data());

                LOC_CHECK_DIST = obj_loc["LOC_CHECK_DIST"].toString().toDouble();
                printf("[CONFIG] LOC_CHECK_DIST, %s\n", obj_loc["LOC_CHECK_DIST"].toString().toLocal8Bit().data());

                LOC_CHECK_IE = obj_loc["LOC_CHECK_IE"].toString().toDouble();
                printf("[CONFIG] LOC_CHECK_IE, %s\n", obj_loc["LOC_CHECK_IE"].toString().toLocal8Bit().data());

                LOC_CHECK_IR = obj_loc["LOC_CHECK_IR"].toString().toDouble();
                printf("[CONFIG] LOC_CHECK_IR, %s\n", obj_loc["LOC_CHECK_IR"].toString().toLocal8Bit().data());

                LOC_ICP_ODO_FUSION_RATIO = obj_loc["LOC_ICP_ODO_FUSION_RATIO"].toString().toDouble();
                printf("[CONFIG] LOC_ICP_ODO_FUSION_RATIO, %s\n", obj_loc["LOC_ICP_ODO_FUSION_RATIO"].toString().toLocal8Bit().data());

                LOC_ARUCO_ODO_FUSION_RATIO = obj_loc["LOC_ARUCO_ODO_FUSION_RATIO"].toString().toDouble();
                printf("[CONFIG] LOC_ARUCO_ODO_FUSION_RATIO, %s\n", obj_loc["LOC_ARUCO_ODO_FUSION_RATIO"].toString().toLocal8Bit().data());

                LOC_ARUCO_ODO_FUSION_DIST = obj_loc["LOC_ARUCO_ODO_FUSION_DIST"].toString().toDouble();
                printf("[CONFIG] LOC_ARUCO_ODO_FUSION_DIST, %s\n", obj_loc["LOC_ARUCO_ODO_FUSION_DIST"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_annot = obj["annotation"].toObject();
            {
                ANNOT_QA_STEP = obj_annot["ANNOT_QA_STEP"].toString().toDouble();
                printf("[CONFIG] ANNOT_QA_STEP, %s\n", obj_annot["ANNOT_QA_STEP"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_debug = obj["debug"].toObject();
            {
                USE_SIM = obj_debug["USE_SIM"].toString().toInt();
                printf("[CONFIG] USE_SIM, %s\n", obj_debug["USE_SIM"].toString().toLocal8Bit().data());

                USE_IMU = obj_debug["USE_IMU"].toString().toInt();
                printf("[CONFIG] USE_IMU, %s\n", obj_debug["USE_IMU"].toString().toLocal8Bit().data());

                USE_BLIDAR = obj_debug["USE_BLIDAR"].toString().toInt();
                printf("[CONFIG] USE_BLIDAR, %s\n", obj_debug["USE_BLIDAR"].toString().toLocal8Bit().data());

                USE_BQR = obj_debug["USE_BQR"].toString().toInt();
                printf("[CONFIG] USE_BQR, %s\n", obj_debug["USE_BQR"].toString().toLocal8Bit().data());

                USE_BEEP = obj_debug["USE_BEEP"].toString().toInt();
                printf("[CONFIG] USE_BEEP, %s\n", obj_debug["USE_BEEP"].toString().toLocal8Bit().data());

                USE_CAM = obj_debug["USE_CAM"].toString().toInt();
                printf("[CONFIG] USE_CAM, %s\n", obj_debug["USE_CAM"].toString().toLocal8Bit().data());

                USE_RTSP = obj_debug["USE_RTSP"].toString().toInt();
                printf("[CONFIG] USE_RTSP, %s\n", obj_debug["USE_RTSP"].toString().toLocal8Bit().data());

                USE_RRS = obj_debug["USE_RRS"].toString().toInt();
                printf("[CONFIG] USE_RRS, %s\n", obj_debug["USE_RRS"].toString().toLocal8Bit().data());

                USE_FMS = obj_debug["USE_FMS"].toString().toInt();
                printf("[CONFIG] USE_FMS, %s\n", obj_debug["USE_FMS"].toString().toLocal8Bit().data());

                USE_QTUI = obj_debug["USE_QTUI"].toString().toInt();
                printf("[CONFIG] USE_QTUI, %s\n", obj_debug["USE_QTUI"].toString().toLocal8Bit().data());

                USE_ARUCO = obj_debug["USE_ARUCO"].toString().toInt();
                printf("[CONFIG] USE_ARUCO, %s\n", obj_debug["USE_ARUCO"].toString().toLocal8Bit().data());

                USE_EARLYSTOP = obj_debug["USE_EARLYSTOP"].toString().toInt();
                printf("[CONFIG] USE_EARLYSTOP, %s\n", obj_debug["USE_EARLYSTOP"].toString().toLocal8Bit().data());

                USE_LVX = obj_debug["USE_LVX"].toString().toInt();
                printf("[CONFIG] USE_LVX, %s\n", obj_debug["USE_LVX"].toString().toLocal8Bit().data());

                USE_COOP = obj_debug["USE_COOP"].toString().toInt();
                printf("[CONFIG] USE_COOP, %s\n", obj_debug["USE_COOP"].toString().toLocal8Bit().data());

                USE_MULTI = obj_debug["USE_MULTI"].toString().toInt();
                printf("[CONFIG] USE_MULTI, %s\n", obj_debug["USE_MULTI"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_control = obj["control"].toObject();
            {
                DRIVE_GOAL_APPROACH_GAIN = obj_control["DRIVE_GOAL_APPROACH_GAIN"].toString().toDouble();
                printf("[CONFIG] DRIVE_GOAL_APPROACH_GAIN, %s\n", obj_control["DRIVE_GOAL_APPROACH_GAIN"].toString().toLocal8Bit().data());

                DRIVE_GOAL_D = obj_control["DRIVE_GOAL_D"].toString().toDouble();
                printf("[CONFIG] DRIVE_GOAL_D, %s\n", obj_control["DRIVE_GOAL_D"].toString().toLocal8Bit().data());

                DRIVE_GOAL_TH = obj_control["DRIVE_GOAL_TH"].toString().toDouble();
                printf("[CONFIG] DRIVE_GOAL_TH, %s\n", obj_control["DRIVE_GOAL_TH"].toString().toLocal8Bit().data());

                DRIVE_EXTENDED_CONTROL_TIME = obj_control["DRIVE_EXTENDED_CONTROL_TIME"].toString().toDouble();
                printf("[CONFIG] DRIVE_EXTENDED_CONTROL_TIME, %s\n", obj_control["DRIVE_EXTENDED_CONTROL_TIME"].toString().toLocal8Bit().data());

                DRIVE_V_DEADZONE = obj_control["DRIVE_V_DEADZONE"].toString().toDouble();
                printf("[CONFIG] DRIVE_V_DEADZONE, %s\n", obj_control["DRIVE_V_DEADZONE"].toString().toLocal8Bit().data());

                DRIVE_W_DEADZONE = obj_control["DRIVE_W_DEADZONE"].toString().toDouble();
                printf("[CONFIG] DRIVE_W_DEADZONE, %s\n", obj_control["DRIVE_W_DEADZONE"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_dock = obj["docking"].toObject();
            {
                DOCK_GOAL_D = obj_dock["DOCK_GOAL_D"].toString().toDouble();
                printf("[CONFIG] DOCK_GOAL_D, %s\n", obj_dock["DOCK_GOAL_D"].toString().toLocal8Bit().data());

                DOCK_GOAL_TH = obj_dock["DOCK_GOAL_TH"].toString().toDouble();
                printf("[CONFIG] DOCK_GOAL_TH, %s\n", obj_dock["DOCK_GOAL_TH"].toString().toLocal8Bit().data());

                DOCK_EXTENDED_CONTROL_TIME = obj_dock["DOCK_EXTENDED_CONTROL_TIME"].toString().toDouble();
                printf("[CONFIG] DOCK_EXTENDED_CONTROL_TIME, %s\n", obj_dock["DOCK_EXTENDED_CONTROL_TIME"].toString().toLocal8Bit().data());

                DOCK_UNDOCK_REVERSING_DISTANCE = obj_dock["DOCK_UNDOCK_REVERSING_DISTANCE"].toString().toDouble();
                printf("[CONFIG] DOCK_UNDOCK_REVERSING_DISTANCE, %s\n", obj_dock["DOCK_UNDOCK_REVERSING_DISTANCE"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_obs = obj["obs"].toObject();
            {
                OBS_AVOID = obj_obs["OBS_AVOID"].toString().toInt();
                printf("[CONFIG] OBS_AVOID, %s\n", obj_obs["OBS_AVOID"].toString().toLocal8Bit().data());

                OBS_DEADZONE = obj_obs["OBS_DEADZONE"].toString().toDouble();
                printf("[CONFIG] OBS_DEADZONE, %s\n", obj_obs["OBS_DEADZONE"].toString().toLocal8Bit().data());

                OBS_LOCAL_GOAL_D = obj_obs["OBS_LOCAL_GOAL_D"].toString().toDouble();
                printf("[CONFIG] OBS_LOCAL_GOAL_D, %s\n", obj_obs["OBS_LOCAL_GOAL_D"].toString().toLocal8Bit().data());

                OBS_SAFE_MARGIN_X = obj_obs["OBS_SAFE_MARGIN_X"].toString().toDouble();
                printf("[CONFIG] OBS_SAFE_MARGIN_X, %s\n", obj_obs["OBS_SAFE_MARGIN_X"].toString().toLocal8Bit().data());

                OBS_SAFE_MARGIN_Y = obj_obs["OBS_SAFE_MARGIN_Y"].toString().toDouble();
                printf("[CONFIG] OBS_SAFE_MARGIN_Y, %s\n", obj_obs["OBS_SAFE_MARGIN_Y"].toString().toLocal8Bit().data());

                OBS_PATH_MARGIN_X = obj_obs["OBS_PATH_MARGIN_X"].toString().toDouble();
                printf("[CONFIG] OBS_PATH_MARGIN_X, %s\n", obj_obs["OBS_PATH_MARGIN_X"].toString().toLocal8Bit().data());

                OBS_PATH_MARGIN_Y = obj_obs["OBS_PATH_MARGIN_Y"].toString().toDouble();
                printf("[CONFIG] OBS_PATH_MARGIN_Y, %s\n", obj_obs["OBS_PATH_MARGIN_Y"].toString().toLocal8Bit().data());

                OBS_MAP_GRID_SIZE = obj_obs["OBS_MAP_GRID_SIZE"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_GRID_SIZE, %s\n", obj_obs["OBS_MAP_GRID_SIZE"].toString().toLocal8Bit().data());

                OBS_MAP_RANGE = obj_obs["OBS_MAP_RANGE"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_RANGE, %s\n", obj_obs["OBS_MAP_RANGE"].toString().toLocal8Bit().data());

                OBS_MAP_MIN_V = obj_obs["OBS_MAP_MIN_V"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MIN_V, %s\n", obj_obs["OBS_MAP_MIN_V"].toString().toLocal8Bit().data());

                OBS_MAP_MIN_Z = obj_obs["OBS_MAP_MIN_Z"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MIN_Z, %s\n", obj_obs["OBS_MAP_MIN_Z"].toString().toLocal8Bit().data());

                OBS_MAP_MAX_Z = obj_obs["OBS_MAP_MAX_Z"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MAX_Z, %s\n", obj_obs["OBS_MAP_MAX_Z"].toString().toLocal8Bit().data());

                OBS_PREDICT_TIME = obj_obs["OBS_PREDICT_TIME"].toString().toDouble();
                printf("[CONFIG] OBS_PREDICT_TIME, %s\n", obj_obs["OBS_PREDICT_TIME"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_cam = obj["cam"].toObject();
            {
                CAM_HEIGHT_MIN = obj_cam["CAM_HEIGHT_MIN"].toString().toDouble();
                printf("[CONFIG] CAM_HEIGHT_MIN, %s\n", obj_cam["CAM_HEIGHT_MIN"].toString().toLocal8Bit().data());

                CAM_HEIGHT_MAX = obj_cam["CAM_HEIGHT_MAX"].toString().toDouble();
                printf("[CONFIG] CAM_HEIGHT_MAX, %s\n", obj_cam["CAM_HEIGHT_MAX"].toString().toLocal8Bit().data());

                CAM_TF_0 = obj_cam["CAM_TF_0"].toString();
                printf("[CONFIG] CAM_TF_0, %s\n", obj_cam["CAM_TF_0"].toString().toLocal8Bit().data());

                CAM_TF_1 = obj_cam["CAM_TF_1"].toString();
                printf("[CONFIG] CAM_TF_1, %s\n", obj_cam["CAM_TF_1"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_server = obj["fms"].toObject();
            {
                SERVER_IP = obj_server["SERVER_IP"].toString();
                printf("[CONFIG] SERVER_IP, %s\n", obj_server["SERVER_IP"].toString().toLocal8Bit().data());

                SERVER_ID = obj_server["SERVER_ID"].toString();
                printf("[CONFIG] SERVER_ID, %s\n", obj_server["SERVER_ID"].toString().toLocal8Bit().data());

                SERVER_PW = obj_server["SERVER_PW"].toString();
                printf("[CONFIG] SERVER_PW, %s\n", obj_server["SERVER_PW"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_lvx = obj["lvx"].toObject();
            {
                LVX_TF = obj_lvx["LVX_TF"].toString();
                printf("[CONFIG] LVX_TF, %s\n", obj_lvx["LVX_TF"].toString().toLocal8Bit().data());

                LVX_FRM_DT = obj_lvx["LVX_FRM_DT"].toString().toDouble();
                printf("[CONFIG] LVX_FRM_DT, %s\n", obj_lvx["LVX_FRM_DT"].toString().toLocal8Bit().data());

                LVX_MIN_RANGE = obj_lvx["LVX_MIN_RANGE"].toString().toDouble();
                printf("[CONFIG] LVX_MIN_RANGE, %s\n", obj_lvx["LVX_MIN_RANGE"].toString().toLocal8Bit().data());

                LVX_MAX_RANGE = obj_lvx["LVX_MAX_RANGE"].toString().toDouble();
                printf("[CONFIG] LVX_MAX_RANGE, %s\n", obj_lvx["LVX_MAX_RANGE"].toString().toLocal8Bit().data());

                LVX_MAX_FEATURE_NUM = obj_lvx["LVX_MAX_FEATURE_NUM"].toString().toInt();
                printf("[CONFIG] LVX_MAX_FEATURE_NUM, %s\n", obj_lvx["LVX_MAX_FEATURE_NUM"].toString().toLocal8Bit().data());

                LVX_SURFEL_NN_NUM = obj_lvx["LVX_SURFEL_NN_NUM"].toString().toInt();
                printf("[CONFIG] LVX_SURFEL_NN_NUM, %s\n", obj_lvx["LVX_SURFEL_NN_NUM"].toString().toLocal8Bit().data());

                LVX_SURFEL_RANGE = obj_lvx["LVX_SURFEL_RANGE"].toString().toDouble();
                printf("[CONFIG] LVX_SURFEL_RANGE, %s\n", obj_lvx["LVX_SURFEL_RANGE"].toString().toLocal8Bit().data());

                LVX_COST_THRESHOLD = obj_lvx["LVX_COST_THRESHOLD"].toString().toDouble();
                printf("[CONFIG] LVX_COST_THRESHOLD, %s\n", obj_lvx["LVX_COST_THRESHOLD"].toString().toLocal8Bit().data());

                LVX_INLIER_CHECK_DIST = obj_lvx["LVX_INLIER_CHECK_DIST"].toString().toDouble();
                printf("[CONFIG] LVX_ERROR_THRESHOLD, %s\n", obj_lvx["LVX_INLIER_CHECK_DIST"].toString().toLocal8Bit().data());
            }

            // complete
            config_file.close();
            printf("[CONFIG] %s, load successed\n", config_path.toLocal8Bit().data());
        }

        // read config
        QFile config_sn_file(config_sn_path);
        if(config_sn_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_sn_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            QJsonObject obj_cam = obj["cam"].toObject();
            {
                CAM_SERIAL_NUMBER_0 = obj_cam["CAM_SERIAL_NUMBER_0"].toString();
                printf("[CONFIG] CAM_SERIAL_NUMBER_0, %s\n", obj_cam["CAM_SERIAL_NUMBER_0"].toString().toLocal8Bit().data());

                CAM_SERIAL_NUMBER_1 = obj_cam["CAM_SERIAL_NUMBER_1"].toString();
                printf("[CONFIG] CAM_SERIAL_NUMBER_1, %s\n", obj_cam["CAM_SERIAL_NUMBER_1"].toString().toLocal8Bit().data());
            }

            // complete
            is_load = true;
            config_sn_file.close();
            printf("[CONFIG] %s, load successed\n", config_sn_path.toLocal8Bit().data());
        }
    }
    else
    {
        printf("[CONFIG] config file load failed\n");
    }
}

void CONFIG::load_code_info(QString path)
{
    // load params
    QFileInfo config_info(path);
    if(config_info.exists() && config_info.isFile())
    {
        // read
        QFile config_file(path);
        if(config_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            QJsonObject obj_code = obj["code"].toObject();
            {
                CODE_A1_X = obj_code["CODE_A1_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A1_X, %s\n", obj_code["CODE_A1_X"].toString().toLocal8Bit().data());

                CODE_A2_X = obj_code["CODE_A2_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A2_X, %s\n", obj_code["CODE_A2_X"].toString().toLocal8Bit().data());

                CODE_A3_X = obj_code["CODE_A3_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A3_X, %s\n", obj_code["CODE_A3_X"].toString().toLocal8Bit().data());

                CODE_A4_X = obj_code["CODE_A4_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A4_X, %s\n", obj_code["CODE_A4_X"].toString().toLocal8Bit().data());

                CODE_A1_Y = obj_code["CODE_A1_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A1_Y, %s\n", obj_code["CODE_A1_Y"].toString().toLocal8Bit().data());

                CODE_A2_Y = obj_code["CODE_A2_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A2_Y, %s\n", obj_code["CODE_A2_Y"].toString().toLocal8Bit().data());

                CODE_A3_Y = obj_code["CODE_A3_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A3_Y, %s\n", obj_code["CODE_A3_Y"].toString().toLocal8Bit().data());

                CODE_A4_Y = obj_code["CODE_A4_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_A4_Y, %s\n", obj_code["CODE_A4_Y"].toString().toLocal8Bit().data());

                CODE_B1_X = obj_code["CODE_B1_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B1_X, %s\n", obj_code["CODE_B1_X"].toString().toLocal8Bit().data());

                CODE_B2_X = obj_code["CODE_B2_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B2_X, %s\n", obj_code["CODE_B2_X"].toString().toLocal8Bit().data());

                CODE_B3_X = obj_code["CODE_B3_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B3_X, %s\n", obj_code["CODE_B3_X"].toString().toLocal8Bit().data());

                CODE_B4_X = obj_code["CODE_B4_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B4_X, %s\n", obj_code["CODE_B4_X"].toString().toLocal8Bit().data());

                CODE_B1_Y = obj_code["CODE_B1_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B1_Y, %s\n", obj_code["CODE_B1_Y"].toString().toLocal8Bit().data());

                CODE_B2_Y = obj_code["CODE_B2_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B2_Y, %s\n", obj_code["CODE_B2_Y"].toString().toLocal8Bit().data());

                CODE_B3_Y = obj_code["CODE_B3_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B3_Y, %s\n", obj_code["CODE_B3_Y"].toString().toLocal8Bit().data());

                CODE_B4_Y = obj_code["CODE_B4_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_B4_Y, %s\n", obj_code["CODE_B4_Y"].toString().toLocal8Bit().data());

                CODE_C1_X = obj_code["CODE_C1_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C1_X, %s\n", obj_code["CODE_C1_X"].toString().toLocal8Bit().data());

                CODE_C2_X = obj_code["CODE_C2_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C2_X, %s\n", obj_code["CODE_C2_X"].toString().toLocal8Bit().data());

                CODE_C3_X = obj_code["CODE_C3_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C3_X, %s\n", obj_code["CODE_C3_X"].toString().toLocal8Bit().data());

                CODE_C4_X = obj_code["CODE_C4_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C4_X, %s\n", obj_code["CODE_C4_X"].toString().toLocal8Bit().data());

                CODE_C1_Y = obj_code["CODE_C1_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C1_Y, %s\n", obj_code["CODE_C1_Y"].toString().toLocal8Bit().data());

                CODE_C2_Y = obj_code["CODE_C2_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C2_Y, %s\n", obj_code["CODE_C2_Y"].toString().toLocal8Bit().data());

                CODE_C3_Y = obj_code["CODE_C3_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C3_Y, %s\n", obj_code["CODE_C3_Y"].toString().toLocal8Bit().data());

                CODE_C4_Y = obj_code["CODE_C4_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_C4_Y, %s\n", obj_code["CODE_C4_Y"].toString().toLocal8Bit().data());

                CODE_D1_X = obj_code["CODE_D1_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D1_X, %s\n", obj_code["CODE_D1_X"].toString().toLocal8Bit().data());

                CODE_D2_X = obj_code["CODE_D2_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D2_X, %s\n", obj_code["CODE_D2_X"].toString().toLocal8Bit().data());

                CODE_D3_X = obj_code["CODE_D3_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D3_X, %s\n", obj_code["CODE_D3_X"].toString().toLocal8Bit().data());

                CODE_D4_X = obj_code["CODE_D4_X"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D4_X, %s\n", obj_code["CODE_D4_X"].toString().toLocal8Bit().data());

                CODE_D1_Y = obj_code["CODE_D1_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D1_Y, %s\n", obj_code["CODE_D1_Y"].toString().toLocal8Bit().data());

                CODE_D2_Y = obj_code["CODE_D2_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D2_Y, %s\n", obj_code["CODE_D2_Y"].toString().toLocal8Bit().data());

                CODE_D3_Y = obj_code["CODE_D3_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D3_Y, %s\n", obj_code["CODE_D3_Y"].toString().toLocal8Bit().data());

                CODE_D4_Y = obj_code["CODE_D4_Y"].toString().toDouble();
                printf("[CONFIG_CODE] CODE_D4_Y, %s\n", obj_code["CODE_D4_Y"].toString().toLocal8Bit().data());
            }
        }
    }
    else
    {
        printf("[CONFIG_CODE] %s, load_code failed\n", path.toLocal8Bit().data());
    }
}

void CONFIG::load_ext(QString path)
{
    // load params
    QFileInfo config_info(path);
    if(config_info.exists() && config_info.isFile())
    {
        // read
        QFile config_file(path);
        if(config_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            QJsonObject obj_obs = obj["obs"].toObject();
            {
                OBS_AVOID = obj_obs["OBS_AVOID"].toString().toInt();
                printf("[CONFIG] OBS_AVOID, %s\n", obj_obs["OBS_AVOID"].toString().toLocal8Bit().data());

                OBS_DEADZONE = obj_obs["OBS_DEADZONE"].toString().toDouble();
                printf("[CONFIG] OBS_DEADZONE, %s\n", obj_obs["OBS_DEADZONE"].toString().toLocal8Bit().data());

                OBS_LOCAL_GOAL_D = obj_obs["OBS_LOCAL_GOAL_D"].toString().toDouble();
                printf("[CONFIG] OBS_LOCAL_GOAL_D, %s\n", obj_obs["OBS_LOCAL_GOAL_D"].toString().toLocal8Bit().data());

                OBS_SAFE_MARGIN_X = obj_obs["OBS_SAFE_MARGIN_X"].toString().toDouble();
                printf("[CONFIG] OBS_SAFE_MARGIN_X, %s\n", obj_obs["OBS_SAFE_MARGIN_X"].toString().toLocal8Bit().data());

                OBS_SAFE_MARGIN_Y = obj_obs["OBS_SAFE_MARGIN_Y"].toString().toDouble();
                printf("[CONFIG] OBS_SAFE_MARGIN_Y, %s\n", obj_obs["OBS_SAFE_MARGIN_Y"].toString().toLocal8Bit().data());

                OBS_PATH_MARGIN_X = obj_obs["OBS_PATH_MARGIN_X"].toString().toDouble();
                printf("[CONFIG] OBS_PATH_MARGIN_X, %s\n", obj_obs["OBS_PATH_MARGIN_X"].toString().toLocal8Bit().data());

                OBS_PATH_MARGIN_Y = obj_obs["OBS_PATH_MARGIN_Y"].toString().toDouble();
                printf("[CONFIG] OBS_PATH_MARGIN_Y, %s\n", obj_obs["OBS_PATH_MARGIN_Y"].toString().toLocal8Bit().data());

                OBS_MAP_GRID_SIZE = obj_obs["OBS_MAP_GRID_SIZE"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_GRID_SIZE, %s\n", obj_obs["OBS_MAP_GRID_SIZE"].toString().toLocal8Bit().data());

                OBS_MAP_RANGE = obj_obs["OBS_MAP_RANGE"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_RANGE, %s\n", obj_obs["OBS_MAP_RANGE"].toString().toLocal8Bit().data());

                OBS_MAP_MIN_V = obj_obs["OBS_MAP_MIN_V"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MIN_V, %s\n", obj_obs["OBS_MAP_MIN_V"].toString().toLocal8Bit().data());

                OBS_MAP_MIN_Z = obj_obs["OBS_MAP_MIN_Z"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MIN_Z, %s\n", obj_obs["OBS_MAP_MIN_Z"].toString().toLocal8Bit().data());

                OBS_MAP_MAX_Z = obj_obs["OBS_MAP_MAX_Z"].toString().toDouble();
                printf("[CONFIG] OBS_MAP_MAX_Z, %s\n", obj_obs["OBS_MAP_MAX_Z"].toString().toLocal8Bit().data());

                OBS_PREDICT_TIME = obj_obs["OBS_PREDICT_TIME"].toString().toDouble();
                printf("[CONFIG] OBS_PREDICT_TIME, %s\n", obj_obs["OBS_PREDICT_TIME"].toString().toLocal8Bit().data());
            }

            config_file.close();
            printf("[CONFIG_EXT] %s, load_ext successed\n", path.toLocal8Bit().data());
        }
    }
    else
    {
        printf("[CONFIG_EXT] %s, load_ext failed\n", path.toLocal8Bit().data());
    }
}

