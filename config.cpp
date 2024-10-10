#include "config.h"

CONFIG::CONFIG(QObject *parent)
    : QObject{parent}
{
}

void CONFIG::load()
{
    // load params
    QFileInfo config_info(config_path);
    if(config_info.exists() && config_info.isFile())
    {
        // read
        QFile config_file(config_path);
        if(config_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = config_file.readAll();
            QJsonDocument doc = QJsonDocument::fromJson(data);
            QJsonObject obj = doc.object();

            // param load
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

                ROBOT_RADIUS = obj_default["ROBOT_RADIUS"].toString().toDouble();
                printf("[CONFIG] ROBOT_RADIUS, %s\n", obj_default["ROBOT_RADIUS"].toString().toLocal8Bit().data());

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

                LOC_FUSION_RATIO = obj_loc["LOC_FUSION_RATIO"].toString().toDouble();
                printf("[CONFIG] LOC_FUSION_RATIO, %s\n", obj_loc["LOC_FUSION_RATIO"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_annot = obj["annotation"].toObject();
            {
                ANNOT_QA_STEP = obj_annot["ANNOT_QA_STEP"].toString().toDouble();
                printf("[CONFIG] ANNOT_QA_STEP, %s\n", obj_annot["ANNOT_QA_STEP"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_debug = obj["debug"].toObject();
            {
                SIM_MODE = obj_debug["SIM_MODE"].toString().toInt();
                printf("[CONFIG] SIM_MODE, %s\n", obj_debug["SIM_MODE"].toString().toLocal8Bit().data());
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
                CAM_SERIAL_NUMBER_0 = obj_cam["CAM_SERIAL_NUMBER_0"].toString();
                printf("[CONFIG] CAM_SERIAL_NUMBER_0, %s\n", obj_cam["CAM_SERIAL_NUMBER_0"].toString().toLocal8Bit().data());

                CAM_SERIAL_NUMBER_1 = obj_cam["CAM_SERIAL_NUMBER_1"].toString();
                printf("[CONFIG] CAM_SERIAL_NUMBER_1, %s\n", obj_cam["CAM_SERIAL_NUMBER_1"].toString().toLocal8Bit().data());

                CAM_TF_0 = obj_cam["CAM_TF_0"].toString();
                printf("[CONFIG] CAM_TF_0, %s\n", obj_cam["CAM_TF_0"].toString().toLocal8Bit().data());

                CAM_TF_1 = obj_cam["CAM_TF_1"].toString();
                printf("[CONFIG] CAM_TF_1, %s\n", obj_cam["CAM_TF_1"].toString().toLocal8Bit().data());

                CAM_HEIGHT_MIN = obj_cam["CAM_HEIGHT_MIN"].toString().toDouble();
                printf("[CONFIG] CAM_HEIGHT_MIN, %s\n", obj_cam["CAM_HEIGHT_MIN"].toString().toLocal8Bit().data());

                CAM_HEIGHT_MAX = obj_cam["CAM_HEIGHT_MAX"].toString().toDouble();
                printf("[CONFIG] CAM_HEIGHT_MAX, %s\n", obj_cam["CAM_HEIGHT_MAX"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_fms = obj["fms"].toObject();
            {
                SERVER_IP = obj_fms["SERVER_IP"].toString();
                printf("[CONFIG] SERVER_IP, %s\n", obj_cam["SERVER_IP"].toString().toLocal8Bit().data());

                SERVER_ID = obj_fms["SERVER_ID"].toString();
                printf("[CONFIG] SERVER_ID, %s\n", obj_cam["SERVER_ID"].toString().toLocal8Bit().data());

                SERVER_PW = obj_fms["SERVER_PW"].toString();
                printf("[CONFIG] SERVER_PW, %s\n", obj_cam["SERVER_PW"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_docking = obj["docking"].toObject();
            {
                DOCKING_GOAL_D = obj_docking["DOCKING_GOAL_D"].toString().toDouble();
                printf("[CONFIG] DOCKING_GOAL_D, %s\n", obj_docking["DOCKING_GOAL_D"].toString().toLocal8Bit().data());

                DOCKING_GOAL_TH = obj_docking["DOCKING_GOAL_TH"].toString().toDouble();
                printf("[CONFIG] DOCKING_GOAL_TH, %s\n", obj_docking["DOCKING_GOAL_TH"].toString().toLocal8Bit().data());

                DOCKING_LIMIT_W_ACC = obj_docking["DOCKING_LIMIT_W_ACC"].toString().toDouble();
                printf("[CONFIG] DOCKING_LIMIT_W_ACC, %s\n", obj_docking["DOCKING_LIMIT_W_ACC"].toString().toLocal8Bit().data());

                DOCKING_LIMIT_V = obj_docking["DOCKING_LIMIT_V"].toString().toDouble();
                printf("[CONFIG] DOCKING_LIMIT_V, %s\n", obj_docking["DOCKING_LIMIT_V"].toString().toLocal8Bit().data());

                DOCKING_LIMIT_W = obj_docking["DOCKING_LIMIT_W"].toString().toDouble();
                printf("[CONFIG] DOCKING_LIMIT_W, %s\n", obj_docking["DOCKING_LIMIT_W"].toString().toLocal8Bit().data());

                DOCKING_DRIVE_T = obj_docking["DOCKING_DRIVE_T"].toString().toDouble();
                printf("[CONFIG] DOCKING_DRIVE_T, %s\n", obj_docking["DOCKING_DRIVE_T"].toString().toLocal8Bit().data());

                DOCKING_GAIN_KP = obj_docking["DOCKING_GAIN_KP"].toString().toDouble();
                printf("[CONFIG] DOCKING_GAIN_KP, %s\n", obj_docking["DOCKING_GAIN_KP"].toString().toLocal8Bit().data());

                DOCKING_GAIN_KD = obj_docking["DOCKING_GAIN_KD"].toString().toDouble();
                printf("[CONFIG] DOCKING_GAIN_KD, %s\n", obj_docking["DOCKING_GAIN_KD"].toString().toLocal8Bit().data());

                DOCKING_EXTENDED_CONTROL_TIME = obj_docking["DOCKING_EXTENDED_CONTROL_TIME"].toString().toDouble();
                printf("[CONFIG] DOCKING_EXTENDED_CONTROL_TIME, %s\n", obj_docking["DOCKING_EXTENDED_CONTROL_TIME"].toString().toLocal8Bit().data());
            }

            QJsonObject obj_map = obj["path"].toObject();
            {
                MAP_PATH = obj_map["MAP_PATH"].toString();
                printf("[CONFIG] MAP_PATH, %s\n", obj_docking["MAP_PATH"].toString().toLocal8Bit().data());
            }

            // complete
            is_load = true;
            config_file.close();
            printf("[CONFIG] %s, load successed\n", config_path.toLocal8Bit().data());
        }
    }
    else
    {
        printf("[CONFIG] %s, load failed\n", config_path.toLocal8Bit().data());
    }
}

void CONFIG::save_map_path()
{
    if(config_path == "")
    {
        return;
    }

    QFile config_file(config_path);
    if(config_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = config_file.readAll();
        config_file.close();

        QJsonDocument doc = QJsonDocument::fromJson(data);
        QJsonObject rootObj = doc.object();

        if(rootObj.contains("path"))
        {
            QJsonObject pathObj = rootObj["path"].toObject();
            if(pathObj.contains("MAP_PATH"))
            {
                pathObj["MAP_PATH"] = MAP_PATH;
            }

            rootObj["path"] = pathObj;
        }

        if(config_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
            doc.setObject(rootObj);
            config_file.write(doc.toJson());
            config_file.close();
        }
    }
}
