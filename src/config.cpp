#include "config.h"

CONFIG::CONFIG(QObject *parent)
    : QObject{parent}
{
}

bool CONFIG::load_common(QString path)
{
    QFile common_file(path);
    if(!common_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "[CONFIG] Failed to open common file:" << config_path;
        return false;
    }

    QByteArray data = common_file.readAll();
    common_file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        qWarning() << "[CONFIG] common JSON parse error:" << parseError.errorString();
        return false;
    }

    QJsonObject obj = doc.object();

    if(obj.contains("PLATFORM_NAME"))
    {
        PLATFORM_NAME = obj["PLATFORM_NAME"].toString();
    }

    // complete
    common_file.close();
    return true;
}

void CONFIG::load()
{
    QFile config_file(config_path);
    if(!config_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "[CONFIG] Failed to open config file:" << config_path;
        return;
    }

    QByteArray data = config_file.readAll();
    config_file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        qWarning() << "[CONFIG] JSON parse error:" << parseError.errorString();
        return;
    }

    QJsonObject obj = doc.object();

    QJsonObject obj_robot = obj["robot"].toObject();
    {
        PLATFORM_NAME = obj_robot["PLATFORM_NAME"].toString();
        printf("[CONFIG] PLATFORM_NAME, %s\n", obj_robot["PLATFORM_NAME"].toString().toLocal8Bit().data());

        PLATFORM_TYPE = obj_robot["PLATFORM_TYPE"].toString();
        printf("[CONFIG] PLATFORM_TYPE, %s\n", obj_robot["PLATFORM_TYPE"].toString().toLocal8Bit().data());

        ROBOT_SIZE_X[0] = obj_robot["ROBOT_SIZE_MIN_X"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MIN_X, %s\n", obj_robot["ROBOT_SIZE_MIN_X"].toString().toLocal8Bit().data());

        ROBOT_SIZE_X[1] = obj_robot["ROBOT_SIZE_MAX_X"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MAX_X, %s\n", obj_robot["ROBOT_SIZE_MAX_X"].toString().toLocal8Bit().data());

        ROBOT_SIZE_Y[0] = obj_robot["ROBOT_SIZE_MIN_Y"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MIN_Y, %s\n", obj_robot["ROBOT_SIZE_MIN_Y"].toString().toLocal8Bit().data());

        ROBOT_SIZE_Y[1] = obj_robot["ROBOT_SIZE_MAX_Y"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MAX_Y, %s\n", obj_robot["ROBOT_SIZE_MAX_Y"].toString().toLocal8Bit().data());

        ROBOT_SIZE_Z[0] = obj_robot["ROBOT_SIZE_MIN_Z"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MIN_Z, %s\n", obj_robot["ROBOT_SIZE_MIN_Z"].toString().toLocal8Bit().data());

        ROBOT_SIZE_Z[1] = obj_robot["ROBOT_SIZE_MAX_Z"].toString().toDouble();
        printf("[CONFIG] ROBOT_SIZE_MAX_Z, %s\n", obj_robot["ROBOT_SIZE_MAX_Z"].toString().toLocal8Bit().data());

        // robot radius calculation
        double lx = std::max<double>(std::abs(ROBOT_SIZE_X[0]), std::abs(ROBOT_SIZE_X[1]));
        double ly = std::max<double>(std::abs(ROBOT_SIZE_Y[0]), std::abs(ROBOT_SIZE_Y[1]));
        ROBOT_RADIUS = std::sqrt(lx*lx + ly*ly);
        printf("[CONFIG] ROBOT_RADIUS(auto calc), %.3f\n", ROBOT_RADIUS);

        ROBOT_WHEEL_RADIUS = obj_robot["ROBOT_WHEEL_RADIUS"].toString().toDouble();
        printf("[CONFIG] ROBOT_WHEEL_RADIUS, %s\n", obj_robot["ROBOT_WHEEL_RADIUS"].toString().toLocal8Bit().data());

        ROBOT_WHEEL_BASE = obj_robot["ROBOT_WHEEL_BASE"].toString().toDouble();
        printf("[CONFIG] ROBOT_WHEEL_BASE, %s\n", obj_robot["ROBOT_WHEEL_BASE"].toString().toLocal8Bit().data());
    }

    QJsonObject obj_sensors = obj["sensors"].toObject();
    {
        USE_LIDAR_2D = obj_sensors["USE_LIDAR_2D"].toBool();
        printf("[CONFIG] USE_LIDAR_2D, %s\n", USE_LIDAR_2D ? "true" : "false");

        LIDAR_2D_TYPE = obj_sensors["LIDAR_2D_TYPE"].toString();
        printf("[CONFIG] LIDAR_2D_TYPE, %s\n", LIDAR_2D_TYPE.toLocal8Bit().data());

        LIDAR_2D_NUM = obj_sensors["LIDAR_2D_NUM"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_NUM, %s\n", obj_sensors["LIDAR_2D_NUM"].toString().toLocal8Bit().data());

        USE_LIDAR_3D = obj_sensors["USE_LIDAR_3D"].toBool();
        printf("[CONFIG] USE_LIDAR_3D, %s\n", USE_LIDAR_3D ? "true" : "false");

        LIDAR_3D_TYPE = obj_sensors["LIDAR_3D_TYPE"].toString();
        printf("[CONFIG] LIDAR_3D_TYPE, %s\n", LIDAR_3D_TYPE.toLocal8Bit().data());

        LIDAR_3D_NUM = obj_sensors["LIDAR_3D_NUM"].toString().toDouble();
        printf("[CONFIG] LIDAR_3D_NUM, %s\n", obj_sensors["LIDAR_3D_NUM"].toString().toLocal8Bit().data());

        USE_CAM = obj_sensors["USE_CAM"].toBool();
        printf("[CONFIG] USE_CAM, %s\n", USE_CAM ? "true" : "false");

        CAM_NUM = obj_sensors["CAM_NUM"].toString().toDouble();
        printf("[CONFIG] CAM_NUM, %s\n", obj_sensors["CAM_NUM"].toString().toLocal8Bit().data());

        USE_BQR = obj_sensors["USE_BQR"].toBool();
        printf("[CONFIG] USE_BQR, %s\n", obj_sensors["USE_BQR"].toString().toLocal8Bit().data());

        USE_IMU = obj_sensors["USE_IMU"].toBool();
        printf("[CONFIG] USE_IMU, %s\n", obj_sensors["USE_IMU"].toString().toLocal8Bit().data());
    }

    QJsonObject obj_loc = obj["localization"].toObject();
    {
        LOC_MODE = obj_loc["MODE"].toString();
        printf("[CONFIG] LOC_MODE, %s\n", obj_loc["MODE"].toString().toLocal8Bit().data());

        USE_ARUCO = obj_loc["USE_ARUCO"].toBool();
        printf("[CONFIG] USE_ARUCO, %s\n", USE_ARUCO ? "true" : "false");
    }

    QJsonObject obj_net = obj["network"].toObject();
    {
        USE_MULTI = obj_net["USE_MULTI"].toBool();
        printf("[CONFIG] USE_MULTI, %s\n", USE_MULTI ? "true" : "false");

        USE_COOP = obj_net["USE_COOP"].toBool();
        printf("[CONFIG] USE_COOP, %s\n", USE_COOP ? "true" : "false");

        USE_RTSP = obj_net["USE_RTSP"].toBool();
        printf("[CONFIG] USE_RTSP, %s\n", USE_RTSP ? "true" : "false");

        USE_RRS = obj_net["USE_RRS"].toBool();
        printf("[CONFIG] USE_RRS, %s\n", USE_RRS ? "true" : "false");

        USE_FMS = obj_net["USE_FMS"].toBool();
        printf("[CONFIG] USE_FMS, %s\n", USE_FMS ? "true" : "false");
    }

    QJsonObject obj_debug = obj["debug"].toObject();
    {
        USE_SIM = obj_debug["USE_SIM"].toBool();
        printf("[CONFIG] USE_SIM, %s\n", USE_SIM ? "true" : "false");

        USE_QTUI = obj_debug["USE_QTUI"].toBool();
        printf("[CONFIG] USE_QTUI, %s\n", USE_QTUI ? "true" : "false");

        USE_BEEP = obj_debug["USE_BEEP"].toBool();
        printf("[CONFIG] USE_BEEP, %s\n", USE_BEEP ? "true" : "false");
    }

    QJsonObject obj_path = obj["map"].toObject();
    {
        MAP_PATH = obj_path["MAP_PATH"].toString();
    }

    if(USE_LIDAR_2D)
    {
        QJsonArray lidar_arr = obj["lidar_2d_config"].toArray();
        for(int i = 0; i < LIDAR_2D_NUM; i++)
        {
            QJsonObject obj_lidar_2d = lidar_arr[i].toObject();
            LIDAR_2D_IP[i] = obj_lidar_2d["IP"].toString();
            LIDAR_2D_TF[i] = obj_lidar_2d["TF"].toString();
            printf("[CONFIG] LIDAR_2D[%d] IP: %s, TF: %s\n",i, qUtf8Printable(LIDAR_2D_IP[i]), qUtf8Printable(LIDAR_2D_TF[i]));
        }
    }

    if(USE_LIDAR_3D)
    {
        QJsonArray lidar_arr = obj["lidar_3d_config"].toArray();
        for(int i = 0; i < LIDAR_3D_NUM; i++)
        {
            QJsonObject obj_lidar_3d = lidar_arr[i].toObject();
            LIDAR_3D_IP[i] = obj_lidar_3d["IP"].toString();
            LIDAR_3D_TF[i] = obj_lidar_3d["TF"].toString();
            printf("[CONFIG] LIDAR_3D[%d] IP: %s, TF: %s\n",i, qUtf8Printable(LIDAR_3D_IP[i]), qUtf8Printable(LIDAR_3D_TF[i]));
        }
    }

    if(USE_CAM)
    {
        QJsonArray cam_arr = obj["cam_config"].toArray();
        for(int i = 0; i < cam_arr.size(); i++)
        {
            QJsonObject obj_cam = cam_arr[i].toObject();
            CAM_SERIAL_NUMBER[i] = obj_cam["SERIAL_NUMBER"].toString();
            CAM_TF[i] = obj_cam["TF"].toString();
            printf("[CONFIG] CAM[%d] Serial: %s, TF: %s\n", i, qUtf8Printable(CAM_SERIAL_NUMBER[i]), qUtf8Printable(CAM_TF[i]));
        }
    }

    if(LIDAR_2D_TYPE == "SICK" && obj.contains("SICK"))
    {
        QJsonObject obj_sick = obj["SICK"].toObject();

        LIDAR_2D_MIN_RANGE = obj_sick["LIDAR_2D_MIN_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MIN_RANGE(SICK), %s\n", obj_sick["LIDAR_2D_MIN_RANGE"].toString().toLocal8Bit().data());

        LIDAR_2D_MAX_RANGE = obj_sick["LIDAR_2D_MAX_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MAX_RANGE(SICK), %s\n", obj_sick["LIDAR_2D_MAX_RANGE"].toString().toLocal8Bit().data());
    }
    else if(LIDAR_2D_TYPE == "LAKI" && obj.contains("LAKI"))
    {
        QJsonObject obj_laki = obj["LAKI"].toObject();

        LIDAR_2D_MIN_RANGE = obj_laki["LIDAR_2D_MIN_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MIN_RANGE(LAKI), %s\n", obj_laki["LIDAR_2D_MIN_RANGE"].toString().toLocal8Bit().data());

        LIDAR_2D_MAX_RANGE = obj_laki["LIDAR_2D_MAX_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MAX_RANGE(LAKI), %s\n", obj_laki["LIDAR_2D_MAX_RANGE"].toString().toLocal8Bit().data());
    }
    else if(LIDAR_2D_TYPE == "RPLIDAR" && obj.contains("RPLIDAR"))
    {
        QJsonObject obj_rplidar = obj["RPLIDAR"].toObject();

        LIDAR_2D_MIN_RANGE = obj_rplidar["LIDAR_2D_MIN_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MIN_RANGE(RPLIDAR), %s\n", obj_rplidar["LIDAR_2D_MIN_RANGE"].toString().toLocal8Bit().data());

        LIDAR_2D_MAX_RANGE = obj_rplidar["LIDAR_2D_MAX_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_2D_MAX_RANGE(RPLIDAR), %s\n", obj_rplidar["LIDAR_2D_MAX_RANGE"].toString().toLocal8Bit().data());
    }

    if(LIDAR_3D_TYPE == "LIVOX" && obj.contains("LIVOX"))
    {
        QJsonObject obj_lvx = obj["LIVOX"].toObject();

        LIDAR_3D_MIN_RANGE = obj_lvx["LIDAR_3D_MIN_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_3D_MIN_RANGE(LIVOX), %s\n", obj_lvx["LIDAR_3D_MIN_RANGE"].toString().toLocal8Bit().data());

        LIDAR_3D_MAX_RANGE = obj_lvx["LIDAR_3D_MAX_RANGE"].toString().toDouble();
        printf("[CONFIG] LIDAR_3D_MAX_RANGE(LIVOX), %s\n", obj_lvx["LIDAR_3D_MAX_RANGE"].toString().toLocal8Bit().data());
    }

    if(CAM_TYPE == "ORBBEC" && obj.contains("ORBBEC"))
    {
        QJsonObject obj_cam = obj["ORBBEC"].toObject();

        CAM_HEIGHT_MAX = obj_cam["CAM_HEIGHT_MAX"].toString().toDouble();
        printf("[CONFIG] CAM_HEIGHT_MAX(ORBBEC), %s\n", obj_cam["CAM_HEIGHT_MAX"].toString().toLocal8Bit().data());

        CAM_HEIGHT_MIN = obj_cam["CAM_HEIGHT_MIN"].toString().toDouble();
        printf("[CONFIG] CAM_HEIGHT_MIN(ORBBEC), %s\n", obj_cam["CAM_HEIGHT_MIN"].toString().toLocal8Bit().data());
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

    QJsonObject obj_annot = obj["annotation"].toObject();
    {
        ANNOT_QA_STEP = obj_annot["ANNOT_QA_STEP"].toString().toDouble();
        printf("[CONFIG] ANNOT_QA_STEP, %s\n", obj_annot["ANNOT_QA_STEP"].toString().toLocal8Bit().data());
    }

    if(LOC_MODE == "2D")
    {
        QJsonObject obj_loc_2d = obj["loc_2d"].toObject();
        {
            LOC_SURFEL_NUM = obj_loc_2d["LOC_SURFEL_NUM"].toString().toInt();
            printf("[CONFIG] LOC_SURFEL_NUM, %s\n", obj_loc_2d["LOC_SURFEL_NUM"].toString().toLocal8Bit().data());

            LOC_SURFEL_RANGE = obj_loc_2d["LOC_SURFEL_RANGE"].toString().toDouble();
            printf("[CONFIG] LOC_SURFEL_RANGE, %s\n", obj_loc_2d["LOC_SURFEL_RANGE"].toString().toLocal8Bit().data());

            LOC_ICP_COST_THRESHOLD_0 = obj_loc_2d["LOC_ICP_COST_THRESHOLD_0"].toString().toDouble();
            printf("[CONFIG] LOC_ICP_COST_THRESHOLD_0, %s\n", obj_loc_2d["LOC_ICP_COST_THRESHOLD_0"].toString().toLocal8Bit().data());

            LOC_ICP_COST_THRESHOLD = obj_loc_2d["LOC_ICP_COST_THRESHOLD"].toString().toDouble();
            printf("[CONFIG] LOC_ICP_COST_THRESHOLD, %s\n", obj_loc_2d["LOC_ICP_COST_THRESHOLD"].toString().toLocal8Bit().data());

            LOC_ICP_ERROR_THRESHOLD = obj_loc_2d["LOC_ICP_ERROR_THRESHOLD"].toString().toDouble();
            printf("[CONFIG] LOC_ICP_ERROR_THRESHOLD, %s\n", obj_loc_2d["LOC_ICP_ERROR_THRESHOLD"].toString().toLocal8Bit().data());

            LOC_ICP_MAX_FEATURE_NUM = obj_loc_2d["LOC_ICP_MAX_FEATURE_NUM"].toString().toInt();
            printf("[CONFIG] LOC_ICP_MAX_FEATURE_NUM, %s\n", obj_loc_2d["LOC_ICP_MAX_FEATURE_NUM"].toString().toLocal8Bit().data());

            LOC_CHECK_DIST = obj_loc_2d["LOC_CHECK_DIST"].toString().toDouble();
            printf("[CONFIG] LOC_CHECK_DIST, %s\n", obj_loc_2d["LOC_CHECK_DIST"].toString().toLocal8Bit().data());

            LOC_CHECK_IE = obj_loc_2d["LOC_CHECK_IE"].toString().toDouble();
            printf("[CONFIG] LOC_CHECK_IE, %s\n", obj_loc_2d["LOC_CHECK_IE"].toString().toLocal8Bit().data());

            LOC_CHECK_IR = obj_loc_2d["LOC_CHECK_IR"].toString().toDouble();
            printf("[CONFIG] LOC_CHECK_IR, %s\n", obj_loc_2d["LOC_CHECK_IR"].toString().toLocal8Bit().data());

            LOC_ICP_ODO_FUSION_RATIO = obj_loc_2d["LOC_ICP_ODO_FUSION_RATIO"].toString().toDouble();
            printf("[CONFIG] LOC_ICP_ODO_FUSION_RATIO, %s\n", obj_loc_2d["LOC_ICP_ODO_FUSION_RATIO"].toString().toLocal8Bit().data());

            LOC_ARUCO_ODO_FUSION_RATIO = obj_loc_2d["LOC_ARUCO_ODO_FUSION_RATIO"].toString().toDouble();
            printf("[CONFIG] LOC_ARUCO_ODO_FUSION_RATIO, %s\n", obj_loc_2d["LOC_ARUCO_ODO_FUSION_RATIO"].toString().toLocal8Bit().data());

            LOC_ARUCO_ODO_FUSION_DIST = obj_loc_2d["LOC_ARUCO_ODO_FUSION_DIST"].toString().toDouble();
            printf("[CONFIG] LOC_ARUCO_ODO_FUSION_DIST, %s\n", obj_loc_2d["LOC_ARUCO_ODO_FUSION_DIST"].toString().toLocal8Bit().data());
        }
    }
    else if(LOC_MODE == "3D")
    {
        QJsonObject obj_loc_3d = obj["loc_3d"].toObject();
        {
            LOC_MAX_FEATURE_NUM = obj_loc_3d["LVX_MAX_FEATURE_NUM"].toString().toInt();
            printf("[CONFIG] LOC_MAX_FEATURE_NUM, %s\n", obj_loc_3d["LOC_MAX_FEATURE_NUM"].toString().toLocal8Bit().data());

            LOC_SURFEL_NN_NUM = obj_loc_3d["LOC_SURFEL_NN_NUM"].toString().toInt();
            printf("[CONFIG] LOC_SURFEL_NN_NUM, %s\n", obj_loc_3d["LOC_SURFEL_NN_NUM"].toString().toLocal8Bit().data());

            LOC_SURFEL_RANGE = obj_loc_3d["LOC_SURFEL_RANGE"].toString().toDouble();
            printf("[CONFIG] LOC_SURFEL_RANGE, %s\n", obj_loc_3d["LOC_SURFEL_RANGE"].toString().toLocal8Bit().data());

            LOC_SURFEL_BALANCE = obj_loc_3d["LOC_SURFEL_BALANCE"].toString().toDouble();
            printf("[CONFIG] LOC_SURFEL_BALANCE, %s\n", obj_loc_3d["LOC_SURFEL_BALANCE"].toString().toLocal8Bit().data());

            LOC_COST_THRESHOLD = obj_loc_3d["LOC_COST_THRESHOLD"].toString().toDouble();
            printf("[CONFIG] LOC_COST_THRESHOLD, %s\n", obj_loc_3d["LOC_COST_THRESHOLD"].toString().toLocal8Bit().data());

            LOC_INLIER_CHECK_DIST = obj_loc_3d["LOC_INLIER_CHECK_DIST"].toString().toDouble();
            printf("[CONFIG] LOC_INLIER_CHECK_DIST, %s\n", obj_loc_3d["LOC_INLIER_CHECK_DIST"].toString().toLocal8Bit().data());

            LOC_ICP_ODO_FUSION_RATIO = obj_loc_3d["LOC_ICP_ODO_FUSION_RATIO"].toString().toDouble();
            printf("[CONFIG] LOC_ICP_ODO_FUSION_RATIO, %s\n", obj_loc_3d["LOC_ICP_ODO_FUSION_RATIO"].toString().toLocal8Bit().data());
        }
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

    QJsonObject obj_obs = obj["obs"].toObject();
    {
        OBS_AVOID = obj_obs["OBS_AVOID"].toString().toInt();
        printf("[CONFIG] OBS_AVOID, %s\n", obj_obs["OBS_AVOID"].toString().toLocal8Bit().data());

        OBS_DEADZONE = obj_obs["OBS_DEADZONE"].toString().toDouble();
        printf("[CONFIG] OBS_DEADZONE, %s\n", obj_obs["OBS_DEADZONE"].toString().toLocal8Bit().data());

        OBS_DEADZONE_DYN = obj_obs["OBS_DEADZONE_DYN"].toString().toDouble();
        printf("[CONFIG] OBS_DEADZONE_DYN, %s\n", obj_obs["OBS_DEADZONE_DYN"].toString().toLocal8Bit().data());

        OBS_DEADZONE_VIR = obj_obs["OBS_DEADZONE_VIR"].toString().toDouble();
        printf("[CONFIG] OBS_DEADZONE_VIR, %s\n", obj_obs["OBS_DEADZONE_VIR"].toString().toLocal8Bit().data());

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
        DOCKING_GOAL_D = obj_dock["DOCKING_GOAL_D"].toString().toDouble();
        printf("[CONFIG] DOCKING_GOAL_D, %s\n", obj_dock["DOCKING_GOAL_D"].toString().toLocal8Bit().data());

        DOCKING_GOAL_TH = obj_dock["DOCKING_GOAL_TH"].toString().toDouble();
        printf("[CONFIG] DOCKING_GOAL_TH, %s\n", obj_dock["DOCKING_GOAL_TH"].toString().toLocal8Bit().data());

        DOCKING_EXTENDED_CONTROL_TIME = obj_dock["DOCKING_EXTENDED_CONTROL_TIME"].toString().toDouble();
        printf("[CONFIG] DOCKING_EXTENDED_CONTROL_TIME, %s\n", obj_dock["DOCKING_EXTENDED_CONTROL_TIME"].toString().toLocal8Bit().data());

        DOCKING_POINTDOCK_MARGIN = obj_dock["DOCKING_POINTDOCK_MARGIN"].toString().toDouble();
        printf("[CONFIG] DOCKING_POINTDOCK_MARGIN, %s\n", obj_dock["DOCKING_POINTDOCK_MARGIN"].toString().toLocal8Bit().data());

        DOCKING_TYPE = obj_dock["DOCKING_TYPE"].toString().toInt();
        printf("[CONFIG] DOCKING_TYPE, %s\n", obj_dock["DOCKING_TYPE"].toString().toLocal8Bit().data());
    }

    QJsonObject obj_map = obj["map"].toObject();
    {
        MAP_PATH = obj_map["MAP_PATH"].toString();
        printf("[CONFIG] MAP_PATH, %s\n", obj_map["MAP_PATH"].toString().toLocal8Bit().data());
    }

    // complete
    is_load = true;
    config_file.close();
    printf("[CONFIG] %s, load successed\n", qUtf8Printable(config_path));
}

void CONFIG::set_map_path(QString path)
{
    if(config_path.isEmpty())
    {
        return;
    }

    MAP_PATH = path;

    QMutexLocker locker(&mtx);
    QFile config_file(config_path);

    if(!config_file.open(QIODevice::ReadWrite))
    {
        printf("[config] failed to open config file for reading and writing.\n");
        return;
    }

    QString data;
    {
        QTextStream in(&config_file);
        data = in.readAll();
        config_file.close();
    }

    QString pattern = R"("MAP_PATH"\s*:\s*".*?")";
    QRegularExpression re(pattern);
    QRegularExpressionMatch match = re.match(data);

    if(match.hasMatch())
    {
        data.replace(re, R"("MAP_PATH": ")" + path + R"(")");
    }
    else
    {
        QRegularExpression mapRe(R"("map"\s*:\s*\{)");
        QRegularExpressionMatch mapMatch = mapRe.match(data);
        if (mapMatch.hasMatch())
        {
            int insertPos = mapMatch.capturedEnd();
            data.insert(insertPos, QString(R"(
            "MAP_PATH": ")") + path + R"(",)");
        }
    }

    if(!config_file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        printf("[config] failed to open config file for writing.\n");
        return;
    }

    QTextStream out(&config_file);
    out << data;
    config_file.close();
}
