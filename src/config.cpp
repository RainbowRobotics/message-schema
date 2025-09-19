#include "config.h"

CONFIG* CONFIG::instance(QObject* parent)
{
    static CONFIG* _instance = nullptr;
    if(!_instance)
    {
        _instance = new CONFIG(parent);
    }
    return _instance;
}

CONFIG::CONFIG(QObject* parent) : QObject(parent)
{

}

CONFIG::~CONFIG()
{

}

void CONFIG::load_version()
{
    QFileInfo version_info(path_version);
    if(version_info.exists() && version_info.isFile())
    {
        QFile version_file(path_version);
        if(version_file.open(QIODevice::ReadOnly))
        {
            QByteArray data = version_file.readAll();
            VERSION_INFO = QString(data);
        }
    }
}

void CONFIG::load_cam_serial_number()
{
    // read config
    QFile config_sn_file(path_cam_serial_number);
    if(config_sn_file.open(QIODevice::ReadOnly))
    {
        QByteArray data = config_sn_file.readAll();
        QJsonDocument doc = QJsonDocument::fromJson(data);
        QJsonObject obj = doc.object();

        QJsonObject obj_cam = obj["cam"].toObject();
        {
            int cam_cnt = get_cam_num();
            for(int p = 0; p < cam_cnt; p++)
            {
                CAM_SERIAL_NUMBER[p] = obj_cam[QString("CAM_SERIAL_NUMBER_%1").arg(p)].toString();
                printf("[CONFIG] CAM_SERIAL_NUMBER_%d, %s\n", p, obj_cam[QString("CAM_SERIAL_NUMBER_%1").arg(p)].toString().toLocal8Bit().data());
            }
        }

        QJsonObject obj_robot = obj["robot"].toObject();
        {
            ROBOT_SERIAL_NUMBER = obj_robot["ROBOT_SERIAL_NUMBER"].toString();
            printf("[CONFIG] ROBOT_SERIAL_NUMBER, %s\n", obj_robot["ROBOT_SERIAL_NUMBER"].toString().toLocal8Bit().data());
        }

        // complete
        is_load = true;
        config_sn_file.close();
        printf("[CONFIG] %s, load successed\n", path_cam_serial_number.toLocal8Bit().data());
    }
}

void CONFIG::load()
{
    missing_variables.clear();

    QFile config_file(path_config);
    if(!config_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "[CONFIG] Failed to open config file:" << path_config;
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

    load_robot_config(obj);
    load_sensors_config(obj);
    load_localization_config(obj);
    load_localization_2d_config(obj);
    load_localization_3d_config(obj);
    load_network_config(obj);
    load_debug_config(obj);
    load_logging_config(obj);
    load_motor_config(obj);
    load_mapping_config(obj);
    load_obstacle_config(obj);
    load_control_config(obj);
    load_docking_config(obj);
    load_map_config(obj);
    load_lidar_configs(obj);
    load_camera_configs(obj);
    load_sensor_specific_configs(obj);

    is_load = true;
    printf("[CONFIG] %s, load successed\n", qUtf8Printable(path_config));

    // spdlog 레벨 설정
    setup_spdlog_level();

    if(has_missing_variables())
    {
        show_missing_variables_dialog();
    }
}

void CONFIG::load_robot_config(const QJsonObject &obj)
{
    QJsonObject obj_robot = obj["robot"].toObject();

    check_and_set_string(obj_robot, "MILEAGE",        MILEAGE,      "robot");
    check_and_set_bool  (obj_robot, "SPEAKER",        USE_SPEAKER,  "robot");

    check_and_set_double(obj_robot, "ROBOT_SIZE_MIN_X",     ROBOT_SIZE_X[0],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_MAX_X",     ROBOT_SIZE_X[1],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_MIN_Y",     ROBOT_SIZE_Y[0],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_MAX_Y",     ROBOT_SIZE_Y[1],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_MIN_Z",     ROBOT_SIZE_Z[0],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_MAX_Z",     ROBOT_SIZE_Z[1],    "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_ADD_X",     ROBOT_SIZE_ADD_X,   "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_ADD_Y",     ROBOT_SIZE_ADD_Y,   "robot");
    check_and_set_double(obj_robot, "ROBOT_SIZE_ADD_Z",     ROBOT_SIZE_ADD_Z,   "robot");
    check_and_set_double(obj_robot, "ROBOT_WHEEL_RADIUS",   ROBOT_WHEEL_RADIUS, "robot");
    check_and_set_double(obj_robot, "ROBOT_WHEEL_BASE",     ROBOT_WHEEL_BASE,   "robot");

    double lx = std::max<double>(std::abs(ROBOT_SIZE_X[0]), std::abs(ROBOT_SIZE_X[1]));
    double ly = std::max<double>(std::abs(ROBOT_SIZE_Y[0]), std::abs(ROBOT_SIZE_Y[1]));
    ROBOT_RADIUS = std::sqrt(lx * lx + ly * ly);
    printf("[CONFIG] ROBOT_RADIUS(auto calc), %.3f\n", ROBOT_RADIUS);
}

void CONFIG::load_sensors_config(const QJsonObject &obj)
{
    QJsonObject obj_sensors = obj["sensors"].toObject();

    check_and_set_bool(obj_sensors,   "USE_LIDAR_2D",   USE_LIDAR_2D,  "sensors");
    check_and_set_bool(obj_sensors,   "USE_LIDAR_3D",   USE_LIDAR_3D,  "sensors");
    check_and_set_bool(obj_sensors,   "USE_CAM",        USE_CAM,       "sensors");
    check_and_set_bool(obj_sensors,   "USE_BQR",        USE_BQR,       "sensors");
    check_and_set_bool(obj_sensors,   "USE_IMU",        USE_IMU,       "sensors");
    check_and_set_int(obj_sensors,    "LIDAR_2D_NUM",   LIDAR_2D_NUM,  "sensors");
    check_and_set_int(obj_sensors,    "LIDAR_3D_NUM",   LIDAR_3D_NUM,  "sensors");
    check_and_set_int(obj_sensors,    "CAM_NUM",        CAM_NUM,       "sensors");
    check_and_set_string(obj_sensors, "LIDAR_2D_TYPE",  LIDAR_2D_TYPE, "sensors");
    check_and_set_string(obj_sensors, "LIDAR_3D_TYPE",  LIDAR_3D_TYPE, "sensors");
    check_and_set_string(obj_sensors, "CAM_TYPE",       CAM_TYPE,      "sensors");
}

void CONFIG::load_localization_config(const QJsonObject &obj)
{
    QJsonObject obj_loc = obj["localization"].toObject();

    check_and_set_string(obj_loc,   "MODE",             LOC_MODE,       "localization");
    check_and_set_bool(obj_loc,     "USE_ARUCO",        USE_ARUCO,      "localization");
}

void CONFIG::load_localization_2d_config(const QJsonObject &obj)
{
    QJsonObject obj_loc = obj["loc_2d"].toObject();

    check_and_set_int(obj_loc,    "LOC_SURFEL_NUM",             LOC_2D_SURFEL_NUM,             "loc_2d");
    check_and_set_int(obj_loc,    "LOC_ICP_MAX_FEATURE_NUM",    LOC_2D_ICP_MAX_FEATURE_NUM,    "loc_2d");
    check_and_set_double(obj_loc, "LOC_ARUCO_ODO_FUSION_DIST",  LOC_2D_ARUCO_ODO_FUSION_DIST,  "loc_2d");
    check_and_set_double(obj_loc, "LOC_ARUCO_ODO_FUSION_RATIO", LOC_2D_ARUCO_ODO_FUSION_RATIO, "loc_2d");
    check_and_set_double(obj_loc, "LOC_CHECK_DIST",             LOC_2D_CHECK_DIST,             "loc_2d");
    check_and_set_double(obj_loc, "LOC_CHECK_IE",               LOC_2D_CHECK_IE,               "loc_2d");
    check_and_set_double(obj_loc, "LOC_CHECK_IR",               LOC_2D_CHECK_IR,               "loc_2d");
    check_and_set_double(obj_loc, "LOC_ICP_COST_THRESHOLD",     LOC_2D_ICP_COST_THRESHOLD,     "loc_2d");
    check_and_set_double(obj_loc, "LOC_ICP_COST_THRESHOLD_0",   LOC_2D_ICP_COST_THRESHOLD_0,   "loc_2d");
    check_and_set_double(obj_loc, "LOC_ICP_ERROR_THRESHOLD",    LOC_2D_ICP_ERROR_THRESHOLD,    "loc_2d");
    check_and_set_double(obj_loc, "LOC_ICP_ODO_FUSION_RATIO",   LOC_2D_ICP_ODO_FUSION_RATIO,   "loc_2d");
    check_and_set_double(obj_loc, "LOC_SURFEL_RANGE",           LOC_2D_SURFEL_RANGE,           "loc_2d");
}

void CONFIG::load_localization_3d_config(const QJsonObject &obj)
{
    QJsonObject obj_loc = obj["loc_3d"].toObject();

    check_and_set_int(obj_loc,    "LOC_MAX_FEATURE_NUM",        LOC_MAX_FEATURE_NUM,      "loc_3d");
    check_and_set_int(obj_loc,    "LOC_SURFEL_NN_NUM",          LOC_SURFEL_NN_NUM,        "loc_3d");
    check_and_set_double(obj_loc, "LOC_SURFEL_BALANCE",         LOC_SURFEL_BALANCE,       "loc_3d");
    check_and_set_double(obj_loc, "LOC_COST_THRESHOLD",         LOC_COST_THRESHOLD,       "loc_3d");
    check_and_set_double(obj_loc, "LOC_INLIER_CHECK_DIST",      LOC_INLIER_CHECK_DIST,    "loc_3d");
}

void CONFIG::load_network_config(const QJsonObject &obj)
{
    QJsonObject obj_net = obj["network"].toObject();

    check_and_set_bool(obj_net, "USE_MULTI", USE_MULTI,     "network");
    check_and_set_bool(obj_net, "USE_COOP",  USE_COMM_COOP, "network");
    check_and_set_bool(obj_net, "USE_RTSP",  USE_COMM_RTSP, "network");
    check_and_set_bool(obj_net, "USE_RRS",   USE_COMM_RRS,  "network");
    check_and_set_bool(obj_net, "USE_FMS",   USE_COMM_FMS,  "network");
}

void CONFIG::load_debug_config(const QJsonObject &obj)
{
    QJsonObject obj_debug = obj["debug"].toObject();

    check_and_set_bool(obj_debug,   "USE_SIM",   USE_SIM,   "debug");
    check_and_set_bool(obj_debug,   "USE_BEEP",  USE_BEEP,  "debug");
    check_and_set_string(obj_debug, "SERVER_IP", SERVER_IP, "debug");
    check_and_set_string(obj_debug, "SERVER_ID", SERVER_ID, "debug");
    check_and_set_string(obj_debug, "SERVER_PW", SERVER_PW, "debug");
}

void CONFIG::load_logging_config(const QJsonObject &obj)
{
    QJsonObject obj_logging = obj["logging"].toObject();

    check_and_set_string(obj_logging, "LOG_LEVEL", LOG_LEVEL, "logging");
    check_and_set_bool(obj_logging, "DEBUG_LIDAR_2D", DEBUG_LIDAR_2D, "logging");
    check_and_set_bool(obj_logging, "DEBUG_MOBILE", DEBUG_MOBILE, "logging");
    check_and_set_bool(obj_logging, "DEBUG_COMM_RRS", DEBUG_COMM_RRS, "logging");
    check_and_set_bool(obj_logging, "LOG_ENABLE_FILE_OUTPUT", LOG_ENABLE_FILE_OUTPUT, "logging");
    check_and_set_string(obj_logging, "LOG_FILE_PATH", LOG_FILE_PATH, "logging");
}

void CONFIG::load_motor_config(const QJsonObject &obj)
{
    QJsonObject obj_motor = obj["motor"].toObject();

    check_and_set_int(obj_motor,    "MOTOR_ID_L",        MOTOR_ID_L,        "motor");
    check_and_set_int(obj_motor,    "MOTOR_ID_R",        MOTOR_ID_R,        "motor");
    check_and_set_double(obj_motor, "MOTOR_DIR",         MOTOR_DIR,         "motor");
    check_and_set_double(obj_motor, "MOTOR_GEAR_RATIO",  MOTOR_GEAR_RATIO,  "motor");
    check_and_set_double(obj_motor, "MOTOR_LIMIT_V",     MOTOR_LIMIT_V,     "motor");
    check_and_set_double(obj_motor, "MOTOR_LIMIT_V_ACC", MOTOR_LIMIT_V_ACC, "motor");
    check_and_set_double(obj_motor, "MOTOR_LIMIT_W",     MOTOR_LIMIT_W,     "motor");
    check_and_set_double(obj_motor, "MOTOR_LIMIT_W_ACC", MOTOR_LIMIT_W_ACC, "motor");
    check_and_set_double(obj_motor, "MOTOR_GAIN_KP",     MOTOR_GAIN_KP,     "motor");
    check_and_set_double(obj_motor, "MOTOR_GAIN_KI",     MOTOR_GAIN_KI,     "motor");
    check_and_set_double(obj_motor, "MOTOR_GAIN_KD",     MOTOR_GAIN_KD,     "motor");
    check_and_set_double(obj_motor, "MOTOR_SAFETY_LIMIT_V",     MOTOR_SAFETY_LIMIT_V,     "motor");
    check_and_set_double(obj_motor, "MOTOR_SAFETY_LIMIT_W",     MOTOR_SAFETY_LIMIT_W,     "motor");


}

void CONFIG::load_mapping_config(const QJsonObject &obj)
{
    QJsonObject obj_mapping = obj["mapping"].toObject();

    check_and_set_int(obj_mapping,      "SLAM_WINDOW_SIZE",         MAPPING_WINDOW_SIZE,         "mapping");
    check_and_set_double(obj_mapping,   "SLAM_VOXEL_SIZE",          MAPPING_VOXEL_SIZE,          "mapping");
    check_and_set_double(obj_mapping,   "SLAM_ICP_COST_THRESHOLD",  MAPPING_ICP_COST_THRESHOLD,  "mapping");
    check_and_set_double(obj_mapping,   "SLAM_ICP_ERROR_THRESHOLD", MAPPING_ICP_ERROR_THRESHOLD, "mapping");
    check_and_set_int(obj_mapping,      "SLAM_ICP_MAX_FEATURE_NUM", MAPPING_ICP_MAX_FEATURE_NUM, "mapping");
    check_and_set_int(obj_mapping,      "SLAM_ICP_DO_ERASE_GAP",    MAPPING_ICP_DO_ERASE_GAP,    "mapping");
    check_and_set_int(obj_mapping,      "SLAM_ICP_DO_ACCUM_NUM",    MAPPING_ICP_DO_ACCUM_NUM,    "mapping");
    check_and_set_double(obj_mapping,   "SLAM_ICP_VIEW_THRESHOLD",  MAPPING_ICP_VIEW_THRESHOLD,  "mapping");
    check_and_set_int(obj_mapping,      "SLAM_KFRM_UPDATE_NUM",     MAPPING_KFRM_UPDATE_NUM,     "mapping");
    check_and_set_double(obj_mapping,   "SLAM_KFRM_LC_TRY_DIST",    MAPPING_KFRM_LC_TRY_DIST,    "mapping");
    check_and_set_double(obj_mapping,   "SLAM_KFRM_LC_TRY_OVERLAP", MAPPING_KFRM_LC_TRY_OVERLAP, "mapping");
}

void CONFIG::load_obstacle_config(const QJsonObject &obj)
{
    QJsonObject obj_obs = obj["obs"].toObject();

    check_and_set_int(obj_obs,    "OBS_AVOID",         OBS_AVOID,         "obs");
    check_and_set_double(obj_obs, "OBS_DEADZONE",      OBS_DEADZONE,      "obs");
    check_and_set_double(obj_obs, "OBS_LOCAL_GOAL_D",  OBS_LOCAL_GOAL_D,  "obs");
    check_and_set_double(obj_obs, "OBS_SAFE_MARGIN_X", OBS_SAFE_MARGIN_X, "obs");
    check_and_set_double(obj_obs, "OBS_SAFE_MARGIN_Y", OBS_SAFE_MARGIN_Y, "obs");
    check_and_set_double(obj_obs, "OBS_PATH_MARGIN_X", OBS_PATH_MARGIN_X, "obs");
    check_and_set_double(obj_obs, "OBS_PATH_MARGIN_Y", OBS_PATH_MARGIN_Y, "obs");
    check_and_set_double(obj_obs, "OBS_MAP_GRID_SIZE", OBS_MAP_GRID_SIZE, "obs");
    check_and_set_double(obj_obs, "OBS_MAP_RANGE",     OBS_MAP_RANGE,     "obs");
    check_and_set_double(obj_obs, "OBS_MAP_MIN_V",     OBS_MAP_MIN_V,     "obs");
    check_and_set_double(obj_obs, "OBS_MAP_MIN_Z",     OBS_MAP_MIN_Z,     "obs");
    check_and_set_double(obj_obs, "OBS_MAP_MAX_Z",     OBS_MAP_MAX_Z,     "obs");
    check_and_set_double(obj_obs, "OBS_PREDICT_TIME",  OBS_PREDICT_TIME,  "obs");
}

void CONFIG::load_control_config(const QJsonObject &obj)
{
    QJsonObject obj_control = obj["control"].toObject();

    check_and_set_double(obj_control, "DRIVE_GOAL_APPROACH_GAIN", DRIVE_GOAL_APPROACH_GAIN, "control");
    check_and_set_double(obj_control, "DRIVE_GOAL_D", DRIVE_GOAL_D, "control");
    check_and_set_double(obj_control, "DRIVE_GOAL_TH", DRIVE_GOAL_TH, "control");
    check_and_set_double(obj_control, "DRIVE_EXTENDED_CONTROL_TIME", DRIVE_EXTENDED_CONTROL_TIME, "control");
    check_and_set_double(obj_control, "DRIVE_V_DEADZONE", DRIVE_V_DEADZONE, "control");
    check_and_set_double(obj_control, "DRIVE_W_DEADZONE", DRIVE_W_DEADZONE, "control");
}

void CONFIG::load_docking_config(const QJsonObject &obj)
{
    QJsonObject obj_dock = obj["docking"].toObject();

    check_and_set_int(obj_dock, "DOCKING_TYPE", DOCKING_TYPE, "docking");
    check_and_set_double(obj_dock, "DOCKING_POINTDOCK_MARGIN", DOCKING_POINTDOCK_MARGIN, "docking");
    check_and_set_double(obj_dock, "DOCKING_GOAL_D", DOCKING_GOAL_D, "docking");
    check_and_set_double(obj_dock, "DOCKING_GOAL_TH", DOCKING_GOAL_TH, "docking");
    check_and_set_double(obj_dock, "DOCKING_KP_d", DOCKING_KP_d, "docking");
    check_and_set_double(obj_dock, "DOCKING_KD_d", DOCKING_KD_d, "docking");
    check_and_set_double(obj_dock, "DOCKING_KP_th", DOCKING_KP_th, "docking");
    check_and_set_double(obj_dock, "DOCKING_KD_th", DOCKING_KD_th, "docking");
    check_and_set_double(obj_dock, "DOCKING_CLUST_DIST_THRESHOLD_MAX", DOCKING_CLUST_DIST_THRESHOLD_MAX, "docking");
    check_and_set_double(obj_dock, "DOCKING_CLUST_ANGLE_THRESHOLD", DOCKING_CLUST_ANGLE_THRESHOLD, "docking");
    check_and_set_double(obj_dock, "DOCKING_EXTENDED_CONTROL_TIME", DOCKING_EXTENDED_CONTROL_TIME, "docking");
    check_and_set_double(obj_dock, "DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX", DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX, "docking");
    check_and_set_double(obj_dock, "DOCKING_CHG_LENGTH", DOCKING_CHG_LENGTH, "docking");
    check_and_set_double(obj_dock, "DOCKING_ICP_ERR_THRESHOLD", DOCKING_ICP_ERR_THRESHOLD, "docking");
    check_and_set_double(obj_dock, "DOCKING_DWA_YAW_WEIGTH", DOCKING_DWA_YAW_WEIGHT, "docking");
    check_and_set_double(obj_dock, "DOCKING_CHECK_MOTOR_A", DOCKING_CHECK_MOTOR_A, "docking");
    check_and_set_double(obj_dock, "DOCKING_WAITING_TIME", DOCKING_WAITING_TIME, "docking");
    check_and_set_double(obj_dock, "DOCKING_X_OFFSET", DOCKING_X_OFFSET, "docking");
    check_and_set_double(obj_dock, "DOCKING_Y_OFFSET", DOCKING_Y_OFFSET, "docking");
    check_and_set_double(obj_dock, "DOCKING_LINEAR_X_OFFSET", DOCKING_LINEAR_X_OFFSET, "docking");
}

void CONFIG::load_map_config(const QJsonObject &obj)
{
    QJsonObject obj_map = obj["map"].toObject();

    check_and_set_string(obj_map, "MAP_PATH", MAP_PATH, "map");
}

void CONFIG::load_lidar_configs(const QJsonObject &obj)
{
    if(USE_LIDAR_2D)
    {
        QJsonArray lidar_arr = obj["lidar_2d_config"].toArray();
        for(int i = 0; i < LIDAR_2D_NUM; i++)
        {
            QJsonObject obj_lidar_2d = lidar_arr[i].toObject();
            LIDAR_2D_IP[i] = obj_lidar_2d["IP"].toString();
            LIDAR_2D_TF[i] = obj_lidar_2d["TF"].toString();
            LIDAR_2D_DEV[i] = obj_lidar_2d["DEV"].toString();
            printf("[CONFIG] LIDAR_2D[%d] IP: %s, TF: %s\n", i, qUtf8Printable(LIDAR_2D_IP[i]), qUtf8Printable(LIDAR_2D_TF[i]));
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
            printf("[CONFIG] LIDAR_3D[%d] IP: %s, TF: %s\n", i, qUtf8Printable(LIDAR_3D_IP[i]), qUtf8Printable(LIDAR_3D_TF[i]));
        }
    }
}

void CONFIG::load_camera_configs(const QJsonObject &obj)
{
    if(USE_CAM)
    {
        QJsonArray cam_arr = obj["cam_config"].toArray();
        for(int i = 0; i < cam_arr.size(); i++)
        {
            QJsonObject obj_cam = cam_arr[i].toObject();
            CAM_TF[i] = obj_cam["TF"].toString();
            CAM_SERIAL_NUMBER[i] = obj_cam["SERIAL_NUMBER"].toString();
            printf("[CONFIG] CAM[%d] TF: %s\n", i, qUtf8Printable(CAM_TF[i]));
            printf("[CONFIG] CAM[%d] SERIAL_NUMBER: %s\n", i, CAM_SERIAL_NUMBER[i].toLocal8Bit().data());
        }
    }
}

void CONFIG::load_sensor_specific_configs(const QJsonObject &obj)
{
    if(LIDAR_2D_TYPE == "SICK" && obj.contains("SICK"))
    {
        QJsonObject obj_sick = obj["SICK"].toObject();
        check_and_set_double(obj_sick, "LIDAR_2D_MIN_RANGE", LIDAR_2D_MIN_RANGE, "SICK");
        check_and_set_double(obj_sick, "LIDAR_2D_MAX_RANGE", LIDAR_2D_MAX_RANGE, "SICK");
    }
    else if(LIDAR_2D_TYPE == "LAKI" && obj.contains("LAKI"))
    {
        QJsonObject obj_laki = obj["LAKI"].toObject();
        check_and_set_double(obj_laki, "LIDAR_2D_MIN_RANGE", LIDAR_2D_MIN_RANGE, "LAKI");
        check_and_set_double(obj_laki, "LIDAR_2D_MAX_RANGE", LIDAR_2D_MAX_RANGE, "LAKI");
    }
    else if(LIDAR_2D_TYPE == "RPLIDAR" && obj.contains("RPLIDAR"))
    {
        QJsonObject obj_rplidar = obj["RPLIDAR"].toObject();
        check_and_set_double(obj_rplidar, "LIDAR_2D_MIN_RANGE", LIDAR_2D_MIN_RANGE, "RPLIDAR");
        check_and_set_double(obj_rplidar, "LIDAR_2D_MAX_RANGE", LIDAR_2D_MAX_RANGE, "RPLIDAR");
    }

    if(LIDAR_3D_TYPE == "LIVOX" && obj.contains("LIVOX"))
    {
        QJsonObject obj_lvx = obj["LIVOX"].toObject();
        check_and_set_double(obj_lvx, "LIDAR_3D_MIN_RANGE", LIDAR_3D_MIN_RANGE, "LIVOX");
        check_and_set_double(obj_lvx, "LIDAR_3D_MAX_RANGE", LIDAR_3D_MAX_RANGE, "LIVOX");
    }

    if(CAM_TYPE == "ORBBEC" && obj.contains("ORBBEC"))
    {
        QJsonObject obj_cam = obj["ORBBEC"].toObject();
        check_and_set_double(obj_cam, "CAM_HEIGHT_MAX", CAM_HEIGHT_MAX, "ORBBEC");
        check_and_set_double(obj_cam, "CAM_HEIGHT_MIN", CAM_HEIGHT_MIN, "ORBBEC");
    }
}

void CONFIG::add_missing_variable(const QString& section, const QString& variable)
{
    missing_variables.append(section + "." + variable);
}

void CONFIG::check_and_set_string(const QJsonObject& obj, const QString& key, QString& target, const QString& section)
{
    if(obj.contains(key) && !obj[key].toString().isEmpty())
    {
        target = obj[key].toString();
        printf("[CONFIG] %s, %s\n", qUtf8Printable(key), qUtf8Printable(target));
    }
    else
    {
        add_missing_variable(section, key);
    }
}

void CONFIG::check_and_set_bool(const QJsonObject& obj, const QString& key, bool& target, const QString& section)
{
    if(obj.contains(key))
    {
        target = obj[key].toBool();
        printf("[CONFIG] %s, %s\n", qUtf8Printable(key), target ? "true" : "false");
    }
    else
    {
        add_missing_variable(section, key);
    }
}

void CONFIG::check_and_set_int(const QJsonObject& obj, const QString& key, int& target, const QString& section)
{
    if(obj.contains(key))
    {
        target = obj[key].toString().toInt();
        printf("[CONFIG] %s, %s\n", qUtf8Printable(key), obj[key].toString().toLocal8Bit().data());
    }
    else
    {
        add_missing_variable(section, key);
    }
}

void CONFIG::check_and_set_double(const QJsonObject& obj, const QString& key, double& target, const QString& section)
{
    if(obj.contains(key))
    {
        target = obj[key].toString().toDouble();
        printf("[CONFIG] %s, %s\n", qUtf8Printable(key), obj[key].toString().toLocal8Bit().data());
    }
    else
    {
        add_missing_variable(section, key);
    }
}

QStringList CONFIG::get_missing_variables()
{
    return missing_variables;
}

bool CONFIG::has_missing_variables()
{
    return !missing_variables.isEmpty();
}

void CONFIG::show_missing_variables_dialog()
{
    if(missing_variables.isEmpty())
    {
        return;
    }

    QString message = "Missing variables in config:\n";
    for(const QString& var : missing_variables)
    {
        message += "- " + var + "\n";
    }

    printf("[CONFIG WARNING] %s\n", qUtf8Printable(message));
}

QStringList CONFIG::load_folder_list()
{
    QString version_path = QCoreApplication::applicationDirPath() + "/config";
    QStringList fileList;

    QDir dir(version_path);
    QFileInfoList entries = dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllDirs);

    for (const QFileInfo& item : entries)
    {
        if (item.isDir())
        {
            fileList.append(item.baseName());
        }
    }
    return fileList;
}

void CONFIG::set_config_path(const QString &path)
{
    path_config = path;
}


void CONFIG::set_version_path(const QString &path)
{
    path_version = path;
}

void CONFIG::set_serial_number_path(const QString &path)
{
    path_cam_serial_number = path;
}

bool CONFIG::load_common(QString path)
{
    QFile common_file(path);
    if(!common_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "[CONFIG] Failed to open common file:" << path;
        return false;
    }

    common_path = path;

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
    if(obj.contains("ROBOT_TYPE"))
    {
        ROBOT_TYPE = obj["ROBOT_TYPE"].toString();

        QStringList robot_model_list = ROBOT_TYPE.split("-");
        if(robot_model_list.size() != 0)
        {
            ROBOT_MODEL = robot_model_list[0];
        }
    }

    if(obj.contains("MILEAGE"))
    {
        MILEAGE = obj["MILEAGE"].toString();
    }

    printf("[CONFIG] ROBOT_TYPE: %s, ROBOT_MODEL: %s\n", qUtf8Printable(ROBOT_TYPE), qUtf8Printable(ROBOT_MODEL));

    // complete
    common_file.close();
    return true;
}

void CONFIG::set_map_path(const QString &path)
{
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        MAP_PATH = path;
    }

    // write path
    QMutexLocker locker(&q_mtx);
    QFile config_file(path_config);

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

bool CONFIG::set_value_change(QString key, QString value)
{
    QMutexLocker locker(&q_mtx);

    QFile file(path_config);
    if(!file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    QByteArray fileData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(fileData, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        return false;
    }

    QJsonObject rootObj = jsonDoc.object();

    // define section list
    QStringList sections = {"robot", "motor", "default", "mapping", "loc","annotation", "debug", "control", "docking", "obs", "cam", "fms", "lvx", "map"};
    bool updated = false;

    for (const QString& section : sections)
    {
        if (!rootObj.contains(section) || !rootObj[section].isObject())
        {
            continue;
        }

        QJsonObject subObj = rootObj[section].toObject();


        if (subObj.contains(key))
        {
            subObj[key] = value;
            rootObj[section] = subObj;
            updated = true;

            QJsonDocument updatedDoc(rootObj);

            QFile wfile(path_config);
            if(!wfile.open(QIODevice::WriteOnly))
            {
                return false;
            }
            wfile.write(updatedDoc.toJson(QJsonDocument::Indented));
            wfile.close();

            return true;
        }
    }

    // If the key does not exist in config.json -> modify common.json.
    if (!updated)
    {
        QFile common_file(common_path);
        if (!common_file.open(QIODevice::ReadOnly))
            return false;

        QByteArray common_fileData = common_file.readAll();
        common_file.close();

        QJsonParseError common_parseError;
        QJsonDocument common_jsonDoc = QJsonDocument::fromJson(common_fileData, &common_parseError);
        if (common_parseError.error != QJsonParseError::NoError)
            return false;

        QJsonObject commonObj = common_jsonDoc.object();

        QStringList fileList = load_folder_list();
        bool foundKeyInList = false;

        for (const QString& item : fileList)
        {
            if (item == value)
            {
                foundKeyInList = true;
                break;
            }
        }

        if (foundKeyInList)
        {
            if (commonObj.contains(key))
            {
                commonObj[key] = value;
                QJsonDocument updatedCommonDoc(commonObj);
                QFile wCommonFile(common_path);
                if (!wCommonFile.open(QIODevice::WriteOnly))
                {
                    return false;
                }
                wCommonFile.write(updatedCommonDoc.toJson(QJsonDocument::Indented));
                wCommonFile.close();
                return true;
            }
            else
            {
                return false; // key is not in common.json
            }
        }
    }
    return false;
}


bool CONFIG::set_cam_order(QString CAM_SERIAL_NUMBER[])
{
    QMutexLocker locker(&q_mtx);

    QFile file(path_config);
    //qDebug()<<"path_config : "<<path_config;
    if(!file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    QByteArray fileData = file.readAll();
    file.close();

    QJsonParseError parseError;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(fileData, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        return false;
    }

    QJsonObject rootObj = jsonDoc.object();
    if(!rootObj.contains("cam_config") || !rootObj["cam_config"].isArray())
    {
        return false;
    }

     QJsonArray camArray = rootObj["cam_config"].toArray();


    for (int i = 0; i < get_cam_num(); i++)
    {
        QJsonObject camObj = camArray[i].toObject();
        camObj["SERIAL_NUMBER"] = CAM_SERIAL_NUMBER[i]; // save serial number
        camArray[i] = camObj; // add object to array
    }

    rootObj["cam_config"] = camArray;


    QJsonDocument updatedDoc(rootObj);
    QFile wfile(path_config);
    if(!wfile.open(QIODevice::WriteOnly))
    {
        return false;
    }
    wfile.write(updatedDoc.toJson(QJsonDocument::Indented));
    wfile.close();

    return true;
}

void CONFIG::set_mileage(const QString &mileage)
{
    {
        std::unique_lock<std::shared_mutex> lock(mtx);
        MILEAGE = mileage;
    }

    // write path
    QMutexLocker locker(&q_mtx);
    QFile config_file(common_path);

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


    QString pattern = R"("MILEAGE"\s*:\s*".*?")";
    QRegularExpression re(pattern);
    QRegularExpressionMatch match = re.match(data);

    if(match.hasMatch())
    {
        data.replace(re, R"("MILEAGE": ")" + mileage + R"(")");
    }
    else
    {
        QRegularExpression mapRe(R"("MILEAGE"\s*:\s*\{)");
        QRegularExpressionMatch mapMatch = mapRe.match(data);

        int bracePos = data.indexOf('{');
        if (bracePos != -1)
        {
            int insertPos = bracePos + 1;
            QString insertText = QString("\n    \"MILEAGE\": \"%1\",").arg(mileage);
            data.insert(insertPos, insertText);
            qDebug() << "삽입 후 내용:" << data;
        }
        else
        {
            qDebug() << "[config] 중괄호 { 를 찾을 수 없습니다.";
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

QString CONFIG::get_robot_serial_number()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SERIAL_NUMBER;
}

QString CONFIG::get_robot_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_TYPE;
}

QString CONFIG::get_robot_model()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_MODEL;
}

double CONFIG::get_mileage()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MILEAGE.toDouble();
}

double CONFIG::get_robot_size_x_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_X[0];
}

double CONFIG::get_robot_size_x_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_X[1];
}

double CONFIG::get_robot_size_y_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_Y[0];
}

double CONFIG::get_robot_size_y_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_Y[1];
}

double CONFIG::get_robot_size_z_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_Z[0];
}

double CONFIG::get_robot_size_z_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_Z[1];
}

double CONFIG::get_robot_size_add_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_ADD_X;
}

double CONFIG::get_robot_size_add_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_ADD_Y;
}

double CONFIG::get_robot_size_add_z()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SIZE_ADD_Z;
}

double CONFIG::get_robot_wheel_base()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_WHEEL_BASE;
}

double CONFIG::get_robot_wheel_radius()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_WHEEL_RADIUS;
}

double CONFIG::get_robot_radius()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_RADIUS;
}

bool CONFIG::get_robot_use_speaker()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SPEAKER;
}

bool CONFIG::get_use_lidar_2d()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_LIDAR_2D;
}

QString CONFIG::get_lidar_2d_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_2D_TYPE;
}

QString CONFIG::get_lidar_2d_dev(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_2D_DEV[idx];
}

int CONFIG::get_lidar_2d_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_2D_NUM;
}

bool CONFIG::get_use_lidar_3d()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_LIDAR_3D;
}

QString CONFIG::get_lidar_3d_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_3D_TYPE;
}

int CONFIG::get_lidar_3d_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_3D_NUM;
}

bool CONFIG::get_use_cam()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_CAM;
}

QString CONFIG::get_cam_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_TYPE;
}

int CONFIG::get_cam_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_NUM;
}

bool CONFIG::get_use_bqr()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_BQR;
}

bool CONFIG::get_use_imu()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_IMU;
}

bool CONFIG::get_use_aruco()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_ARUCO;
}

QString CONFIG::get_loc_mode()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_MODE;
}

int CONFIG::get_loc_2d_icp_max_feature_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ICP_MAX_FEATURE_NUM;
}

int CONFIG::get_loc_2d_surfel_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_SURFEL_NUM;
}

double CONFIG::get_loc_2d_icp_odometry_fusion_ratio()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ICP_ODO_FUSION_RATIO;
}

double CONFIG::get_loc_2d_icp_cost_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ICP_COST_THRESHOLD;
}

double CONFIG::get_loc_2d_icp_cost_threshold0()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ICP_COST_THRESHOLD_0;
}

double CONFIG::get_loc_2d_icp_error_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ICP_ERROR_THRESHOLD;
}

double CONFIG::get_loc_2d_aruco_odometry_fusion_ratio()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ARUCO_ODO_FUSION_RATIO;
}

double CONFIG::get_loc_2d_aruco_odometry_fusion_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ARUCO_ODO_FUSION_DIST;
}

double CONFIG::get_loc_2d_surfel_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_SURFEL_RANGE;
}

double CONFIG::get_loc_2d_check_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_CHECK_DIST;
}

double CONFIG::get_loc_2d_check_inlier_ratio()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_CHECK_IR;
}

double CONFIG::get_loc_2d_check_inlier_error()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_CHECK_IE;
}

int CONFIG::get_loc_3d_icp_max_feature_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_MAX_FEATURE_NUM;
}

int CONFIG::get_loc_3d_surfel_nn_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_SURFEL_NN_NUM;
}

double CONFIG::get_loc_3d_surfel_balance()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_SURFEL_BALANCE;
}

double CONFIG::get_loc_3d_cost_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_COST_THRESHOLD;
}

double CONFIG::get_loc_3d_inlier_check_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_INLIER_CHECK_DIST;
}

bool CONFIG::get_use_multi()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_MULTI;
}

bool CONFIG::get_use_coop()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_COMM_COOP;
}

bool CONFIG::get_use_rtsp()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_COMM_RTSP;
}

bool CONFIG::get_use_rrs()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_COMM_RRS;
}

bool CONFIG::get_use_fms()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_COMM_FMS;
}

bool CONFIG::get_use_sim()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SIM;
}

bool CONFIG::get_use_beep()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_BEEP;
}

QString CONFIG::get_server_ip()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return SERVER_IP;
}

QString CONFIG::get_server_id()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return SERVER_ID;
}

QString CONFIG::get_server_pw()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return SERVER_PW;
}

QString CONFIG::get_log_level()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOG_LEVEL;
}

bool CONFIG::set_debug_lidar_2d()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DEBUG_LIDAR_2D;
}

bool CONFIG::set_debug_mobile()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DEBUG_MOBILE;
}

bool CONFIG::set_debug_comm_rrs()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DEBUG_COMM_RRS;
}

bool CONFIG::get_log_enable_file_output()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOG_ENABLE_FILE_OUTPUT;
}

QString CONFIG::get_log_file_path()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOG_FILE_PATH;
}


void CONFIG::setup_spdlog_level()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    
    spdlog::level::level_enum level = spdlog::level::info;
    if (LOG_LEVEL == "trace")
    {
        level = spdlog::level::trace;
    }
    else if (LOG_LEVEL == "debug")
    {
        level = spdlog::level::debug;
    }
    else if (LOG_LEVEL == "info")
    {
        level = spdlog::level::info;
    }
    else if (LOG_LEVEL == "warn")
    {
        level = spdlog::level::warn;
    }
    else if (LOG_LEVEL == "error")
    {
        level = spdlog::level::err;
    }
    else if (LOG_LEVEL == "critical")
    {
        level = spdlog::level::critical;
    }
    else
    {

    }
    
    // spdlog 기본 레벨 설정
    spdlog::set_level(level);
    
    //printf("[CONFIG] spdlog level set to: %s\n", qUtf8Printable(LOG_LEVEL));
    spdlog::info("[CONFIG] spdlog level set to: {}", LOG_LEVEL.toStdString());

}


double CONFIG::get_lidar_2d_min_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_2D_MIN_RANGE;
}

double CONFIG::get_lidar_2d_max_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_2D_MAX_RANGE;
}

QString CONFIG::get_lidar_2d_ip(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < 2)
    {
        return LIDAR_2D_IP[idx];
    }
    return "";
}

QString CONFIG::get_lidar_2d_tf(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < 2)
    {
        return LIDAR_2D_TF[idx];
    }
    return "";
}

double CONFIG::get_lidar_3d_min_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_3D_MIN_RANGE;
}

double CONFIG::get_lidar_3d_max_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LIDAR_3D_MAX_RANGE;
}

QString CONFIG::get_lidar_3d_ip(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < 2)
    {
        return LIDAR_3D_IP[idx];
    }
    return "";
}

QString CONFIG::get_lidar_3d_tf(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < 2)
    {
        return LIDAR_3D_TF[idx];
    }
    return "";
}

double CONFIG::get_cam_height_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_HEIGHT_MIN;
}

double CONFIG::get_cam_height_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_HEIGHT_MAX;
}

QString CONFIG::get_cam_serial_number(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < 4)
    {
        return CAM_SERIAL_NUMBER[idx];
    }
    return "";
}

QString CONFIG::get_cam_tf(int idx)
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    if(idx >= 0 && idx < get_cam_num())
    {
        return CAM_TF[idx];
    }
    return "";
}

int CONFIG::get_motor_id_left()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_ID_L;
}

int CONFIG::get_motor_id_right()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_ID_R;
}

double CONFIG::get_motor_direction()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_DIR;
}

double CONFIG::get_motor_gear_ratio()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_GEAR_RATIO;
}

double CONFIG::get_motor_limit_v()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_LIMIT_V;
}

double CONFIG::get_motor_limit_v_acc()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_LIMIT_V_ACC;
}

double CONFIG::get_motor_limit_w()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_LIMIT_W;
}

double CONFIG::get_motor_limit_w_acc()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_LIMIT_W_ACC;
}

double CONFIG::get_motor_gain_kp()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_GAIN_KP;
}

double CONFIG::get_motor_gain_ki()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_GAIN_KI;
}

double CONFIG::get_motor_gain_kd()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_GAIN_KD;
}

int CONFIG::get_mapping_icp_max_feature_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_MAX_FEATURE_NUM;
}

int CONFIG::get_mapping_icp_do_erase_gap()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_DO_ERASE_GAP;
}

int CONFIG::get_mapping_icp_do_accum_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_DO_ACCUM_NUM;
}

int CONFIG::get_mapping_kfrm_update_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_KFRM_UPDATE_NUM;
}

int CONFIG::get_mapping_window_size()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_WINDOW_SIZE;
}

double CONFIG::get_mapping_icp_cost_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_COST_THRESHOLD;
}

double CONFIG::get_mapping_icp_error_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_ERROR_THRESHOLD;
}

double CONFIG::get_mapping_icp_view_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_ICP_VIEW_THRESHOLD;
}

double CONFIG::get_mapping_kfrm_lc_try_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_KFRM_LC_TRY_DIST;
}

double CONFIG::get_mapping_kfrm_lc_try_overlap()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_KFRM_LC_TRY_OVERLAP;
}

double CONFIG::get_mapping_voxel_size()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAPPING_VOXEL_SIZE;
}

int CONFIG::get_obs_avoid_mode()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_AVOID;
}

double CONFIG::get_obs_deadzone()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_DEADZONE;
}

double CONFIG::get_obs_local_goal_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_LOCAL_GOAL_D;
}

double CONFIG::get_obs_safe_margin_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_SAFE_MARGIN_X;
}

double CONFIG::get_obs_safe_margin_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_SAFE_MARGIN_Y;
}

double CONFIG::get_obs_path_margin_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_PATH_MARGIN_X;
}

double CONFIG::get_obs_path_margin_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_PATH_MARGIN_Y;
}

double CONFIG::get_obs_map_grid_size()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_MAP_GRID_SIZE;
}

double CONFIG::get_obs_map_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_MAP_RANGE;
}

double CONFIG::get_obs_map_min_v()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_MAP_MIN_V;
}

double CONFIG::get_obs_map_min_z()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_MAP_MIN_Z;
}

double CONFIG::get_obs_map_max_z()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_MAP_MAX_Z;
}

double CONFIG::get_obs_predict_time()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_PREDICT_TIME;
}

double CONFIG::get_drive_goal_approach_gain()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_GOAL_APPROACH_GAIN;
}

double CONFIG::get_drive_goal_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_GOAL_D;
}

double CONFIG::get_drive_goal_th()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_GOAL_TH;
}

double CONFIG::get_drive_extended_control_time()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_EXTENDED_CONTROL_TIME;
}

double CONFIG::get_drive_v_deadzone()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_V_DEADZONE;
}

double CONFIG::get_drive_w_deadzone()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DRIVE_W_DEADZONE;
}

int CONFIG::get_docking_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_TYPE;
}

double CONFIG::get_docking_pointdock_margin()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_POINTDOCK_MARGIN;
}

double CONFIG::get_docking_goal_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_GOAL_D;
}

double CONFIG::get_docking_goal_th()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_GOAL_TH;
}

double CONFIG::get_docking_extended_control_time()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_EXTENDED_CONTROL_TIME;
}

double CONFIG::get_docking_kp_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_KP_d;
}

double CONFIG::get_docking_kd_dist()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_KD_d;
}

double CONFIG::get_docking_kp_th()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_KP_th;
}

double CONFIG::get_docking_kd_th()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_KD_th;
}

double CONFIG::get_docking_clust_dist_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CLUST_D_THRESHOLD;
}

double CONFIG::get_docking_clust_dist_threshold_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CLUST_DIST_THRESHOLD_MIN;
}

double CONFIG::get_docking_clust_dist_threshold_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CLUST_DIST_THRESHOLD_MAX;
}

double CONFIG::get_docking_clust_angle_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CLUST_ANGLE_THRESHOLD;
}

double CONFIG::get_docking_size_x_min()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_DOCK_SIZE_X[0];
}

double CONFIG::get_docking_size_x_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_DOCK_SIZE_X[1];
}

double CONFIG::get_docking_icp_err_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_ICP_ERR_THRESHOLD;
}

double CONFIG::get_docking_find_vmark_dist_threshold_max()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX;
}

double CONFIG::get_docking_chg_length()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CHG_LENGTH;
}

double CONFIG::get_docking_dwa_yaw_weight()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_DWA_YAW_WEIGHT;
}

double CONFIG::get_docking_check_motor_a()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_CHECK_MOTOR_A;
}

double CONFIG::get_docking_waiting_time()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_WAITING_TIME;
}

double CONFIG::get_docking_final_icp_err_threshold()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_FINAL_ICP_ERR_THRESHOLD;
}

double CONFIG::get_docking_x_offset()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_X_OFFSET;
}

double CONFIG::get_docking_y_offset()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_Y_OFFSET;
}

double CONFIG::get_docking_linear_x_offset()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_LINEAR_X_OFFSET;
}


double CONFIG::get_motor_safety_limit_v()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_SAFETY_LIMIT_V;
}

double CONFIG::get_motor_safety_limit_w()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MOTOR_SAFETY_LIMIT_W;
}
QString CONFIG::get_map_path()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MAP_PATH;
} 


