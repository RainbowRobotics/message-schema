#include "config.h"

namespace 
{
    const char* MODULE_NAME = "CONFIG";
}

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
                //printf("[CONFIG] CAM_SERIAL_NUMBER_%d, %s\n", p, obj_cam[QString("CAM_SERIAL_NUMBER_%1").arg(p)].toString().toLocal8Bit().data());
                spdlog::info("[CONFIG] CAM_SERIAL_NUMBER_{}: {}", p, qUtf8Printable(CAM_SERIAL_NUMBER[p]));
            }
        }

        QJsonObject obj_robot = obj["robot"].toObject();
        {
            ROBOT_SERIAL_NUMBER = obj_robot["ROBOT_SERIAL_NUMBER"].toString();
            //printf("[CONFIG] ROBOT_SERIAL_NUMBER, %s\n", obj_robot["ROBOT_SERIAL_NUMBER"].toString().toLocal8Bit().data());
            spdlog::info("[CONFIG] ROBOT_SERIAL_NUMBER: {}", qUtf8Printable(ROBOT_SERIAL_NUMBER));
        }

        // complete
        is_load = true;
        config_sn_file.close();
        //printf("[CONFIG] %s, load successed\n", path_cam_serial_number.toLocal8Bit().data());
        spdlog::info("[CONFIG] {}, load succeeded", qUtf8Printable(path_cam_serial_number));
    }
}

void CONFIG::load()
{
    missing_variables.clear();

    QFile config_file(path_config);
    if(!config_file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        //qWarning() << "[CONFIG] Failed to open config file:" << path_config;
        spdlog::warn("[CONFIG] Failed to open config file: {}", qUtf8Printable(path_config));
        return;
    }

    QByteArray data = config_file.readAll();
    config_file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        //qWarning() << "[CONFIG] JSON parse error:" << parseError.errorString();
        spdlog::warn("[CONFIG] JSON parse error: {}", qUtf8Printable(parseError.errorString()));
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
    load_safety_config(obj);
    load_qa_config(obj);
    load_update_config(obj);

    is_load = true;
    //printf("[CONFIG] %s, load successed\n", qUtf8Printable(path_config));
    spdlog::info("[CONFIG] {}, load succeeded", qUtf8Printable(path_config));

    // spdlog level setup
    set_spdlog_level();

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
    check_and_set_string   (obj_robot, "ROBOT_WHEEL_TYPE", ROBOT_WHEEL_TYPE, "robot");

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

    check_and_set_double(obj_robot, "ROBOT_LX", ROBOT_LX, "robot");
    check_and_set_double(obj_robot, "ROBOT_LY", ROBOT_LY, "robot");

    check_and_set_double(obj_robot, "ROBOT_ALARM_BAT_LOW", ROBOT_ALARM_BAT_LOW, "robot");
    check_and_set_double(obj_robot, "ROBOT_ALARM_BAT_CRITICAL", ROBOT_ALARM_BAT_CRITICAL, "robot");

    check_and_set_bool(obj_robot, "ROBOT_ALARM", USE_ROBOT_ALARM, "robot");

    double lx = std::max<double>(std::abs(ROBOT_SIZE_X[0]), std::abs(ROBOT_SIZE_X[1]));
    double ly = std::max<double>(std::abs(ROBOT_SIZE_Y[0]), std::abs(ROBOT_SIZE_Y[1]));
    ROBOT_RADIUS = std::sqrt(lx * lx + ly * ly);
    //printf("[CONFIG] ROBOT_RADIUS(auto calc), %.3f\n", ROBOT_RADIUS);
    spdlog::info("[CONFIG] ROBOT_RADIUS(auto calc): {:.3f}", ROBOT_RADIUS);
}

void CONFIG::load_sensors_config(const QJsonObject &obj)
{
    QJsonObject obj_sensors = obj["sensors"].toObject();

    check_and_set_bool(obj_sensors,   "USE_LIDAR_2D",   USE_LIDAR_2D,  "sensors");
    check_and_set_bool(obj_sensors,   "USE_LIDAR_3D",   USE_LIDAR_3D,  "sensors");
    check_and_set_bool(obj_sensors,   "USE_CAM",        USE_CAM,       "sensors");
    check_and_set_bool(obj_sensors,   "USE_CAM_RGB",    USE_CAM_RGB,   "sensors");
    check_and_set_bool(obj_sensors,   "USE_CAM_DEPTH",  USE_CAM_DEPTH, "sensors");
    check_and_set_bool(obj_sensors,   "USE_CAM_FILTER", USE_CAM_FILTER,"sensors");
    check_and_set_bool(obj_sensors,   "USE_BQR",        USE_BQR,       "sensors");
    check_and_set_bool(obj_sensors,   "USE_IMU",        USE_IMU,       "sensors");
    check_and_set_int(obj_sensors,    "LIDAR_2D_NUM",   LIDAR_2D_NUM,  "sensors");
    check_and_set_int(obj_sensors,    "LIDAR_3D_NUM",   LIDAR_3D_NUM,  "sensors");
    check_and_set_int(obj_sensors,    "CAM_NUM",        CAM_NUM,       "sensors");
    check_and_set_string(obj_sensors, "LIDAR_2D_TYPE",  LIDAR_2D_TYPE, "sensors");
    check_and_set_string(obj_sensors, "LIDAR_3D_TYPE",  LIDAR_3D_TYPE, "sensors");
    check_and_set_string(obj_sensors, "CAM_TYPE",       CAM_TYPE,      "sensors");

    check_and_set_double(obj_sensors, "CAM_FILTER_ROR_RADIUS",        CAM_FILTER_ROR_RADIUS,        "sensors");
    check_and_set_int(obj_sensors,    "CAM_FILTER_ROR_MIN_NEIGHBORS", CAM_FILTER_ROR_MIN_NEIGHBORS, "sensors");
    check_and_set_int(obj_sensors,    "CAM_FILTER_CLUSTER_MIN_SIZE",  CAM_FILTER_CLUSTER_MIN_SIZE,  "sensors");
}

void CONFIG::load_localization_config(const QJsonObject &obj)
{
    QJsonObject obj_loc = obj["localization"].toObject();

    check_and_set_string(obj_loc,   "MODE",             LOC_MODE,       "localization");
    check_and_set_bool(obj_loc,     "USE_ARUCO",        USE_ARUCO,      "localization");
    check_and_set_bool(obj_loc,     "USE_EKF",        USE_EKF,      "localization");
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
    check_and_set_bool(obj_loc,   "LOC_ROTATION_FALLBACK_USE",   LOC_2D_USE_ROTATION_FALLBACK,   "loc_2d");
    check_and_set_double(obj_loc, "LOC_ROTATION_FALLBACK_STEP",  LOC_2D_ROTATION_FALLBACK_STEP,  "loc_2d");
    check_and_set_double(obj_loc, "LOC_ROTATION_FALLBACK_RANGE", LOC_2D_ROTATION_FALLBACK_RANGE, "loc_2d");
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
    check_and_set_bool(obj_net, "USE_MSA",   USE_COMM_MSA,  "network");
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
    check_and_set_bool(obj_logging, "DEBUG_LIDAR_3D", DEBUG_LIDAR_3D, "logging");
    check_and_set_bool(obj_logging, "DEBUG_MOBILE", DEBUG_MOBILE, "logging");
    check_and_set_bool(obj_logging, "DEBUG_COMM_RRS", DEBUG_COMM_RRS, "logging");
    check_and_set_bool(obj_logging, "DEBUG_AUTOCONTROL", DEBUG_MOBILE, "logging");
    check_and_set_bool(obj_logging, "DEBUG_LOCALIZATION", DEBUG_COMM_RRS, "logging");
    check_and_set_bool(obj_logging, "DEBUG_OBSMAP", DEBUG_COMM_RRS, "logging");

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
    check_and_set_double(obj_obs, "OBS_DISTANCE_LED_NEAR",  OBS_DISTANCE_LED_NEAR,  "obs");
    check_and_set_double(obj_obs, "OBS_DISTANCE_LED_FAR",  OBS_DISTANCE_LED_FAR,  "obs");
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
    check_and_set_double(obj_dock, "DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX", DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX, "docking");
    check_and_set_double(obj_dock, "DOCKING_CHG_LENGTH", DOCKING_CHG_LENGTH, "docking");
    check_and_set_double(obj_dock, "DOCKING_ICP_ERR_THRESHOLD", DOCKING_ICP_ERR_THRESHOLD, "docking");
    check_and_set_double(obj_dock, "DOCKING_DWA_YAW_WEIGTH", DOCKING_DWA_YAW_WEIGHT, "docking");
    check_and_set_double(obj_dock, "DOCKING_CHECK_MOTOR_A", DOCKING_CHECK_MOTOR_A, "docking");
    check_and_set_double(obj_dock, "DOCKING_WAITING_TIME", DOCKING_WAITING_TIME, "docking");
    check_and_set_double(obj_dock, "DOCKING_X_OFFSET", DOCKING_X_OFFSET, "docking");
    check_and_set_double(obj_dock, "DOCKING_Y_OFFSET", DOCKING_Y_OFFSET, "docking");
    check_and_set_double(obj_dock, "DOCKING_LINEAR_X_OFFSET", DOCKING_LINEAR_X_OFFSET, "docking");
    check_and_set_bool(obj_dock, "DOCKING_REVERSE_FLAG", DOCKING_REVERSE_FLAG, "docking");
    check_and_set_string(obj_dock, "CHARGE_TYPE", CHARGE_TYPE, "docking");
    check_and_set_int(obj_dock, "DOCKING_FIELD", DOCKING_FIELD, "docking");
    check_and_set_double(obj_dock, "XNERGY_SET_CURRENT", XNERGY_SET_CURRENT, "docking");
}

void CONFIG::load_safety_config(const QJsonObject &obj)
{
    QJsonObject obj_safety = obj["safety"].toObject();
    
    check_and_set_bool(obj_safety, "USE_SAFETY_CROSS_MONITOR", USE_SAFETY_CROSS_MONITOR, "safety");
    check_and_set_bool(obj_safety, "USE_SAFETY_SPEED_CONTROL", USE_SAFETY_SPEED_CONTROL, "safety");
    check_and_set_bool(obj_safety, "USE_SAFETY_OBSTACLE_DETECT", USE_SAFETY_OBSTACLE_DETECT, "safety");
    check_and_set_bool(obj_safety, "USE_SAFETY_BUMPER", USE_SAFETY_BUMPER, "safety");
    check_and_set_bool(obj_safety, "USE_SAFETY_INTERLOCK", USE_SAFETY_INTERLOCK, "safety");
    check_and_set_bool(obj_safety, "USE_MONITORING_FIELD", USE_MONITORING_FIELD, "safety");
    check_and_set_int(obj_safety, "MONITORING_FIELD_NUM", MONITORING_FIELD_NUM, "safety");
    check_and_set_double(obj_safety, "MONITORING_TRAJ_DT", MONITORING_TRAJ_DT, "safety");
    check_and_set_double(obj_safety, "MONITORING_PREDICT_T", MONITORING_PREDICT_T, "safety");
    
    FIELD_SIZE_MIN_X.clear();
    FIELD_SIZE_MAX_X.clear();
    FIELD_SIZE_MIN_Y.clear();
    FIELD_SIZE_MAX_Y.clear();

    if(obj_safety.contains("MONITORING_FIELDS") && obj_safety["MONITORING_FIELDS"].isArray())
    {
        QJsonArray fields_array = obj_safety["MONITORING_FIELDS"].toArray();

        if(fields_array.size() < MONITORING_FIELD_NUM)
        {
            spdlog::info("[CONFIG] MONITORING_FIELD_NUM is not equal to fields_array.size()");
            return;
        }

        FIELD_SIZE_MIN_X.resize(MONITORING_FIELD_NUM);
        FIELD_SIZE_MAX_X.resize(MONITORING_FIELD_NUM);
        FIELD_SIZE_MIN_Y.resize(MONITORING_FIELD_NUM);
        FIELD_SIZE_MAX_Y.resize(MONITORING_FIELD_NUM);

        for(int i = 0; i < MONITORING_FIELD_NUM; i++)
        {
            if(!fields_array[i].isObject())
            {
                spdlog::warn("[CONFIG] MONITORING_FIELDS[{}] is not an object; set zeros", i);
                FIELD_SIZE_MIN_X[i] = 0.0;
                FIELD_SIZE_MAX_X[i] = 0.0;
                FIELD_SIZE_MIN_Y[i] = 0.0;
                FIELD_SIZE_MAX_Y[i] = 0.0;
                continue;
            }

            QJsonObject obj_safety_field = fields_array[i].toObject();
            check_and_set_double(obj_safety_field, "FIELD_SIZE_MIN_X", FIELD_SIZE_MIN_X[i], QString("safety.field[%1]").arg(i));
            check_and_set_double(obj_safety_field, "FIELD_SIZE_MAX_X", FIELD_SIZE_MAX_X[i], QString("safety.field[%1]").arg(i));
            check_and_set_double(obj_safety_field, "FIELD_SIZE_MIN_Y", FIELD_SIZE_MIN_Y[i], QString("safety.field[%1]").arg(i));
            check_and_set_double(obj_safety_field, "FIELD_SIZE_MAX_Y", FIELD_SIZE_MAX_Y[i], QString("safety.field[%1]").arg(i));
        }
    }
    else
    {
        spdlog::warn("[CONFIG] MONITORING_FIELDS not found or not an array");
        MONITORING_FIELD_NUM = 0;
    }
}

void CONFIG::load_qa_config(const QJsonObject &obj)
{
    QJsonObject obj_qa = obj["qa"].toObject();

    check_and_set_double(obj_qa, "QA_STEP_DISTANCE", QA_STEP_DISTANCE, "qa");
    check_and_set_double(obj_qa, "QA_STOP_MIN_DISTANCE", QA_STOP_MIN_DISTANCE, "qa");
}

void CONFIG::load_update_config(const QJsonObject &obj)
{
    QJsonObject obj_update = obj["update"].toObject();

    check_and_set_bool(obj_update, "CONFIG", USE_CONFIG_UPDATE, "update");
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
            //printf("[CONFIG] LIDAR_2D[%d] IP: %s, TF: %s\n", i, qUtf8Printable(LIDAR_2D_IP[i]), qUtf8Printable(LIDAR_2D_TF[i]));
            spdlog::info("[CONFIG] LIDAR_2D[{}] IP: {}, TF: {}, DEV: {}", i, qUtf8Printable(LIDAR_2D_IP[i]), qUtf8Printable(LIDAR_2D_TF[i]), qUtf8Printable(LIDAR_2D_DEV[i]));
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
            //printf("[CONFIG] LIDAR_3D[%d] IP: %s, TF: %s\n", i, qUtf8Printable(LIDAR_3D_IP[i]), qUtf8Printable(LIDAR_3D_TF[i]));
            spdlog::info("[CONFIG] LIDAR_3D[{}] IP: {}, TF: {}", i, qUtf8Printable(LIDAR_3D_IP[i]), qUtf8Printable(LIDAR_3D_TF[i]));
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

            CAM_COLOR_PROFILE[i] = obj_cam["COLOR_PROFILE"].toString().toInt();
            CAM_DEPTH_PROFILE[i] = obj_cam["DEPTH_PROFILE"].toString().toInt();

            spdlog::info("[CONFIG] CAM[{}] TF: {}, SERIAL_NUMBER: {}, COLOR_PROFILE: {}, DEPTH_PROFILE: {}", i,
                            qUtf8Printable(CAM_TF[i]), qUtf8Printable(CAM_SERIAL_NUMBER[i]),
                            CAM_COLOR_PROFILE[i], CAM_DEPTH_PROFILE[i]);
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
        //printf("[CONFIG] %s, %s\n", qUtf8Printable(key), qUtf8Printable(target));
        spdlog::info("[CONFIG] {}, {}", qUtf8Printable(key), qUtf8Printable(target));
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
        //printf("[CONFIG] %s, %s\n", qUtf8Printable(key), target ? "true" : "false");
        spdlog::info("[CONFIG] {}, {}", qUtf8Printable(key), target ? "true" : "false");
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
        //printf("[CONFIG] %s, %s\n", qUtf8Printable(key), obj[key].toString().toLocal8Bit().data());
        spdlog::info("[CONFIG] {}, {}", qUtf8Printable(key), qUtf8Printable(obj[key].toString()));
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
        //printf("[CONFIG] %s, %s\n", qUtf8Printable(key), obj[key].toString().toLocal8Bit().data());
        spdlog::info("[CONFIG] {}, {}", qUtf8Printable(key), qUtf8Printable(obj[key].toString()));
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

    //printf("[CONFIG WARNING] %s\n", qUtf8Printable(message));
    spdlog::warn("[CONFIG] {}", qUtf8Printable(message));
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
        //qWarning() << "[CONFIG] Failed to open common file:" << path;
        spdlog::warn("[CONFIG] Failed to open common file: {}", qUtf8Printable(path));
        return false;
    }

    common_path = path;

    QByteArray data = common_file.readAll();
    common_file.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if(parseError.error != QJsonParseError::NoError)
    {
        //qWarning() << "[CONFIG] common JSON parse error:" << parseError.errorString();
        spdlog::warn("[CONFIG] common JSON parse error: {}", qUtf8Printable(parseError.errorString()));
        return false;
    }

    QJsonObject obj = doc.object();
    if(obj.contains("ROBOT_TYPE"))
    {
        QString robot_type_str = obj["ROBOT_TYPE"].toString();
        if(robot_type_str == "S100-A")
        {
            ROBOT_TYPE = RobotType::S100_A;
        }
        else if(robot_type_str == "S100-B")
        {
            ROBOT_TYPE = RobotType::S100_B;
        }
        else if(robot_type_str == "S100-A-3D")
        {
            ROBOT_TYPE = RobotType::S100_A_3D;
        }
        else if(robot_type_str == "S100-B-3D")
        {
            ROBOT_TYPE = RobotType::S100_B_3D;
        }
        else if(robot_type_str == "D400")
        {
            ROBOT_TYPE = RobotType::D400;
        }
        else if(robot_type_str == "QD")
        {
            ROBOT_TYPE = RobotType::QD;
        }
        else if(robot_type_str == "MECANUM-Q150")
        {
            ROBOT_TYPE = RobotType::MECANUM_Q150;
        }
        else if(robot_type_str == "MECANUM-VALEO")
        {
            ROBOT_TYPE = RobotType::MECANUM_VALEO;
        }
        else if(robot_type_str == "SEM")
        {
            ROBOT_TYPE = RobotType::SEM;
        }
        else if(robot_type_str == "MECANUM-SEC_CORE")
        {
            ROBOT_TYPE = RobotType::SEC_CORE;
        }
        else if(robot_type_str == "DD-SEC_EE")
        {
            ROBOT_TYPE = RobotType::SEC_EE;
        }
        else if(robot_type_str == "SDC")
        {
            ROBOT_TYPE = RobotType::SDC;
        }
        else
        {
            ROBOT_TYPE = RobotType::NONE;
        }

        QString robot_model_str;
        QStringList robot_model_list = robot_type_str.split("-");
        if(robot_model_list.size() != 0)
        {
            robot_model_str = robot_model_list[0];
            if(robot_model_str == "S100")
            {
                ROBOT_MODEL = RobotModel::S100;
            }
            else if(robot_model_str == "D400")
            {
                ROBOT_MODEL = RobotModel::D400;
            }
            else if(robot_model_str == "QD")
            {
                ROBOT_MODEL = RobotModel::QD;
            }
            else if(robot_model_str == "MECANUM")
            {
                ROBOT_MODEL = RobotModel::MECANUM;
            }
            else if(robot_model_str == "DD")
            {
                ROBOT_MODEL = RobotModel::DD;
            }
            else
            {
                ROBOT_MODEL = RobotModel::NONE;
            }
        }
        //printf("[CONFIG] ROBOT_TYPE: %s, ROBOT_MODEL: %s\n", qUtf8Printable(robot_type_str), qUtf8Printable(robot_model_str));
        spdlog::info("[CONFIG] ROBOT_TYPE: {}, ROBOT_MODEL: {}", qUtf8Printable(robot_type_str), qUtf8Printable(robot_model_str));
    }

    if(obj.contains("ROBOT_SERIAL_NUMBER"))
    {
        ROBOT_SERIAL_NUMBER = obj["ROBOT_SERIAL_NUMBER"].toString();
        spdlog::info("[CONFIG] ROBOT_SERIAL_NUMBER: {}", qUtf8Printable(ROBOT_SERIAL_NUMBER));
    }

    if(obj.contains("MILEAGE"))
    {
        MILEAGE = obj["MILEAGE"].toString();
    }

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
        //printf("[config] failed to open config file for reading and writing.\n");
        spdlog::warn("[config] failed to open config file for reading and writing.");
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
        //printf("[config] failed to open config file for writing.\n");
        spdlog::warn("[config] failed to open config file for writing.");
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
    qDebug()<<"path_config : "<<path_config;
    spdlog::warn("[CONFIG] path_config : {}", qUtf8Printable(path_config));
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
        //printf("[config] failed to open config file for reading and writing.\n");
        spdlog::info("[CONFIG] failed to open config file for reading and writing.");
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
            //qDebug() << "삽입 후 내용:" << data;
            spdlog::info("[CONFIG] 삽입 후 내용: {}", data.toStdString());
        }
        else
        {
            //qDebug() << "[config] 중괄호 { 를 찾을 수 없습니다.";
            spdlog::error("[CONFIG] 중괄호 { 를 찾을 수 없습니다.");
        }
    }

    // save in temp file
    QString tempPath = common_path + ".tmp";
    QFile tempFile(tempPath);
    if (!tempFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
    {
        spdlog::error("[CONFIG] failed to open temp file for writing: {}", tempPath.toStdString());
        return;
    }

    {
        QTextStream out(&tempFile);
        out << data;
        tempFile.close();
    }

    if (!QFile::remove(common_path))
    {
        spdlog::warn("[CONFIG] failed to remove old config file: {}", common_path.toStdString());
    }

    if (!QFile::rename(tempPath, common_path))
    {
        spdlog::error("[CONFIG] failed to rename temp file to config file.");
        return;
    }

    spdlog::debug("[CONFIG] config file updated successfully with mileage: {}", mileage.toStdString());

}

bool CONFIG::get_use_safety_cross_monitor()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SAFETY_CROSS_MONITOR;
}

bool CONFIG::get_use_safety_speed_control()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SAFETY_SPEED_CONTROL;
}

bool CONFIG::get_use_safety_obstacle_detect()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SAFETY_OBSTACLE_DETECT;
}

bool CONFIG::get_use_safety_bumper()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SAFETY_BUMPER;
}

bool CONFIG::get_use_safety_interlock()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_SAFETY_INTERLOCK;
}

bool CONFIG::get_use_monitoring_field()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_MONITORING_FIELD;
}

int CONFIG::get_monitoring_field_num()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MONITORING_FIELD_NUM;
}

double CONFIG::get_monitoring_traj_dt()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MONITORING_TRAJ_DT;
}

double CONFIG::get_monitoring_predict_t()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MONITORING_PREDICT_T;
}

std::vector<double> CONFIG::get_field_size_min_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return FIELD_SIZE_MIN_X;
}

std::vector<double> CONFIG::get_field_size_max_x()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return FIELD_SIZE_MAX_X;
}

std::vector<double> CONFIG::get_field_size_min_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return FIELD_SIZE_MIN_Y;
}

std::vector<double> CONFIG::get_field_size_max_y()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return FIELD_SIZE_MAX_Y;
}

QString CONFIG::get_robot_wheel_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_WHEEL_TYPE;
}

QString CONFIG::get_robot_serial_number()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_SERIAL_NUMBER;
}

RobotType CONFIG::get_robot_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_TYPE;
}

RobotModel CONFIG::get_robot_model()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_MODEL;
}

QString CONFIG::get_robot_type_str()
{
    RobotType _robot_type = RobotType::NONE;
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        _robot_type = ROBOT_TYPE;
    }

    if(_robot_type == RobotType::S100_A)
    {
        return "S100-A";
    }
    else if(_robot_type == RobotType::S100_B)
    {
        return "S100-B";
    }
    else if(_robot_type == RobotType::S100_A_3D)
    {
        return "S100-A-3D";
    }
    else if(_robot_type == RobotType::S100_B_3D)
    {
        return "S100-B-3D";
    }
    else if(_robot_type == RobotType::D400)
    {
        return "D400";
    }
    else if(_robot_type == RobotType::SDC)
    {
        return "SDC";
    }
    else if(_robot_type == RobotType::SEM)
    {
        return "SEM";
    }
    else if(_robot_type == RobotType::QD)
    {
        return "QD";
    }
    else if(_robot_type == RobotType::MECANUM_Q150)
    {
        return "MECANUM-Q150";
    }
    else if(_robot_type == RobotType::MECANUM_VALEO)
    {
        return "MECANUM-VALEO";
    }
    else if(_robot_type == RobotType::SEC_CORE)
    {
        return "MECANUM-SEC_CORE";
    }
    else if(_robot_type == RobotType::SEC_EE)
    {
        return "DD-SEC_EE";
    }
    else
    {
        return "NONE";
    }
}

QString CONFIG::get_robot_model_str()
{
    RobotModel _robot_model = RobotModel::NONE;
    {
        std::shared_lock<std::shared_mutex> lock(mtx);
        _robot_model = ROBOT_MODEL;
    }

    if(_robot_model == RobotModel::S100)
    {
        return "S100";
    }
    else if(_robot_model == RobotModel::D400)
    {
        return "D400";
    }
    else if(_robot_model == RobotModel::SDC)
    {
        return "SDC";
    }
    else if(_robot_model == RobotModel::SEM)
    {
        return "SEM";
    }
    else if(_robot_model == RobotModel::QD)
    {
        return "QD";
    }
    else if(_robot_model == RobotModel::MECANUM)
    {
        return "MECANUM";
    }
    else
    {
        return "NONE";
    }
}

double CONFIG::get_mileage()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return MILEAGE.toDouble();
}

double CONFIG::get_robot_lx()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_LX;
}

double CONFIG::get_robot_ly()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_LY;
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

bool CONFIG::get_robot_use_alarm()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_ROBOT_ALARM;
}

double CONFIG::get_robot_alarm_bat_low()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_ALARM_BAT_LOW;
}

double CONFIG::get_robot_alarm_bat_critical()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return ROBOT_ALARM_BAT_CRITICAL;
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

bool CONFIG::get_use_cam_rgb()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_CAM_RGB;
}

bool CONFIG::get_use_cam_depth()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_CAM_DEPTH;
}

bool CONFIG::get_use_cam_filter()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_CAM_FILTER;
}

double CONFIG::get_cam_filter_ror_radius()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_FILTER_ROR_RADIUS;
}

int CONFIG::get_cam_filter_ror_min_neighbors()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_FILTER_ROR_MIN_NEIGHBORS;
}

int CONFIG::get_cam_filter_cluster_min_size()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CAM_FILTER_CLUSTER_MIN_SIZE;
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

bool CONFIG::get_use_ekf()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_EKF;
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

bool CONFIG::get_loc_2d_use_rotation_fallback()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_USE_ROTATION_FALLBACK;
}

double CONFIG::get_loc_2d_rotation_fallback_step()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ROTATION_FALLBACK_STEP;
}

double CONFIG::get_loc_2d_rotation_fallback_range()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return LOC_2D_ROTATION_FALLBACK_RANGE;
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

bool CONFIG::get_use_msa()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_COMM_MSA;
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

bool CONFIG::get_update_use_config()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return USE_CONFIG_UPDATE;
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

void CONFIG::set_spdlog_level()
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
    
    // set spdlog level
    spdlog::set_level(level);
    
    //printf("[CONFIG] spdlog level set to: %s\n", qUtf8Printable(LOG_LEVEL));
    spdlog::info("[CONFIG] spdlog level set to: {}", LOG_LEVEL.toStdString());

}

void CONFIG::set_update_config_file()
{
    QMutexLocker locker(&q_mtx);
    
    if (path_config.isEmpty()) 
    {
        //ERROR_MANAGER::instance()->logError(ERROR_MANAGER::MAP_INVALID_PATH, ERROR_MANAGER::LOAD_CONFIG, "Config path is empty");
        spdlog::error("[CONFIG] Config path is empty");
        return;
    }

    try 
    {
        QString template_path = path_config + ".template";
        if (!QFile::exists(template_path))
        {
            set_default_config_template();
            spdlog::info("[CONFIG] Template file created: {}", template_path.toStdString());
        }

        QJsonObject existing_config;
        QFile config_file(path_config);
        
        if (config_file.exists() && config_file.open(QIODevice::ReadOnly)) 
        {
            QJsonParseError error;
            QJsonDocument doc = QJsonDocument::fromJson(config_file.readAll(), &error);
            config_file.close();
            
            if (error.error == QJsonParseError::NoError && doc.isObject()) 
            {
                existing_config = doc.object();
                spdlog::info("[CONFIG] Existing config loaded successfully");
            }
            else
            {
                spdlog::error("[CONFIG]docking.XNERGY_SET_CURRENT JSON parse error: {}", error.errorString().toStdString());
                return;
            }
        }
        else
        {
            spdlog::warn("[CONFIG] Config file not found, will create new one: {}", path_config.toStdString());
        }

        QJsonObject default_config = set_default_config_object();

        QJsonObject merged_config = merge_config_objects(existing_config, default_config);

        bool has_changes = (existing_config != merged_config);

        if (has_changes) 
        {
            spdlog::info("[CONFIG] Changes detected, creating backup and updating config");

            if (!set_backup_config_file()) 
            {
                spdlog::warn("[CONFIG] Failed to create backup, but continuing with update");
            }

            QFile output_file(path_config);
            if (output_file.open(QIODevice::WriteOnly | QIODevice::Truncate)) 
            {
                QJsonDocument output_doc(merged_config);
                output_file.write(output_doc.toJson(QJsonDocument::Indented));
                output_file.close();
                
                spdlog::info("[CONFIG] Config file updated successfully: {}", path_config.toStdString());
                
                load();
            } 
            else 
            {
                spdlog::error("[CONFIG] Failed to write updated config file: {}", path_config.toStdString());
            }
        }
        else
        {
            spdlog::info("[CONFIG] Config file is already up to date");
        }
    } 
    catch (const std::exception& e) 
    {
        //QString error_msg = QString("Exception during config update: %1").arg(e.what());
        //ERROR_MANAGER::instance()->logError(ERROR_MANAGER::SYS_PROCESS_FINISH_FAILED, ERROR_MANAGER::LOAD_CONFIG, error_msg);
        spdlog::error("[CONFIG] Exception during config update: {}", e.what());
    }
}

QJsonObject CONFIG::merge_config_objects(const QJsonObject& existing, const QJsonObject& defaults)
{
    QJsonObject merged = existing;
    QStringList added_keys;
    
    for (auto it = defaults.begin(); it != defaults.end(); ++it) 
    {
        const QString& key = it.key();
        const QJsonValue& default_value = it.value();
        
        if (!merged.contains(key)) 
        {
            // if key is missing, add default value
            merged[key] = default_value;
            added_keys.append(key);
            spdlog::info("[CONFIG] Added missing config key: {}", key.toStdString());
        } 
        else if (default_value.isObject() && merged[key].isObject()) 
        {
            // if both are objects, merge recursively
            //merged[key] = merge_config_objects(merged[key].toObject(), default_value.toObject());
            QJsonObject child_merged = merge_config_objects
            (
                merged[key].toObject(), 
                default_value.toObject()
            );

            // check if child_merged is different from existing
            if (child_merged != merged[key].toObject())
            {
                merged[key] = child_merged;
                added_keys.append(key + ".*");  //view lower level keys
            }
        }
        //spdlog::info("[CONFIG] Config key Not changed: {}", key.toStdString());
    }

    if (!added_keys.isEmpty()) 
    {
        //QString log_msg = QString("Added missing config keys: %1").arg(added_keys.join(", "));
        //log_info("{}", log_msg.toStdString());
        spdlog::info("[CONFIG] Added missing keys: {}", added_keys.join(", ").toStdString());
    }
    
    return merged;
}


// 1. backup config function
bool CONFIG::set_backup_config_file()
{
    if (path_config.isEmpty() || !QFile::exists(path_config)) 
    {
        return false;
    }

    QString today = QDateTime::currentDateTime().toString("yyyyMMdd");
    QFileInfo config_info(path_config);
    QDir config_dir(config_info.absolutePath());
    
    // today backup file check
    QStringList today_backup_filters;
    today_backup_filters << config_info.baseName() + "_backup_" + today + "*.json";

    QFileInfoList today_backup = config_dir.entryInfoList(today_backup_filters, QDir::Files);

    if (!today_backup.isEmpty()) 
    {
        QString existing_backup = today_backup.first().fileName();
        spdlog::info("[CONFIG] Backup already exists for today: {}", existing_backup.toStdString());
        
        return true; 
    }

    int max_backups = 5;

    // remove old backups
    cleanup_old_backups(config_info, config_dir, max_backups);

    // create new backup
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    QString backup_path = config_info.absolutePath() + "/" + 
                         config_info.baseName() + "_backup_" + timestamp + "." + 
                         config_info.completeSuffix();

    bool success = QFile::copy(path_config, backup_path);
    
    if (success) 
    {
        spdlog::info("[CONFIG] New backup created: {}", backup_path.toStdString());
    }
    else 
    {
        spdlog::error("[CONFIG] Failed to create backup: {}", backup_path.toStdString());
    }
    
    return success;
}

void CONFIG::cleanup_old_backups(const QFileInfo& config_info, const QDir& config_dir, int max_backups)
{
    // find all backup files
    QStringList backup_filters;
    backup_filters << config_info.baseName() + "_backup_*.json";
    
    // aline date order (newest → oldest)
    QFileInfoList all_backups = config_dir.entryInfoList(backup_filters, QDir::Files, QDir::Time);
    
    if (all_backups.size() >= max_backups) 
    {
        // remove files to make space for new backup
        int files_to_remove = all_backups.size() - max_backups + 1; // 새 백업을 위한 공간 확보
        
        for (int i = all_backups.size() - files_to_remove; i < all_backups.size(); i++) 
        {
            QString old_backup_path = all_backups[i].absoluteFilePath();
            if (QFile::remove(old_backup_path)) 
            {
                spdlog::info("[CONFIG] Removed old backup: {}", old_backup_path.toStdString());
            }
            else 
            {
                spdlog::warn("[CONFIG] Failed to remove old backup: {}", old_backup_path.toStdString());
            }
        }
        
        spdlog::info("[CONFIG] Backup cleanup completed. Removed {} old backup(s)", files_to_remove);
    }
}

void CONFIG::set_restore_config_file_backup()
{
    QFileInfo config_info(path_config);
    QDir config_dir(config_info.absolutePath());
    
    QStringList backup_filters;
    backup_filters << config_info.baseName() + "_backup_*.json";
    
    QFileInfoList backup_files = config_dir.entryInfoList(backup_filters, QDir::Files, QDir::Time);
    
    if (!backup_files.isEmpty()) 
    {
        QString latest_backup = backup_files.first().absoluteFilePath();
        
        QFile::remove(path_config);
        QFile::copy(latest_backup, path_config);
        
        spdlog::info("[CONFIG] Config restored from backup: {}", latest_backup.toStdString());
    }
}

void CONFIG::set_default_config_template()
{
    QString template_path = path_config + ".template";
    
    QJsonObject default_config = set_default_config_object();
    
    QFile template_file(template_path);
    if (template_file.open(QIODevice::WriteOnly)) 
    {
        QJsonDocument doc(default_config);
        template_file.write(doc.toJson(QJsonDocument::Indented));
        template_file.close();
        
        spdlog::info("[CONFIG] Default config template created: {}", template_path.toStdString());
    }
    else 
    {
        spdlog::error("[CONFIG] Failed to create config template: {}", template_path.toStdString());
    }
}

// 2. set default config object
QJsonObject CONFIG::set_default_config_object()
{
    QJsonObject config;
    
    config["robot"] =           set_default_robot_config();
    config["sensors"] =         set_default_sensors_config();
    config["localization"] =    set_default_localization_config();
    config["loc_2d"] =          set_default_localization_2d_config();
    config["loc_3d"] =          set_default_localization_3d_config();
    config["network"] =         set_default_network_config();
    config["debug"] =           set_default_debug_config();
    config["logging"] =         set_default_logging_config();
    config["motor"] =           set_default_motor_config();
    config["mapping"] =         set_default_mapping_config();
    config["obs"] =             set_default_obstacle_config();
    config["control"] =         set_default_control_config();
    config["docking"] =         set_default_docking_config();
    config["safety"] =          set_default_safety_config();
    config["qa"] =              set_default_qa_config();
    config["map"] =             QJsonObject{{"MAP_PATH", MAP_PATH}};

    config["SICK"] =            set_default_sick_config();
    config["LAKI"] =            set_default_laki_config();
    config["RPLIDAR"] =         set_default_rplidar_config();
    config["LIVOX"] =           set_default_livox_config();
    config["ORBBEC"] =          set_default_orbbec_config();

    config["update"] =          set_default_update_config();
    
    return config;
}

// 3. section default config functions
QJsonObject CONFIG::set_default_robot_config()
{
    QJsonObject robot;
    robot["MILEAGE"] =                      QString(MILEAGE);
    robot["SPEAKER"] =                      USE_SPEAKER;
    robot["ROBOT_WHEEL_TYPE"] =             QString(ROBOT_WHEEL_TYPE);
    robot["ROBOT_SIZE_MIN_X"] =             QString::number(ROBOT_SIZE_X[0]);
    robot["ROBOT_SIZE_MAX_X"] =             QString::number(ROBOT_SIZE_X[1]);
    robot["ROBOT_SIZE_MIN_Y"] =             QString::number(ROBOT_SIZE_Y[0]);
    robot["ROBOT_SIZE_MAX_Y"] =             QString::number(ROBOT_SIZE_Y[1]);
    robot["ROBOT_SIZE_MIN_Z"] =             QString::number(ROBOT_SIZE_Z[0]);
    robot["ROBOT_SIZE_MAX_Z"] =             QString::number(ROBOT_SIZE_Z[1]);
    robot["ROBOT_SIZE_ADD_X"] =             QString::number(ROBOT_SIZE_ADD_X);
    robot["ROBOT_SIZE_ADD_Y"] =             QString::number(ROBOT_SIZE_ADD_Y);
    robot["ROBOT_SIZE_ADD_Z"] =             QString::number(ROBOT_SIZE_ADD_Z);
    robot["ROBOT_WHEEL_RADIUS"] =           QString::number(ROBOT_WHEEL_RADIUS);
    robot["ROBOT_WHEEL_BASE"] =             QString::number(ROBOT_WHEEL_BASE);
    robot["ROBOT_LX"] =                     QString::number(ROBOT_LX);
    robot["ROBOT_LY"] =                     QString::number(ROBOT_LY);
    robot["ROBOT_ALARM_BAT_LOW"] =          QString::number(ROBOT_ALARM_BAT_LOW);
    robot["ROBOT_ALARM_BAT_CRITICAL"] =     QString::number(ROBOT_ALARM_BAT_CRITICAL);
    robot["ROBOT_ALARM"] =                  USE_ROBOT_ALARM;
    return robot;
}

QJsonObject CONFIG::set_default_sensors_config()
{
    QJsonObject sensors;
    sensors["USE_LIDAR_2D"] =       USE_LIDAR_2D; //QString(USE_LIDAR_2D ? "true" : "false");
    sensors["USE_LIDAR_3D"] =       USE_LIDAR_3D; //QString(USE_LIDAR_3D ? "true" : "false");
    sensors["USE_CAM"] =            USE_CAM; //QString(USE_CAM ? "true" : "false");
    sensors["USE_CAM_RGB"] =        USE_CAM_RGB; //QString(USE_CAM_RGB ? "true" : "false");
    sensors["USE_CAM_DEPTH"] =      USE_CAM_DEPTH; //QString(USE_CAM_DEPTH ? "true" : "false");
    sensors["USE_CAM_FILTER"] =     USE_CAM_FILTER; //QString(USE_CAM_FILTER ? "true" : "false");
    sensors["USE_BQR"] =            USE_BQR; //QString(USE_BQR ? "true" : "false");
    sensors["USE_IMU"] =            USE_IMU; //QString(USE_IMU ? "true" : "false");
    sensors["LIDAR_2D_NUM"] =       QString::number(LIDAR_2D_NUM);
    sensors["LIDAR_3D_NUM"] =       QString::number(LIDAR_3D_NUM);
    sensors["CAM_NUM"] =            QString::number(CAM_NUM);
    sensors["LIDAR_2D_TYPE"] =      QString(LIDAR_2D_TYPE);
    sensors["LIDAR_3D_TYPE"] =      QString(LIDAR_3D_TYPE);
    sensors["CAM_TYPE"] =           QString(CAM_TYPE);

    sensors["CAM_FILTER_ROR_RADIUS"] = QString::number(CAM_FILTER_ROR_RADIUS);
    sensors["CAM_FILTER_ROR_MIN_NEIGHBORS"] = QString::number(CAM_FILTER_ROR_MIN_NEIGHBORS);
    sensors["CAM_FILTER_CLUSTER_MIN_SIZE"] = QString::number(CAM_FILTER_CLUSTER_MIN_SIZE);

    return sensors;
}

QJsonObject CONFIG::set_default_localization_config()
{
    QJsonObject loc;
    loc["MODE"] =           QString(LOC_MODE);
    loc["USE_ARUCO"] =      USE_ARUCO;
    loc["USE_EKF"] =        USE_EKF;
    return loc;
}

QJsonObject CONFIG::set_default_localization_2d_config()
{
    QJsonObject loc_2d;
    loc_2d["LOC_SURFEL_NUM"] =                  QString::number(LOC_2D_SURFEL_NUM);
    loc_2d["LOC_ICP_MAX_FEATURE_NUM"] =         QString::number(LOC_2D_ICP_MAX_FEATURE_NUM);
    loc_2d["LOC_ARUCO_ODO_FUSION_DIST"] =       QString::number(LOC_2D_ARUCO_ODO_FUSION_DIST);
    loc_2d["LOC_ARUCO_ODO_FUSION_RATIO"] =      QString::number(LOC_2D_ARUCO_ODO_FUSION_RATIO);
    loc_2d["LOC_CHECK_DIST"] =                  QString::number(LOC_2D_CHECK_DIST);
    loc_2d["LOC_CHECK_IE"] =                    QString::number(LOC_2D_CHECK_IE);
    loc_2d["LOC_CHECK_IR"] =                    QString::number(LOC_2D_CHECK_IR);
    loc_2d["LOC_ICP_COST_THRESHOLD"] =          QString::number(LOC_2D_ICP_COST_THRESHOLD);
    loc_2d["LOC_ICP_COST_THRESHOLD_0"] =        QString::number(LOC_2D_ICP_COST_THRESHOLD_0);
    loc_2d["LOC_ICP_ERROR_THRESHOLD"] =         QString::number(LOC_2D_ICP_ERROR_THRESHOLD);
    loc_2d["LOC_ICP_ODO_FUSION_RATIO"] =        QString::number(LOC_2D_ICP_ODO_FUSION_RATIO);
    loc_2d["LOC_SURFEL_RANGE"] =                QString::number(LOC_2D_SURFEL_RANGE);
    loc_2d["LOC_ROTATION_FALLBACK_USE"] =       LOC_2D_USE_ROTATION_FALLBACK;
    loc_2d["LOC_ROTATION_FALLBACK_STEP"] =      QString::number(LOC_2D_ROTATION_FALLBACK_STEP);
    loc_2d["LOC_ROTATION_FALLBACK_RANGE"] =     QString::number(LOC_2D_ROTATION_FALLBACK_RANGE);
    
    return loc_2d;
}

QJsonObject CONFIG::set_default_localization_3d_config()
{
    QJsonObject loc_3d;
    loc_3d["LOC_MAX_FEATURE_NUM"] =     QString::number(LOC_MAX_FEATURE_NUM);
    loc_3d["LOC_SURFEL_NN_NUM"] =       QString::number(LOC_SURFEL_NN_NUM);
    loc_3d["LOC_SURFEL_BALANCE"] =      QString::number(LOC_SURFEL_BALANCE);
    loc_3d["LOC_COST_THRESHOLD"] =      QString::number(LOC_COST_THRESHOLD);
    loc_3d["LOC_INLIER_CHECK_DIST"] =   QString::number(LOC_INLIER_CHECK_DIST);
    return loc_3d;
}

QJsonObject CONFIG::set_default_network_config()
{
    QJsonObject network;
    network["USE_MULTI"] =      USE_MULTI;//QString(USE_MULTI ? "true" : "false");
    network["USE_COOP"] =       USE_COMM_COOP; //QString(USE_COMM_COOP ? "true" : "false");
    network["USE_RTSP"] =       USE_COMM_RTSP;// QString(USE_COMM_RTSP ? "true" : "false");
    network["USE_RRS"] =        USE_COMM_RRS;//QString(USE_COMM_RRS ? "true" : "false");
    network["USE_MSA"] =        USE_COMM_MSA;//QString(USE_COMM_MSA ? "true" : "false");
    network["USE_FMS"] =        USE_COMM_FMS;//QString(USE_COMM_FMS ? "true" : "false");
    return network;
}

QJsonObject CONFIG::set_default_debug_config()
{
    QJsonObject debug;
    debug["USE_SIM"] =      USE_SIM; //QString(USE_SIM ? "true" : "false");
    debug["USE_BEEP"] =     USE_BEEP; //QString(USE_BEEP ? "true" : "false");
    debug["SERVER_IP"] =    QString(SERVER_IP);
    debug["SERVER_ID"] =    QString(SERVER_ID);
    debug["SERVER_PW"] =    QString(SERVER_PW);
    return debug;
}

QJsonObject CONFIG::set_default_logging_config()
{
    QJsonObject logging;
    logging["LOG_LEVEL"] =              QString(LOG_LEVEL);
    logging["DEBUG_LIDAR_2D"] =         DEBUG_LIDAR_2D; //QString(DEBUG_LIDAR_2D ? "true" : "false");
    logging["DEBUG_LIDAR_3D"] =         DEBUG_LIDAR_3D; //QString(DEBUG_LIDAR_3D ? "true" : "false");
    logging["DEBUG_MOBILE"] =           DEBUG_MOBILE; //QString(DEBUG_MOBILE ? "true" : "false");
    logging["DEBUG_COMM_RRS"] =         DEBUG_COMM_RRS; //QString(DEBUG_COMM_RRS ? "true" : "false");
    logging["DEBUG_AUTOCONTROL"] =      DEBUG_AUTOCONTROL; //QString(DEBUG_AUTOCONTROL ? "true" : "false");
    logging["DEBUG_LOCALIZATION"] =     DEBUG_LOCALIZATION; //QString(DEBUG_LOCALIZATION ? "true" : "false");
    logging["DEBUG_OBSMAP"] =           DEBUG_OBSMAP; //QString(DEBUG_OBSMAP ? "true" : "false");
    logging["LOG_ENABLE_FILE_OUTPUT"] = LOG_ENABLE_FILE_OUTPUT; //QString(LOG_ENABLE_FILE_OUTPUT ? "true" : "false");
    logging["LOG_FILE_PATH"] =          QString(LOG_FILE_PATH);
    return logging;
}

QJsonObject CONFIG::set_default_motor_config()
{
    QJsonObject motor;
    motor["MOTOR_ID_L"] = QString::number(MOTOR_ID_L);
    motor["MOTOR_ID_R"] = QString::number(MOTOR_ID_R);
    motor["MOTOR_DIR"] = QString::number(MOTOR_DIR);
    motor["MOTOR_GEAR_RATIO"] = QString::number(MOTOR_GEAR_RATIO);
    motor["MOTOR_LIMIT_V"] = QString::number(MOTOR_LIMIT_V);
    motor["MOTOR_LIMIT_V_ACC"] = QString::number(MOTOR_LIMIT_V_ACC);
    motor["MOTOR_LIMIT_W"] = QString::number(MOTOR_LIMIT_W);
    motor["MOTOR_LIMIT_W_ACC"] = QString::number(MOTOR_LIMIT_W_ACC);
    motor["MOTOR_GAIN_KP"] = QString::number(MOTOR_GAIN_KP);
    motor["MOTOR_GAIN_KI"] = QString::number(MOTOR_GAIN_KI);
    motor["MOTOR_GAIN_KD"] = QString::number(MOTOR_GAIN_KD);
    motor["MOTOR_SAFETY_LIMIT_V"] = QString::number(MOTOR_SAFETY_LIMIT_V);
    motor["MOTOR_SAFETY_LIMIT_W"] = QString::number(MOTOR_SAFETY_LIMIT_W);
    return motor;
}

QJsonObject CONFIG::set_default_mapping_config()
{
    QJsonObject mapping;
    mapping["SLAM_WINDOW_SIZE"] = QString::number(MAPPING_WINDOW_SIZE);
    mapping["SLAM_VOXEL_SIZE"] = QString::number(MAPPING_VOXEL_SIZE);
    mapping["SLAM_ICP_COST_THRESHOLD"] = QString::number(MAPPING_ICP_COST_THRESHOLD);
    mapping["SLAM_ICP_ERROR_THRESHOLD"] = QString::number(MAPPING_ICP_ERROR_THRESHOLD);
    mapping["SLAM_ICP_MAX_FEATURE_NUM"] = QString::number(MAPPING_ICP_MAX_FEATURE_NUM);
    mapping["SLAM_ICP_DO_ERASE_GAP"] = QString::number(MAPPING_ICP_DO_ERASE_GAP);
    mapping["SLAM_ICP_DO_ACCUM_NUM"] = QString::number(MAPPING_ICP_DO_ACCUM_NUM);
    mapping["SLAM_ICP_VIEW_THRESHOLD"] = QString::number(MAPPING_ICP_VIEW_THRESHOLD);
    mapping["SLAM_KFRM_UPDATE_NUM"] = QString::number(MAPPING_KFRM_UPDATE_NUM);
    mapping["SLAM_KFRM_LC_TRY_DIST"] = QString::number(MAPPING_KFRM_LC_TRY_DIST);
    mapping["SLAM_KFRM_LC_TRY_OVERLAP"] = QString::number(MAPPING_KFRM_LC_TRY_OVERLAP);
    return mapping;
}

QJsonObject CONFIG::set_default_obstacle_config()
{
    QJsonObject obs;
    obs["OBS_AVOID"] = QString::number(OBS_AVOID);
    obs["OBS_DEADZONE"] = QString::number(OBS_DEADZONE);
    obs["OBS_LOCAL_GOAL_D"] = QString::number(OBS_LOCAL_GOAL_D);
    obs["OBS_SAFE_MARGIN_X"] = QString::number(OBS_SAFE_MARGIN_X);
    obs["OBS_SAFE_MARGIN_Y"] = QString::number(OBS_SAFE_MARGIN_Y);
    obs["OBS_PATH_MARGIN_X"] = QString::number(OBS_PATH_MARGIN_X);
    obs["OBS_PATH_MARGIN_Y"] = QString::number(OBS_PATH_MARGIN_Y);
    obs["OBS_MAP_GRID_SIZE"] = QString::number(OBS_MAP_GRID_SIZE);
    obs["OBS_MAP_RANGE"] = QString::number(OBS_MAP_RANGE);
    obs["OBS_MAP_MIN_V"] = QString::number(OBS_MAP_MIN_V);
    obs["OBS_MAP_MIN_Z"] = QString::number(OBS_MAP_MIN_Z);
    obs["OBS_MAP_MAX_Z"] = QString::number(OBS_MAP_MAX_Z);
    obs["OBS_PREDICT_TIME"] = QString::number(OBS_PREDICT_TIME);
    obs["OBS_DISTANCE_LED_NEAR"] = QString::number(OBS_DISTANCE_LED_NEAR);
    obs["OBS_DISTANCE_LED_FAR"] = QString::number(OBS_DISTANCE_LED_FAR);
    return obs;
}

QJsonObject CONFIG::set_default_control_config()
{
    QJsonObject control;
    control["DRIVE_GOAL_APPROACH_GAIN"] = QString::number(DRIVE_GOAL_APPROACH_GAIN);
    control["DRIVE_GOAL_D"] = QString::number(DRIVE_GOAL_D);
    control["DRIVE_GOAL_TH"] = QString::number(DRIVE_GOAL_TH);
    control["DRIVE_EXTENDED_CONTROL_TIME"] = QString::number(DRIVE_EXTENDED_CONTROL_TIME);
    control["DRIVE_V_DEADZONE"] = QString::number(DRIVE_V_DEADZONE);
    control["DRIVE_W_DEADZONE"] = QString::number(DRIVE_W_DEADZONE);
    return control;
}

QJsonObject CONFIG::set_default_docking_config()
{
    QJsonObject dock;
    dock["DOCKING_TYPE"] = QString::number(DOCKING_TYPE);
    dock["DOCKING_POINTDOCK_MARGIN"] = QString::number(DOCKING_POINTDOCK_MARGIN);
    dock["DOCKING_GOAL_D"] = QString::number(DOCKING_GOAL_D);
    dock["DOCKING_GOAL_TH"] = QString::number(DOCKING_GOAL_TH);
    dock["DOCKING_KP_d"] = QString::number(DOCKING_KP_d);
    dock["DOCKING_KD_d"] = QString::number(DOCKING_KD_d);
    dock["DOCKING_KP_th"] = QString::number(DOCKING_KP_th);
    dock["DOCKING_KD_th"] = QString::number(DOCKING_KD_th);
    dock["DOCKING_CLUST_DIST_THRESHOLD_MAX"] = QString::number(DOCKING_CLUST_DIST_THRESHOLD_MAX);
    dock["DOCKING_CLUST_ANGLE_THRESHOLD"] = QString::number(DOCKING_CLUST_ANGLE_THRESHOLD);
    dock["DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX"] = QString::number(DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX);
    dock["DOCKING_CHG_LENGTH"] = QString::number(DOCKING_CHG_LENGTH);
    dock["DOCKING_ICP_ERR_THRESHOLD"] = QString::number(DOCKING_ICP_ERR_THRESHOLD);
    dock["DOCKING_DWA_YAW_WEIGTH"] = QString::number(DOCKING_DWA_YAW_WEIGHT);
    dock["DOCKING_CHECK_MOTOR_A"] = QString::number(DOCKING_CHECK_MOTOR_A);
    dock["DOCKING_WAITING_TIME"] = QString::number(DOCKING_WAITING_TIME);
    dock["DOCKING_X_OFFSET"] = QString::number(DOCKING_X_OFFSET);
    dock["DOCKING_Y_OFFSET"] = QString::number(DOCKING_Y_OFFSET);
    dock["DOCKING_LINEAR_X_OFFSET"] = QString::number(DOCKING_LINEAR_X_OFFSET);
    dock["DOCKING_REVERSE_FLAG"] =              DOCKING_REVERSE_FLAG;
    dock["CHARGE_TYPE"] =                       QString(CHARGE_TYPE);
    dock["DOCKING_FIELD"] =                     QString::number(DOCKING_FIELD);
    dock["XNERGY_SET_CURRENT"] =                QString::number(XNERGY_SET_CURRENT);

    return dock;
}

QJsonObject CONFIG::set_default_safety_config()
{
    QJsonObject safety;
    safety["USE_SAFETY_CROSS_MONITOR"] =    USE_SAFETY_CROSS_MONITOR; //QString(USE_SAFETY_CROSS_MONITOR ? "true" : "false");
    safety["USE_SAFETY_SPEED_CONTROL"] =    USE_SAFETY_SPEED_CONTROL; //QString(USE_SAFETY_SPEED_CONTROL ? "true" : "false");
    safety["USE_SAFETY_OBSTACLE_DETECT"] =  USE_SAFETY_OBSTACLE_DETECT; //QString(USE_SAFETY_OBSTACLE_DETECT ? "true" : "false");
    safety["USE_SAFETY_BUMPER"] =           USE_SAFETY_BUMPER; //QString(USE_SAFETY_BUMPER ? "true" : "false");
    safety["USE_SAFETY_INTERLOCK"] =        USE_SAFETY_INTERLOCK; //QString(USE_SAFETY_INTERLOCK ? "true" : "false");
    safety["USE_MONITORING_FIELD"] =        USE_MONITORING_FIELD;
    safety["MONITORING_FIELD_NUM"] =        QString::number(MONITORING_FIELD_NUM);
    safety["MONITORING_TRAJ_DT"] =          QString::number(MONITORING_TRAJ_DT);
    safety["MONITORING_PREDICT_T"] =        QString::number(MONITORING_PREDICT_T);
    
    QJsonArray fields;
    for(int i = 0; i < MONITORING_FIELD_NUM; i++)
    {
        QJsonObject obj_field;
        obj_field["FIELD_SIZE_MIN_X"] = QString::number(FIELD_SIZE_MIN_X[i]);
        obj_field["FIELD_SIZE_MAX_X"] = QString::number(FIELD_SIZE_MAX_X[i]);
        obj_field["FIELD_SIZE_MIN_Y"] = QString::number(FIELD_SIZE_MIN_Y[i]);
        obj_field["FIELD_SIZE_MAX_Y"] = QString::number(FIELD_SIZE_MAX_Y[i]);
        fields.append(obj_field);
    }
    safety["MONITORING_FIELDS"] = fields;

    return safety;
}

QJsonObject CONFIG::set_default_qa_config()
{
    QJsonObject qa;
    qa["QA_STEP_DISTANCE"] = QString::number(QA_STEP_DISTANCE);
    qa["QA_STOP_MIN_DISTANCE"] = QString::number(QA_STOP_MIN_DISTANCE);
    return qa;
}

QJsonObject CONFIG::set_default_sick_config()
{
    QJsonObject sick;
    sick["LIDAR_2D_MIN_RANGE"] = QString::number(LIDAR_2D_MIN_RANGE);
    sick["LIDAR_2D_MAX_RANGE"] = QString::number(LIDAR_2D_MAX_RANGE);
    return sick;
}

QJsonObject CONFIG::set_default_laki_config()
{
    QJsonObject laki;
    laki["LIDAR_2D_MIN_RANGE"] = QString::number(LIDAR_2D_MIN_RANGE);
    laki["LIDAR_2D_MAX_RANGE"] = QString::number(LIDAR_2D_MAX_RANGE);
    return laki;
}

QJsonObject CONFIG::set_default_rplidar_config()
{
    QJsonObject rplidar;
    rplidar["LIDAR_2D_MIN_RANGE"] = QString::number(LIDAR_2D_MIN_RANGE);
    rplidar["LIDAR_2D_MAX_RANGE"] = QString::number(LIDAR_2D_MAX_RANGE);
    return rplidar;
}

QJsonObject CONFIG::set_default_livox_config()
{
    QJsonObject livox;
    livox["LIDAR_3D_MIN_RANGE"] = QString::number(LIDAR_3D_MIN_RANGE);
    livox["LIDAR_3D_MAX_RANGE"] = QString::number(LIDAR_3D_MAX_RANGE);
    return livox;
}

QJsonObject CONFIG::set_default_orbbec_config()
{
    QJsonObject orbbec;
    orbbec["CAM_HEIGHT_MIN"] = QString::number(CAM_HEIGHT_MIN);
    orbbec["CAM_HEIGHT_MAX"] = QString::number(CAM_HEIGHT_MAX);
    return orbbec;
}

QJsonObject CONFIG::set_default_update_config()
{
    QJsonObject update;
    update["CONFIG"] =        USE_CONFIG_UPDATE;
    //update["CONFIG"] = QString(USE_CONFIG_UPDATE ? "true" : "false");

    return update;
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

double CONFIG::get_obs_distance_led_near()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_DISTANCE_LED_NEAR;
}

double CONFIG::get_obs_distance_led_far()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return OBS_DISTANCE_LED_FAR;
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

double CONFIG::get_xnergy_set_current()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return XNERGY_SET_CURRENT;
}

int CONFIG::get_docking_field()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_FIELD;
}

int CONFIG::get_docking_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_TYPE;
}

bool CONFIG::get_docking_reverse_mode()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return DOCKING_REVERSE_FLAG;
}

QString CONFIG::get_charge_type()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return CHARGE_TYPE;
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
double CONFIG::get_qa_step_distance()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return QA_STEP_DISTANCE;
}

double CONFIG::get_qa_stop_min_distance()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return QA_STOP_MIN_DISTANCE;
}
