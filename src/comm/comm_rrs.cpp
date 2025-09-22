#include "comm_rrs.h"

#include "mainwindow.h"

ERROR_MANAGER::ErrorInfo ERROR_MANAGER::getErrorInfo(ErrorCause cause, ErrorContext context)
{
    ErrorInfo info;
    
    QString hex_str = QString::number(cause, 16).toUpper();
    QString error_code = QString("R0%1").arg(hex_str);
    QString alarm_code = QString::number(cause & 0x0FFF);
    
    info.category = getCategoryName(cause);
    info.cause = getCauseName(cause);
    info.context = getContextName(context);
    info.error_code = error_code;
    info.alarm_code = alarm_code;
    
    switch(cause)
    {
    case MAP_NOT_LOADED:
        {
            info.message = "map not loaded";
            info.solution = "Load map file required";
            info.description = "Map is not loaded";
            info.level = "error";
            info.remark = "Navigation/Localization/Initialization unavailable";
            break;
        }

    case MAP_INVALID_PATH:
        {
            info.message = "invalid map dir";
            info.solution = "Check map path required";
            info.description = "Invalid map directory";
            info.level = "error";
            info.remark = "Map load failed";
            break;
        }

    case MAP_LOAD_FAILED:
        {
            info.message = "map not load";
            info.solution = "System restart required";
            info.description = "Map load failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case MAP_COPY_FAILED:
        {
            info.message = "copy failed";
            info.solution = "Check storage space required";
            info.description = "Map copy failed";
            info.level = "warn";
            info.remark = "Storage error";
            break;
        }

    case TOPO_LOAD_FAILED:
        {
            info.message = "topo not load";
            info.solution = "Check topology file required";
            info.description = "Topology load failed";
            info.level = "warn";
            info.remark = "Topology error";
            break;
        }

    case LOC_NOT_INIT:
        {
            info.message = "no localization";
            info.solution = "Restart localization required";
            info.description = "No localization available";
            info.level = "error";
            info.remark = "Navigation unavailable";
            break;
        }

    case LOC_SENSOR_ERROR:
        {
            info.message = "lidar not connected";
            info.solution = "Check LiDAR hardware required";
            info.description = "Localization sensor error";
            info.level = "error";
            info.remark = "Sensor connection error";
            break;
        }

    case LOC_ALREADY_RUNNING:
        {
            info.message = "already running";
            info.solution = "Terminate existing process required";
            info.description = "Already running";
            info.level = "warn";
            info.remark = "Duplicate execution prevention";
            break;
        }

    case LOC_INIT_FAILED:
        {
            info.message = "localization init failed";
            info.solution = "Reinitialize localization required";
            info.description = "Localization initialization failed";
            info.level = "error";
            info.remark = "Initialization failed";
            break;
        }

    case MOVE_NO_TARGET:
        {
            info.message = "no target specified";
            info.solution = "Set target required";
            info.description = "No target specified";
            info.level = "error";
            info.remark = "Target setting error";
            break;
        }

    case MOVE_TARGET_INVALID:
        {
            info.message = "target invalid";
            info.solution = "Reset target required";
            info.description = "Target is invalid";
            info.level = "error";
            info.remark = "Target validity error";
            break;
        }

    case MOVE_TARGET_OCCUPIED:
        {
            info.message = "target location occupied(static obs)";
            info.solution = "Change target position required";
            info.description = "Target position conflicts with static obstacle";
            info.level = "error";
            info.remark = "Target position collision";
            break;
        }

    case MOVE_TARGET_OUT_RANGE:
        {
            info.message = "target location out of range";
            info.solution = "Reset target position required";
            info.description = "Target position is out of map range";
            info.level = "error";
            info.remark = "Target range exceeded";
            break;
        }

    case MOVE_NODE_NOT_FOUND:
        {
            info.message = "can not find node";
            info.solution = "Check node ID/name required";
            info.description = "Node not found";
            info.level = "error";
            info.remark = "Target node missing";
            break;
        }

    case MOVE_EMPTY_NODE_ID:
        {
            info.message = "empty node id";
            info.solution = "Enter target node ID required";
            info.description = "Empty node ID";
            info.level = "error";
            info.remark = "Target setting error";
            break;
        }

    case SENSOR_LIDAR_DISCON:
        {
            info.message = "lidar not connected";
            info.solution = "Check LiDAR hardware required";
            info.description = "LiDAR not connected";
            info.level = "error";
            info.remark = "Sensor connection error/Sensor initialization failed/Mapping unavailable";
            break;
        }
            
    case SENSOR_LIDAR_DATA_ERROR:
        {
            info.message = "lidar data error";
            info.solution = "Check LiDAR data quality required";
            info.description = "LiDAR data error";
            info.level = "warn";
            info.remark = "Data quality degradation";
            break;
        }

    case SENSOR_LIDAR_CALIB_ERROR:
        {
            info.message = "lidar calibration error";
            info.solution = "Re-run LiDAR calibration required";
            info.description = "LiDAR calibration error";
            info.level = "warn";
            info.remark = "Accuracy degradation";
            break;
        }
            
    case SENSOR_IMU_DISCON:
        {
            info.message = "IMU connection lost";
            info.solution = "Check IMU hardware required";
            info.description = "IMU connection lost";
            info.level = "error";
            info.remark = "Attitude control error";
            break;
        }

    case SENSOR_IMU_DATA_ERROR:
        {
            info.message = "IMU data error";
            info.solution = "Check IMU data quality required";
            info.description = "IMU data error";
            info.level = "warn";
            info.remark = "Attitude information error";
            break;
        }

    case SENSOR_CAM_DISCON:
        {
            info.message = "CAM connection lost";
            info.solution = "Check CAM hardware required";
            info.description = "CAM connection lost";
            info.level = "warn";
            info.remark = "CAM function limited";
            break;
        }

    case SENSOR_CAM_DATA_ERROR:
        {
            info.message = "CAM data error";
            info.solution = "Check CAM data quality required";
            info.description = "CAM data error";
            info.level = "info";
            info.remark = "CAM quality degradation";
            break;
        }

    case SENSOR_QR_ERROR:
        {
            info.message = "QR sensor error";
            info.solution = "Check QR sensor required";
            info.description = "QR sensor error";
            info.level = "warn";
            info.remark = "Recognition function limited";
            break;
        }

    case SENSOR_TEMP_ERROR:
        {
            info.message = "temperature sensor error";
            info.solution = "Check temperature sensor required";
            info.description = "Temperature sensor error";
            info.level = "info";
            info.remark = "Monitoring limited";
            break;
        }

    case SYS_NOT_SUPPORTED:
        {
            info.message = "not supported";
            info.solution = "Use supported method required";
            info.description = "Unsupported function";
            info.level = "warn";
            info.remark = "Function not supported";
            break;
        }

    case SYS_MULTI_MODE_LIMIT:
        {
            info.message = "target command not supported by multi. use goal_id";
            info.solution = "Use goal command required";
            info.description = "Using target command in multi-robot mode";
            info.level = "warn";
            info.remark = "Multi-mode limitation";
            break;
        }

    case SYS_PROCESS_START_FAILED:
        {
            info.message = "process failed to start";
            info.solution = "System restart required";
            info.description = "Process start failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case SYS_PROCESS_FINISH_FAILED:
        {
            info.message = "process did not finish";
            info.solution = "Force terminate process and restart required";
            info.description = "Process completion failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case SYS_NETWORK_ERROR:
        {
            info.message = "network connection error";
            info.solution = "Check network connection required";
            info.description = "Network communication error";
            info.level = "warn";
            info.remark = "Communication limited";
            break;
        }

    case SAFETY_EMO_RELEASED:
        {
            info.message = "emo released";
            info.solution = "Check emergency stop status required";
            info.description = "Emergency stop released";
            info.level = "critical";
            info.remark = "Safety risk";
            break;
        }

    case SAFETY_EMO_PRESSED:
        {
            info.message = "EMO pressed";
            info.solution = "Release emergency stop required";
            info.description = "Emergency stop button pressed";
            info.level = "critical";
            info.remark = "Immediate stop";
            break;
        }

    case SAFETY_BUMPER_PRESSED:
        {
            info.message = "bumper pressed";
            info.solution = "Remove obstacle and restart required";
            info.description = "Bumper detected";
            info.level = "critical";
            info.remark = "Obstacle collision";
            break;
        }

    case SAFETY_OBS_DETECTED:
        {
            info.message = "obstacle detected";
            info.solution = "Change path or remove obstacle required";
            info.description = "Obstacle detected";
            info.level = "error";
            info.remark = "Path change required";
            break;
        }

    case SAFETY_ZONE_VIOLATION:
        {
            info.message = "safety zone violation";
            info.solution = "Resolve safety zone violation required";
            info.description = "Safety zone violation";
            info.level = "error";
            info.remark = "Safety risk";
            break;
        }

    case BAT_NOT_CHARGING:
        {
            info.message = "not charging";
            info.solution = "Check charging status required";
            info.description = "Not charging";
            info.level = "warn";
            info.remark = "Battery management";
            break;

        }

    case BAT_LOW:
        {
            info.message = "low battery";
            info.solution = "Charging required";
            info.description = "Low battery";
            info.level = "error";
            info.remark = "Charging required";
            break;
        }

    case BAT_CRITICAL:
        {
            info.message = "battery critical";
            info.solution = "Immediate charging required";
            info.description = "Battery critical level";
            info.level = "critical";
            info.remark = "Immediate charging";
            break;
        }

    case BAT_POWER_ERROR:
        {
            info.message = "power supply error";
            info.solution = "Check power supply system required";
            info.description = "Power supply error";
            info.level = "error";
            info.remark = "System error";
            break;

        }

    case MOTOR_CONNECTION_LOST:
        {
            info.message = "motor connection lost";
            info.solution = "Check motor hardware required";
            info.description = "Motor connection lost";
            info.level = "critical";
            info.remark = "Navigation unavailable";
            break;
        }

    case MOTOR_OVERHEAT:
        {
            info.message = "motor overheat";
            info.solution = "Cool motor and restart required";
            info.description = "Motor overheating";
            info.level = "error";
            info.remark = "Motor protection";
            break;
        }

    case MOTOR_OVERLOAD:
        {
            info.message = "motor overload";
            info.solution = "Reduce motor load required";
            info.description = "Motor overload";
            info.level = "error";
            info.remark = "Performance limitation";
            break;
        }

    case MOTOR_ENCODER_ERROR:
        {
            info.message = "motor encoder error";
            info.solution = "Check motor encoder required";
            info.description = "Motor encoder error";
            info.level = "error";
            info.remark = "Position control error";
            break;
        }

    default:
        {
            info.error_code = "[D]R0Sx0000";
            info.alarm_code = "[D]0000";
            info.category = "[D]System";
            info.cause = "[D]Unknown";
            info.context = "[D]UNKNOWN";
            info.message = "[D]unknown error";
            info.solution = "[D]Contact system administrator required";
            info.description = "[D]Unknown error";
            info.level = "error";
            info.remark = "[D]System error";
            break;
        }

    }
    
    return info;
}

QString ERROR_MANAGER::getErrorMessage(ErrorCause cause, ErrorContext context)
{
    ErrorInfo info = getErrorInfo(cause, context);
    return QString("[%1] %2").arg(info.error_code).arg(info.message);
}

void ERROR_MANAGER::logError(ErrorCause cause, ErrorContext context, const QString& additional_info)
{
    ErrorInfo info = getErrorInfo(cause, context);
    
    QString log_message = QString("[%1] %2 - %3 (Solution: %4)")
                         .arg(info.error_code)
                         .arg(info.message)
                         .arg(info.description)
                         .arg(info.solution);
    
    if(!additional_info.isEmpty())
    {
        log_message += QString(" [Additional Info: %1]").arg(additional_info);
    }
    
    if(info.level == "critical")
    {
        spdlog::critical("{}", log_message.toStdString());
    }
    else if(info.level == "error")
    {
        spdlog::error("[COMM_RRS] {}", log_message.toStdString());
    }
    else if(info.level == "warn")
    {
        spdlog::warn("[COMM_RRS] {}", log_message.toStdString());
    }
    else
    {
        spdlog::info("[COMM_RRS] {}", log_message.toStdString());
    }
}

QString ERROR_MANAGER::getCategoryName(ErrorCause cause)
{
    switch(cause & 0xF000)
    {
    case 0x1000:        return "Map Management";
    case 0x2000:        return "Localization";
    case 0x3000:        return "Navigation";
    case 0x4000:        return "Sensor";
    case 0x5000:        return "System";
    case 0x6000:        return "Safety";
    case 0x7000:        return "Battery";
    case 0x8000:        return "Motor";
    default:            return "Unknown";
    }
}

QString ERROR_MANAGER::getCauseName(ErrorCause cause)
{
    switch(cause)
    {
    case MAP_NOT_LOADED:                return "Map Not Loaded";
    case MAP_INVALID_PATH:              return "Map Path Error";
    case MAP_LOAD_FAILED:               return "Map Load Failed";
    case MAP_COPY_FAILED:               return "Map Copy Failed";
    case TOPO_LOAD_FAILED:              return "Topology Load Failed";
    case LOC_NOT_INIT:                  return "Localization Not Initialized";
    case LOC_SENSOR_ERROR:              return "Localization Sensor Error";
    case LOC_ALREADY_RUNNING:           return "Localization Already Running";
    case LOC_INIT_FAILED:               return "Localization Init Failed";
    case MOVE_NO_TARGET:                return "No Target";
    case MOVE_TARGET_INVALID:           return "Target Invalid";
    case MOVE_TARGET_OCCUPIED:          return "Target Position Occupied";
    case MOVE_TARGET_OUT_RANGE:         return "Target Out of Range";
    case MOVE_NODE_NOT_FOUND:           return "Node Not Found";
    case MOVE_EMPTY_NODE_ID:            return "Empty Node ID";
    case SENSOR_LIDAR_DISCON:           return "LiDAR Disconnected";
    case SENSOR_LIDAR_DATA_ERROR:       return "LiDAR Data Error";
    case SENSOR_LIDAR_CALIB_ERROR:      return "LiDAR Calibration Error";
    case SENSOR_IMU_DISCON:             return "IMU Disconnected";
    case SENSOR_IMU_DATA_ERROR:         return "IMU Data Error";
    case SENSOR_CAM_DISCON:             return "Camera Disconnected";
    case SENSOR_CAM_DATA_ERROR:         return "Camera Data Error";
    case SENSOR_QR_ERROR:               return "QR Sensor Error";
    case SENSOR_TEMP_ERROR:             return "Temperature Sensor Error";
    case SYS_NOT_SUPPORTED:             return "Function Not Supported";
    case SYS_MULTI_MODE_LIMIT:          return "Multi Mode Limitation";
    case SYS_PROCESS_START_FAILED:      return "Process Start Failed";
    case SYS_PROCESS_FINISH_FAILED:     return "Process Finish Failed";
    case SYS_NETWORK_ERROR:             return "Network Error";
    case SAFETY_EMO_RELEASED:           return "Emergency Stop Released";
    case SAFETY_EMO_PRESSED:            return "Emergency Stop Pressed";
    case SAFETY_BUMPER_PRESSED:         return "Bumper Pressed";
    case SAFETY_OBS_DETECTED:           return "Obstacle Detected";
    case SAFETY_ZONE_VIOLATION:         return "Safety Zone Violation";
    case BAT_NOT_CHARGING:              return "Not Charging";
    case BAT_LOW:                       return "Low Battery";
    case BAT_CRITICAL:                  return "Battery Critical";
    case BAT_POWER_ERROR:               return "Power Supply Error";
    case MOTOR_CONNECTION_LOST:         return "Motor Connection Lost";
    case MOTOR_OVERHEAT:                return "Motor Overheat";
    case MOTOR_OVERLOAD:                return "Motor Overload";
    case MOTOR_ENCODER_ERROR:           return "Motor Encoder Error";
    default:                            return "Unknown";
    }
}

QString ERROR_MANAGER::getContextName(ErrorContext context) // for translate langauge
{
    switch(context)
    {
    case MOVE_TARGET:           return "MOVE_TARGET";
    case MOVE_GOAL:             return "MOVE_GOAL";
    case LOAD_MAP:              return "LOAD_MAP";
    case LOAD_TOPO:             return "LOAD_TOPO";
    case LOAD_CONFIG:           return "LOAD_CONFIG";
    case MAPPING_START:         return "MAPPING_START";
    case MAPPING_SAVE:          return "MAPPING_SAVE";
    case LOC_SEMI_AUTO:         return "LOC_SEMIAUTO";
    case LOC_INIT:              return "LOC_INIT";
    case LOC_START:             return "LOC_START";
    case LOC_STOP:              return "LOC_STOP";
    case SOFTWARE_UPDATE:       return "SOFTWARE_UPDATE";
    case DOCK_START:            return "DOCK_START";
    case DOCK_STOP:             return "DOCK_STOP";
    case LED_CONTROL:           return "LED_CONTROL";
    case MOTOR_CONTROL:         return "MOTOR_CONTROL";
    case FIELD_SET:             return "FIELD_SET";
    case FIELD_GET:             return "FIELD_GET";
    default:                    return "UNKNOWN";
    }
}

COMM_RRS* COMM_RRS::instance(QObject* parent)
{
    static COMM_RRS* inst = nullptr;
    if(!inst && parent)
    {
        inst = new COMM_RRS(parent);
    }
    else if(inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

COMM_RRS::COMM_RRS(QObject *parent) : QObject(parent)
  , main(parent)
  , config(nullptr)
  , logger(nullptr)
  , mobile(nullptr)
  , unimap(nullptr)
  , obsmap(nullptr)
  , lidar_2d(nullptr)
  , loc(nullptr)
  , mapping(nullptr)
  , dctrl(nullptr)
{
    // set recv callbacks
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    using std::placeholders::_4;

    io = std::make_unique<sio::client>();
    sio::socket::ptr sock = io->socket();
    io->set_open_listener(std::bind(&COMM_RRS::sio_connected, this));
    io->set_close_listener(std::bind(&COMM_RRS::sio_disconnected, this, _1));
    io->set_fail_listener(std::bind(&COMM_RRS::sio_error, this));

    // bind recv callback function this func emit signal
    BIND_EVENT(sock, "move",            std::bind(&COMM_RRS::recv_move,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "localization",    std::bind(&COMM_RRS::recv_localization,      this, _1, _2, _3, _4));
    BIND_EVENT(sock, "load",            std::bind(&COMM_RRS::recv_load,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "randomseq",       std::bind(&COMM_RRS::recv_randomseq,         this, _1, _2, _3, _4));
    BIND_EVENT(sock, "mapping",         std::bind(&COMM_RRS::recv_mapping,           this, _1, _2, _3, _4));
    BIND_EVENT(sock, "dock",            std::bind(&COMM_RRS::recv_dock,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "lidarOnOff",      std::bind(&COMM_RRS::recv_view_lidar_on_off, this, _1, _2, _3, _4));
    BIND_EVENT(sock, "pathOnOff",       std::bind(&COMM_RRS::recv_view_path_on_off,  this, _1, _2, _3, _4));
    BIND_EVENT(sock, "led",             std::bind(&COMM_RRS::recv_led,               this, _1, _2, _3, _4));
    BIND_EVENT(sock, "motor",           std::bind(&COMM_RRS::recv_motor,             this, _1, _2, _3, _4));
    BIND_EVENT(sock, "path",            std::bind(&COMM_RRS::recv_path,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "vobs",            std::bind(&COMM_RRS::recv_vobs,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "swUpdate",        std::bind(&COMM_RRS::recv_software_update,   this, _1, _2, _3, _4));
    BIND_EVENT(sock, "footStatus",      std::bind(&COMM_RRS::recv_foot,              this, _1, _2, _3, _4));
    BIND_EVENT(sock, "fieldRequest",    std::bind(&COMM_RRS::recv_field_set,         this, _1, _2, _3, _4));
    BIND_EVENT(sock, "safetyioRequest", std::bind(&COMM_RRS::recv_safety_io_set,     this, _1, _2, _3, _4));

    // connect recv signals -> recv slots
    connect(this, &COMM_RRS::signal_move,            this, &COMM_RRS::slot_move);
    connect(this, &COMM_RRS::signal_localization,    this, &COMM_RRS::slot_localization);
    connect(this, &COMM_RRS::signal_load,            this, &COMM_RRS::slot_load);
    connect(this, &COMM_RRS::signal_randomseq,       this, &COMM_RRS::slot_randomseq);
    connect(this, &COMM_RRS::signal_mapping,         this, &COMM_RRS::slot_mapping);
    connect(this, &COMM_RRS::signal_dock,            this, &COMM_RRS::slot_dock);
    connect(this, &COMM_RRS::signal_field_set,       this, &COMM_RRS::slot_field_set);
    connect(this, &COMM_RRS::signal_view_lidar,      this, &COMM_RRS::slot_view_lidar);
    connect(this, &COMM_RRS::signal_view_path,       this, &COMM_RRS::slot_view_path);
    connect(this, &COMM_RRS::signal_led,             this, &COMM_RRS::slot_led);
    connect(this, &COMM_RRS::signal_motor,           this, &COMM_RRS::slot_motor);
    connect(this, &COMM_RRS::signal_path,            this, &COMM_RRS::slot_path);
    connect(this, &COMM_RRS::signal_vobs,            this, &COMM_RRS::slot_vobs);
    connect(this, &COMM_RRS::signal_software_update, this, &COMM_RRS::slot_software_update);
    connect(this, &COMM_RRS::signal_foot,            this, &COMM_RRS::slot_foot);
    connect(this, &COMM_RRS::signal_safety_io,       this, &COMM_RRS::slot_safety_io);

    send_timer = new QTimer(this);
    connect(send_timer, SIGNAL(timeout()), this, SLOT(send_loop()));
    send_timer->start(10);
}

COMM_RRS::~COMM_RRS()
{
    if(io)
    {
        io->socket()->off_all();
        io->socket()->off_error();
        io->sync_close();
    }

    if(semi_auto_init_thread && semi_auto_init_thread->joinable())
    {
        semi_auto_init_thread->join();
    }
    semi_auto_init_thread.reset();
}

QString COMM_RRS::get_json(sio::message::ptr const& data, const QString& key)
{
    if(!data)
    {
        return "";
    }

    auto data_map = data->get_map();
    auto it = data_map.find(key.toStdString());
    if(it == data_map.end() || !it->second)
    {
        return "";
    }

    sio::message::ptr val = it->second;

    if(val->get_flag() == sio::message::flag_string)
    {
        return QString::fromStdString(it->second->get_string());
    }

    if(val->get_flag() == sio::message::flag_integer)
    {
        return QString::number(val->get_int());
    }

    if(val->get_flag() == sio::message::flag_double)
    {
        return QString::number(val->get_double());
    }

    if(val->get_flag() == sio::message::flag_boolean)
    {
        return val->get_bool() ? "true" : "false";
    }

    return "";
}

void COMM_RRS::fillArray_memcpy(unsigned char* arr, int arr_size, const QJsonArray& jsonArr)
{
    if (jsonArr.size() < 8)
    {
        return;
    }

    // Use the smaller value between the array size and the JSON array size
    int n = qMin(arr_size, jsonArr.size());
    unsigned char* tmp = new unsigned char[n];

    for (int i = 0; i < n; ++i)
    {
        tmp[i] = static_cast<unsigned char>(jsonArr[i].toInt());
    }

    memcpy(arr, tmp, n);
    delete[] tmp;
}

QString COMM_RRS::get_multi_state()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return multi_state;
}

QByteArray COMM_RRS::get_last_msg()
{
    std::shared_lock<std::shared_mutex> lock(mtx);
    return lastest_msg_str;
}

void COMM_RRS::init()
{
    if(!config)
    {
        logger->write_log("[COMM_RRS] config module not set", "Orange");
        spdlog::warn("[COMM_RRS] config module not set");
        return;
    }

    if(config->get_use_rrs())
    {
        logger->write_log("[COMM_RRS] try to connect RRS", "Green");
        spdlog::info("[COMM_RRS] try to connect RRS");

        std::map<std::string, std::string> query;
        query["name"] = "slamnav";

        io->connect("ws://localhost:11337", query);
//        io->connect("ws://10.108.1.12:11337", query);
//        qDebug()<<"ws://10.108.1.12:11337";
    }
}

void COMM_RRS::sio_connected()
{
    is_connected = true;

    if(ctrl)
    {
        ctrl->set_is_rrs(true);
    }

    if(logger)
    {
        logger->write_log("[COMM_RRS] connected", "Green");
    }
    spdlog::info("[COMM_RRS] connected");
}

void COMM_RRS::sio_disconnected(sio::client::close_reason const& reason)
{
    is_connected = false;
    if(ctrl)
    {
        ctrl->set_is_rrs(false);
    }
    if(logger)
    {
        logger->write_log("[COMM_RRS] disconnected", "Green");
    }
    spdlog::warn("[COMM_RRS] disconnected");

}

void COMM_RRS::sio_error()
{
    if(logger)
    {
        logger->write_log("[COMM_RRS] some error", "Red");
    }
    spdlog::error("[COMM_RRS] some error");
}

// recv parser -> emit recv signals
void COMM_RRS::recv_move(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MOVE msg;
        msg.command = get_json(data, "command"); // "goal", "jog", "target", "pause", "resume", "stop"
        msg.method = get_json(data, "method");
        msg.goal_node_id = get_json(data, "goal_id");
        msg.preset = get_json(data, "preset").toInt();
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble() * D2R;
        msg.jog_val[0] = get_json(data, "vx").toDouble();
        msg.jog_val[1] = get_json(data, "vy").toDouble();
        msg.jog_val[2] = get_json(data, "wz").toDouble() * D2R;
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, move, command: {}, time: {}",msg.command.toStdString(),msg.time);
        Q_EMIT signal_move(msg);
    }
}

void COMM_RRS::recv_localization(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOCALIZATION msg;
        msg.command = get_json(data, "command"); // "autoinit", "semiautoinit", "init", "start", "stop", "randominit"
        msg.seed = get_json(data, "seed");
        msg.tgt_pose_vec[0] = get_json(data, "x").toDouble();
        msg.tgt_pose_vec[1] = get_json(data, "y").toDouble();
        msg.tgt_pose_vec[2] = get_json(data, "z").toDouble();
        msg.tgt_pose_vec[3] = get_json(data, "rz").toDouble();
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, loc, command: {}, time: {}",msg.command.toStdString(),msg.time);
        Q_EMIT signal_localization(msg);
    }
}

void COMM_RRS::recv_load(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LOAD msg;
        msg.command = get_json(data, "command"); // "mapload", "topoload", "configload"
        msg.map_name = get_json(data, "name");
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, load, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_load(msg);
    }
}

void COMM_RRS::recv_randomseq(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_RANDOMSEQ msg;
        msg.command = get_json(data, "command"); // "randomseq"
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, randomseq, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_randomseq(msg);
    }
}

void COMM_RRS::recv_mapping(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MAPPING msg;
        msg.command = get_json(data, "command"); // "start", "stop", "save", "name", "reload"
        msg.time = get_json(data, "time").toDouble() / 1000;

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, mapping, command: {}, time: {}",msg.command.toStdString(),msg.time);
        Q_EMIT signal_mapping(msg);
    }
}

void COMM_RRS::recv_dock(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_DOCK msg;
        msg.command = get_json(data, "command"); // "dock", "undock"
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, dock, command: {}, time: {}",msg.command.toStdString(),msg.time);
        Q_EMIT signal_dock(msg);
    }
}

void COMM_RRS::recv_field_set(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_FIELD msg;
        msg.command = get_json(data, "command"); // "set", "get"
        msg.set_field = get_json(data, "set_field").toInt();


        //qDebug() << "recv_field_set_field:" << msg.set_field;
        spdlog::info("[COMM_RRS] recv, field set filed",msg.set_field);
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, field_set, command: {}, time: {}",msg.command.toStdString(),msg.time);
        Q_EMIT signal_field_set(msg);
    }
}


void COMM_RRS::recv_view_lidar_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_LIDAR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, lidar_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, lidar_on_off, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_view_lidar(msg);
    }
}

void COMM_RRS::recv_view_path_on_off(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VIEW_PATH msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.frequency = get_json(data, "frequency").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, path_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, path_on_off, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_view_path(msg);
    }
}

void COMM_RRS::recv_led(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_LED msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.led = get_json(data, "led");
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, led_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, led_on_off, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_led(msg);
    }
}

void COMM_RRS::recv_motor(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_MOTOR msg;
        msg.command = get_json(data, "command"); // "on", "off"
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, motor_on_off, command: %1, time: %2").arg(msg.command).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS] recv, motor_on_off, command: {}, time: {}",msg.command.toStdString(),msg.time);

        Q_EMIT signal_motor(msg);
    }
}

void COMM_RRS::recv_path(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_PATH msg;
        msg.command = get_json(data, "command"); // "path"
        msg.path = get_json(data, "path");
        msg.vobs_closures = get_json(data, "vobs_c");
        msg.preset = get_json(data, "preset").toInt();
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        //logger->write_log(QString("[COMM_RRS] recv, command: %1, path: %2, time: %3").arg(msg.command).arg(msg.path). arg(msg.time), "Green");
        spdlog::info("[COMM_RRS] recv, path, command: {}, path: {}, time: {}",msg.command.toStdString(), msg.path.toStdString(), msg.time);

        Q_EMIT signal_path(msg);
    }
}

void COMM_RRS::recv_vobs(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_VOBS msg;
        msg.command = get_json(data, "command"); // "vobs"
        msg.vobs_robots = get_json(data, "vobs_r");
        msg.vobs_clousers = get_json(data, "vobs_c");
        msg.is_vobs_closures_change = get_json(data, "is_vobs_c");
        msg.time = get_json(data, "time").toDouble() / 1000;

        // action
        QString res = QString("[COMM_RRS] recv, command: %1, vobs_r: %2, vobs_c: %3, time: %4").arg(msg.command).arg(msg.vobs_robots).arg(msg.vobs_clousers).arg(msg.time);
        printf("%s\n", res.toLocal8Bit().constData());

        //logger->write_log(QString("[COMM_RRS] recv, command: %1, vobs: %2, time: %3").arg(msg.command).arg(msg.vobs).arg(msg.time), "Green");

        spdlog::info("[COMM_RRS] recv, vobs, command: {}, vobs_r: {}, vobs_c: {}, time: {}",msg.command.toStdString(), msg.vobs_robots.toStdString(), msg.vobs_clousers.toStdString(), msg.time);
        Q_EMIT signal_vobs(msg);
    }
}

void COMM_RRS::recv_software_update(std::string const& name, sio::message::ptr const& data, bool hasAck, sio::message::list &ack_resp)
{
    if(data && data->get_flag() == sio::message::flag_object)
    {
        // parsing
        DATA_SOFTWARE msg;
        msg.version = get_json(data, "version");
        msg.time = get_json(data, "time").toDouble() / 1000;

        Q_EMIT signal_software_update(msg);
    }
}

void COMM_RRS::recv_foot(const std::string& name, const sio::message::ptr& data, bool hasAck, sio::message::list& ack_resp)
{
    // printf("[COMM_RRS][DEBUG] recv_foot called\n");

    if(data && data->get_flag() == sio::message::flag_object)
    {
        QString time_str = get_json(data, "time");
        double time_sec = get_json(data, "time").toDouble() / 1000;

        // printf("[COMM_RRS][DEBUG] time: %s\n", time_str.toStdString().c_str());

        // get foot
        sio::message::ptr foot_msg = data->get_map()["foot"];
        if(!foot_msg || foot_msg->get_flag() != sio::message::flag_object)
        {
            // printf("[COMM_RRS][DEBUG] 'foot' field missing or not object\n");
            return;
        }

        DATA_FOOT msg;
        msg.connection = get_json(foot_msg, "connection") == "true";
        msg.position   = get_json(foot_msg, "position").toInt();
        msg.is_down    = get_json(foot_msg, "is_down") == "true";
        msg.state      = get_json(foot_msg, "foot_state").toInt();
        msg.time       = time_sec;

        // get temperature_sensor
        sio::message::ptr temperature_sensor = data->get_map()["temperature_sensor"];
        if(!temperature_sensor || temperature_sensor->get_flag() != sio::message::flag_object)
        {
            // printf("[COMM_RRS][DEBUG] 'temperature_sensor' field missing or not object\n");
            return;
        }

        DATA_TEMPERATURE temperature_msg;
        temperature_msg.connection = get_json(temperature_sensor, "connection") == "true";
        temperature_msg.temperature_value   = get_json(temperature_sensor, "temperature_value").toFloat();
        temperature_msg.time       = time_sec;

        //        qDebug()<<QString::number(temperature_msg.temperature_value);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        _main->temperature_value = temperature_msg.temperature_value;

        //        qDebug()<<"temprature_msg.temperature_value : "<<temprature_msg.temperature_value;

        // debug
        // printf("[COMM_RRS][DEBUG] recv foot → conn: %d, pos: %d, down: %d, state: %d, time: %.3f\n",
        //        msg.connection, msg.position, msg.is_down, msg.state, msg.time);

        if(logger)
        {
            // logger->write_log(
            //             QString("[COMM_RRS] recv footState - conn: %1, pos: %2, down: %3, state: %4, time: %5")
            //             .arg(msg.connection)
            //             .arg(msg.position)
            //             .arg(msg.is_down)
            //             .arg(msg.state)
            //             .arg(msg.time),
            //             "Green"
            //             );
        }

        Q_EMIT signal_foot(msg);
    }
}

void COMM_RRS::recv_safety_io_set(const std::string& name, const sio::message::ptr& data, bool hasAck, sio::message::list& ack_resp)
{
     //printf("[COMM_RRS][DEBUG] recv_safety io called\n");
    if(config->set_debug_comm_rrs())
    {
        spdlog::debug("[COMM_RRS] recv_safety io called");
    }

    if(data && data->get_flag() == sio::message::flag_object)
    {
        QString time_str = get_json(data, "time");
        double time_sec = get_json(data, "time").toDouble() / 1000;


        // printf("[COMM_RRS][DEBUG] time: %s\n", time_str.toStdString().c_str());
        if(config->set_debug_comm_rrs())
        {
            spdlog::debug("[COMM_RRS] time: {}",time_str.toStdString());
        }

        // get safety io msg
        sio::message::ptr safetyio_msg = data->get_map()["safetyio"];
        if(!safetyio_msg || safetyio_msg->get_flag() != sio::message::flag_object)
        {
            // printf("[COMM_RRS][DEBUG] 'foot' field missing or not object\n");
            if(config->set_debug_comm_rrs())
            {
                spdlog::debug("[COMM_RRS] 'foot' field missing or not object");
            }
            return;
        }

        auto map = safetyio_msg->get_map();

        QJsonArray mcu0_dio_arr;
        for (auto& v : map["mcu0_dio"]->get_vector())
        {
            mcu0_dio_arr.append(static_cast<int>(v->get_int()));
        }

        QJsonArray mcu1_dio_arr;
        for (auto& v : map["mcu1_dio"]->get_vector())
        {
            mcu1_dio_arr.append(static_cast<int>(v->get_int()));
        }

        DATA_SAFTYIO msg;
        fillArray_memcpy(msg.mcu0_dio, 8, mcu0_dio_arr);
        fillArray_memcpy(msg.mcu1_dio, 8, mcu1_dio_arr);

        //        If there are a lot of ios that need to be changed
        for (int i = 0; i < 2; i++) // 0:mcu0, 1:mcu1
        {
            unsigned char* dio_arr = (i == 0) ? msg.mcu0_dio : msg.mcu1_dio;

            for (int n = 0; n < 8; n++) // 0~7 port
            {
                bool value = dio_arr[n]; // 0/1 value
                mobile->set_IO_individual_output(n, value);
            }
        }


        if(logger)
        {
            // logger->write_log(
            //             QString("[COMM_RRS] recv footState - conn: %1, pos: %2, down: %3, state: %4, time: %5")
            //             .arg(msg.connection)
            //             .arg(msg.position)
            //             .arg(msg.is_down)
            //             .arg(msg.state)
            //             .arg(msg.time),
            //             "Green"
            //             );
        }

        Q_EMIT signal_safety_io(msg);
    }
}

// send functions
void COMM_RRS::send_status()
{
    if(!is_connected || !config || !mobile || !unimap || !dctrl)
    {
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;
    MOBILE_STATUS ms = mobile->get_status();

    // Adding the imu object
    Eigen::Vector3d imu = mobile->get_imu();
    QJsonObject imuObj;
    imuObj["acc_x"]  = QString::number(ms.imu_acc_x, 'f', 3);
    imuObj["acc_y"]  = QString::number(ms.imu_acc_y, 'f', 3);
    imuObj["acc_z"]  = QString::number(ms.imu_acc_z, 'f', 3);
    imuObj["gyr_x"]  = QString::number(ms.imu_gyr_x * R2D, 'f', 3);
    imuObj["gyr_y"]  = QString::number(ms.imu_gyr_y * R2D, 'f', 3);
    imuObj["gyr_z"]  = QString::number(ms.imu_gyr_z * R2D, 'f', 3);
    imuObj["imu_rx"] = QString::number(imu[0] * R2D, 'f', 3);
    imuObj["imu_ry"] = QString::number(imu[1] * R2D, 'f', 3);
    imuObj["imu_rz"] = QString::number(imu[2] * R2D, 'f', 3);
    rootObj["imu"] = imuObj;

    // Adding the motor array
    QJsonArray motorArray;
    QJsonObject motorObj1;
    motorObj1["connection"] = (ms.connection_m0 == 1) ? "true" : "false";
    motorObj1["status"]     = QString::number(ms.status_m0);
    motorObj1["temp"]       = QString::number(ms.temp_m0, 'f', 3);
    motorObj1["current"]    = QString::number(static_cast<double>(ms.cur_m0) / 10.0, 'f', 3);
    motorArray.append(motorObj1);

    QJsonObject motorObj2;
    motorObj2["connection"] = (ms.connection_m1 == 1) ? "true" : "false";
    motorObj2["status"]     = QString::number(ms.status_m1);
    motorObj2["temp"]       = QString::number(ms.temp_m1, 'f', 3);
    motorObj2["current"]    = QString::number(static_cast<double>(ms.cur_m1) / 10.0, 'f', 3);
    motorArray.append(motorObj2);
    rootObj["motor"] = motorArray;

    // Adding the condition object
    Eigen::Vector2d ieir = loc->get_cur_ieir();
    QJsonObject conditionObj;
    conditionObj["inlier_error"]  = QString::number(ieir[0], 'f', 3);
    conditionObj["inlier_ratio"]  = QString::number(ieir[1], 'f', 3);
    conditionObj["mapping_error"] = QString::number(ieir[0], 'f', 3);
    conditionObj["mapping_ratio"] = QString::number(ieir[1], 'f', 3);
    rootObj["condition"] = conditionObj;

    // Adding the state object
    QString cur_loc_state = loc->get_cur_loc_state();
    QString charge_st_string = "none";

    QString robot_model = config->get_robot_model();
    if(robot_model == "D400" || robot_model == "MECANUM")
    {
        if(ms.charge_state == CHARGE_STATE_IDLE)
        {
            charge_st_string = "none";
        }
        else if(ms.charge_state == CHARGE_STATE_TRIG_TO_CHARGE)
        {
            charge_st_string = "ready";
        }
        else if(ms.charge_state == CHARGE_STATE_BATTERY_ON)
        {
            charge_st_string = "BAT_on";
        }
        else if(ms.charge_state == CHARGE_STATE_CHARGING)
        {
            charge_st_string = "charging";
        }
        else if(ms.charge_state == CHARGE_STATE_TRIG_TO_STOP_CHARGE)
        {
            charge_st_string = "finish";
        }
        else if(ms.charge_state == CHARGE_STATE_FAIL)
        {
            charge_st_string = "fail";
        }
    }
    else if(robot_model == "S100")
    {
        if(ms.charge_state == 0)
        {
            charge_st_string = "none";
        }
        else if(ms.charge_state == 1)
        {
            charge_st_string = "charging";
        }
    }
    bool is_dock = dctrl->get_dock_state();

    QJsonObject robotStateObj;
    robotStateObj["charge"]       = charge_st_string;
    robotStateObj["dock"]         = (is_dock == true) ? "true" : "false";
    robotStateObj["emo"]          = (ms.motor_stop_state == 1) ? "true" : "false";
    robotStateObj["localization"] = cur_loc_state; // "none", "good", "fail"
    robotStateObj["power"]        = (ms.power_state == 1) ? "true" : "false";
    rootObj["robot_state"]        = robotStateObj;

    auto toJsonArray = [](unsigned char arr[8])
    {
        QJsonArray jsonArr;
        for (int i = 0; i < 8; ++i)
            jsonArr.append(arr[i]);
        return jsonArr;
    };

    QJsonObject robotIOObj;
    robotIOObj["mcu0_dio"] = toJsonArray(ms.mcu0_dio);
    robotIOObj["mcu1_dio"] = toJsonArray(ms.mcu1_dio);
    robotIOObj["mcu0_din"] = toJsonArray(ms.mcu0_din);
    robotIOObj["mcu1_din"] = toJsonArray(ms.mcu1_din);
    rootObj["robot_safety_io_state"]  = robotIOObj;

    // Adding the power object
    QJsonObject powerObj;
    powerObj["bat_in"]         = QString::number(ms.bat_in, 'f', 3);
    powerObj["bat_out"]        = QString::number(ms.bat_out, 'f', 3);
    powerObj["bat_current"]    = QString::number(ms.bat_current, 'f', 3);
    powerObj["total_power"]    = QString::number(ms.total_power, 'f', 3);
    powerObj["power"]          = QString::number(ms.power, 'f', 3);
    powerObj["bat_percent"]    = QString::number(ms.bat_percent);
    powerObj["tabos_voltage"]  = QString::number(ms.tabos_voltage, 'f', 3);
    powerObj["tabos_current"]  = QString::number(ms.tabos_current, 'f', 3);
    powerObj["tabos_status"]   = QString::number(ms.tabos_status);
    powerObj["tabos_ttf"]      = QString::number(ms.tabos_ttf);
    powerObj["tabos_tte"]      = QString::number(ms.tabos_tte);
    powerObj["tabos_soc"]      = QString::number(ms.tabos_soc);
    powerObj["tabos_soh"]      = QString::number(ms.tabos_soh);
    powerObj["tabos_temp"]     = QString::number(ms.tabos_temperature, 'f', 3);
    powerObj["tabos_rc"]       = QString::number(ms.tabos_rc, 'f', 3);
    powerObj["tabos_ae"]       = QString::number(ms.tabos_ae, 'f' ,3);

    if(robot_model == "D400" || robot_model == "MECANUM")
    {
        powerObj["charge_current"]  = QString::number(ms.charge_current, 'f', 3);
        powerObj["contact_voltage"] = QString::number(ms.contact_voltage, 'f', 3);
    }
    else if(robot_model == "S100")
    {
        powerObj["charge_current"]  = QString::number(0.0, 'f', 3);
        powerObj["contact_voltage"] = QString::number(0.0, 'f', 3);
    }
    rootObj["power"] = powerObj;

    QJsonObject settingObj;
    settingObj["platform_type"] = config->get_robot_type();
    settingObj["platform_name"] = "";
    rootObj["setting"] = settingObj;

    QJsonObject mapObj;
    QString map_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED)
    {
        QString map_dir = unimap->get_map_path();
        QStringList map_dir_list = map_dir.split("/");
        if(map_dir_list.size() > 0)
        {
            map_name = unimap->get_map_path().split("/").last();
        }
    }

    QString map_status = "";
    int is_loaded = static_cast<int>(unimap->get_is_loaded());
    if(is_loaded == MAP_NOT_LOADED)
    {
        map_status = "none";
    }
    else if(is_loaded == MAP_LOADING)
    {
        map_status = "loading";
    }
    else if(is_loaded == MAP_LOADED)
    {
        map_status = "loaded";
    }

    mapObj["map_name"]   = map_name;
    mapObj["map_status"] = map_status;
    rootObj["map"] = mapObj;

    // usb temp sensor
    QJsonObject tempObj;
    tempObj["connection"] = QString::number(ms.bat_in, 'f', 3);
    tempObj["temp_sensor"] = QString::number(ms.bat_out, 'f', 3);

    // Adding the time object
    const double time = get_time();
    rootObj["time"] = QString::number(static_cast<long long>(time * 1000));

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        io->socket()->emit("status", res);
    }
}

// send functions
void COMM_RRS::send_move_status()
{
    if(!is_connected || !ctrl || !mobile || !unimap || !dctrl)
    {
        //printf("is_connected : %d\n", (int)is_connected.load());
        spdlog::info("is_connected : {}", (int)is_connected.load());
        return;
    }

    // Creating the JSON object
    QJsonObject rootObj;

    QString cur_node_id = ctrl->get_cur_node_id();

    // Adding the move state object
    QString auto_state = "stop";
    if(ctrl->get_is_pause())
    {
        auto_state = "pause";
    }
    else if(ctrl->get_is_moving())
    {
        auto_state = "move";
    }

    if(mobile->get_cur_pdu_state() != "good" || ctrl->get_multi_inter_lock())
    {
        auto_state = "not ready";
    }

    if(loc->get_cur_loc_state() != "good")
    {
        auto_state = "error";
    }

    if(ctrl->get_obs_condition() == "vir")
    {
        auto_state = "vir";
    }

    if(cur_node_id.isEmpty())
    {
        auto_state = "error";
    }

    QString dock_state = "stop";

    QString jog_state = "none";

    QJsonObject moveStateObj;
    moveStateObj["auto_move"] = auto_state; // "stop", "move", "pause", "error", "not ready", "vir"
    moveStateObj["dock_move"] = dock_state;
    moveStateObj["jog_move"] = jog_state;
    moveStateObj["obs"] = ctrl->get_obs_condition();
    moveStateObj["path"] = ctrl->get_multi_reqest_state(); // "none", "req_path", "recv_path"
    rootObj["move_state"] = moveStateObj;

    // Adding the pose object
    const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    const Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    QJsonObject poseObj;
    poseObj["x"] = QString::number(cur_xi[0], 'f', 3);
    poseObj["y"] = QString::number(cur_xi[1], 'f', 3);
    poseObj["rz"] = QString::number(cur_xi[2] * R2D, 'f', 3);
    rootObj["pose"] = poseObj;

    // Adding the velocity object
    MOBILE_POSE mo = mobile->get_pose();
    QJsonObject velObj;
    velObj["vx"] = QString::number(mo.vel[0], 'f', 3);
    velObj["vy"] = QString::number(mo.vel[1], 'f', 3);
    velObj["wz"] = QString::number(mo.vel[2] * R2D, 'f', 3);
    rootObj["vel"] = velObj;

    // Adding the cur_node object
    QString cur_node_name = "";
    if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
    {
        NODE* node = unimap->get_node_by_id(cur_node_id);
        if(node != nullptr)
        {
            cur_node_name = node->name;
        }
    }

    QJsonObject curNodeObj;
    curNodeObj["id"] = cur_node_id;
    curNodeObj["name"] = cur_node_name;
    curNodeObj["state"] = "";
    curNodeObj["x"] = QString::number(cur_xi[0], 'f', 3);
    curNodeObj["y"] = QString::number(cur_xi[1], 'f', 3);
    curNodeObj["rz"] = QString::number(cur_xi[2] * R2D, 'f', 3);
    rootObj["cur_node"] = curNodeObj;

    // Adding the goal_node object
    QString goal_state = ctrl->get_cur_move_state();
    QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;
    QString goal_node_name = "";
    Eigen::Vector3d goal_xi(0, 0, 0);
    if(unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
    {
        NODE* node = unimap->get_node_by_id(goal_node_id);
        if(node != nullptr)
        {
            goal_node_name = node->name;
            goal_xi = TF_to_se2(node->tf);
        }
    }

    QJsonObject goalNodeObj;
    goalNodeObj["id"] = goal_node_id;
    goalNodeObj["name"] = goal_node_name;
    goalNodeObj["state"] = goal_state; // "", "move", "complete", "fail", "obstacle", "cancel"
    goalNodeObj["x"] = QString::number(goal_xi[0], 'f', 3);
    goalNodeObj["y"] = QString::number(goal_xi[1], 'f', 3);
    goalNodeObj["rz"] = QString::number(goal_xi[2] * R2D, 'f', 3);
    rootObj["goal_node"] = goalNodeObj;

    // Adding the time object
    const double time = get_time();
    rootObj["time"] = QString::number(static_cast<long long>(time * 1000));

    QJsonDocument doc(rootObj);
    if(!doc.isNull())
    {
        sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
        io->socket()->emit("moveStatus", res);
    }
}

void COMM_RRS::send_local_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }

    PATH path = ctrl->get_cur_local_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(size_t p = 0; p < path.pos.size(); p++)
    {
        if(p == 0 || p == path.pos.size() - 1 || p % 10 == 0)
        {
            const Eigen::Vector3d P = path.pos[p];

            sio::array_message::ptr jsonObj = sio::array_message::create();
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[2], 'f', 3).toStdString()));
            jsonArray->get_vector().push_back(jsonObj);
        }
    }

    // send
    io->socket()->emit("localPath", jsonArray);
}

void COMM_RRS::send_global_path()
{
    if(!is_connected || !ctrl)
    {
        return;
    }

    PATH path = ctrl->get_cur_global_path();

    sio::array_message::ptr jsonArray = sio::array_message::create();
    for(const Eigen::Vector3d& P : path.pos)
    {
        sio::array_message::ptr jsonObj = sio::array_message::create();
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[0], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[1], 'f', 3).toStdString()));
        jsonObj->get_vector().push_back(sio::string_message::create(QString::number(P[2], 'f', 3).toStdString()));
        jsonArray->get_vector().push_back(jsonObj);
    }

    // send
    io->socket()->emit("globalPath", jsonArray);
}

void COMM_RRS::send_lidar_2d()
{

    if (!is_connected || !loc || !lidar_2d)
    {
        return;
    }

    std::vector<Eigen::Vector3d> pts = lidar_2d->get_cur_frm().pts;

    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    if(pts.size() > 0)
    {
        sio::object_message::ptr rootObject = sio::object_message::create();
        sio::object_message::ptr poseObject = sio::object_message::create();
        poseObject->get_map()["x"] = sio::string_message::create(QString::number(cur_xi[0], 'f', 3).toStdString());
        poseObject->get_map()["y"] = sio::string_message::create(QString::number(cur_xi[1], 'f', 3).toStdString());
        poseObject->get_map()["rz"] = sio::string_message::create(QString::number(cur_xi[2]*R2D, 'f', 3).toStdString());

        rootObject->get_map()["pose"] = poseObject;

        sio::array_message::ptr jsonArray = sio::array_message::create();

        // initalize vector, get pts -> 1 deg
        std::vector<Eigen::Vector3d> sample_pts(360, Eigen::Vector3d(NAN,NAN,NAN));

        for(size_t p=0; p<pts.size(); p++)
        {
            double yaw_rad = std::atan2(pts[p][1], pts[p][0]);
            double yaw_deg = yaw_rad * R2D;
            if(yaw_deg < 0)
            {
                yaw_deg += 360.0;
            }

            int idx = static_cast<int>(yaw_deg) % 360;
            double dist = std::sqrt(pts[p][0]*pts[p][0] + pts[p][1]*pts[p][1]);

            // if multiple values with similar angles appear, update the one with the shorter distance.
            if(std::isnan(sample_pts[idx][0]) || dist < std::sqrt(sample_pts[idx][0]*sample_pts[idx][0] + sample_pts[idx][1]*sample_pts[idx][1]))
            {
                sample_pts[idx] = Eigen::Vector3d(pts[p][0], pts[p][1], pts[p][2]);
            }
        }

        for(int i=0; i<360; i++)
        {
            // fill missing sample point with previous point
            if(std::isnan(sample_pts[i][0]))
            {
                int prev = (i-1+360)%360;
                sample_pts[i] = sample_pts[prev];  // add pre pts
            }

            sio::array_message::ptr jsonObj = sio::array_message::create();
            if(!std::isnan(sample_pts[i][0]))
            {
                jsonObj->get_vector().push_back(sio::string_message::create(QString::number(sample_pts[i][0],'f',3).toStdString()));
                jsonObj->get_vector().push_back(sio::string_message::create(QString::number(sample_pts[i][1],'f',3).toStdString()));
                jsonObj->get_vector().push_back(sio::string_message::create(QString::number(sample_pts[i][2],'f',3).toStdString()));
            }
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(100,'f',3).toStdString()));
            jsonArray->get_vector().push_back(jsonObj);
        }
//        std::cout << jsonArray->get_vector().size() << std::endl;

        rootObject->get_map()["data"] = jsonArray;
        io->socket()->emit("lidarCloud", rootObject);

    }

}


void COMM_RRS::send_lidar_3d()
{
    if (!is_connected || !loc || !lidar_2d || !lidar_3d)
    {
        return;
    }

    std::vector<Eigen::Vector3d> pts = lidar_2d->get_cur_frm().pts;
    Eigen::Matrix4d cur_tf = loc->get_cur_tf();
    Eigen::Vector3d cur_xi = TF_to_se2(cur_tf);
    if(pts.size() > 0)
    {
        sio::object_message::ptr rootObject = sio::object_message::create();
        sio::object_message::ptr poseObject = sio::object_message::create();
        poseObject->get_map()["x"] = sio::string_message::create(QString::number(cur_xi[0], 'f', 3).toStdString());
        poseObject->get_map()["y"] = sio::string_message::create(QString::number(cur_xi[1], 'f', 3).toStdString());
        poseObject->get_map()["rz"] = sio::string_message::create(QString::number(cur_xi[2]*R2D, 'f', 3).toStdString());
        rootObject->get_map()["pose"] = poseObject;

        sio::array_message::ptr jsonArray = sio::array_message::create();
        for(size_t p = 0; p < pts.size(); p++)
        {
            sio::array_message::ptr jsonObj = sio::array_message::create();

            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pts[p][2], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(100, 'f', 3).toStdString()));

            jsonArray->get_vector().push_back(jsonObj);
        }

        rootObject->get_map()["data"] = jsonArray;
    }
}

void COMM_RRS::send_mapping_cloud()
{
    if(!is_connected || !mapping || !config)
    {
        return;
    }

    if(mapping->get_is_mapping() && last_send_kfrm_idx < static_cast<int>(mapping->get_kfrm_storage_size()))
    {
        // send kfrm
        KFRAME kfrm = mapping->get_kfrm(last_send_kfrm_idx);
        const int accum_num = config->get_mapping_icp_do_accum_num();

        Eigen::Matrix3d R0 = kfrm.opt_G.block(0, 0, 3, 3);
        Eigen::Vector3d t0 = kfrm.opt_G.block(0, 3, 3, 1);

        sio::array_message::ptr jsonArray = sio::array_message::create();
        for(const auto& pt : kfrm.pts)
        {
            if(pt.do_cnt < accum_num)
            {
                continue;
            }

            Eigen::Vector3d P(pt.x, pt.y, pt.z);
            Eigen::Vector3d _P = R0 * P + t0;

            sio::array_message::ptr jsonObj = sio::array_message::create();

            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[0], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[1], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(_P[2], 'f', 3).toStdString()));
            jsonObj->get_vector().push_back(sio::string_message::create(QString::number(pt.r, 'f', 3).toStdString()));

            jsonArray->get_vector().push_back(jsonObj);
        }

        // send
        io->socket()->emit("mappingCloud", jsonArray);
        last_send_kfrm_idx++;
    }
}

void COMM_RRS::slot_move(DATA_MOVE msg)
{
    if(!mobile || !unimap || !obsmap || !ctrl || !loc)
    {
        return;
    }

    MOBILE_STATUS ms = mobile->get_status();
    int bat_percent = ms.bat_percent;

    const QString command = msg.command;
    if(command == "jog")
    {
        // action
        const double vx = msg.jog_val[0];
        const double vy = msg.jog_val[1];
        const double wz = msg.jog_val[2];

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->update_jog_values(vx, vy, wz);
        }
    }
    else if(command == "target")
    {
        const QString method = msg.method;
        if(method == "pp")
        {
            // action
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::MOVE_TARGET);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::MOVE_TARGET);
                send_move_response(msg);
                return;
            }

            if(!loc->get_is_loc())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::LOC_NOT_INIT, ERROR_MANAGER::MOVE_TARGET);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::LOC_NOT_INIT, ERROR_MANAGER::MOVE_TARGET);
                send_move_response(msg);
                return;
            }

            const double x = msg.tgt_pose_vec[0];
            const double y = msg.tgt_pose_vec[1];
            if(x < unimap->get_map_min_x() || x > unimap->get_map_max_x() || y < unimap->get_map_min_y() || y > unimap->get_map_max_y())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MOVE_TARGET_OUT_RANGE, ERROR_MANAGER::MOVE_TARGET);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::MOVE_TARGET_OUT_RANGE, ERROR_MANAGER::MOVE_TARGET);
                send_move_response(msg);
                return;
            }

            const Eigen::Vector4d pose_vec = msg.tgt_pose_vec;
            const Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(pose_vec[0], pose_vec[1], pose_vec[3] * D2R));
            if(obsmap->is_tf_collision(goal_tf))
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MOVE_TARGET_OCCUPIED, ERROR_MANAGER::MOVE_TARGET);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::MOVE_TARGET_OCCUPIED, ERROR_MANAGER::MOVE_TARGET);
                send_move_response(msg);
                return;
            }

            if(config->get_use_multi())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_MULTI_MODE_LIMIT, ERROR_MANAGER::MOVE_TARGET);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::SYS_MULTI_MODE_LIMIT, ERROR_MANAGER::MOVE_TARGET);
                send_move_response(msg);
                return;
            }

            msg.result = "accept";
            msg.message = "";
            msg.bat_percent = bat_percent;

            send_move_response(msg);

            // pure pursuit
            Q_EMIT ctrl->signal_move(msg);
        }
        else
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_TARGET);

            ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_TARGET);
            send_move_response(msg);
        }
    }
    else if(command == "goal" || command == "change_goal")
    {
        const QString method = msg.method;
        if(method == "pp")
        {
            if(unimap->get_is_loaded() != MAP_LOADED)
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::MOVE_GOAL);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::MOVE_GOAL);
                send_move_response(msg);
                return;
            }

            if(!loc->get_is_loc())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::LOC_NOT_INIT, ERROR_MANAGER::MOVE_GOAL);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::LOC_NOT_INIT, ERROR_MANAGER::MOVE_GOAL);
                send_move_response(msg);
                return;
            }

            if(mobile->get_is_inter_lock_foot())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
                send_move_response(msg);
                return;
            }

            QString goal_id = msg.goal_node_id;
            if(goal_id.isEmpty())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MOVE_EMPTY_NODE_ID, ERROR_MANAGER::MOVE_GOAL);
                msg.bat_percent = bat_percent;

                ERROR_MANAGER::logError(ERROR_MANAGER::MOVE_EMPTY_NODE_ID, ERROR_MANAGER::MOVE_GOAL);
                send_move_response(msg);
                return;
            }

            NODE* node = unimap->get_node_by_id(goal_id);
            if(!node)
            {
                node = unimap->get_node_by_name(goal_id);
                if(!node)
                {
                    msg.result = "reject";
                    msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MOVE_NODE_NOT_FOUND, ERROR_MANAGER::MOVE_GOAL);
                    msg.bat_percent = bat_percent;

                    ERROR_MANAGER::logError(ERROR_MANAGER::MOVE_NODE_NOT_FOUND, ERROR_MANAGER::MOVE_GOAL);
                    send_move_response(msg);
                    return;
                }

                // convert name to id
                msg.goal_node_id = node->id;
                msg.goal_node_name = node->name;
            }
            else
            {
                msg.goal_node_name = node->name;
            }

            const Eigen::Matrix4d cur_tf = loc->get_cur_tf();
            const Eigen::Vector3d cur_pos = cur_tf.block(0, 3, 3, 1);
            msg.cur_pos = cur_pos;

            const Eigen::Vector3d xi = TF_to_se2(node->tf);
            msg.tgt_pose_vec[0] = xi[0];
            msg.tgt_pose_vec[1] = xi[1];
            msg.tgt_pose_vec[2] = node->tf(2, 3);
            msg.tgt_pose_vec[3] = xi[2];

            msg.bat_percent = bat_percent;

            // calc eta (estimation time arrival)
            const Eigen::Matrix4d goal_tf = node->tf;
            PATH global_path = ctrl->calc_global_path(goal_tf);
            if(global_path.pos.size() < 2)
            {
                msg.result = "accept";
                msg.message = "success";
                msg.remaining_time = 0.0;
            }
            else
            {
                msg.result = "accept";
                msg.message = "success";
            }

            send_move_response(msg);

            // pure pursuit
            Q_EMIT (ctrl->signal_move(msg));
        }
        else if(method == "hpp")
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            msg.bat_percent = bat_percent;

            ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            send_move_response(msg);
        }
        else if(method == "tng")
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            msg.bat_percent = bat_percent;

            ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            send_move_response(msg);
        }
        else
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            msg.bat_percent = bat_percent;

            ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::MOVE_GOAL);
            send_move_response(msg);
        }
    }
    else if(command == "pause")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        ctrl->set_is_pause(true);
    }
    else if(command == "resume")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        ctrl->set_is_pause(false);
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";
        msg.bat_percent = bat_percent;
        send_move_response(msg);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_Emergency();
        }
    }
}

void COMM_RRS::slot_mapping(DATA_MAPPING msg)
{
    const QString command = msg.command;
    if(command == "start")
    {
        if(lidar_2d && lidar_2d->get_is_connected())
        {
            msg.result = "accept";
            msg.message = "";

            send_mapping_response(msg);

            last_send_kfrm_idx = 0;
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                _main->bt_MapBuild();
            }
        }
        else
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SENSOR_LIDAR_DISCON, ERROR_MANAGER::MAPPING_START);

            ERROR_MANAGER::logError(ERROR_MANAGER::SENSOR_LIDAR_DISCON, ERROR_MANAGER::MAPPING_START);
            send_mapping_response(msg);
        }
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";

        send_mapping_response(msg);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_MapSave();
        }
    }
    else if(command == "save")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            if(msg.map_name != "")
            {
                _main ->change_map_name = true;
            }
            _main -> map_dir =  msg.map_name;
            _main->bt_MapSave();

            const QString map_name = msg.map_name;
            const QString save_dir = "/data/maps/" + map_name;

            const std::string command = "cp -r " + unimap->get_map_path().toStdString() + " '" + save_dir.toStdString() + "'";
            const int result = std::system(command.c_str());
            if(result == 0)
            {
                msg.result = "success";
                msg.message = "";

                send_mapping_response(msg);
            }
            else
            {
                msg.result = "fail";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_COPY_FAILED, ERROR_MANAGER::MAPPING_SAVE);

                ERROR_MANAGER::logError(ERROR_MANAGER::MAP_COPY_FAILED, ERROR_MANAGER::MAPPING_SAVE);
                send_mapping_response(msg);
            }
        }
    }
    else if(command == "reload")
    {
        msg.result = "accept";
        msg.message = "";

        send_mapping_response(msg);

        last_send_kfrm_idx = 0;
    }
}

void COMM_RRS::slot_load(DATA_LOAD msg)
{
    const QString command = msg.command;
    if(command == "mapload")
    {
        const QString map_name = msg.map_name;
        const QString load_dir = "/data/maps/" + map_name;

        if(!load_dir.isNull())
        {
            if(!QDir(load_dir).exists())
            {
                msg.result = "reject";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_INVALID_PATH, ERROR_MANAGER::LOAD_MAP);

                ERROR_MANAGER::logError(ERROR_MANAGER::MAP_INVALID_PATH, ERROR_MANAGER::LOAD_MAP);
                send_load_response(msg);
                return;
            }

            if(loc)
            {
                loc->stop();
            }
            if(obsmap)
            {
                obsmap->clear();
            }

            if(config)
            {
                config->set_map_path(load_dir);
            }

            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                if(unimap)
                {
                    unimap->load_map(load_dir);
                }
                _main->all_update();
            }

            if(unimap && unimap->get_is_loaded() == MAP_LOADED)
            {
                msg.result = "success";
                msg.message = "";
                send_load_response(msg);
            }
            else
            {
                msg.result = "fail";
                msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_LOAD_FAILED, ERROR_MANAGER::LOAD_MAP);

                ERROR_MANAGER::logError(ERROR_MANAGER::MAP_LOAD_FAILED, ERROR_MANAGER::LOAD_MAP);
                send_load_response(msg);
            }
        }
    }
    else if(command == "topoload")
    {
        MOBILE_STATUS ms = mobile->get_status();
        if(config->get_use_sim() || ms.motor_stop_state ==1)
        {
            const QString map_name = msg.map_name;
            const QString load_dir = "/data/maps/" + map_name;
            //qDebug()<<load_dir;

            if(!load_dir.isNull())
            {
                if(!QDir(load_dir).exists())
                {
                    msg.result = "reject";
                    msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_INVALID_PATH, ERROR_MANAGER::LOAD_TOPO);

                    ERROR_MANAGER::logError(ERROR_MANAGER::MAP_INVALID_PATH, ERROR_MANAGER::LOAD_TOPO);
                    send_load_response(msg);
                    return;
                }

                MainWindow* _main = qobject_cast<MainWindow*>(main);
                if(_main)
                {
                    if(unimap)
                    {
                        unimap->load_node();
                    }
                    _main->all_update();
                }

                else
                {
                    msg.result = "fail";
                    msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::TOPO_LOAD_FAILED, ERROR_MANAGER::LOAD_TOPO);

                    ERROR_MANAGER::logError(ERROR_MANAGER::TOPO_LOAD_FAILED, ERROR_MANAGER::LOAD_TOPO);
                    send_load_response(msg);
                }
            }

        }

        //if(unimap && unimap->get_is_loaded() == MAP_LOADED)
        //{
        //    msg.result = "success";
        //    msg.message = "";
        //    send_load_response(msg);
        //}


        //msg.result = "reject";
        //msg.message = "[R0Sx0301]not support yet";

        //send_load_response(msg);
    }
    else if(command == "configload")
    {
        msg.result = "reject";
        msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::LOAD_CONFIG);

        ERROR_MANAGER::logError(ERROR_MANAGER::SYS_NOT_SUPPORTED, ERROR_MANAGER::LOAD_CONFIG);
        send_load_response(msg);
    }
}

void COMM_RRS::slot_randomseq(DATA_RANDOMSEQ msg)
{
    const QString command = msg.command;
    if(command == "randomseq")
    {
        msg.result = "accept";
        msg.message = "";

        send_randomseq_response(msg);

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->slot_sim_random_seq();
        }
    }
}

void COMM_RRS::slot_localization(DATA_LOCALIZATION msg)
{
    const QString command = msg.command;
    if(command == "semiautoinit")
    {
        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::LOC_SEMI_AUTO);

            ERROR_MANAGER::logError(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::LOC_SEMI_AUTO);
            send_localization_response(msg);
            return;
        }

        if(!lidar_2d || !lidar_2d->get_is_connected())
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::LOC_SENSOR_ERROR, ERROR_MANAGER::LOC_SEMI_AUTO);

            ERROR_MANAGER::logError(ERROR_MANAGER::LOC_SENSOR_ERROR, ERROR_MANAGER::LOC_SEMI_AUTO);
            send_localization_response(msg);
            return;
        }

        if(!loc || loc->get_is_busy())
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::LOC_ALREADY_RUNNING, ERROR_MANAGER::LOC_SEMI_AUTO);

            ERROR_MANAGER::logError(ERROR_MANAGER::LOC_ALREADY_RUNNING, ERROR_MANAGER::LOC_SEMI_AUTO);
            send_localization_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_localization_response(msg);

        // do process
        if(logger)
        {
            logger->write_log("[AUTO_INIT] recv_loc, try semi-auto init", "Green", true, false);
        }
        spdlog::info("[COMM_RRS][AUTO_INIT] recv_loc, try semi-auto init");

        if(loc)
        {
            loc->stop();
        }

        // semi auto init
        if(semi_auto_init_thread)
        {
            if(logger)
            {
                logger->write_log("[AUTO_INIT] recv_loc, thread already running.", "Orange", true, false);
            }
            spdlog::info("[COMM_RRS][AUTO_INIT] recv_loc, thread already running");
            if(semi_auto_init_thread->joinable())
            {
                semi_auto_init_thread->join();
            }
            semi_auto_init_thread.reset();
        }

        if(loc)
        {
            semi_auto_init_thread = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);
        }
    }
    else if(command == "init")
    {
        if(!unimap || unimap->get_is_loaded() != MAP_LOADED)
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::LOC_INIT);

            ERROR_MANAGER::logError(ERROR_MANAGER::MAP_NOT_LOADED, ERROR_MANAGER::LOC_INIT);
            send_localization_response(msg);
            return;
        }
        if(!lidar_2d || !lidar_2d->get_is_connected())
        {
            msg.result = "reject";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::LOC_SENSOR_ERROR, ERROR_MANAGER::LOC_INIT);

            ERROR_MANAGER::logError(ERROR_MANAGER::LOC_SENSOR_ERROR, ERROR_MANAGER::LOC_INIT);
            send_localization_response(msg);
            return;
        }

        msg.result = "accept";
        msg.message = "";
        send_localization_response(msg);

        // manual init
        const double x = msg.tgt_pose_vec[0];
        const double y = msg.tgt_pose_vec[1];
        const double test = msg.tgt_pose_vec[2];
        const double rz = msg.tgt_pose_vec[3];

        if(logger)
        {
            logger->write_log(QString("[COMM_RRS] recv, command: %1, (x,y,test,th,th_test):%2,%3,%4,%5,%6 time: %7").arg(msg.command).arg(x).arg(y).arg(test).arg(rz).arg(rz * D2R).arg(msg.time), "Green");
        }
        spdlog::info("[COMM_RRS][INIT] recv_loc, command: {}, (x,y,test,th,th_test):{:.3f},{:.3f},{:.3f},{:.3f},{:.6f} time: {:.6f})",msg.command.toStdString(),x,y,test,rz,rz*D2R, msg.command.toStdString());

        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));
        if(loc)
        {
            loc->stop();
            loc->set_cur_tf(tf);
        }
    }
    else if(command == "start")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(loc->get_is_loc())
        {
            loc->stop();
        }

        const double x = msg.tgt_pose_vec[0];
        const double y = msg.tgt_pose_vec[1];
        const double rz = msg.tgt_pose_vec[3];
        const Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));

        if(loc)
        {
            loc->set_cur_tf(tf);
            loc->start();
        }
    }
    else if(command == "stop")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        if(loc)
        {
            loc->stop();
        }
    }
    else if(command == "randominit")
    {
        msg.result = "accept";
        msg.message = "";

        send_localization_response(msg);

        const QString seed = msg.seed;

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->slot_sim_random_init(seed);
        }
    }
}

void COMM_RRS::slot_field_set(DATA_FIELD msg)
{

    const QString command = msg.command;

    if(command == "set")
    {
        msg.result = "success";
        //qDebug() << "slot msg.sef_field:" << msg.set_field;
        spdlog::info("[COMM_RRS] set, slot msg.sef_filed: {}", msg.set_field);

        unsigned int set_field_ = msg.set_field;
        msg.message = "";

        //qDebug() << "parsing filed set:" << set_field_;
        spdlog::info("[COMM_RRS] set, parsing filed set: {}", msg.set_field);

        if(mobile)
        {
            MOBILE::instance()->setlidarfield(set_field_);
        }

        send_field_set_response(msg);
    }

    else if (command == "get")
    {
        msg.result = "success";
        msg.message = "";

        if(mobile)
        {
            MOBILE_STATUS ms = MOBILE::instance()->get_status();
            //qDebug() << ms.lidar_field;
            spdlog::info("[COMM_RRS] get, slot ms.lidar_field: {}", ms.lidar_field);
            msg.get_field = ms.lidar_field;
        }

        send_field_get_response(msg);

    }
}

void COMM_RRS::slot_dock(DATA_DOCK msg)
{
    const QString command = msg.command;
    if(command == "dock")
    {
        msg.result = "accept";
        msg.message = "";

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_DockStart();
        }

        send_dock_response(msg);
    }
    else if(command == "undock")
    {
        msg.result = "accept";
        msg.message = "";

        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->bt_UnDockStart();
        }

        send_dock_response(msg);
    }
}

void COMM_RRS::slot_view_lidar(DATA_VIEW_LIDAR msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                //_main->lidar_view_frequency = msg.frequency;
            }
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->lidar_view_frequency = -1;
        }
    }
}

void COMM_RRS::slot_view_path(DATA_VIEW_PATH msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(msg.frequency > 0)
        {
            MainWindow* _main = qobject_cast<MainWindow*>(main);
            if(_main)
            {
                //_main->path_view_frequency = msg.frequency;
            }
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            //_main->path_view_frequency = -1;
        }
    }
}

void COMM_RRS::slot_led(DATA_LED msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->is_user_led = true;

            const QString led = msg.led;
            if(led == "none")
            {
                _main->user_led_color = LED_OFF;
            }
            else if(led == "red")
            {
                _main->user_led_color = LED_RED;
            }
            else if(led == "blue")
            {
                _main->user_led_color = LED_BLUE;
            }
            else if(led == "white")
            {
                _main->user_led_color = LED_WHITE;
            }
            else if(led == "green")
            {
                _main->user_led_color = LED_GREEN;
            }
            else if(led == "magenta")
            {
                _main->user_led_color = LED_MAGENTA;
            }
            else if(led == "yellow")
            {
                _main->user_led_color = LED_YELLOW;
            }
            else if(led == "red blink")
            {
                _main->user_led_color = LED_RED_BLINK;
            }
            else if(led == "blue blink")
            {
                _main->user_led_color = LED_BLUE_BLINK;
            }
            else if(led == "white blink")
            {
                _main->user_led_color = LED_WHITE_BLINK;
            }
            else if(led == "green blink")
            {
                _main->user_led_color = LED_GREEN_BLINK;
            }
            else if(led == "magenta blink")
            {
                _main->user_led_color = LED_MAGENTA_BLINK;
            }
            else if(led == "yellow blink")
            {
                _main->user_led_color = LED_YELLOW_BLINK;
            }

            msg.result = "accept";
            msg.message = "";
        }
    }
    else if(command == "off")
    {
        MainWindow* _main = qobject_cast<MainWindow*>(main);
        if(_main)
        {
            _main->is_user_led = false;

            msg.result = "accept";
            msg.message = "";
        }
    }
}

void COMM_RRS::slot_motor(DATA_MOTOR msg)
{
    const QString command = msg.command;
    if(command == "on")
    {
        if(mobile)
        {
            mobile->motor_on();
        }

        msg.result = "accept";
        msg.message = "";
    }
    else if(command == "off")
    {
        if(mobile)
        {
            mobile->motor_off();
        }

        msg.result = "accept";
        msg.message = "";
    }
}

void COMM_RRS::slot_path(DATA_PATH msg)
{
    const QString command = msg.command;
    if(command == "path")
    {
        if(ctrl)
        {
            ctrl->signal_path(msg);
        }
    }
}

void COMM_RRS::slot_vobs(DATA_VOBS msg)
{
    const QString command = msg.command;
    if(command == "vobs" && obsmap && unimap)
    {
        std::vector<Eigen::Vector3d> vobs_r_list;
        {
            const QString vobs_str = msg.vobs_robots;
            const QStringList vobs_str_list = vobs_str.split("\n", Qt::SkipEmptyParts);
            vobs_r_list.reserve(vobs_str_list.size());
            for(const QString& vobs_line : vobs_str_list)
            {
                const QStringList vobs_str_list2 = vobs_line.split(",", Qt::SkipEmptyParts);
                if(vobs_str_list2.size() == 3)
                {
                    Eigen::Vector3d P;
                    P[0] = vobs_str_list2[0].toDouble();
                    P[1] = vobs_str_list2[1].toDouble();
                    P[2] = vobs_str_list2[2].toDouble();
                    vobs_r_list.push_back(P);
                }
            }
        }

        std::vector<Eigen::Vector3d> vobs_c_list;
        {
            const QString vobs_str = msg.vobs_clousers;
            const QStringList vobs_str_list = vobs_str.split(",", Qt::SkipEmptyParts);

            // set vobs
            vobs_c_list.reserve(vobs_str_list.size());
            for(const QString& node_id : vobs_str_list)
            {
                if(!node_id.isEmpty())
                {
                    NODE* node = unimap->get_node_by_id(node_id);
                    if(node)
                    {
                        vobs_c_list.push_back(node->tf.block(0, 3, 3, 1));
                    }
                }
            }
        }

        // update vobs
        {
            obsmap->update_vobs_list_robots(vobs_r_list);
            if(msg.is_vobs_closures_change == "true")
            {
                obsmap->update_vobs_list_closures(vobs_c_list);
            }
        }

        obsmap->update_vobs_map();
    }
}

void COMM_RRS::slot_software_update(DATA_SOFTWARE msg)
{
    if(!is_connected || !mobile)
    {
        return;
    }

    MOBILE_STATUS ms = mobile->get_status();
    if(ms.charge_state == 0 || ms.motor_stop_state >= 1)
    {
        //bool is_not_charging = ms.charge_state != 1 ? true : false;
        if(ms.charge_state == 0)
        {
            msg.result = "false";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::BAT_NOT_CHARGING, ERROR_MANAGER::SOFTWARE_UPDATE);

            ERROR_MANAGER::logError(ERROR_MANAGER::BAT_NOT_CHARGING, ERROR_MANAGER::SOFTWARE_UPDATE);
            send_software_update_response(msg);
        }

        if(ms.motor_stop_state >= 1)
        {
            msg.result = "false";
            msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SAFETY_EMO_RELEASED, ERROR_MANAGER::SOFTWARE_UPDATE);

            ERROR_MANAGER::logError(ERROR_MANAGER::SAFETY_EMO_RELEASED, ERROR_MANAGER::SOFTWARE_UPDATE);
            send_software_update_response(msg);
        }
        return;
    }

    const QString version_str = "--version=" + msg.version;
    QString homePath = QDir::homePath();
    QString scriptPath = homePath + "/rainbow-deploy-kit/slamnav2/slamnav2-update.sh";

    QProcess process;
    QStringList arguments;
    arguments << version_str;

    process.start("bash", QStringList() << scriptPath << arguments);

    if(!process.waitForStarted())
    {
        msg.result = "false";
        msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_PROCESS_START_FAILED, ERROR_MANAGER::SOFTWARE_UPDATE);

        ERROR_MANAGER::logError(ERROR_MANAGER::SYS_PROCESS_START_FAILED, ERROR_MANAGER::SOFTWARE_UPDATE);
        send_software_update_response(msg);
        return;
    }

    if(!process.waitForFinished())
    {
        msg.result = "false";
        msg.message = ERROR_MANAGER::getErrorMessage(ERROR_MANAGER::SYS_PROCESS_FINISH_FAILED, ERROR_MANAGER::SOFTWARE_UPDATE);

        ERROR_MANAGER::logError(ERROR_MANAGER::SYS_PROCESS_FINISH_FAILED, ERROR_MANAGER::SOFTWARE_UPDATE);
        send_software_update_response(msg);
        return;
    }

    const int exitCode = process.exitCode();
    const QProcess::ExitStatus exitStatus = process.exitStatus();

    if(exitStatus == QProcess::NormalExit && exitCode == 0)
    {
        msg.result = "true";
        msg.message = "update succeeded";
    }
    else
    {
        msg.result = "false";

        QString stdOut = process.readAllStandardOutput();
        QString stdErr = process.readAllStandardError();

        msg.message = QString("exitCode:%1\nstdout:\n%2\nstderr:\n%3")
                .arg(exitCode)
                .arg(QString(stdOut))
                .arg(QString(stdErr));
    }

    send_software_update_response(msg);
}


// for interlock
void COMM_RRS::slot_foot(DATA_FOOT msg)
{
    if(msg.state == FOOT_STATE_DONE && msg.is_down == true)
    {
        // logger->write_log(QString("[MOBILE] slot_foot state:%1, set inter lock true").arg(msg.state), "Green");
        spdlog::info("[COMM_RRS][MOBILE] slot_foot state:{}, set inter lock true",static_cast<int>(msg.state));
        mobile->set_is_inter_lock_foot(true);
    }
    else
    {
        // logger->write_log(QString("[MOBILE] slot_foot state:%1, set inter lock false").arg(msg.state), "Green");
        spdlog::info("[COMM_RRS][MOBILE] slot_foot state:{}, set inter lock false",static_cast<int>(msg.state));
        mobile->set_is_inter_lock_foot(false);
    }
}

// for safetyio
void COMM_RRS::slot_safety_io(DATA_SAFTYIO msg)
{
    send_safetyio_response(msg);
}

void COMM_RRS::send_move_response(const DATA_MOVE& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;

    QJsonObject errorCode;
    if(msg.result == "reject" || msg.result == "fail")
    {
        QString error_code = "R0Sx0000";
        QString alarm_code = "0000";
        QString category = "System";
        QString cause = "Unknown";
        QString level = "error";
        QString solution = "Contact system administrator required";
        
        // Error code mapping

        /* Map mangement */
        if(msg.message.contains("[R0Mx1001]") || msg.message.contains("1001") )
        {
            error_code = "R0Mx1001";
            alarm_code = "1001";
            category = "Map Management";
            cause = "Map Not Loaded";
            level = "error";
            solution = "Load map file required";
        }
        else if(msg.message.contains("[R0Mx1002]") || msg.message.contains("1002"))
        {
            error_code = "R0Mx1002";
            alarm_code = "1002";
            category = "Map Management";
            cause = "Map Path Error";
            level = "error";
            solution = "Check map path required";
        }
        else if(msg.message.contains("[R0Mx1003]") || msg.message.contains("1003"))
        {
            error_code = "R0Mx1003";
            alarm_code = "1003";
            category = "Map Management";
            cause = "Map Load Failed";
            level = "error";
            solution = "System restart required";
        }

        /* Localization */
        else if(msg.message.contains("[R0Lx2001]") || msg.message.contains("2001"))
        {
            error_code = "R0Lx2001";
            alarm_code = "2001";
            category = "Localization";
            cause = "Localization Not Initialized";
            level = "error";
            solution = "Restart localization required";
        }
        else if(msg.message.contains("[R0Lx2002]") || msg.message.contains("2002"))
        {
            error_code = "R0Lx2002";
            alarm_code = "2002";
            category = "Localization";
            cause = "Localization Sensor Error";
            level = "error";
            solution = "Check LiDAR hardware required";
        }
        else if(msg.message.contains("[R0Lx2003]") || msg.message.contains("2003"))
        {
            error_code = "R0Lx2003";
            alarm_code = "2003";
            category = "Localization";
            cause = "Localization Already Running";
            level = "warn";
            solution = "Terminate existing process required";
        }

        /* Navigation */
        else if(msg.message.contains("[R0Nx3001]") || msg.message.contains("3001"))
        {
            error_code = "R0Nx3001";
            alarm_code = "3001";
            category = "Navigation";
            cause = "No Target";
            level = "error";
            solution = "Set target required";
        }
        else if(msg.message.contains("[R0Nx3002]")|| msg.message.contains("3002"))
        {
            error_code = "R0Nx3002";
            alarm_code = "3002";
            category = "Navigation";
            cause = "Target Invalid";
            level = "error";
            solution = "Reset target required";
        }
        else if(msg.message.contains("[R0Nx3003]")|| msg.message.contains("3003"))
        {
            error_code = "R0Nx3003";
            alarm_code = "3003";
            category = "Navigation";
            cause = "Target Position Occupied";
            level = "error";
            solution = "Change target position required";
        }
        else if(msg.message.contains("[R0Nx3004]")|| msg.message.contains("3004"))
        {
            error_code = "R0Nx3004";
            alarm_code = "3004";
            category = "Navigation";
            cause = "Target Out of Range";
            level = "error";
            solution = "Reset target position required";
        }
        else if(msg.message.contains("[R0Nx3005]") || msg.message.contains("3005"))
        {
            error_code = "R0Nx3005";
            alarm_code = "3005";
            category = "Navigation";
            cause = "Node Not Found";
            level = "error";
            solution = "Check node ID/name required";
        }
        else if(msg.message.contains("[R0Nx3006]") || msg.message.contains("3006"))
        {
            error_code = "R0Nx3006";
            alarm_code = "3006";
            category = "Navigation";
            cause = "Empty Node ID";
            level = "error";
            solution = "Enter target node ID required";
        }

        /* Sensor */
        else if(msg.message.contains("[R0Sx4001]") || msg.message.contains("4001"))
        {
            error_code = "R0Sx4001";
            alarm_code = "4001";
            category = "Sensor";
            cause = "LiDAR Disconnected";
            level = "error";
            solution = "Check LiDAR hardware required";
        }

        /* System */
        else if(msg.message.contains("[R0Sx5001]") || msg.message.contains("5001"))
        {
            error_code = "R0Sx5001";
            alarm_code = "5001";
            category = "System";
            cause = "Function Not Supported";
            level = "warn";
            solution = "Use supported method required";
        }
        else if(msg.message.contains("[R0Sx5002]") || msg.message.contains("5002"))
        {
            error_code = "R0Sx5002";
            alarm_code = "5002";
            category = "System";
            cause = "Multi Mode Limitation";
            level = "warn";
            solution = "Use goal command required";
        }
        
        errorCode["error_code"] = error_code;
        errorCode["alarm_code"] = alarm_code;
        errorCode["category"] = category;
        errorCode["cause"] = cause;
        errorCode["level"] = level;
        errorCode["solution"] = solution;
        errorCode["timestamp"] = QDateTime::currentDateTime().toUTC().toString(Qt::ISODate);
        obj["message_detail"] = errorCode;
    }

    obj["preset"] = QString::number(msg.preset, 10);
    obj["method"] = msg.method;
    obj["goal_id"] = msg.goal_node_id;
    obj["remaining_dist"] = QString::number(msg.remaining_dist, 'f', 3);
    obj["eta"] = QString::number(msg.remaining_time, 'f', 3);
    obj["bat_percent"] = QString::number(msg.bat_percent, 'f', 3);

    // temporal patch
    QString response_goal_node_name = msg.goal_node_name;
    if(msg.goal_node_name.contains("AMR-WAITING-01"))
    {
        response_goal_node_name = "AMR-WAITING-01";
    }
    else if(msg.goal_node_name.contains("AMR-CHARGING-01"))
    {
        response_goal_node_name = "AMR-CHARGING-01";
    }
    else if(msg.goal_node_name.contains("AMR-PACKING-01"))
    {
        response_goal_node_name = "AMR-PACKING-01";
    }
    else if(msg.goal_node_name.contains("AMR-CONTAINER-01"))
    {
        response_goal_node_name = "AMR-CONTAINER-01";
    }
    obj["goal_name"] = response_goal_node_name;
    obj["cur_x"] = QString::number(msg.cur_pos[0], 'f', 3);
    obj["cur_y"] = QString::number(msg.cur_pos[1], 'f', 3);
    obj["cur_z"] = QString::number(msg.cur_pos[2], 'f', 3);
    obj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
    obj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
    obj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
    obj["rz"] = QString::number(msg.tgt_pose_vec[3] * R2D, 'f', 3);
    obj["vx"] = QString::number(msg.jog_val[0], 'f', 3);
    obj["vy"] = QString::number(msg.jog_val[1], 'f', 3);
    obj["wz"] = QString::number(msg.jog_val[2], 'f', 3);
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("moveResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_localization_response(const DATA_LOCALIZATION& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["x"] = QString::number(msg.tgt_pose_vec[0], 'f', 3);
    obj["y"] = QString::number(msg.tgt_pose_vec[1], 'f', 3);
    obj["z"] = QString::number(msg.tgt_pose_vec[2], 'f', 3);
    obj["rz"] = QString::number(msg.tgt_pose_vec[3], 'f', 3);
    obj["seed"] = msg.seed;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("localizationResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_load_response(const DATA_LOAD& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["name"] = msg.map_name;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("loadResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_randomseq_response(const DATA_RANDOMSEQ& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("randomseqResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_mapping_response(const DATA_MAPPING& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["name"] = msg.map_name;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);



    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("mappingResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_dock_response(const DATA_DOCK& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("dockResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_field_set_response(const DATA_FIELD& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("fieldResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_field_get_response(const DATA_FIELD& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["command"] = msg.command;
    obj["result"] = msg.result;
    obj["message"] = msg.message;
    obj["get_field"] = QString::number(msg.get_field);
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("fieldResponse", res);

    // for plot
    mtx.lock();
    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
    mtx.unlock();
}

void COMM_RRS::send_path_response(const DATA_PATH& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["time"] = QString::number((long long)(msg.time*1000), 10);

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("pathResponse", res);
}

void COMM_RRS::send_software_update_response(const DATA_SOFTWARE& msg)
{
    if(!is_connected)
    {
        return;
    }

    QJsonObject obj;
    obj["version"] = msg.version;
    obj["applyReqUpdate"] = msg.result;
    obj["rejectReason"] = msg.message;

    printf("[SEND_SW_RESPONSE] version:%s, applyReqUpdate:%s, rejectReason:%s\n",
           msg.version.toStdString().c_str(),
           msg.result.toStdString().c_str(),
           msg.message.toStdString().c_str());

    QJsonDocument doc(obj);
    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
    io->socket()->emit("swUpdateResponse", res);
}

void COMM_RRS::send_safetyio_response(const DATA_SAFTYIO& msg)
{
    // 사실 응답이 필요 없을 수도 있음.
//    if(!is_connected)
//    {
//        return;
//    }

//    QJsonArray arr0;
//    for (int i = 0; i < 8; i++)
//    {
//        arr0.append(msg.mcu0_dio[i]);
//    }
//    obj["mcu0_dio"] = arr0;

//    QJsonArray arr1;
//    for (int i = 0; i < 8; i++)
//    {
//        arr0.append(msg.mcu0_dio[i]);
//    }
//    obj["mcu1_dio"] = arr1;

//    for (int i = 0; i < 8; i++)
//    {
//        arr2.append(msg.mcu0_din[i]);
//    }
//        obj["mcu0_din"] = arr2;

//    QJsonArray arr3;
//    for (int i = 0; i < 8; i++)
//    {
//        arr3.append(msg.mcu1_din[i]);
//    }
//    obj["mcu1_din"] = arr3;

//    QJsonDocument doc(obj);
//    sio::message::ptr res = sio::string_message::create(doc.toJson().toStdString());
//    io->socket()->emit("safetyioResponce", res);

//    // for plot
//    mtx.lock();
//    lastest_msg_str = doc.toJson(QJsonDocument::Indented);
//    mtx.unlock();
}

void COMM_RRS::set_config_module(CONFIG* _config)
{
    if(_config)
    {
        config = _config;
    }
}

void COMM_RRS::set_logger_module(LOGGER* _logger)
{
    if(_logger)
    {
        logger = _logger;
    }
}

void COMM_RRS::set_mobile_module(MOBILE* _mobile)
{
    if(_mobile)
    {
        mobile = _mobile;
    }
}

void COMM_RRS::set_lidar_2d_module(LIDAR_2D* _lidar)
{
    if(_lidar)
    {
        lidar_2d = _lidar;
    }
}

void COMM_RRS::set_cam_module(CAM* _cam)
{
    if(_cam)
    {
        cam = _cam;
    }
}

void COMM_RRS::set_localization_module(LOCALIZATION* _loc)
{
    if(_loc)
    {
        loc = _loc;
    }
}

void COMM_RRS::set_mapping_module(MAPPING* _mapping)
{
    if(_mapping)
    {
        mapping = _mapping;
    }
}

void COMM_RRS::set_unimap_module(UNIMAP* _unimap)
{
    if(_unimap)
    {
        unimap = _unimap;
    }
}

void COMM_RRS::set_obsmap_module(OBSMAP* _obsmap)
{
    if(_obsmap)
    {
        obsmap = _obsmap;
    }
}

void COMM_RRS::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if(_ctrl)
    {
        ctrl = _ctrl;
    }
}

void COMM_RRS::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if(_dctrl)
    {
        dctrl = _dctrl;
    }
}

void COMM_RRS::set_global_path_update()
{
    is_global_path_update2 = true;
}

void COMM_RRS::set_local_path_update()
{
    is_local_path_update2 = true;
}

// Modifying part of sending LiDAR data
// working at 10[ms]
void COMM_RRS::send_loop()
{
    if(!is_connected)
    {
        return;
    }

    // Synchronize with the development version
    // 100[ms]
    if(send_cnt % COMM_RRS_INFO::send_move_status_cnt == 0)
    {
        send_move_status();
    }

    // 500[ms]
    if(send_cnt % COMM_RRS_INFO::send_status_cnt == 0)
    {
        send_status();
    }

    // for variable loop
    double time_lidar_view = 1.0/((double)lidar_view_frequency + 1e-06) * 10.0;
    if(time_lidar_view > 0)
    {
        if(lidar_view_cnt > time_lidar_view)
        {
            lidar_view_cnt = 0;

            if(CONFIG::instance()->get_use_lidar_2d())
            {
                send_lidar_2d();
            }

            if(CONFIG::instance()->get_use_lidar_3d())
            {
                send_lidar_3d();
            }
        }
        lidar_view_cnt++;
    }

    double time_path_view = 1.0/((double)path_view_frequency + 1e-06) * 10.0;
    if(time_path_view > 0)
    {
        if(path_view_cnt > time_path_view)
        {
            path_view_cnt = 0;
            send_local_path();
        }
        path_view_cnt++;
    }

    // to give information video streaming data
    //if(config->get_use_rtsp() && config->get_use_cam())
    //{
    //    if(send_cnt % 100 == 0)
    //    {
    //        std::vector<bool> rtsp_flag = cam->get_rtsp_flag();
    //        if(rtsp_flag.size() != 0)
    //        {
    //            for(int p = 0; p < rtsp_flag.size(); p++)
    //            {
    //                QString msg = QString("[COMM] cam%1 rtsp writer %2").arg(p)
    //                                                                    .arg(rtsp_flag[p] ? "open success" : "open failed");
    //                logger->write_log(msg);
    //            }
    //        }
    //    }
    //}

    send_cnt++;
    if(send_cnt > 10000)
    {
        send_cnt = 0;
    }
}
