#include "error_manager.h"

namespace 
{
    const char* MODULE_NAME = "ERROR_MANAGER";
}

//std::shared_ptr<spdlog::logger> ERROR_MANAGER::spd_logger = nullptr;
//QString ERROR_MANAGER::log_path = "error_logs/";

ERROR_MANAGER* ERROR_MANAGER::_instance = nullptr;

ERROR_MANAGER* ERROR_MANAGER::instance(QObject* parent)
{
    if(_instance == nullptr)
    {
        _instance = new ERROR_MANAGER(parent);
    }
    return _instance;
}

ERROR_MANAGER::ERROR_MANAGER(QObject *parent)
    : QObject{parent}
{
    init();
}

ERROR_MANAGER::~ERROR_MANAGER()
{
    if (spd_logger)
    {
        spd_logger->info("ERROR_MANAGER shutting down");
        spdlog::drop("error_manager");
        spd_logger = nullptr;
    }
}

void ERROR_MANAGER::init()
{
    if (spd_logger)
    {
        return; // Logger already initialized
    }

    
        try 
        {
            // create log directory
            QDir log_dir(log_path);
            if (!log_dir.exists())
            {
                log_dir.mkpath(".");
            }

            const QString date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd_HH-mm-ss");
            const QString log_name = log_path + date_time + "_ErrorManager.log";

            // max file save 
            QFileInfoList errorLogs = log_dir.entryInfoList(QStringList() << "*_ErrorManager.log", QDir::Files, QDir::Time);
            const int KeepFiles = 10;

            for (int i = KeepFiles; i < errorLogs.size(); ++i)
            {
                QFile::remove(errorLogs[i].absoluteFilePath());
            }

            if (spdlog::get("error_manager")) 
            {
                spdlog::drop("error_manager");
            }

            if(!QFile::exists(log_name))
            {
                QFile file(log_name);
                if(file.open(QIODevice::WriteOnly | QIODevice::Text))
                {
                    QTextStream out(&file);
                    //out << "<link rel=\"stylesheet\" href=\"log_styles.css\"><link>";
                    //out << "<body bgcolor=\"#252831\"></body>";
                    out<<"ERROR MANAGER LOG VERSION=1.0\n";
                    file.close();
                }
            }

            // create Multi-sink logger (file, console)
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_name.toStdString(), true);
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

            // set log format
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [ERROR_MANAGER] %v");
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [ERROR_MANAGER] %v");

            // create logger
            std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
            spd_logger = std::make_shared<spdlog::logger>("error_manager", sinks.begin(), sinks.end());

            // set log level
            spd_logger->set_level(spdlog::level::info);
            spd_logger->flush_on(spdlog::level::warn);

            // register as global logger (optional)
            spdlog::register_logger(spd_logger);

            spd_logger->info("ERROR_MANAGER logger initialized");

        } 
        catch (const spdlog::spdlog_ex& ex) 
        {
            qCritical() << "ERROR_MANAGER log initialization failed:" << ex.what();
        }
}

std::shared_ptr<spdlog::logger> ERROR_MANAGER::getLogger()
{
    if (!spd_logger) 
    {
        init();
    }

    return spd_logger;
}

ERROR_MANAGER::ErrorInfo ERROR_MANAGER::getErrorInfo(ErrorCause cause, ErrorContext context)
{
    ErrorInfo info;
    
    QString hex_str = QString::number(cause, 16).toUpper();
    QString alarm_code = hex_str;

    info.category = getCategoryName(cause);
    info.cause = getCauseName(cause);
    info.context = getContextName(context);
    info.alarm_code = alarm_code;

    switch(cause)
    {
    // Map
    case MAP_NOT_LOADED:            // 0x1001
        {
            info.error_code = "R0Mx" + info.alarm_code;
            info.message = "map not loaded";
            info.solution = "Load map file required";
            info.description = "Map is not loaded";
            info.level = "error";
            info.remark = "Navigation/Localization/Initialization unavailable";
            break;
        }

    case MAP_INVALID_PATH:          // 0x1002
        {
            info.error_code = "R0Mx" + info.alarm_code;
            info.message = "invalid map dir";
            info.solution = "Check map path required";
            info.description = "Invalid map directory";
            info.level = "error";
            info.remark = "Map load failed";
            break;
        }

    case MAP_LOAD_FAILED:           // 0x1003
        {
            info.error_code = "R0Mx" + info.alarm_code;
            info.message = "map not load";
            info.solution = "System restart required";
            info.description = "Map load failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case MAP_COPY_FAILED:           // 0x1004
        {
            info.error_code = "R0Mx" + info.alarm_code;
            info.message = "copy failed";
            info.solution = "Check storage space required";
            info.description = "Map copy failed";
            info.level = "warn";
            info.remark = "Storage error";
            break;
        }

    case MAP_TOPO_LOAD_FAILED:      // 0x1005
        {
            info.error_code = "R0Mx" + info.alarm_code;
            info.message = "topo not load";
            info.solution = "Check topology file required";
            info.description = "Topology load failed";
            info.level = "warn";
            info.remark = "Topology error";
            break;
        }

    // Localization
    case LOC_NOT_INIT:              // 0x2001
        {
            info.error_code = "R0Lx" + info.alarm_code;
            info.message = "no localization";
            info.solution = "Restart localization required";
            info.description = "No localization available";
            info.level = "error";
            info.remark = "Navigation unavailable";
            break;
        }

    case LOC_SENSOR_ERROR:          // 0x2002
        {
            info.error_code = "R0Lx" + info.alarm_code;
            info.message = "lidar not connected";
            info.solution = "Check LiDAR hardware required";
            info.description = "Localization sensor error";
            info.level = "error";
            info.remark = "Sensor connection error";
            break;
        }

    case LOC_ALREADY_RUNNING:       // 0x2003
        {
            info.error_code = "R0Lx" + info.alarm_code;
            info.message = "already running";
            info.solution = "Terminate existing process required";
            info.description = "Already running";
            info.level = "warn";
            info.remark = "Duplicate execution prevention";
            break;
        }

    case LOC_INIT_FAILED:           // 0x2004
        {
            info.error_code = "R0Lx" + info.alarm_code;
            info.message = "localization init failed";
            info.solution = "Reinitialize localization required";
            info.description = "Localization initialization failed";
            info.level = "error";
            info.remark = "Initialization failed";
            break;
        }

    // Move    
    case MOVE_NO_TARGET:            // 0x3001
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "no target specified";
            info.solution = "Set target required";
            info.description = "No target specified";
            info.level = "error";
            info.remark = "Target setting error";
            break;
        }

    case MOVE_TARGET_INVALID:       // 0x3002
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "target invalid";
            info.solution = "Reset target required";
            info.description = "Target is invalid";
            info.level = "error";
            info.remark = "Target validity error";
            break;
        }

    case MOVE_TARGET_OCCUPIED:      // 0x3003
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "target location occupied(static obs)";
            info.solution = "Change target position required";
            info.description = "Target position conflicts with static obstacle";
            info.level = "error";
            info.remark = "Target position collision";
            break;
        }

    case MOVE_TARGET_OUT_RANGE:     // 0x3004
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "target location out of range";
            info.solution = "Reset target position required";
            info.description = "Target position is out of map range";
            info.level = "error";
            info.remark = "Target range exceeded";
            break;
        }

    case MOVE_NODE_NOT_FOUND:       // 0x3005
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "can not find node";
            info.solution = "Check node ID/name required";
            info.description = "Node not found";
            info.level = "error";
            info.remark = "Target node missing";
            break;
        }

    case MOVE_EMPTY_NODE_ID:        // 0x3006
        {
            info.error_code = "R0Nx" + info.alarm_code;
            info.message = "empty node id";
            info.solution = "Enter target node ID required";
            info.description = "Empty node ID";
            info.level = "error";
            info.remark = "Target setting error";
            break;
        }

    
    // Sensor    
    case SENSOR_LIDAR_DISCON:       // 0x4001
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "lidar not connected";
            info.solution = "Check LiDAR hardware required";
            info.description = "LiDAR not connected";
            info.level = "error";
            info.remark = "Sensor connection error/Sensor initialization failed/Mapping unavailable";
            break;
        }
            
    case SENSOR_LIDAR_DATA_ERROR:   // 0x4002
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "lidar data error";
            info.solution = "Check LiDAR data quality required";
            info.description = "LiDAR data error";
            info.level = "warn";
            info.remark = "Data quality degradation";
            break;
        }

    case SENSOR_LIDAR_CALIB_ERROR:  // 0x4003
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "lidar calibration error";
            info.solution = "Re-run LiDAR calibration required";
            info.description = "LiDAR calibration error";
            info.level = "warn";
            info.remark = "Accuracy degradation";
            break;
        }
            
    case SENSOR_IMU_DISCON:         // 0x4004
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "IMU connection lost";
            info.solution = "Check IMU hardware required";
            info.description = "IMU connection lost";
            info.level = "error";
            info.remark = "Attitude control error";
            break;
        }

    case SENSOR_IMU_DATA_ERROR:     // 0x4005
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "IMU data error";
            info.solution = "Check IMU data quality required";
            info.description = "IMU data error";
            info.level = "warn";
            info.remark = "Attitude information error";
            break;
        }

    case SENSOR_CAM_DISCON:         // 0x4006
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "CAM connection lost";
            info.solution = "Check CAM hardware required";
            info.description = "CAM connection lost";
            info.level = "warn";
            info.remark = "CAM function limited";
            break;
        }

    case SENSOR_CAM_DATA_ERROR:     // 0x4007
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "CAM data error";
            info.solution = "Check CAM data quality required";
            info.description = "CAM data error";
            info.level = "info";
            info.remark = "CAM quality degradation";
            break;
        }

    case SENSOR_QR_ERROR:           // 0x4008
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "QR sensor error";
            info.solution = "Check QR sensor required";
            info.description = "QR sensor error";
            info.level = "warn";
            info.remark = "Recognition function limited";
            break;
        }

    case SENSOR_TEMP_ERROR:         // 0x4009
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "temperature sensor error";
            info.solution = "Check temperature sensor required";
            info.description = "Temperature sensor error";
            info.level = "info";
            info.remark = "Monitoring limited";
            break;
        }

    
    // System
    case SYS_NOT_SUPPORTED:         // 0x5001
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "not supported";
            info.solution = "Use supported method required";
            info.description = "Unsupported function";
            info.level = "warn";
            info.remark = "Function not supported";
            break;
        }

    case SYS_MULTI_MODE_LIMIT:      // 0x5002
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "target command not supported by multi. use goal_id";
            info.solution = "Use goal command required";
            info.description = "Using target command in multi-robot mode";
            info.level = "warn";
            info.remark = "Multi-mode limitation";
            break;
        }

    case SYS_PROCESS_START_FAILED:  // 0x5003
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "process failed to start";
            info.solution = "System restart required";
            info.description = "Process start failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case SYS_PROCESS_FINISH_FAILED: // 0x5004
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "process did not finish";
            info.solution = "Force terminate process and restart required";
            info.description = "Process completion failed";
            info.level = "error";
            info.remark = "System error";
            break;
        }

    case SYS_NETWORK_ERROR:         // 0x5005
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "network connection error";
            info.solution = "Check network connection required";
            info.description = "Network communication error";
            info.level = "warn";
            info.remark = "Communication limited";
            break;
        }

    // Safety
    case SAFETY_EMO_RELEASED:       // 0x6001
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "emo released";
            info.solution = "Check emergency stop status required";
            info.description = "Emergency stop released";
            info.level = "critical";
            info.remark = "Safety risk";
            break;
        }

    case SAFETY_EMO_PRESSED:        // 0x6002
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "EMO pressed";
            info.solution = "Release emergency stop required";
            info.description = "Emergency stop button pressed";
            info.level = "critical";
            info.remark = "Immediate stop";
            break;
        }

    case SAFETY_BUMPER_PRESSED:     // 0x6003
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "bumper pressed";
            info.solution = "Remove obstacle and restart required";
            info.description = "Bumper detected";
            info.level = "critical";
            info.remark = "Obstacle collision";
            break;
        }

    case SAFETY_OBS_DETECTED:       // 0x6004
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "obstacle detected";
            info.solution = "Change path or remove obstacle required";
            info.description = "Obstacle detected";
            info.level = "error";
            info.remark = "Path change required";
            break;
        }

    case SAFETY_ZONE_VIOLATION:     // 0x6005
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "safety zone violation";
            info.solution = "Resolve safety zone violation required";
            info.description = "Safety zone violation";
            info.level = "error";
            info.remark = "Safety risk";
            break;
        }

    // Battery
    case BAT_NOT_CHARGING:          // 0x7001
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "not charging";
            info.solution = "Check charging status required";
            info.description = "Not charging";
            info.level = "warn";
            info.remark = "Battery management";
            break;

        }

    case BAT_LOW:                   // 0x7002
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "low battery";
            info.solution = "Charging required";
            info.description = "Low battery";
            info.level = "error";
            info.remark = "Charging required";
            break;
        }

    case BAT_CRITICAL:              // 0x7003
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "battery critical";
            info.solution = "Immediate charging required";
            info.description = "Battery critical level";
            info.level = "critical";
            info.remark = "Immediate charging";
            break;
        }

    case BAT_POWER_ERROR:           // 0x7004
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "power supply error";
            info.solution = "Check power supply system required";
            info.description = "Power supply error";
            info.level = "error";
            info.remark = "System error";
            break;

        }

    // Motor
    case MOTOR_CONNECTION_LOST:     // 0x8001
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "motor connection lost";
            info.solution = "Check motor hardware required";
            info.description = "Motor connection lost";
            info.level = "critical";
            info.remark = "Navigation unavailable";
            break;
        }

    case MOTOR_OVERHEAT:            // 0x8002
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "motor overheat";
            info.solution = "Cool motor and restart required";
            info.description = "Motor overheating";
            info.level = "error";
            info.remark = "Motor protection";
            break;
        }

    case MOTOR_OVERLOAD:            // 0x8003
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "motor overload";
            info.solution = "Reduce motor load required";
            info.description = "Motor overload";
            info.level = "error";
            info.remark = "Performance limitation";
            break;
        }

    case MOTOR_ENCODER_ERROR:       // 0x8004
        {
            info.error_code = "R0Sx" + info.alarm_code;
            info.message = "motor encoder error";
            info.solution = "Check motor encoder required";
            info.description = "Motor encoder error";
            info.level = "error";
            info.remark = "Position control error";
            break;
        }

    // RRS
    case RRS_CONNECTION_FAILED:     // 0x9001
            {
                info.error_code = "R0Rx" + info.alarm_code; // R: RRS
                info.message = "RRS connection failed";
                info.solution = "Check RRS server status and network";
                info.description = "Failed to connect to RRS server";
                info.level = "critical";
                info.remark = "Remote control unavailable";
                break;
            }

    case RRS_TIMEOUT:               // 0x9002
            {
                info.error_code = "R0Rx" + info.alarm_code;
                info.message = "RRS response timeout";
                info.solution = "Check RRS server for responsiveness";
                info.description = "No response from RRS within the time limit";
                info.level = "warn";
                info.remark = "Communication delay";
                break;
            }

    case RRS_INVALID_MESSAGE:       // 0x9003
            {
                info.error_code = "R0Rx" + info.alarm_code;
                info.message = "invalid message from RRS";
                info.solution = "Check RRS message format";
                info.description = "Received a malformed or unexpected message from RRS";
                info.level = "warn";
                info.remark = "Protocol mismatch";
                break;
            }

    case RRS_RESPONSE_ERROR:        // 0x9004
            {
                info.error_code = "R0Rx" + info.alarm_code;
                info.message = "RRS returned an error";
                info.solution = "Check RRS server logs for details";
                info.description = "RRS responded with an error status to a request";
                info.level = "error";
                info.remark = "Command rejected by RRS";
                break;
            }
    

    // Cobot1        
    case COBOT1_CONNECTION_FAILED:   // 0xA001
        {
            info.error_code = "R0Cx" + info.alarm_code; // C: Cobot
            info.message = "cobot1 connection failed";
            info.solution = "Check Cobot1 power, cable, and network settings";
            info.description = "Failed to establish communication with the cobot1";
            info.level = "critical";
            info.remark = "Cobot1 operation unavailable";
            break;
        }
    case COBOT1_COMMAND_FAILED:      // 0xA002
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot1 command failed";
            info.solution = "Check cobot1 status and command validity";
            info.description = "The command sent to the cobot1 could not be executed";
            info.level = "error";
            info.remark = "Cobot1 task failed";
            break;
        }
    case COBOT1_STATUS_ERROR:        // 0xA003
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot1 status error";
            info.solution = "Reset cobot1 or clear the error";
            info.description = "The cobot1 is in an error or fault state";
            info.level = "error";
            info.remark = "Cobot1 requires intervention";
            break;
        }
    case COBOT1_EMERGENCY_STOP:      // 0xA004
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot1 E-stop pressed";
            info.solution = "Release the emergency stop button on the cobot1";
            info.description = "Cobot1 emergency stop is active";
            info.level = "critical";
            info.remark = "Immediate stop";
            break;
        }
    case COBOT1_TIMEOUT:             // 0xA005
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot1 response timeout";
            info.solution = "Check cobot1 controller and communication link";
            info.description = "The cobot1 did not respond to a command in time";
            info.level = "warn";
            info.remark = "Potential cobot1 freeze or network issue";
            break;
        }

    // Cobot2        
    case COBOT2_CONNECTION_FAILED:   // 0xB001
        {
            info.error_code = "R0Cx" + info.alarm_code; // C: Cobot
            info.message = "cobot2 connection failed";
            info.solution = "Check Cobot2 power, cable, and network settings";
            info.description = "Failed to establish communication with the cobot2";
            info.level = "critical";
            info.remark = "Cobot2 operation unavailable";
            break;
        }
    case COBOT2_COMMAND_FAILED:      // 0xB002
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot2 command failed";
            info.solution = "Check cobot2 status and command validity";
            info.description = "The command sent to the cobot2 could not be executed";
            info.level = "error";
            info.remark = "Cobot2 task failed";
            break;
        }
    case COBOT2_STATUS_ERROR:        // 0xB003
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot2 status error";
            info.solution = "Reset cobot2 or clear the error";
            info.description = "The cobot2 is in an error or fault state";
            info.level = "error";
            info.remark = "Cobot2 requires intervention";
            break;
        }
    case COBOT2_EMERGENCY_STOP:      // 0xB004
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot2 E-stop pressed";
            info.solution = "Release the emergency stop button on the cobot2";
            info.description = "Cobot2 emergency stop is active";
            info.level = "critical";
            info.remark = "Immediate stop";
            break;
        }
    case COBOT2_TIMEOUT:             // 0xB005
        {
            info.error_code = "R0Cx" + info.alarm_code;
            info.message = "cobot2 response timeout";
            info.solution = "Check cobot2 controller and communication link";
            info.description = "The cobot2 did not respond to a command in time";
            info.level = "warn";
            info.remark = "Potential cobot2 freeze or network issue";
            break;
        }    

        // MSA
    case MSA_CONNECTION_FAILED:     // 0xC001
        {
            info.error_code = "R0Ax" + info.alarm_code; // A: MSA
            info.message = "MSA connection failed";
            info.solution = "Check MSA server status and network";
            info.description = "Failed to connect to MSA server";
            info.level = "critical";
            info.remark = "MSA services unavailable";
            break;
        }
    case MSA_TIMEOUT:               // 0xC002
        {
            info.error_code = "R0Ax" + info.alarm_code;
            info.message = "MSA response timeout";
            info.solution = "Check MSA server for responsiveness";
            info.description = "No response from MSA within the time limit";
            info.level = "warn";
            info.remark = "Communication delay";
            break;
        }
    case MSA_INVALID_MESSAGE:       // 0xC003
        {
            info.error_code = "R0Ax" + info.alarm_code;
            info.message = "invalid message from MSA";
            info.solution = "Check MSA message format";
            info.description = "Received a malformed or unexpected message from MSA";
            info.level = "warn";
            info.remark = "Protocol mismatch";
            break;
        }
    case MSA_RESPONSE_ERROR:        // 0xC004
        {
            info.error_code = "R0Ax" + info.alarm_code;
            info.message = "MSA returned an error";
            info.solution = "Check MSA server logs for details";
            info.description = "MSA responded with an error status to a request";
            info.level = "error";
            info.remark = "Command rejected by MSA";
            break;
        }

    default:
        {
            info.error_code =   "[D]R0Sx0000";
            info.alarm_code =   "[D]0000";
            info.category =     "[D]System";
            info.cause =        "[D]Unknown";
            info.context =      "[D]UNKNOWN";
            info.message =      "[D]unknown error";
            info.solution =     "[D]Contact system administrator required";
            info.description =  "[D]Unknown error";
            info.level =        "[D]error";
            info.remark =       "[D]System error";
            break;
        }
    }
    
    return info;
}

QString ERROR_MANAGER::getErrorMessage(ErrorCause cause, ErrorContext context)
{
    ErrorInfo info = getErrorInfo(cause, context);
    return QString("[%1] %2, sol: %3").arg(info.alarm_code).arg(info.message).arg(info.solution);
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

    auto logger = getLogger();
    if (logger)
    {
        std::string msg = log_message.toStdString();
        if(info.level == "critical")
        {
            logger->critical("{}", msg);
        }
        else if(info.level == "error")
        {
            logger->error("{}", msg);
        }
        else if(info.level == "warn")
        {
            logger->warn("{}", msg);
        }
        else if(info.level == "info")
        {
            logger->info("{}", msg);
        }
        else
        {
            logger->debug("{}", msg);
        }
    }
    else
    {
        qWarning() << "ERROR_MANAGER logger not initialized. Log message:" << log_message;
    }
}

//void ERROR_MANAGER::logError(ErrorCause cause, ErrorContext context, LOGGER* logger, const QString& additional_info)
//{
//    ErrorInfo info = getErrorInfo(cause, context);
//    
//    QString log_message = QString("[%1] %2 - %3 (Solution: %4)")
//                         .arg(info.error_code)
//                         .arg(info.message)
//                         .arg(info.description)
//                         .arg(info.solution);
//    
//    if(!additional_info.isEmpty())
//    {
//        log_message += QString(" [Additional Info: %1]").arg(additional_info);
//    }
//    
//    // LOGGER 모듈 사용 (기존 방식)
//    if(logger)
//    {
//        QString color = "Black";
//        if(info.level == "critical")
//        {
//            color = "Red";
//        }
//        else if(info.level == "error")
//        {
//            color = "Red";
//        }
//        else if(info.level == "warn")
//        {
//            color = "Orange";
//        }
//        else
//        {
//            color = "Green";
//        }
//        
//        logger->write_log(log_message, color);
//    }
//    else
//    {
//        // LOGGER가 없으면 spdlog 직접 사용
//        logError(cause, context, additional_info);
//    }
//}


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
        case 0x9000:        return "COMM_RRS";
        case 0xA000:        return "Cobot";
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
        case MAP_TOPO_LOAD_FAILED:          return "Topology Load Failed";

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

        case RRS_CONNECTION_FAILED:         return "RRS Connection Failed";
        case RRS_TIMEOUT:                   return "RRS Timeout";
        case RRS_INVALID_MESSAGE:           return "RRS Invalid Message";
        case RRS_RESPONSE_ERROR:            return "RRS Response Error";

        case COBOT1_CONNECTION_FAILED:      return "Cobot1 Connection Failed";
        case COBOT1_COMMAND_FAILED:         return "Cobot1 Command Failed";
        case COBOT1_STATUS_ERROR:           return "Cobot1 Status Error";
        case COBOT1_EMERGENCY_STOP:         return "Cobot1 Emergency Stop";
        case COBOT1_TIMEOUT:                return "Cobot1 Timeout";

        default:                            return "Unknown";
    }
}

QString ERROR_MANAGER::getContextName(ErrorContext context)
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

        case RRS_COMMUNICATION:     return "RRS_COMMUNICATION";

        case COBOT_OPERATION:       return "COBOT_OPERATION";

        default:                    return "UNKNOWN";
    }
}

ERROR_MANAGER::ErrorCause ERROR_MANAGER::findErrorCauseFromMessage(const QString& message)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    
    // Map Management
    if(message.contains("[R0Mx1001]") || message.contains("1001"))
        return MAP_NOT_LOADED;
    else if(message.contains("[R0Mx1002]") || message.contains("1002"))
        return MAP_INVALID_PATH;
    else if(message.contains("[R0Mx1003]") || message.contains("1003"))
        return MAP_LOAD_FAILED;
    else if(message.contains("[R0Mx1004]") || message.contains("1004"))
        return MAP_COPY_FAILED;
    else if(message.contains("[R0Mx1005]") || message.contains("1005"))
        return MAP_TOPO_LOAD_FAILED;

    // Localization
    else if(message.contains("[R0Lx2001]") || message.contains("2001"))
        return LOC_NOT_INIT;
    else if(message.contains("[R0Lx2002]") || message.contains("2002"))
        return LOC_SENSOR_ERROR;
    else if(message.contains("[R0Lx2003]") || message.contains("2003"))
        return LOC_ALREADY_RUNNING;
    else if(message.contains("[R0Lx2004]") || message.contains("2004"))
        return LOC_INIT_FAILED;

    // Navigation
    else if(message.contains("[R0Nx3001]") || message.contains("3001"))
        return MOVE_NO_TARGET;
    else if(message.contains("[R0Nx3002]") || message.contains("3002"))
        return MOVE_TARGET_INVALID;
    else if(message.contains("[R0Nx3003]") || message.contains("3003"))
        return MOVE_TARGET_OCCUPIED;
    else if(message.contains("[R0Nx3004]") || message.contains("3004"))
        return MOVE_TARGET_OUT_RANGE;
    else if(message.contains("[R0Nx3005]") || message.contains("3005"))
        return MOVE_NODE_NOT_FOUND;
    else if(message.contains("[R0Nx3006]") || message.contains("3006"))
        return MOVE_EMPTY_NODE_ID;

    // Sensor
    else if(message.contains("[R0Sx4001]") || message.contains("4001"))
        return SENSOR_LIDAR_DISCON;
    else if(message.contains("[R0Sx4002]") || message.contains("4002"))
        return SENSOR_LIDAR_DATA_ERROR;
    else if(message.contains("[R0Sx4003]") || message.contains("4003"))
        return SENSOR_LIDAR_CALIB_ERROR;
    else if(message.contains("[R0Sx4004]") || message.contains("4004"))
        return SENSOR_IMU_DISCON;
    else if(message.contains("[R0Sx4005]") || message.contains("4005"))
        return SENSOR_IMU_DATA_ERROR;
    else if(message.contains("[R0Sx4006]") || message.contains("4006"))
        return SENSOR_CAM_DISCON;
    else if(message.contains("[R0Sx4007]") || message.contains("4007"))
        return SENSOR_CAM_DATA_ERROR;
    else if(message.contains("[R0Sx4008]") || message.contains("4008"))
        return SENSOR_QR_ERROR;
    else if(message.contains("[R0Sx4009]") || message.contains("4009"))
        return SENSOR_TEMP_ERROR;

    // System
    else if(message.contains("[R0Sx5001]") || message.contains("5001"))
        return SYS_NOT_SUPPORTED;
    else if(message.contains("[R0Sx5002]") || message.contains("5002"))
        return SYS_MULTI_MODE_LIMIT;
    else if(message.contains("[R0Sx5003]") || message.contains("5003"))
        return SYS_PROCESS_START_FAILED;
    else if(message.contains("[R0Sx5004]") || message.contains("5004"))
        return SYS_PROCESS_FINISH_FAILED;
    else if(message.contains("[R0Sx5005]") || message.contains("5005"))
        return SYS_NETWORK_ERROR;

    // Safety
    else if(message.contains("[R0Sx6001]") || message.contains("6001"))
        return SAFETY_EMO_RELEASED;
    else if(message.contains("[R0Sx6002]") || message.contains("6002"))
        return SAFETY_EMO_PRESSED;
    else if(message.contains("[R0Sx6003]") || message.contains("6003"))
        return SAFETY_BUMPER_PRESSED;
    else if(message.contains("[R0Sx6004]") || message.contains("6004"))
        return SAFETY_OBS_DETECTED;
    else if(message.contains("[R0Sx6005]") || message.contains("6005"))
        return SAFETY_ZONE_VIOLATION;

    // Battery
    else if(message.contains("[R0Bx7001]") || message.contains("7001"))
        return BAT_NOT_CHARGING;
    else if(message.contains("[R0Bx7002]") || message.contains("7002"))
        return BAT_LOW;
    else if(message.contains("[R0Bx7003]") || message.contains("7003"))
        return BAT_CRITICAL;
    else if(message.contains("[R0Bx7004]") || message.contains("7004"))
        return BAT_POWER_ERROR;

    // Motor
    else if(message.contains("[R0Mx8001]") || message.contains("8001"))
        return MOTOR_CONNECTION_LOST;
    else if(message.contains("[R0Mx8002]") || message.contains("8002"))
        return MOTOR_OVERHEAT;
    else if(message.contains("[R0Mx8003]") || message.contains("8003"))
        return MOTOR_OVERLOAD;
    else if(message.contains("[R0Mx8004]") || message.contains("8004"))
        return MOTOR_ENCODER_ERROR;

    // Communication RRS
    else if(message.contains("[R0Rx9001]") || message.contains("9001"))
        return RRS_CONNECTION_FAILED;
    else if(message.contains("[R0Rx9002]") || message.contains("9002"))
        return RRS_TIMEOUT;
    else if(message.contains("[R0Rx9003]") || message.contains("9003"))
        return RRS_INVALID_MESSAGE;
    else if(message.contains("[R0Rx9004]") || message.contains("9004"))
        return RRS_RESPONSE_ERROR;

    // Cobot1
    else if(message.contains("[R0CxA001]") || message.contains("A001"))
        return COBOT1_CONNECTION_FAILED;
    else if(message.contains("[R0CxA002]") || message.contains("A002"))
        return COBOT1_COMMAND_FAILED;
    else if(message.contains("[R0CxA003]") || message.contains("A003"))
        return COBOT1_STATUS_ERROR;
    else if(message.contains("[R0CxA004]") || message.contains("A004"))
        return COBOT1_EMERGENCY_STOP;
    else if(message.contains("[R0CxA005]") || message.contains("A005"))
        return COBOT1_TIMEOUT;

    return SYS_NOT_SUPPORTED; // default
}

QJsonObject ERROR_MANAGER::getErrorCodeMapping(const QString& message)
{
    std::lock_guard<std::recursive_mutex> lock(mtx);
    
    QJsonObject errorCode;
    
    // 메시지에서 에러 원인 찾기
    ErrorCause cause = findErrorCauseFromMessage(message);
    
    // 적절한 컨텍스트 결정 (메시지 내용을 기반으로)
    ErrorContext context = MOVE_GOAL; // 기본값
    
    if(message.contains("map") || message.contains("Map"))
    {
        context = LOAD_MAP;
    }
    else if(message.contains("topo") || message.contains("Topo"))
    {
        context = LOAD_TOPO;
    }
    else if(message.contains("localization") || message.contains("Localization"))
    {
        context = LOC_START;
    }
    else if(message.contains("mapping") || message.contains("Mapping"))
    {
        context = MAPPING_START;
    }
    else if(message.contains("move") || message.contains("Move"))
    {
        context = MOVE_TARGET;
    }
    else if(message.contains("goal") || message.contains("Goal"))
    {
        context = MOVE_GOAL;
    }
    else if(message.contains("dock") || message.contains("Dock"))
    {
        context = DOCK_START;
    }
    else if(message.contains("motor") || message.contains("Motor"))
    {
        context = MOTOR_CONTROL;
    }
    else if(message.contains("software") || message.contains("Software"))
    {
        context = SOFTWARE_UPDATE;
    }
    
    ErrorInfo errorInfo = getErrorInfo(cause, context);
    
    errorCode["error_code"] = errorInfo.error_code;
    errorCode["alarm_code"] = errorInfo.alarm_code;
    errorCode["category"] = errorInfo.category;
    errorCode["cause"] = errorInfo.cause;
    errorCode["level"] = errorInfo.level;
    errorCode["solution"] = errorInfo.solution;
    errorCode["timestamp"] = QDateTime::currentDateTime().toUTC().toString(Qt::ISODate);
    errorCode["remark"] = errorInfo.remark;
    
    return errorCode;
}