#ifndef ERROR_MANAGER_H
#define ERROR_MANAGER_H

#include <QString>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QFileInfoList>
#include "slamnav_common_types.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/fmt.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

class LOGGER;

class ERROR_MANAGER : public QObject
{
    Q_OBJECT

private:
    explicit ERROR_MANAGER(QObject *parent = nullptr);
    static ERROR_MANAGER* _instance;

public:
    static ERROR_MANAGER* instance(QObject* parent = nullptr);
    ~ERROR_MANAGER();

    std::recursive_mutex mtx;

    // other modules
    LOGGER *logger = nullptr;

    // module setters
    void set_logger_module(LOGGER* _logger) 
    { 
        logger = _logger; 
    }

    enum ErrorCause 
    {
        // Map Management (0x1000)
        MAP_NOT_LOADED                      = 0x1001,
        MAP_INVALID_PATH                    = 0x1002,
        MAP_LOAD_FAILED                     = 0x1003,
        MAP_COPY_FAILED                     = 0x1004,
        MAP_TOPO_LOAD_FAILED                = 0x1005,
        MAP_UNKNOWN_ERROR                   = 0x1006,
        MAP_SAVE_FAIL_CSV                   = 0x1007,
        MAP_LOAD_INVALID_DIR                = 0x1008,
        MAP_LOAD_NO_2D_MAP                  = 0x1010,
        MAP_LOAD_NO_3D_MAP                  = 0x1011,

        // Localization (0x2000)            
        LOC_NOT_INIT                        = 0x2001,
        LOC_SENSOR_ERROR                    = 0x2002,
        LOC_ALREADY_RUNNING                 = 0x2003,
        LOC_INIT_FAILED                     = 0x2004,

        // Navigation (0x3000)          
        MOVE_NO_TARGET                      = 0x3001,
        MOVE_TARGET_INVALID                 = 0x3002,
        MOVE_TARGET_OCCUPIED                = 0x3003,
        MOVE_TARGET_OUT_RANGE               = 0x3004,
        MOVE_NODE_NOT_FOUND                 = 0x3005,
        MOVE_EMPTY_NODE_ID                  = 0x3006,
        MOVE_INIT_CONDITION_FAILED          = 0x3007,
        MOVE_TARGET_NOT_SUPPORTED_MULTI     = 0x3008,
        MOVE_TARGET_OUT_OF_RANGE            = 0x3009,
        MOVE_TARGET_OCCUPIED_STATIC_OBS     = 0x3010,
        MOVE_METHOD_NOT_SUPPORTED           = 0x3011,
        MOVE_UNKNOWN_ERROR                  = 0x3012,


        // Sensor (0x4000)  
        SENSOR_LIDAR_DISCON                 = 0x4001,
        SENSOR_LIDAR_DATA_ERROR             = 0x4002,
        SENSOR_LIDAR_CALIB_ERROR            = 0x4003,
        SENSOR_IMU_DISCON                   = 0x4004,
        SENSOR_IMU_DATA_ERROR               = 0x4005,
        SENSOR_CAM_DISCON                   = 0x4006,
        SENSOR_CAM_DATA_ERROR               = 0x4007,
        SENSOR_QR_ERROR                     = 0x4008,
        SENSOR_TEMP_ERROR                   = 0x4009,

        // System (0x5000)      
        SYS_NOT_SUPPORTED                   = 0x5001,
        SYS_MULTI_MODE_LIMIT                = 0x5002,
        SYS_PROCESS_START_FAILED            = 0x5003,
        SYS_PROCESS_FINISH_FAILED           = 0x5004,
        SYS_NETWORK_ERROR                   = 0x5005,
        SYS_NOT_READY                       = 0x5006,
        SYS_UNKNOWN_ERROR                   = 0x5007,
        SYS_CONFIG_ERROR                    = 0x5008,

        // Safety (0x6000)      
        SAFETY_EMO_RELEASED                 = 0x6001,
        SAFETY_EMO_PRESSED                  = 0x6002,
        SAFETY_BUMPER_PRESSED               = 0x6003,
        SAFETY_OBS_DETECTED                 = 0x6004,
        SAFETY_ZONE_VIOLATION               = 0x6005,
        SAFETY_FIELD_ERROR                  = 0x6006,

        // Battery (0x7000)     
        BAT_NOT_CHARGING                    = 0x7001,
        BAT_LOW                             = 0x7002,
        BAT_CRITICAL                        = 0x7003,
        BAT_POWER_ERROR                     = 0x7004,

        // Motor (0x8000)       
        MOTOR_CONNECTION_LOST               = 0x8001,
        MOTOR_OVERHEAT                      = 0x8002,
        MOTOR_OVERLOAD                      = 0x8003,
        MOTOR_ENCODER_ERROR                 = 0x8004,

        // Communication RRS (0x9000)
        RRS_CONNECTION_FAILED               = 0x9001,
        RRS_TIMEOUT                         = 0x9002,
        RRS_INVALID_MESSAGE                 = 0x9003,
        RRS_RESPONSE_ERROR                  = 0x9004,

        // Cobot1 (0xA000)      
        COBOT1_CONNECTION_FAILED            = 0xA001,
        COBOT1_COMMAND_FAILED               = 0xA002,
        COBOT1_STATUS_ERROR                 = 0xA003,
        COBOT1_EMERGENCY_STOP               = 0xA004,
        COBOT1_TIMEOUT                      = 0xA005,

        // Cobot2 (0xB000)      
        COBOT2_CONNECTION_FAILED            = 0xB001,
        COBOT2_COMMAND_FAILED               = 0xB002,
        COBOT2_STATUS_ERROR                 = 0xB003,
        COBOT2_EMERGENCY_STOP               = 0xB004,
        COBOT2_TIMEOUT                      = 0xB005,

        // MSA (0xC000)     
        MSA_CONNECTION_FAILED               = 0xC001,
        MSA_TIMEOUT                         = 0xC002,
        MSA_INVALID_MESSAGE                 = 0xC003,
        MSA_RESPONSE_ERROR                  = 0xC004, 
        MSA_UNKNOWN_ERROR                   = 0xC005,

        // Profile movement (0xD000)
        PROFILE_MOVE_FAILED                 = 0xD001,
        PROFILE_MOVE_TIMEOUT                = 0xD002,
        PROFILE_MOVE_X_INVALID_TARGET_SPEED   = 0xD003,
        PROFILE_MOVE_Y_INVALID_TARGET_SPEED   = 0xD004,
        PROFILE_MOVE_CIRCLE_INVALID_TARGET_SPEED   = 0xD005,
        PROFILE_MOVE_ROTATE_INVALID_TARGET_SPEED   = 0xD006
    };

    enum ErrorContext 
    {
        CONTROL_UNKNOWN_CMD,

        DOCK_START,
        DOCK_STOP,

        MOVE_TARGET,
        MOVE_GOAL,
        MOVE_PROFILE,
        MOVE_STOP,

        LOAD_MAP,
        LOAD_TOPO,
        LOAD_CONFIG,
        LOAD_UNKNOWN_ERROR,

        MAPPING_START,
        MAPPING_SAVE,

        SAVE_MAP,

        LOC_SEMI_AUTO,
        LOC_INIT,
        LOC_START,
        LOC_STOP,
        LOC_RANDOM_INIT,

        RANDOM_SEQ,
        SAFETY_RESET_FIELD_ERROR,

        SOFTWARE_UPDATE,


        LED_CONTROL,

        MOTOR_CONTROL,

        FIELD_SET,
        FIELD_GET,

        RRS_COMMUNICATION,
        MSA_COMMUNICATION,

        COBOT1_OPERATION,
        COBOT2_OPERATION
        
    };

    struct ErrorInfo 
    {
        QString error_code;
        QString alarm_code;
        QString category;
        QString cause;
        QString context;
        QString message;
        QString solution;
        QString description;
        QString level;
        QString remark;
    };

    void init();
    std::shared_ptr<spdlog::logger> getLogger();

    ErrorInfo getErrorInfo(ErrorCause cause, ErrorContext context);
    QString getErrorMessage(ErrorCause cause, ErrorContext context);
    
    void logError(ErrorCause cause, ErrorContext context, const QString& additional_info = "");
    void logError(ErrorCause cause, ErrorContext context, LOGGER* logger, const QString& additional_info = "");

    QJsonObject getErrorCodeMapping(const QString& message);
    
    ErrorCause findErrorCauseFromMessage(const QString& message);

private:
    std::shared_ptr<spdlog::logger> spd_logger = nullptr;
    QString log_path = "error_logs/";

    QString getCategoryName(ErrorCause cause);
    QString getCauseName(ErrorCause cause);
    QString getContextName(ErrorContext context);

Q_SIGNALS:

private Q_SLOTS:
};

#endif // ERROR_MANAGER_H