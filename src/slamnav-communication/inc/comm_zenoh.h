#ifndef COMM_ZENOH_H
#define COMM_ZENOH_H

// global defines
#include "slamnav_communication_types.h"

// other modules
#include "config.h"
#include "logger.h"
#include "timer_queue.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_3d.h"
#include "cam.h"
#include "localization.h"
#include "mapping.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "dockcontrol.h"

// zenoh
#include <zenoh.hxx>

// flatbuffers
#include <flatbuffers/flatbuffers.h>

// qt
#include <QObject>

// std
#include <queue>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <optional>

struct COMM_ZENOH_INFO
{
    static constexpr double status_send_time = 0.1;         // 100ms
    static constexpr double move_status_send_time = 0.5;    // 500ms
    static constexpr double mapping_cloud_send_time = 0.5;  // 500ms
};

// zenoh topic suffix definitions (runtime에 {robotType}/ prefix가 앞에 붙음)
namespace ZENOH_TOPIC
{
    // ---- Move RPC (slamnav_move.fbs) ----
    constexpr const char* MOVE_GOAL     = "move/goal";
    constexpr const char* MOVE_TARGET   = "move/target";
    constexpr const char* MOVE_STOP     = "move/stop";
    constexpr const char* MOVE_PAUSE    = "move/pause";
    constexpr const char* MOVE_RESUME   = "move/resume";
    constexpr const char* MOVE_XLINEAR  = "move/xLinear";
    constexpr const char* MOVE_CIRCULAR = "move/circular";
    constexpr const char* MOVE_ROTATE   = "move/rotate";

    // ---- Move Pub/Sub (slamnav_move.fbs) ----
    constexpr const char* MOVE_JOG          = "move/jog";           // Subscriber
    constexpr const char* MOVE_RESULT       = "move/result";        // Publisher
    // constexpr const char* MOVE_STATE_CHANGE = "move/stateChange"; // 미사용

    // ---- Localization RPC (slamnav_localization.fbs) ----
    constexpr const char* LOC_INIT           = "localization/init";
    constexpr const char* LOC_AUTOINIT       = "localization/autoinit";
    constexpr const char* LOC_SEMIAUTOINIT   = "localization/semiautoinit";
    constexpr const char* LOC_RANDOMINIT     = "localization/randominit";
    constexpr const char* LOC_START          = "localization/start";
    constexpr const char* LOC_STOP           = "localization/stop";
    constexpr const char* LOC_RESULT         = "localization/result";   // Publisher

    // ---- Control RPC (slamnav_control.fbs) ----
    constexpr const char* CTRL_GET_SF        = "control/getSafetyField";
    constexpr const char* CTRL_SET_SF        = "control/setSafetyField";
    constexpr const char* CTRL_GET_SF_FLAG   = "control/getSafetyFlag";
    constexpr const char* CTRL_SET_SF_FLAG   = "control/setSafetyFlag";
    constexpr const char* CTRL_GET_SAFETY_IO = "control/getSafetyIo";
    constexpr const char* CTRL_SET_SAFETY_IO = "control/setSafetyIo";
    constexpr const char* CTRL_DOCK          = "control/dock";
    constexpr const char* CTRL_CHARGE        = "control/chargeTrigger";
    constexpr const char* CTRL_GET_OBS_BOX   = "control/getObsBox";
    constexpr const char* CTRL_SET_OBS_BOX   = "control/setObsBox";
    constexpr const char* CTRL_LED           = "control/led";
    constexpr const char* CTRL_MOTOR         = "control/motor";
    constexpr const char* CTRL_JOG           = "control/jog";
    constexpr const char* CTRL_SENSOR        = "control/sensor";
    constexpr const char* CTRL_PATH          = "control/path";
    constexpr const char* CTRL_DETECT        = "control/detect";
    // constexpr const char* DOCK_STATE_CHANGE  = "control/dock/stateChange"; // 미사용

    // ---- Map RPC (slamnav_map.fbs) ----
    constexpr const char* MAP_GET_LIST       = "map/getList";
    constexpr const char* MAP_GET_CURRENT    = "map/getCurrent";
    constexpr const char* MAP_LOAD           = "map/load";
    constexpr const char* MAP_DELETE         = "map/delete";
    constexpr const char* MAP_GET_FILE       = "map/getFile";
    constexpr const char* MAP_GET_CLOUD      = "map/getCloud";
    constexpr const char* MAP_SET_CLOUD      = "map/setCloud";
    constexpr const char* MAP_GET_TOPOLOGY   = "map/getTopology";
    constexpr const char* MAP_SET_TOPOLOGY   = "map/setTopology";
    constexpr const char* MAP_MAPPING_START  = "map/mapping/start";
    constexpr const char* MAP_MAPPING_STOP   = "map/mapping/stop";
    constexpr const char* MAP_MAPPING_SAVE   = "map/mapping/save";
    constexpr const char* MAP_RESULT         = "map/result";            // Publisher

    // ---- Setting RPC (slamnav_setting.fbs) ----
    constexpr const char* SETTING_GET_SENSOR_INDEX = "setting/getSensorIndex";
    constexpr const char* SETTING_SET_SENSOR_INDEX = "setting/setSensorIndex";
    constexpr const char* SETTING_SET_SENSOR_ON    = "setting/setSensorOn";
    constexpr const char* SETTING_GET_SENSOR_OFF   = "setting/getSensorOff";
    constexpr const char* SETTING_SET_PDU          = "setting/setPduParam";
    constexpr const char* SETTING_GET_PDU          = "setting/getPduParam";
    constexpr const char* SETTING_GET_DRIVE        = "setting/getDriveParam";
    constexpr const char* SETTING_RESULT           = "setting/result";  // Publisher

    // ---- Software RPC (slamnav_update.fbs) ----
    constexpr const char* SW_UPDATE      = "software/update";
    constexpr const char* SW_GET_VERSION = "software/getVersion";
    constexpr const char* SW_RESULT      = "software/result";           // Publisher

    // ---- Multi RPC (slamnav_multi.fbs) ----
    constexpr const char* MULTI_PATH     = "multi/path";
    constexpr const char* MULTI_VOBS     = "multi/vobs";

    // ---- Pub/Sub 상태/스트리밍 (slamnav_status.fbs, slamnav_socket.fbs) ----
    constexpr const char* STATUS         = "status";            // Publisher (100ms)
    constexpr const char* MOVE_STATUS    = "moveStatus";        // Publisher (500ms)
    constexpr const char* LIDAR_2D       = "lidar2d";           // Publisher
    constexpr const char* LIDAR_3D       = "lidar3d";           // Publisher
    constexpr const char* MAPPING_CLOUD  = "mappingCloud";      // Publisher
    constexpr const char* GLOBAL_PATH    = "globalPath";        // Publisher
    constexpr const char* LOCAL_PATH     = "localPath";         // Publisher
}

class COMM_ZENOH : public QObject
{
    Q_OBJECT
    Q_DISABLE_COPY(COMM_ZENOH)

public:
    static COMM_ZENOH* instance(QObject* parent = nullptr);

    // ===== robotType 관리 =====
    void set_robot_type(const std::string& type);
    std::string get_robot_type() const;

    // ===== Zenoh Session 접근 (zenoh_command에서 사용) =====
    zenoh::Session& get_session();
    bool is_session_valid() const;

    // ===== Topic 헬퍼 =====
    std::string make_topic(const char* suffix) const;

    // ===== 모듈 주입 =====
    void set_config_module(CONFIG* _config);
    void set_logger_module(LOGGER* _logger);
    void set_mobile_module(MOBILE* _mobile);
    void set_lidar_2d_module(LIDAR_2D* _lidar_2d);
    void set_lidar_3d_module(LIDAR_3D* _lidar_3d);
    void set_cam_module(CAM* _cam);
    void set_unimap_module(UNIMAP* _unimap);
    void set_obsmap_module(OBSMAP* _obsmap);
    void set_autocontrol_module(AUTOCONTROL* _ctrl);
    void set_dockcontrol_module(DOCKCONTROL* _dctrl);
    void set_localization_module(LOCALIZATION* _loc);
    void set_mapping_module(MAPPING* _mapping);

    // ===== 모듈 접근자 (zenoh_command에서 사용) =====
    CONFIG* get_config() const { return config; }
    LOGGER* get_logger() const { return logger; }
    MOBILE* get_mobile() const { return mobile; }
    LIDAR_2D* get_lidar_2d() const { return lidar_2d; }
    LIDAR_3D* get_lidar_3d() const { return lidar_3d; }
    CAM* get_cam() const { return cam; }
    UNIMAP* get_unimap() const { return unimap; }
    OBSMAP* get_obsmap() const { return obsmap; }
    AUTOCONTROL* get_autocontrol() const { return ctrl; }
    DOCKCONTROL* get_dockcontrol() const { return dctrl; }
    LOCALIZATION* get_localization() const { return loc; }
    MAPPING* get_mapping() const { return mapping; }

    // ===== 스레드 생명주기 관리 =====
    void start_all_thread();
    void stop_all_thread();

    // 개별 스레드 시작
    void start_move_thread();
    void start_control_thread();
    void start_localization_thread();
    void start_map_thread();
    void start_setting_thread();
    void start_update_thread();
    void start_path_thread();
    void start_status_thread();
    void start_sensor_thread();

    // 개별 스레드 종료
    void stop_move_thread();
    void stop_control_thread();
    void stop_localization_thread();
    void stop_map_thread();
    void stop_setting_thread();
    void stop_update_thread();
    void stop_path_thread();
    void stop_status_thread();
    void stop_sensor_thread();

    // ===== 상태 플래그 =====
    bool get_is_connected() const;

    // ===== Running 플래그 접근자 (zenoh_command에서 사용) =====
    bool is_move_running() const { return is_move_running_.load(); }
    bool is_control_running() const { return is_control_running_.load(); }
    bool is_localization_running() const { return is_localization_running_.load(); }
    bool is_map_running() const { return is_map_running_.load(); }
    bool is_setting_running() const { return is_setting_running_.load(); }
    bool is_update_running() const { return is_update_running_.load(); }
    bool is_path_running() const { return is_path_running_.load(); }
    bool is_status_running() const { return is_status_running_.load(); }
    bool is_sensor_running() const { return is_sensor_running_.load(); }

    // ===== 경로 업데이트 플래그 (zenoh_command에서 사용) =====
    void set_global_path_update();
    void set_local_path_update();
    bool get_global_path_update();
    bool get_local_path_update();
    void clear_global_path_update();
    void clear_local_path_update();

Q_SIGNALS:
    void signal_map_build_start();
    void signal_map_build_stop();
    void signal_map_save(const QString& _map_name);

    void signal_auto_profile_move(DATA_MOVE msg);
    void signal_auto_move_stop();

    void signal_mobile_jog_update(const Eigen::Vector3d& val);

    void signal_ui_all_update();

    void signal_docking_start();
    void signal_undocking_start();
    void signal_docking_stop();

    void signal_config_request(DATA_PDU_UPDATE msg);

private:
    explicit COMM_ZENOH(QObject* parent = nullptr);
    ~COMM_ZENOH();

    // ===== Zenoh Session =====
    void open_session();
    void close_session();
    std::optional<zenoh::Session> session_;
    mutable std::shared_mutex session_mtx_;

    // ===== robotType =====
    std::string robot_type_;
    mutable std::shared_mutex robot_type_mtx_;

    // ===== 외부 모듈 포인터 =====
    CONFIG* config = nullptr;
    LOGGER* logger = nullptr;
    MOBILE* mobile = nullptr;
    LIDAR_2D* lidar_2d = nullptr;
    LIDAR_3D* lidar_3d = nullptr;
    CAM* cam = nullptr;
    UNIMAP* unimap = nullptr;
    OBSMAP* obsmap = nullptr;
    AUTOCONTROL* ctrl = nullptr;
    DOCKCONTROL* dctrl = nullptr;
    LOCALIZATION* loc = nullptr;
    MAPPING* mapping = nullptr;

    // ===== 연결 상태 =====
    std::atomic<bool> is_connected_{false};

    // ===== 경로 업데이트 플래그 =====
    std::atomic<bool> is_global_path_update_{false};
    std::atomic<bool> is_local_path_update_{false};

    // ===== 스레드 관련 =====
    // running 플래그
    std::atomic<bool> is_move_running_{false};
    std::atomic<bool> is_control_running_{false};
    std::atomic<bool> is_localization_running_{false};
    std::atomic<bool> is_map_running_{false};
    std::atomic<bool> is_setting_running_{false};
    std::atomic<bool> is_update_running_{false};
    std::atomic<bool> is_path_running_{false};
    std::atomic<bool> is_status_running_{false};
    std::atomic<bool> is_sensor_running_{false};

    // 스레드 객체
    std::unique_ptr<std::thread> move_thread_;
    std::unique_ptr<std::thread> control_thread_;
    std::unique_ptr<std::thread> localization_thread_;
    std::unique_ptr<std::thread> map_thread_;
    std::unique_ptr<std::thread> setting_thread_;
    std::unique_ptr<std::thread> update_thread_;
    std::unique_ptr<std::thread> path_thread_;
    std::unique_ptr<std::thread> status_thread_;
    std::unique_ptr<std::thread> sensor_thread_;

    // ===== 스레드 루프 함수 (zenoh_command/*.cpp에서 구현) =====
    void move_loop();
    void control_loop();
    void localization_loop();
    void map_loop();
    void setting_loop();
    void update_loop();
    void path_loop();
    void status_loop();
    void sensor_loop();
};

#endif // COMM_ZENOH_H
