#ifndef COMM_ZENOH_H
#define COMM_ZENOH_H

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

#include <zenoh.hxx>
#include <flatbuffers/flatbuffers.h>

#include <Eigen/Core>
#include <queue>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <optional>
#include <functional>

struct COMM_ZENOH_INFO
{
    static constexpr double status_send_time = 0.1;         // 100ms
    static constexpr double move_status_send_time = 0.5;    // 500ms
    static constexpr double mapping_cloud_send_time = 0.5;  // 500ms
};

namespace ZenohCallback
{
    // Move 관련 콜백
    using JogUpdate = std::function<void(const Eigen::Vector3d& vel)>;
    using MoveStop = std::function<void()>;

    // Map 관련 콜백
    using MapBuildStart = std::function<void()>;
    using MapBuildStop = std::function<void()>;
    using MapSave = std::function<void(const std::string& map_name)>;

    // Docking 관련 콜백
    using DockingStart = std::function<void()>;
    using UndockingStart = std::function<void()>;
    using DockingStop = std::function<void()>;

    // UI 관련 콜백
    using UiAllUpdate = std::function<void()>;
}

// NOTE: Topic 정의는 각 zenoh_command/*.cpp 내부에서 선언
// 참고: topics.md, ARCHITECTURE.md

class COMM_ZENOH
{
public:
    COMM_ZENOH(const COMM_ZENOH&) = delete;
    COMM_ZENOH& operator=(const COMM_ZENOH&) = delete;
    COMM_ZENOH(COMM_ZENOH&&) = delete;
    COMM_ZENOH& operator=(COMM_ZENOH&&) = delete;

    static COMM_ZENOH* instance();

    // set robotType
    void set_robot_type(const std::string& type);
    std::string get_robot_type() const;

    zenoh::Session& get_session();
    bool is_session_valid() const;
    std::string make_topic(const char* suffix) const;

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

    void start_all_thread();
    void stop_all_thread();
    bool get_is_connected() const;

    // move thread
    void start_move_thread();
    void stop_move_thread();
    bool is_move_running() const { return is_move_running_.load(); }

    // control thread
    void start_control_thread();
    void stop_control_thread();
    bool is_control_running() const { return is_control_running_.load(); }
    
    // localization thread
    void start_localization_thread();
    void stop_localization_thread();
    bool is_localization_running() const { return is_localization_running_.load(); }

    // map thread
    void start_map_thread();
    void stop_map_thread();
    bool is_map_running() const { return is_map_running_.load(); }

    // setting thread
    void start_setting_thread();
    void stop_setting_thread();
    bool is_setting_running() const { return is_setting_running_.load(); }

    // update thread
    void start_update_thread();
    void stop_update_thread();
    bool is_update_running() const { return is_update_running_.load(); }
    
    // path thread
    void start_path_thread();
    void stop_path_thread();
    bool is_path_running() const { return is_path_running_.load(); }
    
    // status thread
    void start_status_thread();
    void stop_status_thread();
    bool is_status_running() const { return is_status_running_.load(); }

    // move_status thread
    void start_move_status_thread();
    void stop_move_status_thread();
    bool is_move_status_running() const { return is_move_status_running_.load(); }

    // sensor thread
    void start_sensor_thread();
    void stop_sensor_thread();
    bool is_sensor_running() const { return is_sensor_running_.load(); }
    
    // path update flag
    void set_global_path_update();
    void set_local_path_update();
    bool get_global_path_update();
    bool get_local_path_update();
    void clear_global_path_update();
    void clear_local_path_update();

    // call back
    void set_jog_callback(ZenohCallback::JogUpdate cb);
    void set_move_stop_callback(ZenohCallback::MoveStop cb);
    void set_map_build_start_callback(ZenohCallback::MapBuildStart cb);
    void set_map_build_stop_callback(ZenohCallback::MapBuildStop cb);
    void set_map_save_callback(ZenohCallback::MapSave cb);
    void set_docking_start_callback(ZenohCallback::DockingStart cb);
    void set_undocking_start_callback(ZenohCallback::UndockingStart cb);
    void set_docking_stop_callback(ZenohCallback::DockingStop cb);
    void set_ui_all_update_callback(ZenohCallback::UiAllUpdate cb);

    // ===== 콜백 호출 (zenoh_command에서 사용) =====
    void invoke_jog_callback(const Eigen::Vector3d& vel);
    void invoke_move_stop_callback();
    void invoke_map_build_start_callback();
    void invoke_map_build_stop_callback();
    void invoke_map_save_callback(const std::string& map_name);
    void invoke_docking_start_callback();
    void invoke_undocking_start_callback();
    void invoke_docking_stop_callback();
    void invoke_ui_all_update_callback();

    // qt dependency
// Q_SIGNALS:
//     void signal_map_build_start();
//     void signal_map_build_stop();
//     void signal_map_save(const QString& _map_name);
//     void signal_auto_profile_move(DATA_MOVE msg);
//     void signal_auto_move_stop();
//     void signal_mobile_jog_update(const Eigen::Vector3d& val);
//     void signal_ui_all_update();
//     void signal_docking_start();
//     void signal_undocking_start();
//     void signal_docking_stop();
//     void signal_config_request(DATA_PDU_UPDATE msg);

private:
    COMM_ZENOH();
    ~COMM_ZENOH();

    // zenoh session
    void open_session();
    void close_session();
    std::optional<zenoh::Session> session_;
    mutable std::shared_mutex session_mtx_;

    // robotType
    std::string robot_type_;
    mutable std::shared_mutex robot_type_mtx_;

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

    // connection flag
    std::atomic<bool> is_connected_{false};

    // path update flags
    std::atomic<bool> is_global_path_update_{false};
    std::atomic<bool> is_local_path_update_{false};

    // thread flags
    std::atomic<bool> is_move_running_{false};
    std::atomic<bool> is_control_running_{false};
    std::atomic<bool> is_localization_running_{false};
    std::atomic<bool> is_map_running_{false};
    std::atomic<bool> is_setting_running_{false};
    std::atomic<bool> is_update_running_{false};
    std::atomic<bool> is_path_running_{false};
    std::atomic<bool> is_status_running_{false};
    std::atomic<bool> is_move_status_running_{false};
    std::atomic<bool> is_sensor_running_{false};

    // threads
    std::unique_ptr<std::thread> move_thread_;
    std::unique_ptr<std::thread> control_thread_;
    std::unique_ptr<std::thread> localization_thread_;
    std::unique_ptr<std::thread> map_thread_;
    std::unique_ptr<std::thread> setting_thread_;
    std::unique_ptr<std::thread> update_thread_;
    std::unique_ptr<std::thread> path_thread_;
    std::unique_ptr<std::thread> status_thread_;
    std::unique_ptr<std::thread> move_status_thread_;
    std::unique_ptr<std::thread> sensor_thread_;

    // thread function
    void move_loop();
    void control_loop();
    void localization_loop();
    void map_loop();
    void setting_loop();
    void update_loop();
    void path_loop();
    void status_loop();
    void move_status_loop();
    void sensor_loop();

    // callback
    ZenohCallback::JogUpdate jog_callback_;
    ZenohCallback::MoveStop move_stop_callback_;
    ZenohCallback::MapBuildStart map_build_start_callback_;
    ZenohCallback::MapBuildStop map_build_stop_callback_;
    ZenohCallback::MapSave map_save_callback_;
    ZenohCallback::DockingStart docking_start_callback_;
    ZenohCallback::UndockingStart undocking_start_callback_;
    ZenohCallback::DockingStop docking_stop_callback_;
    ZenohCallback::UiAllUpdate ui_all_update_callback_;
    std::mutex callback_mtx_;
};

#endif // COMM_ZENOH_H
