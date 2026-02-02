#include "comm_zenoh.h"
#include "global_defines.h"

namespace
{
    constexpr const char* MODULE_NAME = "ZENOH";
}

COMM_ZENOH* COMM_ZENOH::instance()
{
    static COMM_ZENOH* inst = nullptr;
    if (!inst)
    {
        inst = new COMM_ZENOH();
    }
    return inst;
}

COMM_ZENOH::COMM_ZENOH()
{
    open_session();
}

COMM_ZENOH::~COMM_ZENOH()
{
    stop_all_thread();
    close_session();
}

void COMM_ZENOH::open_session()
{
    std::unique_lock<std::shared_mutex> lock(session_mtx_);
    try
    {
        zenoh::Config config = zenoh::Config::create_default();
        session_ = zenoh::Session::open(std::move(config));
        is_connected_ = true;
        log_info("Zenoh session opened");
    }
    catch (const zenoh::ZException& e)
    {
        log_error("Failed to open Zenoh session: {}", e.what());
        is_connected_ = false;
    }
}

void COMM_ZENOH::close_session()
{
    std::unique_lock<std::shared_mutex> lock(session_mtx_);
    if (session_.has_value())
    {
        session_.reset();
        is_connected_ = false;
        log_info("Zenoh session closed");
    }
}

zenoh::Session& COMM_ZENOH::get_session()
{
    std::shared_lock<std::shared_mutex> lock(session_mtx_);
    if (!session_.has_value())
    {
        throw std::runtime_error("Zenoh session is not initialized");
    }
    return session_.value();
}

bool COMM_ZENOH::is_session_valid() const
{
    std::shared_lock<std::shared_mutex> lock(session_mtx_);
    return session_.has_value();
}

bool COMM_ZENOH::get_is_connected() const
{
    return is_connected_.load();
}

// get robotType
void COMM_ZENOH::set_robot_type(const std::string& type)
{
    log_info("robotType set to: {}", type);
    stop_all_thread();
    {
        std::unique_lock<std::shared_mutex> lock(robot_type_mtx_);
        robot_type_ = type;
    }
    start_all_thread();
}

std::string COMM_ZENOH::get_robot_type() const
{
    std::shared_lock<std::shared_mutex> lock(robot_type_mtx_);
    return robot_type_;
}

std::string COMM_ZENOH::make_topic(const char* suffix) const
{
    std::shared_lock<std::shared_mutex> lock(robot_type_mtx_);
    if (robot_type_.empty())
    {
        return std::string(suffix);
    }
    return robot_type_ + "/" + suffix;
}


void COMM_ZENOH::set_global_path_update()
{
    is_global_path_update_ = true;
}

void COMM_ZENOH::set_local_path_update()
{
    is_local_path_update_ = true;
}

bool COMM_ZENOH::get_global_path_update()
{
    return is_global_path_update_.load();
}

bool COMM_ZENOH::get_local_path_update()
{
    return is_local_path_update_.load();
}

void COMM_ZENOH::clear_global_path_update()
{
    is_global_path_update_ = false;
}

void COMM_ZENOH::clear_local_path_update()
{
    is_local_path_update_ = false;
}

void COMM_ZENOH::start_all_thread()
{
    start_move_thread();
    start_control_thread();
    start_localization_thread();
    start_map_thread();
    start_setting_thread();
    start_update_thread();
    start_path_thread();
    start_status_thread();
    start_move_status_thread();
    start_sensor_thread();

    log_info("All threads started");
}

void COMM_ZENOH::stop_all_thread()
{
    stop_move_thread();
    stop_control_thread();
    stop_localization_thread();
    stop_map_thread();
    stop_setting_thread();
    stop_update_thread();
    stop_path_thread();
    stop_status_thread();
    stop_move_status_thread();
    stop_sensor_thread();

    log_info("All threads stopped");
}

// move thread
void COMM_ZENOH::start_move_thread()
{
    if (is_move_running_.load())
    {
        log_warn("Move thread already running");
        return;
    }

    is_move_running_ = true;
    move_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::move_loop, this);
    log_info("Move thread started");
}
void COMM_ZENOH::stop_move_thread()
{
    if (!is_move_running_.load())
    {
        return;
    }

    is_move_running_ = false;
    if (move_thread_ && move_thread_->joinable())
    {
        move_thread_->join();
        move_thread_.reset();
        log_info("Move thread stopped");
    }
}

// control thread
void COMM_ZENOH::start_control_thread()
{
    if (is_control_running_.load())
    {
        log_warn("Control thread already running");
        return;
    }

    is_control_running_ = true;
    control_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::control_loop, this);
    log_info("Control thread started");
}
void COMM_ZENOH::stop_control_thread()
{
    if (!is_control_running_.load())
    {
        return;
    }

    is_control_running_ = false;
    if (control_thread_ && control_thread_->joinable())
    {
        control_thread_->join();
        control_thread_.reset();
        log_info("Control thread stopped");
    }
}

// localization thread
void COMM_ZENOH::start_localization_thread()
{
    if (is_localization_running_.load())
    {
        log_warn("Localization thread already running");
        return;
    }

    is_localization_running_ = true;
    localization_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::localization_loop, this);
    log_info("Localization thread started");
}
void COMM_ZENOH::stop_localization_thread()
{
    if (!is_localization_running_.load())
    {
        return;
    }

    is_localization_running_ = false;
    if (localization_thread_ && localization_thread_->joinable())
    {
        localization_thread_->join();
        localization_thread_.reset();
        log_info("Localization thread stopped");
    }
}

// map theead
void COMM_ZENOH::start_map_thread()
{
    if (is_map_running_.load())
    {
        log_warn("Map thread already running");
        return;
    }

    is_map_running_ = true;
    map_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::map_loop, this);
    log_info("Map thread started");
}
void COMM_ZENOH::stop_map_thread()
{
    if (!is_map_running_.load())
    {
        return;
    }

    is_map_running_ = false;
    if (map_thread_ && map_thread_->joinable())
    {
        map_thread_->join();
        map_thread_.reset();
        log_info("Map thread stopped");
    }
}

// setting thread
void COMM_ZENOH::start_setting_thread()
{
    if (is_setting_running_.load())
    {
        log_warn("Setting thread already running");
        return;
    }

    is_setting_running_ = true;
    setting_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::setting_loop, this);
    log_info("Setting thread started");
}
void COMM_ZENOH::stop_setting_thread()
{
    if (!is_setting_running_.load())
    {
        return;
    }

    is_setting_running_ = false;
    if (setting_thread_ && setting_thread_->joinable())
    {
        setting_thread_->join();
        setting_thread_.reset();
        log_info("Setting thread stopped");
    }
}

// update thread
void COMM_ZENOH::start_update_thread()
{
    if (is_update_running_.load())
    {
        log_warn("Update thread already running");
        return;
    }

    is_update_running_ = true;
    update_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::update_loop, this);
    log_info("Update thread started");
}
void COMM_ZENOH::stop_update_thread()
{
    if (!is_update_running_.load())
    {
        return;
    }

    is_update_running_ = false;
    if (update_thread_ && update_thread_->joinable())
    {
        update_thread_->join();
        update_thread_.reset();
        log_info("Update thread stopped");
    }
}

// path thread
void COMM_ZENOH::start_path_thread()
{
    if (is_path_running_.load())
    {
        log_warn("Path thread already running");
        return;
    }

    is_path_running_ = true;
    path_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::path_loop, this);
    log_info("Path thread started");
}
void COMM_ZENOH::stop_path_thread()
{
    if (!is_path_running_.load())
    {
        return;
    }

    is_path_running_ = false;
    if (path_thread_ && path_thread_->joinable())
    {
        path_thread_->join();
        path_thread_.reset();
        log_info("Path thread stopped");
    }
}

// status thread
void COMM_ZENOH::start_status_thread()
{
    if (is_status_running_.load())
    {
        log_warn("Status thread already running");
        return;
    }

    is_status_running_ = true;
    status_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::status_loop, this);
    log_info("Status thread started");
}
void COMM_ZENOH::stop_status_thread()
{
    if (!is_status_running_.load())
    {
        return;
    }

    is_status_running_ = false;
    if (status_thread_ && status_thread_->joinable())
    {
        status_thread_->join();
        status_thread_.reset();
        log_info("Status thread stopped");
    }
}

// move_status thread
void COMM_ZENOH::start_move_status_thread()
{
    if (is_move_status_running_.load())
    {
        log_warn("MoveStatus thread already running");
        return;
    }

    is_move_status_running_ = true;
    move_status_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::move_status_loop, this);
    log_info("MoveStatus thread started");
}
void COMM_ZENOH::stop_move_status_thread()
{
    if (!is_move_status_running_.load())
    {
        return;
    }

    is_move_status_running_ = false;
    if (move_status_thread_ && move_status_thread_->joinable())
    {
        move_status_thread_->join();
        move_status_thread_.reset();
        log_info("MoveStatus thread stopped");
    }
}

// sensor thread
void COMM_ZENOH::start_sensor_thread()
{
    if (is_sensor_running_.load())
    {
        log_warn("Sensor thread already running");
        return;
    }

    is_sensor_running_ = true;
    sensor_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::sensor_loop, this);
    log_info("Sensor thread started");
}
void COMM_ZENOH::stop_sensor_thread()
{
    if (!is_sensor_running_.load())
    {
        return;
    }

    is_sensor_running_ = false;
    if (sensor_thread_ && sensor_thread_->joinable())
    {
        sensor_thread_->join();
        sensor_thread_.reset();
        log_info("Sensor thread stopped");
    }
}

// =============================================================================
// 스레드 루프 함수 - 빈 구현 (zenoh_command/*.cpp에서 구현 예정)
// =============================================================================

// move_loop()는 zenoh_command/comm_zenoh_move.cpp에서 구현

// control_loop()는 zenoh_command/comm_zenoh_control.cpp에서 구현

// localization_loop()는 zenoh_command/comm_zenoh_localization.cpp에서 구현

// map_loop()는 zenoh_command/comm_zenoh_map.cpp에서 구현

void COMM_ZENOH::setting_loop()
{
    log_info("setting_loop started (placeholder)");

    while (is_setting_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_setting.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log_info("setting_loop ended");
}

void COMM_ZENOH::update_loop()
{
    log_info("update_loop started (placeholder)");

    while (is_update_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_update.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log_info("update_loop ended");
}

void COMM_ZENOH::path_loop()
{
    log_info("path_loop started (placeholder)");

    while (is_path_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_path.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log_info("path_loop ended");
}

// status_loop()는 zenoh_command/comm_zenoh_status.cpp에서 구현

// move_status_loop()는 zenoh_command/comm_zenoh_moveStatus.cpp에서 구현

void COMM_ZENOH::sensor_loop()
{
    log_info("sensor_loop started (placeholder)");

    while (is_sensor_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_sensor.cpp에서 구현
        // lidar2d, lidar3d, mappingCloud 발행
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    log_info("sensor_loop ended");
}

// callback function
void COMM_ZENOH::set_jog_callback(ZenohCallback::JogUpdate cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    jog_callback_ = std::move(cb);
}

void COMM_ZENOH::set_move_stop_callback(ZenohCallback::MoveStop cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    move_stop_callback_ = std::move(cb);
}

void COMM_ZENOH::set_map_build_start_callback(ZenohCallback::MapBuildStart cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    map_build_start_callback_ = std::move(cb);
}

void COMM_ZENOH::set_map_build_stop_callback(ZenohCallback::MapBuildStop cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    map_build_stop_callback_ = std::move(cb);
}

void COMM_ZENOH::set_map_save_callback(ZenohCallback::MapSave cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    map_save_callback_ = std::move(cb);
}

void COMM_ZENOH::set_docking_start_callback(ZenohCallback::DockingStart cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    docking_start_callback_ = std::move(cb);
}

void COMM_ZENOH::set_undocking_start_callback(ZenohCallback::UndockingStart cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    undocking_start_callback_ = std::move(cb);
}

void COMM_ZENOH::set_docking_stop_callback(ZenohCallback::DockingStop cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    docking_stop_callback_ = std::move(cb);
}

void COMM_ZENOH::set_ui_all_update_callback(ZenohCallback::UiAllUpdate cb)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    ui_all_update_callback_ = std::move(cb);
}

// invoke callback
void COMM_ZENOH::invoke_jog_callback(const Eigen::Vector3d& vel)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (jog_callback_)
    {
        jog_callback_(vel);
    }
}

void COMM_ZENOH::invoke_move_stop_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (move_stop_callback_)
    {
        move_stop_callback_();
    }
}

void COMM_ZENOH::invoke_map_build_start_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (map_build_start_callback_)
    {
        map_build_start_callback_();
    }
}

void COMM_ZENOH::invoke_map_build_stop_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (map_build_stop_callback_)
    {
        map_build_stop_callback_();
    }
}

void COMM_ZENOH::invoke_map_save_callback(const std::string& map_name)
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (map_save_callback_)
    {
        map_save_callback_(map_name);
    }
}

void COMM_ZENOH::invoke_docking_start_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (docking_start_callback_)
    {
        docking_start_callback_();
    }
}

void COMM_ZENOH::invoke_undocking_start_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (undocking_start_callback_)
    {
        undocking_start_callback_();
    }
}

void COMM_ZENOH::invoke_docking_stop_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (docking_stop_callback_)
    {
        docking_stop_callback_();
    }
}

void COMM_ZENOH::invoke_ui_all_update_callback()
{
    std::lock_guard<std::mutex> lock(callback_mtx_);
    if (ui_all_update_callback_)
    {
        ui_all_update_callback_();
    }
}

// set modules
void COMM_ZENOH::set_config_module(CONFIG* _config)
{
    if (_config)
    {
        config = _config;
        log_debug("CONFIG module set");
    }
}

void COMM_ZENOH::set_logger_module(LOGGER* _logger)
{
    if (_logger)
    {
        logger = _logger;
        log_debug("LOGGER module set");
    }
}

void COMM_ZENOH::set_mobile_module(MOBILE* _mobile)
{
    if (_mobile)
    {
        mobile = _mobile;
        log_debug("MOBILE module set");
    }
}

void COMM_ZENOH::set_lidar_2d_module(LIDAR_2D* _lidar_2d)
{
    if (_lidar_2d)
    {
        lidar_2d = _lidar_2d;
        log_debug("LIDAR_2D module set");
    }
}

void COMM_ZENOH::set_lidar_3d_module(LIDAR_3D* _lidar_3d)
{
    if (_lidar_3d)
    {
        lidar_3d = _lidar_3d;
        log_debug("LIDAR_3D module set");
    }
}

void COMM_ZENOH::set_cam_module(CAM* _cam)
{
    if (_cam)
    {
        cam = _cam;
        log_debug("CAM module set");
    }
}

void COMM_ZENOH::set_unimap_module(UNIMAP* _unimap)
{
    if (_unimap)
    {
        unimap = _unimap;
        log_debug("UNIMAP module set");
    }
}

void COMM_ZENOH::set_obsmap_module(OBSMAP* _obsmap)
{
    if (_obsmap)
    {
        obsmap = _obsmap;
        log_debug("OBSMAP module set");
    }
}

void COMM_ZENOH::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if (_ctrl)
    {
        ctrl = _ctrl;
        log_debug("AUTOCONTROL module set");
    }
}

void COMM_ZENOH::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if (_dctrl)
    {
        dctrl = _dctrl;
        log_debug("DOCKCONTROL module set");
    }
}

void COMM_ZENOH::set_localization_module(LOCALIZATION* _loc)
{
    if (_loc)
    {
        loc = _loc;
        log_debug("LOCALIZATION module set");
    }
}

void COMM_ZENOH::set_mapping_module(MAPPING* _mapping)
{
    if (_mapping)
    {
        mapping = _mapping;
        log_debug("MAPPING module set");
    }
}