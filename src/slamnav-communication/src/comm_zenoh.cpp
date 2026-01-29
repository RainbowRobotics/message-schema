#include "comm_zenoh.h"
#include "global_defines.h"

#include <iostream>
#include <QDebug>

namespace
{
    const char* MODULE_NAME = "COMM_ZENOH";
}

// =============================================================================
// Singleton Instance
// =============================================================================
COMM_ZENOH* COMM_ZENOH::instance(QObject* parent)
{
    static COMM_ZENOH* inst = nullptr;
    if (!inst && parent)
    {
        inst = new COMM_ZENOH(parent);
    }
    else if (inst && parent && inst->parent() == nullptr)
    {
        inst->setParent(parent);
    }
    return inst;
}

// =============================================================================
// 생성자 / 소멸자
// =============================================================================
COMM_ZENOH::COMM_ZENOH(QObject* parent)
    : QObject(parent)
{
    qDebug() << "[" << MODULE_NAME << "] Constructor called";
    open_session();
}

COMM_ZENOH::~COMM_ZENOH()
{
    qDebug() << "[" << MODULE_NAME << "] Destructor called";
    stop_all_thread();
    close_session();
}

// =============================================================================
// Zenoh Session 관리
// =============================================================================
void COMM_ZENOH::open_session()
{
    std::unique_lock<std::shared_mutex> lock(session_mtx_);
    try
    {
        zenoh::Config config = zenoh::Config::create_default();
        session_ = zenoh::Session::open(std::move(config));
        is_connected_ = true;
        qDebug() << "[" << MODULE_NAME << "] Zenoh session opened";
    }
    catch (const zenoh::ZException& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Failed to open Zenoh session:" << e.what();
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
        qDebug() << "[" << MODULE_NAME << "] Zenoh session closed";
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

// =============================================================================
// robotType 관리
// =============================================================================
void COMM_ZENOH::set_robot_type(const std::string& type)
{
    qDebug() << "[" << MODULE_NAME << "] set_robot_type called:" << QString::fromStdString(type);

    // 1. 모든 스레드 종료
    stop_all_thread();

    // 2. robotType 저장
    {
        std::unique_lock<std::shared_mutex> lock(robot_type_mtx_);
        robot_type_ = type;
    }

    qDebug() << "[" << MODULE_NAME << "] robotType set to:" << QString::fromStdString(type);

    // 3. 모든 스레드 재시작 (각 스레드에서 init 처리)
    start_all_thread();
}

std::string COMM_ZENOH::get_robot_type() const
{
    std::shared_lock<std::shared_mutex> lock(robot_type_mtx_);
    return robot_type_;
}

// =============================================================================
// Topic 헬퍼
// =============================================================================
std::string COMM_ZENOH::make_topic(const char* suffix) const
{
    std::shared_lock<std::shared_mutex> lock(robot_type_mtx_);
    if (robot_type_.empty())
    {
        return std::string(suffix);
    }
    return robot_type_ + "/" + suffix;
}

// =============================================================================
// 모듈 주입
// =============================================================================
void COMM_ZENOH::set_config_module(CONFIG* _config)
{
    if (_config)
    {
        config = _config;
        qDebug() << "[" << MODULE_NAME << "] CONFIG module set";
    }
}

void COMM_ZENOH::set_logger_module(LOGGER* _logger)
{
    if (_logger)
    {
        logger = _logger;
        qDebug() << "[" << MODULE_NAME << "] LOGGER module set";
    }
}

void COMM_ZENOH::set_mobile_module(MOBILE* _mobile)
{
    if (_mobile)
    {
        mobile = _mobile;
        qDebug() << "[" << MODULE_NAME << "] MOBILE module set";
    }
}

void COMM_ZENOH::set_lidar_2d_module(LIDAR_2D* _lidar_2d)
{
    if (_lidar_2d)
    {
        lidar_2d = _lidar_2d;
        qDebug() << "[" << MODULE_NAME << "] LIDAR_2D module set";
    }
}

void COMM_ZENOH::set_lidar_3d_module(LIDAR_3D* _lidar_3d)
{
    if (_lidar_3d)
    {
        lidar_3d = _lidar_3d;
        qDebug() << "[" << MODULE_NAME << "] LIDAR_3D module set";
    }
}

void COMM_ZENOH::set_cam_module(CAM* _cam)
{
    if (_cam)
    {
        cam = _cam;
        qDebug() << "[" << MODULE_NAME << "] CAM module set";
    }
}

void COMM_ZENOH::set_unimap_module(UNIMAP* _unimap)
{
    if (_unimap)
    {
        unimap = _unimap;
        qDebug() << "[" << MODULE_NAME << "] UNIMAP module set";
    }
}

void COMM_ZENOH::set_obsmap_module(OBSMAP* _obsmap)
{
    if (_obsmap)
    {
        obsmap = _obsmap;
        qDebug() << "[" << MODULE_NAME << "] OBSMAP module set";
    }
}

void COMM_ZENOH::set_autocontrol_module(AUTOCONTROL* _ctrl)
{
    if (_ctrl)
    {
        ctrl = _ctrl;
        qDebug() << "[" << MODULE_NAME << "] AUTOCONTROL module set";
    }
}

void COMM_ZENOH::set_dockcontrol_module(DOCKCONTROL* _dctrl)
{
    if (_dctrl)
    {
        dctrl = _dctrl;
        qDebug() << "[" << MODULE_NAME << "] DOCKCONTROL module set";
    }
}

void COMM_ZENOH::set_localization_module(LOCALIZATION* _loc)
{
    if (_loc)
    {
        loc = _loc;
        qDebug() << "[" << MODULE_NAME << "] LOCALIZATION module set";
    }
}

void COMM_ZENOH::set_mapping_module(MAPPING* _mapping)
{
    if (_mapping)
    {
        mapping = _mapping;
        qDebug() << "[" << MODULE_NAME << "] MAPPING module set";
    }
}

// =============================================================================
// 경로 업데이트 플래그
// =============================================================================
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

// =============================================================================
// 스레드 생명주기 관리 - 전체
// =============================================================================
void COMM_ZENOH::start_all_thread()
{
    qDebug() << "[" << MODULE_NAME << "] Starting all threads...";

    start_move_thread();
    start_control_thread();
    start_localization_thread();
    start_map_thread();
    start_setting_thread();
    start_update_thread();
    start_path_thread();
    start_status_thread();
    start_sensor_thread();

    qDebug() << "[" << MODULE_NAME << "] All threads started";
}

void COMM_ZENOH::stop_all_thread()
{
    qDebug() << "[" << MODULE_NAME << "] Stopping all threads...";

    stop_move_thread();
    stop_control_thread();
    stop_localization_thread();
    stop_map_thread();
    stop_setting_thread();
    stop_update_thread();
    stop_path_thread();
    stop_status_thread();
    stop_sensor_thread();

    qDebug() << "[" << MODULE_NAME << "] All threads stopped";
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Move)
// =============================================================================
void COMM_ZENOH::start_move_thread()
{
    if (is_move_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Move thread already running";
        return;
    }

    is_move_running_ = true;
    move_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::move_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Move thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Move thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Control)
// =============================================================================
void COMM_ZENOH::start_control_thread()
{
    if (is_control_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Control thread already running";
        return;
    }

    is_control_running_ = true;
    control_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::control_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Control thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Control thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Localization)
// =============================================================================
void COMM_ZENOH::start_localization_thread()
{
    if (is_localization_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Localization thread already running";
        return;
    }

    is_localization_running_ = true;
    localization_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::localization_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Localization thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Localization thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Map)
// =============================================================================
void COMM_ZENOH::start_map_thread()
{
    if (is_map_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Map thread already running";
        return;
    }

    is_map_running_ = true;
    map_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::map_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Map thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Map thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Setting)
// =============================================================================
void COMM_ZENOH::start_setting_thread()
{
    if (is_setting_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Setting thread already running";
        return;
    }

    is_setting_running_ = true;
    setting_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::setting_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Setting thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Setting thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Update)
// =============================================================================
void COMM_ZENOH::start_update_thread()
{
    if (is_update_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Update thread already running";
        return;
    }

    is_update_running_ = true;
    update_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::update_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Update thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Update thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Path)
// =============================================================================
void COMM_ZENOH::start_path_thread()
{
    if (is_path_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Path thread already running";
        return;
    }

    is_path_running_ = true;
    path_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::path_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Path thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Path thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Status)
// =============================================================================
void COMM_ZENOH::start_status_thread()
{
    if (is_status_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Status thread already running";
        return;
    }

    is_status_running_ = true;
    status_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::status_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Status thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Status thread stopped";
    }
}

// =============================================================================
// 스레드 생명주기 관리 - 개별 (Sensor)
// =============================================================================
void COMM_ZENOH::start_sensor_thread()
{
    if (is_sensor_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] Sensor thread already running";
        return;
    }

    is_sensor_running_ = true;
    sensor_thread_ = std::make_unique<std::thread>(&COMM_ZENOH::sensor_loop, this);
    qDebug() << "[" << MODULE_NAME << "] Sensor thread started";
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
        qDebug() << "[" << MODULE_NAME << "] Sensor thread stopped";
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
    qDebug() << "[" << MODULE_NAME << "] setting_loop started (placeholder)";

    while (is_setting_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_setting.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    qDebug() << "[" << MODULE_NAME << "] setting_loop ended";
}

void COMM_ZENOH::update_loop()
{
    qDebug() << "[" << MODULE_NAME << "] update_loop started (placeholder)";

    while (is_update_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_update.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    qDebug() << "[" << MODULE_NAME << "] update_loop ended";
}

void COMM_ZENOH::path_loop()
{
    qDebug() << "[" << MODULE_NAME << "] path_loop started (placeholder)";

    while (is_path_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_path.cpp에서 구현
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    qDebug() << "[" << MODULE_NAME << "] path_loop ended";
}

// status_loop()는 zenoh_command/comm_zenoh_status.cpp에서 구현

void COMM_ZENOH::sensor_loop()
{
    qDebug() << "[" << MODULE_NAME << "] sensor_loop started (placeholder)";

    while (is_sensor_running_.load())
    {
        // TODO: zenoh_command/comm_zenoh_sensor.cpp에서 구현
        // lidar2d, lidar3d, mappingCloud 발행
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    qDebug() << "[" << MODULE_NAME << "] sensor_loop ended";
}
