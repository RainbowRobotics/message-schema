/**
 * @file comm_zenoh_status.cpp
 * @brief Status 및 MoveStatus Publisher 구현
 *
 * Publisher Topics:
 *   - {robotType}/status      : 로봇 상태 정보 (100ms 주기)
 *   - {robotType}/moveStatus  : 이동 상태 정보 (500ms 주기)
 *
 * 스키마: slamnav_status.fbs
 */

#include "comm_zenoh.h"
#include "global_defines.h"

#include <QDebug>
#include <chrono>
#include <cmath>

// FlatBuffers generated headers
#include "flatbuffer/generated/slamnav_status_generated.h"

namespace
{
    const char* MODULE_NAME = "ZENOH_STATUS";

    // Radian to Degree 변환 상수
    constexpr double R2D = 180.0 / M_PI;

    // 발행 주기 (밀리초)
    constexpr int STATUS_INTERVAL_MS = 100;       // 100ms
    constexpr int MOVE_STATUS_INTERVAL_MS = 500;  // 500ms
}

// =============================================================================
// Helper Functions
// =============================================================================

namespace
{

/**
 * @brief 충전 상태를 문자열로 변환
 */
std::string get_charge_state_string(int charge_state, const std::string& robot_model)
{
    if (robot_model == "D400" || robot_model == "MECANUM")
    {
        switch (charge_state)
        {
            case CHARGE_STATE_IDLE:                return "none";
            case CHARGE_STATE_TRIG_TO_CHARGE:      return "ready";
            case CHARGE_STATE_BATTERY_ON:          return "battery_on";
            case CHARGE_STATE_CHARGING:            return "charging";
            case CHARGE_STATE_TRIG_TO_STOP_CHARGE: return "finish";
            case CHARGE_STATE_FAIL:                return "fail";
            default:                               return "none";
        }
    }
    else if (robot_model == "S100")
    {
        return (charge_state == 1) ? "charging" : "none";
    }
    return "none";
}

/**
 * @brief 맵 로드 상태를 문자열로 변환
 */
std::string get_map_status_string(int is_loaded)
{
    switch (is_loaded)
    {
        case MAP_NOT_LOADED: return "none";
        case MAP_LOADING:    return "loading";
        case MAP_LOADED:     return "loaded";
        default:             return "none";
    }
}

/**
 * @brief 자동 이동 상태를 문자열로 변환
 */
std::string get_auto_move_state_string(AUTOCONTROL* ctrl, MOBILE* mobile, LOCALIZATION* loc)
{
    if (!ctrl || !mobile || !loc)
    {
        return "stop";
    }

    // PDU 상태 또는 multi interlock 확인
    if (mobile->get_cur_pdu_state() != "good" || ctrl->get_multi_inter_lock())
    {
        return "not ready";
    }

    // Localization 상태 확인
    if (loc->get_cur_loc_state() != "good")
    {
        return "not ready";
    }

    // 이동 상태 확인
    if (ctrl->get_is_pause())
    {
        return "pause";
    }
    else if (ctrl->get_is_moving())
    {
        return "move";
    }

    return "stop";
}

/**
 * @brief 도킹 이동 상태를 문자열로 변환
 */
std::string get_dock_move_state_string(DOCKCONTROL* dctrl)
{
    if (!dctrl)
    {
        return "stop";
    }

    if (dctrl->get_dock_state())
    {
        return "docked";
    }
    else if (dctrl->get_is_docking())
    {
        return "docking";
    }
    else if (dctrl->get_is_undocking())
    {
        return "undocking";
    }

    return "stop";
}

} // anonymous namespace

// =============================================================================
// Status Publisher
// =============================================================================

/**
 * @brief Status 메시지 생성 및 발행 (100ms 주기)
 *
 * Topic: {robotType}/status
 * 내용: IMU, Motor, Power, RobotState, SafetyIO, Setting, Map 정보
 */
static void publish_status(COMM_ZENOH* zenoh)
{
    if (!zenoh->is_session_valid())
    {
        return;
    }

    // 필수 모듈 확인
    MOBILE* mobile = zenoh->get_mobile();
    LOCALIZATION* loc = zenoh->get_localization();
    DOCKCONTROL* dctrl = zenoh->get_dockcontrol();
    CONFIG* config = zenoh->get_config();
    UNIMAP* unimap = zenoh->get_unimap();

    if (!mobile || !loc || !dctrl || !config || !unimap)
    {
        return;
    }

    try
    {
        flatbuffers::FlatBufferBuilder fbb(2048);

        // 모듈에서 데이터 수집
        MOBILE_STATUS ms = mobile->get_status();
        Eigen::Vector3d imu_data = mobile->get_imu();
        Eigen::Vector2d ieir = loc->get_cur_ieir();
        QString cur_loc_state = loc->get_cur_loc_state();
        QString robot_model = config->get_robot_model();
        bool is_dock = dctrl->get_dock_state();

        // 1. StatusCondition (struct)
        SLAMNAV::StatusCondition condition(
            static_cast<float>(ieir[0]),  // inlier_error
            static_cast<float>(ieir[1]),  // inlier_ratio
            static_cast<float>(ieir[0]),  // mapping_error
            static_cast<float>(ieir[1])   // mapping_ratio
        );

        // 2. StatusImu (struct)
        SLAMNAV::StatusImu status_imu(
            static_cast<float>(ms.imu_acc_x),
            static_cast<float>(ms.imu_acc_y),
            static_cast<float>(ms.imu_acc_z),
            static_cast<float>(ms.imu_gyr_x * R2D),
            static_cast<float>(ms.imu_gyr_y * R2D),
            static_cast<float>(ms.imu_gyr_z * R2D),
            static_cast<float>(imu_data[0] * R2D),
            static_cast<float>(imu_data[1] * R2D),
            static_cast<float>(imu_data[2] * R2D)
        );

        // 3. StatusMotor (struct) - motor0, motor1
        SLAMNAV::StatusMotor motor0(
            ms.connection_m0 == 1,
            ms.status_m0,
            static_cast<float>(ms.temp_m0),
            static_cast<float>(ms.cur_m0) / 10.0f
        );

        SLAMNAV::StatusMotor motor1(
            ms.connection_m1 == 1,
            ms.status_m1,
            static_cast<float>(ms.temp_m1),
            static_cast<float>(ms.cur_m1) / 10.0f
        );

        // 4. StatusPower (struct)
        float charge_current = 0.0f;
        float contact_voltage = 0.0f;
        if (robot_model == "D400" || robot_model == "MECANUM")
        {
            charge_current = static_cast<float>(ms.charge_current);
            contact_voltage = static_cast<float>(ms.contact_voltage);
        }

        SLAMNAV::StatusPower power(
            static_cast<float>(ms.bat_in),
            static_cast<float>(ms.bat_out),
            static_cast<float>(ms.bat_current),
            static_cast<float>(ms.total_power),
            static_cast<float>(ms.power),
            static_cast<float>(ms.bat_percent),
            static_cast<float>(ms.tabos_voltage),
            static_cast<float>(ms.tabos_current),
            static_cast<float>(ms.tabos_status),
            static_cast<float>(ms.tabos_ttf),
            static_cast<float>(ms.tabos_tte),
            static_cast<float>(ms.tabos_soc),
            static_cast<float>(ms.tabos_soh),
            static_cast<float>(ms.tabos_temperature),
            static_cast<float>(ms.tabos_rc),
            static_cast<float>(ms.tabos_ae),
            charge_current,
            contact_voltage
        );

        // 5. StatusRobotState (table)
        std::string charge_state = get_charge_state_string(
            ms.charge_state, robot_model.toStdString());

        auto robot_state = SLAMNAV::CreateStatusRobotStateDirect(
            fbb,
            charge_state.c_str(),                     // charge
            is_dock,                                  // dock
            ms.motor_stop_state == 1,                 // emo
            cur_loc_state.toStdString().c_str(),      // localization
            ms.power_state == 1,                      // power
            false,  // sss_recovery
            false,  // sw_reset
            false,  // sw_stop
            false,  // sw_start
            false,  // sf_bumper_detect
            false,  // sf_obs_detect
            false   // sf_operational_stop
        );

        // 6. StatusRobotSafetyIoState (table)
        std::vector<uint8_t> mcu0_dio, mcu1_dio, mcu0_din, mcu1_din;
        for (int i = 0; i < 8; i++)
        {
            mcu0_dio.push_back(ms.mcu0_dio[i] ? 1 : 0);
            mcu1_dio.push_back(ms.mcu1_dio[i] ? 1 : 0);
            mcu0_din.push_back(ms.mcu0_din[i] ? 1 : 0);
            mcu1_din.push_back(ms.mcu1_din[i] ? 1 : 0);
        }

        auto safety_io = SLAMNAV::CreateStatusRobotSafetyIoStateDirect(
            fbb,
            &mcu0_dio,
            &mcu1_dio,
            &mcu0_din,
            &mcu1_din
        );

        // 7. StatusSetting (table)
        auto setting = SLAMNAV::CreateStatusSettingDirect(
            fbb,
            config->get_robot_type().toStdString().c_str(),  // platform_type
            ""                                                // platform_name
        );

        // 8. StatusMap (table)
        std::string map_name;
        int is_loaded = static_cast<int>(unimap->get_is_loaded());
        if (is_loaded == MAP_LOADED)
        {
            QString map_path = unimap->get_map_path();
            QStringList parts = map_path.split("/");
            if (!parts.isEmpty())
            {
                map_name = parts.last().toStdString();
            }
        }
        std::string map_status = get_map_status_string(is_loaded);

        auto map_info = SLAMNAV::CreateStatusMapDirect(
            fbb,
            map_name.c_str(),
            map_status.c_str()
        );

        // Status 루트 메시지 생성
        auto status = SLAMNAV::CreateStatus(
            fbb,
            &condition,
            &status_imu,
            &motor0,
            &motor1,
            &power,
            robot_state,
            safety_io,
            setting,
            map_info
        );

        fbb.Finish(status);

        // Zenoh로 발행
        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        std::string topic = zenoh->make_topic(ZENOH_TOPIC::STATUS);
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        qDebug() << "[" << MODULE_NAME << "] publish_status error:" << e.what();
    }
}

// =============================================================================
// MoveStatus Publisher
// =============================================================================

/**
 * @brief MoveStatus 메시지 생성 및 발행 (500ms 주기)
 *
 * Topic: {robotType}/moveStatus
 * 내용: 현재 노드, 목표 노드, 이동 상태, 위치, 속도 정보
 */
static void publish_move_status(COMM_ZENOH* zenoh)
{
    if (!zenoh->is_session_valid())
    {
        return;
    }

    // 필수 모듈 확인
    MOBILE* mobile = zenoh->get_mobile();
    LOCALIZATION* loc = zenoh->get_localization();
    AUTOCONTROL* ctrl = zenoh->get_autocontrol();
    DOCKCONTROL* dctrl = zenoh->get_dockcontrol();

    if (!mobile || !loc || !ctrl)
    {
        return;
    }

    try
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        // 현재 위치 및 속도
        Eigen::Vector4d cur_pose = loc->get_cur_pose();
        Eigen::Vector3d cur_vel = mobile->get_cur_vel();

        // 현재 노드 정보
        QString cur_node_id = ctrl->get_cur_node_id();
        QString cur_node_name = ctrl->get_cur_node_name();

        // 목표 노드 정보
        QString goal_node_id = ctrl->get_goal_node_id();
        QString goal_node_name = ctrl->get_goal_node_name();
        Eigen::Vector4d goal_pose = ctrl->get_goal_pose();

        // 이동 상태 문자열
        std::string auto_move = get_auto_move_state_string(ctrl, mobile, loc);
        std::string dock_move = get_dock_move_state_string(dctrl);
        std::string jog_move = mobile->get_is_jog_moving() ? "move" : "stop";
        std::string obs_state = ctrl->get_obs_state() ? "detected" : "none";
        std::string path_state = ctrl->get_has_path() ? "exist" : "none";

        // 1. cur_node (table)
        auto cur_node = SLAMNAV::CreateMoveStatusNodeDirect(
            fbb,
            cur_node_id.toStdString().c_str(),
            cur_node_name.toStdString().c_str(),
            "",  // state
            static_cast<float>(cur_pose[0]),
            static_cast<float>(cur_pose[1]),
            static_cast<float>(cur_pose[3] * R2D)
        );

        // 2. goal_node (table)
        auto goal_node = SLAMNAV::CreateMoveStatusNodeDirect(
            fbb,
            goal_node_id.toStdString().c_str(),
            goal_node_name.toStdString().c_str(),
            "",  // state
            static_cast<float>(goal_pose[0]),
            static_cast<float>(goal_pose[1]),
            static_cast<float>(goal_pose[3] * R2D)
        );

        // 3. move_state (table)
        auto move_state = SLAMNAV::CreateMoveStatusMoveStateDirect(
            fbb,
            auto_move.c_str(),
            dock_move.c_str(),
            jog_move.c_str(),
            obs_state.c_str(),
            path_state.c_str()
        );

        // 4. pose (struct)
        SLAMNAV::MoveStatusPose pose(
            static_cast<float>(cur_pose[0]),
            static_cast<float>(cur_pose[1]),
            static_cast<float>(cur_pose[2]),
            static_cast<float>(cur_pose[3] * R2D)
        );

        // 5. vel (struct)
        SLAMNAV::MoveStatusVel vel(
            static_cast<float>(cur_vel[0]),
            static_cast<float>(cur_vel[1]),
            static_cast<float>(cur_vel[2] * R2D)
        );

        // Move_Status 루트 메시지 생성
        auto move_status = SLAMNAV::CreateMove_Status(
            fbb,
            cur_node,
            goal_node,
            move_state,
            &pose,
            &vel
        );

        fbb.Finish(move_status);

        // Zenoh로 발행
        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        std::string topic = zenoh->make_topic(ZENOH_TOPIC::MOVE_STATUS);
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        qDebug() << "[" << MODULE_NAME << "] publish_move_status error:" << e.what();
    }
}

// =============================================================================
// Status Loop Implementation
// =============================================================================

/**
 * @brief Status 스레드 메인 루프
 *
 * - robotType이 설정될 때까지 대기
 * - 100ms 주기로 Status 발행
 * - 500ms 주기로 MoveStatus 발행
 */
void COMM_ZENOH::status_loop()
{
    qDebug() << "[" << MODULE_NAME << "] status_loop started";

    // robotType이 설정될 때까지 대기
    while (is_status_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_status_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] status_loop ended (stopped before init)";
        return;
    }

    qDebug() << "[" << MODULE_NAME << "] status_loop initialized with robotType:"
             << QString::fromStdString(get_robot_type());

    // 타이밍 관리
    auto last_status_time = std::chrono::steady_clock::now();
    auto last_move_status_time = std::chrono::steady_clock::now();

    const auto status_interval = std::chrono::milliseconds(STATUS_INTERVAL_MS);
    const auto move_status_interval = std::chrono::milliseconds(MOVE_STATUS_INTERVAL_MS);

    // 메인 루프
    while (is_status_running_.load())
    {
        auto now = std::chrono::steady_clock::now();

        // Status 발행 (100ms 주기)
        if (now - last_status_time >= status_interval)
        {
            publish_status(this);
            last_status_time = now;
        }

        // MoveStatus 발행 (500ms 주기)
        if (now - last_move_status_time >= move_status_interval)
        {
            publish_move_status(this);
            last_move_status_time = now;
        }

        // CPU 사용량 최소화를 위한 짧은 sleep (10ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    qDebug() << "[" << MODULE_NAME << "] status_loop ended";
}
