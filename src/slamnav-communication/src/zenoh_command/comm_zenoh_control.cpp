/**
 * @file comm_zenoh_control.cpp
 * @brief Control 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - control/getSafetyField, control/setSafetyField
 *   - control/getSafetyFlag, control/setSafetyFlag
 *   - control/getSafetyIo, control/setSafetyIo
 *   - control/dock, control/chargeTrigger
 *   - control/getObsBox, control/setObsBox
 *   - control/led, control/motor, control/jog
 *   - control/sensor, control/path, control/detect
 *
 * Publisher:
 *   - (없음 - Control_Result는 RPC Response로 처리)
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "slamnav_control_generated.h"

#include <QDebug>
#include <chrono>
#include <functional>

namespace
{
    const char* MODULE_NAME = "COMM_ZENOH_CONTROL";

    // =========================================================================
    // Helper: Build Response FlatBuffers
    // =========================================================================

    // Response_Get_Safety_Field
    std::vector<uint8_t> build_response_get_safety_field(const QString& id,
                                                          int safety_field,
                                                          const QString& result,
                                                          const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Get_Safety_Field(
            fbb,
            fbb.CreateString(id.toStdString()),
            safety_field,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Safety_Field
    std::vector<uint8_t> build_response_set_safety_field(const QString& id,
                                                          int safety_field,
                                                          const QString& result,
                                                          const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Set_Safety_Field(
            fbb,
            fbb.CreateString(id.toStdString()),
            safety_field,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Safety_Flag
    std::vector<uint8_t> build_response_get_safety_flag(
        const QString& id,
        const std::vector<std::pair<std::string, bool>>& flags,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SafetyFlag>> flag_offsets;
        for (const auto& flag : flags)
        {
            auto sf = SLAMNAV::CreateSafetyFlag(
                fbb,
                fbb.CreateString(flag.first),
                flag.second
            );
            flag_offsets.push_back(sf);
        }

        auto resp = SLAMNAV::CreateResponse_Get_Safety_Flag(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(flag_offsets),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Safety_Flag
    std::vector<uint8_t> build_response_set_safety_flag(
        const QString& id,
        const std::vector<std::pair<std::string, bool>>& flags,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SafetyFlag>> flag_offsets;
        for (const auto& flag : flags)
        {
            auto sf = SLAMNAV::CreateSafetyFlag(
                fbb,
                fbb.CreateString(flag.first),
                flag.second
            );
            flag_offsets.push_back(sf);
        }

        auto resp = SLAMNAV::CreateResponse_Set_Safety_Flag(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(flag_offsets),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Safety_Io
    std::vector<uint8_t> build_response_get_safety_io(
        const QString& id,
        const std::vector<bool>& mcu0_dio,
        const std::vector<bool>& mcu1_dio,
        const std::vector<bool>& mcu0_din,
        const std::vector<bool>& mcu1_din,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        // FlatBuffers bool vector
        std::vector<uint8_t> mcu0_dio_u8(mcu0_dio.begin(), mcu0_dio.end());
        std::vector<uint8_t> mcu1_dio_u8(mcu1_dio.begin(), mcu1_dio.end());
        std::vector<uint8_t> mcu0_din_u8(mcu0_din.begin(), mcu0_din.end());
        std::vector<uint8_t> mcu1_din_u8(mcu1_din.begin(), mcu1_din.end());

        auto resp = SLAMNAV::CreateResponse_Get_Safety_Io(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(mcu0_dio_u8),
            fbb.CreateVector(mcu1_dio_u8),
            fbb.CreateVector(mcu0_din_u8),
            fbb.CreateVector(mcu1_din_u8),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Safety_Io
    std::vector<uint8_t> build_response_set_safety_io(
        const QString& id,
        const QString& command,
        const std::vector<bool>& mcu0_din,
        const std::vector<bool>& mcu1_din,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<uint8_t> mcu0_din_u8(mcu0_din.begin(), mcu0_din.end());
        std::vector<uint8_t> mcu1_din_u8(mcu1_din.begin(), mcu1_din.end());

        auto resp = SLAMNAV::CreateResponse_Set_Safety_Io(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            fbb.CreateVector(mcu0_din_u8),
            fbb.CreateVector(mcu1_din_u8),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Dock
    std::vector<uint8_t> build_response_dock(const QString& id,
                                              const QString& command,
                                              const QString& result,
                                              const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Dock(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Charge_Trigger
    std::vector<uint8_t> build_response_charge_trigger(const QString& id,
                                                        bool onoff,
                                                        const QString& result,
                                                        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Charge_Trigger(
            fbb,
            fbb.CreateString(id.toStdString()),
            onoff,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Obs_Box
    std::vector<uint8_t> build_response_get_obs_box(const QString& id,
                                                     const QString& command,
                                                     const SLAMNAV::ObsBox* min_box,
                                                     const SLAMNAV::ObsBox* max_box,
                                                     float range,
                                                     const QString& result,
                                                     const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Get_Obs_Box(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            min_box,
            max_box,
            range,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Obs_Box
    std::vector<uint8_t> build_response_set_obs_box(const QString& id,
                                                     const QString& command,
                                                     const SLAMNAV::ObsBox* min_box,
                                                     const SLAMNAV::ObsBox* max_box,
                                                     float range,
                                                     const QString& result,
                                                     const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Set_Obs_Box(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            min_box,
            max_box,
            range,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Led
    std::vector<uint8_t> build_response_led(const QString& id,
                                             bool onoff,
                                             const QString& color,
                                             const QString& result,
                                             const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Led(
            fbb,
            fbb.CreateString(id.toStdString()),
            onoff,
            fbb.CreateString(color.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Motor
    std::vector<uint8_t> build_response_motor(const QString& id,
                                               bool onoff,
                                               const QString& result,
                                               const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Motor(
            fbb,
            fbb.CreateString(id.toStdString()),
            onoff,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Jog
    std::vector<uint8_t> build_response_jog(const QString& id,
                                             bool onoff,
                                             const QString& result,
                                             const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Jog(
            fbb,
            fbb.CreateString(id.toStdString()),
            onoff,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Sensor
    std::vector<uint8_t> build_response_sensor(const QString& id,
                                                const QString& command,
                                                bool onoff,
                                                int frequency,
                                                const QString& result,
                                                const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Sensor(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            onoff,
            frequency,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Path
    std::vector<uint8_t> build_response_path(const QString& id,
                                              bool onoff,
                                              int frequency,
                                              const QString& result,
                                              const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Path(
            fbb,
            fbb.CreateString(id.toStdString()),
            onoff,
            frequency,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Detect
    std::vector<uint8_t> build_response_detect(const QString& id,
                                                const QString& command,
                                                int camera_number,
                                                const QString& camera_serial,
                                                float marker_size,
                                                const std::vector<float>& tf,
                                                const QString& result,
                                                const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);
        auto resp = SLAMNAV::CreateResponse_Detect(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(command.toStdString()),
            camera_number,
            fbb.CreateString(camera_serial.toStdString()),
            marker_size,
            fbb.CreateVector(tf),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

} // anonymous namespace

// =============================================================================
// COMM_ZENOH::control_loop()
// =============================================================================
void COMM_ZENOH::control_loop()
{
    qDebug() << "[" << MODULE_NAME << "] control_loop started";

    // 1. robotType이 설정될 때까지 대기
    while (is_control_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_control_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] control_loop aborted (not running)";
        return;
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        qDebug() << "[" << MODULE_NAME << "] control_loop aborted (session invalid)";
        return;
    }

    try
    {
        zenoh::Session& session = get_session();

        // 3. Topic 생성
        std::string topic_get_sf      = make_topic(ZENOH_TOPIC::CTRL_GET_SF);
        std::string topic_set_sf      = make_topic(ZENOH_TOPIC::CTRL_SET_SF);
        std::string topic_get_sf_flag = make_topic(ZENOH_TOPIC::CTRL_GET_SF_FLAG);
        std::string topic_set_sf_flag = make_topic(ZENOH_TOPIC::CTRL_SET_SF_FLAG);
        std::string topic_get_sio     = make_topic(ZENOH_TOPIC::CTRL_GET_SAFETY_IO);
        std::string topic_set_sio     = make_topic(ZENOH_TOPIC::CTRL_SET_SAFETY_IO);
        std::string topic_dock        = make_topic(ZENOH_TOPIC::CTRL_DOCK);
        std::string topic_charge      = make_topic(ZENOH_TOPIC::CTRL_CHARGE);
        std::string topic_get_obs     = make_topic(ZENOH_TOPIC::CTRL_GET_OBS_BOX);
        std::string topic_set_obs     = make_topic(ZENOH_TOPIC::CTRL_SET_OBS_BOX);
        std::string topic_led         = make_topic(ZENOH_TOPIC::CTRL_LED);
        std::string topic_motor       = make_topic(ZENOH_TOPIC::CTRL_MOTOR);
        std::string topic_jog         = make_topic(ZENOH_TOPIC::CTRL_JOG);
        std::string topic_sensor      = make_topic(ZENOH_TOPIC::CTRL_SENSOR);
        std::string topic_path        = make_topic(ZENOH_TOPIC::CTRL_PATH);
        std::string topic_detect      = make_topic(ZENOH_TOPIC::CTRL_DETECT);

        qDebug() << "[" << MODULE_NAME << "] Registering topics with prefix:" << QString::fromStdString(get_robot_type());

        // ---- control/getSafetyField ----
        auto q_get_sf = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sf),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Field(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                if (!mobile)
                {
                    auto resp = build_response_get_safety_field(id, 0, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto status = mobile->get_status();
                int field = static_cast<int>(status.lidar_field);

                auto resp = build_response_get_safety_field(id, field, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_get_sf);

        // ---- control/setSafetyField ----
        auto q_set_sf = session.declare_queryable(
            zenoh::KeyExpr(topic_set_sf),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_set_safety_field("", 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Set_Safety_Field(bytes.data());
                if (!req)
                {
                    auto resp = build_response_set_safety_field("", 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                int field = req->safety_field();

                if (!mobile)
                {
                    auto resp = build_response_set_safety_field(id, field, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                mobile->setlidarfield(static_cast<unsigned int>(field));

                auto resp = build_response_set_safety_field(id, field, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_set_sf);

        // ---- control/getSafetyFlag ----
        auto q_get_sf_flag = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sf_flag),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Flag(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                if (!mobile)
                {
                    std::vector<std::pair<std::string, bool>> empty_flags;
                    auto resp = build_response_get_safety_flag(id, empty_flags, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto ms = mobile->get_status();

                bool obstacle = (ms.safety_state_obstacle_detected_1 != 0) || (ms.safety_state_obstacle_detected_2 != 0);
                bool bumper = (ms.safety_state_bumper_stop_1 != 0) || (ms.safety_state_bumper_stop_2 != 0);
                bool interlock = (ms.safety_state_interlock_stop_1 != 0) || (ms.safety_state_interlock_stop_2 != 0);
                bool opStop = (ms.operational_stop_state_flag_1 != 0) || (ms.operational_stop_state_flag_2 != 0);

                std::vector<std::pair<std::string, bool>> flags = {
                    {"obstacle", obstacle},
                    {"bumper", bumper},
                    {"interlock", interlock},
                    {"operationStop", opStop}
                };

                auto resp = build_response_get_safety_flag(id, flags, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_get_sf_flag);

        // ---- control/setSafetyFlag ----
        auto q_set_sf_flag = session.declare_queryable(
            zenoh::KeyExpr(topic_set_sf_flag),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    std::vector<std::pair<std::string, bool>> empty_flags;
                    auto resp = build_response_set_safety_flag("", empty_flags, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Set_Safety_Flag(bytes.data());
                if (!req)
                {
                    std::vector<std::pair<std::string, bool>> empty_flags;
                    auto resp = build_response_set_safety_flag("", empty_flags, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");

                if (!mobile)
                {
                    std::vector<std::pair<std::string, bool>> empty_flags;
                    auto resp = build_response_set_safety_flag(id, empty_flags, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::vector<std::pair<std::string, bool>> processed_flags;
                bool all_success = true;

                if (req->reset_flag())
                {
                    for (size_t i = 0; i < req->reset_flag()->size(); ++i)
                    {
                        auto sf = req->reset_flag()->Get(i);
                        if (!sf || !sf->name()) continue;

                        std::string name = sf->name()->str();
                        bool value = sf->value();

                        // value가 false일 때 리셋 수행
                        if (!value)
                        {
                            if (name == "bumper")
                            {
                                mobile->clearbumperstop();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "interlock")
                            {
                                mobile->clearinterlockstop();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "obstacle")
                            {
                                mobile->clearobs();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "operationStop")
                            {
                                mobile->recover();
                                processed_flags.push_back({name, false});
                            }
                            else
                            {
                                all_success = false;
                            }
                        }
                    }
                }

                QString result = all_success ? "accept" : "partial";
                auto resp = build_response_set_safety_flag(id, processed_flags, result, "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_set_sf_flag);

        // ---- control/getSafetyIo ----
        auto q_get_sio = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sio),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Io(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                std::vector<bool> empty_io(8, false);
                if (!mobile)
                {
                    auto resp = build_response_get_safety_io(id, empty_io, empty_io, empty_io, empty_io, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                MOBILE_STATUS ms = mobile->get_status();
                std::vector<bool> mcu0_dio(8), mcu1_dio(8), mcu0_din(8), mcu1_din(8);

                for (int i = 0; i < 8; i++)
                {
                    mcu0_dio[i] = (ms.mcu0_dio[i] == 1);
                    mcu1_dio[i] = (ms.mcu1_dio[i] == 1);
                    mcu0_din[i] = false; // din은 별도 처리 필요시 추가
                    mcu1_din[i] = false;
                }

                auto resp = build_response_get_safety_io(id, mcu0_dio, mcu1_dio, mcu0_din, mcu1_din, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_get_sio);

        // ---- control/setSafetyIo ----
        auto q_set_sio = session.declare_queryable(
            zenoh::KeyExpr(topic_set_sio),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::vector<bool> empty_io(8, false);

                if (!payload.has_value())
                {
                    auto resp = build_response_set_safety_io("", "", empty_io, empty_io, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Set_Safety_Io(bytes.data());
                if (!req)
                {
                    auto resp = build_response_set_safety_io("", "", empty_io, empty_io, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString command = QString::fromStdString(req->command() ? req->command()->str() : "");

                if (!mobile)
                {
                    auto resp = build_response_set_safety_io(id, command, empty_io, empty_io, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::vector<bool> mcu0_din(8, false), mcu1_din(8, false);

                if (req->mcu0_din())
                {
                    for (size_t i = 0; i < std::min(static_cast<size_t>(8), req->mcu0_din()->size()); ++i)
                    {
                        mcu0_din[i] = req->mcu0_din()->Get(i);
                        mobile->set_IO_individual_output(static_cast<int>(i), mcu0_din[i] ? 1 : 0);
                    }
                }

                if (req->mcu1_din())
                {
                    for (size_t i = 0; i < std::min(static_cast<size_t>(8), req->mcu1_din()->size()); ++i)
                    {
                        mcu1_din[i] = req->mcu1_din()->Get(i);
                        mobile->set_IO_individual_output(8 + static_cast<int>(i), mcu1_din[i] ? 1 : 0);
                    }
                }

                auto resp = build_response_set_safety_io(id, command, mcu0_din, mcu1_din, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_set_sio);

        // ---- control/dock ----
        auto q_dock = session.declare_queryable(
            zenoh::KeyExpr(topic_dock),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_dock("", "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Dock(bytes.data());
                if (!req)
                {
                    auto resp = build_response_dock("", "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString command = QString::fromStdString(req->command() ? req->command()->str() : "");

                if (!dctrl || !ctrl || !mobile || !config)
                {
                    auto resp = build_response_dock(id, command, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (dctrl) dctrl->set_cmd_id(id);

                if (command == "dock")
                {
                    if (ctrl->get_is_moving())
                    {
                        auto resp = build_response_dock(id, command, "reject", "robot is moving");
                        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                        return;
                    }

                    int d_field = config->get_docking_field();
                    if (d_field == -1) mobile->set_detect_mode(0.0);
                    else               mobile->setlidarfield(d_field);

                    ctrl->set_is_moving(true);
                    Q_EMIT signal_docking_start();

                    auto resp = build_response_dock(id, command, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else if (command == "undock")
                {
                    if (dctrl->get_dock_fsm_state() != DOCKING_FSM_OFF)
                    {
                        auto resp = build_response_dock(id, command, "reject", "docking in progress");
                        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                        return;
                    }

                    int d_field = config->get_docking_field();
                    if (d_field == -1) mobile->set_detect_mode(0.0);
                    else               mobile->setlidarfield(d_field);

                    Q_EMIT signal_undocking_start();

                    auto resp = build_response_dock(id, command, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else if (command == "dockstop" || command == "dockStop")
                {
                    int d_field = config->get_docking_field();
                    if (d_field == -1) mobile->set_detect_mode(0.0);

                    dctrl->stop();
                    ctrl->set_is_moving(false);
                    Q_EMIT signal_docking_stop();

                    auto resp = build_response_dock(id, command, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else
                {
                    auto resp = build_response_dock(id, command, "reject", "unknown command");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_dock);

        // ---- control/chargeTrigger ----
        auto q_charge = session.declare_queryable(
            zenoh::KeyExpr(topic_charge),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_charge_trigger("", false, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Charge_Trigger(bytes.data());
                if (!req)
                {
                    auto resp = build_response_charge_trigger("", false, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                bool onoff = req->onoff();

                if (!mobile || !config)
                {
                    auto resp = build_response_charge_trigger(id, onoff, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (config->get_charge_type() == "XNERGY")
                {
                    int non_used_int = 0;
                    mobile->xnergy_command(0, non_used_int);

                    // Note: delayed task는 여기서 직접 처리하지 않음
                    // 필요시 별도 타이머 메커니즘 사용

                    auto resp = build_response_charge_trigger(id, onoff, "accept", "type: XNERGY");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else
                {
                    auto resp = build_response_charge_trigger(id, onoff, "reject", "type: not XNERGY");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_charge);

        // ---- control/getObsBox ----
        auto q_get_obs = session.declare_queryable(
            zenoh::KeyExpr(topic_get_obs),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";
                QString command = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Obs_Box(bytes.data());
                    if (req)
                    {
                        if (req->id()) id = QString::fromStdString(req->id()->str());
                        if (req->command()) command = QString::fromStdString(req->command()->str());
                    }
                }

                SLAMNAV::ObsBox min_box(0, 0, 0);
                SLAMNAV::ObsBox max_box(0, 0, 0);

                // TODO: 실제 obs box 값 조회 구현
                auto resp = build_response_get_obs_box(id, command, &min_box, &max_box, 0.0f, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_get_obs);

        // ---- control/setObsBox ----
        auto q_set_obs = session.declare_queryable(
            zenoh::KeyExpr(topic_set_obs),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";
                QString command = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Set_Obs_Box(bytes.data());
                    if (req)
                    {
                        if (req->id()) id = QString::fromStdString(req->id()->str());
                        if (req->command()) command = QString::fromStdString(req->command()->str());
                    }
                }

                SLAMNAV::ObsBox min_box(0, 0, 0);
                SLAMNAV::ObsBox max_box(0, 0, 0);

                // TODO: 실제 obs box 값 설정 구현
                auto resp = build_response_set_obs_box(id, command, &min_box, &max_box, 0.0f, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_set_obs);

        // ---- control/led ----
        auto q_led = session.declare_queryable(
            zenoh::KeyExpr(topic_led),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_led("", false, "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Led(bytes.data());
                if (!req)
                {
                    auto resp = build_response_led("", false, "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                bool onoff = req->onoff();
                QString color = QString::fromStdString(req->color() ? req->color()->str() : "");

                if (!mobile)
                {
                    auto resp = build_response_led(id, onoff, color, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // LED 제어는 signal을 통해 처리 (실제 구현 필요)
                auto resp = build_response_led(id, onoff, color, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_led);

        // ---- control/motor ----
        auto q_motor = session.declare_queryable(
            zenoh::KeyExpr(topic_motor),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_motor("", false, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Motor(bytes.data());
                if (!req)
                {
                    auto resp = build_response_motor("", false, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                bool onoff = req->onoff();

                if (!mobile)
                {
                    auto resp = build_response_motor(id, onoff, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (onoff)
                {
                    mobile->motor_on();
                    auto resp = build_response_motor(id, onoff, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else
                {
                    // motor_off 지원 여부에 따라 처리
                    auto resp = build_response_motor(id, onoff, "reject", "motor off not supported");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_motor);

        // ---- control/jog ----
        auto q_jog = session.declare_queryable(
            zenoh::KeyExpr(topic_jog),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_jog("", false, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Jog(bytes.data());
                if (!req)
                {
                    auto resp = build_response_jog("", false, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                bool onoff = req->onoff();

                // Jog 모드 on/off 처리
                auto resp = build_response_jog(id, onoff, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_jog);

        // ---- control/sensor ----
        auto q_sensor = session.declare_queryable(
            zenoh::KeyExpr(topic_sensor),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_sensor("", "", false, 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Sensor(bytes.data());
                if (!req)
                {
                    auto resp = build_response_sensor("", "", false, 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString command = QString::fromStdString(req->command() ? req->command()->str() : "");
                bool onoff = req->onoff();
                int frequency = req->frequency();

                // 센서 on/off 및 주파수 설정 처리
                // command: camera, lidar2d, lidar3d
                auto resp = build_response_sensor(id, command, onoff, frequency, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_sensor);

        // ---- control/path ----
        auto q_path = session.declare_queryable(
            zenoh::KeyExpr(topic_path),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_path("", false, 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Path(bytes.data());
                if (!req)
                {
                    auto resp = build_response_path("", false, 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                bool onoff = req->onoff();
                int frequency = req->frequency();

                // 경로 발행 on/off 및 주파수 설정 처리
                auto resp = build_response_path(id, onoff, frequency, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_path);

        // ---- control/detect ----
        auto q_detect = session.declare_queryable(
            zenoh::KeyExpr(topic_detect),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    std::vector<float> empty_tf;
                    auto resp = build_response_detect("", "", 0, "", 0.0f, empty_tf, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Detect(bytes.data());
                if (!req)
                {
                    std::vector<float> empty_tf;
                    auto resp = build_response_detect("", "", 0, "", 0.0f, empty_tf, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString command = QString::fromStdString(req->command() ? req->command()->str() : "");
                int camera_number = req->camera_number();
                QString camera_serial = QString::fromStdString(req->camera_serial() ? req->camera_serial()->str() : "");
                float marker_size = req->marker_size();

                // TODO: 마커 검출 처리 구현
                std::vector<float> tf(16, 0.0f);
                // 단위 행렬 초기화
                tf[0] = tf[5] = tf[10] = tf[15] = 1.0f;

                auto resp = build_response_detect(id, command, camera_number, camera_serial, marker_size, tf, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_detect);

        // 7. Main loop - keep alive
        while (is_control_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        qDebug() << "[" << MODULE_NAME << "] control_loop ending, resources will be released";
    }
    catch (const zenoh::ZException& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Zenoh exception:" << e.what();
    }
    catch (const std::exception& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Exception:" << e.what();
    }

    qDebug() << "[" << MODULE_NAME << "] control_loop ended";
}
