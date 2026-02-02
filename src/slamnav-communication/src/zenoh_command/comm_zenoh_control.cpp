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
 *   - control/sensor, control/path, control/detectMarker
 *
 * Publisher:
 *   - control/dock/result (도킹 결과)
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "flatbuffer/generated/slamnav_control_generated.h"

namespace
{
    constexpr const char* MODULE_NAME = "CONTROL";

    // =========================================================================
    // Helper: Build Response FlatBuffers
    // =========================================================================

    std::vector<uint8_t> build_response_get_safety_field(const std::string& id,
                                                          int safety_field,
                                                          const std::string& result,
                                                          const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Get_Safety_Field(
            fbb,
            fbb.CreateString(id),
            safety_field,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_set_safety_field(const std::string& id,
                                                          int safety_field,
                                                          const std::string& result,
                                                          const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Set_Safety_Field(
            fbb,
            fbb.CreateString(id),
            safety_field,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_get_safety_flag(
        const std::string& id,
        const std::vector<std::pair<std::string, bool>>& flags,
        const std::string& result,
        const std::string& message)
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
            fbb.CreateString(id),
            fbb.CreateVector(flag_offsets),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_set_safety_flag(
        const std::string& id,
        const std::vector<std::pair<std::string, bool>>& flags,
        const std::string& result,
        const std::string& message)
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
            fbb.CreateString(id),
            fbb.CreateVector(flag_offsets),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_get_safety_io(
        const std::string& id,
        const std::vector<bool>& mcu0_dio,
        const std::vector<bool>& mcu1_dio,
        const std::vector<bool>& mcu0_din,
        const std::vector<bool>& mcu1_din,
        const std::string& result,
        const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<uint8_t> mcu0_dio_u8(mcu0_dio.begin(), mcu0_dio.end());
        std::vector<uint8_t> mcu1_dio_u8(mcu1_dio.begin(), mcu1_dio.end());
        std::vector<uint8_t> mcu0_din_u8(mcu0_din.begin(), mcu0_din.end());
        std::vector<uint8_t> mcu1_din_u8(mcu1_din.begin(), mcu1_din.end());

        auto resp = SLAMNAV::CreateResponse_Get_Safety_Io(
            fbb,
            fbb.CreateString(id),
            fbb.CreateVector(mcu0_dio_u8),
            fbb.CreateVector(mcu1_dio_u8),
            fbb.CreateVector(mcu0_din_u8),
            fbb.CreateVector(mcu1_din_u8),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_set_safety_io(
        const std::string& id,
        const std::string& command,
        const std::vector<bool>& mcu0_din,
        const std::vector<bool>& mcu1_din,
        const std::string& result,
        const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<uint8_t> mcu0_din_u8(mcu0_din.begin(), mcu0_din.end());
        std::vector<uint8_t> mcu1_din_u8(mcu1_din.begin(), mcu1_din.end());

        auto resp = SLAMNAV::CreateResponse_Set_Safety_Io(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            fbb.CreateVector(mcu0_din_u8),
            fbb.CreateVector(mcu1_din_u8),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_dock(const std::string& id,
                                              const std::string& command,
                                              const std::string& result,
                                              const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Dock(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_charge_trigger(const std::string& id,
                                                        bool onoff,
                                                        const std::string& result,
                                                        const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Charge_Trigger(
            fbb,
            fbb.CreateString(id),
            onoff,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_get_obs_box(const std::string& id,
                                                     const std::string& command,
                                                     const SLAMNAV::ObsBox* min_box,
                                                     const SLAMNAV::ObsBox* max_box,
                                                     float range,
                                                     const std::string& result,
                                                     const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Get_Obs_Box(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            min_box,
            max_box,
            range,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_set_obs_box(const std::string& id,
                                                     const std::string& command,
                                                     const SLAMNAV::ObsBox* min_box,
                                                     const SLAMNAV::ObsBox* max_box,
                                                     float range,
                                                     const std::string& result,
                                                     const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Set_Obs_Box(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            min_box,
            max_box,
            range,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_led(const std::string& id,
                                             bool onoff,
                                             const std::string& color,
                                             const std::string& result,
                                             const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Led(
            fbb,
            fbb.CreateString(id),
            onoff,
            fbb.CreateString(color),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_motor(const std::string& id,
                                               bool onoff,
                                               const std::string& result,
                                               const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Motor(
            fbb,
            fbb.CreateString(id),
            onoff,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_jog(const std::string& id,
                                             bool onoff,
                                             const std::string& result,
                                             const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Jog(
            fbb,
            fbb.CreateString(id),
            onoff,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_sensor(const std::string& id,
                                                const std::string& command,
                                                bool onoff,
                                                int frequency,
                                                const std::string& result,
                                                const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Sensor(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            onoff,
            frequency,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_path(const std::string& id,
                                              bool onoff,
                                              int frequency,
                                              const std::string& result,
                                              const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Path(
            fbb,
            fbb.CreateString(id),
            onoff,
            frequency,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    std::vector<uint8_t> build_response_detect(const std::string& id,
                                                const std::string& command,
                                                int camera_number,
                                                const std::string& camera_serial,
                                                float marker_size,
                                                const std::vector<float>& tf,
                                                const std::string& result,
                                                const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);
        auto resp = SLAMNAV::CreateResponse_Detect(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(command),
            camera_number,
            fbb.CreateString(camera_serial),
            marker_size,
            fbb.CreateVector(tf),
            fbb.CreateString(result),
            fbb.CreateString(message)
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
    log_info("control_loop started");

    // 1. robotType이 설정될 때까지 대기
    while (is_control_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_control_running_.load())
    {
        log_info("control_loop ended (stopped before init)");
        return;
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        log_error("control_loop aborted: session invalid");
        return;
    }

    log_info("control_loop initialized with robotType: {}", get_robot_type());

    try
    {
        zenoh::Session& session = get_session();

        // 3. Topic 생성
        std::string topic_get_sf      = make_topic("control/getSafetyField");
        std::string topic_set_sf      = make_topic("control/setSafetyField");
        std::string topic_get_sf_flag = make_topic("control/getSafetyFlag");
        std::string topic_set_sf_flag = make_topic("control/setSafetyFlag");
        std::string topic_get_sio     = make_topic("control/getSafetyIo");
        std::string topic_set_sio     = make_topic("control/setSafetyIo");
        std::string topic_dock        = make_topic("control/dock");
        std::string topic_charge      = make_topic("control/chargeTrigger");
        std::string topic_get_obs     = make_topic("control/getObsBox");
        std::string topic_set_obs     = make_topic("control/setObsBox");
        std::string topic_led         = make_topic("control/led");
        std::string topic_motor       = make_topic("control/motor");
        std::string topic_jog         = make_topic("control/jog");
        std::string topic_sensor      = make_topic("control/sensor");
        std::string topic_path        = make_topic("control/path");
        std::string topic_detect      = make_topic("control/detectMarker");

        log_info("control_loop registering topics with prefix: {}", get_robot_type());

        // ---- control/getSafetyField ----
        auto q_get_sf = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sf),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Field(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    auto resp = build_response_get_safety_field(id, 0, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto status = mobile_ptr->get_status();
                int field = static_cast<int>(status.lidar_field);

                auto resp = build_response_get_safety_field(id, field, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_get_sf);

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

                std::string id = req->id() ? req->id()->str() : "";
                int field = req->safety_field();

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    auto resp = build_response_set_safety_field(id, field, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                mobile_ptr->setlidarfield(static_cast<unsigned int>(field));

                auto resp = build_response_set_safety_field(id, field, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_set_sf);

        // ---- control/getSafetyFlag ----
        auto q_get_sf_flag = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sf_flag),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Flag(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    std::vector<std::pair<std::string, bool>> empty_flags;
                    auto resp = build_response_get_safety_flag(id, empty_flags, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto ms = mobile_ptr->get_status();

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
        log_info("Queryable registered: {}", topic_get_sf_flag);

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

                std::string id = req->id() ? req->id()->str() : "";

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
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

                        if (!value)
                        {
                            if (name == "bumper")
                            {
                                mobile_ptr->clearbumperstop();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "interlock")
                            {
                                mobile_ptr->clearinterlockstop();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "obstacle")
                            {
                                mobile_ptr->clearobs();
                                processed_flags.push_back({name, false});
                            }
                            else if (name == "operationStop")
                            {
                                mobile_ptr->recover();
                                processed_flags.push_back({name, false});
                            }
                            else
                            {
                                all_success = false;
                            }
                        }
                    }
                }

                std::string result = all_success ? "accept" : "partial";
                auto resp = build_response_set_safety_flag(id, processed_flags, result, "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_set_sf_flag);

        // ---- control/getSafetyIo ----
        auto q_get_sio = session.declare_queryable(
            zenoh::KeyExpr(topic_get_sio),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Safety_Io(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                std::vector<bool> empty_io(8, false);
                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    auto resp = build_response_get_safety_io(id, empty_io, empty_io, empty_io, empty_io, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                MOBILE_STATUS ms = mobile_ptr->get_status();
                std::vector<bool> mcu0_dio(8), mcu1_dio(8), mcu0_din(8), mcu1_din(8);

                for (int i = 0; i < 8; i++)
                {
                    mcu0_dio[i] = (ms.mcu0_dio[i] == 1);
                    mcu1_dio[i] = (ms.mcu1_dio[i] == 1);
                    mcu0_din[i] = false;
                    mcu1_din[i] = false;
                }

                auto resp = build_response_get_safety_io(id, mcu0_dio, mcu1_dio, mcu0_din, mcu1_din, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_get_sio);

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

                std::string id = req->id() ? req->id()->str() : "";
                std::string command = req->command() ? req->command()->str() : "";

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
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
                        mobile_ptr->set_IO_individual_output(static_cast<int>(i), mcu0_din[i] ? 1 : 0);
                    }
                }

                if (req->mcu1_din())
                {
                    for (size_t i = 0; i < std::min(static_cast<size_t>(8), req->mcu1_din()->size()); ++i)
                    {
                        mcu1_din[i] = req->mcu1_din()->Get(i);
                        mobile_ptr->set_IO_individual_output(8 + static_cast<int>(i), mcu1_din[i] ? 1 : 0);
                    }
                }

                auto resp = build_response_set_safety_io(id, command, mcu0_din, mcu1_din, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_set_sio);

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

                std::string id = req->id() ? req->id()->str() : "";
                std::string command = req->command() ? req->command()->str() : "";

                DOCKCONTROL* dctrl_ptr = get_dockcontrol();
                AUTOCONTROL* ctrl_ptr = get_autocontrol();
                MOBILE* mobile_ptr = get_mobile();
                CONFIG* config_ptr = get_config();

                if (!dctrl_ptr || !ctrl_ptr || !mobile_ptr || !config_ptr)
                {
                    auto resp = build_response_dock(id, command, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                dctrl_ptr->set_cmd_id(QString::fromStdString(id));

                if (command == "dock")
                {
                    if (ctrl_ptr->get_is_moving())
                    {
                        auto resp = build_response_dock(id, command, "reject", "robot is moving");
                        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                        return;
                    }

                    int d_field = config_ptr->get_docking_field();
                    if (d_field == -1)
                    {
                        mobile_ptr->set_detect_mode(0.0);
                    }
                    else
                    {
                        mobile_ptr->setlidarfield(d_field);
                    }

                    ctrl_ptr->set_is_moving(true);
                    invoke_docking_start_callback();

                    auto resp = build_response_dock(id, command, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else if (command == "undock")
                {
                    if (dctrl_ptr->get_dock_fsm_state() != DOCKING_FSM_OFF)
                    {
                        auto resp = build_response_dock(id, command, "reject", "docking in progress");
                        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                        return;
                    }

                    int d_field = config_ptr->get_docking_field();
                    if (d_field == -1)
                    {
                        mobile_ptr->set_detect_mode(0.0);
                    }
                    else
                    {
                        mobile_ptr->setlidarfield(d_field);
                    }

                    invoke_undocking_start_callback();

                    auto resp = build_response_dock(id, command, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else if (command == "dockstop" || command == "dockStop")
                {
                    int d_field = config_ptr->get_docking_field();
                    if (d_field == -1)
                    {
                        mobile_ptr->set_detect_mode(0.0);
                    }

                    dctrl_ptr->stop();
                    ctrl_ptr->set_is_moving(false);
                    invoke_docking_stop_callback();

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
        log_info("Queryable registered: {}", topic_dock);

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

                std::string id = req->id() ? req->id()->str() : "";
                bool onoff = req->onoff();

                MOBILE* mobile_ptr = get_mobile();
                CONFIG* config_ptr = get_config();
                if (!mobile_ptr || !config_ptr)
                {
                    auto resp = build_response_charge_trigger(id, onoff, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (config_ptr->get_charge_type() == "XNERGY")
                {
                    int non_used_int = 0;
                    mobile_ptr->xnergy_command(0, non_used_int);

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
        log_info("Queryable registered: {}", topic_charge);

        // ---- control/getObsBox ----
        auto q_get_obs = session.declare_queryable(
            zenoh::KeyExpr(topic_get_obs),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;
                std::string command;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Obs_Box(bytes.data());
                    if (req)
                    {
                        if (req->id()) id = req->id()->str();
                        if (req->command()) command = req->command()->str();
                    }
                }

                SLAMNAV::ObsBox min_box(0, 0, 0);
                SLAMNAV::ObsBox max_box(0, 0, 0);

                auto resp = build_response_get_obs_box(id, command, &min_box, &max_box, 0.0f, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_get_obs);

        // ---- control/setObsBox ----
        auto q_set_obs = session.declare_queryable(
            zenoh::KeyExpr(topic_set_obs),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;
                std::string command;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Set_Obs_Box(bytes.data());
                    if (req)
                    {
                        if (req->id()) id = req->id()->str();
                        if (req->command()) command = req->command()->str();
                    }
                }

                SLAMNAV::ObsBox min_box(0, 0, 0);
                SLAMNAV::ObsBox max_box(0, 0, 0);

                auto resp = build_response_set_obs_box(id, command, &min_box, &max_box, 0.0f, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_set_obs);

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

                std::string id = req->id() ? req->id()->str() : "";
                bool onoff = req->onoff();
                std::string color = req->color() ? req->color()->str() : "";

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    auto resp = build_response_led(id, onoff, color, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto resp = build_response_led(id, onoff, color, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_led);

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

                std::string id = req->id() ? req->id()->str() : "";
                bool onoff = req->onoff();

                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr)
                {
                    auto resp = build_response_motor(id, onoff, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (onoff)
                {
                    mobile_ptr->motor_on();
                    auto resp = build_response_motor(id, onoff, "accept", "");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
                else
                {
                    auto resp = build_response_motor(id, onoff, "reject", "motor off not supported");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                }
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_motor);

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

                std::string id = req->id() ? req->id()->str() : "";
                bool onoff = req->onoff();

                auto resp = build_response_jog(id, onoff, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_jog);

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

                std::string id = req->id() ? req->id()->str() : "";
                std::string command = req->command() ? req->command()->str() : "";
                bool onoff = req->onoff();
                int frequency = req->frequency();

                auto resp = build_response_sensor(id, command, onoff, frequency, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_sensor);

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

                std::string id = req->id() ? req->id()->str() : "";
                bool onoff = req->onoff();
                int frequency = req->frequency();

                auto resp = build_response_path(id, onoff, frequency, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_path);

        // ---- control/detectMarker ----
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

                std::string id = req->id() ? req->id()->str() : "";
                std::string command = req->command() ? req->command()->str() : "";
                int camera_number = req->camera_number();
                std::string camera_serial = req->camera_serial() ? req->camera_serial()->str() : "";
                float marker_size = req->marker_size();

                std::vector<float> tf(16, 0.0f);
                tf[0] = tf[5] = tf[10] = tf[15] = 1.0f;

                auto resp = build_response_detect(id, command, camera_number, camera_serial, marker_size, tf, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_detect);

        // 4. Main loop - keep alive
        while (is_control_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        log_info("control_loop ending, resources will be released");
    }
    catch (const zenoh::ZException& e)
    {
        log_error("control_loop Zenoh exception: {}", e.what());
    }
    catch (const std::exception& e)
    {
        log_error("control_loop exception: {}", e.what());
    }

    log_info("control_loop ended");
}
