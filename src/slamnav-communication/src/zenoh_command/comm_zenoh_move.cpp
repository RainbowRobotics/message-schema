/**
 * @file comm_zenoh_move.cpp
 * @brief Move 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - move/goal, move/target, move/stop, move/pause, move/resume
 *   - move/xLinear, move/circular, move/rotate
 *
 * Subscriber:
 *   - move/jog
 *
 * Publisher:
 *   - move/result
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "flatbuffer/generated/slamnav_move_generated.h"

#include <chrono>

namespace
{
    constexpr const char* MODULE_NAME = "MOVE";
    constexpr double D2R = M_PI / 180.0;

    // =========================================================================
    // Helper: DATA_MOVE -> Move_Result FlatBuffer
    // =========================================================================
    std::vector<uint8_t> build_move_result(const std::string& id,
                                           const std::string& result,
                                           const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);

        auto fb_result = SLAMNAV::CreateMove_Result(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(fb_result);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // =========================================================================
    // Helper: Build Response FlatBuffers
    // =========================================================================

    // Response_Move_Goal
    std::vector<uint8_t> build_response_goal(const std::string& id,
                                             const std::string& goal_id,
                                             const std::string& goal_name,
                                             const std::string& method,
                                             int preset,
                                             const std::string& result,
                                             const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);
        auto resp = SLAMNAV::CreateResponse_Move_Goal(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(goal_id),
            fbb.CreateString(goal_name),
            fbb.CreateString(method),
            preset,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Target
    std::vector<uint8_t> build_response_target(const std::string& id,
                                               const std::string& method,
                                               const SLAMNAV::MovePose* goal_pose,
                                               int preset,
                                               const std::string& result,
                                               const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);
        auto resp = SLAMNAV::CreateResponse_Move_Target(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(method),
            goal_pose,
            preset,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Stop
    std::vector<uint8_t> build_response_stop(const std::string& id,
                                             const std::string& result,
                                             const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_Stop(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Pause
    std::vector<uint8_t> build_response_pause(const std::string& id,
                                              const std::string& result,
                                              const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_Pause(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Resume
    std::vector<uint8_t> build_response_resume(const std::string& id,
                                               const std::string& result,
                                               const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_Resume(
            fbb,
            fbb.CreateString(id),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_XLinear
    std::vector<uint8_t> build_response_xlinear(const std::string& id,
                                                float target,
                                                float speed,
                                                const std::string& result,
                                                const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_XLinear(
            fbb,
            fbb.CreateString(id),
            target,
            speed,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Circular
    std::vector<uint8_t> build_response_circular(const std::string& id,
                                                 float target,
                                                 float speed,
                                                 const std::string& direction,
                                                 const std::string& result,
                                                 const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_Circular(
            fbb,
            fbb.CreateString(id),
            target,
            speed,
            fbb.CreateString(direction),
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Move_Rotate
    std::vector<uint8_t> build_response_rotate(const std::string& id,
                                               float target,
                                               float speed,
                                               const std::string& result,
                                               const std::string& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Move_Rotate(
            fbb,
            fbb.CreateString(id),
            target,
            speed,
            fbb.CreateString(result),
            fbb.CreateString(message)
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

} // anonymous namespace

// =============================================================================
// Move Loop Implementation
// =============================================================================

void COMM_ZENOH::move_loop()
{
    log_info("move_loop started");

    // 1. robotType이 설정될 때까지 대기
    while (is_move_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_move_running_.load())
    {
        log_info("move_loop ended (stopped before init)");
        return;
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        log_error("move_loop aborted: session invalid");
        return;
    }

    log_info("move_loop initialized with robotType: {}", get_robot_type());

    try
    {
        zenoh::Session& session = get_session();

        // 3. Topic 생성
        std::string topic_goal     = make_topic("move/goal");
        std::string topic_target   = make_topic("move/target");
        std::string topic_stop     = make_topic("move/stop");
        std::string topic_pause    = make_topic("move/pause");
        std::string topic_resume   = make_topic("move/resume");
        std::string topic_xlinear  = make_topic("move/xLinear");
        std::string topic_circular = make_topic("move/circular");
        std::string topic_rotate   = make_topic("move/rotate");
        std::string topic_jog      = make_topic("move/jog");
        std::string topic_result   = make_topic("move/result");

        log_info("move_loop registering topics with prefix: {}", get_robot_type());

        // 4. Result Publisher
        auto pub_result = session.declare_publisher(zenoh::KeyExpr(topic_result));

        // 5. Jog Subscriber
        auto sub_jog = session.declare_subscriber(
            zenoh::KeyExpr(topic_jog),
            [this](const zenoh::Sample& sample)
            {
                MOBILE* mobile_ptr = get_mobile();
                if (!mobile_ptr) return;

                const auto& payload = sample.get_payload();
                auto bytes = payload.as_vector();

                auto jog = SLAMNAV::GetMove_Jog(bytes.data());
                if (!jog) return;

                double vx = static_cast<double>(jog->vx());
                double vy = static_cast<double>(jog->vy());
                double wz = static_cast<double>(jog->wz()) * D2R;

                invoke_jog_callback(Eigen::Vector3d(vx, vy, wz));
            },
            zenoh::closures::none
        );

        log_info("Jog subscriber registered: {}", topic_jog);

        // 6. RPC Queryables

        // ---- move/goal ----
        auto q_goal = session.declare_queryable(
            zenoh::KeyExpr(topic_goal),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_goal("", "", "", "", 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Move_Goal(bytes.data());
                if (!req)
                {
                    auto resp = build_response_goal("", "", "", "", 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::string id = req->id() ? req->id()->str() : "";
                std::string goal_id = req->goal_id() ? req->goal_id()->str() : "";
                std::string goal_name = req->goal_name() ? req->goal_name()->str() : "";
                std::string method = req->method() ? req->method()->str() : "pp";
                int preset = req->preset();

                // Get modules
                UNIMAP* unimap_ptr = get_unimap();
                LOCALIZATION* loc_ptr = get_localization();
                AUTOCONTROL* ctrl_ptr = get_autocontrol();
                MOBILE* mobile_ptr = get_mobile();

                // Validate
                if (!unimap_ptr || !loc_ptr || !ctrl_ptr || !mobile_ptr)
                {
                    auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (unimap_ptr->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (!loc_ptr->get_is_loc())
                {
                    auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "not localized");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Resolve goal
                NODE* node = nullptr;
                QString goal_id_q = QString::fromStdString(goal_id);
                QString goal_name_q = QString::fromStdString(goal_name);

                if (!goal_id.empty())
                {
                    node = unimap_ptr->get_node_by_id(goal_id_q);
                }
                else if (!goal_name.empty())
                {
                    node = unimap_ptr->get_node_by_name(goal_name_q);
                    if (node)
                    {
                        goal_id = node->id.toStdString();
                    }
                }

                if (!node)
                {
                    auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "goal not found");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (goal_name.empty())
                {
                    goal_name = node->name.toStdString();
                }

                // Build DATA_MOVE and invoke callback
                DATA_MOVE msg;
                msg.id = QString::fromStdString(id);
                msg.command = "goal";
                msg.goal_node_id = QString::fromStdString(goal_id);
                msg.goal_node_name = QString::fromStdString(goal_name);
                msg.method = QString::fromStdString(method);
                msg.preset = preset;

                Eigen::Vector3d xi = TF_to_se2(node->tf);
                msg.tgt_pose_vec[0] = xi[0];
                msg.tgt_pose_vec[1] = xi[1];
                msg.tgt_pose_vec[2] = node->tf(2, 3);
                msg.tgt_pose_vec[3] = xi[2];

                Eigen::Matrix4d cur_tf = loc_ptr->get_cur_tf();
                msg.cur_pos = cur_tf.block(0, 3, 3, 1);

                mobile_ptr->move(0, 0, 0);

                // Response
                auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Invoke move callback
                ctrl_ptr->invoke_move(msg);

                // Publish result
                auto result_buf = build_move_result(id, "success", "goal accepted");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_goal);

        // ---- move/target ----
        auto q_target = session.declare_queryable(
            zenoh::KeyExpr(topic_target),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    SLAMNAV::MovePose pose(0, 0, 0, 0);
                    auto resp = build_response_target("", "", &pose, 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Move_Target(bytes.data());
                if (!req)
                {
                    SLAMNAV::MovePose pose(0, 0, 0, 0);
                    auto resp = build_response_target("", "", &pose, 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::string id = req->id() ? req->id()->str() : "";
                std::string method = req->method() ? req->method()->str() : "pp";
                int preset = req->preset();

                auto goal_pose = req->goal_pose();
                float x = goal_pose ? goal_pose->x() : 0.0f;
                float y = goal_pose ? goal_pose->y() : 0.0f;
                float z = goal_pose ? goal_pose->z() : 0.0f;
                float rz = goal_pose ? goal_pose->rz() : 0.0f;

                SLAMNAV::MovePose resp_pose(x, y, z, rz);

                // Get modules
                UNIMAP* unimap_ptr = get_unimap();
                LOCALIZATION* loc_ptr = get_localization();
                OBSMAP* obsmap_ptr = get_obsmap();
                CONFIG* config_ptr = get_config();
                AUTOCONTROL* ctrl_ptr = get_autocontrol();
                MOBILE* mobile_ptr = get_mobile();

                // Validate
                if (!unimap_ptr || !loc_ptr || !obsmap_ptr || !config_ptr || !ctrl_ptr || !mobile_ptr)
                {
                    auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (unimap_ptr->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (!loc_ptr->get_is_loc())
                {
                    auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "not localized");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check bounds
                if (x < unimap_ptr->get_map_min_x() || x > unimap_ptr->get_map_max_x() ||
                    y < unimap_ptr->get_map_min_y() || y > unimap_ptr->get_map_max_y())
                {
                    auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "out of bounds");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check collision
                Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(x, y, static_cast<double>(rz) * D2R));
                goal_tf(2, 3) = z;
                if (obsmap_ptr->is_tf_collision(goal_tf))
                {
                    auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "collision");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Build DATA_MOVE
                DATA_MOVE msg;
                msg.id = QString::fromStdString(id);
                msg.command = "target";
                msg.method = QString::fromStdString(method);
                msg.preset = preset;
                msg.tgt_pose_vec[0] = x;
                msg.tgt_pose_vec[1] = y;
                msg.tgt_pose_vec[2] = z;
                msg.tgt_pose_vec[3] = rz;

                Eigen::Matrix4d cur_tf = loc_ptr->get_cur_tf();
                msg.cur_pos = cur_tf.block(0, 3, 3, 1);

                // Response
                auto resp = build_response_target(id, method, &resp_pose, preset, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                ctrl_ptr->invoke_move(msg);

                auto result_buf = build_move_result(id, "success", "target accepted");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_target);

        // ---- move/stop ----
        auto q_stop = session.declare_queryable(
            zenoh::KeyExpr(topic_stop),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Move_Stop(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                auto resp = build_response_stop(id, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                invoke_move_stop_callback();

                auto result_buf = build_move_result(id, "success", "stop executed");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_stop);

        // ---- move/pause ----
        auto q_pause = session.declare_queryable(
            zenoh::KeyExpr(topic_pause),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Move_Pause(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                AUTOCONTROL* ctrl_ptr = get_autocontrol();
                if (ctrl_ptr)
                {
                    ctrl_ptr->set_is_pause(true);
                }

                auto resp = build_response_pause(id, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_move_result(id, "success", "paused");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_pause);

        // ---- move/resume ----
        auto q_resume = session.declare_queryable(
            zenoh::KeyExpr(topic_resume),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                std::string id;

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Move_Resume(bytes.data());
                    if (req && req->id())
                    {
                        id = req->id()->str();
                    }
                }

                AUTOCONTROL* ctrl_ptr = get_autocontrol();
                if (ctrl_ptr)
                {
                    ctrl_ptr->set_is_pause(false);
                }

                auto resp = build_response_resume(id, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_move_result(id, "success", "resumed");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_resume);

        // ---- move/xLinear ----
        auto q_xlinear = session.declare_queryable(
            zenoh::KeyExpr(topic_xlinear),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_xlinear("", 0, 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Move_XLinear(bytes.data());
                if (!req)
                {
                    auto resp = build_response_xlinear("", 0, 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::string id = req->id() ? req->id()->str() : "";
                float target = req->target();
                float speed = req->speed();

                // Validate
                if (fabs(target) > 10.0f || fabs(speed) > 1.5f)
                {
                    auto resp = build_response_xlinear(id, target, speed, "reject", "value out of range");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                DATA_MOVE msg;
                msg.id = QString::fromStdString(id);
                msg.command = "xLinear";
                msg.target = target;
                msg.speed = speed;

                auto resp = build_response_xlinear(id, target, speed, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                invoke_profile_move_callback(msg);

                auto result_buf = build_move_result(id, "success", "xLinear started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_xlinear);

        // ---- move/circular ----
        auto q_circular = session.declare_queryable(
            zenoh::KeyExpr(topic_circular),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_circular("", 0, 0, "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Move_Circular(bytes.data());
                if (!req)
                {
                    auto resp = build_response_circular("", 0, 0, "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::string id = req->id() ? req->id()->str() : "";
                float target = req->target();
                float speed = req->speed();
                std::string direction = req->direction() ? req->direction()->str() : "";

                // Validate (target in degrees)
                if (fabs(target) > 360.0f || fabs(speed) > 60.0f)
                {
                    auto resp = build_response_circular(id, target, speed, direction, "reject", "value out of range");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                DATA_MOVE msg;
                msg.id = QString::fromStdString(id);
                msg.command = "circular";
                msg.target = target * D2R;
                msg.speed = speed * D2R;
                msg.direction = QString::fromStdString(direction);

                auto resp = build_response_circular(id, target, speed, direction, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                invoke_profile_move_callback(msg);

                auto result_buf = build_move_result(id, "success", "circular started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_circular);

        // ---- move/rotate ----
        auto q_rotate = session.declare_queryable(
            zenoh::KeyExpr(topic_rotate),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_rotate("", 0, 0, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Move_Rotate(bytes.data());
                if (!req)
                {
                    auto resp = build_response_rotate("", 0, 0, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                std::string id = req->id() ? req->id()->str() : "";
                float target = req->target();
                float speed = req->speed();

                // Validate (target/speed in degrees)
                if (fabs(target) > 360.0f || fabs(speed) > 60.0f)
                {
                    auto resp = build_response_rotate(id, target, speed, "reject", "value out of range");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                DATA_MOVE msg;
                msg.id = QString::fromStdString(id);
                msg.command = "rotate";
                msg.target = target * D2R;
                msg.speed = speed * D2R;

                auto resp = build_response_rotate(id, target, speed, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                invoke_profile_move_callback(msg);

                auto result_buf = build_move_result(id, "success", "rotate started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        log_info("Queryable registered: {}", topic_rotate);

        // 7. Main loop - keep alive
        while (is_move_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        log_info("move_loop ending, resources will be released");
    }
    catch (const zenoh::ZException& e)
    {
        log_error("move_loop Zenoh exception: {}", e.what());
    }
    catch (const std::exception& e)
    {
        log_error("move_loop exception: {}", e.what());
    }

    log_info("move_loop ended");
}
