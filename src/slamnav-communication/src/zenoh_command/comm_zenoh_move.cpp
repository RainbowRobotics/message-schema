/**
 * @file comm_zenoh_move.cpp
 * @brief Move 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - move/goal, move/target, move/stop, move/pause, move/resume
 *   - move/xLinear, move/yLinear, move/circular, move/rotate
 *
 * Subscriber:
 *   - move/jog
 *
 * Publisher:
 *   - move/result
 */

#include "comm_zenoh.h"
#include "flatbuffer/generated/slamnav_move_generated.h"

namespace
{
  constexpr const char* MODULE_NAME = "ZENOH_MOVE";

  // // result data buffer
  // static std::vector<uint8_t> result_data_buffer;

  // // result 읽기 함수
  // std::vector<uint8_t> get_data() const {
  //     std::shared_lock lock(move_mtx);
  //     return result_data_buffer;
  // }

  // // result 쓰기 함수
  // void set_data(std::vector<uint8_t> new_data) {
  //     std::unique_lock lock(move_mtx);
  //     result_data_buffer = std::move(new_data);
  // }


  std::vector<uint8_t> build_result_move(const std::string& id,
                                         const std::string& goal_id,
                                         const std::string& goal_name,
                                         const std::string& method,
                                         int preset,
                                         const SLAMNAV::MovePose* goal_pose,
                                         float target,
                                         float speed,
                                         const std::string& direction,
                                         const std::string& result,
                                         const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(512);

    auto fb_result = SLAMNAV::CreateResultMove(
      fbb,
      fbb.CreateString(id),
      fbb.CreateString(goal_id),
      fbb.CreateString(goal_name),
      fbb.CreateString(method),
      preset,
      goal_pose,
      target,
      speed,
      fbb.CreateString(direction),
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(fb_result);

    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // Simple result helper (for profile move)
  std::vector<uint8_t> build_profile_result(const std::string& id,
                                            float target,
                                            float speed,
                                            const std::string& direction,
                                            const std::string& result,
                                            const std::string& message)
  {
    SLAMNAV::MovePose empty_pose(0, 0, 0, 0);
    return build_result_move(id, "", "", "",
                             0, &empty_pose, target, speed,
                             direction, result, message);
  }

  // ResponseMoveGoal
  std::vector<uint8_t> build_response_goal(const std::string& id,
                                           const std::string& goal_id,
                                           const std::string& goal_name,
                                           const std::string& method,
                                           int preset,
                                           const std::string& result,
                                           const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(512);
    auto resp = SLAMNAV::CreateResponseMoveGoal(
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

  // ResponseMoveTarget
  std::vector<uint8_t> build_response_target(const std::string& id,
                                             const std::string& method,
                                             const SLAMNAV::MovePose* goal_pose,
                                             int preset,
                                             const std::string& result,
                                             const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(512);
    auto resp = SLAMNAV::CreateResponseMoveTarget(
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

  // ResponseMoveStop
  std::vector<uint8_t> build_response_stop(const std::string& id,
                                           const std::string& result,
                                           const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveStop(
      fbb,
      fbb.CreateString(id),
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseMovePause
  std::vector<uint8_t> build_response_pause(const std::string& id,
                                            const std::string& result,
                                            const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMovePause(
      fbb,
      fbb.CreateString(id),
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseMoveResume
  std::vector<uint8_t> build_response_resume(const std::string& id,
                                             const std::string& result,
                                             const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveResume(
      fbb,
      fbb.CreateString(id),
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseMoveXLinear
  std::vector<uint8_t> build_response_xlinear(const std::string& id,
                                              float target,
                                              float speed,
                                              const std::string& result,
                                              const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveXLinear(
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

  // ResponseMoveYLinear
  std::vector<uint8_t> build_response_ylinear(const std::string& id,
                                              float target,
                                              float speed,
                                              const std::string& result,
                                              const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveYLinear(
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

  // ResponseMoveCircular
  std::vector<uint8_t> build_response_circular(const std::string& id,
                                               float target,
                                               float speed,
                                               const std::string& direction,
                                               const std::string& result,
                                               const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveCircular(
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

  // ResponseMoveRotate
  std::vector<uint8_t> build_response_rotate(const std::string& id,
                                             float target,
                                             float speed,
                                             const std::string& result,
                                             const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseMoveRotate(
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

}

void COMM_ZENOH::move_loop()
{
  log_info("move_loop started");

  while (is_move_running_.load() && get_robot_type().empty())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  if (!is_session_valid())
  {
    log_error("move_loop aborted: session invalid");
    return;
  }

  log_info("move_loop initialized with robotType: {}", get_robot_type());

  try
  {
    zenoh::Session& session = get_session();

    // Topic
    std::string topic_goal     = make_topic("move/goal");
    std::string topic_target   = make_topic("move/target");
    std::string topic_stop     = make_topic("move/stop");
    std::string topic_pause    = make_topic("move/pause");
    std::string topic_resume   = make_topic("move/resume");
    std::string topic_xlinear  = make_topic("move/xLinear");
    std::string topic_ylinear  = make_topic("move/yLinear");
    std::string topic_circular = make_topic("move/circular");
    std::string topic_rotate   = make_topic("move/rotate");
    std::string topic_jog      = make_topic("move/jog");
    std::string topic_result   = make_topic("move/result");

    log_info("move_loop registering topics with prefix: {}", get_robot_type());

    // topic_result Request
    auto send_result = [this, &topic_result](std::vector<uint8_t> data) {                                                                                                          
      if (!is_session_valid()) return;

      zenoh::Session::GetOptions opts;
      opts.payload = zenoh::Bytes(std::move(data));

      get_session().get(
          zenoh::KeyExpr(topic_result),
          "",
          [](zenoh::Reply&) {},
          []() {},
          std::move(opts)
      );
    };

    // Jog Subscriber
    auto sub_jog = session.declare_subscriber(
      zenoh::KeyExpr(topic_jog),
      [this](const zenoh::Sample& sample)
      {
        MOBILE* mobile_ptr = get_mobile();
        if (!mobile_ptr) return;

        const auto& payload = sample.get_payload();
        auto iter = payload.slice_iter();
        auto slice = iter.next();
        if (!slice.has_value()) return;

        auto jog = flatbuffers::GetRoot<SLAMNAV::MoveJog>(slice->data);
        if (!jog) return;

        double vx = static_cast<double>(jog->vx());
        double vy = static_cast<double>(jog->vy());
        double wz = static_cast<double>(jog->wz()) * D2R;

        mobile_ptr->slot_jog_update(Eigen::Vector3d(vx, vy, wz));
      },
      zenoh::closures::none
    );

    // ---- move/goal ----
    auto q_goal = session.declare_queryable(
      zenoh::KeyExpr(topic_goal),
      [this](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_goal("", "", "", "", 0, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_goal("", "", "", "", 0, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveGoal>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        std::string goal_id = req->goal_id() ? req->goal_id()->str() : "";
        std::string goal_name = req->goal_name() ? req->goal_name()->str() : "";
        std::string method = req->method() ? req->method()->str() : "pp";
        int preset = req->preset();

        // modules
        UNIMAP* unimap_ptr = get_unimap();
        LOCALIZATION* loc_ptr = get_localization();
        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        MOBILE* mobile_ptr = get_mobile();

        if (!unimap_ptr || !loc_ptr || !ctrl_ptr || !mobile_ptr)
        {
          auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (!loc_ptr->get_is_loc())
        {
          auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "not localized");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Resolve goal
        NODE* node = nullptr;
        QString qstr_goal_id = QString::fromStdString(goal_id);
        QString qstr_goal_name = QString::fromStdString(goal_name);

        if (!goal_id.empty())
        {
          node = unimap_ptr->get_node_by_id(qstr_goal_id);
        }
        else if (!goal_name.empty())
        {
          node = unimap_ptr->get_node_by_name(qstr_goal_name);
          if (node)
          {
            goal_id = node->id.toStdString();
          }
        }

        if (!node)
        {
          auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "reject", "goal not found");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (goal_name.empty())
        {
          goal_name = node->name.toStdString();
        }

        mobile_ptr->move(0, 0, 0);

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

        // Response
        auto resp = build_response_goal(id, goal_id, goal_name, method, preset, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        Q_EMIT (ctrl_ptr->signal_move(msg));
      },
      zenoh::closures::none
    );

    // ---- move/target ----
    auto q_target = session.declare_queryable(
      zenoh::KeyExpr(topic_target),
      [this](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          SLAMNAV::MovePose pose(0, 0, 0, 0);
          auto resp = build_response_target("", "", &pose, 0, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          SLAMNAV::MovePose pose(0, 0, 0, 0);
          auto resp = build_response_target("", "", &pose, 0, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveTarget>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        std::string method = req->method() ? req->method()->str() : "pp";
        int preset = req->preset();

        auto goal_pose = req->goal_pose();
        float x = goal_pose ? goal_pose->x() : 0.0f;
        float y = goal_pose ? goal_pose->y() : 0.0f;
        float z = goal_pose ? goal_pose->z() : 0.0f;
        float rz = goal_pose ? goal_pose->rz() : 0.0f;

        SLAMNAV::MovePose resp_pose(x, y, z, rz);


        //
        //
        //
        //
        //
        //
        // must reject
        
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "not developed");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        //
        //
        //
        //
        //
        //
        //
        //
        //





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
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (!loc_ptr->get_is_loc())
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "not localized");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        if (config_ptr->get_use_multi())
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "using multi");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check bounds
        if (x < unimap_ptr->get_map_min_x() || x > unimap_ptr->get_map_max_x() ||
            y < unimap_ptr->get_map_min_y() || y > unimap_ptr->get_map_max_y())
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "out of bounds");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check collision
        Eigen::Matrix4d goal_tf = se2_to_TF(Eigen::Vector3d(x, y, static_cast<double>(rz) * D2R));
        goal_tf(2, 3) = z;
        if (obsmap_ptr->is_tf_collision(goal_tf))
        {
          auto resp = build_response_target(id, method, &resp_pose, preset, "reject", "collision");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
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
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        Q_EMIT (ctrl_ptr->signal_move(msg));
      },
      zenoh::closures::none
    );

    // ---- move/stop ----
    auto q_stop = session.declare_queryable(
      zenoh::KeyExpr(topic_stop),
      [this](const zenoh::Query& query)
      {
        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        if (!ctrl_ptr)
        {
          auto resp = build_response_stop("", "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        // Acted first
        Q_EMIT (signal_auto_move_stop());

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveStop>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        std::vector<uint8_t> resp;

        if (id == "")
        {
          resp = build_response_stop("", "accept", "abnormal ID detected");
        }
        else
        {
          resp = build_response_stop(id, "accept", "");
        }
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      },
      zenoh::closures::none
    );

    // ---- move/pause ----
    auto q_pause = session.declare_queryable(
      zenoh::KeyExpr(topic_pause),
      [this](const zenoh::Query& query)
      {
        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        if (!ctrl_ptr)
        {
          auto resp = build_response_pause("", "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        // Acted first
        ctrl_ptr->set_is_pause(true);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestMovePause>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        std::vector<uint8_t> resp;

        if (id == "")
        {
          resp = build_response_pause("", "accept", "abnormal ID detected");
        }
        else
        {
          resp = build_response_pause(id, "accept", "");
        }
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      },
      zenoh::closures::none
    );

    // ---- move/resume ----
    auto q_resume = session.declare_queryable(
      zenoh::KeyExpr(topic_resume),
      [this](const zenoh::Query& query)
      {
        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        if (!ctrl_ptr)
        {
          auto resp = build_response_resume("", "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        // Acted first
        ctrl_ptr->set_is_pause(false);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveResume>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        std::vector<uint8_t> resp;

        if (id == "")
        {
          resp = build_response_resume("", "accept", "abnormal ID detected");
        }
        else
        {
          resp = build_response_resume(id, "accept", "");
        }
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      },
      zenoh::closures::none
    );

    // ---- move/xLinear ----
    auto q_xlinear = session.declare_queryable(
      zenoh::KeyExpr(topic_xlinear),
      [this, &send_result](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_xlinear("", 0, 0, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_xlinear("", 0, 0, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveXLinear>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        float target = req->target();
        float speed = req->speed();

        // Validate
        if (fabs(target) > 10.0f || fabs(speed) > 1.5f)
        {
          auto resp = build_response_xlinear(id, target, speed, "reject", "value out of range");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        MOBILE* mobile_ptr = get_mobile();

        if (!ctrl_ptr || !mobile_ptr)
        {
          auto resp = build_response_xlinear(id, target, speed, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto resp = build_response_xlinear(id, target, speed, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        ctrl_ptr->stop();
        ctrl_ptr->set_is_moving(true);
        mobile_ptr->move_linear_x(target, speed);

        double t = std::abs(target/(speed + 1e-06)) + 0.5;
        delayed_tasks.schedule(std::chrono::milliseconds(static_cast<int>(t*1000)),
        [this, send_result, id, target, speed]() {

          AUTOCONTROL* ctrl_ptr = get_autocontrol();
          MOBILE* mobile_ptr = get_mobile();
          if (!ctrl_ptr || !mobile_ptr)
          {
            send_result(build_profile_result(id, target, speed, "", "fail", "module not ready"));
            return;
          }
          ctrl_ptr->set_is_moving(false);
          // float res_linear_dist = mobile_ptr->get_res_linear_dist();
          // float res_linear_remain_dist = mobile_ptr->get_res_linear_remain_dist();

          send_result(build_profile_result(id, target, speed, "", "success", ""));
        });
      },
      zenoh::closures::none
    );

    // ---- move/yLinear ----
    auto q_ylinear = session.declare_queryable(
      zenoh::KeyExpr(topic_ylinear),
      [this, &send_result](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_ylinear("", 0, 0, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_ylinear("", 0, 0, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveYLinear>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        float target = req->target();
        float speed = req->speed();

        // Validate
        if (fabs(target) > 10.0f || fabs(speed) > 1.5f)
        {
          auto resp = build_response_ylinear(id, target, speed, "reject", "value out of range");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        MOBILE* mobile_ptr = get_mobile();

        if (!ctrl_ptr || !mobile_ptr)
        {
          auto resp = build_response_ylinear(id, target, speed, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto resp = build_response_ylinear(id, target, speed, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        ctrl_ptr->stop();
        ctrl_ptr->set_is_moving(true);
        mobile_ptr->move_linear_y(target, speed);

        double t = std::abs(target/(speed + 1e-06)) + 0.5;
        delayed_tasks.schedule(std::chrono::milliseconds(static_cast<int>(t*1000)),
        [this, send_result, id, target, speed]() {

          AUTOCONTROL* ctrl_ptr = get_autocontrol();
          MOBILE* mobile_ptr = get_mobile();
          if (!ctrl_ptr || !mobile_ptr)
          {
            send_result(build_profile_result(id, target, speed, "", "fail", "module not ready"));
            return;
          }
          ctrl_ptr->set_is_moving(false);
          // float res_linear_dist = mobile_ptr->get_res_linear_dist();
          // float res_linear_remain_dist = mobile_ptr->get_res_linear_remain_dist();

          send_result(build_profile_result(id, target, speed, "", "success", ""));
        });
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_ylinear);

    // ---- move/circular ----
    auto q_circular = session.declare_queryable(
      zenoh::KeyExpr(topic_circular),
      [this, &send_result](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_circular("", 0, 0, "", "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_circular("", 0, 0, "", "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveCircular>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        float target = req->target();
        float speed = req->speed();
        std::string direction = req->direction() ? req->direction()->str() : "";

        // Validate (target in degrees)
        if (fabs(target) > 360.0f || fabs(speed) > 60.0f)
        {
          auto resp = build_response_circular(id, target, speed, direction, "reject", "value out of range");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        MOBILE* mobile_ptr = get_mobile();

        if (!ctrl_ptr || !mobile_ptr)
        {
          auto resp = build_response_circular(id, target, speed, direction, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto resp = build_response_circular(id, target, speed, direction, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        int dir = -1;
        if(direction == "right") dir = 0;
        else if (direction == "left") dir = 1;

        ctrl_ptr->stop();
        ctrl_ptr->set_is_moving(true);
        mobile_ptr->move_circular(target * D2R, speed * D2R, dir);

        double t = std::abs(target/(speed + 1e-06)) + 1.0;
        delayed_tasks.schedule(std::chrono::milliseconds(static_cast<int>(t*1000)),
        [this, send_result, id, target, speed, &direction]() {

          AUTOCONTROL* ctrl_ptr = get_autocontrol();
          MOBILE* mobile_ptr = get_mobile();
          if (!ctrl_ptr || !mobile_ptr)
          {
            send_result(build_profile_result(id, target, speed, direction, "fail", "module not ready"));
            return;
          }
          ctrl_ptr->set_is_moving(false);
          // float res_linear_dist = mobile_ptr->get_res_linear_dist();
          // float res_linear_remain_dist = mobile_ptr->get_res_linear_remain_dist();

          send_result(build_profile_result(id, target, speed, direction, "success", ""));
        });
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_circular);

    // ---- move/rotate ----
    auto q_rotate = session.declare_queryable(
      zenoh::KeyExpr(topic_rotate),
      [this, &send_result](const zenoh::Query& query)
      {
        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_rotate("", 0, 0, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_rotate("", 0, 0, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        auto req = flatbuffers::GetRoot<SLAMNAV::RequestMoveRotate>(slice->data);

        std::string id = req->id() ? req->id()->str() : "";
        float target = req->target();
        float speed = req->speed();

        // Validate (target/speed in degrees)
        if (fabs(target) > 360.0f || fabs(speed) > 60.0f)
        {
          auto resp = build_response_rotate(id, target, speed, "reject", "value out of range");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        AUTOCONTROL* ctrl_ptr = get_autocontrol();
        MOBILE* mobile_ptr = get_mobile();

        if (!ctrl_ptr || !mobile_ptr)
        {
          auto resp = build_response_rotate(id, target, speed, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto resp = build_response_rotate(id, target, speed, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        ctrl_ptr->stop();
        ctrl_ptr->set_is_moving(true);
        mobile_ptr->move_rotate(target * D2R, speed * D2R);

        double t = std::abs(target/(speed + 1e-06)) + 0.5;
        delayed_tasks.schedule(std::chrono::milliseconds(static_cast<int>(t*1000)),
        [this, send_result, id, target, speed]() {

          AUTOCONTROL* ctrl_ptr = get_autocontrol();
          MOBILE* mobile_ptr = get_mobile();
          if (!ctrl_ptr || !mobile_ptr)
          {
            send_result(build_profile_result(id, target, speed, "", "fail", "module not ready"));
            return;
          }
          ctrl_ptr->set_is_moving(false);
          // float res_linear_dist = mobile_ptr->get_res_linear_dist();
          // float res_linear_remain_dist = mobile_ptr->get_res_linear_remain_dist();

          send_result(build_profile_result(id, target, speed, "", "success", ""));
        });
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_rotate);


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
