/**
 * @file comm_zenoh_localization.cpp
 * @brief Localization 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - localization/init, localization/autoInit, localization/semiAutoInit
 *   - localization/randomInit, localization/start, localization/stop
 *
 * Publisher:
 *   - localization/result/init
 */

#include "comm_zenoh.h"
#include "flatbuffer/generated/slamnav_localization_generated.h"

namespace
{
  constexpr const char* MODULE_NAME = "ZENOH_LOC";

  // semi_auto_init 스레드 관리용
  std::unique_ptr<std::thread> semi_auto_init_thread_;

  std::vector<uint8_t> build_result_init(const std::string& id,
                                          const SLAMNAV::LocalizationPose* pose,
                                          const std::string& result,
                                          const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);

    auto fb_result = SLAMNAV::CreateResultLocalizationInit(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(fb_result);

    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // =========================================================================
  // Helper: Build Response FlatBuffers (Queryable용)
  // =========================================================================

  // ResponseLocalizationInit
  std::vector<uint8_t> build_response_init(const std::string& id,
                                            const SLAMNAV::LocalizationPose* pose,
                                            const std::string& result,
                                            const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationInit(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseLocalizationAutoInit
  std::vector<uint8_t> build_response_autoinit(const std::string& id,
                                                const SLAMNAV::LocalizationPose* pose,
                                                const std::string& result,
                                                const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationAutoInit(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseLocalizationSemiAutoInit
  std::vector<uint8_t> build_response_semiautoinit(const std::string& id,
                                                    const SLAMNAV::LocalizationPose* pose,
                                                    const std::string& result,
                                                    const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationSemiAutoInit(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseLocalizationRandomInit
  std::vector<uint8_t> build_response_randominit(const std::string& id,
                                                  const SLAMNAV::LocalizationPose* pose,
                                                  const std::string& result,
                                                  const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationRandomInit(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseLocalizationStart
  std::vector<uint8_t> build_response_start(const std::string& id,
                                             const SLAMNAV::LocalizationPose* pose,
                                             const std::string& result,
                                             const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationStart(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // ResponseLocalizationStop
  std::vector<uint8_t> build_response_stop(const std::string& id,
                                            const SLAMNAV::LocalizationPose* pose,
                                            const std::string& result,
                                            const std::string& message)
  {
    flatbuffers::FlatBufferBuilder fbb(256);
    auto resp = SLAMNAV::CreateResponseLocalizationStop(
      fbb,
      fbb.CreateString(id),
      pose,
      fbb.CreateString(result),
      fbb.CreateString(message)
    );
    fbb.Finish(resp);
    const uint8_t* buf = fbb.GetBufferPointer();
    return std::vector<uint8_t>(buf, buf + fbb.GetSize());
  }

  // Helper: Get current pose from localization module
  SLAMNAV::LocalizationPose get_current_pose(LOCALIZATION* loc)
  {
    if (!loc)
    {
      return SLAMNAV::LocalizationPose(0, 0, 0, 0);
    }

    Eigen::Matrix4d tf = loc->get_cur_tf();
    Eigen::Vector3d xi = TF_to_se2(tf);
    float x = static_cast<float>(xi[0]);
    float y = static_cast<float>(xi[1]);
    float z = static_cast<float>(tf(2, 3));
    float rz = static_cast<float>(xi[2] * R2D);

    return SLAMNAV::LocalizationPose(x, y, z, rz);
  }

} // anonymous namespace

// =============================================================================
// COMM_ZENOH::localization_loop()
// =============================================================================
void COMM_ZENOH::localization_loop()
{
  log_info("localization_loop started");

  // 1. robotType이 설정될 때까지 대기
  while (is_localization_running_.load() && get_robot_type().empty())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // 2. Session 유효성 확인
  if (!is_session_valid())
  {
    log_error("localization_loop aborted: session invalid");
    return;
  }

  log_info("localization_loop initialized with robotType: {}", get_robot_type());

  try
  {
    zenoh::Session& session = get_session();

    // 3. Topic 생성
    std::string topic_init         = make_topic(ZENOH_TOPIC::LOC_INIT);
    std::string topic_autoinit     = make_topic(ZENOH_TOPIC::LOC_AUTOINIT);
    std::string topic_semiautoinit = make_topic(ZENOH_TOPIC::LOC_SEMIAUTOINIT);
    std::string topic_randominit   = make_topic(ZENOH_TOPIC::LOC_RANDOMINIT);
    std::string topic_start        = make_topic(ZENOH_TOPIC::LOC_START);
    std::string topic_stop         = make_topic(ZENOH_TOPIC::LOC_STOP);
    std::string topic_result       = make_topic(ZENOH_TOPIC::LOC_RESULT);

    log_info("localization_loop registering topics with prefix: {}", get_robot_type());

    // 4. Result 발행 람다
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

    // ---- localization/init ----
    auto q_init = session.declare_queryable(
      zenoh::KeyExpr(topic_init),
      [this, &send_result](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        if (!payload.has_value())
        {
          auto resp = build_response_init("", &pose, "reject", "no payload");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto iter = payload->get().slice_iter();
        auto slice = iter.next();
        if (!slice.has_value())
        {
          auto resp = build_response_init("", &pose, "reject", "invalid slice, no data");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationInit>(slice->data);
        if (!req)
        {
          auto resp = build_response_init("", &pose, "reject", "invalid request");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        std::string id = req->id() ? req->id()->str() : "";

        // Validate modules
        UNIMAP* unimap_ptr = get_unimap();
        LOCALIZATION* loc_ptr = get_localization();
        CONFIG* config_ptr = get_config();
        LIDAR_2D* lidar_2d_ptr = get_lidar_2d();
        LIDAR_3D* lidar_3d_ptr = get_lidar_3d();

        if (!unimap_ptr || !loc_ptr || !config_ptr || !lidar_2d_ptr || !lidar_3d_ptr)
        {
          auto resp = build_response_init(id, &pose, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check map loaded
        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_init(id, &pose, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check sensor connected
        std::string loc_mode = config_ptr->get_loc_mode().toStdString();
        if (loc_mode == "2D" && !lidar_2d_ptr->get_is_connected())
        {
          auto resp = build_response_init(id, &pose, "reject", "lidar 2d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        else if (loc_mode == "3D" && !lidar_3d_ptr->get_is_connected())
        {
          auto resp = build_response_init(id, &pose, "reject", "lidar 3d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Get pose from request
        auto req_pose = req->pose();
        float x = req_pose ? req_pose->x() : 0.0f;
        float y = req_pose ? req_pose->y() : 0.0f;
        float z = req_pose ? req_pose->z() : 0.0f;
        float rz = req_pose ? req_pose->rz() : 0.0f;

        // Set pose
        loc_ptr->stop();
        Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));
        tf(2, 3) = z;
        loc_ptr->set_cur_tf(tf);

        pose = SLAMNAV::LocalizationPose(x, y, z, rz);

        // Response
        auto resp = build_response_init(id, &pose, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        // Publish result
        send_result(build_result_init(id, &pose, "success", "init accepted"));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_init);

    // ---- localization/autoInit ----
    auto q_autoinit = session.declare_queryable(
      zenoh::KeyExpr(topic_autoinit),
      [this, &send_result](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationAutoInit>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        // Validate modules
        UNIMAP* unimap_ptr = get_unimap();
        LOCALIZATION* loc_ptr = get_localization();
        CONFIG* config_ptr = get_config();
        LIDAR_2D* lidar_2d_ptr = get_lidar_2d();
        LIDAR_3D* lidar_3d_ptr = get_lidar_3d();

        if (!unimap_ptr || !loc_ptr || !config_ptr || !lidar_2d_ptr || !lidar_3d_ptr)
        {
          auto resp = build_response_autoinit(id, &pose, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check map loaded
        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_autoinit(id, &pose, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check sensor connected
        std::string loc_mode = config_ptr->get_loc_mode().toStdString();
        if (loc_mode == "2D" && !lidar_2d_ptr->get_is_connected())
        {
          auto resp = build_response_autoinit(id, &pose, "reject", "lidar 2d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        else if (loc_mode == "3D" && !lidar_3d_ptr->get_is_connected())
        {
          auto resp = build_response_autoinit(id, &pose, "reject", "lidar 3d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check busy
        if (loc_ptr->get_is_busy())
        {
          auto resp = build_response_autoinit(id, &pose, "reject", "localization busy");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Start auto init
        loc_ptr->stop();

        if (semi_auto_init_thread_)
        {
          if (semi_auto_init_thread_->joinable())
          {
            semi_auto_init_thread_->join();
          }
          semi_auto_init_thread_.reset();
        }
        semi_auto_init_thread_ = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc_ptr);

        pose = get_current_pose(loc_ptr);

        // Response
        auto resp = build_response_autoinit(id, &pose, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        // Publish result
        send_result(build_result_init(id, &pose, "success", "autoinit started"));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_autoinit);

    // ---- localization/semiAutoInit ----
    auto q_semiautoinit = session.declare_queryable(
      zenoh::KeyExpr(topic_semiautoinit),
      [this, &send_result](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationSemiAutoInit>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        // Validate modules
        UNIMAP* unimap_ptr = get_unimap();
        LOCALIZATION* loc_ptr = get_localization();
        CONFIG* config_ptr = get_config();
        LIDAR_2D* lidar_2d_ptr = get_lidar_2d();
        LIDAR_3D* lidar_3d_ptr = get_lidar_3d();

        if (!unimap_ptr || !loc_ptr || !config_ptr || !lidar_2d_ptr || !lidar_3d_ptr)
        {
          auto resp = build_response_semiautoinit(id, &pose, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check map loaded
        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_semiautoinit(id, &pose, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check sensor connected
        std::string loc_mode = config_ptr->get_loc_mode().toStdString();
        if (loc_mode == "2D" && !lidar_2d_ptr->get_is_connected())
        {
          auto resp = build_response_semiautoinit(id, &pose, "reject", "lidar 2d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        else if (loc_mode == "3D" && !lidar_3d_ptr->get_is_connected())
        {
          auto resp = build_response_semiautoinit(id, &pose, "reject", "lidar 3d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check busy
        if (loc_ptr->get_is_busy())
        {
          auto resp = build_response_semiautoinit(id, &pose, "reject", "localization busy");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Start semi-auto init
        loc_ptr->stop();

        if (semi_auto_init_thread_)
        {
          if (semi_auto_init_thread_->joinable())
          {
            semi_auto_init_thread_->join();
          }
          semi_auto_init_thread_.reset();
        }
        semi_auto_init_thread_ = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc_ptr);

        pose = get_current_pose(loc_ptr);

        // Response
        auto resp = build_response_semiautoinit(id, &pose, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        // Publish result
        send_result(build_result_init(id, &pose, "success", "semiautoinit started"));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_semiautoinit);

    // ---- localization/randomInit ----
    auto q_randominit = session.declare_queryable(
      zenoh::KeyExpr(topic_randominit),
      [this](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationRandomInit>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        // Validate modules
        UNIMAP* unimap_ptr = get_unimap();
        LOCALIZATION* loc_ptr = get_localization();
        CONFIG* config_ptr = get_config();
        LIDAR_2D* lidar_2d_ptr = get_lidar_2d();
        LIDAR_3D* lidar_3d_ptr = get_lidar_3d();

        if (!unimap_ptr || !loc_ptr || !config_ptr || !lidar_2d_ptr || !lidar_3d_ptr)
        {
          auto resp = build_response_randominit(id, &pose, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check map loaded
        if (unimap_ptr->get_is_loaded() != MAP_LOADED)
        {
          auto resp = build_response_randominit(id, &pose, "reject", "map not loaded");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Check sensor connected
        std::string loc_mode = config_ptr->get_loc_mode().toStdString();
        if (loc_mode == "2D" && !lidar_2d_ptr->get_is_connected())
        {
          auto resp = build_response_randominit(id, &pose, "reject", "lidar 2d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }
        else if (loc_mode == "3D" && !lidar_3d_ptr->get_is_connected())
        {
          auto resp = build_response_randominit(id, &pose, "reject", "lidar 3d not connected");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // TODO: Implement random init logic
        auto resp = build_response_randominit(id, &pose, "reject", "not implemented");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_randominit);

    // ---- localization/start ----
    auto q_start = session.declare_queryable(
      zenoh::KeyExpr(topic_start),
      [this, &send_result](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationStart>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        // Validate modules
        LOCALIZATION* loc_ptr = get_localization();
        if (!loc_ptr)
        {
          auto resp = build_response_start(id, &pose, "reject", "module not ready");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Get current pose for validation
        pose = get_current_pose(loc_ptr);
        float x = pose.x();
        float y = pose.y();
        float rz = pose.rz();

        if (std::isnan(x) || std::isnan(y) || std::isnan(rz))
        {
          auto resp = build_response_start(id, &pose, "reject", "invalid pose");
          query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));
          return;
        }

        // Start localization
        loc_ptr->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        loc_ptr->start();

        pose = get_current_pose(loc_ptr);

        // Response
        auto resp = build_response_start(id, &pose, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        // Publish result
        send_result(build_result_init(id, &pose, "success", "localization started"));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_start);

    // ---- localization/stop ----
    auto q_stop = session.declare_queryable(
      zenoh::KeyExpr(topic_stop),
      [this, &send_result](const zenoh::Query& query)
      {
        SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

        const auto& payload = query.get_payload();
        std::string id;

        if (payload.has_value())
        {
          auto iter = payload->get().slice_iter();
          auto slice = iter.next();
          if (slice.has_value())
          {
            auto req = flatbuffers::GetRoot<SLAMNAV::RequestLocalizationStop>(slice->data);
            if (req && req->id())
            {
              id = req->id()->str();
            }
          }
        }

        // Stop localization
        LOCALIZATION* loc_ptr = get_localization();
        if (loc_ptr)
        {
          loc_ptr->stop();
          pose = get_current_pose(loc_ptr);
        }

        // Response
        auto resp = build_response_stop(id, &pose, "accept", "");
        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes(std::move(resp)));

        // Publish result
        send_result(build_result_init(id, &pose, "success", "localization stopped"));
      },
      zenoh::closures::none
    );
    log_info("Queryable registered: {}", topic_stop);

    // Main loop - keep alive
    while (is_localization_running_.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup semi_auto_init_thread
    if (semi_auto_init_thread_)
    {
      if (semi_auto_init_thread_->joinable())
      {
        semi_auto_init_thread_->join();
      }
      semi_auto_init_thread_.reset();
    }

    log_info("localization_loop ending, resources will be released");
  }
  catch (const zenoh::ZException& e)
  {
    log_error("localization_loop Zenoh exception: {}", e.what());
  }
  catch (const std::exception& e)
  {
    log_error("localization_loop exception: {}", e.what());
  }

  log_info("localization_loop ended");
}
