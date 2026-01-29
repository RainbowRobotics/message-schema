/**
 * @file comm_zenoh_localization.cpp
 * @brief Localization 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - localization/init, localization/autoinit, localization/semiautoinit
 *   - localization/randominit, localization/start, localization/stop
 *
 * Publisher:
 *   - localization/result
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "slamnav_localization_generated.h"

#include <QDebug>
#include <chrono>
#include <functional>

namespace
{
    const char* MODULE_NAME = "COMM_ZENOH_LOC";

    // semi_auto_init 스레드 관리용
    std::unique_ptr<std::thread> semi_auto_init_thread_;

    // =========================================================================
    // Helper: Localization_Result FlatBuffer
    // =========================================================================
    std::vector<uint8_t> build_localization_result(const QString& id,
                                                    const QString& result,
                                                    const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);

        auto fb_result = SLAMNAV::CreateLocalization_Result(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(fb_result);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // =========================================================================
    // Helper: Build Response FlatBuffers
    // =========================================================================

    // Response_Localization_Init
    std::vector<uint8_t> build_response_init(const QString& id,
                                              const SLAMNAV::LocalizationPose* pose,
                                              const QString& result,
                                              const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_Init(
            fbb,
            fbb.CreateString(id.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Localization_AutoInit
    std::vector<uint8_t> build_response_autoinit(const QString& id,
                                                  const SLAMNAV::LocalizationPose* pose,
                                                  const QString& result,
                                                  const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_AutoInit(
            fbb,
            fbb.CreateString(id.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Localization_SemiAutoInit
    std::vector<uint8_t> build_response_semiautoinit(const QString& id,
                                                      const SLAMNAV::LocalizationPose* pose,
                                                      const QString& result,
                                                      const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_SemiAutoInit(
            fbb,
            fbb.CreateString(id.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Localization_RandomInit
    std::vector<uint8_t> build_response_randominit(const QString& id,
                                                    const QString& random_seed,
                                                    const SLAMNAV::LocalizationPose* pose,
                                                    const QString& result,
                                                    const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_RandomInit(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(random_seed.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Localization_Start
    std::vector<uint8_t> build_response_start(const QString& id,
                                               const SLAMNAV::LocalizationPose* pose,
                                               const QString& result,
                                               const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_Start(
            fbb,
            fbb.CreateString(id.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Localization_Stop
    std::vector<uint8_t> build_response_stop(const QString& id,
                                              const SLAMNAV::LocalizationPose* pose,
                                              const QString& result,
                                              const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Localization_Stop(
            fbb,
            fbb.CreateString(id.toStdString()),
            pose,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
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
    qDebug() << "[" << MODULE_NAME << "] localization_loop started";

    // 1. robotType이 설정될 때까지 대기
    while (is_localization_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_localization_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] localization_loop aborted (not running)";
        return;
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        qDebug() << "[" << MODULE_NAME << "] localization_loop aborted (session invalid)";
        return;
    }

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

        qDebug() << "[" << MODULE_NAME << "] Registering topics with prefix:" << QString::fromStdString(get_robot_type());

        // 4. Result Publisher
        auto pub_result = session.declare_publisher(zenoh::KeyExpr(topic_result));

        // 5. RPC Queryables

        // ---- localization/init ----
        auto q_init = session.declare_queryable(
            zenoh::KeyExpr(topic_init),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_init("", &pose, "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Localization_Init(bytes.data());
                if (!req)
                {
                    auto resp = build_response_init("", &pose, "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");

                // Validate modules
                if (!unimap || !loc || !config || !lidar_2d || !lidar_3d)
                {
                    auto resp = build_response_init(id, &pose, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check map loaded
                if (unimap->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_init(id, &pose, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check sensor connected
                QString loc_mode = config->get_loc_mode();
                if (loc_mode == "2D" && !lidar_2d->get_is_connected())
                {
                    auto resp = build_response_init(id, &pose, "reject", "lidar 2d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }
                else if (loc_mode == "3D" && !lidar_3d->get_is_connected())
                {
                    auto resp = build_response_init(id, &pose, "reject", "lidar 3d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Get pose from request
                auto req_pose = req->pose();
                float x = req_pose ? req_pose->x() : 0.0f;
                float y = req_pose ? req_pose->y() : 0.0f;
                float z = req_pose ? req_pose->z() : 0.0f;
                float rz = req_pose ? req_pose->rz() : 0.0f;

                // Set pose
                loc->stop();
                Eigen::Matrix4d tf = se2_to_TF(Eigen::Vector3d(x, y, rz * D2R));
                tf(2, 3) = z;
                loc->set_cur_tf(tf);

                pose = SLAMNAV::LocalizationPose(x, y, z, rz);

                // Response
                auto resp = build_response_init(id, &pose, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Publish result
                auto result_buf = build_localization_result(id, "success", "init accepted");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_init);

        // ---- localization/autoinit ----
        auto q_autoinit = session.declare_queryable(
            zenoh::KeyExpr(topic_autoinit),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Localization_AutoInit(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Validate modules
                if (!unimap || !loc || !config || !lidar_2d || !lidar_3d)
                {
                    auto resp = build_response_autoinit(id, &pose, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check map loaded
                if (unimap->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_autoinit(id, &pose, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check sensor connected
                QString loc_mode = config->get_loc_mode();
                if (loc_mode == "2D" && !lidar_2d->get_is_connected())
                {
                    auto resp = build_response_autoinit(id, &pose, "reject", "lidar 2d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }
                else if (loc_mode == "3D" && !lidar_3d->get_is_connected())
                {
                    auto resp = build_response_autoinit(id, &pose, "reject", "lidar 3d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check busy
                if (loc->get_is_busy())
                {
                    auto resp = build_response_autoinit(id, &pose, "reject", "localization busy");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Start auto init (same as semiautoinit)
                loc->stop();

                if (semi_auto_init_thread_)
                {
                    if (semi_auto_init_thread_->joinable())
                    {
                        semi_auto_init_thread_->join();
                    }
                    semi_auto_init_thread_.reset();
                }
                semi_auto_init_thread_ = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);

                pose = get_current_pose(loc);

                // Response
                auto resp = build_response_autoinit(id, &pose, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Publish result
                auto result_buf = build_localization_result(id, "success", "autoinit started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_autoinit);

        // ---- localization/semiautoinit ----
        auto q_semiautoinit = session.declare_queryable(
            zenoh::KeyExpr(topic_semiautoinit),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Localization_SemiAutoInit(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Validate modules
                if (!unimap || !loc || !config || !lidar_2d || !lidar_3d)
                {
                    auto resp = build_response_semiautoinit(id, &pose, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check map loaded
                if (unimap->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_semiautoinit(id, &pose, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check sensor connected
                QString loc_mode = config->get_loc_mode();
                if (loc_mode == "2D" && !lidar_2d->get_is_connected())
                {
                    auto resp = build_response_semiautoinit(id, &pose, "reject", "lidar 2d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }
                else if (loc_mode == "3D" && !lidar_3d->get_is_connected())
                {
                    auto resp = build_response_semiautoinit(id, &pose, "reject", "lidar 3d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check busy
                if (loc->get_is_busy())
                {
                    auto resp = build_response_semiautoinit(id, &pose, "reject", "localization busy");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Start semi-auto init
                loc->stop();

                if (semi_auto_init_thread_)
                {
                    if (semi_auto_init_thread_->joinable())
                    {
                        semi_auto_init_thread_->join();
                    }
                    semi_auto_init_thread_.reset();
                }
                semi_auto_init_thread_ = std::make_unique<std::thread>(&LOCALIZATION::start_semiauto_init, loc);

                pose = get_current_pose(loc);

                // Response
                auto resp = build_response_semiautoinit(id, &pose, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Publish result
                auto result_buf = build_localization_result(id, "success", "semiautoinit started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_semiautoinit);

        // ---- localization/randominit ----
        auto q_randominit = session.declare_queryable(
            zenoh::KeyExpr(topic_randominit),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                QString id = "";
                QString random_seed = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Localization_RandomInit(bytes.data());
                    if (req)
                    {
                        if (req->id())
                        {
                            id = QString::fromStdString(req->id()->str());
                        }
                        if (req->random_seed())
                        {
                            random_seed = QString::fromStdString(req->random_seed()->str());
                        }
                    }
                }

                // Validate modules
                if (!unimap || !loc || !config || !lidar_2d || !lidar_3d)
                {
                    auto resp = build_response_randominit(id, random_seed, &pose, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check map loaded
                if (unimap->get_is_loaded() != MAP_LOADED)
                {
                    auto resp = build_response_randominit(id, random_seed, &pose, "reject", "map not loaded");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check sensor connected
                QString loc_mode = config->get_loc_mode();
                if (loc_mode == "2D" && !lidar_2d->get_is_connected())
                {
                    auto resp = build_response_randominit(id, random_seed, &pose, "reject", "lidar 2d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }
                else if (loc_mode == "3D" && !lidar_3d->get_is_connected())
                {
                    auto resp = build_response_randominit(id, random_seed, &pose, "reject", "lidar 3d not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // TODO: Implement random init logic with random_seed
                // For now, reject as not implemented
                auto resp = build_response_randominit(id, random_seed, &pose, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_randominit);

        // ---- localization/start ----
        auto q_start = session.declare_queryable(
            zenoh::KeyExpr(topic_start),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Localization_Start(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Validate modules
                if (!loc)
                {
                    auto resp = build_response_start(id, &pose, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Get current pose for validation
                pose = get_current_pose(loc);
                float x = pose.x();
                float y = pose.y();
                float rz = pose.rz();

                if (std::isnan(x) || std::isnan(y) || std::isnan(rz))
                {
                    auto resp = build_response_start(id, &pose, "reject", "invalid pose");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Start localization
                loc->stop();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                loc->start();

                pose = get_current_pose(loc);

                // Response
                auto resp = build_response_start(id, &pose, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Publish result
                auto result_buf = build_localization_result(id, "success", "localization started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_start);

        // ---- localization/stop ----
        auto q_stop = session.declare_queryable(
            zenoh::KeyExpr(topic_stop),
            [this, &pub_result](const zenoh::Query& query)
            {
                SLAMNAV::LocalizationPose pose(0, 0, 0, 0);

                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Localization_Stop(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Stop localization
                if (loc)
                {
                    loc->stop();
                    pose = get_current_pose(loc);
                }

                // Response
                auto resp = build_response_stop(id, &pose, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                // Publish result
                auto result_buf = build_localization_result(id, "success", "localization stopped");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_stop);

        // 6. Main loop - keep alive
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

        qDebug() << "[" << MODULE_NAME << "] localization_loop ending, resources will be released";
    }
    catch (const zenoh::ZException& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Zenoh exception:" << e.what();
    }
    catch (const std::exception& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Exception:" << e.what();
    }

    qDebug() << "[" << MODULE_NAME << "] localization_loop ended";
}
