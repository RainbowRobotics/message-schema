/**
 * @file comm_zenoh_map.cpp
 * @brief Map 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - map/getList, map/getCurrent, map/load, map/delete
 *   - map/getFile, map/getCloud, map/setCloud
 *   - map/getTopology, map/setTopology
 *   - map/mapping/start, map/mapping/stop, map/mapping/save
 *
 * Publisher:
 *   - map/result
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "slamnav_map_generated.h"

#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <chrono>
#include <functional>

namespace
{
    const char* MODULE_NAME = "COMM_ZENOH_MAP";

    // =========================================================================
    // Helper: Map_Result FlatBuffer
    // =========================================================================
    std::vector<uint8_t> build_map_result(const QString& id,
                                          const QString& result,
                                          const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);

        auto fb_result = SLAMNAV::CreateMap_Result(
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

    // Response_Map_Current
    std::vector<uint8_t> build_response_current(const QString& id,
                                                 const QString& map_name,
                                                 const QString& result,
                                                 const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Map_Current(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(map_name.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Map_Load
    std::vector<uint8_t> build_response_load(const QString& id,
                                              const QString& map_name,
                                              const QString& result,
                                              const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Map_Load(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(map_name.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Map_Delete
    std::vector<uint8_t> build_response_delete(const QString& id,
                                                const QString& map_name,
                                                const QString& result,
                                                const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Map_Delete(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(map_name.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Map_Cloud
    std::vector<uint8_t> build_response_set_cloud(const QString& id,
                                                   const QString& map_name,
                                                   const QString& file_name,
                                                   uint32_t size,
                                                   const QString& result,
                                                   const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Set_Map_Cloud(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(map_name.toStdString()),
            fbb.CreateString(file_name.toStdString()),
            size,
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Mapping_Start
    std::vector<uint8_t> build_response_mapping_start(const QString& id,
                                                       const QString& result,
                                                       const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Mapping_Start(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Mapping_Stop
    std::vector<uint8_t> build_response_mapping_stop(const QString& id,
                                                      const QString& result,
                                                      const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Mapping_Stop(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Mapping_Save
    std::vector<uint8_t> build_response_mapping_save(const QString& id,
                                                      const QString& map_name,
                                                      const QString& result,
                                                      const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);
        auto resp = SLAMNAV::CreateResponse_Mapping_Save(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(map_name.toStdString()),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);
        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Helper: Get map list from directory
    QStringList get_map_directories()
    {
        QString map_base_path = "/data/maps";
        QDir dir(map_base_path);
        if (!dir.exists())
        {
            return QStringList();
        }
        return dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    }

} // anonymous namespace

// =============================================================================
// COMM_ZENOH::map_loop()
// =============================================================================
void COMM_ZENOH::map_loop()
{
    qDebug() << "[" << MODULE_NAME << "] map_loop started";

    // 1. robotType이 설정될 때까지 대기
    while (is_map_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_map_running_.load())
    {
        qDebug() << "[" << MODULE_NAME << "] map_loop aborted (not running)";
        return;
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        qDebug() << "[" << MODULE_NAME << "] map_loop aborted (session invalid)";
        return;
    }

    try
    {
        zenoh::Session& session = get_session();

        // 3. Topic 생성
        std::string topic_getlist     = make_topic(ZENOH_TOPIC::MAP_GET_LIST);
        std::string topic_getcurrent  = make_topic(ZENOH_TOPIC::MAP_GET_CURRENT);
        std::string topic_load        = make_topic(ZENOH_TOPIC::MAP_LOAD);
        std::string topic_delete      = make_topic(ZENOH_TOPIC::MAP_DELETE);
        std::string topic_getfile     = make_topic(ZENOH_TOPIC::MAP_GET_FILE);
        std::string topic_getcloud    = make_topic(ZENOH_TOPIC::MAP_GET_CLOUD);
        std::string topic_setcloud    = make_topic(ZENOH_TOPIC::MAP_SET_CLOUD);
        std::string topic_gettopo     = make_topic(ZENOH_TOPIC::MAP_GET_TOPOLOGY);
        std::string topic_settopo     = make_topic(ZENOH_TOPIC::MAP_SET_TOPOLOGY);
        std::string topic_map_start   = make_topic(ZENOH_TOPIC::MAP_MAPPING_START);
        std::string topic_map_stop    = make_topic(ZENOH_TOPIC::MAP_MAPPING_STOP);
        std::string topic_map_save    = make_topic(ZENOH_TOPIC::MAP_MAPPING_SAVE);
        std::string topic_result      = make_topic(ZENOH_TOPIC::MAP_RESULT);

        qDebug() << "[" << MODULE_NAME << "] Registering topics with prefix:" << QString::fromStdString(get_robot_type());

        // 4. Result Publisher
        auto pub_result = session.declare_publisher(zenoh::KeyExpr(topic_result));

        // 5. RPC Queryables

        // ---- map/getList ----
        auto q_getlist = session.declare_queryable(
            zenoh::KeyExpr(topic_getlist),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Map_List(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                flatbuffers::FlatBufferBuilder fbb(4096);
                std::vector<flatbuffers::Offset<SLAMNAV::MapFile>> map_list;

                // Get map directories
                QStringList maps = get_map_directories();
                QString map_base_path = "/data/maps";

                for (const QString& map_name : maps)
                {
                    QString map_path = map_base_path + "/" + map_name;
                    QFileInfo dir_info(map_path);

                    auto file_name = fbb.CreateString(map_name.toStdString());
                    auto created_at = fbb.CreateString(dir_info.birthTime().toString(Qt::ISODate).toStdString());
                    auto update_at = fbb.CreateString(dir_info.lastModified().toString(Qt::ISODate).toStdString());
                    auto file_type = fbb.CreateString("map");

                    auto map_file = SLAMNAV::CreateMapFile(fbb, file_name, created_at, update_at, file_type);
                    map_list.push_back(map_file);
                }

                auto resp = SLAMNAV::CreateResponse_Map_List(
                    fbb,
                    fbb.CreateString(id.toStdString()),
                    fbb.CreateVector(map_list),
                    fbb.CreateString("accept"),
                    fbb.CreateString("")
                );
                fbb.Finish(resp);
                const uint8_t* buf = fbb.GetBufferPointer();
                std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));

                auto result_buf = build_map_result(id, "success", "list retrieved");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_getlist);

        // ---- map/getCurrent ----
        auto q_getcurrent = session.declare_queryable(
            zenoh::KeyExpr(topic_getcurrent),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Map_Current(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                QString map_name = "";
                if (unimap && unimap->get_is_loaded() == MAP_LOADED)
                {
                    QString map_path = unimap->get_map_path();
                    QDir dir(map_path);
                    map_name = dir.dirName();
                }

                auto resp = build_response_current(id, map_name, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_getcurrent);

        // ---- map/load ----
        auto q_load = session.declare_queryable(
            zenoh::KeyExpr(topic_load),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_load("", "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Map_Load(bytes.data());
                if (!req)
                {
                    auto resp = build_response_load("", "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString map_name = QString::fromStdString(req->map_name() ? req->map_name()->str() : "");

                if (!unimap || !loc || !obsmap || !config)
                {
                    auto resp = build_response_load(id, map_name, "reject", "module not ready");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                if (map_name.isEmpty())
                {
                    auto resp = build_response_load(id, map_name, "reject", "empty map name");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Construct map path
                QString map_path = "/data/maps/" + map_name;
                if (!QDir(map_path).exists())
                {
                    auto resp = build_response_load(id, map_name, "reject", "map not found");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Validate map files
                QString check_msg = unimap->is_load_map_check(map_path);
                if (check_msg == "no 2d map!")
                {
                    auto resp = build_response_load(id, map_name, "reject", "no 2d map");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }
                if (config->get_use_lidar_3d() && check_msg == "no 3d map!")
                {
                    auto resp = build_response_load(id, map_name, "reject", "no 3d map");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Stop localization and clear obsmap before loading
                loc->stop();
                obsmap->clear();
                config->set_map_path(map_path);

                // Load map
                unimap->load_map(map_path);

                auto resp = build_response_load(id, map_name, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_map_result(id, "success", "map loading");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_load);

        // ---- map/delete ----
        auto q_delete = session.declare_queryable(
            zenoh::KeyExpr(topic_delete),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_delete("", "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Map_Delete(bytes.data());
                if (!req)
                {
                    auto resp = build_response_delete("", "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString map_name = QString::fromStdString(req->map_name() ? req->map_name()->str() : "");

                QString map_path = "/data/maps/" + map_name;
                QDir dir(map_path);

                if (!dir.exists())
                {
                    auto resp = build_response_delete(id, map_name, "reject", "map not found");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Check if this map is currently loaded
                if (unimap && unimap->get_is_loaded() == MAP_LOADED)
                {
                    QString cur_map_path = unimap->get_map_path();
                    if (cur_map_path == map_path)
                    {
                        auto resp = build_response_delete(id, map_name, "reject", "cannot delete loaded map");
                        query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                        return;
                    }
                }

                // Delete the map directory
                if (!dir.removeRecursively())
                {
                    auto resp = build_response_delete(id, map_name, "reject", "delete failed");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto resp = build_response_delete(id, map_name, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_map_result(id, "success", "map deleted");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_delete);

        // ---- map/getFile ----
        auto q_getfile = session.declare_queryable(
            zenoh::KeyExpr(topic_getfile),
            [this](const zenoh::Query& query)
            {
                // TODO: Implement file retrieval
                flatbuffers::FlatBufferBuilder fbb(256);
                auto resp = SLAMNAV::CreateResponse_Get_Map_FileDirect(
                    fbb, "", "", "", 0, nullptr, "reject", "not implemented"
                );
                fbb.Finish(resp);
                const uint8_t* buf = fbb.GetBufferPointer();
                std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_getfile);

        // ---- map/getCloud ----
        auto q_getcloud = session.declare_queryable(
            zenoh::KeyExpr(topic_getcloud),
            [this](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";
                QString map_name = "";
                QString file_name = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Get_Map_Cloud(bytes.data());
                    if (req)
                    {
                        if (req->id()) id = QString::fromStdString(req->id()->str());
                        if (req->map_name()) map_name = QString::fromStdString(req->map_name()->str());
                        if (req->file_name()) file_name = QString::fromStdString(req->file_name()->str());
                    }
                }

                if (!unimap || unimap->get_is_loaded() != MAP_LOADED)
                {
                    flatbuffers::FlatBufferBuilder fbb(256);
                    auto resp = SLAMNAV::CreateResponse_Get_Map_CloudDirect(
                        fbb, id.toStdString().c_str(), map_name.toStdString().c_str(),
                        file_name.toStdString().c_str(), 0, nullptr, "reject", "map not loaded"
                    );
                    fbb.Finish(resp);
                    const uint8_t* buf = fbb.GetBufferPointer();
                    std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));
                    return;
                }

                // Get cloud data from unimap
                auto cloud_ptr = unimap->get_kdtree_cloud();
                std::vector<SLAMNAV::CloudData> cloud_data;

                if (cloud_ptr)
                {
                    for (const auto& pt : *cloud_ptr)
                    {
                        cloud_data.emplace_back(
                            static_cast<float>(pt.x),
                            static_cast<float>(pt.y),
                            static_cast<float>(pt.z),
                            static_cast<float>(pt.range)
                        );
                    }
                }

                flatbuffers::FlatBufferBuilder fbb(1024 * 1024);  // Large buffer for cloud data
                auto resp = SLAMNAV::CreateResponse_Get_Map_Cloud(
                    fbb,
                    fbb.CreateString(id.toStdString()),
                    fbb.CreateString(map_name.toStdString()),
                    fbb.CreateString(file_name.toStdString()),
                    static_cast<uint32_t>(cloud_data.size()),
                    fbb.CreateVectorOfStructs(cloud_data),
                    fbb.CreateString("accept"),
                    fbb.CreateString("")
                );
                fbb.Finish(resp);
                const uint8_t* buf = fbb.GetBufferPointer();
                std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_getcloud);

        // ---- map/setCloud ----
        auto q_setcloud = session.declare_queryable(
            zenoh::KeyExpr(topic_setcloud),
            [this](const zenoh::Query& query)
            {
                // TODO: Implement cloud data setting
                auto resp = build_response_set_cloud("", "", "", 0, "reject", "not implemented");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_setcloud);

        // ---- map/getTopology ----
        auto q_gettopo = session.declare_queryable(
            zenoh::KeyExpr(topic_gettopo),
            [this](const zenoh::Query& query)
            {
                // TODO: Implement topology retrieval with nodes and links
                flatbuffers::FlatBufferBuilder fbb(256);
                auto resp = SLAMNAV::CreateResponse_Get_Map_TopologyDirect(
                    fbb, "", "", nullptr, "reject", "not implemented"
                );
                fbb.Finish(resp);
                const uint8_t* buf = fbb.GetBufferPointer();
                std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_gettopo);

        // ---- map/setTopology ----
        auto q_settopo = session.declare_queryable(
            zenoh::KeyExpr(topic_settopo),
            [this](const zenoh::Query& query)
            {
                // TODO: Implement topology setting
                flatbuffers::FlatBufferBuilder fbb(256);
                auto resp = SLAMNAV::CreateResponse_Set_Map_TopologyDirect(
                    fbb, "", "", "reject", "not implemented"
                );
                fbb.Finish(resp);
                const uint8_t* buf = fbb.GetBufferPointer();
                std::vector<uint8_t> data(buf, buf + fbb.GetSize());
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(data));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_settopo);

        // ---- map/mapping/start ----
        auto q_map_start = session.declare_queryable(
            zenoh::KeyExpr(topic_map_start),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Mapping_Start(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Check lidar connection
                if (!lidar_2d || !lidar_2d->get_is_connected())
                {
                    auto resp = build_response_mapping_start(id, "reject", "lidar not connected");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Emit signal to start mapping
                Q_EMIT signal_map_build_start();

                auto resp = build_response_mapping_start(id, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_map_result(id, "success", "mapping started");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_map_start);

        // ---- map/mapping/stop ----
        auto q_map_stop = session.declare_queryable(
            zenoh::KeyExpr(topic_map_stop),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                QString id = "";

                if (payload.has_value())
                {
                    auto bytes = payload->as_vector();
                    auto req = SLAMNAV::GetRequest_Mapping_Stop(bytes.data());
                    if (req && req->id())
                    {
                        id = QString::fromStdString(req->id()->str());
                    }
                }

                // Emit signal to stop mapping
                Q_EMIT signal_map_build_stop();

                auto resp = build_response_mapping_stop(id, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_map_result(id, "success", "mapping stopped");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_map_stop);

        // ---- map/mapping/save ----
        auto q_map_save = session.declare_queryable(
            zenoh::KeyExpr(topic_map_save),
            [this, &pub_result](const zenoh::Query& query)
            {
                const auto& payload = query.get_payload();
                if (!payload.has_value())
                {
                    auto resp = build_response_mapping_save("", "", "reject", "no payload");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                auto bytes = payload->as_vector();
                auto req = SLAMNAV::GetRequest_Mapping_Save(bytes.data());
                if (!req)
                {
                    auto resp = build_response_mapping_save("", "", "reject", "invalid request");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                QString id = QString::fromStdString(req->id() ? req->id()->str() : "");
                QString map_name = QString::fromStdString(req->map_name() ? req->map_name()->str() : "");

                if (map_name.isEmpty())
                {
                    auto resp = build_response_mapping_save(id, map_name, "reject", "map name required");
                    query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));
                    return;
                }

                // Emit signal to save mapping
                Q_EMIT signal_map_save(map_name);

                auto resp = build_response_mapping_save(id, map_name, "accept", "");
                query.reply(zenoh::KeyExpr(query.get_keyexpr()), zenoh::Bytes::serialize(resp));

                auto result_buf = build_map_result(id, "success", "mapping save requested");
                pub_result.put(zenoh::Bytes::serialize(result_buf));
            },
            zenoh::closures::none
        );
        qDebug() << "[" << MODULE_NAME << "] Queryable registered:" << QString::fromStdString(topic_map_save);

        // 6. Main loop - keep alive
        while (is_map_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        qDebug() << "[" << MODULE_NAME << "] map_loop ending, resources will be released";
    }
    catch (const zenoh::ZException& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Zenoh exception:" << e.what();
    }
    catch (const std::exception& e)
    {
        qDebug() << "[" << MODULE_NAME << "] Exception:" << e.what();
    }

    qDebug() << "[" << MODULE_NAME << "] map_loop ended";
}
