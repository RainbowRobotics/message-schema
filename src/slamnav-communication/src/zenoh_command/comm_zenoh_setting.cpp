/**
 * @file comm_zenoh_setting.cpp
 * @brief Setting 도메인 Zenoh 통신 구현
 *
 * RPC (Queryable):
 *   - setting/getSensorIndex, setting/setSensorIndex
 *   - setting/setSensorOn, setting/getSensorOff
 *   - setting/getPduParam, setting/setPduParam, setting/getDriveParam
 *
 * Publisher:
 *   - setting/result
 */

#include "comm_zenoh.h"
#include "global_defines.h"
#include "slamnav_setting_generated.h"

#include <QDebug>
#include <chrono>
#include <functional>

namespace
{
    const char* MODULE_NAME = "COMM_ZENOH_SETTING";

    // =========================================================================
    // Helper: Setting_Result FlatBuffer
    // =========================================================================
    std::vector<uint8_t> build_setting_result(const QString& id,
                                               const QString& result,
                                               const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(256);

        auto fb_result = SLAMNAV::CreateSetting_Result(
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

    // Response_Get_Sensor_Index (카메라 정보)
    std::vector<uint8_t> build_response_get_sensor_index(
        const QString& id,
        const QString& target,
        const std::vector<std::pair<int, QString>>& index,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SensorInfo>> index_vec;
        for (const auto& v : index)
        {
            auto si = SLAMNAV::CreateSensorInfo(fbb,
                v.first,
                fbb.CreateString(v.second.toStdString())
            );
            index_vec.push_back(si);
        }

        auto resp = SLAMNAV::CreateResponse_Get_Sensor_Index(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(target.toStdString()),
            fbb.CreateVector(index_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Sensor_Index
    std::vector<uint8_t> build_response_set_sensor_index(
        const QString& id,
        const QString& target,
        const std::vector<std::pair<int, QString>>& index,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SensorInfo>> index_vec;
        for (const auto& v : index)
        {
            auto si = SLAMNAV::CreateSensorInfo(fbb,
                v.first,
                fbb.CreateString(v.second.toStdString())
            );
            index_vec.push_back(si);
        }

        auto resp = SLAMNAV::CreateResponse_Set_Sensor_Index(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(target.toStdString()),
            fbb.CreateVector(index_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Sensor_On
    std::vector<uint8_t> build_response_set_sensor_on(
        const QString& id,
        const QString& target,
        const std::vector<std::pair<int, QString>>& index,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SensorInfo>> index_vec;
        for (const auto& v : index)
        {
            auto si = SLAMNAV::CreateSensorInfo(fbb,
                v.first,
                fbb.CreateString(v.second.toStdString())
            );
            index_vec.push_back(si);
        }

        auto resp = SLAMNAV::CreateResponse_Set_Sensor_On(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(target.toStdString()),
            fbb.CreateVector(index_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Sensor_Off
    std::vector<uint8_t> build_response_get_sensor_off(
        const QString& id,
        const QString& target,
        const std::vector<std::pair<int, QString>>& index,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(512);

        std::vector<flatbuffers::Offset<SLAMNAV::SensorInfo>> index_vec;
        for (const auto& v : index)
        {
            auto si = SLAMNAV::CreateSensorInfo(fbb,
                v.first,
                fbb.CreateString(v.second.toStdString())
            );
            index_vec.push_back(si);
        }

        auto resp = SLAMNAV::CreateResponse_Get_Sensor_Off(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateString(target.toStdString()),
            fbb.CreateVector(index_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Pdu_Param
    std::vector<uint8_t> build_response_get_pdu_param(
        const QString& id,
        const std::vector<std::tuple<QString, QString, QString>>& params,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(1024);

        std::vector<flatbuffers::Offset<SLAMNAV::SettingParam>> params_vec;
        for (const auto& p : params)
        {
            params_vec.push_back(SLAMNAV::CreateSettingParam(fbb,
                fbb.CreateString(std::get<0>(p).toStdString()),
                fbb.CreateString(std::get<1>(p).toStdString()),
                fbb.CreateString(std::get<2>(p).toStdString())
            ));
        }

        auto resp = SLAMNAV::CreateResponse_Get_Pdu_Param(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(params_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Set_Pdu_Param
    std::vector<uint8_t> build_response_set_pdu_param(
        const QString& id,
        const std::vector<std::tuple<QString, QString, QString>>& params,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(1024);

        std::vector<flatbuffers::Offset<SLAMNAV::SettingParam>> params_vec;
        for (const auto& p : params)
        {
            params_vec.push_back(SLAMNAV::CreateSettingParam(fbb,
                fbb.CreateString(std::get<0>(p).toStdString()),
                fbb.CreateString(std::get<1>(p).toStdString()),
                fbb.CreateString(std::get<2>(p).toStdString())
            ));
        }

        auto resp = SLAMNAV::CreateResponse_Set_Pdu_Param(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(params_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

    // Response_Get_Drive_Param
    std::vector<uint8_t> build_response_get_drive_param(
        const QString& id,
        const std::vector<std::tuple<QString, QString, QString>>& params,
        const QString& result,
        const QString& message)
    {
        flatbuffers::FlatBufferBuilder fbb(1024);

        std::vector<flatbuffers::Offset<SLAMNAV::SettingParam>> params_vec;
        for (const auto& p : params)
        {
            params_vec.push_back(SLAMNAV::CreateSettingParam(fbb,
                fbb.CreateString(std::get<0>(p).toStdString()),
                fbb.CreateString(std::get<1>(p).toStdString()),
                fbb.CreateString(std::get<2>(p).toStdString())
            ));
        }

        auto resp = SLAMNAV::CreateResponse_Get_Drive_Param(
            fbb,
            fbb.CreateString(id.toStdString()),
            fbb.CreateVector(params_vec),
            fbb.CreateString(result.toStdString()),
            fbb.CreateString(message.toStdString())
        );
        fbb.Finish(resp);

        const uint8_t* buf = fbb.GetBufferPointer();
        return std::vector<uint8_t>(buf, buf + fbb.GetSize());
    }

} // anonymous namespace

// =============================================================================
// setting_loop
// =============================================================================
void COMM_ZENOH::setting_loop()
{
    // 1. robotType 대기
    while (get_robot_type().empty())
    {
        if (!is_setting_running_.load()) return;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 2. Session 유효성 확인
    if (!is_session_valid())
    {
        qWarning() << "[" << MODULE_NAME << "] Session not valid, exiting setting_loop";
        return;
    }

    // 3. Topic 생성
    std::string topic_get_sensor_index = make_topic(ZENOH_TOPIC::SETTING_GET_SENSOR_INDEX);
    std::string topic_set_sensor_index = make_topic(ZENOH_TOPIC::SETTING_SET_SENSOR_INDEX);
    std::string topic_set_sensor_on    = make_topic(ZENOH_TOPIC::SETTING_SET_SENSOR_ON);
    std::string topic_get_sensor_off   = make_topic(ZENOH_TOPIC::SETTING_GET_SENSOR_OFF);
    std::string topic_get_pdu          = make_topic(ZENOH_TOPIC::SETTING_GET_PDU);
    std::string topic_set_pdu          = make_topic(ZENOH_TOPIC::SETTING_SET_PDU);
    std::string topic_get_drive        = make_topic(ZENOH_TOPIC::SETTING_GET_DRIVE);
    std::string topic_result           = make_topic(ZENOH_TOPIC::SETTING_RESULT);

    qDebug() << "[" << MODULE_NAME << "] Topics:";
    qDebug() << "  - getSensorIndex:" << QString::fromStdString(topic_get_sensor_index);
    qDebug() << "  - setSensorIndex:" << QString::fromStdString(topic_set_sensor_index);
    qDebug() << "  - setSensorOn:" << QString::fromStdString(topic_set_sensor_on);
    qDebug() << "  - getSensorOff:" << QString::fromStdString(topic_get_sensor_off);
    qDebug() << "  - getPduParam:" << QString::fromStdString(topic_get_pdu);
    qDebug() << "  - setPduParam:" << QString::fromStdString(topic_set_pdu);
    qDebug() << "  - getDriveParam:" << QString::fromStdString(topic_get_drive);
    qDebug() << "  - result:" << QString::fromStdString(topic_result);

    auto& session = zenoh_session_->get_session();

    // 4. Result Publisher 등록
    auto pub_result = session.declare_publisher(
        zenoh::KeyExpr(topic_result),
        zenoh::Session::PublisherOptions::create_default()
    );

    // =========================================================================
    // 5. Queryable: getSensorIndex (카메라 인덱스/시리얼 조회)
    // =========================================================================
    auto q_get_sensor_index = session.declare_queryable(
        zenoh::KeyExpr(topic_get_sensor_index),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            QString target = "";
            std::vector<std::pair<int, QString>> index;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Get_Sensor_Index(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());
                    target = req->target() ? QString::fromStdString(req->target()->str()) : "";

                    if (!config)
                    {
                        message = "config not available";
                    }
                    else if (target == "cam")
                    {
                        // 카메라 인덱스 조회
                        for (int idx = 0; idx < config->get_cam_num(); ++idx)
                        {
                            auto serial = config->get_cam_serial_number(idx);
                            index.emplace_back(idx, serial);
                        }
                        result_str = "accept";
                    }
                    else if (target == "lidar3d")
                    {
                        // 3D LiDAR 인덱스 조회
                        for (int idx = 0; idx < config->get_lidar_3d_num(); ++idx)
                        {
                            index.emplace_back(idx, QString("lidar3d_%1").arg(idx));
                        }
                        result_str = "accept";
                    }
                    else
                    {
                        message = "invalid target";
                    }
                }
            }

            auto resp_buf = build_response_get_sensor_index(id, target, index, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 6. Queryable: setSensorIndex (카메라 순서 설정)
    // =========================================================================
    auto q_set_sensor_index = session.declare_queryable(
        zenoh::KeyExpr(topic_set_sensor_index),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            QString target = "";
            std::vector<std::pair<int, QString>> index;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Set_Sensor_Index(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());
                    target = req->target() ? QString::fromStdString(req->target()->str()) : "";

                    if (!config)
                    {
                        message = "config not available";
                    }
                    else if (target == "cam")
                    {
                        // 요청에서 인덱스 추출
                        if (req->index())
                        {
                            for (size_t i = 0; i < req->index()->size(); ++i)
                            {
                                auto si = req->index()->Get(i);
                                if (si)
                                {
                                    int idx = si->index();
                                    QString serial = si->serial_number() ?
                                        QString::fromStdString(si->serial_number()->str()) : "";
                                    index.emplace_back(idx, serial);
                                }
                            }
                        }

                        int cam_num = config->get_cam_num();
                        bool valid = true;
                        for (const auto& v : index)
                        {
                            if (v.first >= cam_num)
                            {
                                valid = false;
                                message = "invalid index";
                                break;
                            }
                        }

                        if (valid)
                        {
                            // 카메라 순서 설정
                            std::vector<QString> cam_serial_number;
                            cam_serial_number.resize(cam_num);
                            for (int i = 0; i < cam_num; ++i)
                            {
                                bool found = false;
                                for (const auto& v : index)
                                {
                                    if (v.first == i)
                                    {
                                        cam_serial_number[i] = v.second;
                                        found = true;
                                        break;
                                    }
                                }
                                if (!found)
                                {
                                    cam_serial_number[i] = config->get_cam_serial_number(i);
                                }
                            }

                            if (config->set_cam_order(cam_serial_number))
                            {
                                result_str = "accept";
                            }
                            else
                            {
                                message = "failed to set camera order";
                            }
                        }
                    }
                    else
                    {
                        message = "invalid target";
                    }
                }
            }

            auto resp_buf = build_response_set_sensor_index(id, target, index, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 7. Queryable: setSensorOn (센서 켜기)
    // =========================================================================
    auto q_set_sensor_on = session.declare_queryable(
        zenoh::KeyExpr(topic_set_sensor_on),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            QString target = "";
            std::vector<std::pair<int, QString>> index;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Set_Sensor_On(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());
                    target = req->target() ? QString::fromStdString(req->target()->str()) : "";

                    // 인덱스 추출
                    if (req->index())
                    {
                        for (size_t i = 0; i < req->index()->size(); ++i)
                        {
                            auto si = req->index()->Get(i);
                            if (si)
                            {
                                int idx = si->index();
                                QString serial = si->serial_number() ?
                                    QString::fromStdString(si->serial_number()->str()) : "";
                                index.emplace_back(idx, serial);
                            }
                        }
                    }

                    if (target == "lidar3d")
                    {
                        if (!lidar_3d || !lidar_3d->get_is_connected())
                        {
                            message = "lidar3d not connected";
                        }
                        else
                        {
                            std::vector<int> indexs;
                            int lidar_num = config ? config->get_lidar_3d_num() : 0;
                            bool valid = true;

                            for (const auto& v : index)
                            {
                                if (v.first >= lidar_num)
                                {
                                    valid = false;
                                    message = "invalid index";
                                    break;
                                }
                                indexs.push_back(v.first);
                            }

                            if (valid)
                            {
                                Q_EMIT(lidar_3d->signal_set_on(indexs));
                                result_str = "accept";
                            }
                        }
                    }
                    else
                    {
                        message = "invalid target";
                    }
                }
            }

            auto resp_buf = build_response_set_sensor_on(id, target, index, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 8. Queryable: getSensorOff (센서 끄기)
    // =========================================================================
    auto q_get_sensor_off = session.declare_queryable(
        zenoh::KeyExpr(topic_get_sensor_off),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            QString target = "";
            std::vector<std::pair<int, QString>> index;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Get_Sensor_Off(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());
                    target = req->target() ? QString::fromStdString(req->target()->str()) : "";

                    // 인덱스 추출
                    if (req->index())
                    {
                        for (size_t i = 0; i < req->index()->size(); ++i)
                        {
                            auto si = req->index()->Get(i);
                            if (si)
                            {
                                int idx = si->index();
                                QString serial = si->serial_number() ?
                                    QString::fromStdString(si->serial_number()->str()) : "";
                                index.emplace_back(idx, serial);
                            }
                        }
                    }

                    if (target == "lidar3d")
                    {
                        if (!lidar_3d || !lidar_3d->get_is_connected())
                        {
                            message = "lidar3d not connected";
                        }
                        else
                        {
                            std::vector<int> indexs;
                            int lidar_num = config ? config->get_lidar_3d_num() : 0;
                            bool valid = true;

                            for (const auto& v : index)
                            {
                                if (v.first >= lidar_num)
                                {
                                    valid = false;
                                    message = "invalid index";
                                    break;
                                }
                                indexs.push_back(v.first);
                            }

                            if (valid)
                            {
                                Q_EMIT(lidar_3d->signal_set_off(indexs));
                                result_str = "accept";
                            }
                        }
                    }
                    else
                    {
                        message = "invalid target";
                    }
                }
            }

            auto resp_buf = build_response_get_sensor_off(id, target, index, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 9. Queryable: getPduParam (PDU 파라미터 조회)
    // =========================================================================
    auto q_get_pdu = session.declare_queryable(
        zenoh::KeyExpr(topic_get_pdu),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            std::vector<std::tuple<QString, QString, QString>> params;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Get_Pdu_Param(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());

                    if (!mobile)
                    {
                        message = "mobile not connected";
                    }
                    else
                    {
                        // 현재 PDU 파라미터 조회
                        bool use_sf_obstacle_detect = mobile->get_detect_mode();
                        params.emplace_back("use_sf_obstacle_detect", "boolean",
                            use_sf_obstacle_detect ? "true" : "false");
                        result_str = "accept";
                    }
                }
            }

            auto resp_buf = build_response_get_pdu_param(id, params, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 10. Queryable: setPduParam (PDU 파라미터 설정)
    // =========================================================================
    auto q_set_pdu = session.declare_queryable(
        zenoh::KeyExpr(topic_set_pdu),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            std::vector<std::tuple<QString, QString, QString>> params;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Set_Pdu_Param(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());

                    if (!mobile)
                    {
                        message = "mobile not connected";
                    }
                    else if (!req->params() || req->params()->size() == 0)
                    {
                        message = "no params";
                    }
                    else
                    {
                        // 파라미터 추출
                        for (size_t i = 0; i < req->params()->size(); ++i)
                        {
                            auto p = req->params()->Get(i);
                            if (p && p->key())
                            {
                                QString key = QString::fromStdString(p->key()->str());
                                QString type = p->type() ?
                                    QString::fromStdString(p->type()->str()) : "";
                                QString value = p->value() ?
                                    QString::fromStdString(p->value()->str()) : "";
                                params.emplace_back(key, type, value);
                            }
                        }

                        if (params.size() != 1)
                        {
                            message = "only one param supported";
                        }
                        else
                        {
                            auto& [key, type, value] = params[0];

                            if (key == "use_sf_obstacle_detect")
                            {
                                if (type != "boolean")
                                {
                                    message = "invalid type for use_sf_obstacle_detect";
                                }
                                else if (value != "true" && value != "false")
                                {
                                    message = "invalid value for use_sf_obstacle_detect";
                                }
                                else
                                {
                                    bool is_true = (value == "true");
                                    mobile->set_detect_mode(is_true);
                                    result_str = "accept";
                                }
                            }
                            else
                            {
                                message = "unknown parameter: " + key;
                            }
                        }
                    }
                }
            }

            auto resp_buf = build_response_set_pdu_param(id, params, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );

            if (result_str == "accept" && !id.isEmpty())
            {
                auto result_buf = build_setting_result(id, "success", "");
                pub_result.put(zenoh::Bytes(std::move(result_buf)));
            }
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    // =========================================================================
    // 11. Queryable: getDriveParam (드라이브 파라미터 조회)
    // =========================================================================
    auto q_get_drive = session.declare_queryable(
        zenoh::KeyExpr(topic_get_drive),
        [this, &pub_result](const zenoh::Query& query) {
            QString id = "";
            std::vector<std::tuple<QString, QString, QString>> params;
            QString result_str = "reject";
            QString message = "";

            auto payload_opt = query.get_payload();
            if (!payload_opt.has_value())
            {
                message = "no payload";
            }
            else
            {
                auto payload_bytes = payload_opt->get().as_vector();
                auto req = SLAMNAV::GetRequest_Get_Drive_Param(payload_bytes.data());
                if (!req || !req->id())
                {
                    message = "invalid request";
                }
                else
                {
                    id = QString::fromStdString(req->id()->str());

                    if (!mobile)
                    {
                        message = "mobile not connected";
                    }
                    else
                    {
                        // 로봇에 파라미터 요청
                        mobile->robot_request();

                        // 비동기로 응답하기 위해 delayed_tasks 사용
                        auto query_copy = query;
                        auto id_copy = id;
                        auto pub_result_ptr = &pub_result;

                        delayed_tasks_.schedule(std::chrono::milliseconds(1000), [this, query_copy, id_copy, pub_result_ptr]() {
                            auto ms = mobile->get_setting();

                            std::vector<std::tuple<QString, QString, QString>> drive_params;
                            drive_params.emplace_back("version", "int", QString::number(ms.version));
                            drive_params.emplace_back("robot_type", "int", QString::number(ms.robot_type));
                            drive_params.emplace_back("v_limit", "float", QString::number(ms.v_limit, 'f', 3));
                            drive_params.emplace_back("w_limit", "float", QString::number(ms.w_limit * R2D, 'f', 3));
                            drive_params.emplace_back("a_limit", "float", QString::number(ms.a_limit, 'f', 3));
                            drive_params.emplace_back("b_limit", "float", QString::number(ms.b_limit * R2D, 'f', 3));

                            auto resp_buf = build_response_get_drive_param(id_copy, drive_params, "accept", "");
                            query_copy.reply(
                                zenoh::KeyExpr(query_copy.get_keyexpr()),
                                zenoh::Bytes(std::move(resp_buf))
                            );

                            auto result_buf = build_setting_result(id_copy, "success", "");
                            pub_result_ptr->put(zenoh::Bytes(std::move(result_buf)));
                        });

                        // 비동기 처리이므로 여기서는 리턴
                        return;
                    }
                }
            }

            // 에러인 경우 즉시 응답
            auto resp_buf = build_response_get_drive_param(id, params, result_str, message);
            query.reply(
                zenoh::KeyExpr(query.get_keyexpr()),
                zenoh::Bytes(std::move(resp_buf))
            );
        },
        zenoh::closures::none,
        zenoh::Session::QueryableOptions::create_default()
    );

    qDebug() << "[" << MODULE_NAME << "] All queryables registered, entering main loop";

    // =========================================================================
    // 12. Main loop (keep alive)
    // =========================================================================
    while (is_setting_running_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    qDebug() << "[" << MODULE_NAME << "] Exiting setting_loop";
    // RAII: 리소스 자동 해제
}
