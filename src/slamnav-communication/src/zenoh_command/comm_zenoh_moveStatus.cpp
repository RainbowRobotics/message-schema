#include "comm_zenoh.h"
#include "global_defines.h"
#include "flatbuffer/generated/slamnav_status_generated.h"

namespace
{
    constexpr const char* MODULE_NAME = "MOVE_STATUS";
    constexpr int MOVE_STATUS_INTERVAL_MS = 500;  // 500ms
}

void COMM_ZENOH::move_status_loop()
{
    log_info("move_status_loop started");

    while (is_move_status_running_.load() && get_robot_type().empty())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (!is_move_status_running_.load())
    {
        log_info("move_status_loop ended (stopped before init)");
        return;
    }

    log_info("move_status_loop initialized with robotType: {}", get_robot_type());

    auto last_time = std::chrono::steady_clock::now();
    const auto interval = std::chrono::milliseconds(MOVE_STATUS_INTERVAL_MS);

    while (is_move_status_running_.load())
    {
        auto now = std::chrono::steady_clock::now();
        if (now - last_time >= interval)
        {
            publish_move_status(this);
            last_time = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    log_info("move_status_loop ended");
}


static void publish_move_status(COMM_ZENOH* zenoh)
{
    if (!zenoh->get_is_connected())
    {
        return;
    }

    if (!zenoh->is_session_valid())
    {
        return;
    }

    AUTOCONTROL* ctrl = zenoh->get_autocontrol();
    MOBILE* mobile = zenoh->get_mobile();
    UNIMAP* unimap = zenoh->get_unimap();
    DOCKCONTROL* dctrl = zenoh->get_dockcontrol();
    LOCALIZATION* loc = zenoh->get_localization();

    if (!ctrl || !mobile || !unimap || !dctrl || !loc)
    {
        return;
    }

    try
    {
        // flatbuffer
        flatbuffers::FlatBufferBuilder fbb(1024);

        const MOBILE_POSE mo = mobile->get_pose();
        const Eigen::Vector3d cur_xi = TF_to_se2(loc->get_cur_tf());
        QString cur_node_id = ctrl->get_cur_node_id();
        QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;

        // cur node
        QString cur_node_name = "";
        if (unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
        {
            if (NODE* node = unimap->get_node_by_id(cur_node_id))
            {
                cur_node_name = node->name;
            }
        }

        // goal node
        QString goal_node_name = "";
        Eigen::Vector3d goal_xi(0, 0, 0);
        if (unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
        {
            if (NODE* node = unimap->get_node_by_id(goal_node_id))
            {
                goal_node_name = node->name;
                goal_xi = TF_to_se2(node->tf);
            }
        }

        // StatusMoveState (table)
        // auto_move , dock_move , jog_move, obs , path
        auto move_state = SLAMNAV::CreateStatusMoveState(fbb,
            fbb.CreateString(ctrl->get_auto_state().toStdString()),
            fbb.CreateString("stop"),
            fbb.CreateString("none"),
            fbb.CreateString(ctrl->get_obs_condition().toStdString()),
            fbb.CreateString(ctrl->get_multi_reqest_state().toStdString())
        );

        // StatusPose (struct)
        // x , y , z , rz
        SLAMNAV::StatusPose pose(
            static_cast<float>(cur_xi[0]),
            static_cast<float>(cur_xi[1]),
            0.0f,
            static_cast<float>(cur_xi[2] * R2D)
        );

        // StatusVel (struct)
        // vx , vy , wz
        SLAMNAV::StatusVel vel(
            static_cast<float>(mo.vel[0]),
            static_cast<float>(mo.vel[1]),
            static_cast<float>(mo.vel[2] * R2D)
        );

        // StatusNode (cur_node) (table)
        // node_id , name , state , x , y , rz
        auto cur_node = SLAMNAV::CreateStatusNode(fbb,
            fbb.CreateString(cur_node_id.toStdString()),
            fbb.CreateString(cur_node_name.toStdString()),
            fbb.CreateString(""),
            static_cast<float>(cur_xi[0]),
            static_cast<float>(cur_xi[1]),
            static_cast<float>(cur_xi[2] * R2D)
        );

        // StatusNode (goal_node) (table)
        // node_id , name , state , x , y , rz
        auto goal_node = SLAMNAV::CreateStatusNode(fbb,
            fbb.CreateString(goal_node_id.toStdString()),
            fbb.CreateString(goal_node_name.toStdString()),
            fbb.CreateString(ctrl->get_cur_move_state().toStdString()),
            static_cast<float>(goal_xi[0]),
            static_cast<float>(goal_xi[1]),
            static_cast<float>(goal_xi[2] * R2D)
        );

        // Move_Status (root)
        // StatusNode , StatusNode , StatusMoveState , StatusPose , StatusVel
        auto move_status = SLAMNAV::CreateMove_Status(fbb,
            cur_node,
            goal_node,
            move_state,
            &pose,
            &vel
        );
        fbb.Finish(move_status);

        const uint8_t* data = fbb.GetBufferPointer();
        size_t size = fbb.GetSize();
        std::string payload(reinterpret_cast<const char*>(data), size);

        std::string topic = zenoh->make_topic("moveStatus");
        zenoh->get_session().put(zenoh::KeyExpr(topic), payload);
    }
    catch (const std::exception& e)
    {
        log_error("publish_move_status error: {}", e.what());
    }
}