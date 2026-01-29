// slamnav_status.fbs (Move_Status, MoveStatusMoveState, MoveStatusPose, MoveStatusVel, MoveStatusNode)
#include "comm_zenoh.h"
#include "slamnav_status_generated.h"

// ============================================================================
// Send Move Status (periodic, root_type Move_Status)
// ============================================================================
void COMM_ZENOH::send_move_status()
{
  if(!is_connected || !ctrl || !mobile || !unimap || !dctrl || !loc)
  {
    return;
  }

  const MOBILE_POSE mo = mobile->get_pose();
  const Eigen::Vector3d cur_xi = TF_to_se2(loc->get_cur_tf());

  QString cur_node_id  = ctrl->get_cur_node_id();
  QString goal_node_id = ctrl->get_cur_move_info().goal_node_id;

  QString cur_node_name = "";
  if(unimap->get_is_loaded() == MAP_LOADED && !cur_node_id.isEmpty())
  {
    if(NODE* node = unimap->get_node_by_id(cur_node_id))
    {
      cur_node_name = node->name;
    }
  }

  QString goal_node_name = "";
  Eigen::Vector3d goal_xi(0, 0, 0);
  if(unimap->get_is_loaded() == MAP_LOADED && !goal_node_id.isEmpty())
  {
    if(NODE* node = unimap->get_node_by_id(goal_node_id))
    {
      goal_node_name = node->name;
      goal_xi = TF_to_se2(node->tf);
    }
  }

  flatbuffers::FlatBufferBuilder fbb(1024);

  // MoveStatusMoveState (table)
  auto move_state = SLAMNAV::CreateMoveStatusMoveState(fbb,
    fbb.CreateString(ctrl->get_auto_state().toStdString()),
    fbb.CreateString("stop"),
    fbb.CreateString("none"),
    fbb.CreateString(ctrl->get_obs_condition().toStdString()),
    fbb.CreateString(ctrl->get_multi_reqest_state().toStdString())
  );

  // MoveStatusPose (struct - inline)
  SLAMNAV::MoveStatusPose pose(
    static_cast<float>(cur_xi[0]),
    static_cast<float>(cur_xi[1]),
    0.0f,
    static_cast<float>(cur_xi[2] * R2D)
  );

  // MoveStatusVel (struct - inline)
  SLAMNAV::MoveStatusVel vel(
    static_cast<float>(mo.vel[0]),
    static_cast<float>(mo.vel[1]),
    static_cast<float>(mo.vel[2] * R2D)
  );

  // MoveStatusNode (cur_node) (table)
  auto cur_node = SLAMNAV::CreateMoveStatusNode(fbb,
    fbb.CreateString(cur_node_id.toStdString()),
    fbb.CreateString(cur_node_name.toStdString()),
    fbb.CreateString(""),
    static_cast<float>(cur_xi[0]),
    static_cast<float>(cur_xi[1]),
    static_cast<float>(cur_xi[2] * R2D)
  );

  // MoveStatusNode (goal_node) (table)
  auto goal_node = SLAMNAV::CreateMoveStatusNode(fbb,
    fbb.CreateString(goal_node_id.toStdString()),
    fbb.CreateString(goal_node_name.toStdString()),
    fbb.CreateString(ctrl->get_cur_move_state().toStdString()),
    static_cast<float>(goal_xi[0]),
    static_cast<float>(goal_xi[1]),
    static_cast<float>(goal_xi[2] * R2D)
  );

  // Move_Status (root)
  auto ms = SLAMNAV::CreateMove_Status(fbb,
    cur_node,
    goal_node,
    move_state,
    &pose,
    &vel
  );
  fbb.Finish(ms);

  fb_publish(ZENOH_TOPIC::MOVE_STATUS, fbb);
}
