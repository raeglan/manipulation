#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>

const char *controller = 
"scope:\n\
  # definition of some nice short-cuts\n\
  - unit_x: {vector3: [1, 0, 0]}\n\
  - unit_y: {vector3: [0, 1, 0]}\n\
  - unit_z: {vector3: [0, 0, 1]}\n\
\n\
\n\
  # definition of joint input variables\n\
  - torso_lift_joint: {input-var: 0}\n\
  - l_shoulder_pan_joint: {input-var: 1}\n\
  - l_shoulder_lift_joint: {input-var: 2}\n\
  - l_upper_arm_roll_joint: {input-var: 3}\n\
  - l_elbow_flex_joint: {input-var: 4}\n\
  - l_forearm_roll_joint: {input-var: 5}\n\
  - l_wrist_flex_joint: {input-var: 6}\n\
  - l_wrist_roll_joint: {input-var: 7}\n\
  - r_shoulder_pan_joint: {input-var: 8}\n\
  - r_shoulder_lift_joint: {input-var: 9}\n\
  - r_upper_arm_roll_joint: {input-var: 10}\n\
  - r_elbow_flex_joint: {input-var: 11}\n\
  - r_forearm_roll_joint: {input-var: 12}\n\
  - r_wrist_flex_joint: {input-var: 13}\n\
  - r_wrist_roll_joint: {input-var: 14}\n\
\n\
  # definition goal input variables\n\
  - r_pos_goal_x: {input-var: 15}\n\
  - r_pos_goal_y: {input-var: 16}\n\
  - r_pos_goal_z: {input-var: 17}\n\
\n\
  # definition of joint transforms\n\
  - torso_lift:\n\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [-0.05, 0, {double-add: [0.739675, torso_lift_joint]}]}]\n\
  - l_shoulder_pan:\n\
      frame: [{axis-angle: [unit_z, l_shoulder_pan_joint]}, {vector3: [0.0, 0.188, 0.0]}]\n\
  - l_shoulder_lift:\n\
      frame: [{axis-angle: [unit_y, l_shoulder_lift_joint]}, {vector3: [0.1, 0, 0]}]\n\
  - l_upper_arm_roll:\n\
      frame: [{axis-angle: [unit_x, l_upper_arm_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - l_elbow_flex:\n\
      frame: [{axis-angle: [unit_y, l_elbow_flex_joint]}, {vector3: [0.4, 0, 0]}]\n\
  - l_forearm_roll:\n\
      frame: [{axis-angle: [unit_x, l_forearm_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - l_wrist_flex:\n\
      frame: [{axis-angle: [unit_y, l_wrist_flex_joint]}, {vector3: [0.321, 0, 0]}]\n\
  - l_wrist_roll:\n\
      frame: [{axis-angle: [unit_x, l_wrist_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - l_gripper_offset:\n\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [0.18, 0, 0]}]\n\
  - r_shoulder_pan:\n\
      frame: [{axis-angle: [unit_z, r_shoulder_pan_joint]}, {vector3: [0, -0.188, 0]}]\n\
  - r_shoulder_lift:\n\
      frame: [{axis-angle: [unit_y, r_shoulder_lift_joint]}, {vector3: [0.1, 0, 0]}]\n\
  - r_upper_arm_roll: \n\
      frame: [{axis-angle: [unit_x, r_upper_arm_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - r_elbow_flex:\n\
      frame: [{axis-angle: [unit_y, r_elbow_flex_joint]}, {vector3: [0.4, 0, 0]}]\n\
  - r_forearm_roll:\n\
      frame: [{axis-angle: [unit_x, r_forearm_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - r_wrist_flex:\n\
      frame: [{axis-angle: [unit_y, r_wrist_flex_joint]}, {vector3: [0.321, 0, 0]}]\n\
  - r_wrist_roll:\n\
      frame: [{axis-angle: [unit_x, r_wrist_roll_joint]}, {vector3: [0, 0, 0]}]\n\
  - r_gripper_offset:\n\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [0.18, 0, 0]}]\n\
\n\
\n\
  # definition of elbow FK\n\
  - left_elbow:\n\
      frame-mul:\n\
      - torso_lift\n\
      - l_shoulder_pan\n\
      - l_shoulder_lift\n\
      - l_upper_arm_roll\n\
      - l_elbow_flex\n\
  - right_elbow:\n\
      frame-mul:\n\
      - torso_lift\n\
      - r_shoulder_pan\n\
      - r_shoulder_lift\n\
      - r_upper_arm_roll\n\
      - r_elbow_flex\n\
\n\
  # defintion of EE FK\n\
  - left_ee:\n\
      frame-mul:\n\
      - left_elbow\n\
      - l_forearm_roll\n\
      - l_wrist_flex\n\
      - l_wrist_roll\n\
      - l_gripper_offset\n\
  - right_ee:\n\
      frame-mul:\n\
      - right_elbow\n\
      - r_forearm_roll\n\
      - r_wrist_flex\n\
      - r_wrist_roll\n\
      - r_gripper_offset\n\
\n\
  # control params\n\
  - pos_p_gain: 1.0\n\
  - rot_p_gain: 1.0\n\
  - pos_thresh: 0.05\n\
  - rot_thresh: 0.1\n\
  - weight_arm_joints: 0.001\n\
  - weight_torso_joint: 0.001\n\
  - weight_pos_goal: 1\n\
  - weight_align_goal: 0.5\n\
  - weight_rot_control: 1\n\
  - weight_elbow_control: 0.4\n\
  - neg_vel_limit_arm_joints: -0.6\n\
  - pos_vel_limit_arm_joints: 0.6\n\
  - neg_vel_limit_torso_joint: -0.02\n\
  - pos_vel_limit_torso_joint: 0.02\n\
\n\
  # definition EE goals and control laws\n\
  #  # right arm position\n\
  - r_trans: {origin-of: right_ee}\n\
  #  # right arm goal diff\n\
  - r_goal_diff: { vector-sub: [{vector3: [r_pos_goal_x, r_pos_goal_y, r_pos_goal_z]}, r_trans]}\n\
  - r_diff_x: {x-coord: r_goal_diff}\n\
  - r_diff_y: {y-coord: r_goal_diff}\n\
  - r_diff_z: {z-coord: r_goal_diff}\n\
  #  - r_rot_align: {vector-sub: [{origin-of: right_elbow}, r_trans]}\n\
  #  - r_gripper_align: {rot-vector: {orientation-of: right_ee}}\n\
  # right arm rotation\n\
  - r_goal_rot: {quaternion: [0, 0, 0, 1]}\n\
  - r_rot: {orientation-of: right_ee}\n\
  - r_rot_error: {vector-norm: {rot-vector: {rotation-mul: [{inverse-rotation: r_rot}, r_goal_rot]}}}\n\
  - r_rot_scaling: \n\
      double-if:\n\
      - {double-sub: [rot_thresh, r_rot_error]}\n\
      - 1\n\
      - {double-div: [rot_thresh, r_rot_error]}\n\
  - r_intermediate_goal_rot:\n\
      slerp:\n\
      - r_rot\n\
      - r_goal_rot\n\
      - r_rot_scaling\n\
  - r_rot_control:\n\
      scale-vector: [rot_p_gain, {rotate-vector: [r_rot, {rot-vector: {rotation-mul: [{inverse-rotation: r_rot}, r_intermediate_goal_rot]}}]}]\n\
  - r_elbow_diff: {vector-sub: [{origin-of: right_ee}, {origin-of: right_elbow}]}\n\
\n\
controllable-constraints:\n\
  # torso joint\n\
  - controllable-constraint: [neg_vel_limit_torso_joint, pos_vel_limit_torso_joint, weight_torso_joint, 0, torso_lift_joint]\n\
  # left arm joints\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 1, l_shoulder_pan_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 2, l_shoulder_lift_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 3, l_upper_arm_roll_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 4, l_elbow_flex_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 5, l_forearm_roll_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 6, l_wrist_flex_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 7, l_wrist_roll_joint]\n\
  # right arm joints\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 8, r_shoulder_pan_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 9, r_shoulder_lift_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 10, r_upper_arm_roll_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 11, r_elbow_flex_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 12, r_forearm_roll_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 13, r_wrist_flex_joint]\n\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 14, r_wrist_roll_joint]\n\
\n\
soft-constraints:\n\
  # Position constraint\n\
  - soft-constraint: [r_diff_x, r_diff_x, weight_pos_goal, {x-coord: r_trans}, right EE x-vel slack]\n\
  - soft-constraint: [r_diff_y, r_diff_y, weight_pos_goal, {y-coord: r_trans}, right EE y-vel slack]\n\
  - soft-constraint: [r_diff_z, r_diff_z, weight_pos_goal, {z-coord: r_trans}, right EE z-vel slack]\n\
  \n\
  # Rotation constraint\n\
  - soft-constraint: [{x-coord: r_rot_control}, {x-coord: r_rot_control}, weight_rot_control, {x-coord: {rot-vector: r_rot}}, right EE x-rot control slack]\n\
  - soft-constraint: [{y-coord: r_rot_control}, {y-coord: r_rot_control}, weight_rot_control, {y-coord: {rot-vector: r_rot}}, right EE y-rot control slack]\n\
  - soft-constraint: [{z-coord: r_rot_control}, {z-coord: r_rot_control}, weight_rot_control, {z-coord: {rot-vector: r_rot}}, right EE z-rot control slack]\n\
\n\
  # Level elbow and gripper control\n\
  - soft-constraint: [0.01, 0.01, weight_elbow_control, {y-coord: r_elbow_diff}, right elbow up control slack]\n\
\n\
hard-constraints:\n\
  - hard-constraint:\n\
      - {double-sub: [0.0115, torso_lift_joint]}\n\
      - {double-sub: [0.325, torso_lift_joint]}\n\
      - torso_lift_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-0.5646, l_shoulder_pan_joint]}\n\
      - {double-sub: [2.1353, l_shoulder_pan_joint]}\n\
      - l_shoulder_pan_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-0.3536, l_shoulder_lift_joint]}\n\
      - {double-sub: [1.2963, l_shoulder_lift_joint]}\n\
      -  l_shoulder_lift_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-0.65, l_upper_arm_roll_joint]}\n\
      - {double-sub: [3.75, l_upper_arm_roll_joint]}\n\
      - l_upper_arm_roll_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-2.1213, l_elbow_flex_joint]}\n\
      - {double-sub: [-0.15, l_elbow_flex_joint]}\n\
      - l_elbow_flex_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-2.0, l_wrist_flex_joint]}\n\
      - {double-sub: [-0.1, l_wrist_flex_joint]}\n\
      - l_wrist_flex_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-2.1353, r_shoulder_pan_joint]}\n\
      - {double-sub: [0.5646, r_shoulder_pan_joint]}\n\
      - r_shoulder_pan_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-0.3536, r_shoulder_lift_joint]}\n\
      - {double-sub: [1.2963, r_shoulder_lift_joint]}\n\
      -  r_shoulder_lift_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-3.75, r_upper_arm_roll_joint]}\n\
      - {double-sub: [0.65, r_upper_arm_roll_joint]}\n\
      - r_upper_arm_roll_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-2.1213, r_elbow_flex_joint]}\n\
      - {double-sub: [-0.15, r_elbow_flex_joint]}\n\
      - r_elbow_flex_joint\n\
  - hard-constraint:\n\
      - {double-sub: [-2.0, r_wrist_flex_joint]}\n\
      - {double-sub: [-0.1, r_wrist_flex_joint]}\n\
      - r_wrist_flex_joint";

int main (int argc, char **argv)
{
  ros::init(argc, argv, "client_test");
  ros::NodeHandle nh("~");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<suturo_manipulation_msgs::MoveRobotAction> ac("/graspkard/movement_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  suturo_manipulation_msgs::MoveRobotGoal goal;
  goal.params.push_back("0");
  goal.params.push_back("0");
  goal.params.push_back("0");
  goal.params.push_back("base_link");
  goal.params.push_back("0.5");
  goal.params.push_back("-0.2");
  goal.params.push_back("0.8");
  goal.controlled_joints.push_back("torso_lift_joint");
  goal.controlled_joints.push_back("l_shoulder_pan_joint");
  goal.controlled_joints.push_back("l_shoulder_lift_joint");
  goal.controlled_joints.push_back("l_upper_arm_roll_joint");
  goal.controlled_joints.push_back("l_elbow_flex_joint");
  goal.controlled_joints.push_back("l_forearm_roll_joint");
  goal.controlled_joints.push_back("l_wrist_flex_joint");
  goal.controlled_joints.push_back("l_wrist_roll_joint");
  goal.controlled_joints.push_back("r_shoulder_pan_joint");
  goal.controlled_joints.push_back("r_shoulder_lift_joint");
  goal.controlled_joints.push_back("r_upper_arm_roll_joint");
  goal.controlled_joints.push_back("r_elbow_flex_joint");
  goal.controlled_joints.push_back("r_forearm_roll_joint");
  goal.controlled_joints.push_back("r_wrist_flex_joint");
  goal.controlled_joints.push_back("r_wrist_roll_joint");
  goal.controller_yaml = std::string(controller);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}