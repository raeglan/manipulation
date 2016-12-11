#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <suturo_manipulation_msgs/MoveRobotAction.h>

const char *controller = 
"scope:\
  # definition of some nice short-cuts\
  - unit_x: {vector3: [1, 0, 0]}\
  - unit_y: {vector3: [0, 1, 0]}\
  - unit_z: {vector3: [0, 0, 1]}\
\
\
  # definition of joint input variables\
  - torso_lift_joint: {input-var: 0}\
  - l_shoulder_pan_joint: {input-var: 1}\
  - l_shoulder_lift_joint: {input-var: 2}\
  - l_upper_arm_roll_joint: {input-var: 3}\
  - l_elbow_flex_joint: {input-var: 4}\
  - l_forearm_roll_joint: {input-var: 5}\
  - l_wrist_flex_joint: {input-var: 6}\
  - l_wrist_roll_joint: {input-var: 7}\
  - r_shoulder_pan_joint: {input-var: 8}\
  - r_shoulder_lift_joint: {input-var: 9}\
  - r_upper_arm_roll_joint: {input-var: 10}\
  - r_elbow_flex_joint: {input-var: 11}\
  - r_forearm_roll_joint: {input-var: 12}\
  - r_wrist_flex_joint: {input-var: 13}\
  - r_wrist_roll_joint: {input-var: 14}\
\
  # definition goal input variables\
  - r_pos_goal_x: {input-var: 15}\
  - r_pos_goal_y: {input-var: 16}\
  - r_pos_goal_z: {input-var: 17}\
\
  # definition of joint transforms\
  - torso_lift:\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [-0.05, 0, {double-add: [0.739675, torso_lift_joint]}]}]\
  - l_shoulder_pan:\
      frame: [{axis-angle: [unit_z, l_shoulder_pan_joint]}, {vector3: [0.0, 0.188, 0.0]}]\
  - l_shoulder_lift:\
      frame: [{axis-angle: [unit_y, l_shoulder_lift_joint]}, {vector3: [0.1, 0, 0]}]\
  - l_upper_arm_roll:\
      frame: [{axis-angle: [unit_x, l_upper_arm_roll_joint]}, {vector3: [0, 0, 0]}]\
  - l_elbow_flex:\
      frame: [{axis-angle: [unit_y, l_elbow_flex_joint]}, {vector3: [0.4, 0, 0]}]\
  - l_forearm_roll:\
      frame: [{axis-angle: [unit_x, l_forearm_roll_joint]}, {vector3: [0, 0, 0]}]\
  - l_wrist_flex:\
      frame: [{axis-angle: [unit_y, l_wrist_flex_joint]}, {vector3: [0.321, 0, 0]}]\
  - l_wrist_roll:\
      frame: [{axis-angle: [unit_x, l_wrist_roll_joint]}, {vector3: [0, 0, 0]}]\
  - l_gripper_offset:\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [0.18, 0, 0]}]\
  - r_shoulder_pan:\
      frame: [{axis-angle: [unit_z, r_shoulder_pan_joint]}, {vector3: [0, -0.188, 0]}]\
  - r_shoulder_lift:\
      frame: [{axis-angle: [unit_y, r_shoulder_lift_joint]}, {vector3: [0.1, 0, 0]}]\
  - r_upper_arm_roll: \
      frame: [{axis-angle: [unit_x, r_upper_arm_roll_joint]}, {vector3: [0, 0, 0]}]\
  - r_elbow_flex:\
      frame: [{axis-angle: [unit_y, r_elbow_flex_joint]}, {vector3: [0.4, 0, 0]}]\
  - r_forearm_roll:\
      frame: [{axis-angle: [unit_x, r_forearm_roll_joint]}, {vector3: [0, 0, 0]}]\
  - r_wrist_flex:\
      frame: [{axis-angle: [unit_y, r_wrist_flex_joint]}, {vector3: [0.321, 0, 0]}]\
  - r_wrist_roll:\
      frame: [{axis-angle: [unit_x, r_wrist_roll_joint]}, {vector3: [0, 0, 0]}]\
  - r_gripper_offset:\
      frame: [{axis-angle: [unit_x, 0]}, {vector3: [0.18, 0, 0]}]\
\
\
  # definition of elbow FK\
  - left_elbow:\
      frame-mul:\
      - torso_lift\
      - l_shoulder_pan\
      - l_shoulder_lift\
      - l_upper_arm_roll\
      - l_elbow_flex\
  - right_elbow:\
      frame-mul:\
      - torso_lift\
      - r_shoulder_pan\
      - r_shoulder_lift\
      - r_upper_arm_roll\
      - r_elbow_flex\
\
  # defintion of EE FK\
  - left_ee:\
      frame-mul:\
      - left_elbow\
      - l_forearm_roll\
      - l_wrist_flex\
      - l_wrist_roll\
      - l_gripper_offset\
  - right_ee:\
      frame-mul:\
      - right_elbow\
      - r_forearm_roll\
      - r_wrist_flex\
      - r_wrist_roll\
      - r_gripper_offset\
\
  # control params\
  - pos_p_gain: 1.0\
  - rot_p_gain: 1.0\
  - pos_thresh: 0.05\
  - rot_thresh: 0.1\
  - weight_arm_joints: 0.001\
  - weight_torso_joint: 0.001\
  - weight_pos_goal: 1\
  - weight_align_goal: 0.5\
  - weight_rot_control: 1\
  - weight_elbow_control: 0.4\
  - neg_vel_limit_arm_joints: -0.6\
  - pos_vel_limit_arm_joints: 0.6\
  - neg_vel_limit_torso_joint: -0.02\
  - pos_vel_limit_torso_joint: 0.02\
\
  # definition EE goals and control laws\
  #  # right arm position\
  - r_trans: {origin-of: right_ee}\
  #  # right arm goal diff\
  - r_goal_diff: { vector-sub: [{vector3: [r_pos_goal_x, r_pos_goal_y, r_pos_goal_z]}, r_trans]}\
  - r_diff_x: {x-coord: r_goal_diff}\
  - r_diff_y: {y-coord: r_goal_diff}\
  - r_diff_z: {z-coord: r_goal_diff}\
  #  - r_rot_align: {vector-sub: [{origin-of: right_elbow}, r_trans]}\
  #  - r_gripper_align: {rot-vector: {orientation-of: right_ee}}\
  # right arm rotation\
  - r_goal_rot: {quaternion: [0, 0, 0, 1]}\
  - r_rot: {orientation-of: right_ee}\
  - r_rot_error: {vector-norm: {rot-vector: {rotation-mul: [{inverse-rotation: r_rot}, r_goal_rot]}}}\
  - r_rot_scaling: \
      double-if:\
      - {double-sub: [rot_thresh, r_rot_error]}\
      - 1\
      - {double-div: [rot_thresh, r_rot_error]}\
  - r_intermediate_goal_rot:\
      slerp:\
      - r_rot\
      - r_goal_rot\
      - r_rot_scaling\
  - r_rot_control:\
      scale-vector: [rot_p_gain, {rotate-vector: [r_rot, {rot-vector: {rotation-mul: [{inverse-rotation: r_rot}, r_intermediate_goal_rot]}}]}]\
  - r_elbow_diff: {vector-sub: [{origin-of: right_ee}, {origin-of: right_elbow}]}\
\
controllable-constraints:\
  # torso joint\
  - controllable-constraint: [neg_vel_limit_torso_joint, pos_vel_limit_torso_joint, weight_torso_joint, 0, torso_lift_joint]\
  # left arm joints\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 1, l_shoulder_pan_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 2, l_shoulder_lift_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 3, l_upper_arm_roll_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 4, l_elbow_flex_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 5, l_forearm_roll_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 6, l_wrist_flex_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 7, l_wrist_roll_joint]\
  # right arm joints\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 8, r_shoulder_pan_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 9, r_shoulder_lift_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 10, r_upper_arm_roll_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 11, r_elbow_flex_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 12, r_forearm_roll_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 13, r_wrist_flex_joint]\
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 14, r_wrist_roll_joint]\
\
soft-constraints:\
  # Position constraint\
  - soft-constraint: [r_diff_x, r_diff_x, weight_pos_goal, {x-coord: r_trans}, right EE x-vel slack]\
  - soft-constraint: [r_diff_y, r_diff_y, weight_pos_goal, {y-coord: r_trans}, right EE y-vel slack]\
  - soft-constraint: [r_diff_z, r_diff_z, weight_pos_goal, {z-coord: r_trans}, right EE z-vel slack]\
  \
  # Rotation constraint\
  - soft-constraint: [{x-coord: r_rot_control}, {x-coord: r_rot_control}, weight_rot_control, {x-coord: {rot-vector: r_rot}}, right EE x-rot control slack]\
  - soft-constraint: [{y-coord: r_rot_control}, {y-coord: r_rot_control}, weight_rot_control, {y-coord: {rot-vector: r_rot}}, right EE y-rot control slack]\
  - soft-constraint: [{z-coord: r_rot_control}, {z-coord: r_rot_control}, weight_rot_control, {z-coord: {rot-vector: r_rot}}, right EE z-rot control slack]\
\
  # Level elbow and gripper control\
  - soft-constraint: [0.01, 0.01, weight_elbow_control, {y-coord: r_elbow_diff}, right elbow up control slack]\
\
hard-constraints:\
  - hard-constraint:\
      - {double-sub: [0.0115, torso_lift_joint]}\
      - {double-sub: [0.325, torso_lift_joint]}\
      - torso_lift_joint\
  - hard-constraint:\
      - {double-sub: [-0.5646, l_shoulder_pan_joint]}\
      - {double-sub: [2.1353, l_shoulder_pan_joint]}\
      - l_shoulder_pan_joint\
  - hard-constraint:\
      - {double-sub: [-0.3536, l_shoulder_lift_joint]}\
      - {double-sub: [1.2963, l_shoulder_lift_joint]}\
      -  l_shoulder_lift_joint\
  - hard-constraint:\
      - {double-sub: [-0.65, l_upper_arm_roll_joint]}\
      - {double-sub: [3.75, l_upper_arm_roll_joint]}\
      - l_upper_arm_roll_joint\
  - hard-constraint:\
      - {double-sub: [-2.1213, l_elbow_flex_joint]}\
      - {double-sub: [-0.15, l_elbow_flex_joint]}\
      - l_elbow_flex_joint\
  - hard-constraint:\
      - {double-sub: [-2.0, l_wrist_flex_joint]}\
      - {double-sub: [-0.1, l_wrist_flex_joint]}\
      - l_wrist_flex_joint\
  - hard-constraint:\
      - {double-sub: [-2.1353, r_shoulder_pan_joint]}\
      - {double-sub: [0.5646, r_shoulder_pan_joint]}\
      - r_shoulder_pan_joint\
  - hard-constraint:\
      - {double-sub: [-0.3536, r_shoulder_lift_joint]}\
      - {double-sub: [1.2963, r_shoulder_lift_joint]}\
      -  r_shoulder_lift_joint\
  - hard-constraint:\
      - {double-sub: [-3.75, r_upper_arm_roll_joint]}\
      - {double-sub: [0.65, r_upper_arm_roll_joint]}\
      - r_upper_arm_roll_joint\
  - hard-constraint:\
      - {double-sub: [-2.1213, r_elbow_flex_joint]}\
      - {double-sub: [-0.15, r_elbow_flex_joint]}\
      - r_elbow_flex_joint\
  - hard-constraint:\
      - {double-sub: [-2.0, r_wrist_flex_joint]}\
      - {double-sub: [-0.1, r_wrist_flex_joint]}\
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