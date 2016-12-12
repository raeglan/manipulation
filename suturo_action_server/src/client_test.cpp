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
  - r_gripper_joint: {input-var: 15}\n\
  \n\
  # definition goal input variables\n\
  - cylinder_pos_x: {input-var: 16}\n\
  - cylinder_pos_y: {input-var: 17}\n\
  - cylinder_pos_z: {input-var: 18}\n\
  \n\
  - cylinder_rot_x: {input-var: 19}\n\
  - cylinder_rot_y: {input-var: 20}\n\
  - cylinder_rot_z: {input-var: 21}\n\
  - cylinder_rot_a: {input-var: 22}\n\
  \n\
  - cylinder_width: {input-var: 23}\n\
  - cylinder_height: {double-mul: [{input-var: 24}, 0.5]}\n\
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
  - cylinder_rot_axis: {vector3: [cylinder_rot_x, cylinder_rot_y, cylinder_rot_z]}\n\
  - cylinder_rot: {axis-angle: [cylinder_rot_axis, cylinder_rot_a]}\n\
  - cylinder_pos: {vector3: [cylinder_pos_x, cylinder_pos_y, cylinder_pos_z]}\n\
  \n\
  - cylinder_frame:\n\
      frame: [cylinder_rot, cylinder_pos]\n\
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
  - weight_align_goal: 0.5\n\
  - weight_elbow_control: 0.4\n\
  - weight_grasp_goal: 2.0\n\
  - neg_vel_limit_arm_joints: -0.6\n\
  - pos_vel_limit_arm_joints: 0.6\n\
  - neg_vel_limit_torso_joint: -0.02\n\
  - pos_vel_limit_torso_joint: 0.02\n\
  - gripper_safety_margin: 0.01\n\
  \n\
  # definition EE goals and control laws\n\
  #  # right arm position\n\
  - r_trans: {origin-of: right_ee}\n\
  #  # right arm goal diff\n\
  - r_goal_diff: {vector-sub: [cylinder_pos, r_trans]}\n\
  - r_goal_dist: {vector-norm: r_goal_diff}\n\
  - r_goal_err: {vector-sub: [r_goal_diff, {scale-vector: [{double-div: [cylinder_width, r_goal_dist]}, r_goal_diff]}]}\n\
  - r_diff_x: {x-coord: r_goal_err} # \n\
  - r_diff_y: {y-coord: r_goal_err}\n\
  - r_diff_z: {z-coord: r_goal_err}\n\
  \n\
  - r_gripper_z: {rotate-vector: [{orientation-of: right_ee}, unit_z]}\n\
  - r_gripper_x: {rotate-vector: [{orientation-of: right_ee}, unit_x]}\n\
  - cylinder_z: {rotate-vector: [cylinder_rot, unit_z]}\n\
  - r_goal_dot: {vector-dot: [cylinder_z, r_gripper_z]}\n\
  - r_ori_ctrl: {double-sub: [1, r_goal_dot]}\n\
  \n\
  - cylinder_safety_width: {double-add: [cylinder_width, gripper_safety_margin]}\n\
  - r_gripper_ctrl: {double-sub: [cylinder_safety_width, r_gripper_joint]}\n\
  - r_gripper_frac: {double-div: [r_gripper_joint, cylinder_width]}\n\
  \n\
  #- dist: {vector-norm: {vector-add: [r_gripper_x, {scale-vector: [{vector-dot: [{scale-vector: [-1, r_gripper_x]},]}, cylinder_z]}]}}\n\
\n\
  - cross_x: {double-sub: [double-mul: [y-coord: r_gripper_x, z-coord: cylinder_z], double-mul: [z-coord: r_gripper_x, y-coord: cylinder_z]]}\n\
  - cross_y: {double-sub: [double-mul: [z-coord: r_gripper_x, x-coord: cylinder_z], double-mul: [x-coord: r_gripper_x, z-coord: cylinder_z]]}\n\
  - cross_z: {double-sub: [double-mul: [x-coord: r_gripper_x, y-coord: cylinder_z], double-mul: [y-coord: r_gripper_x, x-coord: cylinder_z]]}\n\
  \n\
  - cross: {vector3: [cross_x, cross_y,cross_z]}\n\
  - cross_scaled: {scale-vector: [double-div: [1, vector-norm: cross], cross]}\n\
  \n\
  - dist: {abs: {vector-dot: [vector-sub: [cylinder_pos, r_trans], cross_scaled]}}\n\
  - dist2: {double-mul: [dist, -1]}\n\
  \n\
  - p_a: {vector-sub: [r_trans, cylinder_pos]}\n\
  \n\
  - cross_d_x: {double-sub: [double-mul: [y-coord: p_a, z-coord: cylinder_z], double-mul: [z-coord: p_a, y-coord: cylinder_z]]}\n\
  - cross_d_y: {double-sub: [double-mul: [z-coord: p_a, x-coord: cylinder_z], double-mul: [x-coord: p_a, z-coord: cylinder_z]]}\n\
  - cross_d_z: {double-sub: [double-mul: [x-coord: p_a, y-coord: cylinder_z], double-mul: [y-coord: p_a, x-coord: cylinder_z]]}\n\
  \n\
  - weight_gripper_frac: 0.5\n\
  - pos_scale: {min: [{double-add: [r_goal_dot, {double-mul: [r_gripper_frac, weight_gripper_frac]}]}, 1]}\n\
  \n\
  - cross_d: {vector3: [cross_d_x, cross_d_y,cross_d_z]}\n\
  - dist_xy: {vector-norm: cross_d}\n\
  - dist_xy_ctrl: {double-mul: [{double-sub: [0, dist_xy]}, pos_scale]}  \n\
  \n\
  - dist_z: {vector-norm: {vector-sub: [{vector-add: [r_trans, cross_d]}, cylinder_pos] }}\n\
  - dist_z_safety_margin: 0.02\n\
  - cylinder_safety_height: {double-sub: [cylinder_height, dist_z_safety_margin]}\n\
  - dist_z_lower: {double-mul: [{min: [{double-sub: [cylinder_safety_height, dist_z]}, 0]}, pos_scale]}\n\
  - dist_z_upper: {double-mul: [{max: [{double-sub: [cylinder_safety_height, dist_z]}, 0]}, pos_scale]}\n\
  \n\
  - r_elbow_diff: {vector-sub: [{origin-of: right_ee}, {origin-of: right_elbow}]}\n\
  \n\
  - weight_pos_goal: 1 \n\
  - weight_rot_control: 1\n\
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
  - controllable-constraint: [neg_vel_limit_arm_joints, pos_vel_limit_arm_joints, weight_arm_joints, 15, r_gripper_joint]\n\
  \n\
soft-constraints:\n\
  # Position constraint\n\
  #- soft-constraint: [0, 0, weight_pos_goal, {vector-norm: r_goal_err}, right EE x-vel slack]\n\
  - soft-constraint: [dist_xy_ctrl, dist_xy_ctrl, weight_pos_goal, dist_xy, right EE x-vel slack]\n\
  - soft-constraint: [dist_z_lower, dist_z_lower, weight_pos_goal, dist_z, right EE x-vel slack]\n\
  \n\
  # Rotation constraint\n\
  - soft-constraint: [r_ori_ctrl, r_ori_ctrl, weight_rot_control, r_goal_dot, right EE rot slack]\n\
  - soft-constraint: [dist2, dist2, weight_rot_control, dist, right EE rot stack]\n\
  \n\
  - soft-constraint: [r_gripper_ctrl, 0.2, weight_grasp_goal, r_gripper_joint, right EE gripper stack]\n\
  \n\
  # Level elbow and gripper control\n\
  #- soft-constraint: [0.01, 0.01, weight_elbow_control, {y-coord: r_elbow_diff}, right elbow up control slack]\n\
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
      - r_wrist_flex_joint\n\
  - hard-constraint:\n\
      - {double-sub: [0.0, r_gripper_joint]}\n\
      - {double-sub: [0.1, r_gripper_joint]}\n\
      - r_gripper_joint";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client_test");
  //ros::NodeHandle nh("~");
  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<suturo_manipulation_msgs::MoveRobotAction> ac("/graspkard/movement_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  suturo_manipulation_msgs::MoveRobotGoal goal;
  goal.feedbackValue = "dist2";
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
  goal.controlled_joints.push_back("r_gripper_joint");
  goal.controller_yaml = std::string(controller);
  suturo_manipulation_msgs::TypedParam tpFrame, tpWidth, tpHeight;
  tpFrame.isConst = false;
  tpFrame.type = suturo_manipulation_msgs::TypedParam::TRANSFORM;
  tpFrame.name = "object_id";
  tpFrame.value = "cylinder";
  goal.params.push_back(tpFrame);
  
  tpWidth.isConst = true;
  tpWidth.type = suturo_manipulation_msgs::TypedParam::DOUBLE;
  tpWidth.name = "width";
  tpWidth.value = "0.06";
  goal.params.push_back(tpWidth);

  tpHeight.isConst = true;
  tpHeight.type = suturo_manipulation_msgs::TypedParam::DOUBLE;
  tpHeight.name = "height";
  tpHeight.value = "0.2";
  goal.params.push_back(tpHeight);

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


/*
add_executable(client_test src/client_test.cpp)

add_dependencies(client_test 
  graspkard_msgs
  suturo_manipulation_msgs_generate_messages_cpp
  #${graspkard_EXPORTED_TARGETS}
)

target_link_libraries(client_test 
  ${catkin_LIBRARIES}
)*/