alias basepose=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_upper_body_joint_control.yaml\ config/pr2_upper_body.yaml\ test_params/upper_body_praying_mantis.yaml\ feedback
alias rgrasp=rosrun\ suturo_action_server\ client_test\ controller_specs/gripper_control.yaml\ config/pr2_right_gripper.yaml\ test_params/grasp_r_50.yaml\ feedback
alias lgrasp=rosrun\ suturo_action_server\ client_test\ controller_specs/gripper_control.yaml\ config/pr2_left_gripper.yaml\ test_params/grasp_l_50.yaml\ feedback
alias rrelease=rosrun\ suturo_action_server\ client_test\ controller_specs/gripper_control.yaml\ config/pr2_right_gripper.yaml\ test_params/release_r_50.yaml\ feedback
alias lrelease=rosrun\ suturo_action_server\ client_test\ controller_specs/gripper_control.yaml\ config/pr2_left_gripper.yaml\ test_params/release_l_50.yaml\ feedback
alias rapproach=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_grasp_control_r.yaml\ config/pr2_upper_body_right_arm.yaml\ test_params/approach_cylinder_r.yaml\ feedback
alias lapproach=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_grasp_control_l.yaml\ config/pr2_upper_body_left_arm.yaml\ test_params/approach_cylinder_l.yaml\ feedback
alias rplace=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_place_control_r.yaml\ config/pr2_upper_body_right_arm.yaml\ test_params/place_cylinder_r.yaml\ feedback
alias lplace=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_place_control_l.yaml\ config/pr2_upper_body_left_arm.yaml\ test_params/place_cylinder_l.yaml\ feedback
alias ra_plate=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_grasp_plate_r.yaml\ config/pr2_upper_body_right_arm.yaml\ test_params/approach_plate_r.yaml\ feedback
alias selfcoll=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_self_collision_test.giskard\ config/pr2_upper_body.yaml\ test_params/self_collision_test.yaml\ feedback
alias cutpos_r=rosrun\ suturo_action_server\ client_test\ controller_specs/cut_position.yaml\ config/pr2_upper_body_right_arm.yaml\ test_params/cut_pos.yaml\ feedback
alias cut_r=rosrun\ suturo_action_server\ client_test\ controller_specs/cut.yaml\ config/pr2_upper_body_right_arm.yaml\ test_params/cut_pos.yaml\ feedback

alias rsimple_collision=rosrun\ suturo_action_server\ client_test\ controller_specs/pr2_r_arm_collision_avoidance.giskard\ config/pr2_upper_body_right_arm.yaml\ test_params/r_arm_collision_test.yaml\ feedback