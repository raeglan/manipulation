scope = {
	unitX = vec3(1,0,0);
	unitY = vec3(0,1,0);
	unitZ = vec3(0,0,1);

	# Joint input
	j_torso_lift    = input(0);

	j_shoulder_pan_l  = input(1);
	j_shoulder_lift_l = input(2);
	j_upper_arm_roll_l= input(3);
	j_elbow_flex_l    = input(4);
	j_forearm_roll_l  = input(5);
	j_wrist_flex_l    = input(6);
	j_wrist_roll_l    = input(7);

	j_shoulder_pan_r  = input(8);
	j_shoulder_lift_r = input(9);
	j_upper_arm_roll_r= input(10);
	j_elbow_flex_r    = input(11);
	j_forearm_roll_r  = input(12);
	j_wrist_flex_r    = input(13);
	j_wrist_roll_r    = input(14);

	# goal input
	pos_goal_l = vec3(input(15), input(16), input(17));
	pos_goal_r = vec3(input(18), input(19), input(20));

	# Robot
	torso_lift     = frame(rotation(unitX, 0),                vec3(-0.05, 0, 0.739675 + j_torso_lift));
	
	shoulder_pan_l   = frame(rotation(unitZ, j_shoulder_pan_l),   vec3(0, 0.188, 0));
	shoulder_lift_l  = frame(rotation(unitY, j_shoulder_lift_l),  vec3(0.1, 0, 0));
	upper_arm_roll_l = frame(rotation(unitX, j_upper_arm_roll_l), vec3(0, 0, 0));
	elbow_flex_l     = frame(rotation(unitY, j_elbow_flex_l),     vec3(0.4, 0, 0));
	forearm_roll_l   = frame(rotation(unitX, j_forearm_roll_l),   vec3(0, 0, 0));
	wrist_flex_l     = frame(rotation(unitY, j_wrist_flex_l),     vec3(0.321, 0, 0));
	wrist_roll_l     = frame(rotation(unitX, j_wrist_roll_l),     vec3(0, 0, 0));
	gripper_offset_l = frame(rotation(unitX, 0),                vec3(0.18, 0, 0));
	
	shoulder_pan_r   = frame(rotation(unitZ, j_shoulder_pan_r),   vec3(0, -0.188, 0));
	shoulder_lift_r  = frame(rotation(unitY, j_shoulder_lift_r),  vec3(0.1, 0, 0));
	upper_arm_roll_r = frame(rotation(unitX, j_upper_arm_roll_r), vec3(0, 0, 0));
	elbow_flex_r     = frame(rotation(unitY, j_elbow_flex_r),     vec3(0.4, 0, 0));
	forearm_roll_r   = frame(rotation(unitX, j_forearm_roll_r),   vec3(0, 0, 0));
	wrist_flex_r     = frame(rotation(unitY, j_wrist_flex_r),     vec3(0.321, 0, 0));
	wrist_roll_r     = frame(rotation(unitX, j_wrist_roll_r),     vec3(0, 0, 0));
	gripper_offset_r = frame(rotation(unitX, 0),                vec3(0.18, 0, 0));

	# Frame chains
	elbow_l = torso_lift * shoulder_pan_l * shoulder_lift_l * upper_arm_roll_l * elbow_flex_l ;
	ee_l = elbow_l * forearm_roll_l * wrist_flex_l * wrist_roll_l * gripper_offset_l ;

	elbow_r = torso_lift * shoulder_pan_r * shoulder_lift_r * upper_arm_roll_r * elbow_flex_r ;
	ee_r = elbow_r * forearm_roll_r * wrist_flex_r * wrist_roll_r * gripper_offset_r ;

	margin = 0.12;

	ee_dist = norm(originOf(ee_l) - originOf(ee_r));
	ee_r_el_dist = norm(originOf(ee_r) - originOf(elbow_l));
	ee_l_el_dist = norm(originOf(ee_l) - originOf(elbow_r));
	elbow_dist = norm(originOf(elbow_r) - originOf(elbow_l));

	ee_dist_err = abs(max(0, ee_dist - margin));
	ee_r_el_dist_err = abs(max(0, ee_r_el_dist - margin));
	ee_l_el_dist_err = abs(max(0, ee_l_el_dist - margin));
	elbow_dist_err = abs(max(0, elbow_dist));

	r_goal_dist = norm(originOf(ee_r) - pos_goal_r);
	l_goal_dist = norm(originOf(ee_l) - pos_goal_l);


	# Feedback
	feedback = gRimX - transXrim + hpDist;

	# Limits, weights and the like
	negVelLimitTorso = -0.02;
	posVelLimitTorso =  0.02;
	negVelLimitArm   = -0.6;
	posVelLimitArm   =  0.6;

	weightTorso = 0.001;
	weightArm   = 0.001;

	weightPositionGoal = 1;
	weightCollisionControl = 1000
}

controllableConstraints = {
	controllableConstraint(negVelLimitTorso, posVelLimitTorso, weightTorso, 0, "j_torso_lift");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 1, "j_shoulder_pan_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 2, "j_shoulder_lift_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 3, "j_upper_arm_roll_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 4, "j_elbow_flex_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 5, "j_forearm_roll_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 6, "j_wrist_flex_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 7, "j_wrist_roll_l");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 8, "j_shoulder_pan_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 9, "j_shoulder_lift_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 10, "j_upper_arm_roll_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 11, "j_elbow_flex_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 12, "j_forearm_roll_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 13, "j_wrist_flex_r");
	controllableConstraint(negVelLimitArm, posVelLimitArm, weightArm, 14, "j_wrist_roll_r")
}

softConstraints = {
	# Self collision
	softConstraint(ee_dist_err, 1000, weightCollisionControl, ee_dist, "Collision avoidance EE");
	softConstraint(ee_r_el_dist_err, 1000, weightCollisionControl, ee_r_el_dist, "Collision avoidance r-EE l-elbow");
	softConstraint(ee_l_el_dist_err, 1000, weightCollisionControl, ee_l_el_dist, "Collision avoidance l-EE r-elbow");
	softConstraint(elbow_dist_err, 1000, weightCollisionControl, elbow_dist, "Collision avoidance elbows");

	# Goals
	softConstraint(-r_goal_dist, -r_goal_dist, weightPositionGoal, r_goal_dist, "r-Goal position");
	softConstraint(-l_goal_dist, -l_goal_dist, weightPositionGoal, l_goal_dist, "l-Goal position")
}	

hardConstraints = {
	hardConstraint(0.0115 - j_torso_lift, 0.325 - j_torso_lift, j_torso_lift);
	# Right arm constraints
	hardConstraint(-2.1353 - j_shoulder_pan_r,  0.5646 - j_shoulder_pan_r,   j_shoulder_pan_r);
	hardConstraint(-0.3536 - j_shoulder_lift_r, 1.2963 - j_shoulder_lift_r,  j_shoulder_lift_r);
	hardConstraint(-3.75   - j_upper_arm_roll_r,  0.65 - j_upper_arm_roll_r, j_upper_arm_roll_r);
	hardConstraint(-2.1213 - j_elbow_flex_r,     -0.15 - j_elbow_flex_r,     j_elbow_flex_r);
	hardConstraint(-2.0    - j_wrist_flex_r,      -0.1 - j_wrist_flex_r,     j_wrist_flex_r);

	# Left arm constraints
	hardConstraint(-0.5646 - j_shoulder_pan_l,  2.1353 - j_shoulder_pan_l,   j_shoulder_pan_l);
	hardConstraint(-0.3536 - j_shoulder_lift_l, 1.2963 - j_shoulder_lift_l,  j_shoulder_lift_l);
	hardConstraint(-0.65   - j_upper_arm_roll_l,  3.75 - j_upper_arm_roll_l, j_upper_arm_roll_l);
	hardConstraint(-2.1213 - j_elbow_flex_l,     -0.15 - j_elbow_flex_l,     j_elbow_flex_l);
	hardConstraint(-2.0    - j_wrist_flex_l,      -0.1 - j_wrist_flex_l,     j_wrist_flex_l);
}