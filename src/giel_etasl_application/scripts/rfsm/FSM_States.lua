local FSM_Functions =  {}

-- =================================================================================================================
-- CONFIGURE ROBOT STATE
-- checks if the real robot is used and configures it if neccesary
-- =================================================================================================================
function state_configure_robot(fsm, sensor_tool_frame_transf, use_real_robot)
	-- Configuration for Kuka iiwa: The driver of the iiwa needs to send zero velocities and wait a bit to configure
	if robot.robot_name == "kuka_lwr" and not simulation then
		local lwr = depl:getPeer("lwr")
		local port_vel = rttlib.port_clone_conn(lwr:getPort("JointVelocityCommand"))
		depl:import("rtt_motion_control_msgs")
		local vel_values = rtt.Variable("motion_control_msgs.JointVelocities")
		vel_values.names = robot.joint_names()
		vel_values.velocities:fromtab({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
		port_vel:write(vel_values)
		rtt.sleep(2,0) --Sleep for 2 seconds
	end
	-- General configuration
	reporter:configure()
	reporter:start()

	-- Set neccesary Properties
	p1 = gcomp_gui:getProperty("pr_sensor_tool_transx")
	p1:set(sensor_tool_frame_transf[1])
	p2 = gcomp_gui:getProperty("pr_sensor_tool_transy")
	p2:set(sensor_tool_frame_transf[2])
	p3 = gcomp_gui:getProperty("pr_sensor_tool_transz")
	p3:set(sensor_tool_frame_transf[3])
	p4 = gcomp_gui:getProperty("pr_sensor_tool_rotx")
	p4:set(sensor_tool_frame_transf[4])
	p5 = gcomp_gui:getProperty("pr_sensor_tool_roty")
	p5:set(sensor_tool_frame_transf[5])
	p6 = gcomp_gui:getProperty("pr_sensor_tool_rotz")
	p6:set(sensor_tool_frame_transf[6])
	p7 = gcomp_gui:getProperty("pr_sensor_tool_rotz")
	p7:set(sensor_tool_frame_transf[6])

	-- Select correct transition
	if simulation == false or use_real_robot == true then
		rfsm.send_events(fsm, "e_config_robot")
	else
		rfsm.send_events(fsm, "e_config_sim")
	end
end

-- =================================================================================================================
-- SAFETY CHECK
-- checks if all relevent coordinates are withing the safe operating domain
-- =================================================================================================================
function state_safety_check(fsm, safe_coordinates)
	publish_pose()
	-- Check if all poses are safe.
	if safe_coordinates == false then
		rfsm.send_events(fsm, "e_safety_error")
		print("ERROR: Specified coordinates not in the safe operating domain")
	else
		rfsm.send_events(fsm, "e_safety_checked")
	end
end

-- =================================================================================================================
-- INITIAL MOVEMENT STATE/ MOVE TO HOME POSE
-- initial movement of the robot based on joint positions
-- =================================================================================================================
function state_move_to_home_pose(max_vel, max_acc, force_torque_limits)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Moving_Jointspace_Trap.lua")
	etaslcore:set_etaslvar("global.maxvel",max_vel)
	etaslcore:set_etaslvar("global.maxacc",max_acc)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
		  for i=1,#joint_pos do
				etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
		  end
	end
	for i=1,#joint_pos do
		  etaslcore:set_etaslvar("global.end_j"..i,joint_pos[i])
	end
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- MOVE TO POSE STATE
-- pose to move to a defined pose in the world frame
-- =================================================================================================================
function state_move_to_pose_WF(endframe, max_vel, max_acc, force_torque_limits)
	local endpose = rtt.Variable("KDL.Frame")
	endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
	endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Moving_To_Coordinates.lua")
	etaslcore:set_etaslvar("global.maxvel",max_vel)
	etaslcore:set_etaslvar("global.maxacc",max_acc)
	etaslcore:set_etaslvar("global.eq_r",0.08)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:set_etaslvar_frame("global.endpose",endpose)
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- SENSOR COMPENSATION STATE
-- determine the parameters for the sensor compensation
-- =================================================================================================================
function state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size)
	configure_basic_etaslcore()
	sc1 = gcomp_gui:getProperty("pr_sensor_compensation")
	sc1:set(sensor_compensation)
	sc3 = gcomp_gui:getProperty("pr_sensor_compensation_sample_size")
	sc3:set(sensor_comp_sample_size)
	sc4 = gcomp_gui:getProperty("pr_sensor_compensation_flag")
	if sc4:get() == 0 then
		sc4:set(1)
	elseif sc4:get() == 2 then
		rfsm.send_events(fsm, "e_sensor_comp")
	else
		rfsm.send_events(fsm, "e_sensor_comp_error")
		print("ERROR: Wrong flag for calculation sensor compensation parameters")
	end
end

-- =================================================================================================================
-- INTIAL FORCE
-- apply a force of ?N Newton
-- =================================================================================================================
function state_initial_force(max_vel, max_acc, max_z, force_torque_limits, force_start, force_stop)
	pr1 = gcomp_gui:getProperty("pr_stiffness_force_start")
	pr1:set(force_start)
	pr2 = gcomp_gui:getProperty("pr_stiffness_force_stop")
	pr2:set(force_stop)
	pr3 = gcomp_gui:getProperty("pr_stiffness_calculation_flag")
	pr3:set(1)

	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Initial_Force.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("delta_x", 0)
	etaslcore:set_etaslvar("delta_y", 0)
	etaslcore:set_etaslvar("delta_z", max_z)
	force_set = (force_stop + force_start)/2
	etaslcore:set_etaslvar("force_setpoint",force_set)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- STATE FOR OSCILLATING MOVEMENT
-- apply a timedependend force to calculate the stiffness
-- =================================================================================================================
function state_oscillation(max_vel, max_acc, max_z, k_val, force_torque_limits, force_start, force_stop, oscillation_freq, time_max, K_controller)
	pr1 = gcomp_gui:getProperty("pr_stiffness_force_start")
	pr1:set(force_start)
	pr2 = gcomp_gui:getProperty("pr_stiffness_force_stop")
	pr2:set(force_stop)
	pr3 = gcomp_gui:getProperty("pr_stiffness_calculation_flag")
	pr3:set(1)

	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Force_Oscillation.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("stiffness_val",k_val)
	etaslcore:set_etaslvar("force_stop",force_stop)
	etaslcore:set_etaslvar("force_start", force_start)
	etaslcore:set_etaslvar("time_max", time_max)
	etaslcore:set_etaslvar("freq", oscillation_freq)
	etaslcore:set_etaslvar("Kc", K_controller)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- STIFFNESS CALCULATION STATE
-- apply force to calculate the stiffness
-- =================================================================================================================
function state_stiffness_calculation(max_vel, max_acc, max_z, force_torque_limits, force_setpoint, force_start, force_stop)
	pr1 = gcomp_gui:getProperty("pr_stiffness_force_start")
	pr1:set(force_start)
	pr2 = gcomp_gui:getProperty("pr_stiffness_force_stop")
	pr2:set(force_stop)
	pr3 = gcomp_gui:getProperty("pr_stiffness_calculation_flag")
	pr3:set(1)

	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Stiffness_Calculation.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("delta_x", 0)
	etaslcore:set_etaslvar("delta_y", 0)
	etaslcore:set_etaslvar("delta_z", max_z)
	etaslcore:set_etaslvar("force_stop",force_stop)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- STATE MOVE TO FORCE SETPOINT
-- move to the desired force setpoint
-- =================================================================================================================
function state_move_to_forceZ(max_vel, max_acc, max_z, force_torque_limits, force_setpoint, stiffness, controller_gain, force_tolerance)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Force_Control.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("delta_x", 0)
	etaslcore:set_etaslvar("delta_y", 0)
	etaslcore:set_etaslvar("delta_z", 0)
	etaslcore:set_etaslvar("force_setpoint",force_setpoint)
	etaslcore:set_etaslvar("stiffness_val",stiffness)
	etaslcore:set_etaslvar("K_controller",controller_gain)
	etaslcore:set_etaslvar("force_tolerance",force_tolerance)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- US SCANNING
-- US Scanning while controlling the force
-- =================================================================================================================
function state_scanning(endframe, scanning_vel, max_acc, force_setpoint, stiffness, force_torque_limits, controller_gain)
	local endpose = rtt.Variable("KDL.Frame")
	endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
	endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Scanning.lua")
	etaslcore:set_etaslvar("global.maxvel",scanning_vel)
	etaslcore:set_etaslvar("global.maxacc",max_acc)
	etaslcore:set_etaslvar("global.eq_r",0.08)
	etaslcore:set_etaslvar("global.force_set",force_setpoint)
	etaslcore:set_etaslvar("global.stiffness_val",stiffness)
	etaslcore:set_etaslvar("global.K_controller",controller_gain)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	etaslcore:set_etaslvar_frame("global.endpose",endpose)
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

-- =================================================================================================================
-- STATE FINISHED
-- Robot moves back to home position using joint positions and stops there
-- =================================================================================================================
function state_finished(max_vel, max_acc, force_torque_limits)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Moving_Jointspace_Trap.lua")
	etaslcore:set_etaslvar("global.maxvel",max_vel)
	etaslcore:set_etaslvar("global.maxacc",max_acc)
	etaslcore:set_etaslvar("global.force_x_limits", force_torque_limits[1])
	etaslcore:set_etaslvar("global.force_y_limits", force_torque_limits[2])
	etaslcore:set_etaslvar("global.force_z_limits", force_torque_limits[3])
	etaslcore:set_etaslvar("global.torque_x_limits", force_torque_limits[4])
	etaslcore:set_etaslvar("global.torque_y_limits", force_torque_limits[5])
	etaslcore:set_etaslvar("global.torque_z_limits", force_torque_limits[6])
	if robot.is_continuous_joints then --If the function exists it modifies it. Otherwise the defaults are used (non continuous)
		  for i=1,#joint_pos do
				etaslcore:set_etaslvar("global.continuous_j"..i,robot.is_continuous_joints()[i])
		  end
	end
	for i=1,#joint_pos do
		  etaslcore:set_etaslvar("global.end_j"..i,joint_pos[i])
	end
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end
