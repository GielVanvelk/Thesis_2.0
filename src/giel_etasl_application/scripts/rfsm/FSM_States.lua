local FSM_Functions =  {}

-- =================================================================================================================
-- CONFIGURE ROBOT STATE
-- checks if the real robot is used and configures it if neccesary
-- =================================================================================================================
function state_configure_robot(fsm, sensor_tool_frame_transf)
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
	p1 = gcomp_gui:getProperty("sensor_tool_transx")
	p1:set(sensor_tool_frame_transf[1])
	p2 = gcomp_gui:getProperty("sensor_tool_transy")
	p2:set(sensor_tool_frame_transf[2])
	p3 = gcomp_gui:getProperty("sensor_tool_transz")
	p3:set(sensor_tool_frame_transf[3])
	p4 = gcomp_gui:getProperty("sensor_tool_rotx")
	p4:set(sensor_tool_frame_transf[4])
	p5 = gcomp_gui:getProperty("sensor_tool_roty")
	p5:set(sensor_tool_frame_transf[5])
	p6 = gcomp_gui:getProperty("sensor_tool_rotz")
	p6:set(sensor_tool_frame_transf[6])
	p7 = gcomp_gui:getProperty("sensor_tool_rotz")
	p7:set(sensor_tool_frame_transf[6])

	-- Select correct transition
	if simulation then
		rfsm.send_events(fsm, "e_config_sim")
	else
		rfsm.send_events(fsm, "e_config_robot")
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
	rtt.sleep(2,0)
end

-- =================================================================================================================
-- INITIAL MOVEMENT STATE/ MOVE TO HOME POSE
-- initial movement of the robot based on joint positions
-- =================================================================================================================
function state_initial_movement(max_vel, max_acc, force_torque_limits)
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
-- IDLE STATE
-- just keep the robot still
-- =================================================================================================================
function state_idle(fsm, sensor_compensation)
	rtt.sleep(2,0)
	--transitions
	p1 = gcomp_gui:getProperty("sensor_compensation_params_calculated")
	if sensor_compensation and p1:get() == 0 then
		rfsm.send_events(fsm, "i_sensor_comp") -- move to sensor compensation state
	else
		rfsm.send_events(fsm, "e_idle") -- move to next state
	end
end

-- =================================================================================================================
-- SENSOR COMPENSATION STATE
-- determine the parameters for the sensor compensation
-- =================================================================================================================
function state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size, sensor_compensation_type, reduce_zero_noice, reduce_zero_noice_cutoff, use_fifo_buffer, fifo_buffer_size)
	configure_basic_etaslcore()
	sc1 = gcomp_gui:getProperty("sensor_compensation")
	sc1:set(sensor_compensation)
	sc2 = gcomp_gui:getProperty("sensor_compensation_type")
	sc2:set(sensor_compensation_type)
	sc3 = gcomp_gui:getProperty("sensor_compensation_sample_size")
	sc3:set(sensor_comp_sample_size)
	sc4 = gcomp_gui:getProperty("sensor_compensation_params_calculated")
	sc5 = gcomp_gui:getProperty("reduce_zero_noice")
	sc5:set(reduce_zero_noice)
	sc6 = gcomp_gui:getProperty("reduce_zero_noice_cutoff")
	sc6:set(reduce_zero_noice_cutoff)

	if sc4:get() == 0 then
		sc4:set(1)
	elseif sc4:get() == 2 then
		sc7 = gcomp_gui:getProperty('use_fifo_buffer')
		sc7:set(use_fifo_buffer)
		sc8 = gcomp_gui:getProperty('fifo_buffer_size')
		sc8:set(fifo_buffer_size)
		rfsm.send_events(fsm, "e_sensor_comp")
	else
		rfsm.send_events(fsm, "e_sensor_comp_error")
		print("ERROR: Wrong flag for calculation sensor compensation parameters")
	end
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
-- STIFFNESS CALCULATION STATE
-- apply force to calculate the stiffness
-- =================================================================================================================
function state_stiffness_calculation(max_vel, max_acc, max_z, force_torque_limits, force_setpoint, force_start, force_stop)
	stiffness_calc_force_start = gcomp_gui:getProperty("stiffness_calc_force_start")
	stiffness_calc_force_start:set(force_start)
	--print(force_stop)

	stiffness_calc = gcomp_gui:getProperty("stiffness_calculation")
	stiffness_calc:set(1)

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
-- US SCANNING
-- US Scanning while controlling the force
-- =================================================================================================================
function state_scanning(endframe, scanning_vel, max_acc, force_setpoint, stiffness_val, force_torque_limits)
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
	etaslcore:set_etaslvar("global.stiffness_val",k_val)
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




--=====================================================================================
--=====================================================================================
--=====================================================================================
-- =================================================================================================================
-- INTIAL FORCE
-- apply a force of ?N Newton
-- =================================================================================================================
function state_initial_force(max_vel, max_acc, max_z, force_torque_limits, force_start, force_stop)
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
-- STATE FOR STIFFNESS TESTING
-- apply a timedependend force to calculate the stiffness
-- =================================================================================================================
function state_stiffness_test(max_vel, max_acc, max_z, k_val, force_torque_limits, force_start, force_stop, oscillation_freq, time_max, K_controller)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Stiffness_Test.lua")
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
