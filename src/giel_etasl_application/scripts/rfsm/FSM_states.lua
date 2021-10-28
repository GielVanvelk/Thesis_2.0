local FSM_functions =  {}

-- CONFIGURE ROBOT STATE
-- checks if the real robot is used and configures it if neccesary
--
function state_configure_robot(fsm)
	-- Configuration for Kuka iiwa: The driver of the iiwa needs to send zero velocities and wait a bit to configure
	if robot.robot_name == "kuka_lwr" and not simulation then
		local lwr = depl:getPeer("lwr")
		local port_vel = rttlib.port_clone_conn(lwr:getPort("JointVelocityCommand"))
		depl:import("rtt_motion_control_msgs")
		local vel_values = rtt.Variable("motion_control_msgs.JointVelocities")
		vel_values.names = robot.joint_names()
		vel_values.velocities:fromtab({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})
		-- print(vel_values)
		port_vel:write(vel_values)
		rtt.sleep(2,0) --Sleep for 2 seconds
	end
	-- General configuration
	reporter:configure()
	reporter:start()
	-- Select correct transition
	if simulation then
		rfsm.send_events(fsm, "e_config_sim")
	else
		rfsm.send_events(fsm, "e_config_robot")
	end
end

-- SAFETY CHECK STATE
-- checks if all relevent coordinates are withing the safe operating domain
--
function state_safety_check(fsm, safe_coordinates)
	if all_coordinates_safe == false then
		rfsm.send_events(fsm, "e_safety_error")
		print("ERROR: Specified coordinates not in the safe operating domain")
	else
		rfsm.send_events(fsm, "e_safety_checked")
	end
end

-- IDLE STATE
-- just keep the robot still
--
function state_idle(fsm, sensor_compensation)
	configure_basic_etaslcore()
	rtt.sleep(2,0)
	--transitions
	sc = gcomp_gui:getProperty("sensor_compensation_params_calculated")
	if sensor_compensation and sc:get() == 0 then
		rfsm.send_events(fsm, "i_sensor_comp") -- move to sensor compensation state
	else
		rfsm.send_events(fsm, "e_idle") -- move to next state
	end
end

-- SENSOR COMPENSATION STATE
-- determine the parameters for the sensor compensation
--
function state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size, sensor_compensation_type, reduce_zero_noice, reduce_zero_noice_cutoff)
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
		rfsm.send_events(fsm, "e_sensor_comp")
	else
		rfsm.send_events(fsm, "e_sensor_comp_error")
		print("ERROR: Wrong flag for calculation sensor compensation parameters")
	end
end

-- INITIAL MOVEMENT STATE
-- initial movement of the robot based on joint positions
--
function state_initial_movement(max_vel, max_acc)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_jointspace_trap.lua")
	etaslcore:set_etaslvar("global.maxvel",max_vel)
	etaslcore:set_etaslvar("global.maxacc",max_acc)
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

-- MOVE TO POSE STATE
-- pose to move to a defined pose in the world frame
--
function state_move_to_pose_WF(endframe, max_vel, max_acc, force_torque_limits)
	local endpose = rtt.Variable("KDL.Frame")
	endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
	endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Moving_Coordinates_WF.lua")
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

--=====================================================================================
--=====================================================================================
--=====================================================================================

-- TEST STATE: FORCE CONTROL
-- move up and down to regulate force
--
function state_force_control(max_vel, max_acc, force_setpoint)
	solver:create_and_set_solver("etaslcore")
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
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
	rtt.sleep(2,0) --Sleep for 2 seconds
end

-- TEST STATE: APPLY FORCE
-- apply the desired force and calculate the stiffness
--
function state_increase_force(max_vel, max_acc, force_setpoint, max_z, stiffness)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Stiffness_calculation.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("delta_x", 0)
	etaslcore:set_etaslvar("delta_y", 0)
	etaslcore:set_etaslvar("delta_z", max_z)
	etaslcore:set_etaslvar("force_setpoint",force_setpoint)
	--etaslcore:set_etaslvar("stiffness", stiffness)
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
	rtt.sleep(2,0) --Sleep for 2 seconds


	configure_basic_etaslcore()
	stiffness_calc = gcomp_gui:getProperty("stifness_calculation")
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
		rfsm.send_events(fsm, "e_sensor_comp")
	else
		rfsm.send_events(fsm, "e_sensor_comp_error")
		print("ERROR: Wrong flag for calculation sensor compensation parameters")
	end





end

--=====================================================================================
--=====================================================================================
--=====================================================================================

function state_scanning(end_pose, scanning_vel, max_acc, force_setpoint)

	-- convert end_pose to rtt-ready endpose
	local endpose = rtt.Variable("KDL.Frame")
	endpose.p:fromtab{X =end_pose[1] ,Y= end_pose[2],Z= end_pose[3] }
	endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(end_pose[4] , end_pose[5], end_pose[6] )

	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Scanning.lua")
	etaslcore:set_etaslvar("maxvel",scanning_vel)
	etaslcore:set_etaslvar("maxacc",max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar_frame("endpose",endpose)
	etaslcore:set_etaslvar("force_set",force_setpoint)
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
	rtt.sleep(2,0) --Sleep for 2 seconds
end
