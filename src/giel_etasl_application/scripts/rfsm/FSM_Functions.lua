local fSM_States =  {}

function driver_particularities()
	if robot.robot_name == "franka_panda" and not simulation then
		local panda = depl:getPeer("panda")
		panda:low_level_velocity()
	end
end

function set_state_transition_flag(value)
	tr_flag = gcomp_gui:getProperty("pr_transition_flag")
	tr_flag:set(value)
end

function create_origin_matrix(pose)
	origin_x = pose[1]
	origin_y = pose[2]
	origin_z = pose[3]
	local origin = {origin_x, origin_y, origin_z}
	return origin
end

function create_rot_matrix(pose)
	rot_x = pose[4]
	rot_y = pose[5]
	rot_z = pose[6]
	local rot = {rot_x, rot_y, rot_z}
	return rot
end

function configure_basic_etaslcore()
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
end

function publish_pose()
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Publish_Pose.lua")
	etaslcore:configure()
	etaslcore:initialize()
	etaslcore:start()
	driver_particularities()
end

function check_all_coordinates(op_domain, pose1, pose2, pose3)

	pose1_safe = check_coordinates(op_domain, pose1)
	pose2_safe = check_coordinates(op_domain, pose2)
	pose3_safe = check_coordinates(op_domain, pose3)
	if pose1_safe and pose2_safe and pose3_safe then
		all_coordinates_safe = true
	end

	return all_coordinates_safe

end

function check_coordinates(op_domain, pose)

	-- local variable
	local isSafe = false

	-- Check x coordinates
	if pose[1] >= op_domain[2] and  pose[1] <= op_domain[1] then
		if pose[2] >= op_domain[3] and pose[2] <= op_domain[4] then
			if pose[3] >= op_domain[5] and pose[3] <= op_domain[6] then
				isSafe = true
			end
		end
	end

	return isSafe

end

function set_transition_flag(flag)
	tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
	tr_flag:set(flag)
end

function set_stiffness_calculation_flag(flag)
	stiffness_calc = gcomp_gui:getProperty("pr_stiffness_calculation_flag")
	stiffness_calc:set(flag)
end

function evaluate_control_parameters(fsm, stiffness, controller_gain, stiffness_limits, controller_gain_limits)
	print('   >>> The stiffness that will be used is:',stiffness,'N/m')
	if stiffness >= stiffness_limits[1] and stiffness <= stiffness_limits[2] then
		print('   >>> The stiffness looks OK!')
		print('   >>> The controller gain that will be used is:',controller_gain,'-')
		if controller_gain >= controller_gain_limits[1] and controller_gain <= controller_gain_limits[2] then
			print('   >>> The controller gain looks OK!')
			if move_to_setpoint_first then
				rfsm.send_events(fsm, "e_checkCP1")
			else
				rfsm.send_events(fsm, "e_checkCP2")
			end
		else
			print('   >>> The controller gain seems wrong --> assuming an error!')
			rfsm.send_events(fsm, "e_checkCP_error")
		end
	else
		print('   >>> The stiffness seems wrong --> assuming an error!')
		rfsm.send_events(fsm, "e_checkCP_error")
	end
	rtt.sleep(2,0)
end



--==========================================================================
--==========================================================================
--==========================================================================
--==========================================================================


function check_force(fsm)

	F2H = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_high"))
	local fs, data= F2H:read()
	local F2H= data
	print("Been here")

	F2L = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_low"))
	local fs, data= F2L:read()
	local F2L= data

	if(F2H == false or F2L == true) then
		rfsm.send_events(fsm, 'e_adjust_force')
	end

end

function sensor_compensation()

	F2H = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_high"))
	local fs, data= F2H:read()
	local F2H= data
	print("Been here")

	F2L = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_low"))
	local fs, data= F2L:read()
	local F2L= data

	if(F2H == false or F2L == true) then
		rfsm.send_events(fsm, 'e_adjust_force')
	end

end
