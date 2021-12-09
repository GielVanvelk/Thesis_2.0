-- =================================================================================================================
-- STATE MOVE TO FORCE SETPOINT
-- move to the desired force setpoint
-- =================================================================================================================
function state_move_to_force(max_vel, max_acc, max_z, force_torque_limits, force_setpoint, movement_direction)
	solver:create_and_set_solver("etaslcore")
	etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Move_To_Force.lua")
	etaslcore:set_etaslvar("maxvel", max_vel)
	etaslcore:set_etaslvar("maxacc", max_acc)
	etaslcore:set_etaslvar("eq_r",0.08)
	etaslcore:set_etaslvar("delta_x", 0)
	etaslcore:set_etaslvar("delta_y", 0)
	etaslcore:set_etaslvar("delta_z", max_z)
	etaslcore:set_etaslvar("force_setpoint",force_setpoint)
	etaslcore:set_etaslvar("movement_direction",movement_direction)
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
