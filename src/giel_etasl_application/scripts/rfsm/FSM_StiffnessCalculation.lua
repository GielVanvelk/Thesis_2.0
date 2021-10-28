-- ==============================================================================
-- Definition of the state machine
-- KU Leuven 2020
-- ==============================================================================

require("rtt")
require("rttlib")
require("kdlutils")
require("rttros")
require("deployer_utils")

require("rfsm_timeevent")
gettime = rtt.getTime
rfsm_timeevent.set_gettime_hook(gettime)

require("FSM_functions")
require("FSM_states")

gs=rtt.provides()

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
etaslcore   = depl:getPeer("etaslcore")
reporter    = depl:getPeer("Reporter")
solver      = depl:getPeer("solver")

ati_iface = depl:getPeer( "AtiIface" )
soem_master = depl:getPeer( "soem_master" )
gcomp_gui   = depl:getPeer("GielComponent")

depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("etasl_rtt")
ros:import("rtt_rospack")
rttlib.color = true

simulation = tc:getProperty("simulation"):get()
robot_etasl_dir = tc:getProperty("robot_etasl_dir"):get()
depl_robot_file = tc:getProperty("depl_robot_file"):get()
robot = require(depl_robot_file)
joint_pos = robot.home_joint_positions()

cp=rtt.Variable("ConnPolicy")

etasl_application_dir = rtt.provides("ros"):find("giel_etasl_application")

--===========================================================================================
--                                  OPERATOR INPUT
--===========================================================================================

-- Use real robot?
use_real_robot = false

-- Specify safe operating domain.
op_domain = {-0.0, -0.5, -0.25, 0.25, 0.07, 0.6} --{x_lower, x_upper, y_lower, y_upper, z_lower, z_upper} [m]

-- Specify Relevant Poses
home_pose = {-0.40, 0, 0.4, 3.14, 0, 3.14} 			--[m]
starting_pose = {-0.35, -0.1, 0.25, 3.14, 0, 3.14}  --[m]
end_pose = {-0.35, 0.1, 0.25, 3.14, 0, 3.14} 		--[m]

-- Check if all poses are safe.
all_coordinates_safe = check_all_coordinates(op_domain, home_pose, starting_pose, end_pose)

-- Velocity and Acceleration Limits
if use_real_robot then
	max_vel = 0.2 			--[m/s]
	max_acc= 0.2 			--[m/s^2]
	scanning_vel = 0.02 	--[m/s]
else
	max_vel = 0.5 			--[m/s]
	max_acc= 0.5 			--[m/s^2]
	scanning_vel = 0.1 		--[m/s]
end

-- Force Parameters
force_setpoint = -4.0		--[N]
force_min = 2 				--[N]
force_max = 10				--[N]

-- Force/Torque limits
force_torque_limits = {10, 10, 10, 1.5, 1.2, 0.3} --[N, Nm]

-- Sensor Compenstation
sensor_compensation = true
sensor_compensation_type = 1  --0:manual, 1:auto
sensor_comp_sample_size = 10000
reduce_zero_noice = true
reduce_zero_noice_cutoff = 0.01

-- Stiffness
automatic_stiffness_calc = false
custom_stiffness = 0.1


--===========================================================================================
--                                     DEFINE FSM
--===========================================================================================

return rfsm.state {

	--============================================
	--CONFIGURATION STATE
	--============================================
	configured = rfsm.state {
		entry=function(fsm)
			state_configure_robot(fsm)
		end,
	},

	--============================================
	-- SAFETY CHECK STATE
	--============================================
	safety_check = rfsm.state {
	   entry=function(fsm, all_safe_coordinates)
		   print('Doing a safety check.')
		   state_safety_check(fsm)
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
	   end,
	},

	--============================================
	-- GET THE REAL ROBOT READY
	--============================================
	get_robot_ready = rfsm.state {
       entry=function()
		   print('Get the real robot ready.')
		   configure_basic_etaslcore()
		   rtt.sleep(2,0) --Sleep for 2 seconds
       end,
       exit=function()
             etaslcore:stop()
             etaslcore:cleanup()
       end,
    },

	--============================================
	-- ERROR STATE
	--============================================
	error = rfsm.state {
	   entry=function()
		   configure_basic_etaslcore()
		   print(" AN ERROR OCCURED --> PROGRAM STOPPED")
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
	   end,
   },

	--============================================
	-- IDLE STATE
	--============================================
	idle = rfsm.state {
		entry=function(fsm)
		    print('Going idle.')
			state_idle(fsm, sensor_compensation)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	--============================================
	-- SENSOR COMPENSATION STATE
	--============================================
	sensor_compensation = rfsm.state {
		entry=function(fsm)
		    print('Start calculating sensor compensation parameters.')
			state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size, sensor_compensation_type, reduce_zero_noice, reduce_zero_noice_cutoff)
		end,
		doo = function(fsm)
			prop = gcomp_gui:getProperty("sensor_compensation_params_calculated")
		 	while true do
				if prop:get() == 2 then
			   		rfsm.send_events(fsm, "e_sensor_comp")
				end
				rfsm.yield()
				--print("been here 4")
		 	end
	 	end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	--============================================
	-- MOVE TO HOME POSE
	--============================================
	move_to_home_pose = rfsm.state {
	   entry=function()
		   print('Moving to home position.')
		   state_initial_movement(max_vel, max_acc)
		   rtt.sleep(2,0) --Sleep for 2 seconds
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
	   end,
	},

	--============================================
	-- MOVE TO STARTING POSE
	--============================================
	move_to_starting_pose = rfsm.state {
		entry=function()
			print('Moving to start position.')
			state_move_to_pose_WF(starting_pose, max_vel, max_acc, force_torque_limits)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	--============================================
	-- APPLY FORCE
	--============================================
	apply_force = rfsm.state {
		entry=function()
			solver:create_and_set_solver("etaslcore")
			etaslcore:readTaskSpecificationFile(robot_etasl_dir)
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/increase_force.lua")
			etaslcore:set_etaslvar("maxvel", max_vel)
			etaslcore:set_etaslvar("maxacc", max_acc)
			etaslcore:set_etaslvar("eq_r",0.08)
			etaslcore:set_etaslvar("delta_x", 0)
			etaslcore:set_etaslvar("delta_y", 0)
			etaslcore:set_etaslvar("delta_z", 15)
			etaslcore:set_etaslvar("force_setpoint",force_setpoint)
			etaslcore:configure()
			etaslcore:initialize()
			etaslcore:start()
			driver_particularities()
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	--============================================
	-- TRANSITIONS
	--============================================
	rfsm.trans {src="initial",                   tgt="configured"                                                   },
	rfsm.trans {src="configured",                tgt="get_robot_ready",      	   events={"e_config_robot"}        },
	rfsm.trans {src="get_robot_ready",           tgt="safety_check",               events={'e_after(1)'}           },
	rfsm.trans {src="configured",                tgt="safety_check",               events={"e_config_sim"}          },

	rfsm.trans {src="safety_check",              tgt="move_to_home_pose",          events={"e_safety_checked"}      },
	rfsm.trans {src="safety_check",              tgt="error",                      events={"e_safety_error"}        },

	rfsm.trans {src="move_to_home_pose",         tgt="idle",                       events={'e_finished@etaslcore'}  },
	rfsm.trans {src="idle",                      tgt="sensor_compensation",        events={"i_sensor_comp"}         },
	rfsm.trans {src="sensor_compensation",       tgt="idle",                       events={"e_sensor_comp"}         },
	rfsm.trans {src="sensor_compensation",       tgt="error",                      events={"e_sensor_comp_error"}   },
	rfsm.trans {src="idle",                      tgt="move_to_starting_pose",      events={"e_idle"}                },

	rfsm.trans {src="move_to_starting_pose",     tgt="apply_force",              events={'e_finished@etaslcore'}    },
	rfsm.trans {src="apply_force",               tgt="move_to_starting_pose",    events={'e_forceReached'}    },



}
