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

require("FSM_Functions")
require("FSM_States")
require("FSM_UserInput")

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
--                                     DEFINE FSM
--===========================================================================================

return rfsm.state {

	--============================================
	-- CONFIGURATION STATE [COMPLETED]
	--============================================
	configured = rfsm.state {
		entry=function(fsm)
			state_configure_robot(fsm, sensor_tool_frame_transf, use_real_robot)
		end,
	},

   --============================================
   -- GET THE REAL ROBOT READY [COMPLETED]
   --============================================
   get_robot_ready = rfsm.state {
		 entry=function()
			 print('Get the real robot ready.')
			 rtt.sleep(1,0)
  		   	 configure_basic_etaslcore()
		 end,
		 exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
			 set_state_transition_flag(1)
		 end,
	 },

	--============================================
 	-- SAFETY CHECK STATE [COMPLETED]
 	--============================================
 	safety_check = rfsm.state {
 	   entry=function(fsm)
		   print('Doing a safety check.')
 		   rtt.sleep(1,0)
		   set_state_transition_flag(0)
 		   state_safety_check(fsm, all_safe_coordinates)
 	   end,
 	   exit=function()
 			 etaslcore:stop()
 			 etaslcore:cleanup()
			 set_state_transition_flag(1)
 	   end,
 	},

	--============================================
 	-- MOVE TO HOME POSE [COMPLETED]
 	--============================================
 	move_to_home_pose = rfsm.state {
 	   entry=function()
		   print('Moving to home pose.')
 		   rtt.sleep(1,0)
		   set_state_transition_flag(0)
 		   state_move_to_home_pose(max_vel, max_acc, force_torque_limits)
 	   end,
 	   exit=function()
 		   etaslcore:stop()
		   etaslcore:cleanup()
		   set_state_transition_flag(1)
 	   end,
 	},

	--============================================
	-- MOVE TO STARTING POSE [COMPLETED]
	--============================================
	move_to_starting_pose = rfsm.state {
		entry=function(fsm)
			if testpoint_flag == testpoints_amount then
				rfsm.send_events(fsm, "e_finished")
			end
			starting_pose[2] = testpoint_distance * testpoint_flag
			print('Moving to start position.')
			rtt.sleep(1,0)
			set_state_transition_flag(0)
			state_move_to_pose_WF(starting_pose, max_vel, max_acc, force_torque_limits)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			set_state_transition_flag(1)
			testpoint_flag = testpoint_flag + 1
		end,
	},

	--============================================
	-- SENSOR COMPENSATION STATE [COMPLETED]
	--============================================
	sensor_compensation = rfsm.state {
		entry=function(fsm)
			print('Start calculating sensor compensation parameters.')
			rtt.sleep(1,0)
			set_state_transition_flag(0)
			state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size)
		end,
		doo = function(fsm)
			prop = gcomp_gui:getProperty("pr_sensor_compensation_flag")
		 	while true do
				if prop:get() == 2 then
			   		rfsm.send_events(fsm, "e_sensor_comp")
				end
				rfsm.yield()
		 	end
	 	end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			set_state_transition_flag(1)
		end,
	},

	--============================================
	-- INITIAL FORCE [COMPLETED]
	--============================================
	initial_force= rfsm.state {
		entry=function()
			print('Applying the initial force.')
			rtt.sleep(1,0)
			set_state_transition_flag(0)
			state_initial_force(stiffness_calc_vel, max_acc, max_z_stiffness_calculation, force_torque_limits, a_k_calc_force_start, a_k_calc_force_stop)
		end,
		exit=function()
			set_stiffness_calculation_flag(2)
			etaslcore:stop()
			etaslcore:cleanup()
			set_state_transition_flag(1)

			pr1 = gcomp_gui:getProperty("pr_calculated_stiffness")
			print('Point:',testpoint_flag,'Calculated stiffness:',pr1:get(),'[N/m]')
		end,
	},

	--============================================
	-- OSCILLATION MOVEMENT [COMPLETED]
	--============================================
	oscillation = rfsm.state {
		entry=function(fsm)
			print('Starting Oscillating Movement.')
			rtt.sleep(1,0)
			set_state_transition_flag(0)
			state_oscillation(max_vel, max_acc, max_z, stiffness_custom, force_torque_limits, a_k_calc_force_start, a_k_calc_force_stop, oscillation_freq, testing_time, controller_gain)
		end,
		exit=function(fsm)
			set_stiffness_calculation_flag(2)
			etaslcore:stop()
			etaslcore:cleanup()
			set_state_transition_flag(1)

			pr1 = gcomp_gui:getProperty("pr_calculated_stiffness")
			print('Point:',testpoint_flag,'Calculated stiffness:',pr1:get(),'[N/m]')
		end,
	},

	--============================================
	-- FINISHED: MOVE BACK HOME AND STOP [COMPLETED]
	--============================================
	finished = rfsm.state {
		entry=function()
			rtt.sleep(1,0)
			set_state_transition_flag(0)
			print('FINISHED: MOVING HOME AND STOPPING')
			state_finished(max_vel, max_acc, force_torque_limits)
	    end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			set_state_transition_flag(1)
		end,
	},

	--============================================
	-- ERROR STATE [COMPLETED]
	--============================================
	error = rfsm.state {
	   entry=function()
		   print("AN ERROR OCCURED --> PROGRAM STOPPED")
		   rtt.sleep(1,0)
		   set_state_transition_flag(0)
		   configure_basic_etaslcore()
	   end,
	   exit=function()
		   set_state_transition_flag(1)
		   etaslcore:stop()
		   etaslcore:cleanup()
	   end,
   },

	--============================================
	-- TRANSITION
	--============================================
	rfsm.trans {src="initial",                   tgt="configured"                                                   },
	rfsm.trans {src="configured",                tgt="get_robot_ready",      	   events={"e_config_robot"}        },
	rfsm.trans {src="get_robot_ready",           tgt="safety_check",               events={'e_after(10)'}           },
	rfsm.trans {src="configured",                tgt="safety_check",               events={"e_config_sim"}          },

	rfsm.trans {src="safety_check",              tgt="move_to_home_pose",          events={"e_safety_checked"}      },
	rfsm.trans {src="safety_check",              tgt="error",                      events={"e_safety_error"}        },

	rfsm.trans {src="move_to_home_pose",         tgt="move_to_starting_pose",      events={'e_finished@etaslcore'}  },
	rfsm.trans {src="move_to_starting_pose",     tgt="sensor_compensation",        events={'e_finished@etaslcore'}  },

	rfsm.trans {src="sensor_compensation",       tgt="error",                      events={"e_sensor_comp_error"}   },
	rfsm.trans {src="sensor_compensation",       tgt="initial_force",               events={'e_sensor_comp'}  		},

	rfsm.trans {src="initial_force",             tgt="error",   				   events={'e_finished@etaslcore'}  },
	rfsm.trans {src="initial_force",             tgt="oscillation",     		   events={'e_forceReached'}        },
	rfsm.trans {src="initial_force",             tgt="error",     		           events={'e_force_too_high'}      },
	rfsm.trans {src="initial_force",             tgt="error",     		           events={'e_torque_too_high'}     },

	rfsm.trans {src="oscillation",               tgt="error",     			       events={'e_force_too_high'}      },
	rfsm.trans {src="oscillation",               tgt="error",     			       events={'e_torque_too_high'}     },
	rfsm.trans {src="oscillation",               tgt="move_to_starting_pose",      events={'e_finished@etaslcore'}  },
	rfsm.trans {src="move_to_starting_pose",     tgt="finished",                   events={'e_finished'}            },
}
