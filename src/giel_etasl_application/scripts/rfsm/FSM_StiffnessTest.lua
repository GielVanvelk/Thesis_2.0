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
	-- CONFIGURATION STATE
	--============================================
	configured = rfsm.state {
		entry=function(fsm)
			state_configure_robot(fsm, sensor_tool_frame_transf)
		end,
	},

	--============================================
	-- ERROR STATE
	--============================================
	error = rfsm.state {
	   entry=function()
		   rtt.sleep(1,0) --Sleep for 2 seconds
		   tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
		   tr_flag:set(0)
		   configure_basic_etaslcore()
		   print(" AN ERROR OCCURED --> PROGRAM STOPPED")
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
			 tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			 tr_flag:set(1)
	   end,
   },

   --============================================
   -- GET THE REAL ROBOT READY
   --============================================
   get_robot_ready = rfsm.state {
		 entry=function()
			 rtt.sleep(1,0) --Sleep for 2 seconds
			 tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			 tr_flag:set(0)
			 print('Get the real robot ready.')
  		   	 configure_basic_etaslcore()
		 end,
		 exit=function()
			   etaslcore:stop()
			   etaslcore:cleanup()
			   tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			   tr_flag:set(1)
		 end,
	 },

	--============================================
	-- SAFETY CHECK STATE
	--============================================
	safety_check = rfsm.state {
	   entry=function(fsm)
		   rtt.sleep(1,0) --Sleep for 2 seconds
		   tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
		   tr_flag:set(0)
		   print('Doing a safety check.')
		   state_safety_check(fsm, all_safe_coordinates)
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
			 tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			 tr_flag:set(1)
	   end,
	},

	--============================================
	-- MOVE TO HOME POSE
	--============================================
	move_to_home_pose = rfsm.state {
	   entry=function()
		   rtt.sleep(1,0) --Sleep for 2 seconds
		   tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
		   tr_flag:set(0)
		   print('Moving to home position.')
		   state_initial_movement(max_vel, max_acc, force_torque_limits)
		   rtt.sleep(2,0) --Sleep for 2 seconds
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
			 tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			 tr_flag:set(1)
	   end,
	},

	--============================================
	-- IDLE STATE
	--============================================
	idle = rfsm.state {
		entry=function(fsm)
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
			print('Going idle.')
			state_idle(fsm, sensor_compensation)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
		end,
	},

	--============================================
	-- SENSOR COMPENSATION STATE
	--============================================
	sensor_compensation = rfsm.state {
		entry=function(fsm)
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
		    print('Start calculating sensor compensation parameters.')
			state_sensor_compensation(fsm, sensor_compensation, sensor_comp_sample_size, sensor_compensation_type, reduce_zero_noice, reduce_zero_noice_cutoff, use_fifo_buffer, fifo_buffer_size)
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
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
		end,
	},

	--============================================
	-- MOVE TO STARTING POSE
	--============================================
	move_to_starting_pose = rfsm.state {
		entry=function(fsm)
			if testpoint_flag == testpoints_amount then
				rfsm.send_events(fsm, "e_finished")
			end
			starting_pose[2] = testpoint_distance * testpoint_flag
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
			print('Moving to start position.')
			state_move_to_pose_WF(starting_pose, max_vel, max_acc, force_torque_limits)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
			testpoint_flag = testpoint_flag + 1
		end,
	},

	--============================================
	-- Initial Force
	--============================================
	initial_force= rfsm.state {
		entry=function()
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
			print('Applying the initial force.')
			state_initial_force(stiffness_calc_vel, max_acc, max_z_stiffness_calculation, force_torque_limits, a_k_calc_force_start, a_k_calc_force_stop)
		end,
		exit=function()
			stiffness_calc = gcomp_gui:getProperty("stiffness_calculation")
			stiffness_calc:set(2)
			etaslcore:stop()
			etaslcore:cleanup()
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
		end,
	},

	--============================================
	-- STIFFNESS TEST
	--============================================
	stiffness_test = rfsm.state {
		entry=function(fsm)
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
			print('Starting Oscillating Movement.')
			k_val = custom_stiffness
			state_stiffness_test(max_vel, max_acc, max_z, k_val, force_torque_limits, a_k_calc_force_start, a_k_calc_force_stop, oscillation_freq, testing_time, K_controller)
		end,
		exit=function(fsm)
			etaslcore:stop()
			etaslcore:cleanup()
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
		end,
	},

	--============================================
	-- FINISHED: MOVE BACK HOME AND STOP
	--============================================
	finished = rfsm.state {
		entry=function()
			rtt.sleep(1,0) --Sleep for 2 seconds
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(0)
		   print('FINISHED: MOVING HOME AND STOPPING')
 		   state_finished(max_vel, max_acc, force_torque_limits)
 		   rtt.sleep(2,0) --Sleep for 2 seconds
	    end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
			tr_flag = gcomp_gui:getProperty("pr_state_transition_flag")
			tr_flag:set(1)
		end,
	},

	--============================================
	-- TRAN
	--============================================
	rfsm.trans {src="initial",                   tgt="configured"                                                   },
	rfsm.trans {src="configured",                tgt="get_robot_ready",      	   events={"e_config_robot"}        },
	rfsm.trans {src="get_robot_ready",           tgt="safety_check",               events={'e_after(10)'}           },
	rfsm.trans {src="configured",                tgt="safety_check",               events={"e_config_sim"}          },

	rfsm.trans {src="safety_check",              tgt="move_to_home_pose",          events={"e_safety_checked"}      },
	rfsm.trans {src="safety_check",              tgt="error",                      events={"e_safety_error"}        },

	rfsm.trans {src="move_to_home_pose",         tgt="idle",                       events={'e_finished@etaslcore'}  },
	rfsm.trans {src="idle",                      tgt="sensor_compensation",        events={"i_sensor_comp"}         },
	rfsm.trans {src="sensor_compensation",       tgt="idle",                       events={"e_sensor_comp"}         },
	rfsm.trans {src="sensor_compensation",       tgt="error",                      events={"e_sensor_comp_error"}   },
	rfsm.trans {src="idle",                      tgt="move_to_starting_pose",      events={"e_idle"}                },

	rfsm.trans {src="move_to_starting_pose",     tgt="initial_force",              events={'e_finished@etaslcore'}  },
	rfsm.trans {src="initial_force",             tgt="error",   				   events={'e_finished@etaslcore'}  },
	rfsm.trans {src="initial_force",             tgt="stiffness_test",     		   events={'e_forceReached'}        },

	rfsm.trans {src="stiffness_test",            tgt="error",     			       events={'e_force_too_high'}      },
	rfsm.trans {src="stiffness_test",            tgt="error",     			       events={'e_torque_too_high'}     },
	rfsm.trans {src="stiffness_test",            tgt="move_to_starting_pose",      events={'e_finished@etaslcore'}  },
	rfsm.trans {src="move_to_starting_pose",     tgt="finished",                   events={'e_finished'}            },

	--rfsm.trans {src="finished",                  tgt="move_to_starting_pose",      events={'e_restart'}             },
}
