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

require("fsm_functions")
require("fsm_states")


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
-- SPECIFY THE SAFE OPERATING DOMAIN
-- {x_lower, x_upper, y_lower, y_upper, z_lower, z_upper}
op_domain = {-0.0, -0.5, -0.25, 0.25, 0.07, 0.6} --m

-- specify relevant poses
home_pose = {-0.40, 0, 0.4, 3.14, 0, 3.14} --m
starting_pose = {-0.35, -0.1, 0.25, 3.14, 0, 3.14} --m
end_pose = {-0.35, 0.1, 0.25, 3.14, 0, 3.14} --m

-- check if all poses are safe
pose_safe = check_coordinates(op_domain, home_pose)
starting_safe = check_coordinates(op_domain, starting_pose)
end_safe = check_coordinates(op_domain, end_pose)
if pose_safe and starting_safe and end_safe then
	all_safe = true
end

-- velocity and acceleration limits
max_vel = 0.2 --0.05 --m/s
max_acc= 0.2 --0.05 --m/s^2
scanning_vel = 0.02 -- 0.02 --m/s

-- force Parameters
force_setpoint = -4.0--N
force_min = 2 --N
force_max = 10 --N

-- sensor Compenstation
sensor_compensation = true
sensor_compensation_type = 1  --0:manual, 1:auto
sensor_comp_sample_size = 1000
reduce_zero_noice = true;
reduce_zero_noice_cutoff = 0.01;


--===========================================================================================
--                                     DEFINE FSM
--===========================================================================================

return rfsm.state {

	-- CONFIGURATION STATE
	configured = rfsm.state {
		entry=function()
			state_configure_robot()
		end,
	},

	-- GET THE REAL ROBOT READY
	get_robot_ready = rfsm.state {
       entry=function()
		   configure_basic_etaslcore()
		   rtt.sleep(2,0) --Sleep for 2 seconds
       end,
       exit=function()
             etaslcore:stop()
             etaslcore:cleanup()
       end,
    },

	-- ERROR
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

	-- IDLE STATE
	idle = rfsm.state {
		entry=function(fsm)
			state_idle(fsm, all_safe, sensor_compensation, sensor_comp_sample_size, sensor_compensation_type, reduceZeroNoice, reduce_zero_noice_cutoff)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		doo = function(fsm)
			sc = gcomp_gui:getProperty("sensor_compensation_params_calculated")
			sc_value = sc:get()
         while true do
            if sc_val == 2 then
               rfsm.send_events(fsm, "idle_exit")
            end
            rfsm.yield()
         end
	 end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE TO HOME POSE
	move_to_home_pose = rfsm.state {
	   entry=function()
		   state_initial_movement(max_vel, max_acc)
		   rtt.sleep(2,0) --Sleep for 2 seconds
	   end,
	   exit=function()
			 etaslcore:stop()
			 etaslcore:cleanup()
	   end,
	},

	-- MOVE TO STARTING POSE
	move_to_starting_pose = rfsm.state {
		entry=function()
			print('Moving to start position')
			state_move_to_pose_WF(starting_pose, max_vel, max_acc)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE ALONG Z_AXIS TO INCREASE FORCE
	initial_force = rfsm.state {
		entry=function()
			print('Adjusting initial force')
			state_increase_force(scanning_vel, max_acc, force_setpoint, 0.05)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- STATE: US SCANNING
    scanning = rfsm.state {
      entry=function(fsm)
		  print('Start scanning')
		  rtt.sleep(2,0) --Sleep for 2 seconds
		  state_scanning(end_pose, scanning_vel, max_acc, force_setpoint)
		  rtt.sleep(2,0) --Sleep for 2 seconds
      end,
      exit=function()
        etaslcore:stop()
        etaslcore:cleanup()
      end,
    },

	-- ================================ Transitions =====================================
	rfsm.trans {src="initial",                   tgt="configured"                                                   },
  --rfsm.trans {src="configured",                tgt="get_robot_ready",      	   events={}                        },
  --rfsm.trans {src="get_robot_ready",           tgt="idle",                       events={'e_after(10)'}           },
	rfsm.trans {src="configured",                tgt="idle",                       events={}                        },

	rfsm.trans {src="idle",                      tgt="move_to_home_pose",          events={"idle_exit"}             },
	rfsm.trans {src="idle",                      tgt="error",                      events={"idle_error"}            },

	rfsm.trans {src="move_to_home_pose",         tgt="move_to_starting_pose",      events={'e_finished@etaslcore'}  },
	rfsm.trans {src="move_to_starting_pose",     tgt="initial_force",              events={'e_finished@etaslcore'}  },

  --rfsm.trans {src="initial_force",             tgt="scanning",                   events={'e_forceReached'}      },
	rfsm.trans {src="initial_force",             tgt="scanning",                   events={'e_finished@etaslcore'}  },
  --rfsm.trans {src="initial_force",             tgt="error",                      events={'e_finished@etaslcore'}  },

    rfsm.trans {src="scanning",                  tgt="move_to_home_pose",          events={'e_finished@etaslcore'}  },
	rfsm.trans {src="scanning",                  tgt="error",                      events={'force_error_x'}         },
	rfsm.trans {src="scanning",                  tgt="error",                      events={'force_error_y'}         },
	rfsm.trans {src="scanning",                  tgt="error",                      events={'force_error_z'}         },
}
