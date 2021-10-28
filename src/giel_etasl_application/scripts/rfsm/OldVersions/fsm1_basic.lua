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
starting_pose = {-0.40, 0, 0.15, 3.14, 0, 3.14} --m
end_pose = {-0.50, 0, 0.15, 3.14, 0, 3.14} --m

-- check if all poses are safe
pose_safe = check_coordinates(op_domain, home_pose)
starting_safe = check_coordinates(op_domain, starting_pose)
end_safe = check_coordinates(op_domain, end_pose)
if pose_safe and starting_safe and end_safe then
	all_safe = true
end

-- velocity and acceleration limits
max_vel = 0.02 --m/s
max_acc= 0.02 --m/s^2
scanning_vel = 0.02 --m/s

-- force Parameters
force_setpoint = 5 --N
force_min = 2 --N
force_max = 10 --N


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

	-- ROBOT_IDLE
	robot_idle = rfsm.state {
       entry=function()
             solver:create_and_set_solver("etaslcore")
             etaslcore:readTaskSpecificationFile(robot_etasl_dir)
             etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
             etaslcore:configure()
             etaslcore:initialize()
             etaslcore:start()

       end,
       exit=function()
             etaslcore:stop()
             etaslcore:cleanup()
       end,
    },

	-- IDLE STATE
	idle = rfsm.state {
		entry=function(fsm)
			state_idle(fsm, all_safe)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE TO HOME POSE
	move_to_home_pose = rfsm.state {
		entry=function()
			state_move_to_pose_WF(home_pose, max_vel, max_acc)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE TO HOME POSE
	move_to_starting_pose = rfsm.state {
		entry=function()
			state_move_to_pose_WF(starting_pose, max_vel, max_acc)
			rtt.sleep(2,0) --Sleep for 2 seconds
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE ALONG Z_AXIS TO INCREASE FORCE
	increase_force = rfsm.state {
		entry=function()

			force = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_data"))
			local fs, data= force:read()
			local force = data

			solver:create_and_set_solver("etaslcore")
			etaslcore:readTaskSpecificationFile(robot_etasl_dir)
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/increase_force.lua")
			etaslcore:set_etaslvar("maxvel", scanning_vel)
			etaslcore:set_etaslvar("maxacc", max_acc)
			etaslcore:set_etaslvar("eq_r",0.08)
			etaslcore:set_etaslvar("delta_x", 0)
			etaslcore:set_etaslvar("delta_y", 0)
			etaslcore:set_etaslvar("delta_z", 0.05)
			etaslcore:set_etaslvar("force_setpoint",force_setpoint)
			etaslcore:set_etaslvar("force_max",force_max)
			etaslcore:set_etaslvar("force_min",force_min)
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

	-- MOVE ALONG Z_AXIS TO INCREASE FORCE
	decrease_force = rfsm.state {
		entry=function()

			force = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_data"))
			local fs, data= force:read()
			local force = data

			solver:create_and_set_solver("etaslcore")
			etaslcore:readTaskSpecificationFile(robot_etasl_dir)
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
			etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/decrease_force.lua")
			etaslcore:set_etaslvar("maxvel", scanning_vel)
			etaslcore:set_etaslvar("maxacc", max_acc)
			etaslcore:set_etaslvar("eq_r",0.08)
			etaslcore:set_etaslvar("delta_x", 0)
			etaslcore:set_etaslvar("delta_y", 0)
			etaslcore:set_etaslvar("delta_z", -0.05)
			etaslcore:set_etaslvar("force_setpoint",force_setpoint)
			etaslcore:set_etaslvar("force_max",force_max)
			etaslcore:set_etaslvar("force_min",force_min)
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

	-- STATE: MOVE ALONG TRAJECTORY
    scanning = rfsm.state {
      entry=function(fsm)

        -- convert end_pose to rtt-ready endpose
        local endpose = rtt.Variable("KDL.Frame")
        endpose.p:fromtab{X =end_pose[1] ,Y= end_pose[2],Z= end_pose[3] }
        endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(end_pose[4] , end_pose[5], end_pose[6] )

        solver:create_and_set_solver("etaslcore")
        etaslcore:readTaskSpecificationFile(robot_etasl_dir)
        etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
        etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/StraightLine_ForceSetpoint.lua")
        etaslcore:set_etaslvar("maxvel",scanning_vel)
        etaslcore:set_etaslvar("maxacc",max_acc)
        etaslcore:set_etaslvar("eq_r",0.08)
        etaslcore:set_etaslvar_frame("endpose",endpose)
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





	-- ================================ Transitions =====================================
	rfsm.trans {src="initial",                   tgt="configured"                                               },
	rfsm.trans {src="configured",                tgt="robot_idle",      	   events={}                        },
	--rfsm.trans {src="robot_idle",                tgt="idle",                   events={'e_after(10)'}           },
	rfsm.trans {src="robot_idle",                      tgt="move_to_home_pose",      events={'e_after(10)'}             },
	rfsm.trans {src="move_to_home_pose",         tgt="move_to_starting_pose",  events={'e_finished@etaslcore'}  },
	rfsm.trans {src="move_to_starting_pose",     tgt="increase_force",         events={'e_finished@etaslcore'}  },

	rfsm.trans {src="increase_force",            tgt="scanning",               events={'e_finished@etaslcore'}  },
	rfsm.trans {src="increase_force",            tgt="scanning",               events={'e_forceOK1'}            },


	rfsm.trans {src="scanning",        tgt="move_to_home_pose",      events={'e_finished@etaslcore'}  },
	rfsm.trans {src="scanning",        tgt="increase_force",         events={'e_increase_force'}      },
	rfsm.trans {src="scanning",        tgt="decrease_force",         events={'e_decrease_force'}      },

	rfsm.trans {src="decrease_force",            tgt="scanning",     events={'e_finished@etaslcore'}  },
	rfsm.trans {src="decrease_force",            tgt="scanning",     events={'e_forceOK2'}            },

}
