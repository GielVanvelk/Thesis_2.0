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
op_domain = {0.0, 0.5, -0.25, 0.25, 0.07, 0.6} --m

-- specify relevant poses
pose = {0.3, 0, 0.4, 3.14, 0, 3.14} --m

-- check if all poses are safe
pose_safe = check_coordinates(op_domain, pose)
if pose_safe then
	all_safe = true
end

-- velocity and acceleration limits
max_vel = 0.2 --m/s
max_acc= 0.05 --m/s^2
scanning_vel = 0.065 --m/s

-- force Parameters
force_setpoint = 5 --N
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
			state_move_to_pose_WF(pose, max_vel, max_acc)
		end,
		exit=function()
			etaslcore:stop()
			etaslcore:cleanup()
		end,
	},

	-- MOVE ALONG Z_AXIS TO INCREASE FORCE
	adjust_force = rfsm.state {
	    entry=function()

	      force = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_data"))
	      local fs, data= force:read()
	      local force= data

	      solver:create_and_set_solver("etaslcore")
	      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
	      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
	      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/Straight_Line_Contstrained.lua")
	      etaslcore:set_etaslvar("maxvel", scanning_vel)
	      etaslcore:set_etaslvar("maxacc", max_acc)
	      etaslcore:set_etaslvar("eq_r",0.08)
		  etaslcore:set_etaslvar("delta_x", 0)
	      etaslcore:set_etaslvar("delta_y", 0)
	      etaslcore:set_etaslvar("delta_z",0.1)
		  etaslcore:set_etaslvar("force",force_setpoint)
	      etaslcore:configure()
	      etaslcore:initialize()
	      etaslcore:start()
	      driver_particularities()
	    end,
	    exit=function()
	      etaslcore:stop()
	      etaslcore:cleanup()
	    end,
	  },

	-- ================================ Transitions =====================================
	rfsm.trans {src="initial",                   tgt="configured"                                               },
	rfsm.trans {src="configured",                tgt="idle",      			   events={}                        },
	rfsm.trans {src="idle",                      tgt="move_to_home_pose",      events={'idle_exit'}             },
	rfsm.trans {src="move_to_home_pose",         tgt="adjust_force",           events={"e_finished@etaslcore"}  },
	rfsm.trans {src="adjust_force",              tgt="move_to_home_pose",      events={"e_force_good"}          },
}
