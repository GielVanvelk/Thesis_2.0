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
op_domain = {0.0, 0.6, -0.3, 0.3, 0.3, 0.6} --m

-- specify relevant poses
pose = {-0.45, 0, 0.35, 3.14, 0, 3.14} --m
print(pose[1])
print(pose[2])
print(pose[3])

-- check if all poses are safe
pose_safe = check_coordinates(op_domain, pose)
if pose_safe then
	all_safe = true
end

all_safe = true

-- velocity and acceleration limits
max_vel = 0.02 --m/s
max_acc= 0.02 --m/s^2
scanning_vel = 0.02 --m/s

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

	moving_joint_space_1 = rfsm.state {
	   entry=function()
			 solver:create_and_set_solver("etaslcore")
			 etaslcore:readTaskSpecificationFile(robot_etasl_dir)
			 etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
			 etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_jointspace_trap.lua")
			 etaslcore:set_etaslvar("global.maxvel",0.08)
			 etaslcore:set_etaslvar("global.maxacc",0.08)
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

	-- ================================ Transitions =====================================
	rfsm.trans {src="initial",                   tgt="configured"                                               },
	rfsm.trans {src="configured",                tgt="robot_idle",      			   events={}                        },
	rfsm.trans {src="robot_idle",                tgt="idle",  events={'e_after(10)'}             },
	rfsm.trans {src="idle",                      tgt="moving_joint_space_1",  events={'idle_exit'}             },
	rfsm.trans {src="moving_joint_space_1",     tgt="move_to_home_pose",       events={'e_finished@etaslcore'} },
}
