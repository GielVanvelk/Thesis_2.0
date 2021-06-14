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


gs=rtt.provides()

tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
etaslcore   = depl:getPeer("etaslcore")
reporter    = depl:getPeer("Reporter")
solver      = depl:getPeer("solver")

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

-- ====================================== eTaSL components ===================================

-- ====================================== Solver
--ros:import("etasl_solver_qpoases")
--depl:loadComponent("solver","etasl_solver_qpoases")
--solver = depl:getPeer("solver")

-- ====================================== jointstate I/O factories
--os:import("etasl_iohandler_jointstate")
--depl:loadComponent("jointstate","Etasl_IOHandler_Jointstate")
--jointstate = depl:getPeer("jointstate")

-- ====================================== eTaSL core
--ros:import("etasl_rtt")
--depl:loadComponent("etaslcore", "etasl_rtt")
--create LuaComponents
--etaslcore = depl:getPeer("etaslcore")
--depl:connectPeers("etaslcore","solver")
--depl:connectPeers("etaslcore","jointstate")

etasl_application_dir = rtt.provides("ros"):find("giel_etasl_application")

--===========================================================================================
--                                     DEFINE FSM
--===========================================================================================

return rfsm.state {

  -- STATE: CONFIGURE KUKA ROBOT
  configured = rfsm.state {
    entry=function()
      -- ================================ Configuration for Kuka iiwa: The driver of the iiwa needs to send zero velocities and wait a bit to configure ============
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
      -- ================================ General configuration ============================
      reporter:configure()
      reporter:start()
    end,
  },

  -- STATE: IDLE
  idle = rfsm.state {
    entry=function()
      solver:create_and_set_solver("etaslcore")
      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
      etaslcore:configure()
      etaslcore:initialize()
      etaslcore:start()
    end,
    doo = function(fsm)
      while true do
        rtt.sleep(2,0) --Sleep for 2 seconds
        rfsm.send_events(fsm,"idle_exit")
        rfsm.yield()
      end
    end,
    exit=function()
      etaslcore:stop()
      etaslcore:cleanup()
    end,
  },

  -- STATE: ERROR
  error = rfsm.state {
    entry=function()
      solver:create_and_set_solver("etaslcore")
      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
      etaslcore:configure()
      etaslcore:initialize()
      etaslcore:start()
    end,
    doo = function(fsm)
      while true do
        rtt.sleep(5,0) --Sleep for 2 seconds
        print ("Error detected - try to reset")
        rfsm.send_events(fsm,"error_exit")
        rfsm.yield()
      end
    end,
    exit=function()
      etaslcore:stop()
      etaslcore:cleanup()
    end,
  },

  -- STATE: MOVE TO STARTING POSE
  move_to_starting_pose = rfsm.state {
    entry=function()
      print("Moving to starting pose")
      moving_cartesian_frame_absolute_config({0.3, 0.3, 0.4, 3.14, 0, 3.14})
    end,
    exit=function()
      etaslcore:stop()
      etaslcore:cleanup()
      print("Arrived at starting pose")
    end,
  },

  -- STATE: MOVE TO HOME POSE
  move_to_home_pose = rfsm.state {
    entry=function()
      print("Moving to home pose")
      rtt.sleep(2,0)
      moving_cartesian_frame_absolute_config({0.4, 0, 0.6, 3.14, 0, 3.14})
    end,
    exit=function()
      etaslcore:stop()
      etaslcore:cleanup()
      print("Moved to home pose")
    end,
  },

  -- STATE: MOVE ALONG TRAJECTORY
  move_straight_line = rfsm.state {
    entry=function()

      F2H = gcomp_gui:getPort("out_force_too_high")
      local fs, data= F2H:read()
      local F2H= data

      F2L = gcomp_gui:getPort("out_force_too_low")
      local fs, data= F2L:read()
      local F2L= data

      print("Started Scanning")
      solver:create_and_set_solver("etaslcore")
      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_straight_line.lua")
      etaslcore:set_etaslvar("global.maxvel",0.03)
      etaslcore:set_etaslvar("global.maxacc",0.05)
      etaslcore:set_etaslvar("global.eq_r",0.08)
      etaslcore:set_etaslvar("global.delta_x",0)
      etaslcore:set_etaslvar("global.delta_y",0.3)
      etaslcore:set_etaslvar("global.delta_z",0)
      etaslcore:configure()
      etaslcore:initialize()
      etaslcore:start()
      driver_particularities()
    end,
    exit=function()
      etaslcore:stop()
      etaslcore:cleanup()
      print("Scanning completed")
    end,
  },

  -- STATE: ADJUST FORCE [NOT COMPLETED]
  adjust_force = rfsm.state {
    entry=function()
      solver:create_and_set_solver("etaslcore")
      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_straight_line.lua")
      etaslcore:set_etaslvar("global.maxvel",0.05)
      etaslcore:set_etaslvar("global.maxacc",0.05)
      etaslcore:set_etaslvar("global.eq_r",0.08)
      etaslcore:set_etaslvar("global.delta_x",0.0)
      etaslcore:set_etaslvar("global.delta_y",0.0)
      etaslcore:set_etaslvar("global.delta_z",0.01)
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

  -- STATE: ADJUST POSE [NOT COMPLETED]
  adjust_pose = rfsm.state {
    entry=function()
      solver:create_and_set_solver("etaslcore")
      etaslcore:readTaskSpecificationFile(robot_etasl_dir)
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
      etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_straight_line.lua")
      etaslcore:set_etaslvar("global.maxvel",0.05)
      etaslcore:set_etaslvar("global.maxacc",0.05)
      etaslcore:set_etaslvar("global.eq_r",0.08)
      etaslcore:set_etaslvar("global.delta_x",0.0)
      etaslcore:set_etaslvar("global.delta_y",0.0)
      etaslcore:set_etaslvar("global.delta_z",0.1)
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
  rfsm.trans {src="initial",                   tgt="configured"                                                   },
  rfsm.trans {src="configured",                tgt="idle",                       events={}                        },
  rfsm.trans {src="idle",                      tgt="move_to_starting_pose",      events={'idle_exit'}             },
  rfsm.trans {src="move_to_starting_pose",     tgt="move_straight_line",         events={'e_finished@etaslcore'}  },
  rfsm.trans {src="move_straight_line",        tgt="move_to_home_pose",          events={'e_finished@etaslcore'}  },
  rfsm.trans {src="move_to_home_pose",         tgt="idle",                       events={'e_finished@etaslcore'}  },

  rfsm.trans {src="move_straight_line",        tgt="adjust_force",               events={'e_adjust_force'}        },
  rfsm.trans {src="adjust_force",              tgt="move_straight_line",         events={'e_adjusted_force'}      },

  rfsm.trans {src="move_straight_line",        tgt="adjust_pose",                events={'e_adjust_force'}        },
  rfsm.trans {src="adjust_pose",               tgt="move_straight_line",         events={'e_adjusted_pose'}       },

}
