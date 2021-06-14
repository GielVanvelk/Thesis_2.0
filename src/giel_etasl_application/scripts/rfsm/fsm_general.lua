-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- Definition of the state machine
-- KU Leuven 2020
-- ==============================================================================

require("rtt")
require("rttlib")
-- require("rfsm_timeevent")
--
-- gettime = rtt.getTime
-- rfsm_timeevent.set_gettime_hook(gettime)


tc          = rtt.getTC()
depl        = tc:getPeer("Deployer")
etaslcore   = depl:getPeer("etaslcore")
reporter    = depl:getPeer("Reporter")
solver      = depl:getPeer("solver")

simulation = tc:getProperty("simulation"):get()
robot_etasl_dir = tc:getProperty("robot_etasl_dir"):get()
depl_robot_file = tc:getProperty("depl_robot_file"):get()
robot = require(depl_robot_file)
joint_pos = robot.home_joint_positions()

function driver_particularities()
  if robot.robot_name == "franka_panda" and not simulation then
    local panda = depl:getPeer("panda")
    panda:low_level_velocity()
  end

end

etasl_application_dir = rtt.provides("ros"):find("giel_etasl_application")
return rfsm.state {
   configured = rfsm.state {
      entry=function()
        -- ================================Configuration for Kuka iiwa: The driver of the iiwa needs to send zero velocities and wait a bit to configure ============
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

   idle = rfsm.state {
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

   moving_joint_space_1 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.1)
            etaslcore:set_etaslvar("global.maxacc",0.1)
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

   moving_in_task_frame_coordinates_1 = rfsm.state {
         entry=function()
               solver:create_and_set_solver("etaslcore")
               etaslcore:readTaskSpecificationFile(robot_etasl_dir)
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_in_task_frame_coordinates.lua")
               etaslcore:set_etaslvar("global.maxvel",0.1)
               etaslcore:set_etaslvar("global.maxacc",0.05)
               etaslcore:set_etaslvar("global.eq_r",0.08)
               etaslcore:set_etaslvar("global.delta_x",0.0)
               etaslcore:set_etaslvar("global.delta_y",0.0)
               etaslcore:set_etaslvar("global.delta_z",-0.1)
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

   moving_spline = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_spline.lua")
            etaslcore:set_etaslvar("global.maxvel",0.05)
            etaslcore:set_etaslvar("global.maxacc",0.05)
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


   moving_in_task_frame_coordinates_2 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_in_task_frame_coordinates.lua")
            etaslcore:set_etaslvar("global.maxvel",0.1)
            etaslcore:set_etaslvar("global.maxacc",0.05)
            etaslcore:set_etaslvar("global.eq_r",0.08)
            etaslcore:set_etaslvar("global.delta_x",0.0)
            etaslcore:set_etaslvar("global.delta_y",0.0)
            etaslcore:set_etaslvar("global.delta_z",0.05)
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

   moving_joint_space_2 = rfsm.state {
      entry=function()
            solver:create_and_set_solver("etaslcore")
            etaslcore:readTaskSpecificationFile(robot_etasl_dir)
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
            etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_jointspace_trap.lua")
            etaslcore:set_etaslvar("global.maxvel",0.1)
            etaslcore:set_etaslvar("global.maxacc",0.1)
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

   admittance = rfsm.state {
         entry=function()
               solver:create_and_set_solver("etaslcore")
               etaslcore:readTaskSpecificationFile(robot_etasl_dir)
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_admittance.lua")
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

   moving_learned_model = rfsm.state {
         entry=function()
               solver:create_and_set_solver("etaslcore")
               etaslcore:readTaskSpecificationFile(robot_etasl_dir)
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
               etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_learned_model.lua")
               etaslcore:set_etaslvar("global.maxvel",0.1)
               etaslcore:set_etaslvar("global.maxacc",0.1)
               etaslcore:set_etaslvar("global.c_tgt_x",0.8)
               etaslcore:set_etaslvar("global.c_tgt_y",0.15)
               etaslcore:set_etaslvar("global.c_tgt_z",0.1)
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

   rfsm.trans {src="initial", tgt="configured" },


-- ============================== RUN sequence spline ===================================
rfsm.trans {src="configured",                         tgt="moving_joint_space_1",               events={}},
rfsm.trans {src="moving_joint_space_1",               tgt="moving_in_task_frame_coordinates_1", events={"e_finished@etaslcore"}},
rfsm.trans {src="moving_in_task_frame_coordinates_1", tgt="moving_spline",                      events={"e_finished@etaslcore"}},
rfsm.trans {src="moving_spline",                      tgt="moving_in_task_frame_coordinates_2", events={"e_finished@etaslcore"}},
rfsm.trans {src="moving_in_task_frame_coordinates_2", tgt="moving_joint_space_2",               events={"e_finished@etaslcore"}},

-- -- ============================== RUN sequence learned model ===================================
--    rfsm.trans {src="configured",                         tgt="moving_joint_space_1",               events={}},
--    rfsm.trans {src="moving_joint_space_1",               tgt="moving_learned_model",               events={"e_finished@etaslcore"}},
--    rfsm.trans {src="moving_learned_model",               tgt="moving_in_task_frame_coordinates_2", events={"e_finished@etaslcore"}},
--    rfsm.trans {src="moving_in_task_frame_coordinates_2", tgt="moving_joint_space_2",               events={"e_finished@etaslcore"}},
}
