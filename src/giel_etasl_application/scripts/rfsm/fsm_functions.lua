local fsm_functions =  {}

function driver_particularities()
  if robot.robot_name == "franka_panda" and not simulation then
    local panda = depl:getPeer("panda")
    panda:low_level_velocity()
  end
end

function moving_cartesian_frame_absolute_config(endframe)
    local endpose = rtt.Variable("KDL.Frame")
    endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
    endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_coordinates.lua")
    etaslcore:set_etaslvar("global.maxvel",0.2)
    etaslcore:set_etaslvar("global.maxacc",0.05)
    etaslcore:set_etaslvar("global.eq_r",0.08)
    etaslcore:set_etaslvar_frame("global.endpose",endpose)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end
