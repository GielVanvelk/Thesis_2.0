local fsm_functions =  {}

function driver_particularities()
  if robot.robot_name == "franka_panda" and not simulation then
    local panda = depl:getPeer("panda")
    panda:low_level_velocity()
  end
end

function create_origin_matrix(point)
  origin_x = point[1]
  origin_y = point[2]
  origin_z = point[3]
  local origin = {origin_x, origin_y, origin_z}
  return origin
end

function create_rot_matrix(point)
  rot_x = point[4]
  rot_y = point[5]
  rot_z = point[6]
  local rot = {rot_x, rot_y, rot_z}
  return rot
end

function moving_cartesian_frame_absolute_config(endframe)
    local endpose = rtt.Variable("KDL.Frame")
    endpose.p:fromtab{X =endframe[1] ,Y= endframe[2],Z= endframe[3] }
    endpose.M = rtt.provides("KDL"):provides("Rotation"):RPY(endframe[4] , endframe[5], endframe[6] )
    solver:create_and_set_solver("etaslcore")
    etaslcore:readTaskSpecificationFile(robot_etasl_dir)
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/default_ports.lua")
    etaslcore:readTaskSpecificationFile(etasl_application_dir.."/scripts/etasl/moving_coordinates.lua")
    etaslcore:set_etaslvar("global.maxvel",0.8)
    etaslcore:set_etaslvar("global.maxacc",0.2)
    etaslcore:set_etaslvar("global.eq_r",0.08)
    etaslcore:set_etaslvar_frame("global.endpose",endpose)
    etaslcore:configure()
    etaslcore:initialize()
    etaslcore:start()
    driver_particularities()
end

function check_force(fsm)

  F2H = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_high"))
  local fs, data= F2H:read()
  local F2H= data
  print("Been here")

  F2L = rttlib.port_clone_conn(gcomp_gui:getPort("out_force_too_low"))
  local fs, data= F2L:read()
  local F2L= data

  if(F2H == false or F2L == true) then
    rfsm.send_events(fsm, 'e_adjust_force')
  end

end
