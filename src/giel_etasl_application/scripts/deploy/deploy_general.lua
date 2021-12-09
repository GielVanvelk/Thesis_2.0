-- ==============================================================================
-- Author: Cristian Vergara
-- email: <cristian.vergara@kuleuven.be>
-- KU Leuven 2020
-- ==============================================================================

require "rttlib"
require "rttros"
require "deployer_utils"

-- ====================================== User Parameters =========================================

robot_name = "kuka_lwr"
use_jr3 = false
freq = 1000
sensorfreq = 1/freq

-- ====================================== Standard deployment stuff =========================================
rtt.setLogLevel("Warning")

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end
depl:import("rtt_ros")
ros = gs:provides("ros")
ros:import("etasl_rtt")
ros:import("rtt_rospack")
rttlib.color = true

etasl_application_dir = rtt.provides("ros"):find("giel_etasl_application")
robot_def_dir = etasl_application_dir .. "/scripts/etasl/robot_def"

-- The following will make run always in simulation, unless you provide "deploy_robot as the first argument"
-- Example to run in real robot: rttlua -i deploy_general.lua "real_robot"
if arg[1] == "real_robot" then
  simulation = false
else
  simulation = true
end


cp=rtt.Variable("ConnPolicy")

-- ====================================== Robot Hardware definition =========================================
-- robot = require("etasl_UR10")
-- robot = require("etasl_kinova_gen3")
depl_robot_file,robot_etasl_dir = determine_robot(robot_name)
robot = require(depl_robot_file)

-- ====================================== eTaSL components ===================================
-- ====================================== Solver
ros:import("etasl_solver_qpoases")
depl:loadComponent("solver","etasl_solver_qpoases")
solver = depl:getPeer("solver")

-- ====================================== jointstate I/O factories
ros:import("etasl_iohandler_jointstate")
depl:loadComponent("jointstate","Etasl_IOHandler_Jointstate")
jointstate = depl:getPeer("jointstate")

-- ====================================== eTaSL core
ros:import("etasl_rtt")
depl:loadComponent("etaslcore", "etasl_rtt")
-- create LuaComponents
etaslcore = depl:getPeer("etaslcore")
depl:connectPeers("etaslcore","solver")
depl:connectPeers("etaslcore","jointstate")

-- ====================================== Output ports task
etaslcore:add_etaslvar_outputport("tf_pose","Executed pose of the task frame",s{"x_tf","y_tf","z_tf","roll_tf","pitch_tf","yaw_tf"})
etaslcore:add_etaslvar_inputport("force_sensor", "Force read from force sensor", s{"Fx_raw", "Fy_raw", "Fz_raw", "Tx_raw", "Ty_raw", "Tz_raw"}, d{0,0,0,0,0,0})
etaslcore:add_etaslvar_inputport("force_compensated", "Compensated force signal", s{"Fx_comp", "Fy_comp", "Fz_comp", "Tx_comp", "Ty_comp", "Tz_comp"}, d{0,0,0,0,0,0})
etaslcore:add_etaslvar_outputport("Fz_des","The desired force in the Z-direction",s{"Fz_desired"})
etaslcore:add_etaslvar_outputport("path_coordinate","Information of the path coordinate",s{"s"})

-- ====================================== Configure eTaSL ports for the robot
robot.create_etasl_ports(etaslcore,jointstate)

depl:setActivity("etaslcore", 1/freq, 50, rtt.globals.ORO_SCHED_RT)

-- ====================================== simulation
if simulation then
-- deploy simulated robot:
    depl:loadComponent("simrobot", "OCL::LuaComponent")
    simrobot = depl:getPeer("simrobot")
    simrobot:exec_file(etasl_application_dir.."/scripts/components/simple_robot_sim.lua")
    init_jnts = robot.initial_joints_states()
    simrobot:getProperty("initial_position"):set( init_jnts )
    depl:setActivity("simrobot", 1/freq, 50, rtt.globals.ORO_SCHED_RT)
    depl:connect("etaslcore.jointvel","simrobot.jointvel",cp )
    depl:connect("simrobot.jointpos","etaslcore.jointpos",cp )
    simrobot:configure()
    simrobot:start()

else
-- ====================================== Real robot
    robot.deploy_driver(1/freq)
end

-- ====================================== Reporter =========================================
function exists(file)
  local ok, err, code = os.rename(file, file)
  if not ok then
if code == 13 then
  -- Permission denied, but it exists
  return true
end
  end
  return ok
end

function isdir(path)
  -- "/" works on both Unix and Windows
  return exists(path.."/")
end

date = os.date("%Y_%m_%d")
tmstamp = os.date("%Y_%m_%d_%H_%M_%S")
dir_name = "/reports_from_" .. date
dir_path = etasl_application_dir .. "/reports/all_data" .. dir_name

file_name = "/report_of_" .. tmstamp ..'.dat'

if not isdir(dir_path) then
  os.execute("mkdir -p " .. dir_path )
  print('Directory ' .. dir_name .. ' created')
end

depl:loadComponent("Reporter","OCL::FileReporting")
reporter=depl:getPeer("Reporter")
depl:connectPeers("etaslcore","Reporter")
reporter:reportPort("etaslcore","jointvel")
reporter:reportPort("etaslcore","tf_pose")
reporter:reportPort("etaslcore","path_coordinate")
reporter:getProperty("ReportFile"):set(dir_path .. file_name)

-- =====================　seom master + force sensor + components =========================================================　
--

dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/SoemMasterComponent_properties.lua")
dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/AtiIface_properties.lua" )
dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/Application_properties.lua" )

depl:import( "ati_iface" )
depl:loadComponent( "AtiIface", "ati::AtiIface" )
ati_iface = depl:getPeer( "AtiIface" )

depl:setActivity( ati_iface:getName( ), sensorfreq, 50, rtt.globals.ORO_SCHED_RT)
ati_iface:getProperty( "scale_matrix" ):set( scale_matrix )
ati_iface:getProperty( "calibration_matrix" ):set( calibration_matrix )
ati_iface:getProperty( "lowpass_filter_on" ):set( lowpass_filter_on )
ati_iface:getProperty( "filter_bandwidth" ):set( filter_bandwidth )
ati_iface:getProperty( "filter_order" ):set( filter_order )
ati_iface:getProperty( "compensate_deadband" ):set( compensate_deadband )
ati_iface:getProperty( "f_deadband" ):set( f_deadband )
ati_iface:getProperty( "m_deadband" ):set( m_deadband )
ati_iface:configure()

depl:import( "soem_master" )
depl:import( "soem_beckhoff_drivers" )
depl:loadComponent( "soem_master", "soem_master::SoemMasterComponent" )
soem_master = depl:getPeer( "soem_master" )

depl:setActivity( soem_master:getName( ), sensorfreq, 50, rtt.globals.ORO_SCHED_RT)
soem_master:getProperty( "ifname" ):set( ifname )
soem_master:configure( )
if not soem_master:isConfigured( ) then -- Switch the interface names in case soem_master doesn't configure.
    soem_master:getProperty( "ifname" ):set( ifname2 )
    soem_master:configure( )
end

cp = rtt.Variable( "ConnPolicy" )
depl:connect( soem_master:getName( )..".Slave_1002.values", ati_iface:getName( )..".in_ai1_port", cp )
depl:connect( soem_master:getName( )..".Slave_1003.values", ati_iface:getName( )..".in_ai2_port", cp )
soem_master:start( )
depl:connect(ati_iface:getName( )..".out_W_ati","etaslcore.force_sensor",cp )
depl:stream( ati_iface:getName( )..".out_W_ati" , rtt.provides( "ros" ):topic("/wrench_raw") )
ati_iface:start( )
--

-- ====================================== Giel Component
depl:import("giel_component") --folder name
depl:loadComponent("GielComponent","gcomp::GielComponent") --import component

gcomp_gui = depl:getPeer("GielComponent")
depl:setActivity(gcomp_gui:getName(), sensorfreq, 50, rtt.globals.ORO_SCHED_RT)

--depl:stream( gcomp_gui:getName( )..".in_force_data" , rtt.provides( "ros" ):topic("chatter") ) -- for mock data publisher

-- CONNECTION TO THE POSE OUTPUT OF ETASLCORE
depl:connect(gcomp_gui:getName( )..".in_pose_data","etaslcore.tf_pose",cp )
depl:stream( gcomp_gui:getName( )..".in_pose_data" , rtt.provides( "ros" ):topic("tf_pose") ) -- define were gcomp_gui can find a port

-- CONNECTION TO THE FORCE SENSOR
depl:connect(gcomp_gui:getName( )..".in_force_data", ati_iface:getName( )..".out_W_ati" ,cp ) -- to connect the output of ati-iface to the input of gcomp_gui
depl:stream( gcomp_gui:getName( )..".in_force_data" , rtt.provides( "ros" ):topic("/wrench_raw") ) -- define were gcomp_gui can find a port

-- CONNECTION TO THE FORCE INPUT OF ETASLCORE
depl:connect(gcomp_gui:getName( )..".out_force_data","etaslcore.force_compensated",cp )
depl:stream( gcomp_gui:getName( )..".out_force_data" , rtt.provides( "ros" ):topic("G_forceCompensated") )

-- CONNECTION TO THE FORCE INPUT OF ETASLCORE
--depl:connect(gcomp_gui:getName( )..".out_pose_data","etaslcore.force_compensated",cp )
depl:stream( gcomp_gui:getName( )..".out_pose_data" , rtt.provides( "ros" ):topic("G_tf_pose") )

-- WRENCH AND POSE
depl:stream( gcomp_gui:getName( )..".out_wrench" , rtt.provides( "ros" ):topic("G_wrench") )
depl:stream( gcomp_gui:getName( )..".out_pose" , rtt.provides( "ros" ):topic("G_pose") )

-- WRENCH AND POSE FOR STIFFNESS CALCULATION
depl:stream( gcomp_gui:getName( )..".out_stiffness_wrench" , rtt.provides( "ros" ):topic("G_stiffness_wrench") )
depl:stream( gcomp_gui:getName( )..".out_stiffness_pose" , rtt.provides( "ros" ):topic("G_stiffness_pose") )

-- TRANSITION FLAG
depl:stream( gcomp_gui:getName( )..".out_transition_flag" , rtt.provides( "ros" ):topic("G_state_trans") )

gcomp_gui:configure()
gcomp_gui:start()

-- ====================================== Supervisor =========================================
depl:loadComponent("Supervisor", "OCL::LuaComponent")
sup = depl:getPeer("Supervisor")

define_property( sup, "simulation", "bool", simulation, "Boolean value to set simulation mode" )
define_property( sup, "robot_etasl_dir", "string", robot_etasl_dir, "Directory of the etasl robot definition" )
define_property( sup, "depl_robot_file", "string", depl_robot_file, "Directory of the file containing deployment of the robot" )

sup:exec_file(etasl_application_dir.."/scripts/components/fsm_component.lua")
sup:getProperty("state_machine"):set(etasl_application_dir.."/scripts/rfsm/FSM_ScanningTests.lua")

sup:getProperty("viz_on"):set(false)
sup:addPeer(depl)
depl:setActivity("Supervisor", 1/freq, 50, rtt.globals.ORO_SCHED_RT) --needed to use the time events of rfsm
sup:configure()
sup:start()
cmd = rttlib.port_clone_conn(sup:getPort("events"))

-- connect ports:
if not simulation then
  robot.connect_ports_driver(etaslcore,1/freq)
end
depl:connect("etaslcore.eventPort","Supervisor.events",cp)
depl:stream("etaslcore.joint_state", ros:topic("/joint_states"))
depl:stream("etaslcore.Fz_des", ros:topic("/Fz_desired"))
function restart()
	local cmd = rttlib.port_clone_conn(sup:getPort("events"))
	cmd:write("e_restart")
end
