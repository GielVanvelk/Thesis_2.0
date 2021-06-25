require "os"
require "rttlib"
require "rttros"
require "deployer_utils"

-- ==================================== Standard deployment stuff =========================================

gs=rtt.provides()
tc=rtt.getTC()
if tc:getName() == "lua" then
  depl=tc:getPeer("Deployer")
elseif tc:getName() == "Deployer" then
  depl=tc
end


depl:import("rtt_ros")
depl:import( "rtt_std_msgs" )
depl:import( "rtt_rosnode" )
depl:import( "rtt_rospack" )
depl:import( "rtt_roscomm" )
depl:import( "rtt_rosclock" )
depl:import( "soem_master" )
depl:import( "soem_beckhoff_drivers" )
depl:import( "ati_iface" )



timefreq = 1/100

dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/SoemMasterComponent_properties.lua")
dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/AtiIface_properties.lua" )
dofile("/home/giel/etasl/ws/giel_workspace/src/giel_etasl_application/configuration/Application_properties.lua" )


depl:loadComponent( "soem_master", "soem_master::SoemMasterComponent" )
depl:addPeer( "soem_master","Deployer" )
soem_master = depl:getPeer( "soem_master" )

depl:loadComponent( "AtiIface", "ati::AtiIface" )
depl:addPeer( "AtiIface", "Deployer" )
ati_iface = depl:getPeer( "AtiIface" )



-- Properties of soem_master.
depl:setActivity( soem_master:getName( ), timefreq,  50, rtt.globals.ORO_SCHED_RT)
soem_master:getProperty( "ifname" ):set( ifname )


-- Properties of ati_iface.
depl:setActivity( ati_iface:getName( ), timefreq, 50, rtt.globals.ORO_SCHED_RT )
ati_iface:getProperty( "scale_matrix" ):set( scale_matrix )
ati_iface:getProperty( "calibration_matrix" ):set( calibration_matrix )
ati_iface:getProperty( "lowpass_filter_on" ):set( lowpass_filter_on )
ati_iface:getProperty( "filter_bandwidth" ):set( filter_bandwidth )
ati_iface:getProperty( "filter_order" ):set( filter_order )
ati_iface:getProperty( "compensate_deadband" ):set( compensate_deadband )
ati_iface:getProperty( "f_deadband" ):set( f_deadband )
ati_iface:getProperty( "m_deadband" ):set( m_deadband )

-- depl:setActivity( gui_connect:getName( ), virtuose_period, 1, 1 )

-- -------------------- --
-- Configure components --
-- -------------------- --
soem_master:getProperty( "ifname" ):set( ifname )
soem_master:configure( )
if not soem_master:isConfigured( ) then -- Switch the interface names in case soem_master doesn't configure.
    soem_master:getProperty( "ifname" ):set( ifname2 )
    soem_master:configure( )
end
ati_iface:configure()
-- gui_connect:configure( )


-- ------------------------------ --
-- Create OROCOS port connections --
-- ------------------------------ --
cp = rtt.Variable( "ConnPolicy" )
depl:connect( soem_master:getName( )..".Slave_1002.values", ati_iface:getName( )..".in_ai1_port", cp )
depl:connect( soem_master:getName( )..".Slave_1003.values", ati_iface:getName( )..".in_ai2_port", cp )
depl:stream(ati_iface:getName( )..".out_W_ati", rtt.provides( "ros" ):topic("/wrench_raw") )
-- dep:stream( gui_connect:getName( )..".in_gui_message", rtt.provides( "ros" ):topic( "/vag_assistant/gui_message" ) )


-- ---------------- --
-- Start components --
-- ---------------- --
soem_master:start( )
ati_iface:start( )
-- gui_connect:start( )
