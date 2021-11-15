--===========================================================================================
--                                  OPERATOR INPUT
--===========================================================================================
local FSM_UserInput =  {}

-- Use real robot?
use_real_robot = false

-- Specify safe operating domain.
op_domain = {-0.0, -0.5, -0.25, 0.25, 0.07, 0.6} --{x_lower, x_upper, y_lower, y_upper, z_lower, z_upper} [m]

-- Specify Relevant Poses
configure_pose = {0, 0, 1.43, 0, 0, 0}								--[m]
home_pose = {-0.40, 0, 0.4, 3.14, 0, 3.14} 							--[m]
starting_pose = {-0.50, 0, 0.20, 3.14, 0, 3.14}  			    	--[m]
end_pose = {-0.50, 0.1, 0.25, 3.14, 0, 3.14} 						--[m]
all_coordinates_safe = check_all_coordinates(op_domain, home_pose, starting_pose, end_pose)

-- Velocity and Acceleration Limits
if simulation or use_real_robot then
	max_vel = 0.05   												--[m/s]
	max_acc = 0.05													--[m/s^2]
	scanning_vel = 0.005 											--[m/s]
	stiffness_calc_vel = 0.002   									--[m/s]
else
	max_vel = 0. 													--[m/s]
	max_acc= 0.5 													--[m/s^2]
	scanning_vel = 0.005 												--[m/s]
	stiffness_calc_vel = 0.008 										--[m/s]
end

-- Force Parameters
-- Fx, Fy, Fz, Tx, Ty, Tz
force_torque_limits = {12, 12, 12, 1.5, 1.2, 0.3} 				    --[N, Nm]
force_setpoint = -4												--[N]

-- Sensor-frame to Tool-frame transformation
-- x, y, z, rotX, rotY, rotZ
sensor_tool_frame_transf = {0, 0, -0.235, 0, 0, 0}					--[m, rad]

-- Sensor Steady-State-Error Compenstation
sensor_compensation = true
sensor_compensation_type = 1  --0:manual, 1:auto
sensor_comp_sample_size = 10000
reduce_zero_noice = false
reduce_zero_noice_cutoff = 0.02

-- FIFO Buffer
use_fifo_buffer = false
fifo_buffer_size = 1

-- Stiffness
automatic_stiffness_calc = true
max_z_stiffness_calculation = 0.15  								--[m]

-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
-- STIFFNESS CALCULATION TEST PARAMETERS
a_k_calc_force_start = -2										    --[N]
a_k_calc_force_stop =  -8										    --[N]
custom_stiffness = 1000.0										    --[N/m]

oscillation_time = 2 --time of half a period
oscillation_freq = 3.141529/oscillation_time
oscillation_amount = 3
testing_time = 2 * oscillation_time * oscillation_amount
testpoints_amount = 3
testpoint_flag = 0
testpoint_distance = 0.03 --[m]
K_controller = 1
