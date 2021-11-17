--===========================================================================================
--                                     OPERATOR INPUT
--===========================================================================================
-- File for all operator input.
-- All parameters needed by the program need to be changed in this file.
--===========================================================================================
local FSM_UserInput =  {}

-- Use real robot?
use_real_robot = true

--===========================================================================================
--VELOCITIES & ACCELERATIONS
--===========================================================================================

if simulation or use_real_robot then
	max_vel = 0.05   												--[m/s]
	max_acc = 0.05													--[m/s^2]
	scanning_vel = 0.002 											--[m/s]
	stiffness_calc_vel = 0.002   									--[m/s]
else
	max_vel = 0.5 													--[m/s]
	max_acc= 0.5 													--[m/s^2]
	scanning_vel = 0.005 											--[m/s]
	stiffness_calc_vel = 0.008 										--[m/s]
end

--===========================================================================================
--FORCES AND TORQUES
--===========================================================================================
-- Specify force and torque limits. {Fx, Fy, Fz, Tx, Ty, Tz}
force_torque_limits = {12, 12, 12, 12, 12, 12} 					-- [N,Nm]

-- Sensor-frame to Tool-frame transformation. {x, y, z, rotX, rotY, rotZ}
sensor_tool_frame_transf = {0, 0, -0.235, 0, 0, 0}					--[m, rad]

force_setpoint = -4

--===========================================================================================
--SENSOR COMPENSATION
--===========================================================================================

-- Sensor Steady-State-Error Compenstation
sensor_compensation = true
sensor_compensation_type = 1  --0:manual, 1:auto
sensor_comp_sample_size = 10000
reduce_zero_noice = false
reduce_zero_noice_cutoff = 0.02

-- FIFO Buffer
use_fifo_buffer = false
fifo_buffer_size = 1

--===========================================================================================
--STIFFNESS CALCULATION
--===========================================================================================

-- Stiffness
automatic_stiffness_calc = false
max_z_stiffness_calculation = 0.15  								--[m]

-- Stiffness Calculation Parameters
a_k_calc_force_start = -2										    --[N]
a_k_calc_force_stop =  -8										    --[N]

--===========================================================================================
--STIFFNESS CALCULATION OSCILLATION
--===========================================================================================

testpoint_flag = 0
oscillation_time = 2 --time of half a period
oscillation_amount = 3
testpoints_amount = 10
testpoint_distance = 0.005 											--[m]

oscillation_freq = 3.141529/oscillation_time
testing_time = 2 * oscillation_time * oscillation_amount

testpoint_distance = 0.005 --[m]
test_total_distance = (testpoints_amount-1) * testpoint_distance 	--[m]

--===========================================================================================
--CONTROLLER PARAMETERS
--===========================================================================================

controller_gain = 1
stiffness_custom = 1000.0										    --[N/m]

--===========================================================================================
--RELEVANT POSES
--===========================================================================================
-- Specify safe operating domain. {x_lower, x_upper, y_lower, y_upper, z_lower, z_upper}
op_domain = {-0.0, -0.5, -0.25, 0.25, 0.07, 0.6} 					--[m]
configure_pose = {0, 0, 1.43, 0, 0, 0}								--[m]
home_pose = {-0.40, 0, 0.4, 3.14, 0, 3.14} 							--[m]
starting_pose = {-0.50, 0, 0.14, 3.14, 0, 3.14}  			    	--[m]
end_pose = {-0.50, test_total_distance, 0.14 , 3.14, 0, 3.14} 		--[m]
all_coordinates_safe = check_all_coordinates(op_domain, home_pose, starting_pose, end_pose)
