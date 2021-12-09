import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32
import numpy as np

#####################################
### GET DATA FILE
print(">>>>> Getting Ready .................... 1/6")
#####################################
FILENAME = 'Test1'
ROOT_DIR = '/home/giel/etasl/ws/giel_workspace/src/Rosbag/RecordedData'

BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'
bag = rosbag.Bag(BAGFILE)

create_figures = True
open_figures = False

#####################################
### DATA LISTS
#####################################
# Force
forcesX = []
forcesY = []
forcesZ = []
forces_time = []

# Force setpoint
Fz_desired = []
Fz_des_time = []

# Pose
posesX = []
posesY = []
posesZ = []
poses_time = []

# State flags
state_flags = []
state_flags_smaller = []
state_flags_time = []
state_start_times = []

#####################################
### STATE ORDER
#####################################

state_mth = 1   											       # move_to_home
state_idle = 2						    					       # idle
state_mts = [3, 7, 11, 15, 19, 23, 27, 31, 35, 39 ]  	           # move_to_start
state_sensor_compensation = [4, 8, 12, 16, 20, 24, 28, 32, 36, 40] # sensor_compensation
state_initial_force = [5, 9, 13, 17, 21, 25, 29, 33, 37, 41]   	   # initial_force
state_stiffness_test = [6, 10, 14, 18, 22, 26, 30, 34, 38, 42]	   # stiffness_test
state_finished = [43]											   # test completed

#####################################
### EXTRACT DATA
print(">>>>> Extracting Data .................. 2/6")
#####################################
for topic, msg, t in bag.read_messages(topics=['/G_wrench', '/G_pose', "/G_state_trans", "/Fz_desired"]):
	# Force data
	if topic == '/G_wrench':
		forcesX.append(msg.force.x)
		forcesY.append(msg.force.y)
		forcesZ.append(msg.force.z)
		forces_time.append(t.secs + 1e-9 * t.nsecs)
	# Pose data
	if topic == '/G_pose':
		posesX.append(msg.linear.x)
		posesY.append(msg.linear.y)
		posesZ.append(msg.linear.z)
		poses_time.append(t.secs + 1e-9 * t.nsecs)
	# State transition flags
	if topic == '/G_state_trans':
		state_flags.append(msg.data)
		state_flags_smaller.append(msg.data * 0.1)
		state_flags_time.append(t.secs + 1e-9 * t.nsecs)
	# Get desired force
	if topic == '/Fz_desired':
		Fz_desired.append(msg.data)
		Fz_des_time.append(t.secs + 1e-9 * t.nsecs)
bag.close()

#####################################
### CREATE TIMELINES
print(">>>>> Creating Timelines ............... 3/6")
#####################################
# forces timeline
start_time = forces_time[0]
forces_time = [x - start_time for x in forces_time]

# poses timeline
start_time = poses_time[0]
poses_time = [x - start_time for x in poses_time]

# state flag timeline
start_time = state_flags_time[0]
state_flags_time = [x - start_time for x in state_flags_time]

# desired force timeline
start_time = Fz_des_time[0]
Fz_des_time = [x - start_time for x in Fz_des_time]

#####################################
### SEPERATE STATES
print(">>>>> Seperating States ................ 4/6")
#####################################
state_list = []
state_list_starts = []
state_list = np.multiply(state_flags,state_flags_time)

# For every element in the state_flags list
for i in range(len(state_flags)-1):
	# Current value
	current_val = state_list[i]
	# Next value
	next_val = state_list[i+1]
	# Check if the next value is the first non-zero element
	#if current_val == 0 and next_val !=0:
		#state_list_starts.append(next_val)
	if current_val == 0 and next_val !=0:
		state_list_starts.append(next_val)

oscillation_starts = []
oscillation_stops = []

for i in range(len(state_stiffness_test)):  #-1
	# Get the state order of the oscillation test
	oscillation_state_order = state_stiffness_test[i]
	# Get the relevant starting time
	oscillation_start_time = state_list_starts[oscillation_state_order]
	# Safe the retreived start time in a list
	oscillation_starts.append(oscillation_start_time)

for i in range(1, len(state_mts)): # start at the second element
	# Get the state order of the move to start sta
	mts_state_order = state_mts[i]
	# Get the relevant starting time
	oscillation_stop_time = state_list_starts[mts_state_order]
	# Safe the retreived start time in a list
	oscillation_stops.append(oscillation_stop_time)



for os_num in range(len(state_stiffness_test)-1):

	oscillation_force = []
	oscillation_pose = []
	oscillation_pose_mm = []
	oscillation_timeF = []
	oscillation_timeP = []
	oscillation_Fdes = []
	oscillation_Fdes_time = []

	for i in range(len(forces_time)):
		if forces_time[i] >= oscillation_starts[os_num] and forces_time[i] <= oscillation_stops[os_num]:
			oscillation_force.append(forcesZ[i])
			oscillation_timeF.append(forces_time[i])

	for i in range(len(poses_time)):
		if poses_time[i] >= oscillation_starts[os_num] and poses_time[i] <= oscillation_stops[os_num]:
			oscillation_pose.append(posesZ[i])
			oscillation_timeP.append(poses_time[i])

	for i in range(len(Fz_des_time)):
		if Fz_des_time[i] >= oscillation_starts[os_num] and Fz_des_time[i] <= oscillation_stops[os_num]:
			oscillation_Fdes.append(Fz_desired[i])
			oscillation_Fdes_time.append(Fz_des_time[i])

	for i in range(len(oscillation_pose)):
		oscillation_pose_mm.append(oscillation_pose[i] * 1000)

	# Linear interpolation to fit the pose and force data
	fitted_1 = []
	fitted_2 = []

	for i in range(len(oscillation_timeP)-1):
		X = oscillation_timeP[i]
		for j in range(len(oscillation_timeF)-1):
			X1 = oscillation_timeF[j]
			X2 = oscillation_timeF[j+1]
			if (X1 <= X and X2>= X):
				Y1 = oscillation_force[j]
				Y2 = oscillation_force[j+1]
				Y = ((Y2-Y1)/(X2-X1))*(X-X1)+Y1
				fitted_1.append(oscillation_pose_mm[i])
				fitted_2.append(Y)

	##################################### Plot Forcez, PoseZ during first oscillation.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	#ax.set_ylim(10,20)
	lns1 = ax.plot(oscillation_timeP, oscillation_pose_mm, 'b', label = 'Z-coordinate')
	ax2 = ax.twinx()
	lns2 = ax2.plot(oscillation_timeF, oscillation_force, 'r', label = 'Z-force')
	lns3 = ax2.plot(oscillation_Fdes_time, oscillation_Fdes, 'g', label = 'Fz_desired')
	# added these three lines
	lns = lns1+lns2+lns3
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=4, prop={'size': 6})
	ax.grid()
	ax.set_xlabel("Time [s]")
	ax.set_ylabel('Z-coordinate [mm]')
	ax2.set_ylabel('Measured Z-force [N]')
	#plt.autoscale(enable=True, axis='both', tight=None)
	plt.title('Z-Force and Z-Position during oscillation (P %i)' % os_num)
	plt.savefig("Fig_Test1_ForceZ_PoseZ_ScanningState_P%i.png" % os_num)
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
	plt.close()
	##################################### Plot stiffness calculation data
	import numpy as np
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fit = np.polyfit(fitted_1, fitted_2, 1)
	fit_slope = fit[0]*1000
	y_new = np.polyval(fit, fitted_1)

	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.scatter(fitted_1, fitted_2, c='r', label = 'Fz(z-coordinate)')
	#lns1 = ax.plot(fitted_1, fitted_2, 'r', label = 'Fz(z-coordinate)')
	lns2 = ax.plot(fitted_1, y_new, '-', label = 'Polyfitted Line with slope %i' % fit_slope)

	# added these three lines
	lns = lns2
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0, prop={'size': 6})

	ax.grid()
	ax.set_xlabel('Z-coordinate of the tool-frame relative to the worldframe [mm]')
	ax.set_ylabel('Measured z-force in the tool-frame [N]')
	plt.title('Z-Force and Z-Position during Stiffness Calculation State (P %i)' % os_num)
	plt.xlim(plt.xlim()[::-1])
	plt.autoscale(enable=True, axis='both', tight=None)
	#plt.xlim(left=144)
	#plt.xlim(right=142)
	plt.savefig("Fig_Test1_Stiffness_Calculation_P%i.png" % os_num)
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
	plt.close()
#####################################
### PLOT DATA
#####################################
if create_figures == True:
	print(">>>>> Plotting Data .................... 5/6")

	##################################### Plot complete Forcez, PoseZ and State transition flag data.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(poses_time, posesZ, 'b', label = 'Z-coordinate')
	ax2 = ax.twinx()
	lns2 = ax2.plot(forces_time, forcesZ, 'r', label = 'Z-force')
	lns3 = ax2.plot(Fz_des_time, Fz_desired, 'g', label = 'Fz_desired')
	lns4 = ax2.plot(state_flags_time, state_flags, 'k', label = 'State transition flag')
	lns = lns1+lns2+lns3+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc='lower right', prop={'size': 6})
	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate [m]')
	ax2.set_ylabel('Measured Z-force [N]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title('Measured Z-Force and Z-Position of the tool-frame relative to the worldframe')
	plt.savefig("Fig_Test1_Stiffness_Oscillation.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
	plt.close()
	##################################### Plot the coordinates.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(poses_time, posesX, 'b', label = 'X-coordinate')
	lns2 = ax.plot(poses_time, posesZ, 'r', label = 'Z-coordinate')
	lns4 = ax.plot(state_flags_time, state_flags_smaller, 'k', label = 'State Transition Flags')
	ax2 = ax.twinx()
	lns3 = ax2.plot(poses_time, posesY, 'g', label = 'Y-coordinate')
	lns = lns1+lns2+lns3+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc='lower left', prop={'size': 6})
	ax.grid()
	ax.set_xlabel("Time [s]")
	ax.set_ylabel('X- and Z-coordinate [m]')
	ax2.set_ylabel('Y-coordinate [m]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title(' Coordinates of the tool-frame releative to the worldframe')
	plt.savefig("Fig_Test1_Coordinates.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
	plt.close()
	##################################### Plot the forces.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(forces_time, forcesX, 'b', label = 'Fx')
	lns2 = ax.plot(forces_time, forcesY, 'r', label = 'Fy')
	lns3 = ax.plot(forces_time, forcesZ, 'k', label = 'Fx')
	#lns4 = ax.plot(Fz_des_time, Fz_desired, 'g', label = 'Fz_desired')
	lns = lns1+lns2+lns3#+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc='lower left', prop={'size': 6})
	ax.grid()
	ax.set_xlabel("Time [s]")
	ax.set_ylabel('Force [N]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title(' Coordinates of the tool-frame releative to the worldframe')
	plt.savefig("Fig_Test1_Forces.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
	plt.close()
print(">>>>> Finished ......................... 6/6")
