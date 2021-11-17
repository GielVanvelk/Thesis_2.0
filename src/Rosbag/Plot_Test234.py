import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32

FILENAME = 'Test2'
ROOT_DIR = '/home/giel/etasl/ws/giel_workspace/src/Rosbag/RecordedData'

BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'
bag = rosbag.Bag(BAGFILE)
forcesZ = []


create_figures = True
open_figures = True #True

##################################### STATE ORDER
safety_check = 0
move_to_home = 1
idle = 2
move_to_start = 3
sensor_compensation = 4
stiffness_calculation = 5
control_parameter_check = 6
scanning = 7
finished = 8

##################################### DATA ARRAYS

# Force
forcesX = []
forcesY = []
forcesZ = []
forces_time = []

# Pose
posesX = []
posesY = []
posesZ = []
poses_time = []

# Stiffness calculation state
forces_kz = []
poses_kz = []
time_kz =[]

# State flags
state_flags = []
state_flags_smaller = []
state_flags_time = []

##################################### EXTRACT DATA
print(" >>>>> Extracting Data 1/5")

for topic, msg, t in bag.read_messages(topics=['/G_wrench', '/G_pose', '/G_stiffness_wrench', '/G_stiffness_pose', "/G_state_trans"]):
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
	# Stiffness data
	if topic == '/G_stiffness_wrench':
		forces_kz.append(msg.force.z)
		time_kz.append(t.secs + 1e-9 * t.nsecs)
	if topic == '/G_stiffness_pose':
		poses_kz.append(msg.linear.z)
	# State transition flags
	if topic == '/G_state_trans':
		state_flags.append(msg.data)
		state_flags_time.append(t.secs + 1e-9 * t.nsecs)
bag.close()

print(len(poses_kz))
print(len(forces_kz))
print(len(poses_kz))

#n= len(poses_kz)- len(forces_kz)
#del(poses_kz[-n:])


##################################### CREATE TIMELINES
print(" >>>>> Creating Timelines 2/5")

# TIMELINE: COMPLETE DATA
start_time = forces_time[0]
forces_time = [x - start_time for x in forces_time]
poses_time = [x - start_time for x in poses_time]
state_flags_time = [x - start_time for x in state_flags_time]

# TIMELINE: STIFFNESS CALCULATION
start_time = time_kz[0]
time_kz = [x - start_time for x in time_kz]


##################################### SEPERATE STATES
print(" >>>>> Seperating States 3/5")
import numpy as np
state_list = []
state_list_sep = []
state_list = np.multiply(state_flags,state_flags_time)

for i in range(len(state_flags)-1):
	current_val = state_list[i]
	next_val = state_list[i+1]
	if current_val == 0 and next_val !=0:
		state_list_sep.append(next_val)

print(state_list_sep)
stiffness_calc_start_time = state_list_sep[control_parameter_check-2]

scanning_start_time = state_list_sep[scanning-2]

finished_start_time = state_list_sep[finished-2]


scanningFz = []
scanningPz = []
scanningPz_mm = []
scanning_timeF = []
scanning_timeP = []


for i in range(len(forces_time)):
	time = forces_time[i]
	if time >= scanning_start_time and time <= finished_start_time:
		scanningFz.append(forcesZ[i])
		scanning_timeF.append(forces_time[i])

for i in range(len(poses_time)):
	time = poses_time[i]
	if time >= scanning_start_time and time <= finished_start_time:
		scanningPz.append(posesZ[i])
		scanning_timeP.append(poses_time[i])

for i in range(len(scanningPz)):
	scanningPz_mm.append(scanningPz[i] * 1000)

for i in range(len(state_flags)):
	state_flags_smaller.append(state_flags[i] * 0.1)

##################################### PLOT DATA
if create_figures == True:
	print(" >>>>> Plotting Data 4/5")

	##################################### Plot complete Forcez, PoseZ and State transition flag data.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fig = plt.figure()
	ax = fig.add_subplot(111)

	lns1 = ax.plot(poses_time, posesZ, 'b', label = 'Z-coordinate')
	ax2 = ax.twinx()
	lns2 = ax2.plot(state_flags_time, state_flags, 'k', label = 'State transition flag')
	lns3 = ax2.plot(forces_time, forcesZ, 'r', label = 'Z-force')

	# added these three lines
	lns = lns1+lns2+lns3
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0)

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
	ax2.set_ylabel('Measured z-force in the tool-frame [N]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title('Z-Force and Z-Position data')
	plt.savefig("Fig_ForceZ_PoseZ.png")
	plt.savefig("Fig_Stiffness_Calculation.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

	##################################### Plot all position data
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(poses_time, posesX, '-', label = 'X-coordinate')
	lns2 = ax.plot(poses_time, posesY, '-', label = 'Y-coordinate')
	lns3 = ax.plot(poses_time, posesZ, '-', label = 'Z-coordinate')
	lns4 = ax.plot(state_flags_time, state_flags_smaller, 'k', label = 'State transition flag')

	# added these three lines
	lns = lns1+lns2+lns3+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0)

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Measured force in the tool-frame [N]')
	#ax2.set_ylim(0, 35)
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=4)
	plt.title('Position data')
	plt.savefig("Fig_Position.png")
	plt.savefig("Fig_Stiffness_Calculation.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

	##################################### Plot all force data
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(forces_time, forcesX, '-', label = 'X-force')
	lns2 = ax.plot(forces_time, forcesY, '-', label = 'Y-force')
	lns3 = ax.plot(forces_time, forcesZ, '-', label = 'Z-force')
	lns4 = ax.plot(state_flags_time, state_flags, 'k', label = 'State transition flag')

	# added these three lines
	lns = lns1+lns2+lns3+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0)

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Measured force in the tool-frame [N]')
	#ax2.set_ylim(0, 35)
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title('Force data')
	plt.savefig("Fig_Force.png")
	plt.savefig("Fig_Stiffness_Calculation.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

	##################################### Plot stiffness calculation data
	import numpy as np
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fit = np.polyfit(poses_kz, forces_kz, 1)
	print(fit)
	y_new = np.polyval(fit, poses_kz)

	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(poses_kz, forces_kz, '-', label = 'Fz(z-coordinate)')
	lns2 = ax.plot(poses_kz, y_new, '-', label = 'Polyfitted Line')

	# added these three lines
	lns = lns1 + lns2
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0)

	ax.grid()
	ax.set_xlabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
	ax.set_ylabel('Measured z-force in the tool-frame [N]')
	plt.title('Z-Force and Z-Position during Stiffness Calculation State')
	plt.xlim(plt.xlim()[::-1])
	plt.autoscale(enable=True, axis='both', tight=None)
	#plt.xlim(left=0)
	plt.savefig("Fig_Stiffness_Calculation.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

	##################################### Plot Forcez, PoseZ during the scanning state.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fig = plt.figure()
	ax = fig.add_subplot(111)
	#ax.set_ylim(10,20)
	lns1 = ax.plot(scanning_timeP, scanningPz_mm, 'b', label = 'Z-coordinate')
	ax2 = ax.twinx()
	lns2 = ax2.plot(scanning_timeF, scanningFz, 'r', label = 'Z-force')

	# added these three lines
	lns = lns1+lns2
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=4)

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
	ax2.set_ylabel('Measured z-force in the tool-frame [N]')
	#plt.autoscale(enable=True, axis='both', tight=None)
	plt.title('Z-Force and Z-Position during the scanning state')
	plt.savefig("Fig_ForceZ_PoseZ_ScanningState.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

##################################### FINISHED
print(" >>>>> Finished 5/5")
