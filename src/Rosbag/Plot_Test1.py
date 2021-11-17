import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32


##################################### GET DATA FILE
FILENAME = 'Test1'
ROOT_DIR = '/home/giel/etasl/ws/giel_workspace/src/Rosbag/RecordedData'

BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'
bag = rosbag.Bag(BAGFILE)

create_figures = True
open_figures = True #True

##################################### DATA ARRAYS

# Force
forcesX = []
forcesY = []
forcesZ = []
forces_time = []

# Pose
posesX =
posesY = []
posesZ = []
poses_time = []

# State flags
state_flags = []
state_flags_smaller = []
state_flags_time = []

Fz_desired = []
Fz_des_time = []

##################################### EXTRACT DATA
print(" >>>>> Extracting Data 1/4")

for topic, msg, t in bag.read_messages(topics=['/G_wrench', '/G_pose', '/G_stiffness_wrench', '/G_stiffness_pose', "/G_state_trans", "/Fz_desired"]):
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
	if topic == '/Fz_desired':
		Fz_desired.append(msg.data)
		Fz_des_time.append(t.secs + 1e-9 * t.nsecs)
bag.close()

##################################### CREATE TIMELINES
print(" >>>>> Creating Timelines 2/4")

# TIMELINE: COMPLETE DATA
start_time = forces_time[0]
forces_time = [x - start_time for x in forces_time]
poses_time = [x - start_time for x in poses_time]
state_flags_time = [x - start_time for x in state_flags_time]

start_time = Fz_des_time[0]
Fz_des_time = [x - start_time for x in Fz_des_time]

##################################### PLOT DATA
if create_figures == True:
	print(" >>>>> Plotting Data 3/4")

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
	lns4 = ax2.plot(Fz_des_time, Fz_desired, 'g', label = 'Fz_desired')
	# added these three lines
	lns = lns1+lns2+lns3+lns4
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc='lower left')

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
	ax2.set_ylabel('Measured z-force in the tool-frame [N]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title('Z-Force and Z-Position data during stiffness test')
	plt.savefig("Fig_Stiffness_Test.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()

	##################################### Plot complete Forcez, PoseZ and State transition flag data.
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')

	fig = plt.figure()
	ax = fig.add_subplot(111)

	lns1 = ax.plot(poses_time, posesY, 'b', label = 'Z-coordinate')
	# added these three lines
	lns = lns1
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc='lower left')

	ax.grid()
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
	plt.autoscale(enable=True, axis='both', tight=None)
	plt.xlim(left=0)
	plt.title(' y-position during stiffness test')
	plt.savefig("Fig_Stiffness_Test_y_co.png")
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
print(" >>>>> Finished 4/4")
