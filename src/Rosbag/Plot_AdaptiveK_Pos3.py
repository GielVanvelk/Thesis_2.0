import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32
import numpy as np

FILENAME = 'Test4'
ROOT_DIR = '/home/giel/etasl/ws/giel_workspace/src/Rosbag/RecordedData'

BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'
bag = rosbag.Bag(BAGFILE)

#create_figures = True
#open_figures = True #True
create_scanning_graph = False

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
poses_kz_mm = []
timeF_kz =[]
timeP_kz =[]
# State flags
state_flags = []
state_flags_smaller = []
state_flags_time = []
##################################### EXTRACT DATA
print(" >>>>> Extracting Data")
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
		timeF_kz.append(t.secs + 1e-9 * t.nsecs)
	if topic == '/G_stiffness_pose':
		poses_kz.append(msg.linear.z)
		poses_kz_mm.append(msg.linear.z * 1000)
		timeP_kz.append(t.secs + 1e-9 * t.nsecs)
	# State transition flags
	if topic == '/G_state_trans':
		state_flags.append(msg.data)
		state_flags_smaller.append(msg.data * 0.1)
		state_flags_time.append(t.secs + 1e-9 * t.nsecs)
bag.close()
##################################### CREATE TIMELINES
print(" >>>>> Creating Timelines")
# forces timeline
start_time = forces_time[0]
forces_time = [x - start_time for x in forces_time]
# poses timeline
start_time = poses_time[0]
poses_time = [x - start_time for x in poses_time]
# state flag timeline
start_time = state_flags_time[0]
state_flags_time = [x - start_time for x in state_flags_time]
##################################### STIFFNESS ARRAYS
print(" >>>>> Creating Stiffness Arrays")
REF_STIFF = [3903, 4346,4413,4455,4350,4151,4322,4811,4754]
Y_REF = [0, 5,10,15,20,25,30,35,40]
INITIAL_STIFF = 3327
CALC_STIFF = []
ADJ_STIFF = []
Y_STIFF = []
STIFF_RATIO = 0.1
#CALC_STIFF.append(INITIAL_STIFF)
#ADJ_STIFF.append(INITIAL_STIFF)
##################################### SEPERATE SCANNING DATA
print(" >>>>> Get Scanning Data")
state_list = []
state_list_sep = []
state_list = np.multiply(state_flags,state_flags_time)
for i in range(len(state_flags)-1):
	current_val = state_list[i]
	next_val = state_list[i+1]
	if current_val != 0 and next_val ==0:
		state_list_sep.append(current_val)

stiffness_calc_start_time = state_list_sep[control_parameter_check]
scanning_start_time = state_list_sep[scanning]
finished_start_time = state_list_sep[finished]
scanningFz = []
scanningPz = []
scanningPz_mm = []
scanning_timeF = []
scanning_timeP = []
fittedAD1 = []
fittedAD2 = []
# Get Scanning Data
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

# Downsizing data
downsizing = 5
#del scanningFz[::downsizing]
#del scanningPz_mm[::downsizing]
#del scanning_timeF[::downsizing]
#del scanning_timeP[::downsizing]

# force setpoint
setpoint = [-4 for i in range(len(scanningFz))]
# Linear interpolation to fit the pose and force data
for i in range(len(scanning_timeP)-1):
	X = scanning_timeP[i]
	NumberOfLoops = len(scanningFz)
	if i%2500 ==0:
		print(' >>>>>>> %i of %i' %(i, NumberOfLoops))
	for j in range(len(scanning_timeF)-1):
		X1 = scanning_timeF[j]
		X2 = scanning_timeF[j+1]
		if (X1 <= X and X2>= X):
			Y1 = scanningFz[j]
			Y2 = scanningFz[j+1]
			Y = ((Y2-Y1)/(X2-X1))*(X-X1)+Y1
			fittedAD1.append(scanningPz_mm[i])
			fittedAD2.append(Y)
##################################### ADAPTIVE STIFFNESS CALCULATION
print(" >>>>> Adaptive Stiffness Calculation")
FIFO_BUFFER_P = []
FIFO_BUFFER_F = []
BUFFER_SIZE = 500
import matplotlib.pyplot as plt
from matplotlib import rc
fig = plt.figure()
ax = fig.add_subplot(111)
rc('mathtext', default='regular')
for i in range(len(fittedAD1)):
	if i%2500 ==0:
		print(' >>>>>>> %i of %i' %(i, NumberOfLoops))
	if len(FIFO_BUFFER_P) >= BUFFER_SIZE:
		del FIFO_BUFFER_P[0]
		#FIFO_BUFFER_P.pop(0)
		FIFO_BUFFER_P.append(fittedAD1[i])
		#FIFO_BUFFER_F.pop(0)
		del FIFO_BUFFER_F[0]
		FIFO_BUFFER_F.append(fittedAD2[i])

		fit = np.polyfit(FIFO_BUFFER_P, FIFO_BUFFER_F, 1)
		Force_new = np.polyval(fit, FIFO_BUFFER_P)
		CALC_STIFF.append(fit[0]*1000)
		ADJ_STIFF.append(CALC_STIFF)
		ax.scatter(FIFO_BUFFER_P, Force_new, c='k', label = 'Data Points')
		ax.plot(FIFO_BUFFER_P, Force_new, c='b', label = 'Stiffne')
		#print(' I plotted the thing boss')
	else:
		FIFO_BUFFER_P.append(fittedAD1[i])
		FIFO_BUFFER_F.append(fittedAD2[i])
		CALC_STIFF.append(0)
		ADJ_STIFF.append(CALC_STIFF)

Y_AXIS = np.linspace(0, 45, num=len(fittedAD1))
ax.grid()
ax.set_xlabel('Z-coordinate of the tool-frame relative to the worldframe [mm]')
ax.set_ylabel('Fz [N]')
plt.title('Stiffness')
plt.autoscale(enable=True, axis='both', tight=None)
plt.savefig("Fig_AD_Calc.png")

print(len(fittedAD1))
print(len(CALC_STIFF))
print(len(ADJ_STIFF))
print(len(Y_AXIS))

# Force in function of position
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
rc('mathtext', default='regular')
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(Y_REF, REF_STIFF, c='r', label = 'Reference stiffness')
ax.scatter(Y_AXIS, CALC_STIFF, c='b', label = 'Calculated stiffness')
#ax.scatter(Y_AXIS, ADJ_STIFF, c='r', label = 'Adjusted stiffness')
ax.legend(loc='upper left')
ax.grid()
ax.set_xlim(-5,45)
ax.set_xlabel('Y-coordinate of the tool-frame relative to the worldframe [mm]')
ax.set_ylabel('Stiffness [N/m]')
plt.title('Stiffness')
#plt.autoscale(enable=True, axis='both', tight=None)
plt.savefig("Fig_CAD.png")
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
