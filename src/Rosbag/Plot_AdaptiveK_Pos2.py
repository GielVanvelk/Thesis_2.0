import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32

FILENAME = 'Test4'
ROOT_DIR = '/home/giel/etasl/ws/giel_workspace/src/Rosbag/RecordedData'

BAGFILE = ROOT_DIR + '/' + FILENAME + '.bag'
bag = rosbag.Bag(BAGFILE)

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

REF_STIFF = [3903, 4346,4413,4455,4350,4151,4322,4811,4754]
Y_REF = [0, 5,10,15,20,25,30,35,40]
CALC_STIFF = []
ADJ_STIFF = []
Y_STIFF = []
STIFF_RATIO = 0.1

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
print(" >>>>> Creating Timelines 2/5")

# forces timeline
start_time = forces_time[0]
forces_time = [x - start_time for x in forces_time]

# poses timeline
start_time = poses_time[0]
poses_time = [x - start_time for x in poses_time]

# state flag timeline
start_time = state_flags_time[0]
state_flags_time = [x - start_time for x in state_flags_time]

##################################### SEPERATE STATES
print(" >>>>> Seperating States 3/5")
import numpy as np
state_list = []
state_list_sep = []
state_list = np.multiply(state_flags,state_flags_time)

for i in range(len(state_flags)-1):
	current_val = state_list[i]
	next_val = state_list[i+1]
	if current_val != 0 and next_val ==0:
		state_list_sep.append(current_val)

##################################### INITIAL STIFFNESS ESTIMATION
print(" >>>>> Calculate Initial Stiffness 4/5")
fitted_1 = []
fitted_2 = []

for i in range(len(timeP_kz)-1):
	X = timeP_kz[i]
	for j in range(len(timeF_kz)-1):
		X1 = timeF_kz[j]
		X2 = timeF_kz[j+1]
		if (X1 <= X and X2>= X):
			Y1 = forces_kz[j]
			Y2 = forces_kz[j+1]
			Y = ((Y2-Y1)/(X2-X1))*(X-X1)+Y1
			fitted_1.append(poses_kz_mm[i])
			fitted_2.append(Y)

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
rc('mathtext', default='regular')
fit = np.polyfit(fitted_1, fitted_2, 1)
fit_slope = fit[0]*1000
CALC_STIFF.append(fit_slope)
ADJ_STIFF.append(fit_slope)
INITIAL_STIFFNESS = fit_slope
y_new = np.polyval(fit, fitted_1)
fig = plt.figure()
ax = fig.add_subplot(111)
lns1 = ax.scatter(fitted_1, fitted_2, c='r', label = 'Fz(z-coordinate)')
#lns1 = ax.plot(fitted_1, fitted_2, 'r', label = 'Fz(z-coordinate)')
lns2 = ax.plot(fitted_1, y_new, '-', label = 'Polyfitted Line with slope %i' % fit_slope)
# added these three lines
lns = lns2
labs = [l.get_label() for l in lns]
ax.legend(lns, labs, loc=0)
ax.grid()
ax.set_xlabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
ax.set_ylabel('Measured z-force in the tool-frame [N]')
plt.title('Z-Force and Z-Position during Stiffness Calculation State(%s)' % FILENAME)
plt.xlim(plt.xlim()[::-1])
plt.autoscale(enable=True, axis='both', tight=None)
#plt.xlim(left=0)
plt.savefig("Fig_%s_Stiffness_Calculation_AD.png" % FILENAME)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()


##################################### GET SCANNING DATA
print(" >>>>> Get Scanning Data 4/5")
stiffness_calc_start_time = state_list_sep[control_parameter_check]
scanning_start_time = state_list_sep[scanning]
finished_start_time = state_list_sep[finished]

scanningFz = []
scanningPz = []
scanningPz_mm = []
scanning_timeF = []
scanning_timeP = []

# Force Setpoint
setpoint = []
setpoint = [-4 for i in range(len(scanningFz))]

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

import matplotlib.pyplot as plt
from matplotlib import rc
rc('mathtext', default='regular')

fig = plt.figure()
ax = fig.add_subplot(111)
#ax.set_ylim(10,20)
lns1 = ax.plot(scanning_timeP, scanningPz_mm, 'b', label = 'Z-coordinate')
ax2 = ax.twinx()
lns2 = ax2.plot(scanning_timeF, scanningFz, 'r', label = 'Z-force')
#lns3 = ax2.plot(scanning_timeF, setpoint, 'k', label = 'Z-force setpoint')

# added these three lines
lns = lns1+lns2#+lns3
labs = [l.get_label() for l in lns]
ax.legend(lns, labs, loc=4)

ax.grid()
ax.set_xlabel("Time (s)")
ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [m]')
ax2.set_ylabel('Measured z-force in the tool-frame [N]')
#plt.autoscale(enable=True, axis='both', tight=None)
plt.title('Z-Force and Z-Position during the scanning state (%s)' % FILENAME)
plt.savefig("Fig_%s_ForceZ_PoseZ_ScanningState_AD.png" % FILENAME)
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()


##################################### ADAPTIVE STIFFNESS CALCULATION
print(" >>>>> Adaptive Stiffness Calculation 5/5")

startY = 0
Y_STIFF.append(startY)
stopY = 0.045
calc_distance = 0.05
calculations =(stopY - startY)/calc_distance
calc_times = []
calc_times.append(scanning_timeF[0])

for j in range(int(calculations)-1):
	step = j + 1
	pointY = step * calc_distance
	Y_STIFF.append(pointY*1000)

	for i in range(len(posesY)-1):
		current_val = posesY[i]
		next_val = posesY[i+1]
		if current_val < pointY and next_val > pointY:
			calc_times.append(poses_time[i])

for i in range(len(calc_times)-1):
	calc_F = []
	calc_TF = []
	calc_P = []
	calc_TP = []
	fittedAD1 = []
	fittedAD2 = []
	start_range = calc_times[i]
	end_range = calc_times[i+1]

	# Get data in ranges
	for j in range(len(scanning_timeF)):
		time = scanning_timeF[j]
		if time >= start_range and time <= end_range:
			calc_F.append(scanningFz[j])
			calc_TF.append(scanning_timeF[j])

	for j in range(len(scanning_timeP)):
		time = scanning_timeP[j]
		if time >= start_range and time <= end_range:
			calc_P.append(scanningPz_mm[j])
			calc_TP.append(scanning_timeP[j])


	# Data in function of time
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(calc_TP, calc_P, 'b', label = 'Z-coordinate')
	ax2 = ax.twinx()
	lns2 = ax2.plot(calc_TF, calc_F, 'r', label = 'Z-force')
	lns = lns1+lns2
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=4)
	ax.grid()
	ax.set_ylim(111,114)
	ax.xaxis.get_offset_text().set_visible(False)
	ax.yaxis.get_offset_text().set_visible(False)
	ax2.xaxis.get_offset_text().set_visible(False)
	ax2.yaxis.get_offset_text().set_visible(False)
	ax.set_xlabel("Time (s)")
	ax.set_ylabel('Z-coordinate of the tool-frame relative to the worldframe [mm]')
	ax2.set_ylabel('Measured z-force in the tool-frame [N]')
	#plt.autoscale(enable=True, axis='both', tight=None)
	plt.title('Range (%i)' %i)
	plt.savefig("Fig_AD_Range_%i.png" %i)
	if open_figures == True:
			mng = plt.get_current_fig_manager()
			mng.resize(*mng.window.maxsize())
			plt.show()
			plt.close()

	# Linear interpolation to fit the pose and force data
	for k in range(len(calc_TF)-1):
		X = calc_TF[k]
		for l in range(len(calc_TP)-1):
			X1 = calc_TP[l]
			X2 = calc_TP[l+1]
			if (X1 <= X and X2>= X):
				Y1 = calc_P[l]
				Y2 = calc_P[l+1]
				Y = ((Y2-Y1)/(X2-X1))*(X-X1)+Y1
				fittedAD2.append(calc_F[k])
				fittedAD1.append(Y)

	stiffness_graph = np.polyfit(fittedAD1, fittedAD2, 1)
	stiffness_slope = stiffness_graph[0]*1000
	force_new = np.polyval(stiffness_graph, fittedAD1)
	CALC_STIFF.append(stiffness_slope)
	ADJ_STIFF.append(((1-STIFF_RATIO)*ADJ_STIFF[i])+(STIFF_RATIO*stiffness_slope))

	for n in range(len(fittedAD2)-1):
		cval = fittedAD2[i]
		high_out =
		low_out
		if cval





	# Force in function of position
	import numpy as np
	import matplotlib.pyplot as plt
	from matplotlib import rc
	rc('mathtext', default='regular')
	fig = plt.figure()
	ax = fig.add_subplot(111)
	lns1 = ax.plot(fittedAD1, force_new, '-', label = 'Polyfitted Line with slope %i' % stiffness_slope)
	lns2 = ax.scatter(fittedAD1, fittedAD2, c='r', label = 'Fz(z-coordinate)')
	lns = lns1
	labs = [l.get_label() for l in lns]
	ax.legend(lns, labs, loc=0)
	ax.grid()
	plt.xlim(plt.xlim()[::-1])
	#ax.xaxis.get_offset_text().set_visible(False)
	ax.set_xlim(113.2,111.5)
	ax.set_xlabel('Z-coordinate of the tool-frame relative to the worldframe [mm]')
	ax.set_ylabel('Measured z-force in the tool-frame [N]')
	plt.title('Range (%i) stiffness calc' %i)
	#plt.autoscale(enable=True, axis='both', tight=None)
	plt.savefig("Fig_AD_Range_%i_k_calc.png" %i)
	if open_figures == True:
		mng = plt.get_current_fig_manager()
		mng.resize(*mng.window.maxsize())
		plt.show()
		plt.close()

# Force in function of position
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rc
rc('mathtext', default='regular')
fig = plt.figure()
ax = fig.add_subplot(111)
lns1 = ax.plot(Y_REF, REF_STIFF, '-k', label = 'Reference stiffness')
lns2 = ax.plot(Y_STIFF, CALC_STIFF, '-b', label = 'Calculated stiffness')
lns3 = ax.plot(Y_STIFF, ADJ_STIFF, '-r', label = 'Adjusted stiffness')
lns = lns1+lns2+lns3
labs = [l.get_label() for l in lns]
ax.legend(lns, labs, loc=0)
ax.grid()
#ax.xaxis.get_offset_text().set_visible(False)
#ax.set_xlim(113.2,111.5)
ax.set_xlabel('Y-coordinate of the tool-frame relative to the worldframe [mm]')
ax.set_ylabel('Stiffness [N/m]')
plt.title('Stiffness')
#plt.autoscale(enable=True, axis='both', tight=None)
plt.savefig("Fig_AD_Stiffness.png")
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
