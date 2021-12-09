import rosbag
from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import Float64MultiArray, Int32

# plot line of best for multiple robust regression algorithms
import numpy as np
from sklearn.datasets import make_regression
from sklearn.linear_model import LinearRegression
from sklearn.linear_model import HuberRegressor
from sklearn.linear_model import RANSACRegressor
from sklearn.linear_model import TheilSenRegressor
#from matplotlib import pyplot
import matplotlib.pyplot as plt

# dictionary of model names and model objects
def get_models():
	models = list()
	models.append(LinearRegression())
	models.append(HuberRegressor(epsilon=1))
	models.append(RANSACRegressor())
	models.append(TheilSenRegressor())
	return models

# plot the dataset and the model's line of best fit
def plot_best_fit(X, y, xaxis, model):
	# fit the model on all data
	model.fit(X, y)
	model_name = type(model).__name__
	if model_name != 'RANSACRegressor':
		k = model.coef_ * 1000
	else:
		k = 0

	# calculate outputs for grid across the domain
	yaxis = model.predict(xaxis.reshape((len(xaxis), 1)))
	# plot the line of best fit
	plt.plot(xaxis, yaxis, label='%s : %i' %(model_name, k) )

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

##################################### SEPERATE STATES
print(" >>>>> Seperating States")
state_list = []
state_list_sep = []
state_list = np.multiply(state_flags,state_flags_time)
for i in range(len(state_flags)-1):
	current_val = state_list[i]
	next_val = state_list[i+1]
	if current_val != 0 and next_val ==0:
		state_list_sep.append(current_val)

##################################### INITIAL STIFFNESS ESTIMATION
print(" >>>>> Calculate Initial Stiffness")
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

fit = np.polyfit(fitted_1, fitted_2, 1)
fit_slope = fit[0]*1000
CALC_STIFF.append(fit_slope)
ADJ_STIFF.append(fit_slope)

##################################### GET SCANNING DATA
print(" >>>>> Get Scanning Data")
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

##################################### ADAPTIVE STIFFNESS CALCULATION
print(" >>>>> Adaptive Stiffness Calculation")

startY = 0
Y_STIFF.append(startY)
stopY = 0.045
calc_distance = 0.005
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

	# load the dataset
	#X = fittedAD1
	#y = fittedAD2
	X = np.array(fittedAD1).reshape((-1, 1))
	y = np.array(fittedAD2)
	#print(y)
	#print(X)
	# define a uniform grid across the input domain
	fig = plt.figure()
	ax = fig.add_subplot(111)
	xaxis = np.arange(X.min(), X.max(), 0.01)
	for model in get_models():
		# plot the line of best fit
		plot_best_fit(X, y, xaxis, model)
	# plot the dataset
	plt.scatter(X, y, c='k')
	# show the plot
	plt.title('Robust Regression range %i' % i)
	plt.legend()
	plt.savefig("mygraph_%i.png" % i)
	#plt.show()
