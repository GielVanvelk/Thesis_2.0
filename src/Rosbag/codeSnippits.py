
##################################### ADAPTIVE STIFFNESS CALCULATION
print(" >>>>> Adaptive Stiffness Calculation 5/5")

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
print("Been here")
CALC_STIFF.append(stiffness_slope)
ADJ_STIFF.append(((1-STIFF_RATIO)*ADJ_STIFF[i])+(STIFF_RATIO*stiffness_slope))

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
ax.scatter(Y_REF, REF_STIFF, c='k', label = 'Reference stiffness')
ax.scatter(Y_STIFF, CALC_STIFF, c='b', label = 'Calculated stiffness')
ax.scatter(Y_STIFF, ADJ_STIFF, c='r', label = 'Adjusted stiffness')
#lns1 = ax.plot(Y_REF, REF_STIFF, '-k', label = 'Reference stiffness')
#lns2 = ax.plot(Y_STIFF, CALC_STIFF, '-b', label = 'Calculated stiffness')
#lns3 = ax.plot(Y_STIFF, ADJ_STIFF, '-r', label = 'Adjusted stiffness')
#lns = lns1+lns2+lns3
#labs = [l.get_label() for l in lns]
#ax.legend(lns, labs, loc=0)
ax.legend(loc='lower left')
ax.grid()
#ax.xaxis.get_offset_text().set_visible(False)
ax.set_xlim(-5,45)
ax.set_xlabel('Y-coordinate of the tool-frame relative to the worldframe [mm]')
ax.set_ylabel('Stiffness [N/m]')
plt.title('Stiffness')
#plt.autoscale(enable=True, axis='both', tight=None)
plt.savefig("Fig_AD_Stiffness.png")
mng = plt.get_current_fig_manager()
mng.resize(*mng.window.maxsize())
plt.show()
