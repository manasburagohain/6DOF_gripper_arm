import matplotlib.pyplot as plt
import ast
from mpl_toolkits import mplot3d
import numpy as np
import json
fast_not_smooth = open("traj_1_not_smooth.txt","r")
#fast_not_smooth = open("traj_05_not_smooth.txt","r")


lines_fast_not_smooth = fast_not_smooth.readlines()
indices = [i for i, x in enumerate(lines_fast_not_smooth) if x == "and\n"]
traj1_fast_not_smooth = [ast.literal_eval(o) for o in lines_fast_not_smooth[0:indices[0]]]
traj2_fast_not_smooth = [ast.literal_eval(o) for o in lines_fast_not_smooth[indices[0]+1:indices[1]]]
traj3_fast_not_smooth =[ast.literal_eval(o) for o in lines_fast_not_smooth[indices[1]+1:]]
traj1_fast_not_smooth = np.array(traj1_fast_not_smooth).T
traj2_fast_not_smooth = np.array(traj2_fast_not_smooth).T
traj3_fast_not_smooth = np.array(traj3_fast_not_smooth).T

fast_smooth = open("traj_1_smooth.txt","r")
#fast_smooth = open("traj_05_smooth.txt","r") 
lines_fast_smooth = fast_smooth.readlines()
indices = [i for i, x in enumerate(lines_fast_smooth) if x == "and\n"]
print(indices)
lines1_fast_smooth = lines_fast_smooth[0:indices[0]]
lines2_fast_smooth = lines_fast_smooth[indices[0]+1:indices[1]]
lines3_fast_smooth = lines_fast_smooth[indices[1]+1:]
#print(lines1_fast_smooth)
def get_traj_from_line(lines):
	traj_fast_smooth = []
	for line in lines:
		xyz = line.split("[")[1].split("]")[0].split(" ")
		XYZ = []
		for a in xyz:
			if a is not "":
				XYZ.append(float(a))
		#print(XYZ)
		traj_fast_smooth.append(XYZ)
	traj_fast_smooth = np.array(traj_fast_smooth).T
	return traj_fast_smooth
traj1_fast_smooth = get_traj_from_line(lines1_fast_smooth)
traj2_fast_smooth = get_traj_from_line(lines2_fast_smooth)
traj3_fast_smooth = get_traj_from_line(lines3_fast_smooth)

x_diff = np.diff(traj2_fast_smooth[0])
y_diff = np.diff(traj2_fast_smooth[1])
z_diff = np.diff(traj2_fast_smooth[2])
dist_diff = np.sqrt(np.power(x_diff,2)+np.power(y_diff,2)+np.power(z_diff,2))
velo_fast_smooth = [0]+(np.abs(dist_diff/10/0.1)).tolist()+[0]
print(velo_fast_smooth)


x_diff = np.diff(traj2_fast_not_smooth[0])
y_diff = np.diff(traj2_fast_not_smooth[1])
z_diff = np.diff(traj2_fast_not_smooth[2])
dist_diff = np.sqrt(np.power(x_diff,2)+np.power(y_diff,2)+np.power(z_diff,2))
velo_fast_not_smooth = [0]+(np.abs(dist_diff/10/0.2)).tolist()+[0]
print(velo_fast_not_smooth)

velo_fast_not = [0]
for velo in velo_fast_not_smooth:
	if velo < 5:
		pass
	else:
		velo_fast_not+=[velo*2]
velo_fast_not+=[0]
print(velo_fast_not)


fig = plt.figure()
ax = fig.add_subplot(1, 3, 1, projection='3d')
ax.plot(traj1_fast_not_smooth[0],traj1_fast_not_smooth[1],traj1_fast_not_smooth[2],'-x', label='without smoothing')
ax.plot(traj1_fast_smooth[0],traj1_fast_smooth[1],traj1_fast_smooth[2],'-o', label='with smoothing')
ax.set_xlabel("x(mm)")
ax.set_ylabel("y(mm)")
ax.set_zlabel("z(mm)")
ax.legend()
ax = fig.add_subplot(1, 3, 2, projection='3d')
ax.plot(traj2_fast_not_smooth[0],traj2_fast_not_smooth[1],traj2_fast_not_smooth[2],'-x',label='without smoothing')
ax.plot(traj2_fast_smooth[0],traj2_fast_smooth[1],traj2_fast_smooth[2],'-o', label='with smoothing')
ax.set_xlabel("x(mm)")
ax.set_ylabel("y(mm)")
ax.set_zlabel("z(mm)")
ax.legend()
ax = fig.add_subplot(1, 3, 3, projection='3d')
ax.plot(traj3_fast_not_smooth[0],traj3_fast_not_smooth[1],traj3_fast_not_smooth[2],'-x',label='without smoothing')
ax.plot(traj3_fast_smooth[0],traj3_fast_smooth[1],traj3_fast_smooth[2],'-o', label='with smoothing')
ax.set_xlabel("x(mm)")
ax.set_ylabel("y(mm)")
ax.set_zlabel("z(mm)")
ax.legend()
plt.show()
'''

fig = plt.figure()
T = 0.1*len(velo_fast_smooth)
plt.plot(np.linspace(0, T, len(velo_fast_smooth)), velo_fast_smooth,label="with smoothing")
T = 0.1*len(velo_fast_not)
plt.plot(np.linspace(0,T,len(velo_fast_not)), velo_fast_not, label="without smoothing")
plt.xlabel('Time (s)')
plt.ylabel('Velocity of the end-effector (cm/s)')
plt.legend()
plt.show()
'''



