from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
per_true = open("perception_true.txt","r")
per_calc = open("perception_calc.txt","r")
z_true=3.8
z_calc=4.0
per_true = per_true.readlines()
per_calc = per_calc.readlines()


def get_XY(datas):
	XY = []
	for line in datas:
		#print(line)
		xy = line.split("\n")[0].split(",")
		xy = [float(o) for o in xy]
		XY.append(xy)
	XY = np.array(XY).T
	return XY

XY_true =get_XY(per_true)
print(XY_true)
Z_true = z_true *np.ones(len(XY_true[0]))
print(Z_true)

XY_calc = get_XY(per_calc)
Z_calc = z_calc *np.ones(len(XY_calc[0]))

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.plot(XY_true[0],XY_true[1],Z_true,'x', label='True z=3.8cm')
ax.plot(XY_calc[0],XY_calc[1],Z_calc,'o', label='Calculated z=4cm')
ax.set_xlabel("x(cm)")
ax.set_ylabel("y(cm)")
ax.set_zlabel("z(cm)")
ax.set_zlim(0,5)
ax.legend()
plt.show()
