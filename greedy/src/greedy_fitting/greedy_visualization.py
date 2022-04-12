import matplotlib.pyplot as plt
from general_robotics_toolbox import *
from pandas import *
import sys, traceback
import numpy as np
sys.path.append('../toolbox')
from robot_def import *
from error_check import *
sys.path.append('../circular_fit')
from toolbox_circular_fit import *

def main():
	col_names=['X', 'Y', 'Z','direction_x', 'direction_y', 'direction_z'] 
	data = read_csv("../data/from_ge/Curve_backproj_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T

	col_names=['X', 'Y', 'Z','direction_x', 'direction_y', 'direction_z'] 
	data = read_csv("curve_fit_backproj.csv")
	curve_x=data['x'].tolist()
	curve_y=data['y'].tolist()
	curve_z=data['z'].tolist()
	curve_fit=np.vstack((curve_x, curve_y, curve_z)).T

	data = read_csv("command_backproj.csv")
	breakpoints=np.array(data['breakpoints'].tolist())
	primitives=data['primitives'].tolist()
	points=data['points'].tolist()

	breakpoints[1:]=breakpoints[1:]-1
	####only every 100 points
	steps=10
	curve=curve[0:breakpoints[-1]:steps]
	curve_fit=curve_fit[::steps]
	breakpoints=(breakpoints/steps).astype(int)


	###plane projection visualization
	curve_mean = curve.mean(axis=0)
	curve_centered = curve - curve_mean
	U,s,V = np.linalg.svd(curve_centered)
	# Normal vector of fitting plane is given by 3rd column in V
	# Note linalg.svd returns V^T, so we need to select 3rd row from V^T
	normal = V[2,:]

	curve_2d_vis = rodrigues_rot(curve_centered+curve_mean, normal, [0,0,1])[:,:2]
	curve_fit_2d_vis = rodrigues_rot(curve_fit, normal, [0,0,1])[:,:2]
	plt.plot(curve_2d_vis[:,0],curve_2d_vis[:,1])
	plt.plot(curve_fit_2d_vis[:,0],curve_fit_2d_vis[:,1])
	# plt.scatter(curve_fit_2d_vis[:,0],curve_fit_2d_vis[:,1])
	plt.scatter(curve_fit_2d_vis[breakpoints,0],curve_fit_2d_vis[breakpoints,1])
	plt.legend(['original curve','curve fit','breakpoints'])
	plt.xlim([-1300, -500])
	plt.ylim([200,1800])

	# fig = plt.figure()
	# ax = plt.axes(projection='3d')
	# ax.plot3D(curve[:,0], curve[:,1], curve[:,2],label='original',c='gray')
	# ax.plot3D(curve_fit[:,0], curve_fit[:,1], curve_fit[:,2],label='curve_fit',c='red')
	# ax.scatter(curve_fit[breakpoints.astype(int),0],curve_fit[breakpoints.astype(int),1],curve_fit[breakpoints.astype(int),2])
	# ax.legend()

	

	plt.show()


if __name__ == "__main__":
	main()