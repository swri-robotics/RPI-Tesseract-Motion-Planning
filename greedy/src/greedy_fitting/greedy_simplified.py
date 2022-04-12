import numpy as np
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d.axes3d import Axes3D
import matplotlib.pyplot as plt
from pandas import *
from fitting_toolbox import *
import sys, argparse
sys.path.append('../circular_fit')
from toolbox_circular_fit import *
sys.path.append('../toolbox')
from robots_def import *
from direction2R import *
from general_robotics_toolbox import *
from error_check import *
# from robotstudio_send import MotionSend

#####################3d curve-fitting with MoveL, MoveJ, MoveC; stepwise incremental bi-section searched self.breakpoints###############################

class greedy_fit(fitting_toolbox):
	def __init__(self,robot,curve,curve_normal,curve_backproj_js,d=50):
		super().__init__(robot,curve,curve_normal,curve_backproj_js,d)
		self.slope_constraint=np.radians(180)
		self.break_early=False

		###initial primitive candidates
		self.primitives={'movel_fit':self.movel_fit_greedy,'movej_fit':self.movej_fit_greedy,'movec_fit':self.movec_fit_greedy}

	def movel_fit_greedy(self,curve,curve_backproj,curve_backproj_js,curve_R):	###unit vector slope
		
		curve_fit,curve_fit_R,curve_fit_js,max_error=self.movel_fit(curve,curve_backproj,curve_backproj_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [])
		###get slopes
		slope=curve_fit[-1]-curve_fit[0]
		R_diff=np.dot(curve_fit_R[-1],curve_fit_R[0].T)
		k,theta=R2rot(R_diff)

		if len(self.cartesian_slope_prev)>0:
			slope_diff1=self.get_angle(slope,self.cartesian_slope_prev)
			ori_slope_diff1=self.get_angle(k,self.rotation_axis_prev,less90=True)
		else:
			slope_diff1=0
			ori_slope_diff1=0

		next_point_idx=self.breakpoints[-1]+len(curve_fit)
		slope_diff2=self.get_angle(slope,self.curve_backproj[min(next_point_idx+500,len(self.curve)-1)]-curve_fit[-1])

		rotation_axis_next,theta=R2rot(np.dot(self.curve_R[min(next_point_idx+500,len(self.curve)-1)],curve_fit_R[-1].T))
		ori_slope_diff2=self.get_angle(k,rotation_axis_next,less90=True)

		if np.any(np.array([slope_diff1,slope_diff2,ori_slope_diff1,ori_slope_diff2])>self.slope_constraint):

			slope_temp=slope if slope_diff1<self.slope_constraint else self.threshold_slope(self.cartesian_slope_prev,slope,self.slope_constraint)
			rot_axis_temp=k if ori_slope_diff1<self.slope_constraint else self.threshold_slope(self.rotation_axis_prev,k,self.slope_constraint)

			constrained_slope=np.hstack((slope_temp,rot_axis_temp))




			curve_fit,curve_fit_R,curve_fit_js,max_error=self.movel_fit(curve,curve_backproj,curve_backproj_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [],constrained_slope)
			###get slopes
			slope=curve_fit[-1]-curve_fit[0]
			R_diff=np.dot(curve_fit_R[-1],curve_fit_R[0].T)
			k,theta=R2rot(R_diff)

			if len(self.cartesian_slope_prev)>0:
				slope_diff1=self.get_angle(slope,self.cartesian_slope_prev)
				ori_slope_diff1=self.get_angle(k,self.rotation_axis_prev,less90=True)
			else:
				slope_diff1=0
				ori_slope_diff1=0

			next_point_idx=self.breakpoints[-1]+len(curve_fit)
			slope_diff2=self.get_angle(slope,self.curve_backproj[min(next_point_idx+500,len(self.curve)-1)]-curve_fit[-1])

			rotation_axis_next,theta=R2rot(np.dot(self.curve_R[min(next_point_idx+500,len(self.curve)-1)],curve_fit_R[-1].T))
			ori_slope_diff2=self.get_angle(k,rotation_axis_next,less90=True)

			print(np.degrees([slope_diff1,slope_diff2,ori_slope_diff1,ori_slope_diff2]))

			if np.any(np.array([slope_diff1,slope_diff2,ori_slope_diff1,ori_slope_diff2])>self.slope_constraint):
				return [],[],[],999
		return curve_fit,curve_fit_R,curve_fit_js,max_error


	def movej_fit_greedy(self,curve,curve_backproj,curve_backproj_js,curve_R):
		

		return self.movej_fit(curve,curve_backproj,curve_backproj_js,curve_R,self.curve_fit_js[-1] if len(self.curve_fit_js)>0 else [])


	def movec_fit_greedy(self,curve,curve_backproj,curve_backproj_js,curve_R):
		curve_fit,curve_fit_R,curve_fit_js,max_error=self.movec_fit(curve,curve_backproj,curve_backproj_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [])
		###get slopes
		slope_init=curve_fit[1]-curve_fit[0]
		slope_init_end=curve_fit[-1]-curve_fit[-2]
		R_diff=np.dot(curve_fit_R[-1],curve_fit_R[0].T)
		k,theta=R2rot(R_diff)

		if len(self.cartesian_slope_prev)>0:
			slope_diff1=self.get_angle(slope_init,self.cartesian_slope_prev)
			ori_slope_diff1=self.get_angle(k,self.rotation_axis_prev,less90=True)
		else:
			slope_diff1=0
			ori_slope_diff1=0

		next_point_idx=self.breakpoints[-1]+len(curve_fit)
		slope_diff2=self.get_angle(slope_init_end,self.curve_backproj[min(next_point_idx+500,len(self.curve)-1)]-curve_fit[-1])

		rotation_axis_next,theta=R2rot(np.dot(self.curve_R[min(next_point_idx+500,len(self.curve)-1)],curve_fit_R[-1].T))
		ori_slope_diff2=self.get_angle(k,rotation_axis_next,less90=True)

		if np.any(np.array([slope_diff1,slope_diff2,ori_slope_diff1,ori_slope_diff2])>self.slope_constraint):

			slope_temp=slope_init if slope_diff1<self.slope_constraint else self.threshold_slope(self.cartesian_slope_prev,slope_init,self.slope_constraint)
			rot_axis_temp=k if ori_slope_diff1<self.slope_constraint else self.threshold_slope(self.rotation_axis_prev,k,self.slope_constraint)

			constrained_slope=np.hstack((slope_temp,rot_axis_temp))




			curve_fit,curve_fit_R,curve_fit_js,max_error=self.movec_fit(curve,curve_backproj,curve_backproj_js,curve_R,self.curve_fit[-1] if len(self.curve_fit)>0 else [],self.curve_fit_R[-1] if len(self.curve_fit_R)>0 else [],constrained_slope)
			###get slopes
			slope_init=curve_fit[1]-curve_fit[0]
			slope_init_end=curve_fit[-1]-curve_fit[-2]
			R_diff=np.dot(curve_fit_R[-1],curve_fit_R[0].T)
			k,theta=R2rot(R_diff)

			if len(self.cartesian_slope_prev)>0:
				slope_diff1=self.get_angle(slope_init,self.cartesian_slope_prev)
				ori_slope_diff1=self.get_angle(k,self.rotation_axis_prev,less90=True)
			else:
				slope_diff1=0
				ori_slope_diff1=0

			next_point_idx=self.breakpoints[-1]+len(curve_fit)
			slope_diff2=self.get_angle(slope_init_end,self.curve_backproj[min(next_point_idx+500,len(self.curve)-1)]-curve_fit[-1])

			rotation_axis_next,theta=R2rot(np.dot(self.curve_R[min(next_point_idx+500,len(self.curve)-1)],curve_fit_R[-1].T))
			ori_slope_diff2=self.get_angle(k,rotation_axis_next,less90=True)


			if np.any(np.array([slope_diff1,slope_diff2,ori_slope_diff1,ori_slope_diff2])>self.slope_constraint):
				return [],[],[],999
		return curve_fit,curve_fit_R,curve_fit_js,max_error

	def fit_under_error(self,max_error_threshold):

		###initialize
		self.breakpoints=[0]
		primitives_choices=[]
		points=[]


		results_max_cartesian_error=[]
		results_max_cartesian_error_index=[]
		results_avg_cartesian_error=[]
		results_max_orientation_error=[]
		results_max_dz_error=[]
		results_avg_dz_error=[]

		self.curve_fit=[]
		self.curve_fit_R=[]
		self.curve_fit_js=[]
		self.cartesian_slope_prev=[]
		self.rotation_axis_prev=[]
		self.slope_diff=[]
		self.slope_diff_ori=[]
		self.js_slope_prev=None

		while self.breakpoints[-1]<len(self.curve)-1:
			
			next_point = min(2000,len(self.curve)-self.breakpoints[-1])
			prev_point=0
			prev_possible_point=0

			max_errors={'movel_fit':999,'movej_fit':999,'movec_fit':999}
			###initial error map update:
			for key in self.primitives: 
				curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives[key](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
				max_errors[key]=max_error

			###bisection search self.breakpoints
			while True:
				print('index: ',self.breakpoints[-1]+next_point,'max_error: ',max_errors[min(max_errors, key=max_errors.get)])
				###bp going backward to meet threshold
				if min(list(max_errors.values()))>max_error_threshold:
					prev_point_temp=next_point
					next_point-=int(np.abs(next_point-prev_point)/2)
					prev_point=prev_point_temp
					
					for key in self.primitives: 
						curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives[key](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
						max_errors[key]=max_error



				###bp going forward to get close to threshold
				else:
					prev_possible_point=next_point
					prev_point_temp=next_point
					next_point= min(next_point + int(np.abs(next_point-prev_point)),len(self.curve)-self.breakpoints[-1])
					prev_point=prev_point_temp
					

					for key in self.primitives: 
						curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives[key](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
						max_errors[key]=max_error

				# print(max_errors)
				if next_point==prev_point:
					print('stuck, restoring previous possible index')		###if ever getting stuck, restore
					next_point=max(prev_possible_point,2)
					# if self.breakpoints[-1]+next_point+1==len(self.curve)-1:
					# 	next_point=3

					primitives_added=False
					for key in self.primitives: 
						curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives[key](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
						if max_error<max_error_threshold:
							primitives_added=True
							primitives_choices.append(key)
							if key=='movec_fit':
								points.append([curve_fit[int(len(curve_fit)/2)],curve_fit[-1]])
							elif key=='movel_fit':
								points.append([curve_fit[-1]])
							else:
								points.append([curve_fit_js[-1]])
							break
					if not primitives_added:
						curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives['movej_fit'](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
						print('primitive skipped1')
						primitives_choices.append('movej_fit')
						points.append([curve_fit_js[-1]])
	
					break

				###find the closest but under max_threshold
				if (min(list(max_errors.values()))<=max_error_threshold and np.abs(next_point-prev_point)<10):
					primitives_added=False
					for key in self.primitives: 
						curve_fit,curve_fit_R,curve_fit_js,max_error=self.primitives[key](self.curve[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_backproj_js[self.breakpoints[-1]:self.breakpoints[-1]+next_point],self.curve_R[self.breakpoints[-1]:self.breakpoints[-1]+next_point])
						if max_error<max_error_threshold:
							primitives_added=True
							primitives_choices.append(key)
							if key=='movec_fit':
								points.append([curve_fit[int(len(curve_fit)/2)],curve_fit[-1]])
							elif key=='movel_fit':
								points.append([curve_fit[-1]])
							else:
								points.append([curve_fit_js[-1]])	
							break
					if not primitives_added:
						print('primitive skipped2')
						primitives_choices.append('movel_fit')
						points.append([curve_fit[-1]])

					break
	

			if self.break_early:
				idx=max(self.breakearly(self.curve_backproj[self.breakpoints[-1]:self.breakpoints[-1]+next_point],curve_fit),2)
			else:
				idx=next_point

			self.breakpoints.append(min(self.breakpoints[-1]+idx,len(self.curve)))
			self.curve_fit.extend(curve_fit[:len(curve_fit)-(next_point-idx)])
			self.curve_fit_R.extend(curve_fit_R[:len(curve_fit)-(next_point-idx)])

			if primitives_choices[-1]=='movej_fit':
				self.curve_fit_js.extend(curve_fit_js[:len(curve_fit)-(next_point-idx)])
			else:
				###inv here to save time
				self.curve_fit_js.extend(self.car2js(curve_fit[:len(curve_fit)-(next_point-idx)],curve_fit_R[:len(curve_fit)-(next_point-idx)]))
			###calculating ending slope
			R_diff=np.dot(curve_fit_R[0].T,curve_fit_R[-(next_point-idx)-1])
			k,theta=R2rot(R_diff)
			if len(self.cartesian_slope_prev)>0:
				self.slope_diff.append(self.get_angle(self.cartesian_slope_prev,curve_fit[1]-curve_fit[0]))
				self.slope_diff_ori.append(self.get_angle(self.rotation_axis_prev,k))

			self.rotation_axis_prev=k	
			self.cartesian_slope_prev=(self.curve_fit[-1]-self.curve_fit[-2])/np.linalg.norm(self.curve_fit[-1]-self.curve_fit[-2])
			self.js_slope_prev=(self.curve_fit_js[-1]-self.curve_fit_js[-2])/np.linalg.norm(self.curve_fit_js[-1]-self.curve_fit_js[-2])
			self.q_prev=self.curve_fit_js[-1]


			print(self.breakpoints)
			print(primitives_choices)
			
			# if len(self.breakpoints)>2:
			# 	break

		##############################check error (against fitting back projected curve)##############################

		# max_error,max_error_idx=calc_max_error(self.curve_fit,self.curve_backproj)
		# print('max error: ', max_error)
		print('slope diff: ',np.degrees(self.slope_diff))
		print('slope diff ori: ',np.degrees(self.slope_diff_ori))
		self.curve_fit=np.array(self.curve_fit)
		self.curve_fit_R=np.array(self.curve_fit_R)
		self.curve_fit_js=np.array(self.curve_fit_js)

		return self.breakpoints,primitives_choices,points

	def smooth_slope(self,slope_diff,slope_diff_ori,curve_fit,curve_fit_R,breakpoints,primitives_choices,points):
		new_breakpoints=[0]
		new_breakpoint_idx=[]
		primitives_choices_new=[primitives_choices[0]]
		points_new=[points[0]]
		steps=150
		for i in range(len(slope_diff)):
			if slope_diff[i] > np.radians(0) or slope_diff_ori[i] > np.radians(0):
				
				# curve_fit,curve_fit_circle=circle_fit(self.curve_backproj[self.breakpoints[i+1]-steps:min(self.breakpoints[i+1]+steps,len(self.curve_fit))],self.curve_fit[self.breakpoints[i+1]-steps],self.curve_fit[min(self.breakpoints[i+1]+steps-1,len(self.curve_fit)-1)])
				
				###add blending
				if breakpoints[i+1]+steps-1>len(curve_fit):
					new_breakpoints.append(breakpoints[i+1])
					break

				###moveC blending
				slope1=curve_fit[breakpoints[i+1]-steps]-curve_fit[breakpoints[i+1]-steps-2]
				slope2=curve_fit[breakpoints[i+1]+steps+2]-curve_fit[breakpoints[i+1]+steps]
				curve_fit_temp,curve_fit_circle=circle_fit_w_2slope(self.curve_backproj[breakpoints[i+1]-steps:breakpoints[i+1]+steps],curve_fit[breakpoints[i+1]-steps],curve_fit[breakpoints[i+1]+steps-1],slope1,slope2)
				
				###moveL blending
				# curve_fit_temp=self.movel_interp(curve_fit[breakpoints[i+1]-steps],curve_fit[breakpoints[i+1]+steps-1],steps*2)
				
				
				###orientation interpolation
				curve_fit_R_temp=self.orientation_interp(curve_fit_R[breakpoints[i+1]-steps],curve_fit_R[breakpoints[i+1]+steps-1],steps*2)

				curve_fit[breakpoints[i+1]-steps:breakpoints[i+1]+steps]=curve_fit_temp
				curve_fit_R[breakpoints[i+1]-steps:breakpoints[i+1]+steps]=curve_fit_R_temp

				primitives_choices_new.append(primitives_choices[i+1])
				if primitives_choices_new[-1]=='movec_fit':
					points_new.append([curve_fit[int((new_breakpoints[-1]+breakpoints[i+1]-steps)/2)],curve_fit_temp[0]])
				else:
					points_new.append([curve_fit_temp[0]])

				###moveC blending
				primitives_choices_new.append('movec_fit')
				points_new.append([curve_fit_temp[int(len(curve_fit_temp)/2)],curve_fit_temp[-1]])
				###moveL blending
				# primitives_choices_new.append('movel_fit')
				# points_new.append([curve_fit_temp[-1]])




				new_breakpoints.append(breakpoints[i+1]-steps)
				new_breakpoints.append(breakpoints[i+1]+steps-1)
				
				new_breakpoint_idx.extend([len(new_breakpoints)-2,len(new_breakpoints)-1])
			else:
				new_breakpoints.append(breakpoints[i+1])
				primitives_choices_new.append(primitives_choices[i+1])
				points_new.append(points[i+1])

		new_breakpoints.append(len(self.curve_backproj))
		primitives_choices_new.append(primitives_choices[-1])
		points_new.append(points[-1])

		self.curve_fit=np.array(curve_fit)
		self.curve_fit_R=np.array(curve_fit_R)

		self.curve_fit_js=np.array(self.car2js(self.curve_fit,self.curve_fit_R))

		slope_diff,slope_diff_ori=self.get_slope(self.curve_fit,self.curve_fit_R,new_breakpoints)


		print("new slope diff: ",np.degrees(slope_diff))
		print("new ori diff: ",np.degrees(slope_diff_ori))

		max_error1=np.max(np.linalg.norm(self.curve_backproj-self.curve_fit,axis=1))
		curve_proj=self.project(self.curve_fit,self.curve_fit_R)
		max_error2=np.max(np.linalg.norm(self.curve-curve_proj,axis=1))
		max_error=(max_error1+max_error2)/2
		# print('new final error: ',max_error)
		print(len(new_breakpoints),len(primitives_choices_new),len(points_new))
		return new_breakpoints,primitives_choices_new,points_new



def main():
	#Accept the names of the robots from command line
	parser = argparse.ArgumentParser()
	parser.add_argument("--file-name",type=str, default='abb6640.yml')
	args, _ = parser.parse_known_args()
	file_name=args.file_name

	###read in robot info file
	import yaml
	with open('../toolbox/robot_info/'+file_name) as file:
		robot_yml=yaml.full_load(file)
		kin_chain=robot_yml['chains'][0]
		joint_info=robot_yml['joint_info']
		tool_pose=kin_chain['flange_pose']

	###kin chain
	H = []
	P = []

	for i in range(len(kin_chain['H'])):
		H.append(list(kin_chain['H'][i].values()))
		P.append(list(kin_chain['P'][i].values()))
	P.append(list(kin_chain['P'][-1].values()))
	H=np.array(H).reshape((len(kin_chain['H']),3)).T
	P=np.array(P).reshape((len(kin_chain['P']),3)).T*1000	###make sure in mm

	###joint info
	joint_type=[]	
	upper_limit=[]
	lowerer_limit=[]
	joint_vel_limit=[]
	for i in range(len(joint_info)):
		joint_type.append(0 if joint_info[i]['joint_type']=='revolute' else 1)
		upper_limit.append(joint_info[i]['joint_limits']['upper'])
		lowerer_limit.append(joint_info[i]['joint_limits']['lower'])
		joint_vel_limit.append(joint_info[i]['joint_limits']['velocity'])

	###tool pose
	R_tool=q2R(list(tool_pose['orientation'].values()))
	p_tool=np.array(list(tool_pose['position'].values()))*1000

	###create a robot
	robot=arb_robot(H,P,joint_type,upper_limit,lowerer_limit, joint_vel_limit,R_tool=R_tool,p_tool=p_tool)

	###read in points
	col_names=['X', 'Y', 'Z','direction_x', 'direction_y', 'direction_z'] 
	data = read_csv("../data/from_ge/Curve_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve_direction_x=data['direction_x'].tolist()
	curve_direction_y=data['direction_y'].tolist()
	curve_direction_z=data['direction_z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T
	curve_normal=np.vstack((curve_direction_x, curve_direction_y, curve_direction_z)).T

	col_names=['q1', 'q2', 'q3','q4', 'q5', 'q6'] 
	data = read_csv("../data/from_ge/curve_backproj_js.csv", names=col_names)
	curve_q1=data['q1'].tolist()
	curve_q2=data['q2'].tolist()
	curve_q3=data['q3'].tolist()
	curve_q4=data['q4'].tolist()
	curve_q5=data['q5'].tolist()
	curve_q6=data['q6'].tolist()
	curve_js=np.vstack((curve_q1, curve_q2, curve_q3,curve_q4,curve_q5,curve_q6)).T

	greedy_fit_obj=greedy_fit(robot,curve,curve_normal,curve_js,d=50)


	###set primitive choices, defaults are all 3
	greedy_fit_obj.primitives={'movel_fit':greedy_fit_obj.movel_fit_greedy,'movec_fit':greedy_fit_obj.movec_fit_greedy}

	breakpoints,primitives_choices,points=greedy_fit_obj.fit_under_error(1.)

	###plt
	###3D plot
	plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(greedy_fit_obj.curve[:,0], greedy_fit_obj.curve[:,1],greedy_fit_obj.curve[:,2], 'gray')
	
	ax.scatter3D(greedy_fit_obj.curve_fit[:,0], greedy_fit_obj.curve_fit[:,1], greedy_fit_obj.curve_fit[:,2], c=greedy_fit_obj.curve_fit[:,2], cmap='Greens')
	plt.show()

	############insert initial configuration#################
	primitives_choices.insert(0,'movej_fit')
	q_all=np.array(robot.inv(greedy_fit_obj.curve_fit[0],greedy_fit_obj.curve_fit_R[0]))
	###choose inv_kin closest to previous joints
	temp_q=q_all-curve_js[0]
	order=np.argsort(np.linalg.norm(temp_q,axis=1))
	q_init=q_all[order[0]]
	points.insert(0,[q_init])

	print(len(breakpoints))
	print(len(primitives_choices))
	print(len(points))

	df=DataFrame({'breakpoints':breakpoints,'primitives':primitives_choices,'points':points})
	df.to_csv('command_backproj.csv',header=True,index=False)
	df=DataFrame({'x':greedy_fit_obj.curve_fit[:,0],'y':greedy_fit_obj.curve_fit[:,1],'z':greedy_fit_obj.curve_fit[:,2],\
		'R1':greedy_fit_obj.curve_fit_R[:,0,0],'R2':greedy_fit_obj.curve_fit_R[:,0,1],'R3':greedy_fit_obj.curve_fit_R[:,0,2],\
		'R4':greedy_fit_obj.curve_fit_R[:,1,0],'R5':greedy_fit_obj.curve_fit_R[:,1,1],'R6':greedy_fit_obj.curve_fit_R[:,1,2],\
		'R7':greedy_fit_obj.curve_fit_R[:,2,0],'R8':greedy_fit_obj.curve_fit_R[:,2,1],'R9':greedy_fit_obj.curve_fit_R[:,2,2]})
	df.to_csv('curve_fit_backproj.csv',header=True,index=False)
	df=DataFrame({'j1':greedy_fit_obj.curve_fit_js[:,0],'j2':greedy_fit_obj.curve_fit_js[:,1],'j3':greedy_fit_obj.curve_fit_js[:,2],'j4':greedy_fit_obj.curve_fit_js[:,3],'j5':greedy_fit_obj.curve_fit_js[:,4],'j6':greedy_fit_obj.curve_fit_js[:,5]})
	df.to_csv('curve_fit_js.csv',header=False,index=False)

if __name__ == "__main__":
	main()
