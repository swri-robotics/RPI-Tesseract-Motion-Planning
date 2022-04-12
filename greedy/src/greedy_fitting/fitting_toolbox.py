import numpy as np
import sys,copy
sys.path.append('../circular_fit')
from toolbox_circular_fit import *
sys.path.append('../toolbox')
from robot_def import *

class fitting_toolbox(object):
	def __init__(self,robot,curve,curve_normal,curve_backproj_js,d=50):
		self.d=d 			###standoff distance
		self.curve=curve
		self.curve_backproj=curve-self.d*curve_normal
		self.curve_normal=curve_normal
		self.curve_backproj_js=curve_backproj_js

		self.robot=robot

		###get full orientation list
		self.curve_R=[]
		for i in range(len(curve_backproj_js)):
			self.curve_R.append(self.robot.fwd(curve_backproj_js[i]).R)

		###seed initial js for inv
		self.q_prev=curve_backproj_js[0]

		self.curve_fit=[]
		self.curve_fit_R=[]
		self.curve_fit_js=[]
		self.cartesian_slope_prev=None
		self.js_slope_prev=None
		
	
		###find path length
		self.lam=[0]
		for i in range(len(curve)-1):
			self.lam.append(self.lam[-1]+np.linalg.norm(curve[i+1]-curve[i]))

	def R2w(self, curve_R,R_constraint=[]):
		if len(R_constraint)==0:
			R_init=curve_R[0]
			curve_w=[np.zeros(3)]
		else:
			R_init=R_constraint
			R_diff=np.dot(curve_R[0],R_init.T)
			k,theta=R2rot(R_diff)
			k=np.array(k)
			curve_w=[k*theta]
		
		for i in range(1,len(curve_R)):
			R_diff=np.dot(curve_R[i],R_init.T)
			k,theta=R2rot(R_diff)
			k=np.array(k)
			curve_w.append(k*theta)
		return np.array(curve_w)
	def w2R(self,curve_w,R_init):
		curve_R=[]
		for i in range(len(curve_w)):
			theta=np.linalg.norm(curve_w[i])
			if theta==0:
				curve_R.append(R_init)
			else:
				curve_R.append(np.dot(rot(curve_w[i]/theta,theta),R_init))

		return np.array(curve_R)

	def get_angle(self,v1,v2,less90=False):
		v1=v1/np.linalg.norm(v1)
		v2=v2/np.linalg.norm(v2)
		dot=np.dot(v1,v2)
		if abs(dot)>0.999999:
			return 0
		angle=np.arccos(dot)
		if less90 and angle>np.pi/2:
			angle=np.pi-angle
		return angle

	def project(self,curve_fit,curve_fit_R):
		###project fitting curve by standoff distance
		curve_fit_proj=curve_fit+self.d*curve_fit_R[:,:,-1]

		return curve_fit_proj
	def movel_interp(self,p_start,p_end,steps):
		slope=(p_end-p_start)/(steps-1)
		return np.dot(np.arange(0,steps).reshape(-1,1),slope.reshape(1,-1))+p_start

	def orientation_interp(self,R_init,R_end,steps):
		curve_fit_R=[]
		###find axis angle first
		R_diff=np.dot(R_init.T,R_end)
		k,theta=R2rot(R_diff)
		for i in range(steps):
			###linearly interpolate angle
			angle=theta*i/(steps-1)
			R=rot(k,angle)
			curve_fit_R.append(np.dot(R_init,R))
		curve_fit_R=np.array(curve_fit_R)
		return curve_fit_R

	def car2js(self,curve_fit,curve_fit_R):
		###calculate corresponding joint configs
		curve_fit_js=[]
		for i in range(len(curve_fit)):
			q_all=np.array(self.robot.inv(curve_fit[i],curve_fit_R[i]))

			###choose inv_kin closest to previous joints
			temp_q=q_all-self.q_prev
			order=np.argsort(np.linalg.norm(temp_q,axis=1))
			curve_fit_js.append(q_all[order[0]])
		return curve_fit_js

	def quatera(self,curve_quat,initial_quat=[]):
		###quaternion regression
		if len(initial_quat)==0:
			Q=np.array(curve_quat).T
			Z=np.dot(Q,Q.T)
			u, s, vh = np.linalg.svd(Z)

			w=np.dot(quatproduct(u[:,1]),quatcomplement(u[:,0]))
			k,theta=q2rot(w)	#get the axis of rotation

			theta1=2*np.arctan2(np.dot(u[:,1],curve_quat[0]),np.dot(u[:,0],curve_quat[0]))
			theta2=2*np.arctan2(np.dot(u[:,1],curve_quat[-1]),np.dot(u[:,0],curve_quat[-1]))

			#get the angle of rotation
			theta=(theta2-theta1)%(2*np.pi)
			if theta>np.pi:
				theta-=2*np.pi

		else:
			###TODO: find better way for orientation continuous constraint 
			curve_quat_cons=np.vstack((curve_quat,np.tile(initial_quat,(999999,1))))
			Q=np.array(curve_quat_cons).T
			Z=np.dot(Q,Q.T)
			u, s, vh = np.linalg.svd(Z)

			w=np.dot(quatproduct(u[:,1]),quatcomplement(u[:,0]))
			k,theta=q2rot(w)

			theta1=2*np.arctan2(np.dot(u[:,1],curve_quat[0]),np.dot(u[:,0],curve_quat[0]))
			theta2=2*np.arctan2(np.dot(u[:,1],curve_quat[-1]),np.dot(u[:,0],curve_quat[-1]))

			#get the angle of rotation
			theta=(theta2-theta1)%(2*np.pi)
			if theta>np.pi:
				theta-=2*np.pi

		curve_fit_R=[]
		R_init=q2R(curve_quat[0])
		
		for i in range(len(curve_quat)):
			###linearly interpolate angle
			angle=theta*i/len(curve_quat)
			R=rot(k,angle)
			curve_fit_R.append(np.dot(R,R_init))
		curve_fit_R=np.array(curve_fit_R)
		return curve_fit_R
	
	def threshold_slope(self,slope_prev,slope,slope_thresh):
		slope_norm=np.linalg.norm(slope)
		slope_prev=slope_prev/np.linalg.norm(slope_prev)
		slope=slope.flatten()/slope_norm

		angle=np.arccos(np.dot(slope_prev,slope))

		if abs(angle)>slope_thresh:
			slope_ratio=np.sin(slope_thresh)/np.sin(abs(angle)-slope_thresh)
			slope_new=slope_prev+slope_ratio*slope
			slope_new=slope_norm*slope_new/np.linalg.norm(slope_new)

			return slope_new

		else:
			return slope


	def movel_fit(self,curve,curve_backproj,curve_backproj_js,curve_R,p_constraint=[],R_constraint=[],slope_constraint=[]):	###unit vector slope
		###convert orientation to w first
		curve_w=self.R2w(curve_R,R_constraint)
		
		###no constraint
		if len(p_constraint)==0:
			A=np.vstack((np.ones(len(curve_backproj)),np.arange(0,len(curve_backproj)))).T
			###assemble b matrix
			b=np.hstack((curve_backproj,curve_w))
			
			res=np.linalg.lstsq(A,b,rcond=None)[0]
			p_start_point=res[0,:3]
			w_start_point=res[0,3:]
			slope=res[1].reshape(1,-1)
			p_slope=slope[:,:3]
			w_slope=slope[:,3:]

			start_R=curve_R[0]
			curve_fit=np.dot(np.arange(0,len(curve_backproj)).reshape(-1,1),p_slope)+p_start_point
			curve_fit_w=np.dot(np.arange(0,len(curve_backproj)).reshape(-1,1),w_slope)+w_start_point

		###with constraint point
		else:
			p_start_point=p_constraint
			w_start_point=np.zeros(3)
			start_R=R_constraint

			if len(slope_constraint)!=0:
				
				p_slope=slope_constraint[:3].reshape(1,-1)*np.linalg.norm(curve_backproj[-1]-curve_backproj[0])/len(curve_backproj)
				w_slope=slope_constraint[-3:].reshape(1,-1)*np.linalg.norm(curve_w[-1]-curve_w[0])/len(curve_w)
			else:

				A=np.arange(1,len(curve_backproj)+1).reshape(-1,1)
				###assemble b matrix
				b=np.hstack((curve_backproj-p_start_point,(curve_w-w_start_point)))

				res=np.linalg.lstsq(A,b,rcond=None)[0]
				slope=res.reshape(1,-1)
				p_slope=slope[:,:3]
				w_slope=slope[:,3:]


			curve_fit=np.dot(np.arange(1,len(curve_backproj)+1).reshape(-1,1),p_slope)+p_start_point
			curve_fit_w=np.dot(np.arange(1,len(curve_backproj)+1).reshape(-1,1),w_slope)+w_start_point

		curve_fit_R=self.w2R(curve_fit_w,start_R)

		###calculate fitting error
		max_error1=np.max(np.linalg.norm(curve_backproj-curve_fit,axis=1))


		###calculate corresponding joint configs, leave black to skip inv during searching
		curve_fit_js=[]

		###calculating projection error
		curve_proj=self.project(curve_fit,curve_fit_R)
		max_error2=np.max(np.linalg.norm(curve-curve_proj,axis=1))
		max_error=(max_error1+max_error2)/2

		# print(max_error1,max_error2)
		return curve_fit,curve_fit_R,curve_fit_js,max_error


	def movej_fit(self,curve,curve_backproj,curve_backproj_js,curve_R,q_constraint=[],slope_constraint=[]):
		###no constraint
		if len(q_constraint)==0:
			A=np.vstack((np.ones(len(curve_backproj_js)),np.arange(0,len(curve_backproj_js)))).T
			b=curve_backproj_js
			res=np.linalg.lstsq(A,b,rcond=None)[0]
			start_point=res[0]
			slope=res[1].reshape(1,-1)

			start_pose=self.robot.fwd(curve_backproj_js[0])
			curve_fit_js=np.dot(np.arange(0,len(curve_backproj_js)).reshape(-1,1),slope)+start_point
		###with constraint point
		else:
			start_point=q_constraint
			start_pose=self.robot.fwd(start_point)

			if len(slope_constraint)!=0:
				slope=self.curve_fit_js*np.linalg.norm(curve_backproj_js[-1]-curve_backproj_js[0])/len(curve_backproj_js)
				slope=slope.reshape(1,-1)
			else:
				A=np.arange(1,len(curve_backproj_js)+1).reshape(-1,1)
				b=curve_backproj_js-start_point
				res=np.linalg.lstsq(A,b,rcond=None)[0]
				slope=res.reshape(1,-1)

			curve_fit_js=np.dot(np.arange(1,len(curve_backproj_js)+1).reshape(-1,1),slope)+start_point

		

		###necessary to fwd every search to get error calculation
		curve_fit=[]
		curve_fit_R=[]
		for i in range(len(curve_fit_js)):
			pose_temp=self.robot.fwd(curve_fit_js[i])
			curve_fit.append(pose_temp.p)
			curve_fit_R.append(pose_temp.R)
		curve_fit=np.array(curve_fit)
		curve_fit_R=np.array(curve_fit_R)

		###calculate fitting error
		max_error1=np.max(np.linalg.norm(curve_backproj-curve_fit,axis=1))

		###calculating projection error
		curve_proj=self.project(curve_fit,curve_fit_R)
		max_error2=np.max(np.linalg.norm(curve-curve_proj,axis=1))
		max_error=(max_error1+max_error2)/2

		return curve_fit,curve_fit_R,curve_fit_js,max_error


	def movec_fit(self,curve,curve_backproj,curve_backproj_js,curve_R,p_constraint=[],R_constraint=[],slope_constraint=[]):
		curve_w=self.R2w(curve_R,R_constraint)	

		###no previous constraint
		if len(p_constraint)==0:
			curve_fit,curve_fit_circle=circle_fit(curve_backproj)	
			###fit orientation with regression
			A=np.vstack((np.ones(len(curve_backproj)),np.arange(0,len(curve_backproj)))).T
			###assemble b matrix
			b=curve_w
			
			res=np.linalg.lstsq(A,b,rcond=None)[0]
			w_start_point=res[0]
			slope=res[1].reshape(1,-1)
			w_slope=slope

			start_R=curve_R[0]
			curve_fit_w=np.dot(np.arange(0,len(curve_backproj)).reshape(-1,1),w_slope)+w_start_point
		###with constraint point
		else:
			w_start_point=np.zeros(3)
			start_R=R_constraint
			if len(slope_constraint)!=0:
				curve_fit,curve_fit_circle=circle_fit_w_slope1(curve_backproj,p_constraint,slope_constraint[:3])

				w_slope=slope_constraint[-3:].reshape(1,-1)*np.linalg.norm(curve_w[-1]-curve_w[0])/len(curve_w)
				w_slope.reshape(1,-1)
			else:
				curve_fit,curve_fit_circle=circle_fit(curve_backproj,p_constraint)
				###fit orientation with regression
				A=np.arange(1,len(curve_backproj)+1).reshape(-1,1)
				###assemble b matrix
				b=curve_w-w_start_point

				res=np.linalg.lstsq(A,b,rcond=None)[0]
				slope=res.reshape(1,-1)
				w_slope=slope
			

			curve_fit_w=np.dot(np.arange(1,len(curve_backproj)+1).reshape(-1,1),w_slope)+w_start_point
		curve_fit_R=self.w2R(curve_fit_w,start_R)

		max_error1=np.max(np.linalg.norm(curve_backproj-curve_fit,axis=1))


		###calculate corresponding joint configs, leave black to skip inv during searching
		curve_fit_js=[]

		###calculating projection error
		curve_proj=self.project(curve_fit,curve_fit_R)
		max_error2=np.max(np.linalg.norm(curve-curve_proj,axis=1))
		max_error=(max_error1+max_error2)/2

		# print(max_error1,max_error2)
		return curve_fit,curve_fit_R,curve_fit_js,max_error

	def get_slope(self,curve_fit,curve_fit_R,breakpoints):
		slope_diff=[]
		slope_diff_ori=[]
		for i in range(1,len(breakpoints)-1):
			slope_diff.append(self.get_angle(curve_fit[breakpoints[i]-1]-curve_fit[breakpoints[i]-2],curve_fit[breakpoints[i]]-curve_fit[breakpoints[i]-1]))

			R_diff_prev=np.dot(curve_fit_R[breakpoints[i]],curve_fit_R[breakpoints[i-1]].T)
			k_prev,theta=R2rot(R_diff_prev)
			R_diff_next=np.dot(curve_fit_R[breakpoints[i+1]-1],curve_fit_R[breakpoints[i]].T)
			k_next,theta=R2rot(R_diff_next)
			slope_diff_ori.append(self.get_angle(k_prev,k_next,less90=True))

		return slope_diff,slope_diff_ori


def main():
	###read in points
	col_names=['X', 'Y', 'Z','direction_x', 'direction_y', 'direction_z'] 
	data = read_csv("../data/from_cad/Curve_in_base_frame.csv", names=col_names)
	curve_x=data['X'].tolist()
	curve_y=data['Y'].tolist()
	curve_z=data['Z'].tolist()
	curve=np.vstack((curve_x, curve_y, curve_z)).T


	curve_fit,max_error_all=fit_w_breakpoints(curve,[movel_fit,movec_fit,movec_fit],[0,int(len(curve)/3),int(2*len(curve)/3),len(curve)])

	print(max_error_all)

	fig = plt.figure()
	ax = plt.axes(projection='3d')
	ax.plot3D(curve[:,0], curve[:,1], curve[:,2], 'gray')
	ax.plot3D(curve_fit[:,0], curve_fit[:,1], curve_fit[:,2], 'red')

	plt.show()
if __name__ == "__main__":
	main()
