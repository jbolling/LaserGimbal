import numpy as np

BALL_RADIUS = .05 #m

class ball_ukf:
	"""
	State vector:
	x		Right along gimbal's base
	y		Away from gimabl
	z		Up
	Vx		velocity in x
	Vy		velocity in y
	Vz		velocity in z
	theta	gimbal pitch
	phi		gimbal yaw
	"""
	def __init__(self, x0, P0):
		"""
		Create a new ball_ukf object with initial state x0 and covariance P0
		"""
		self.x = x0
		self.P = P0
		
		C920_data = np.load("C920_calib_data.npz")
		self.F = C920_data['intrinsic_matrix']
		
	def propogate_state(self,tstep,u,Q):
		"""
		Propagate the modules state estimate tstep seconds into the future with 
		control input u and disturbance covariance Q
		"""
		self.x, self.P = self.predict(tstep,u,Q)
		
	def predict(self, tstep, u, Q):
		"""
		Propagate the state and covariance matrix to a time t seconds in
		the future
		"""
		A = np.array([[1, 0, 0, tstep,     0,     0, 0, 0],
					  [0, 1, 0,     0, tstep,     0, 0, 0],
					  [0, 0, 1,     0,     0, tstep, 0, 0],
					  [1, 0, 0,     0,     0,     0, 0, 0],
					  [0, 1, 0,     0,     0,     0, 0, 0],
					  [0, 0, 1,     0,     0,     0, 0, 0],
					  [0, 0, 0,     0,     0,     0, 0, 0],
					  [0, 0, 0,     0,     0,     0, 0, 0]])
		B = np.array([[0, 0],
					  [0, 0],
					  [0, 0],
					  [0, 0],
					  [0, 0],
					  [0, 0],
					  [1, 0],
					  [0, 1]])
		N = np.array([[0, 0, 0, 0, 0],
					  [0, 0, 0, 0, 0],
					  [0, 0, 0, 0, 0],
					  [1, 0, 0, 0, 0],
					  [0, 1, 0, 0, 0],
					  [0, 0, 1, 0, 0],
					  [0, 0, 0, 1, 0],
					  [0, 0, 0, 0, 1]])
		
		xnew = A.dot(self.x) + B.dot(u)
		Pnew = A.dot(self.P.dot(A.T)) +  N.dot(Q.dot(N.T))
		return xnew, Pnew
		
	def laser_measurement(self, x):
		"""
		Returns the laser measurement and covariance corresponding to state x
		"""
		r = np.sqrt(x[0]**2 + x[1]**2 + x[2]**2)
		return r - BALL_RADIUS
		
	def camera_measurement(self, x):
		"""
		Returns the camera measurement and covariance corresponding frame state x
		"""
		att = np.array([x[6,0], 0, x[7,0]])
		R1 = self.rotation_matrix(att)			  # Rotate from world frame to body frame
		R2 = self.body2cam_matrix()				  # Rotate from body frame to camera frame
		F = self.F								  # Project from camera frame to image coords
		K = F.dot(R2.dot(R1))
		pos = x[0:3,:]
		im_hom = K.dot(pos)
		meas = np.array([im_hom[0,0]/im_hom[2,0],
						 im_hom[1,0]/im_hom[2,0]])
		return meas
		
	def rotation_matrix(self, att):    
		att = np.deg2rad(att)
		r_pitch = np.array([[ np.cos(att[1]), 0, -np.sin(att[1])],
							[              0, 1,               0],
							[ np.sin(att[1]), 0,  np.cos(att[1])]])
		r_roll = np.array([[1,               0,              0],
						   [0,  np.cos(att[0]), np.sin(att[0])],
						   [0, -np.sin(att[0]), np.cos(att[0])]])
		r_yaw = np.array([[  np.cos(att[2]), np.sin(att[2]), 0],
						  [ -np.sin(att[2]), np.cos(att[2]), 0],
						  [               0,              0, 1]])
		R = r_roll.dot(r_pitch.dot(r_yaw))
		return R
		
	def body2cam_matrix(self):
		R = np.array([[1, 0,  0],
					  [0, 0, -1],
					  [0, 1,  0]])
		return R

def test():
	bu = ball_ukf(None,None)
	x = np.array([[1,2,3,0,0,0,0,0]]).T
	print bu.laser_measurement(x)
	x = np.array([[0,1,0,0,0,0,0,0]]).T
	print bu.camera_measurement(x)
	
test()