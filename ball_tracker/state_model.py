import numpy as np
from scipy import linalg

BALL_RADIUS = .05 #m
KAPPA = 3.0

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
		
	def laser_update(self, z, s):
		"""
		Fuse a laser measurement into the current state estimate with distance r in meters
		and variance s
		"""
		n = len(self.x) + 1
		laser_noise_state = np.vstack((self.x, 0))
		laser_noise_cov = np.zeros((n,n))
		laser_noise_cov[0:n-1,0:n-1] = self.P
		laser_noise_cov[n-1,n-1] = s
		
		X, w = self.sigma_points(laser_noise_state, laser_noise_cov)
		Xz = np.zeros((1,w.shape[0]))
		for i in range(X.shape[1]):
			Xz[0,i] = self.laser_measurement(X[:,i])
		
		Uz, Pz, Pxz = self.unscented_transform(Xz, X[0:n-1,:], w)
		res = z - Uz
		K = Pxz.dot(np.linalg.inv(Pz))
		newx = self.x + K*res		# TODO: Change this to a .dot
		newP = self.P - np.outer(Pz*K, K)
		self.x = newx
		self.P = newP
		
	def camera_update(self, z, s):
		"""
		Fuse a camera measurement into the current state estimate with z = (x pixel, y pixel, radius)
		and covariance matrix s
		"""
		nx = len(self.x)
		n = len(self.x) + len(z)
		cam_noise_state = np.vstack((self.x, np.zeros((len(z),1))))
		cam_noise_cov = np.zeros((n,n))
		cam_noise_cov[0:nx,0:nx] = self.P
		cam_noise_cov[nx:n,nx:n] = s
		
		X, w = self.sigma_points(cam_noise_state, cam_noise_cov)
		Xz = np.zeros((3,w.shape[0]))
		for i in range(X.shape[1]):
			Xz[:,i:i+1] = self.camera_measurement(X[:,i:i+1])
		
		Uz, Pz, Pxz = self.unscented_transform(Xz, X[0:n-3,:], w)
		print z.shape, Uz.shape
		res = z - Uz[:,None]
		K = Pxz.dot(np.linalg.inv(Pz))
		newx = self.x + K.dot(res)
		newP = self.P - K.dot(Pz.dot(K.T))
		self.x = newx
		self.P = newP
		
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
		
	def sigma_points(self, u, P):
		"""
		Returns the sigma points about the state u with covariance P
		Also returns the appropriate weight for each sigma point
		"""
		n = len(u)
		X = np.zeros((n, 2*n+1))
		w = np.zeros((2*n+1))
		
		# The first sigma point is the mean
		X[:,0] = u[:,0]
		w[0] = KAPPA/(n+KAPPA)
		
		X[:,1:n+1] = np.outer(np.ones(n),u).T + linalg.sqrtm((n+KAPPA)*P)
		w[1:n+1] = 1.0/(2*(n+KAPPA))*np.ones(n)
		X[:,n+1:2*n+1] = np.outer(np.ones(n),u).T - linalg.sqrtm((n+KAPPA)*P)
		w[n+1:2*n+1] = 1.0/(2*(n+KAPPA))*np.ones(n)
		
		return X, w
		
	def unscented_transform(self, Xz, X, w):
		"""
		Computes the unscented transform of a set of sigma points X using weights w.
		Returns the distributions new mean and covariance
		"""
		Uz = np.average(Xz, axis=1, weights = w)
		Ux = np.average(X, axis=1, weights =w)
		Xz_c = Xz - Uz[:,None]
		X_c = X - Ux[:,None]
		Pz = 0
		Pxz = 0
		for i in range(Xz_c.shape[1]):
			Pz = Pz + w[i]*np.outer(Xz_c[:,i],Xz_c[:,i])
			Pxz = Pxz + w[i]*np.outer(X_c[:,i],Xz_c[:,i])
		return Uz, Pz, Pxz
		
	def laser_measurement(self, x):
		"""
		Returns the laser measurement and covariance corresponding to state x
		"""
		r = np.sqrt(x[0]**2 + x[1]**2 + x[2]**2) 
		return r - BALL_RADIUS + x[8]
		
	def camera_measurement(self, x):
		"""
		Returns the camera measurement and covariance corresponding frame state x
		"""
		att = np.array([x[6,0], 0, x[7,0]])
		R1 = self.rotation_matrix(att)			  # Rotate from world frame to body frame
		R2 = self.body2cam_matrix()				  # Rotate from body frame to camera frame
		F = self.F								  # Project from camera frame to image coords
		
		pos_world = x[0:3,:]					  # Position of ball in world coordinates
		pos_cam = R2.dot(R1.dot(pos_world))		  # Position of ball in camera coordinates 
		edge_cam = pos_cam + np.array([[BALL_RADIUS],
									   [0],
									   [0]])	  # Position of ball's edge in camera coordinates
		pos_im = F.dot(pos_cam)					  # Position of ball in image coordinates
		edge_im = F.dot(edge_cam)
		
		#Convert from homogeneous to pixel coordinates
		pos_pix = np.array([[pos_im[0,0]/pos_im[2,0]],
				 		    [pos_im[1,0]/pos_im[2,0]]])
		edge_pix = np.array([[edge_im[0,0]/edge_im[2,0]],
				 		     [edge_im[1,0]/edge_im[2,0]]])
		r = np.linalg.norm(pos_pix-edge_pix)
		meas = np.vstack((pos_pix,r)) + x[8:11,:]
		return meas
		
	def rotation_matrix(self, att):
		"""
		Returns the right-handed frame rotation matrix for gimbal attitude att
		"""
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
		"""
		Returns the right-handed frame rotation matrix for converting from
		gimbal body coordinates to camera coordinates
		"""
		R = np.array([[1, 0,  0],
					  [0, 0, -1],
					  [0, 1,  0]])
		return R

def test():
	x = np.array([[0.,1.,0.,0.,0.,0.,0.,0.]]).T
	P = np.diag((.5,.5,.5, 1.,1.,1. ,1.,1.))
	bu = ball_ukf(x,P)
	z = np.array([[960,571,80]]).T
	s = np.diag((5,5,5))
	bu.camera_update(z,s)
	
	bu.laser_update(1.1,.05)
	print bu.x
	print bu.P
	
	x = np.array([[1,2,3,0,0,0,0,0,0]]).T
	print bu.laser_measurement(x)
	x = np.array([[0,1,0,0,0,0,0,0,0,0,0]]).T
	print bu.camera_measurement(x)
	
	x = np.array([[1,3]]).T
	P = np.array([[1,1],[1,3]])
	X, w = bu.sigma_points(x,P)
	u = np.average(X, axis=1, weights = w)
	X_c = X - u[:,None]
	cov = 0
	for i in range(X_c.shape[1]):
		cov = cov + w[i]*np.outer(X_c[:,i],X_c[:,i])
		
	print u, cov
	
	
	
	
	
test()