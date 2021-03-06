'''
train_svm.py:
	Train two class SVM for ball tracking.  Save the model as 'model/clf.pkl'
Author:
	Ankush Gola
'''

import numpy as np
import scipy as sp
import matplotlib.image as mpimg
from sklearn import svm
from sklearn.externals import joblib

def flatten(img):
	"""
	Flatten a color image into a vector of pixels
	"""
	M, N, P = img.shape
	return img.reshape(M*N, P)

ball_imgs = ['ball_s_1', 'ball_s_2', 'ball_s_3', 'ball_s_5', 'ball_s_6']
back_imgs = ['back_t_1', 'back_t_2', 'back_t_3', 'back_t_4']

ball, back = {}, {}

# Flatten images into a single vector
for img in ball_imgs:
	ball_img = (mpimg.imread('train/'+img+'.jpg').astype(float))/255
	train = flatten(ball_img)
	ball[img] = train

for img in back_imgs:
	back_img = (mpimg.imread('train/'+img+'.jpg').astype(float))/255
	train = flatten(back_img)
	back[img] = train

X_ball, X_back = np.vstack(tuple([ball[k] for k in ball])), np.vstack(tuple([back[k] for k in back]))
y_ball, y_back = np.ones(X_ball.shape[0]).astype(np.uint8), np.zeros(X_back.shape[0]).astype(np.uint8)

X = np.vstack((X_ball, X_back))
y = np.hstack((y_ball, y_back))

# Train the SVM
C = 30
gamma = 4
clf = svm.SVC(C=C, kernel='rbf', gamma=gamma, cache_size=1500).fit(X, y)

# Save the model
joblib.dump(clf, 'model/clf.pkl')