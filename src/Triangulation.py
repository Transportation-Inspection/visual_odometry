"""

Author: Pranjal Singh

This Software is part of the CS676: 3D Reconstruction from Several Images(Structure from Motion) project.
The Software can be found in pranjals16's GitHub repositories at:

https://github.com/pranjals16/cs676/tree/master/project/project_group6_cs676/code/final

"""

from Common_Modules import *

EPSILON = 0.0001

def LinearLSTriangulation(u, P, u1, P1):
  A = np.array([[u[0]*P[2,0]-P[0,0],u[0]*P[2,1]-P[0,1],u[0]*P[2,2]-P[0,2]],
               [u[1]*P[2,0]-P[1,0],u[1]*P[2,1]-P[1,1],u[1]*P[2,2]-P[1,2]],
               [u1[0]*P1[2,0]-P1[0,0],u1[0]*P1[2,1]-P1[0,1],u1[0]*P1[2,2]-P1[0,2]],
               [u1[1]*P1[2,0]-P1[1,0],u1[1]*P1[2,1]-P1[1,1],u1[1]*P1[2,2]-P1[1,2]]])
  B = np.array([[-(u[0]*P[2,3]-P[0,3])],
                [-(u[1]*P[2,3]-P[1,3])],
                [-(u1[0]*P1[2,3]-P1[0,3])],
                [-(u1[1]*P1[2,3]-P1[1,3])]])
  (retval, X) = cv2.solve(A,B,flags=cv2.DECOMP_SVD)
  return X

def IterativeLinearLSTriangulation(u, P, u1, P1):
  wi = 1
  wi1 = 1
  X = np.empty([4,1])
  for i in xrange(0, 10):
    X_ = LinearLSTriangulation(u,P,u1,P1)
    X[0] = X_[0] 
    X[1] = X_[1] 
    X[2] = X_[2] 
    X[3] = 1.0

    p2x = (P[2,].dot(X))[0]
    p2x1 = (P1[2,].dot(X))[0]

    if (abs(wi - p2x) <= EPSILON and abs(wi1 - p2x1) <= EPSILON):
      break

    wi = p2x
    wi1 = p2x1

    A = np.array([[(u[0]*P[2,0]-P[0,0])/wi, (u[0]*P[2,1]-P[0,1])/wi, (u[0]*P[2,2]-P[0,2])/wi],
                  [(u[1]*P[2,0]-P[1,0])/wi, (u[1]*P[2,1]-P[1,1])/wi, (u[1]*P[2,2]-P[1,2])/wi],
                  [(u1[0]*P1[2,0]-P1[0,0])/wi1, (u1[0]*P1[2,1]-P1[0,1])/wi1, (u1[0]*P1[2,2]-P1[0,2])/wi1],
                  [(u1[1]*P1[2,0]-P1[1,0])/wi1, (u1[1]*P1[2,1]-P1[1,1])/wi1, (u1[1]*P1[2,2]-P1[1,2])/wi1]])

    B = np.array([[-(u[0]*P[2,3]-P[0,3])/wi],
                  [-(u[1]*P[2,3]-P[1,3])/wi],
                  [-(u1[0]*P1[2,3]-P1[0,3])/wi1],
                  [-(u1[1]*P1[2,3]-P1[1,3])/wi1]])

    (retval, X_) = cv2.solve(A,B,flags=cv2.DECOMP_SVD)

    X[0] = X_[0]
    X[1] = X_[1]
    X[2] = X_[2] 
    X[3] = 1.0

  return X

def TriangulatePoints(pt_set1,pt_set2,K,P,P1):

  P1_ = np.array([[P1[0,0],P1[0,1],P1[0,2],P1[0,3]],
                  [P1[1,0],P1[1,1],P1[1,2],P1[1,3]],
                  [P1[2,0],P1[2,1],P1[2,2],P1[2,3]],
                  [0,		0,		0,		1]])
  P1inv = np.linalg.inv(P1_)

  (pts_size,_) = pt_set1.shape

  KP1 = K.dot(P1)
  Kinv = np.linalg.inv(K)
  pointcloud = []

  for i in xrange(0,pts_size):
    kp = pt_set1[i]
    u = np.array([kp[0],kp[1],1.0])
    um = Kinv.dot(u)
    u = um

    kp1 = pt_set2[i]
    u1 = np.array([kp1[0],kp1[0],1.0])
    um1 = Kinv.dot(u1) 
    u1 = um1
    
    X = IterativeLinearLSTriangulation(u,P,u1,P1)
    
    pointcloud.append((X[0,0],X[1,0],X[2,0]))

  return pointcloud
