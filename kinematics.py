import numpy as np
#expm is a matrix exponential function
from scipy.linalg import expm
from se3 import *

""" 
TODO: Here is where you will write all of your kinematics functions 
There are some functions to start with, you may need to implement a few more

"""

def FK_dh(joint_angles,link):
    """
    TODO: mplement this function
    a1=0
    d1

    Calculate forward kinematics for rexarm using DH convention
    return a transformation matrix representing the pose of the 
    desired link

    note: phi is the euler angle about the y-axis in the base frame

    """
    # print ("DOING DH")

    base_theta=joint_angles[0]
    shoulder_theta=joint_angles[1]
    elbow_theta=joint_angles[2]
    w1_theta=joint_angles[3]
    w2_theta=joint_angles[4]

    # Defining DH table parameters  

    # Distances are in mm
    d1=122.14 
    a2=105
    a3=126.77
    a4=122.12

    a=np.array([0,a2,a3,a4])
    alpha=np.array([np.pi/2,0,0,0])
    d=np.array([d1,0,0,0])
    theta=np.array([base_theta,shoulder_theta+np.pi/2,elbow_theta,w2_theta])

    # Defining functions to compute matrices

    def Trans_z_d (d):
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])

    def Trans_x_a (a):
        return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def Rot_z_theta (theta):
        return np.array([[np.cos(theta),-np.sin(theta),0,0],[np.sin(theta),np.cos(theta),0,0],[0,0,1,0],[0,0,0,1]])

    def Rot_x_alpha (alpha):
        return np.array([[1,0,0,0],[0,np.cos(alpha),-np.sin(alpha),0],[0,np.sin(alpha),np.cos(alpha),0],[0,0,0,1]])

    # Computing the H matrix 
    H=np.identity(4)
    
    for i in range(link-1):
        A=np.matmul(Rot_z_theta(theta[i]),np.matmul(Trans_z_d(d[i]),np.matmul(Trans_x_a(a[i]),Rot_x_alpha(alpha[i]))))
        H=np.matmul(H,A)

    return H[0:3,-1]
    pass

def FK_pox(joint_angles):
    """
    TODO: implement this function

    Calculate forward kinematics for rexarm
    using product of exponential formulation

    return a 4-tuple (x, y, z, phi) representing the pose of the 
    desired link

    note: phi is the euler angle about y in the base frame

    """
    pass

def IK(pose):
    """
    TODO: implement this function

    Calculate inverse kinematics for rexarm

    return the required joint angles

    """
    pass


def get_euler_angles_from_T(T):
    """
    TODO: implement this function
    return the Euler angles from a T matrix
    
    """
    pass

def get_pose_from_T(T):
    """
    TODO: implement this function
    return the joint pose from a T matrix
    of the form (x,y,z,phi) where phi is rotation about base frame y-axis
    
    """
    pass




def to_s_matrix(w,v):
    """
    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)
    """
    pass
