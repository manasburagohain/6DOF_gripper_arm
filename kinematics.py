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
    wrist_theta=joint_angles[2]
    w1_theta=joint_angles[3]
    w2_theta=joint_angles[4]

    # Defining DH table parameters  

    # Distances are in mm
    a2=100.5 
    d1=117.1
    d3=71
    d4=41.5

    a=np.array([0,a2,0,0,0])
    alpha=np.array([-np.pi/2,0,np.pi/2,-np.pi/2,np.pi/2])
    d=np.array([d1,0,0,d3+d4,0])
    theta=np.array([base_theta,shoulder_theta-np.pi/2,wrist_theta+np.pi/2,w1_theta,w2_theta])

    # Defining functions to compute matrices
    # Rot matrices are obtained from se3.py

    def Trans_z_d (d)
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])

    def Trans_x_a (a)
        return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    # Computing the H matrix 
    H=np.identity(4)
    
    for i in range(link)
        H=H.dot(aaToRot([0,0,theta[i]]).dot(Trans_z_d(d[i])).dot(Trans_x_a(a[i])).dot(aaToRot([alpha[i],0,0])))


    # # Defining d parameters
    # d1=76.2
    # d2=0
    # d3=0
    # d4=112
    # d5=0

    # A1= np.array([[np.cos(base_theta), -np.sin(base_theta)*np.cos(np.pi/2), np.sin(base_theta)*np.sin(np.pi/2), a1*np.cos(base_theta)], 
    #     [np.sin(base_theta), np.cos(base_theta)*np.cos(np.pi/2), -np.cos(base_theta)*np.sin(np.pi/2), a1*np.sin(base_theta)],
    #     [0,np.sin(np.pi/2),np.cos(np.pi/2),d1],
    #     [0,0,0,1]])

    # A2= np.array([[np.cos(shoulder_theta), -np.sin(shoulder_theta)*np.cos(0), np.sin(shoulder_theta)*np.sin(0), a2*np.cos(shoulder_theta)], 
    #     [np.sin(shoulder_theta), np.cos(shoulder_theta)*np.cos(0), -np.cos(shoulder_theta)*np.sin(0), a2*np.sin(shoulder_theta)],
    #     [0,np.sin(0),np.cos(0),d2],
    #     [0,0,0,1]])

    # A3= np.array([[np.cos(wrist_theta-np.pi/2), -np.sin(wrist_theta-np.pi/2)*np.cos(np.pi/2), np.sin(wrist_theta-np.pi/2)*np.sin(np.pi/2), a3*np.cos(wrist_theta-np.pi/2)], 
    #     [np.sin(wrist_theta-np.pi/2), np.cos(wrist_theta-np.pi/2)*np.cos(np.pi/2), -np.cos(wrist_theta-np.pi/2)*np.sin(np.pi/2), a3*np.sin(wrist_theta-np.pi/2)],
    #     [0,np.sin(np.pi/2),np.cos(np.pi/2),d3],
    #     [0,0,0,1]])

    # A4= np.array([[np.cos(w1_theta), -np.sin(w1_theta)*np.cos(-1*np.pi/2), np.sin(w1_theta)*np.sin(-1*np.pi/2), a4*np.cos(w1_theta)], 
    #     [np.sin(w1_theta), np.cos(w1_theta)*np.cos(-1*np.pi/2), -np.cos(w1_theta)*np.sin(-1*np.pi/2), a4*np.sin(w1_theta)],
    #     [0,np.sin(-1*np.pi/2),np.cos(-1*np.pi/2),d4],
    #     [0,0,0,1]])

    # A5= np.array([[np.cos(wrist_theta+np.pi/2), -np.sin(wrist_theta+np.pi/2)*np.cos(-1*np.pi/2), np.sin(wrist_theta+np.pi/2)*np.sin(-1*np.pi/2), a5*np.cos(w2_theta+np.pi/2)], 
    #     [np.sin(w2_theta+np.pi/2), np.cos(w2_theta+np.pi/2)*np.cos(-1*np.pi/2), -np.cos(w2_theta+np.pi/2)*np.sin(-1*np.pi/2), a5*np.sin(w2_theta+np.pi/2)],
    #     [0,np.sin(-1*np.pi/2),np.cos(-1*np.pi/2),d5],
    #     [0,0,0,1]])

    # H=np.matmul(np.matmul(np.matmul(A1,A2),np.matmul(A3,A4)),A5)

    # print ("H matrix is ", H)
    return H[0:3,2:3]
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