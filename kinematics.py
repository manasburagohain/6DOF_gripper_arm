import numpy as np
import sys
from math import *
#expm is a matrix exponential function
# from scipy.linalg import expm
# from se3 import *

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

    d1=118
    a2=99
    a3=112
    a4=109

    # d1=122.14 
    # a2=105
    # a3=126.77
    # a4=122.12

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
    
    for i in range(4):
        A=np.matmul(Rot_z_theta(theta[i]),np.matmul(Trans_z_d(d[i]),np.matmul(Trans_x_a(a[i]),Rot_x_alpha(alpha[i]))))
        H=np.matmul(H,A)

    # Calculating phi as the euler angle about the y-axis in the base frame

    phi=np.array([joint_angles[1]+joint_angles[2]+joint_angles[4]])

    # Extracting the required x,y and z elements from H matrix
    #print(H)
    H=H[0:3,-1]
    #print(H)
    np.append(H, phi) 

    return H
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

    # Inverse Kinematics Equations for Elbow Up Configuration

    # Extracting the required information from pose
   # Given link geometry
    l1=118
    l2=99
    l3=112
    l4=128

    x=pose[0]
    y=pose[1]
    z=pose[2]

    # extended distance

    d=sqrt(x**2+y**2)
    # print('d',d)
    # print('l23',l2+l3)
    if d<=l2+l3:
        # Base angle
        base_theta = atan2(y,x)
        #print(base_theta/np.pi*180)

        if z<=l1:
            zl1=l1-z
            # elbow angle theta2
            phi2 = np.arctan2(l4-zl1,d)
            m1_squared = d**2+(l4-zl1)**2
            elbow_theta = np.pi-acos((l2**2 + l3**2 - m1_squared)/(2*l2*l3))
            #print(elbow_theta/np.pi*180)
        else:
            zl1=z-l1
            # elbow angle theta2
            phi2 = np.arctan2(l4+zl1,d)
            m1_squared = d**2+(l4+zl1)**2
            elbow_theta = np.pi-acos((l2**2 + l3**2 - m1_squared)/(2*l2*l3))
            #print(elbow_theta/np.pi*180)


        # shoulder angle theta1
        phi1 = elbow_theta - acos((m1_squared+l3**2-l2**2)/(2*l3*sqrt(m1_squared)))
        shoulder_theta = np.pi-(phi1+phi2)
        #print(shoulder_theta/np.pi*180)

        # wrist angle theta3
        w2_theta = np.pi-(elbow_theta-phi1+np.pi/2-phi2)
        #print(w2_theta/np.pi*180)

        # print(base_theta,shoulder_theta-np.pi/2,elbow_theta,0,w2_theta)
        # print("Angle in degrees",base_theta*180/np.pi, shoulder_theta*180/np.pi, elbow_theta*180/np.pi,w2_theta*180/np.pi)

        return [[base_theta,shoulder_theta-np.pi/2,elbow_theta,0,w2_theta,0]]
        pass

    else:
        print ("Not reachable")
        return None


def IK2(pose, alpha):
    l1=118
    l2=99
    l3=112
    l4=128

    x = pose[0]
    y = pose[1]
    z = pose[2]
    alpha = alpha
    print("IK Worksapce Location:", pose)
    print("Arm Angle:", alpha)
    d=sqrt(x**2+y**2) # gripper distance away from origin in xy plane
    base_theta = atan2(y,x) # Base angle
    far_range = l2+l3+sin(alpha)*l4
    if d <= far_range: # Check valid furthest range
        # Two cases z<l1, z>l1
        if z<=l1: # z is low
            p = l4*cos(alpha)-(l1-z)
            theta2 = atan2(p/d)
            m2 = d**2+p**2
        else: # stack, z is high
            p = l4*cos(alpha)
            f = d-l4*sin(alpha)
            theta2 = atan2((p+z-l1)/f)
            m2 = f**2 + (z-l1+p)**2
        # The rest of IK are the same for both cases    
        if -1 <= (l2**2 + l3**2 - m2)/(2*l2*l3) <= 1: # arccos domain check
            elbow_theta = np.pi-acos((l2**2 + l3**2 - m2)/(2*l2*l3))
            phi1 = elbow_theta - acos((m2+l3**2-l2**2)/(2*l3*sqrt(m2)))
            shoulder_theta = np.pi-(phi1+phi2)
            w2_theta = np.pi-(elbow_theta-phi1+np.pi/2-phi2+alpha)
            print("IK Configuration:", [base_theta*180/np.pi, shoulder_theta*180/np.pi, elbow_theta*180/np.pi, 0, w2_theta*180/np.pi, 0])
            return [[base_theta, shoulder_theta-np.pi/2, elbow_theta, 0, w2_theta, 0]]
        else: # arccos domain check false
            return None
    else: # valid furthest range false
        return None


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

#
# if __name__ == '__main__':
#     x=float(sys.argv[1])#124.46
#     y=float(sys.argv[2])#129.54
#     z=float(sys.argv[3])#50.8
#     # print("X,Y,Z:", [124.46, 129.54, 50.8])
    # print("expecting:", [46.11, 137-90, 59.74,0, 75.22])
    # IK([x,y,z])