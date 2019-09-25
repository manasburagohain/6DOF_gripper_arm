import numpy as np
import sys
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

    # Calculating phi as the euler angle about the y-axis in the base frame

    phi=np.array([joint_angles[1]+joint_angles[2]+joint_angles[4]])

    # Extracting the required x,y and z elements from H matrix

    H=H[0:3,-1]
    np.append(H, phi) 

    return 
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
    x=pose[0]
    y=pose[1]
    z=pose[2]
    phi=pose[3]

    # Defining the length of links
    # l1=122.14
    l1=112
    l2=105
    l3=126.77
    l4=122.12

    if z<=(l2+l3+l4): 
        #  Calculating the end effector location
        end_eff_dist=np.sqrt(x**2+y**2)

        # Calculating the angle of the base
        # Arctan convention is (y,x)
        base_theta=np.arctan2(y,x)

        # Calculation for angle for the elbow

        c_y=z+l4*np.cos(phi)-l1
        c_x=end_eff_dist-l4*np.sin(phi)

        d_c=np.sqrt(c_x**2+c_y**2)

        elbow_theta=np.arccos((d_c**2-l2**2-l3**2)/(2*l2*l3))

        # Calculation for angle for the shoulder

        theta_shoulder_1=np.arccos((l3**2-d_c**2-l2**2)/(-2*l2*d_c))
        theta_shoulder_2=np.arctan2(c_y,c_x)

        shoulder_theta=np.pi/2-theta_shoulder_1-theta_shoulder_2

        # Calculation for angle for the wrist
     
        w2_theta=np.pi-shoulder_theta-elbow_theta-phi

        angles=np.array([base_theta, shoulder_theta, elbow_theta,w2_theta])
        print (angles)

        # Defining angle limits
        angle_limits = np.array([
                            [-180, 179.99],
                            [-122, 122],
                            [-115, 104],
                            [-150, 150],
                            [-128, 129],], dtype=np.float)*(180/np.pi)

        # Checking if the angle are within limits

        for i in range(len(angles)):
            if angles[i]<angle_limits[i][0] or angles[i]>angle_limits[i][1]:
                print ("Not reachable")
                return None


        return (base_theta, shoulder_theta, elbow_th eta,w2_theta)
        pass

    else:
        print ("Not reachable")
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

if __name__ == '__main__':
    x=float(sys.argv[1])
    y=float(sys.argv[2])
    z=float(sys.argv[3])
    phi=float(sys.argv[4])
    IK([x,y,z,phi])
