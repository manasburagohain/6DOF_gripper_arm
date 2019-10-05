import time
import numpy as np
import cv2
import csv
from math import * 
import kinematics as kine
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""

# Defining globa variables for sending to other scripts
affine_matrix_rgb=0
affine_rgb2depth=0
affine_depth2rgb=0
solverot = 0
solvetrans = 0
calibration_done=False
pixel_center=0
camMatrix = 0


class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.WC = [0,0,0]


    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "execute"):
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "execute"):
                self.execute()    
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "operation"):
                self.operation()
            if(self.next_state == "opplay"):
                self.opplay()    
            if(self.next_state == "FK_check"):
                self.FK_check()
            if(self.next_state == "block_detect"):
                self.block_detect()  
            if(self.next_state == "Task 1"):
                self.task1()
            if(self.next_state == "Task 2"):
                self.task1()
        
        if(self.current_state == "operation"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "execute"):
                self.execute()    
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()

        if(self.current_state == "opplay"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
        
        if(self.current_state == "block_detect"):
            if(self.next_state == "idle"):
                self.idle()

        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
               
        if(self.current_state == "FK_check"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "Task 1"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "Task 2"):
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
    """Functions run for each state"""


    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]

        # First Check if the Calibration Matrix is prepared before

        # strec = str(rec_joints)[1:-1]+"\n"
        # if path.exists("calibration.csv"):
        #     with open('op_joints.csv','a') as f:
        #         f.write(strec)

        #         with open("op_joints.csv") as f:
        #     csv_reader = csv.reader(f, delimiter = ",")
        #     for r in csv_reader :
        #         temp = np.array([float(o) for o in r])
        #         exec_joints.append(temp)
        # else :
        #     with open('op_joints.csv','w') as f:
        #         f.write(strec)

        # exec_joints = []
        

        # Extracting the location of the above positions in rgb camera image
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        # Extracting the location of the above positions in depth camera image
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        
        global camMatrix

        # Camera Instrinsic Matrix obtained through chequerboard calibration
        camMatrix = np.array([[562.04399308,0.00,327.78253347],
                              [0.00,559.59009967,249.41949401],
                                [0.00,0.00,1.00]])
        distCoeffs = np.array([2.86261843e-01,-1.06215288e+00,-6.38736068e-04,-6.91259011e-04,1.42539697e+00])

        # Recording the edge coordinates of the board locations in the world frame (in mm)

        world_coords = np.array([[-31.22,-29.72,0.0],[-31.22,30.61,3.85],[29.46,30.61,7.7],[29.46,-29.72,11.55]])
        #world_coords = np.array([[-31.22,-29.72],[-31.22,30.61],[29.46,30.61],[29.46,-29.72],[0.0,0.0]]) #mm

        # Storing the pixel coordinates recorded earlier in variable
        pixel_coords = self.kinect.rgb_click_points.copy()

        # Converting the pixel coordinates assuming center of board as camera (0,0)
        for i in range(4):
            for j in range (2):
                pixel_coords[i][j]=(-1)**j*(pixel_coords[i][j]-pixel_coords[4][j])

        #########################################################################
        #       SOLVING FOR AFFINE TRANSFORM MATRIX  FROM CAMERA TO WORLD       #
        #########################################################################

        # Need to solve Ax=B for x matrix (contains coeffiecient of 3X3 affine matrix)

        # Creating the A matrix from the pixel coordinates

        A = np.zeros(shape=(len(location_strings)*2-2,6))
        count=0
        for i in range(0,len(location_strings)-1):
                A[i+count]=[pixel_coords[i][0], pixel_coords[i][1], 1, 0, 0, 0]
                A[i+count+1]=[0, 0, 0, pixel_coords[i][0], pixel_coords[i][1], 1]
                count=count+1

        # B matrix contains the world coordinate points (measured)
        b=np.array([-29.5, -30.48, -29.5, 30.48,29.5,30.48, 29.5,-30.48])

        # Solving for Ax=B (Taking pseudo inverse since A is not square)
        x=(np.linalg.inv(A.T.dot(A))).dot(A.T).dot(b.T)
        # Matrix returned is 6x1 vector which is to be reshaped to 2x3 for generating 3x3 matrix
        x_matrix=np.reshape(x,(2,3))

        # Affine Matrix from transformation from image frame to world frame
        global affine_matrix_rgb
        affine_matrix_rgb=np.vstack((x_matrix,[0,0,1]))

        ##############################################################
        # AFFINE TRANSFORM FROM RGB TO DEPTH FRAME AND VICE-VERSA    #
        ##############################################################

        global affine_rgb2depth, affine_depth2rgb

        affine_rgb2depth = self.kinect.getAffineTransform(self.kinect.rgb_click_points,self.kinect.depth_click_points) #changed to affine from perspective
        affine_depth2rgb = self.kinect.getAffineTransform(self.kinect.depth_click_points,self.kinect.rgb_click_points)
        self.kinect.kinectCalibrated = True        

        ##################
        #Trying Solve PnP#
        ##################

        # new_pixel_coords = np.zeros(shape=(4,2),dtype=np.float32)
        # blah = self.kinect.rgb_click_points.copy()
        # for i in range(4):
        #     for j in range (2):
        #         new_pixel_coords[i][j]=blah[i][j]
        # print(new_pixel_coords)
        # world_coords.astype(np.float64)
        # ret, rotvec, transvec = cv2.solvePnP(world_coords,new_pixel_coords,camMatrix,distCoeffs)
        # rotmat,jac = cv2.Rodrigues(rotvec)
        # print(rotmat,transvec)
        # extmat = np.column_stack((rotmat,transvec))

        # global solverot, solvetrans
        # solverot = np.linalg.inv(rotmat)
        # solvetrans = transvec

        ''' 
        new_pixel_coords.astype(np.float32)
        world_coords.astype(np.float64)
        # print(new_pixel_coords)
        # print(type(new_pixel_coords),type(world_coords))
        ret, rotvec, transvec = cv2.solvePnP(world_coords,new_pixel_coords,camMatrix,distCoeffs)
        rotmat = cv2.Rodrigues(rotvec)
        print(rotmat, transvec)
        affine1 = np.concatenate((rotmat,transvec),axis=1)
        affinecv2 = np.concatenate((affine1,np.array([0,0,0,1])), axis=0)
        print(affine1)
        print(affinecv2)
        '''

        ########################################
        #   Recording Camera Origin Location   #
        ########################################

        # To record the location of pixel center
        global pixel_center
        pixel_center=np.array([pixel_coords[4][0],pixel_coords[4][1]]) 

        # If calibration has been done set value to True (Only once callibration is done should program display world coordinates)
        global calibration_done
        calibration_done=True

        # After Calibration the Robot Arm State should be idle
        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)
        self.next_state="idle"


    # Below Defined Functions are used to return affine transforms and pixel center locations
    def return_affine(self):
        return affine_matrix_rgb

    def return_rgb2depth(self):
        return affine_rgb2depth

    def return_depth2rgb(self):
        return affine_rgb2depth

    def return_solvepnp(self):
        return solverot, solvetrans

    def return_intrinsic(self):
        return camMatrix

    def calibration_state(self):
        return calibration_done

    def pixel_center_loc(self):
        return pixel_center

        
    # Creating function for executing picking a block and placing it at other locations
    def click_and_grab(self):
        print("Waiting to click the block")
        # Waiting for 3 seconds for user to click the block location
        time.sleep(3)

        # Check if the click has been made by the user
        if(self.kinect.new_click == True):
            # X, Y desired coordinates in camera image frame
            x =self.kinect.last_click[0].copy()
            y=self.kinect.last_click[1].copy()

            #############################################
            #       CAMERA FRAME TO DEPTH FRAME         #
            #############################################

            # Taking in the pixel values in camera frame and transforming to the kinect depth frame
            pixel_value=np.array([x,y])
            # Converting 10 bit depth to real distance using provided analytical function
            z = self.kinect.currentDepthFrame[int(pixel_value.item(1))][int(pixel_value.item(0))]
            Z = 12.36 * np.tan(float(z)/2842.5 + 1.1863)
            # 95 cm marks the z location of the base plane wrt to the camera. Subtracting 95 to measure +z from the base plane
            Z = 95-Z

            #############################################
            #       CAMERA FRAME TO WORLD FRAME         #
            #############################################

            # Extracting the origin of the camera frame (Following 4 quadrant system)
            pix_center=self.pixel_center_loc()
            # X and Y locations in the RGB space in pixels with (0,0) at the robot base center
            x=x-pix_center.item(0)
            y=pix_center.item(1)-y
            # Taking in the pixel values in camera frame and transforming to the world frame
            pixel_value=np.array([x,y])
            pixel_value=np.transpose(pixel_value)
            # Extracting the affine matrix computed during camera calibration
            affine=self.return_affine()
            affine=affine[0:2,0:2]
            # World x,y location corresponding to iamge frame x,y location
            world_value=np.matmul(affine,pixel_value)

            # Generating the pose matrix (Multiplying X,Y,Z by 10 since inputs to IK are in mm)
            # Pose 1 is position 3 cm above the block location
            pose1=[world_value.item(0)*10,world_value.item(1)*10,(Z+3)*10]
            # Pose 2 is position to grab the block
            pose2=[world_value.item(0)*10,world_value.item(1)*10,Z*10]
            print ("X, Y, Z values of the location to pick block is ",pose2)
            
            # Calling the Inverse Kinematics function to determine the required joint angles for Pose 1
            execute_states = kine.IK(pose1)

            # Trajectory Planning to Pick the block
            if execute_states is not None:
                self.rexarm.toggle_gripper() # open
                
                for i, wp in enumerate(execute_states):
                    if i==0 and wp==np.zeros(self.rexarm.num_joints).tolist():
                        pass
                    else:
                        print(wp)
                        # print(type(wp))
                        initial_wp = self.tp.set_initial_wp()
                        final_wp = self.tp.set_final_wp(wp)
                        T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 1)
                        plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                        self.tp.execute_plan(plan_pts, plan_velos)
                        self.rexarm.pause(1)
                
                # Calling the Inverse Kinematics function to determine the required joint angles for Pose 2 
                
                down_states = kine.IK(pose2)
                if down_states is not None:
                    for i, wp in enumerate(down_states):
                        if i==0 and wp==np.zeros(self.rexarm.num_joints).tolist():
                            pass
                        else:
                            print(wp)
                            print(type(wp))
                            initial_wp = self.tp.set_initial_wp()
                            final_wp = self.tp.set_final_wp(wp)
                            T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 0.2)
                            plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                            self.tp.execute_plan(plan_pts, plan_velos)
                            self.rexarm.pause(1)
                    self.rexarm.toggle_gripper() #close

                    ## Once the block has been picked the arm should open up to ensure block is properly gripped. This pose is defined by idlePos
                    idlePos = [[0.0, 0, 0.0, 0.0, -np.pi/4,0]]

                    for i, wp in enumerate(idlePos):
                        if i==0 and wp==np.zeros(self.rexarm.num_joints).tolist():
                            pass
                        else:
                            print(wp)
                            print(type(wp))
                            initial_wp = self.tp.set_initial_wp()
                            final_wp = self.tp.set_final_wp(wp)
                            T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 1)
                            plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                            self.tp.execute_plan(plan_pts, plan_velos)
                            self.rexarm.pause(1)
                    self.rexarm.toggle_gripper()
                    self.rexarm.toggle_gripper()

                    # Now the system can accept new input from user
                    self.kinect.new_click = 0
            else:
                print("Not Reachable, Retry")


    def click_and_grab_task1(self, block_coordinates, drop_coordinates):
        print("Executing Task 1")
        # Waiting for 3 seconds for user to click the block location
        time.sleep(3)

        ####################################################
        #       CAMERA FRAME TO DEPTH FRAME FUNCTION        #
        ####################################################

        def find_z_at_xy(x,y):
            # Taking in the pixel values in camera frame and transforming to the kinect depth frame
            pixel_value=np.array([x,y])
            # Converting 10 bit depth to real distance using provided analytical function
            z = self.kinect.currentDepthFrame[int(pixel_value.item(1))][int(pixel_value.item(0))]
            Z = 12.36 * np.tan(float(z)/2842.5 + 1.1863)
            # 95 cm marks the z location of the base plane wrt to the camera. Subtracting 95 to measure +z from the base plane
            Z = 95-Z
            return Z

        def pixel_to_world_coords(x,y):
            #############################################
            #       CAMERA FRAME TO WORLD FRAME         #
            #############################################
            # Extracting the origin of the camera frame (Following 4 quadrant system)
            pix_center=self.pixel_center_loc()
            # X and Y locations in the RGB space in pixels with (0,0) at the robot base center
            x=x-pix_center.item(0)
            y=pix_center.item(1)-y
            # Taking in the pixel values in camera frame and transforming to the world frame
            pixel_value=np.array([x,y])
            pixel_value=np.transpose(pixel_value)
            # Extracting the affine matrix computed during camera calibration
            affine=self.return_affine()
            affine=affine[0:2,0:2]
            # print('affine:',affine,'\nxy:',pixel_value)
            # World x,y location corresponding to iamge frame x,y location
            world_value=np.matmul(affine,pixel_value.T)
            return (world_value)

        def execute_fast_movement(pose_togo):
            for i, wp in enumerate(pose_togo):
                if i==0 and wp==np.zeros(self.rexarm.num_joints).tolist():
                    pass
                else:
                    # print(wp)
                    # print(type(wp))
                    initial_wp = self.tp.set_initial_wp()
                    final_wp = self.tp.set_final_wp(wp)
                    T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 1)
                    plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                    self.tp.execute_plan(plan_pts, plan_velos)
                    self.rexarm.pause(1)

        def execute_slow_movement(pose_togo):
            for i, wp in enumerate(pose_togo):
                if i==0 and wp==np.zeros(self.rexarm.num_joints).tolist():
                    pass
                else:
                    # print(wp)
                    # print(type(wp))
                    initial_wp = self.tp.set_initial_wp()
                    final_wp = self.tp.set_final_wp(wp)
                    T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 0.2)
                    plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                    self.tp.execute_plan(plan_pts, plan_velos)
                    self.rexarm.pause(1)


        # Check if the click has been made by the user
        if(len(block_coordinates) == 2):

            x=block_coordinates[0]
            y=block_coordinates[1]

            Z=find_z_at_xy(x,y)

            world_value=pixel_to_world_coords(x,y)

            
            # Generating the pose matrix (Multiplying X,Y,Z by 10 since inputs to IK are in mm)
            # Pose 1 is position 3 cm above the block location
            pose1=[world_value.item(0)*10,world_value.item(1)*10,(Z+3)*10]
            # Pose 2 is position to grab the block
            pose2=[world_value.item(0)*10,world_value.item(1)*10,Z*10]
            # print ("X, Y, Z values of the location to pick block is ",pose2)
            
            # Calling the Inverse Kinematics function to determine the required joint angles for Pose 1
            execute_states = kine.IK(pose1)

            # Trajectory Planning to Pick the block and drop it at place
            if execute_states is None:
                print ("Cannot go to pose above the block location for picking")
            else:
                print ("Goint to step 1 to pick item at pose",execute_states)
                print("Z to pick up item 3 cm above is",Z+3)
                self.rexarm.toggle_gripper() # open
                execute_fast_movement(execute_states)
                
                # Calling the Inverse Kinematics function to determine the required joint angles for Pose 2 
                
                down_states = kine.IK(pose2)
                if down_states is None:
                    print("Cannot go to pose to drop the block")
                else:
                    print ("Goint to step 2 to pick item at pose",down_states)
                    print("Z to pick up item above is",Z)
                    execute_slow_movement(down_states)
                    self.rexarm.toggle_gripper() #close

                    ## Once the block has been picked the arm should open up to ensure block is properly gripped. This pose is defined by idlePos
                    idlePos = [[0.0, 0, 0.0, 0.0, -np.pi/4,0]]
                    execute_fast_movement(idlePos)
                    self.rexarm.toggle_gripper() # Opening the gripper
                    self.rexarm.toggle_gripper() # Closing the gripper

                    # Dropping the block as per Block Drop Coordinates
                    
                    x_drop=drop_coordinates[0][0]  # These are in world reference frame
                    y_drop=drop_coordinates[1][0]  # Hence no conversion to word coordinate frame as before

                    z_drop=find_z_at_xy(x_drop,y_drop)

                    world_value=pixel_to_world_coords(x_drop,y_drop)

                    # Constructing the pose for pose 

                    pose_drop_intermediate=[world_value.item(0)*10,world_value.item(1)*10,(z_drop+7)*10]
                    down_states_intermediate = kine.IK(pose_drop_intermediate)

                    if down_states_intermediate is None:
                        print ("Cannot go to pose above the block location for dropping")
                    else:
                        print ("Goint to step 3 to drop item at pose",down_states_intermediate)
                        print ("Z to drop up item 3 cm above is",z_drop+7)
                        execute_fast_movement(down_states_intermediate)

                        pose_drop=[world_value.item(0)*10,world_value.item(1)*10,(z_drop+3)*10]
                        down_states = kine.IK(pose_drop)
                        if down_states is None:
                            print("Cannot go to pose to drop the block (block picked up")
                        else:
                            print ("Goint to step 4 to drop item at pose",down_states)
                            print ("Z to drop up item 1 cm above is",z_drop+3)
                            execute_slow_movement(down_states)
                            self.rexarm.toggle_gripper() # Opening the gripper

                            pose_interm_up=[world_value.item(0)*10,world_value.item(1)*10,(z_drop+7)*10]
                            intermediate_up_states = kine.IK(pose_interm_up)
                            if intermediate_up_states is None:
                                print("Cannot go to pose to drop the block (block picked up")
                            else:
                                print ("Goint to step 5 to interm up at pose",intermediate_up_states)
                                execute_slow_movement(intermediate_up_states)
                                # self.rexarm.toggle_gripper() # Opening the gripper


                                idlePos = [[0.0, 0, 0.0, 0.0, -np.pi/4,0]]
                                execute_fast_movement(idlePos)
                                self.rexarm.toggle_gripper() # Opening the gripper
                            # self.rexarm.toggle_gripper() # Closing the gripper
                    

    def execute(self):

        self.click_and_grab()
        self.rexarm.get_feedback()
        self.next_state = "idle"

    def pass_through_waypoints(self):
        # Define the waypoint to pass through
        execute_states= np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [ 1.0, 0.8, 1.0, 0.5, 1.0],
                                  [-1.0,-0.8,-1.0,-0.5, -1.0],
                                  [-1.0, 0.8, 1.0, 0.5, 1.0],
                                  [1.0, -0.8,-1.0,-0.5, -1.0],
                                  [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        execute_states.tolist()
        self.status_message = "State: Execute - Following Set Path"
        self.current_state = "execute"

        for i, wp in enumerate(execute_states):
            if i==0 and wp.tolist()==np.zeros(wp.shape).tolist():
                pass
            else:
                initial_wp = self.tp.set_initial_wp()
                final_wp = self.tp.set_final_wp(wp)
                T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 1)
                plan_pts, plan_velos = self.tp.generate_quintic_spline(initial_wp, final_wp,T)
                self.tp.execute_plan(plan_pts, plan_velos)
                self.rexarm.pause(1)
                self.rexarm.toggle_gripper()
                self.rexarm.toggle_gripper()
        #for i,_ in enumerate(execute_states) :
        #    self.rexarm.set_positions(execute_states[i])
        #    self.rexarm.pause(1)
        

    def operation(self):
        self.status_message = "Operation - Recording User Positions"
        self.current_state = "operation"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()

    def opplay(self):
        self.status_message = "Operation Play- Following saved positions"
        self.current_state = "opplay"
        exec_joints = []
        with open("op_joints.csv") as f:
            csv_reader = csv.reader(f, delimiter = ",")
            for r in csv_reader :
                temp = np.array([float(o) for o in r])
                exec_joints.append(temp)
        #print(exec_joints)
        for i, wp in enumerate(exec_joints):
            if i==0 and wp.tolist()==np.zeros(wp.shape).tolist():
                pass
            else:
                initial_wp = self.tp.set_initial_wp()
                final_wp = self.tp.set_final_wp(wp)
                print(initial_wp)
                T = self.tp.calc_time_from_waypoints(initial_wp, final_wp, 1)
                plan_pts, plan_velos = self.tp.generate_cubic_spline(initial_wp, final_wp,T)
                self.tp.execute_plan(plan_pts, plan_velos)
                self.rexarm.pause(1)
                self.rexarm.toggle_gripper()
                self.rexarm.toggle_gripper()


        #for i,_ in enumerate(exec_joints) :
        #    self.rexarm.set_positions(exec_joints[i])
        #    self.rexarm.pause(1)
        self.rexarm.get_feedback()
        self.next_state = "idle"

    def block_detect(self):
        self.status_message = "Detecting Blocks"
        self.current_state = "block_detect"
        self.kinect.detectBlocksInDepthImage()
        self.kinect.blockDetector()
        self.next_state = "idle"

    def FK_check(self):
        self.status_message = "Checking Forward Kinematics"
        self.current_state = "FK_check"
        kine.FK_dh(self.rexarm.get_positions())
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        self.next_state="idle"


    def task1(self):
        self.status_message = "Performing task 1"
        self.current_state = "Task 1"
        # Calling the block detection function to detect block contours
        self.block_detect()
        # Denoting the location for dropping the block
        drop_coordinates=np.array([[350],[350]])
        count=0

        for i in range(len(self.kinect.block_coordinates)-2):
            block_coordinates=np.array([[self.kinect.block_coordinates[i+count]],[self.kinect.block_coordinates[i+count+1]]])
            self.click_and_grab_task1(block_coordinates, drop_coordinates)
            count=count+1

        self.next_state = "idle"


    def task1(self):
        self.status_message = "Performing task 2"
        self.current_state = "Task 2"
        # Calling the block detection function to detect block contours
        self.block_detect()
        # Denoting the location for dropping the block
        drop_coordinates=np.array([[350],[350]])
        count=0
        distance=0

        for i in range(len(self.kinect.block_coordinates)-2):
            block_coordinates=np.array([[self.kinect.block_coordinates[i+count]],[self.kinect.block_coordinates[i+count+1]]])
            self.click_and_grab_task1(block_coordinates, drop_coordinates)
            drop_coordinates=np.array([[350+distance],[350]])
            distance=distance+50
            count=count+1

        self.next_state = "idle"



