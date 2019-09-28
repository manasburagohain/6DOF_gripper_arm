import time
import numpy as np
import cv2
import csv
from math import * 
import kinematics as kine
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""

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
        
        

        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
               
        if(self.current_state == "FK_check"):
            if(self.next_state == "idle"):
                self.idle()
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
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        # print(self.kinect.rgb_click_points)
        # print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""
        #global affine_rgb2depth
        #affine_rgb2depth = self.kinect.getAffineTransform(self.kinect.rgb_click_points, self.kinect.depth_click_points)
        
        global camMatrix
        camMatrix = np.array([[562.04399308,0.00,327.78253347],[0.00,559.59009967,249.41949401],[0.00,0.00,1.00]])
        distCoeffs = np.array([2.86261843e-01,-1.06215288e+00,-6.38736068e-04,-6.91259011e-04,1.42539697e+00])

        # Hardcoding the edge coordinates of the board

        world_coords = np.array([[-31.22,-29.72,0.0],[-31.22,30.61,3.85],[29.46,30.61,7.7],[29.46,-29.72,11.55]]) #mm
        #world_coords = np.array([[-31.22,-29.72],[-31.22,30.61],[29.46,30.61],[29.46,-29.72],[0.0,0.0]]) #mm

        pixel_coords = self.kinect.rgb_click_points.copy()

        #global affine_matrix_rgb
        #affine_matrix_rgb=self.kinect.getAffineTransform(world_coords, pixel_coords)

        new_pixel_coords = np.zeros(shape=(4,2),dtype=np.float32)
        # Converting the pixel coordinates assuming center of board as camera (0,0)
        for i in range(4):
            for j in range (2):
                pixel_coords[i][j]=(-1)**j*(pixel_coords[i][j]-pixel_coords[4][j])

        A=np.array([[pixel_coords[0][0], pixel_coords[0][1], 1, 0, 0, 0],
                    [0, 0, 0, pixel_coords[0][0], pixel_coords[0][1], 1],
                    [pixel_coords[1][0], pixel_coords[1][1], 1, 0, 0, 0],
                    [0, 0, 0, pixel_coords[1][0], pixel_coords[1][1], 1],
                    [pixel_coords[2][0], pixel_coords[2][1], 1, 0, 0, 0],
                    [0, 0, 0, pixel_coords[2][0], pixel_coords[2][1], 1],
                    [pixel_coords[3][0], pixel_coords[3][1], 1, 0, 0, 0],
                    [0, 0, 0, pixel_coords[3][0], pixel_coords[3][1], 1]])

        b=np.array([-29.5, -30.48, -29.5, 30.48,29.5,30.48, 29.5,-30.48])

        x=(np.linalg.inv(A.T.dot(A))).dot(A.T).dot(b.T)
        x_matrix=np.reshape(x,(2,3))

        global affine_matrix_rgb
        affine_matrix_rgb=np.vstack((x_matrix,[0,0,1]))
        #print('Affine Trans Matrix is:', affine_matrix_rgb)

        ##################
        #Trying Solve PnP#
        ##################
        blah = self.kinect.rgb_click_points.copy()
        for i in range(4):
            for j in range (2):
                new_pixel_coords[i][j]=blah[i][j]
        print(new_pixel_coords)
        world_coords.astype(np.float64)
        ret, rotvec, transvec = cv2.solvePnP(world_coords,new_pixel_coords,camMatrix,distCoeffs)
        rotmat,jac = cv2.Rodrigues(rotvec)
        print(rotmat,transvec)
        extmat = np.column_stack((rotmat,transvec))

        global solverot, solvetrans
        solverot = np.linalg.inv(rotmat)
        solvetrans = transvec

    
        ########################
        #RGB to Depth transform#
        ########################
        rgb_coords = self.kinect.rgb_click_points
        rgb_coords = rgb_coords[0:3,:]
        depth_coords = self.kinect.depth_click_points
        depth_coords = depth_coords[0:3,:]
        

        # print(rgb_coords,depth_coords)
        # rgb_coords = rgb_coords.astype('float32')
        # depth_coords = depth_coords.astype('float32')

        
        # A=np.array([[rgb_coords[0][0], rgb_coords[0][1], 0, 0],
        #             [0, 0,  rgb_coords[0][0], rgb_coords[0][1]],
        #             [rgb_coords[1][0], rgb_coords[1][1], 0, 0],
        #             [0, 0, rgb_coords[1][0], rgb_coords[1][1]],
        #             [rgb_coords[2][0], rgb_coords[2][1], 0, 0],
        #             [0, 0,rgb_coords[2][0], rgb_coords[2][1]],
        #             [rgb_coords[3][0], rgb_coords[3][1], 0, 0],
        #             [0, 0, rgb_coords[3][0], rgb_coords[3][1]]])

        # b=np.array([depth_coords[0][0], depth_coords[0][1], depth_coords[1][0], depth_coords[1][1],depth_coords[2][0],depth_coords[2][1], depth_coords[3][0],depth_coords[3][1]])

        # x=(np.linalg.inv(A.T.dot(A))).dot(A.T).dot(b.T)
        # x_matrix=np.reshape(x,(2,2))

        global affine_rgb2depth, affine_depth2rgb

        rgb_coords = rgb_coords.astype('float32')
        depth_coords = depth_coords.astype('float32')

        affine_rgb2depth = cv2.getAffineTransform(rgb_coords, depth_coords) #changed to affine from perspective
        
        affine_depth2rgb = cv2.getAffineTransform(depth_coords, rgb_coords)
        
        

       





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
        #To record the location of pixel center
        global pixel_center
        pixel_center=np.array([pixel_coords[4][0],pixel_coords[4][1]]) 
        print (pixel_center)

        global calibration_done


        ###############################################################################

        # Depth Camera Affine Matrix Calculation

        # depth_coords=self.kinect.depth_click_points

        # for i in range(4):
        #     for j in range (2):
        #         depth_coords[i][j]=(-1)**j*(depth_coords[i][j]-depth_coords[4][j])

        # A=np.array([[depth_coords[0][0], depth_coords[0][1], 1, 0, 0, 0],
        #                    [0, 0, 0, depth_coords[0][0], depth_coords[0][1], 1],
        #                    [depth_coords[1][0], depth_coords[1][1], 1, 0, 0, 0],
        #                    [0, 0, 0, depth_coords[1][0], depth_coords[1][1], 1],
        #                    [depth_coords[2][0], depth_coords[2][1], 1, 0, 0, 0],
        #                    [0, 0, 0, depth_coords[2][0], depth_coords[2][1], 1],
        #                    [depth_coords[3][0], depth_coords[3][1], 1, 0, 0, 0],
        #                    [0, 0, 0, depth_coords[3][0], depth_coords[3][1], 1]])

        # b=np.array([-29.5, -30.48, -29.5, 30.48,29.5,30.48, 29.5,-30.48])

        # x=(np.linalg.inv(A.T.dot(A))).dot(A.T).dot(b.T)
        # x_matrix=np.reshape(x,(2,3))

        # global affine_matrix_depth
        # affine_matrix_depth=np.vstack((x_matrix,[0,0,1]))

        # # If calibration has been done set value to True

        calibration_done=True

        self.next_state="idle"


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



        # print(cv2.findHomography(pixel_coords,world_coords))


        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)
 
    def click_and_grab(self):
        if(self.kinect.new_click == True):
            # Checking if a click has been made
            # Finds the x and y location in RGB image of the clicked point
            x=self.kinect.last_click[0]
            y=self.kinect.last_click[1]

        # Need to convert this rgb points to world coordinate

        pix_center=self.pixel_center_loc()

        # X and Y locations in the RGB space in pixels. Finding the location of these RGB points in world space
        x=x-pix_center.item(0)
        y=pix_center.item(1)-y

        # Preparing pixel matrix for transform
        pixel_value=np.array([x,y])
        pixel_value=np.transpose(pixel_value)

        # Extracting affine transform between rgb and depth
        affine_rgb2depth=self.return_rgb2depth()

        # Extrinsic Matrix
        affine=self.return_affine()
        affine=affine[0:2,0:2]

        world_value=np.matmul(affine,pixel_value)

        if(self.kinect.currentDepthFrame.any() != 0):
            rgb_pt = np.array([[pixel_value[0],pixel_value[1],1]])
            depth_value=np.matmul(affine_rgb2depth,rgb_pt.T)
            z = self.kinect.currentDepthFrame[int(depth_value.item(1))][int(depth_value.item(0))]
            Z = 12.36 * np.tan(float(z)/2842.5 + 1.1863)
            Z=100

        

        phi=0
        pose=[world_value[0],world_value[1],Z,0]

        print ("X, Y and Z values of the point are noted",pose)

        execute_states = kine.IK(pose)
        for i, wp in enumerate(execute_states):
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
        self.kinect.new_click = 0




    def execute(self):
        '''
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
        '''
        self.click_and_grab()
        self.rexarm.get_feedback()
        self.next_state = "idle"


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


    def FK_check(self):
        self.status_message = "Checking Forward Kinematics"
        self.current_state = "FK_check"
        kine.FK_dh(self.rexarm.get_positions())
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        self.next_state="idle"