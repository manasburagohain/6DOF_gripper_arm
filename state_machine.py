import time
import numpy as np
import cv2
import csv 
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""

affine_matrix_rgb=0
affine_rgb2depth=0
calibration_done=False
pixel_center=0
rgb2dep = 0


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
   
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""

        camMatrix = np.array([[562.04399308,0.00,327.78253347],[0.00,559.59009967,249.41949401],[0.00,0.00,1.00]])
        distCoeffs = np.array([2.86261843e-01,-1.06215288e+00,-6.38736068e-04,-6.91259011e-04,1.42539697e+00])

        # Hardcoding the edge coordinates of the board

        world_coords = np.array([[-11.61,-12.0,0.0],[-11.61,12.0,0.0],[11.61,12.0,0],[11.61,-12.0,0.0]]) # inch
        pixel_coords = self.kinect.rgb_click_points
        #new_pixel_coords = np.zeros(shape=(4,2),dtype=np.float32)
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


        rgb_coords = self.kinect.rgb_click_points
        rgb_coords = rgb_coords[0:4,:]
        depth_coords = self.kinect.depth_click_points
        depth_coords = depth_coords[0:4,:]



        # print(rgb_coords,depth_coords)
        # rgb_coords = rgb_coords.astype('float32')
        # depth_coords = depth_coords.astype('float32')

        
        A=np.array([[rgb_coords[0][0], rgb_coords[0][1], 0, 0],
                    [0, 0,  rgb_coords[0][0], rgb_coords[0][1]],
                    [rgb_coords[1][0], rgb_coords[1][1], 0, 0],
                    [0, 0, rgb_coords[1][0], rgb_coords[1][1]],
                    [rgb_coords[2][0], rgb_coords[2][1], 0, 0],
                    [0, 0,rgb_coords[2][0], rgb_coords[2][1]],
                    [pixel_coords[3][0], rgb_coords[3][1], 0, 0],
                    [0, 0, rgb_coords[3][0], rgb_coords[3][1]]])

        b=np.array([depth_coords[0][0], depth_coords[0][1], depth_coords[1][0], depth_coords[1][1],depth_coords[2][0],depth_coords[2][1], depth_coords[3][0],depth_coords[3][1]])

        x=(np.linalg.inv(A.T.dot(A))).dot(A.T).dot(b.T)
        x_matrix=np.reshape(x,(2,2))

        global affine_rgb2depth
        #affine_rgb2depth=x_matrix
        #print(affine_rgb2depth)  





        global rgb2dep

        rgb_coords = rgb_coords.astype('float32')
        depth_coords = depth_coords.astype('float32')

        affine_rgb2depth = cv2.getPerspectiveTransform(rgb_coords, depth_coords)
        #dep2rgt2 = cv2.estimateRigidTransform(depth_coords, rgb_coords, 1)
        #print(rgb2dep)
        #print(dep2rgt2)


       





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

    def return_correction_affine(self):
        return affine_rgb2depth

    def calibration_state(self):
        return calibration_done

    def pixel_center_loc(self):
        return pixel_center



        # print(cv2.findHomography(pixel_coords,world_coords))


        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)
    
    def execute(self) :
        execute_states= np.array([[ 0.0, 0.0, 0.0, 0.0, 0.0],
                                  [ 1.0, 0.8, 1.0, 0.5, 1.0],
                                  [-1.0,-0.8,-1.0,-0.5, -1.0],
                                  [-1.0, 0.8, 1.0, 0.5, 1.0],
                                  [1.0, -0.8,-1.0,-0.5, -1.0],
                                  [ 0.0, 0.0, 0.0, 0.0, 0.0]])
        execute_states.tolist()
        self.status_message = "State: Execute - Following Set Path"
        self.current_state = "execute"
        for i,_ in enumerate(execute_states) :
            self.rexarm.set_positions(execute_states[i])
            self.rexarm.pause(1)
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
        for i,_ in enumerate(exec_joints) :
            self.rexarm.set_positions(exec_joints[i])
            self.rexarm.pause(1)
        self.rexarm.get_feedback()
        self.next_state = "idle"