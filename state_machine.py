import time
import numpy as np
import csv 
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
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