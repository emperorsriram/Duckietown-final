#!/usr/bin/env python
import math
import time
import numpy as np
import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose, FSMState
import time
import numpy as np
from controller import Controller


class my_lane_controller():

    def __init__(self):
        self.velocity_to_m_per_s = 0.67
        self.omega_to_rad_per_s = 0.45 * 2 * math.pi
        self.omega_max = 50.0
        self.omega_min = -50.0

        #PID for CTE
        self.Kp_d = 6
        self.Ki_d = 0
        self.Kd_d = 0

        #PID for phi
        self.Kp_phi = 0.8
        self.Ki_phi = 0.1
        self.Kd_phi = 0

        #Useful Info
        self.last_dt = 0
        self.t_start = rospy.get_time()
        self.prev_pose_msg = LanePose()
        self.desired_d = 0
        self.desired_phi = 0
        self.in_lane_bool = False
        self.Lane_Control = False

        self.ct_integral_top_cutoff = 0.3
        self.ct_integral_bottom_cutoff = -0.3

        # Publication
	self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions
        self.sub_fsm_mode = rospy.Subscriber("/tyscrduckie/fsm_node/mode", FSMState, self.cbMode, queue_size=1)
        self.sub_lane_reading = rospy.Subscriber("/tyscrduckie/lane_filter_node/lane_pose", LanePose, self.PoseHandling)

        # The PID
        self.cte_control = Controller(self.Kp_d,self.Ki_d,self.Kd_d, self.ct_integral_top_cutoff, self.ct_integral_bottom_cutoff)
        self.phi_control = Controller(self.Kp_phi,self.Ki_phi,self.Kd_phi, self.ct_integral_top_cutoff,self.ct_integral_bottom_cutoff)

    # Used to return the state of the finite state machine, in our case, the lane following state is important
    def cbMode(self, fsm_state_msg):
        if fsm_state_msg.state == "LANE_FOLLOWING":
            self.Lane_Control = True
        else:
            self.Lane_Control = False


    
    def publishCmd(self, car_cmd_msg):
        self.pub_car_cmd.publish(car_cmd_msg)


    def sendStop(self):
        # Send stop command
        car_control_msg = Twist2DStamped()
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.publishCmd(car_control_msg)

    # The lane following operation 
    def PoseHandling(self,input_pose_msg):
        # Don't start if not in lane following
        if self.Lane_Control == True: 
            # Make sure you are in-lane
            if input_pose_msg.in_lane == True:
                print("my distance d is %f" % input_pose_msg.d)
                print("my direction phi is %f" % input_pose_msg.phi)
	        command_d = 0
                command_phi = 0
                command = 0
	        dt = 0

                # Configure dt
	        if self.last_dt == 0:
	            dt = rospy.get_time() - self.t_start
	        else:
	            dt = rospy.get_time() - self.last_dt

	        self.last_dt = dt
		    
                # Two controllers, one for d and another for phi. 
                # Capable of straight lane and turning lanes, finer tunes provide better output
	        command_d = self.cte_control.updateParams(input_pose_msg.d, self.desired_d, dt)
	        command_phi = self.phi_control.updateParams(input_pose_msg.phi, self.desired_phi, dt)
                command = command_d + command_phi
                 
	        print("my command is %f " % command)

                # Make sure omega isn't crazy high
                omega = command
	        if omega > self.omega_max: omega = self.omega_max
	        if omega < self.omega_min: omega = self.omega_min

	        car_control_msg = Twist2DStamped()

                # Limit the speed depending on whether the error is very high. If error is high, speed is 0
	        if command != 0:
                    car_control_msg.v = abs(0.10 / command)
                else:
                    car_control_msg.v = 0.10
	        print("my velocity is %f " % car_control_msg.v)

                # Handles close to zero values of command, don't want velocity too high.
                if car_control_msg.v > 0.10:
                    car_control_msg.v = 0.10
	            print("my velocity cut down to %f " % car_control_msg.v)
                if car_control_msg.v < 0.0:
                    car_control_msg.v = 0

                # apply magic conversion factors
	        car_control_msg.omega = omega * self.omega_to_rad_per_s
	        print("my omega is %f \n" % car_control_msg.omega)
	        self.publishCmd(car_control_msg)

	    else:
#	        print("Not in lane or bad messages")
                self.sendStop()
                
        else:
	    # print("Not in lane control")
            self.sendStop()
  

if __name__ == "__main__":

    rospy.init_node("my_lane_controller_node", anonymous=False)  # adapted to sonjas default file
    my_lane_control_node = my_lane_controller()
    rospy.spin()
