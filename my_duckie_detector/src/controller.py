import math
import numpy as np


class Controller():

    def __init__(self, KP, KI, KD, upper_integral_cutoff, lower_integral_cutoff):
        
        self.P_gain = KP
        self.I_gain = KI
        self.D_gain = KD
        self.integral = 0
        self.last_error = 0
        self.u_int_cut = upper_integral_cutoff
        self.l_int_cut = lower_integral_cutoff

        

    def updateParams(self, actual, desired, dt):
        
        # The error 
        error = desired - actual

        # The Proportional term, how fast will it reach setpoint
        proportional_term = error * self.P_gain

        # The Integral term, use past values to reduce steady state error
        self.integral += error * dt

        if self.integral > self.u_int_cut:
            self.integral = self.u_int_cut
        if self.integral < self.l_int_cut:
            self.integral = self.l_int_cut

        integral_term = self.I_gain * self.integral

        # The Deriviative term, value prediction to reduce overshoot
        derivative_temp = (error - self.last_error)/dt
        derivative_term = self.D_gain * derivative_temp

        # Process output command, 
        command = proportional_term + integral_term + derivative_term 

        # Update last_error 
        self.last_error = error

        # Return the data
        return command

        



