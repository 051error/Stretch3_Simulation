import numpy as np
import math

class LiftPID:
    def __init__(self, Kp=300.0, Ki=50.0, Kd=20.0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        
    def compute(self, z_desired, z_current, zd_current, dt):
        z_desired = float(z_desired)
        z_current = float(z_current)
        zd_current = float(zd_current)
        dt = float(dt)
        error = z_desired - z_current
        P = self.Kp * error
        if abs(error) < 0.1:  
            self.integral += error * dt
            self.integral = np.clip(self.integral, -2.0, 2.0)
        I = self.Ki * self.integral
        D = -self.Kd * zd_current
        force = P + I + D
        return float(force)
    
    def reset(self):
        self.integral = 0.0

class ArmExtendPID:
    def __init__(self, Kp=500.0, Ki=5.0, Kd=10.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        
    def compute(self, x_desired, x_current, xd_current, dt):
        error = x_desired - x_current
        P = self.Kp * error
        err=sum(error)
        if abs(err) < 0.4: 
            self.integral += error * dt
            self.integral = np.clip(self.integral, -1.0, 1.0)
        I = self.Ki * self.integral
        D = -self.Kd * xd_current
        tau = P + I + D
        som=sum(tau)
        tau = np.clip(som, -56.0, 56.0)
        return tau
    
    def reset(self):
        self.integral = 0.0

class WristYawPID:
    def __init__(self, Kp=50.0, Ki=10.0, Kd=8.0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        
    def compute(self, target, current, velocity, dt):

        target = float(target)
        current = float(current)
        velocity = float(velocity)
        dt = float(dt)
        error = target - current

        error = np.arctan2(np.sin(error), np.cos(error))
        P = self.Kp * error
        if abs(error) < 0.1: 
            self.integral += error * dt
            self.integral = np.clip(self.integral, -0.5, 0.5)
        I = self.Ki * self.integral
        D = -self.Kd * velocity
        tau = P + I + D
        tau = float(np.clip(tau, -5.0, 5.0))
        
        return tau
    
    def reset(self):
        self.integral = 0.0
