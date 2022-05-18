import numpy as np

class PIDController:
    def __init__ (self, Kp, Ki, Kd, tau, limMin, limMax, T, integrator, differentiator):
        #Coefficient constants
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        #Time constant for derivation
        self.tau = tau
        
        #Limits for the output
        self.limMin = limMin
        self.limMax = limMax
        
        #Sample time in seconds
        self.T = T
        
        #Controller memory
        self.integrator = integrator
        self.prevError = np.zeros(20)
        self.differentiator = differentiator
        self.prevMeasurement = 0
        
        #Controller output
        self.out = 0.0
        
    def Clear(self):
        self.integrator = 0.0
        self.prevError = 0.0
        self.differentiator = 0.0
        self.prevMeasuement = 0.0
                
        self.out = 0.0
        
    def Update(self, setpoint, measurement):
        
        #Error signal
        error = setpoint - measurement
        
        #Proportional term
        proportional = self.Kp * error
        
        #Integral calc
        self.integrator = self.Ki * np.sum(self.prevError) / len(self.prevError)
                
        #Limit integrator swings
        self.integratorSwingLimit(proportional)
        
        #Derivative calc
        self.differentiator = measurement - self.prevMeasurement
        
        #Get output and limit it
        self.out = proportional + self.integrator + self.differentiator
        
        if (self.out > self.limMax):
            self.out = self.limMax
        elif (self.out < self.limMin):
            self.out = self.limMin
            
        #Store values for next iteration
        self.prevError = np.append(self.prevError, error)
        self.prevError = self.prevError[1:]
        self.prevMeasurement = measurement
        
        #Give output
        return self.out
        
    def integratorSwingLimit(self, proportional):
        #Vars for determining need of limiting
        limMaxInt = 0.0
        limMinInt = 0.0
        
        if (self.limMax > proportional):
            limMaxInt = self.limMax - proportional
            
        if (self.limMin < proportional):
            limMinInt = self.limMin - proportional
        
        #Limit swings
        if (self.integrator > limMaxInt):
            self.integrator = limMaxInt
        elif (self.integrator < limMinInt):
            self.integrator = limMinInt