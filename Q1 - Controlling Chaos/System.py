import numpy as np 


class MotorSystem():
    def __init__(self,f_c = 0.69,b=0.67,E_max = 5.0, noise_std = 0.013):
        
        #Initializing variables

        self.f_c = f_c
        self.b = b
        self.E_max = E_max
        self.noise_std = noise_std
        
        # Initializing System variables
        self.x = 0.0
        self.v = 0.0 

    # Reseting the System 
    def reset(self,x,v):
        self.x = x 
        self.v = v 

    # To get the Saturated Value of E incase the Throttle is too much
    def saturation_E (self,E):
        return np.clip(E,-self.E_max,self.E_max)

    def step(self,E,dt):
        self.E = self.saturation_E(E)
        self.noise = np.random.randn()*self.noise_std
        self.x_dot = self.v
        self.v_dot = self.E - self.b*self.v -self.f_c*np.tanh(self.v)+self.noise

        self.x+=self.x_dot*dt
        self.v+=self.v_dot*dt
        return self.x , self.v   


        
