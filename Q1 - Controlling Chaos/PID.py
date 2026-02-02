class PID:
    def __init__(self,kp=1.0,ki=1.0,kd=1.0):
        self.kp =kp
        self.ki = ki
        self.kd = kd 

        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0 

    def update(self,dt,error):
        
        self.integral = self.integral + error * dt
        self.derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return self.kp*error +self.ki*self.integral +self.kd*self.derivative
