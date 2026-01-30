import numpy as np 
import matplotlib.pyplot as plt 
from System import MotorSystem
from PID import PID


# Simulation Parameters (Do not change)
dt = 0.01
T = 50.0 
steps = int(T/dt)

position_choices = [-4, -2, 0, 2, 5] #desired Positions, if you are stuck check line 57

Switching_interval = 10.0 # Switching Time 
switching_steps = int(Switching_interval/dt)

"""
Give code for initializing System and PID controller
"""
system = MotorSystem()

pid = PID(kp=10.0, ki=8.0, kd=1.0)

# Data Storage
time = np.zeros(steps)
x_log = np.zeros(steps)
x_des_log = np.zeros(steps)

x_des = np.random.choice(position_choices) # For selecting Intiail position 

for i in range(steps):

    time[i] = i*dt
    """
    Write each iteration for the control loop
    """
    if i%switching_steps==0:
        x_des = np.random.choice(position_choices) ## For changing the desired position at each switching interval
        pid.reset()  ## Reset PID controller to avoid integral collection from previous setpoint
    x = system.x
    e = x_des - x
    u = pid.update(dt, e)
    system.step(u, dt)

    #For Logs 
    x_log[i] = x
    x_des_log[i] = x_des

# Write code to plot the data, Use matplotlib to plot graphs, Giving you a code for reference

fig, ax = plt.subplots(2,1, figsize=(10, 10))

## plot img
ax[0].set_xlim(0, T)
ax[0].set_ylim(-6, 6)
actual_line, = ax[0].plot(time, x_log, label="Actual Position", lw=2)
desired_line, = ax[0].plot(time, x_des_log, "--", label="Desired Position", lw=2)

ax[0].set_xlabel("Time (s)")
ax[0].set_ylabel("Position")
ax[0].set_title("Question 1: Direct Position PID")
ax[0].legend()
ax[0].grid()

## plot animation
ax[1].set_xlim(0, T)
ax[1].set_ylim(-6, 6)

actual_line, = ax[1].plot(time, x_log, label="Actual Position", lw=2)
desired_line, = ax[1].plot(time, x_des_log, "--", label="Desired Position", lw=2)
ax[1].set_xlabel("Time (s)")
ax[1].set_ylabel("Position")
ax[1].set_title("Question 1: Direct Position PID (Animated)")
ax[1].legend()
ax[1].grid()

def init():
    actual_line.set_data([], []) ## Initialising empty data for actual line
    desired_line.set_data([], []) ## Initialising empty data for desired line
    return actual_line, desired_line
def update(frame):
    actual_line.set_data(time[:frame], x_log[:frame]) ## Updating actual line data
    desired_line.set_data(time[:frame], x_des_log[:frame]) ## Updating desired line data
    return actual_line, desired_line
from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(time), init_func=init, blit=True, interval=dt*100)
update(len(time)-1) ## To show the final frame
plt.show()



# For bonus question try to animate the plot 
# If you are Stuck with the question try tuning with one desired set point rather than taking from the list, so its a bit easy to tune
