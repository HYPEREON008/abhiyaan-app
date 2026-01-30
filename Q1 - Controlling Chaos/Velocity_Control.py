import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from System import MotorSystem
from PID import PID


# Simulation Parameters (Do not change)
dt = 0.01
T = 50.0 
steps = int(T/dt)

velocity_choices = [-1, -5, 3, 2, 6] #desired Velocity, if you are stuck check line 57

Switching_interval = 10 # Switching Time 
switching_steps = int(Switching_interval/dt)

"""
Give code for initializing System and PID controller
"""
# Initialize Motor System
motor = MotorSystem()
# Initialize PID Controller with some params
pid = PID(kp=20.0, ki=2.0, kd=0.0)
# Data Storage


time = np.zeros(steps)
v_log = np.zeros(steps)
v_des_log = np.zeros(steps)

v_des = np.random.choice(velocity_choices) # For selecting Intiail position 

for i in range(steps):

    time[i] = i*dt
    """
    Write each iteration for the control loop
    """
    if i % switching_steps == 0:
        v_des = np.random.choice(velocity_choices)  # For changing desired velocity at each switching interval

        pid.reset()  # Reset PID controller to avoid integral collection from previous setpoint
    
    v = motor.v
    e = v_des - v
    u = pid.update(dt, e)
    motor.step(u, dt)

    v_log[i] = motor.v
    v_des_log[i] = v_des
    # ##For Logs 
    # x_log[i] = x
    # x_des_log[i] = x_des

# Write code to plot the data, Use matplotlib to plot graphs, Giving you a code for reference

fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, T)
ax.set_ylim(-6, 6)

## Initializing the lines for actual and desired velocity
actual_line, = ax.plot([], [], label="Actual Velocity", lw=2)
desired_line, = ax.plot([], [], "--", label="Desired Velocity", lw=2)

ax.set_xlabel("Time (s)")
ax.set_ylabel("Velocity")
ax.set_title("Velocity Control using PID (Animated)")
ax.legend()
ax.grid()

def init():
    actual_line.set_data([], []) ## Initialising empty data for actual line
    desired_line.set_data([], []) ## Initialising empty data for desired line
    return actual_line, desired_line
def update(frame):
    actual_line.set_data(time[:frame], v_log[:frame]) ## Updating actual line data
    desired_line.set_data(time[:frame], v_des_log[:frame]) ## Updating desired line data
    return actual_line, desired_line
ani = FuncAnimation(fig, update, frames=len(time), init_func=init, blit=True, interval=dt*10)
## blit true for better performance;
update(len(time)-1) ## To show the final frame

plt.show()

# For bonus question try to animate the plot 
# If you are Stuck with the question try tuning with one desired set point rather than taking from the list, so its a bit easy to tune
