import numpy as np 
import matplotlib.pyplot as plt 
from System import MotorSystem
from PID import PID


"""
In this loop we are doing velocity control in the inner loop and 
position control in outer loop.
"""

# Simulation Parameters (Do not Change)

dt = 0.01
T = 50.0 
steps = int(T/dt)

x_desired_choices = [-4,-2,4,5,1,3,6]

switching_interval = 10
switching_steps = int(switching_interval/dt)

"""
Initialise System and PID
"""
# Initialize Motor System
motor = MotorSystem()
# Initialize Outer Loop PID Controller for Position
pid_x = PID(kp=10.0, ki=1.5, kd=2.0)
# Initialize Inner Loop PID Controller for Velocity
pid_v = PID(kp=3.0, ki=0.1, kd=0.1)

# For logging purposes

time = np.zeros(steps)
x_log = np.zeros(steps)
x_des_log = np.zeros(steps)
v_log = np.zeros(steps)
v_ref_log = np.zeros(steps)
E_log = np.zeros(steps)

# Initial desired position
x_des = np.random.choice(x_desired_choices)

for i in range(steps):
    time[i] = i * dt

    
    if i % switching_steps == 0:
        x_des = np.random.choice(x_desired_choices)
        pid_x.reset()

    """
    Write Outer Loop first then the inner Loop
    """
    # Outer Loop - Position Control
    x = motor.x
    e_x = x_des - x
    v_ref = np.clip(pid_x.update(dt, e_x), -100,100) ## Limiting the velocity reference to avoid too high values
    # Inner Loop - Velocity Control
    v = motor.v
    e_v = v_ref - v
    E = pid_v.update(dt, e_v)
    motor.step(E, dt)

    # Data Logging (Comments are for reference )

    x_log[i] = x
    x_des_log[i] = x_des
    v_log[i] = v
    v_ref_log[i] = v_ref
    E_log[i] = E

# Plotting Part 

fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

# Position
axs[0].plot(time, x_log, label="Actual Position")
axs[0].plot(time, x_des_log, "--", label="Desired Position")
axs[0].set_ylabel("Position")
axs[0].legend()
axs[0].grid()

# Velocity
axs[1].plot(time, v_log, label="Actual Velocity")
axs[1].plot(time, v_ref_log, "--", label="Velocity Reference")
axs[1].set_ylabel("Velocity")
axs[1].legend()
axs[1].grid()

# Effort
axs[2].plot(time, E_log, label="Effort")
axs[2].set_ylabel("Effort")
axs[2].set_xlabel("Time (s)")
axs[2].legend()
axs[2].grid()

plt.show()
