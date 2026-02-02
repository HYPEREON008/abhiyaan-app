import csv
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

encoder_log_file = 'Q4 - Are We There Yet_\Encoder_log.csv'
QEI_readings_file = 'Q4 - Are We There Yet_\QEI_readings.csv'

QEI_state_map = {
    (0, 0): 0, ## means both A and B are low
    (0, 1): 1, ## means A is low, B is high
    (1, 1): 2, ## means both A and B are high
    (1, 0): 3, ## means A is high, B is low
}

## this qei state map is the order of states when rotating forward
## so when QEI state moves from 0 -> 1 -> 2 -> 3 -> 0, each step is +1 count
## and when it moves from 0 -> 3 -> 2 -> 1 -> 0, each step is -1 count

PPR = 500 # Pulses Per Revolution
counts_per_revolution = PPR * 4 # 4X decoding
time_windows = [10, 20, 50, 100, 200, 500] # in milliseconds

QEI_data = pd.read_csv(QEI_readings_file)
encoder_data = pd.read_csv(encoder_log_file)

## for a time window dt and count difference dc, speed in RPM is:
def rpm(dt_ms, dc):
    if dt_ms == 0:
        return 0
    return (dc / counts_per_revolution) * (60000 / dt_ms)

## QEI readings data encoder count calculation
QEI_data['encoder_count'] = 0 ## initialize encoder count column
prev_state = QEI_state_map[(QEI_data.loc[0, 'A'], QEI_data.loc[0, 'B'])]
for i in range(1, len(QEI_data)):
    A = QEI_data.loc[i, 'A']
    B = QEI_data.loc[i, 'B']
    curr_state = QEI_state_map[(A, B)]

    diff = curr_state - prev_state
    if diff == 1 or diff == -3:
        QEI_data.loc[i, 'encoder_count'] = QEI_data.loc[i-1, 'encoder_count'] + 1
    elif diff == -1 or diff == 3:
        QEI_data.loc[i, 'encoder_count'] = QEI_data.loc[i-1, 'encoder_count'] - 1
    else: ## no change in state or invalid state change
        QEI_data.loc[i, 'encoder_count'] = QEI_data.loc[i-1, 'encoder_count']
    prev_state = curr_state

## Window wise rpm calculation for QEI data
for window in time_windows:
    rpm_col_name = f"rpm_{window}ms"
    QEI_data[rpm_col_name] = 0 ## initialize rpm column

    prev_count = 0

    for i in range(0, len(QEI_data), window): ## each QEI reading is 1ms apart
        dt = window ## time difference in ms
        dc = QEI_data.loc[i, 'encoder_count'] - prev_count ## count difference
        QEI_data.loc[i:i+window-1, rpm_col_name] = rpm(dt, dc)
        prev_count = QEI_data.loc[i, 'encoder_count']


## encoder log data RPM calculation
encoder_data['rpm'] = 0 ## initialize rpm column
for i in range(1,len(encoder_data)):
    dt = encoder_data.loc[i, 'time_ms'] - encoder_data.loc[i-1, 'time_ms'] ## time difference in ms == 1
    dc = encoder_data.loc[i, 'encoder_count'] - encoder_data.loc[i-1, 'encoder_count'] ## count difference
    encoder_data.loc[i, 'rpm'] = rpm(dt, dc)


## plotting
## first plot encoder log rpm vs time
plt.figure(figsize=(24, 6))
plt.subplot(1, 2, 1)
plt.plot(encoder_data['time_ms'], encoder_data['rpm'], label='Encoder Log RPM 50ms', color='blue')
plt.xlabel('Time (ms)')
plt.ylabel('RPM')
plt.title('Encoder Log RPM vs Time')


## then plot QEI rpm for different time windows
plt.subplot(1, 2, 2)
for window in time_windows:
    rpm_col_name = f"rpm_{window}ms"
    plt.plot(QEI_data['time_ms'], QEI_data[rpm_col_name], label=f'QEI RPM {window}ms', alpha=0.7)


plt.xlabel('Time (ms)')
plt.ylabel('RPM')
plt.title('QEI RPM vs Time')
plt.legend()
plt.show()
