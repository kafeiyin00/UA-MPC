import matplotlib.pyplot as plt
import numpy as np

# Update font settings
plt.rcParams['font.family'] = ['DejaVu Serif', 'serif']
plt.rcParams['mathtext.fontset'] = 'dejavuserif'

plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42
# plt.rcParams['text.usetex'] = True

# Read data from file
data = np.loadtxt('/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/timeperformance.txt', delimiter=',')

# Extract columns
time = data[0:5000, 0] - data[0, 0]
control_time = data[0:5000, 1]
render_time = data[0:5000, 2]
lo_time = data[0:5000, 3]

# Calculate average times
avg_control_time = np.mean(control_time)
avg_render_time = np.mean(render_time)
avg_lo_time = np.mean(lo_time)

print(f'Average MPC Solving Time: {avg_control_time:.6f} s')
print(f'Average Depth Rendering Time: {avg_render_time:.6f} s')
print(f'Average I2EKF-LO Time: {avg_lo_time:.6f} s')

# Create scatter plot
fig, ax = plt.subplots()

ax.scatter(time, render_time, label='Uncertainty Prediction', alpha=0.3, s=3)
ax.scatter(time, control_time, label='MPC Solving', alpha=0.3, s=3)
ax.scatter(time, lo_time, label='I2EKF-LO', alpha=0.3, s=3)

# Add labels and title
ax.set_xlabel('Time (s)')
ax.set_ylabel('Processing Time (s)')
ax.set_title('Time Performance (s)')

# Update legend and tick labels font
ax.legend()
plt.xticks()
plt.yticks()

# Set y-axis limit
ax.set_ylim(top=0.2)
ax.set_ylim(bottom=0.0)
ax.set_xlim(left=0)
ax.set_xlim(right=500)

# Show plot
plt.show()
