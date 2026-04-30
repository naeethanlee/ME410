import matplotlib.pyplot as plt
import sys

# Usage:
#   On the Pi, redirect output to a file:  ./week2 > imu_data.txt
#   Then copy imu_data.txt to your PC and run:  python plot_imu.py imu_data.txt
#   Or pipe live:  ./week2 | python plot_imu.py

# python3 plot_imu.py data.txt

data = []

filename = sys.argv[1] if len(sys.argv) > 1 else None
source = open(filename) if filename else sys.stdin

try:
    for line in source:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 7:
            continue
        try:
            data.append([float(x) for x in parts])
        except ValueError:
            continue
finally:
    if filename:
        source.close()

if not data:
    print("No data found. Make sure the file has 7 columns per line.")
    sys.exit(1)

t           = [row[0] for row in data]
roll_filt   = [row[1] for row in data]
roll_accel  = [row[2] for row in data]
roll_gyro   = [row[3] for row in data]
pitch_filt  = [row[4] for row in data]
pitch_accel = [row[5] for row in data]
seven = [row[6] for row in data]


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

# ax1.plot(t, roll_filt,  label='Motor 1', linewidth=1)
# ax1.plot(t, roll_accel, label='Motor 2',   linewidth=1, linestyle='--')
# ax1.plot(t, roll_gyro,  label='Motor 3',      linewidth=1, linestyle=':')
# ax1.plot(t, pitch_filt,  label='Motor 4', linewidth=1)
ax1.plot(t, pitch_accel,  label='Filtered Pitch x 100', linewidth=1)
ax1.plot(t, seven,  label='Desired Pitch x 100', linewidth=1)
ax1.set_ylabel('Roll Angle (deg)')
ax1.set_title('Roll')
ax1.legend()
ax1.grid(True)
ax1.axhline(0, color='black', linewidth=0.5)


# ax2.plot(t, pitch_gyro,  label='Gyro Integrated',      linewidth=1, linestyle=':')
ax2.set_ylabel('Pitch Angle (deg)')
ax2.set_xlabel('Time (s)')
ax2.set_title('Pitch')
ax2.legend()
ax2.grid(True)
ax2.axhline(0, color='black', linewidth=0.5)

plt.tight_layout()
plt.savefig('imu_plot.png', dpi=150)
plt.show()
print("Plot saved to imu_plot.png")
