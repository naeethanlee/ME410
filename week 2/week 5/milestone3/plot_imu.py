import matplotlib.pyplot as plt
import sys

# python3 plot_imu.py data.txt
# columns: time m0 m1 m2 m3 pitch pitch_desired roll roll_desired thrust

data = []

filename = sys.argv[1] if len(sys.argv) > 1 else None
source = open(filename) if filename else sys.stdin

try:
    for line in source:
        line = line.strip()
        if not line:
            continue
        parts = line.split()
        if len(parts) != 10:
            continue
        try:
            data.append([float(x) for x in parts])
        except ValueError:
            continue
finally:
    if filename:
        source.close()

if not data:
    print("No data found. Make sure the file has 10 columns per line.")
    sys.exit(1)

t             = [row[0] for row in data]
m0            = [row[1] for row in data]
m1            = [row[2] for row in data]
m2            = [row[3] for row in data]
m3            = [row[4] for row in data]
pitch         = [row[5] for row in data]
pitch_desired = [row[6] for row in data]
roll          = [row[7] for row in data]
roll_desired  = [row[8] for row in data]
thrust        = [row[9] for row in data]

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

ax1.plot(t, m0, label='M0 front-left',  linewidth=1)
ax1.plot(t, m1, label='M1 back-left',   linewidth=1)
ax1.plot(t, m2, label='M2 front-right', linewidth=1)
ax1.plot(t, m3, label='M3 back-right',  linewidth=1)
ax1.plot(t, thrust, label='Thrust', linewidth=1, linestyle='--', color='black')
ax1.set_ylabel('PWM')
ax1.set_title('Motor Commands + Thrust')
ax1.legend()
ax1.grid(True)

ax2.plot(t, pitch,         label='Pitch (filtered)', linewidth=1)
ax2.plot(t, pitch_desired, label='Pitch desired',    linewidth=1, linestyle='--')
ax2.set_ylabel('Degrees')
ax2.set_title('Pitch')
ax2.legend()
ax2.grid(True)
ax2.axhline(0, color='black', linewidth=0.5)

ax3.plot(t, roll,         label='Roll (filtered)', linewidth=1)
ax3.plot(t, roll_desired, label='Roll desired',    linewidth=1, linestyle='--')
ax3.set_ylabel('Degrees')
ax3.set_xlabel('Time (s)')
ax3.set_title('Roll')
ax3.legend()
ax3.grid(True)
ax3.axhline(0, color='black', linewidth=0.5)

plt.tight_layout()
plt.savefig('imu_plot.png', dpi=150)
plt.show()
print("Plot saved to imu_plot.png")
