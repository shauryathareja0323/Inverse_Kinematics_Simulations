import math
import matplotlib.pyplot as plt
import numpy as np

# Link lengths
l1 = 200  # thigh
l2 = 200  # shin

plt.ion()
fig, ax = plt.subplots(figsize=(6, 6))

y_values = np.linspace(-250, 250, 100)  # up-down motion
y_values = np.concatenate((y_values, y_values[::-1]))  # back and forth

x_fixed = 100  # fixed horizontal distance from hip

while True:
    for y in y_values:
        # Distance from hip to foot
        a = math.sqrt(x_fixed**2 + y**2)

        # Check reachability
        if a > (l1 + l2) or a < abs(l1 - l2):
            continue  # skip unreachable points

        # Hip angle using inverse kinematics
        a1_offset = math.atan2(y, x_fixed)
        a1_inner = math.acos((l1**2 + a**2 - l2**2) / (2 * l1 * a))
        a1 = a1_offset + a1_inner

        # Knee angle
        a2 = math.acos((l1**2 + l2**2 - a**2) / (2 * l1 * l2))

        # Joint coordinates
        hip = (0, 0)
        knee = (l1 * math.cos(a1), l1 * math.sin(a1))
        foot = (x_fixed, y)

        # Plot
        ax.clear()
        ax.plot([hip[0], knee[0]], [hip[1], knee[1]], 'bo-', linewidth=4, markersize=10, label="Thigh")
        ax.plot([knee[0], foot[0]], [knee[1], foot[1]], 'ro-', linewidth=4, markersize=10, label="Shin")
        ax.plot(foot[0], foot[1], 'go', markersize=10, label="Foot")
        ax.set_xlim(-l1-l2, l1+l2)
        ax.set_ylim(-l1-l2, l1+l2)
        ax.set_aspect('equal', adjustable='box')
        ax.set_title(f"Hip: {math.degrees(a1):.1f}°, Knee: {math.degrees(a2):.1f}°, Foot Y: {y:.2f}")
        ax.grid(True)
        ax.legend()

        plt.draw()
        plt.pause(0.02)
