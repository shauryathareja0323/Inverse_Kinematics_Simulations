import math
import matplotlib.pyplot as plt

# Fixed lengths
l1 = 200  # thigh length
l2 = 200  # shin length

while True:
    try:
        a = float(input("Enter vertical drop of foot from hip (positive value), or -1 to exit: "))
        if a < 0:
            break
        # Set target position in 4th quadrant
        y_foot = -a
        x_foot = 150  # forward position from hip (adjust as needed)

        # Distance from hip to foot
        dist = math.sqrt(x_foot**2 + y_foot**2)

        # Check reachability
        if dist > (l1 + l2) or dist < abs(l1 - l2):
            print("❌ Target unreachable with given link lengths!")
            continue

        # Hip angle
        a1_offset = math.atan2(y_foot, x_foot)
        a1_inner = math.acos((l1**2 + dist**2 - l2**2) / (2 * l1 * dist))
        a1 = a1_offset + a1_inner

        # Knee angle
        a2 = math.acos((l1**2 + l2**2 - dist**2) / (2 * l1 * l2))

        # Joint coordinates
        hip = (0, 0)
        knee = (l1 * math.cos(a1), l1 * math.sin(a1))
        foot = (x_foot, y_foot)

        # Plot
        plt.figure(figsize=(6, 6))
        plt.plot([hip[0], knee[0]], [hip[1], knee[1]], 'bo-', linewidth=4, markersize=10, label="Thigh")
        plt.plot([knee[0], foot[0]], [knee[1], foot[1]], 'ro-', linewidth=4, markersize=10, label="Shin")
        plt.plot(foot[0], foot[1], 'go', markersize=10, label="Foot")
        plt.axhline(0, color='black', linewidth=1)  # Ground line
        plt.xlim(-l1-l2, l1+l2)
        plt.ylim(-l1-l2, l1+l2)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.title(f"Hip: {math.degrees(a1):.1f}°, Knee: {math.degrees(a2):.1f}°")
        plt.legend()
        plt.grid(True)
        plt.show()

    except ValueError:
        print("❌ Please enter a valid number.")
