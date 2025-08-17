import numpy as np
import matplotlib.pyplot as plt

# Link lengths
L1 = 20.0  # hip to knee
L2 = 20.0  # knee to foot

hip_pos = np.array([0, 0])

# Angle constraints (degrees)
hip_min, hip_max = 180, 360
knee_min, knee_max = 0, 360  # Clockwise bent leg only

# Neutral walking posture
neutral_hip = 270
neutral_knee = 270

# Initial foot position
foot_x, foot_y = 0, -40  # starting position

# Track last chosen angles to keep continuity
last_hip = neutral_hip
last_knee = neutral_knee

def inverse_kinematics_all(x, y):
    d = np.sqrt(x**2 + y**2)
    if d > (L1 + L2) or d < abs(L1 - L2):
        return []

    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)

    knee_angle_down = np.pi - np.arccos(cos_knee)
    knee_angle_up   = np.pi + np.arccos(cos_knee)

    cos_hip_offset = (d**2 + L1**2 - L2**2) / (2 * L1 * d)
    cos_hip_offset = np.clip(cos_hip_offset, -1.0, 1.0)
    hip_offset = np.arccos(cos_hip_offset)
    base_angle = np.arctan2(y, x)

    hip_down = base_angle - hip_offset
    hip_up   = base_angle + hip_offset

    sols = [
        (np.degrees(hip_down) % 360, np.degrees(knee_angle_down) % 360),
        (np.degrees(hip_up) % 360, np.degrees(knee_angle_up) % 360)
    ]
    return sols

def is_within_constraints(hip, knee):
    return (hip_min <= hip <= hip_max) and (knee_min <= knee <= knee_max)

def choose_best_solution(solutions):
    global last_hip, last_knee
    valid = [(hip, knee) for hip, knee in solutions if is_within_constraints(hip, knee)]
    if not valid:
        return None
    # Prefer the solution closest to the last chosen one
    valid.sort(key=lambda s: (abs(s[0] - last_hip) + abs(s[1] - last_knee)))
    chosen = valid[0]
    last_hip, last_knee = chosen
    return chosen

def forward_kinematics(hip_angle_deg, knee_angle_deg):
    hip_angle = np.radians(hip_angle_deg)
    knee_angle = np.radians(knee_angle_deg)
    knee_x = hip_pos[0] + L1 * np.cos(hip_angle)
    knee_y = hip_pos[1] + L1 * np.sin(hip_angle)
    knee_pos = np.array([knee_x, knee_y])
    foot_x = knee_pos[0] + L2 * np.cos(hip_angle + knee_angle)
    foot_y = knee_pos[1] + L2 * np.sin(hip_angle + knee_angle)
    foot_pos = np.array([foot_x, foot_y])
    return knee_pos, foot_pos

def update_plot():
    global foot_x, foot_y
    plt.clf()
    all_solutions = inverse_kinematics_all(foot_x, foot_y)
    best = choose_best_solution(all_solutions)

    if best is None:
        plt.title(f"No solution in constraints for ({foot_x:.1f}, {foot_y:.1f})")
    else:
        hip_angle, knee_angle = best
        knee_pos, foot_pos = forward_kinematics(hip_angle, knee_angle)

        plt.axhline(0, color='gray', lw=0.5)
        plt.axvline(0, color='gray', lw=0.5)
        plt.plot([hip_pos[0], knee_pos[0]], [hip_pos[1], knee_pos[1]], 'o-', lw=3, label='Thigh')
        plt.plot([knee_pos[0], foot_pos[0]], [knee_pos[1], foot_pos[1]], 'o-', lw=3, label='Shank')
        plt.scatter(*hip_pos, c='red', s=100, label='Hip')
        plt.scatter(*knee_pos, c='blue', s=100, label='Knee')
        plt.scatter(*foot_pos, c='green', s=100, label='Foot')
        plt.xlim(-50, 50)
        plt.ylim(-50, 50)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        plt.legend()
        hip_graph = 180-hip_angle
        knee_graph = 180-knee_angle
        plt.title(f"Hip: {-hip_graph:.2f}°, Knee: {knee_graph:.2f}°\nFoot: ({foot_x:.1f}, {foot_y:.1f})")
        hip_exact = 360 - hip_angle - 90
        knee_exact_movement = knee_angle/36
        print(f"Hip: {hip_angle:.2f}°, Knee: {knee_angle:.2f}°")
        print(f"Hip Movement: {hip_exact/36}, Knee Movement:{knee_exact_movement:2f}")
        print(f"Hip Movement Degree: {hip_exact}, Knee Movement Degree:{knee_angle:2f}")


    plt.draw()

def on_key(event):
    global foot_x, foot_y
    step = 1.0
    if event.key == 'up':
        foot_y += step
    elif event.key == 'down':
        foot_y -= step
    elif event.key == 'left':
        foot_x -= step
    elif event.key == 'right':
        foot_x += step
    update_plot()

# Interactive mode
plt.ion()
fig = plt.figure(figsize=(6,6))
fig.canvas.mpl_connect('key_press_event', on_key)
update_plot()
plt.show(block=True)