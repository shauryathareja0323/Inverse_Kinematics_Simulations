import numpy as np
import matplotlib.pyplot as plt


L1 = 20.0  # cm
L2 = 20.0  # cm
hip_pos = np.array([0, 0])


ENC_STEP_DEG = 36.0  # Gear Reduction of 10:1

HIP_MIN = -60.0   # angles relative to zero position
HIP_MAX = 60.0
KNEE_MIN = -91.0
KNEE_MAX = 91.0

def inverse_kinematics(x, y):
    d = np.sqrt(x**2 + y**2)
    if d > (L1 + L2) or d < abs(L1 - L2):
        return None

    cos_knee = (L1**2 + L2**2 - d**2) / (2 * L1 * L2)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle_rad = np.pi - np.arccos(cos_knee)
    knee_angle_deg = np.degrees(knee_angle_rad)

    cos_hip_offset = (d**2 + L1**2 - L2**2) / (2 * L1 * d)
    cos_hip_offset = np.clip(cos_hip_offset, -1.0, 1.0)
    hip_offset = np.arccos(cos_hip_offset)
    base_angle = np.arctan2(y, x)
    hip_angle_abs = base_angle - hip_offset
    hip_angle_deg = np.degrees(hip_angle_abs)

    hip_neutral = -90.0
    hip_angle_rel = hip_angle_deg - hip_neutral
    knee_angle_rel = knee_angle_deg

    if not (HIP_MIN <= hip_angle_rel <= HIP_MAX):
        return None
    if not (KNEE_MIN <= knee_angle_rel <= KNEE_MAX):
        return None

    return hip_angle_rel, knee_angle_rel

def forward_kinematics(hip_angle_rel, knee_angle_rel):
    hip_neutral = -90.0
    hip_angle_abs = np.radians(hip_neutral + hip_angle_rel)
    knee_angle_abs = hip_angle_abs + np.radians(knee_angle_rel)

    knee_x = hip_pos[0] + L1 * np.cos(hip_angle_abs)
    knee_y = hip_pos[1] + L1 * np.sin(hip_angle_abs)
    knee_pos = np.array([knee_x, knee_y])

    foot_x = knee_pos[0] + L2 * np.cos(knee_angle_abs)
    foot_y = knee_pos[1] + L2 * np.sin(knee_angle_abs)
    foot_pos = np.array([foot_x, foot_y])

    return knee_pos, foot_pos

def plot_leg(hip_angle_rel, knee_angle_rel):
    knee_pos, foot_pos = forward_kinematics(hip_angle_rel, knee_angle_rel)
    plt.figure(figsize=(6, 6))
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
    plt.title(f'Hip: {hip_angle_rel:.2f}°, Knee: {knee_angle_rel:.2f}°')
    plt.show()


while True:
    user_input = input("\nEnter foot X,Y coordinates in cm (or 'q' to quit): ")
    if user_input.lower() == 'q':
        print("Exiting...")
        break
    try:
        x_str, y_str = user_input.split(',')
        x = float(x_str.strip())
        y = float(y_str.strip())

        result = inverse_kinematics(x, y)
        if result is None:
            print("No solution (unreachable point or violates joint limits).")
        else:
            hip_angle_rel, knee_angle_rel = result
            hip_enc = hip_angle_rel / ENC_STEP_DEG
            knee_enc = knee_angle_rel / ENC_STEP_DEG
            print(f"Hip Angle (rel): {hip_angle_rel:.2f}°  → Encoder: {hip_enc:.2f}")
            print(f"Knee Angle (rel): {knee_angle_rel:.2f}° → Encoder: {knee_enc:.2f}")
            plot_leg(hip_angle_rel, knee_angle_rel)
    except Exception as e:
        print("Invalid input. Please enter coordinates like: 0,-28.28")
