import numpy as np
from scipy.optimize import minimize

# ===== ADD (Serial PWM sending) =====
import serial
import time

SERIAL_PORT = "COM3"
BAUDRATE    = 115200

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"✅ Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
except Exception as e:
    print(f"⚠️ Serial NOT connected ({SERIAL_PORT}). PWM will only print.\nReason: {e}")

def send_pwm_command(cmd: str):
    if ser is None:
        return
    ser.write((cmd + "\r").encode())
# ===== END ADD =====


# =============================================================================
# ✅ GRIPPER SETTINGS (CALIBRATE THESE!)
# =============================================================================
GRIPPER_CH = 4

# These depend on your gripper servo + linkage:
GRIPPER_OPEN_PULSE   = 2000   # fully open (try 1800~2400)
GRIPPER_CLOSED_PULSE = 900    # fully closed (try 600~1200)

# At "fully open", what is the max object width your gripper can accept? (cm)
GRIPPER_MAX_WIDTH_CM = 6.0

GRIPPER_MOVE_TIME_MS = 800


def width_to_gripper_pwm(width_cm: float) -> int:
    """
    Map object width (cm) -> gripper PWM.
    width = GRIPPER_MAX_WIDTH_CM => fully open
    width = 0 => fully closed
    """
    w = float(np.clip(width_cm, 0.0, GRIPPER_MAX_WIDTH_CM))
    # ratio: 0 -> closed, 1 -> open
    ratio = w / GRIPPER_MAX_WIDTH_CM if GRIPPER_MAX_WIDTH_CM > 0 else 0.0
    pwm = GRIPPER_CLOSED_PULSE + ratio * (GRIPPER_OPEN_PULSE - GRIPPER_CLOSED_PULSE)
    return int(np.clip(pwm, min(GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE),
                            max(GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE)))


def set_gripper_pwm(pwm: int, t_ms: int = GRIPPER_MOVE_TIME_MS):
    cmd = f"#{GRIPPER_CH} P{pwm} T{t_ms}"
    print(f"🦾 Gripper cmd: {cmd}")
    send_pwm_command(cmd)
    time.sleep(t_ms / 1000.0 + 0.2)


# =============================================================================
# ✅ HOME POSITION SUPPORT (Home → Move → Pick → Home)
# =============================================================================
HOME_PULSES   = [1500, 1500, 1500, 1500]  # Servo 0..3
HOME_TIME_MS  = 2000

def go_home():
    """Move arm to HOME pulses + gripper fully OPEN."""
    cmd = (
        f"#0 P{HOME_PULSES[0]} "
        f"#1 P{HOME_PULSES[1]} "
        f"#2 P{HOME_PULSES[2]} "
        f"#3 P{HOME_PULSES[3]} "
        f"#{GRIPPER_CH} P{GRIPPER_OPEN_PULSE} "
        f"T{HOME_TIME_MS}"
    )
    print("\n🏠 Going HOME (gripper open):")
    print(f"   {cmd}")
    send_pwm_command(cmd)
    time.sleep(HOME_TIME_MS / 1000.0 + 0.3)


# =============================================================================
# ARM DIMENSIONS (AL5D Specifications)
# =============================================================================
BASE_HEIGHT = 6.7
SHOULDER_LENGTH = 14.605
ELBOW_LENGTH = 18.7325
WRIST_LENGTH = 8.5725


# =============================================================================
# TRANSFORMATION UTILITIES
# =============================================================================
def translate(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def rotate_z(theta):
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0:2, 0:2] = [[c, -s], [s, c]]
    return R

def rotate_y(theta):
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, s, -s, c
    return R


# =============================================================================
# FORWARD KINEMATICS
# =============================================================================
def forward_kinematics(theta):
    theta1, theta2, theta3, theta4 = theta

    T_base = translate(0, 0, BASE_HEIGHT)
    T1 = T_base @ rotate_z(theta1)
    T2 = T1 @ rotate_y(theta2)
    T3 = T2 @ translate(SHOULDER_LENGTH, 0, 0) @ rotate_y(theta3)
    T4 = T3 @ translate(ELBOW_LENGTH, 0, 0) @ rotate_y(theta4)
    T_end = T4 @ translate(WRIST_LENGTH, 0, 0)

    return T_end[:3, 3]


# =============================================================================
# INVERSE KINEMATICS
# =============================================================================
def ik_objective(theta, target):
    eff_pos = forward_kinematics(theta)
    return np.linalg.norm(eff_pos - target)

def solve_ik(target, initial_guess=[0, -np.pi/4, np.pi/4, 0]):
    bounds = [
        (-np.pi/2, np.pi/2),      # Base: ±90°
        (-2*np.pi/3, np.pi/6),    # Shoulder: -120° to +30°
        (-np.pi/2, np.pi/2),      # Elbow: ±90°
        (-np.pi/2, np.pi/2)       # Wrist: ±90°
    ]

    result = minimize(
        ik_objective,
        initial_guess,
        args=(target,),
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 2000, 'disp': False}
    )

    theta = result.x
    final_pos = forward_kinematics(theta)
    error = np.linalg.norm(final_pos - target)

    if error < 0.5:
        return theta, error
    else:
        return None, error


# =============================================================================
# PWM CONVERSION
# =============================================================================
def angles_deg_to_pwm(theta_deg, pwm_min=500, pwm_max=2500):
    limits = [
        (-90, 90),     # Base
        (-120, 30),    # Shoulder
        (-90, 90),     # Elbow
        (-90, 90)      # Wrist
    ]

    pwm = []
    for angle, (amin, amax) in zip(theta_deg, limits):
        angle = np.clip(angle, amin, amax)
        pwm_val = pwm_min + (angle - amin) * (pwm_max - pwm_min) / (amax - amin)
        pwm.append(int(np.clip(pwm_val, pwm_min, pwm_max)))

    return pwm


# =============================================================================
# MAIN FUNCTION: CALCULATE ANGLES FROM COORDINATES + PICK
# =============================================================================
def calculate_angles_from_coordinates(x_cm, y_cm, z_cm, object_width_cm=None):
    """
    If object_width_cm is provided:
      - move to target
      - close gripper to match object width
      - then go home (opens gripper again)
    """
    # Always start HOME (includes gripper open)
    go_home()

    target = np.array([x_cm, y_cm, z_cm])
    print(f"\n🎯 Target Coordinates: ({x_cm:.1f}, {y_cm:.1f}, {z_cm:.1f}) cm")

    horizontal_dist = np.sqrt(x_cm**2 + y_cm**2)
    vertical_dist = abs(z_cm - BASE_HEIGHT)
    total_dist = np.sqrt(horizontal_dist**2 + vertical_dist**2)
    max_reach = SHOULDER_LENGTH + ELBOW_LENGTH + WRIST_LENGTH

    if total_dist > max_reach or total_dist < 5.0:
        print(f"❌ Target NOT reachable! (distance: {total_dist:.1f} cm, max: {max_reach:.1f} cm)")
        return None

    theta, error = solve_ik(target, [0, -np.pi/4, np.pi/4, 0])

    if theta is None:
        print(f"❌ IK failed! Error: {error:.2f} cm")
        return None

    angles_deg = np.degrees(theta)

    print(f"✅ IK Solved! Position error: {error:.3f} cm")
    print(f"\n📐 Joint Angles (degrees):")
    print(f"   θ1 (Base):     {angles_deg[0]:+.2f}°")
    print(f"   θ2 (Shoulder): {angles_deg[1]:+.2f}°")
    print(f"   θ3 (Elbow):    {angles_deg[2]:+.2f}°")
    print(f"   θ4 (Wrist):    {angles_deg[3]:+.2f}°")

    pwm = angles_deg_to_pwm(angles_deg)

    print(f"\n📡 PWM Pulse Widths (microseconds):")
    print(f"   Servo 0 (Base):     {pwm[0]} μs")
    print(f"   Servo 1 (Shoulder): {pwm[1]} μs")
    print(f"   Servo 2 (Elbow):    {pwm[2]} μs")
    print(f"   Servo 3 (Wrist):    {pwm[3]} μs")

    # Move arm to target
    cmd_move = f"#0 P{pwm[0]} #1 P{pwm[1]} #2 P{pwm[2]} #3 P{pwm[3]} T2000"
    print(f"\n🤖 Move Command:")
    print(f"   {cmd_move}")
    send_pwm_command(cmd_move)
    time.sleep(2.3)

    # ✅ PICK: close gripper based on object width
    if object_width_cm is not None:
        grip_pwm = width_to_gripper_pwm(object_width_cm)
        print(f"\n📦 Picking object width = {object_width_cm:.2f} cm")
        print(f"   -> Gripper PWM = {grip_pwm} μs")
        set_gripper_pwm(grip_pwm, GRIPPER_MOVE_TIME_MS)

    # Return HOME (opens gripper again)
    go_home()

    calculated_pos = forward_kinematics(theta)
    print(f"\n✔️  Verification (Forward Kinematics):")
    print(f"   Target:     ({x_cm:.2f}, {y_cm:.2f}, {z_cm:.2f}) cm")
    print(f"   Calculated: ({calculated_pos[0]:.2f}, {calculated_pos[1]:.2f}, {calculated_pos[2]:.2f}) cm")
    print(f"   Error:      {error:.3f} cm")

    return angles_deg


# =============================================================================
# MAIN PROGRAM
# =============================================================================
if __name__ == "__main__":
    print("="*70)
    print("🤖 KINEMATICS CALCULATOR - Coordinates to Angles + PWM + GRIPPER PICK")
    print("="*70)

    # Example moves:
    calculate_angles_from_coordinates(20, 0, 15, object_width_cm=3.0)
    calculate_angles_from_coordinates(15, 15, 10, object_width_cm=4.5)

    print("\n" + "="*70)
    print("INTERACTIVE MODE")
    print("="*70)
    print("Enter: x,y,z,width   (cm) | width optional | type 'quit' to exit\n")
    print("Examples:")
    print("  20,0,15,3.2")
    print("  20,0,15       (just move, no pick)\n")

    while True:
        try:
            coords = input("Enter (x,y,z,width?): ").strip()
            if coords.lower() in ['quit', 'exit', 'q']:
                break

            parts = [p.strip() for p in coords.replace('(', '').replace(')', '').split(',')]
            if len(parts) < 3:
                raise ValueError

            x, y, z = map(float, parts[:3])
            width = float(parts[3]) if len(parts) >= 4 else None

            calculate_angles_from_coordinates(x, y, z, object_width_cm=width)

        except ValueError:
            print("❌ Invalid format. Use: x,y,z or x,y,z,width  (example: 20,0,15,3.2)")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

    print("\n✅ Calculator closed!")