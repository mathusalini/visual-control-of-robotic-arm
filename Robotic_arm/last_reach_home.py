import numpy as np
from scipy.optimize import minimize

# ===== Serial PWM sending =====
import serial
import time

SERIAL_PORT = "COM7"      # CHANGE if needed
BAUDRATE    = 9600      # CHANGE if needed

ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"✅ Serial connected: {SERIAL_PORT} @ {BAUDRATE}")
except Exception as e:
    print(f"⚠️ Serial NOT connected ({SERIAL_PORT}). PWM will only print.\nReason: {e}")

def send_pwm_command(cmd: str):
    """Send command to controller via serial (SSC-32U style)."""
    if ser is None:
        return
    ser.write((cmd + "\r").encode())
# ===== END Serial =====


# =============================================================================
# GRIPPER SETTINGS (CALIBRATE THESE!)
# =============================================================================
GRIPPER_CH = 4

# Fully open / fully closed PWM for YOUR gripper servo (calibrate!)
GRIPPER_OPEN_PULSE   = 2000
GRIPPER_CLOSED_PULSE = 900

# Maximum object width when fully open (cm) (calibrate!)
GRIPPER_MAX_WIDTH_CM = 6.0

GRIPPER_MOVE_TIME_MS = 800


def width_to_gripper_pwm(width_cm: float) -> int:
    """
    Map object width (cm) -> gripper PWM.
    width = GRIPPER_MAX_WIDTH_CM => fully open
    width = 0 => fully closed
    """
    w = float(np.clip(width_cm, 0.0, GRIPPER_MAX_WIDTH_CM))
    ratio = w / GRIPPER_MAX_WIDTH_CM if GRIPPER_MAX_WIDTH_CM > 0 else 0.0
    pwm = GRIPPER_CLOSED_PULSE + ratio * (GRIPPER_OPEN_PULSE - GRIPPER_CLOSED_PULSE)
    return int(np.clip(
        pwm,
        min(GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE),
        max(GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE)
    ))


def set_gripper_pwm(pwm: int, t_ms: int = GRIPPER_MOVE_TIME_MS):
    """Move only the gripper servo."""
    cmd = f"#{GRIPPER_CH} P{pwm} T{t_ms}"
    print(f"🦾 Gripper cmd: {cmd}")
    send_pwm_command(cmd)
    time.sleep(t_ms / 1000.0 + 0.2)


# =============================================================================
# HOME POSITION (arm + gripper open)
# =============================================================================
HOME_PULSES   = [1500, 1500, 1500, 1500]  # Servo 0..3 (calibrate!)
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
# ARM DIMENSIONS (AL5D)
# =============================================================================
BASE_HEIGHT = 6.7
SHOULDER_LENGTH = 14.605
ELBOW_LENGTH = 18.7325
WRIST_LENGTH = 8.5725


# =============================================================================
# TRANSFORMATIONS
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
    return None, error


# =============================================================================
# ANGLES -> PWM
# =============================================================================
def angles_deg_to_pwm(theta_deg, pwm_min=500, pwm_max=2500):
    """
    Convert [θ1, θ2, θ3, θ4] degrees -> PWM (µs) using joint angle limits.
    """
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
# MOVE ARM TO XYZ (IK + PWM + SEND)
# =============================================================================
def move_to_xyz(x_cm, y_cm, z_cm, move_time_ms=2000):
    target = np.array([x_cm, y_cm, z_cm])

    # Reachability quick check
    horizontal_dist = np.sqrt(x_cm**2 + y_cm**2)
    vertical_dist = abs(z_cm - BASE_HEIGHT)
    total_dist = np.sqrt(horizontal_dist**2 + vertical_dist**2)
    max_reach = SHOULDER_LENGTH + ELBOW_LENGTH + WRIST_LENGTH

    if total_dist > max_reach or total_dist < 5.0:
        print(f"❌ Target NOT reachable: ({x_cm:.1f},{y_cm:.1f},{z_cm:.1f}) cm")
        return None

    theta, error = solve_ik(target, [0, -np.pi/4, np.pi/4, 0])
    if theta is None:
        print(f"❌ IK failed at ({x_cm:.1f},{y_cm:.1f},{z_cm:.1f}) | error={error:.2f} cm")
        return None

    angles_deg = np.degrees(theta)
    pwm = angles_deg_to_pwm(angles_deg)

    cmd = f"#0 P{pwm[0]} #1 P{pwm[1]} #2 P{pwm[2]} #3 P{pwm[3]} T{move_time_ms}"
    print(f"\n🤖 Move to ({x_cm:.1f},{y_cm:.1f},{z_cm:.1f}) | err={error:.3f} cm")
    print(f"   {cmd}")
    send_pwm_command(cmd)
    time.sleep(move_time_ms / 1000.0 + 0.3)

    return angles_deg


# =============================================================================
# PICK AND PLACE (ALWAYS RETURNS HOME AT END)
# =============================================================================
def pick_and_place(pick_xyz, place_xyz, object_width_cm, lift_height_cm=5.0):
    """
    Sequence:
      HOME(open) -> PICK -> GRIP -> LIFT -> PLACE(above) -> LOWER -> RELEASE
      -> LIFT -> HOME(open)
    """
    print("\n================ PICK & PLACE START ================\n")

    try:
        # 1) HOME (open)
        go_home()

        # 2) MOVE to PICK
        if move_to_xyz(*pick_xyz) is None:
            return

        # 3) GRIP to width
        grip_pwm = width_to_gripper_pwm(object_width_cm)
        print(f"\n📦 Gripping width={object_width_cm:.2f} cm -> gripper PWM={grip_pwm}")
        set_gripper_pwm(grip_pwm)

        # 4) LIFT after pick
        pick_lift = (pick_xyz[0], pick_xyz[1], pick_xyz[2] + lift_height_cm)
        move_to_xyz(*pick_lift)

        # 5) MOVE above PLACE
        place_lift = (place_xyz[0], place_xyz[1], place_xyz[2] + lift_height_cm)
        if move_to_xyz(*place_lift) is None:
            # Can't reach place: release and exit (finally will home)
            set_gripper_pwm(GRIPPER_OPEN_PULSE)
            return

        # 6) LOWER to PLACE
        if move_to_xyz(*place_xyz) is None:
            set_gripper_pwm(GRIPPER_OPEN_PULSE)
            return

        # 7) RELEASE
        print("\n📤 Releasing object (gripper open)")
        set_gripper_pwm(GRIPPER_OPEN_PULSE)

        # 8) LIFT after place
        move_to_xyz(*place_lift)

    finally:
        # 9) ALWAYS go HOME at the end
        print("\n🏁 Returning to HOME position (end of pick & place)")
        go_home()
        print("\n================ PICK & PLACE DONE =================\n")


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print("="*70)
    print("🤖 PICK & PLACE - Coordinates → IK → PWM → Servos + Gripper + HOME")
    print("="*70)

    # Example:
    PICK  = (20, 0, 15)       # object location (cm)
    PLACE = (10, 15, 12)      # drop location (cm)
    WIDTH = 3.0               # object width (cm)

    pick_and_place(PICK, PLACE, WIDTH, lift_height_cm=5.0)

    print("\n" + "="*70)
    print("INTERACTIVE MODE")
    print("="*70)
    print("Enter: pickx,picky,pickz, placex,placey,placez, width")
    print("Example: 20,0,15, 10,15,12, 3.0\n")

    while True:
        try:
            s = input("Enter values (or 'quit'): ").strip()
            if s.lower() in ["quit", "q", "exit"]:
                break

            parts = [p.strip() for p in s.replace("(", "").replace(")", "").split(",")]
            if len(parts) != 7:
                raise ValueError

            px, py, pz, qx, qy, qz, w = map(float, parts)
            pick_and_place((px, py, pz), (qx, qy, qz), w, lift_height_cm=5.0)

        except ValueError:
            print("❌ Format: pickx,picky,pickz,placex,placey,placez,width")
        except KeyboardInterrupt:
            print("\nExiting...")
            break

    print("\n✅ Done!")