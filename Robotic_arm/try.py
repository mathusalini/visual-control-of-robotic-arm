import time
import serial
import numpy as np
from scipy.optimize import minimize

# =============================================================================
# SSC-32U
# =============================================================================
class SSC32U:
    def __init__(self, port="COM7", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port} @ {baudrate}")

    def send(self, cmd: str):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        print("➡️", cmd)

    def close(self):
        self.ser.close()
        print("🔌 Connection closed")


# =============================================================================
# ARM DIMENSIONS
# =============================================================================
BASE_HEIGHT = 6.7
SHOULDER_LENGTH = 14.605
ELBOW_LENGTH = 18.7325
WRIST_LENGTH = 8.5725

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

def forward_kinematics(theta):
    theta1, theta2, theta3, theta4 = theta
    T_base = translate(0, 0, BASE_HEIGHT)
    T1 = T_base @ rotate_z(theta1)
    T2 = T1 @ rotate_y(theta2)
    T3 = T2 @ translate(SHOULDER_LENGTH, 0, 0) @ rotate_y(theta3)
    T4 = T3 @ translate(ELBOW_LENGTH, 0, 0) @ rotate_y(theta4)
    T_end = T4 @ translate(WRIST_LENGTH, 0, 0)
    return T_end[:3, 3]

def ik_objective(theta, target):
    return np.linalg.norm(forward_kinematics(theta) - target)

def solve_ik(target, initial_guess=[0, -np.pi/4, np.pi/4, 0]):
    bounds = [
        (-np.pi/2, np.pi/2),      # Base
        (-2*np.pi/3, np.pi/6),    # Shoulder
        (-np.pi/2, np.pi/2),      # Elbow
        (-np.pi/2, np.pi/2)       # Wrist
    ]
    result = minimize(
        ik_objective,
        initial_guess,
        args=(target,),
        method="L-BFGS-B",
        bounds=bounds,
        options={"maxiter": 2000, "disp": False}
    )
    theta = result.x
    error = np.linalg.norm(forward_kinematics(theta) - target)
    if error < 0.5:
        return theta, error
    return None, error

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
# SPEED SETTINGS (YOUR RULE)
# =============================================================================
MS_PER_100US = 200         # 100us difference takes 5ms
MS_PER_US = MS_PER_100US / 100.0   # 0.05 ms per 1us

MIN_TIME_MS = 5000         # so tiny moves are still visible / stable
MAX_TIME_MS = 60000        # limit so huge moves don't take forever


# =============================================================================
# MOVE ONE BY ONE, TIME = f(PWM DIFFERENCE)
# =============================================================================
CH = [0, 1, 2, 3]  # base, shoulder, elbow, wrist

# Track current PWM for each channel (start at home = 1500)
current_pwm = {0: 1500, 1: 1500, 2: 1500, 3: 1500}

def move_servo_smooth(controller: SSC32U, ch: int, target_pw: int, gap_ms=50):
    prev = current_pwm[ch]
    delta = abs(target_pw - prev)

    # time based on delta
    t_ms = int(delta * MS_PER_US)
    t_ms = max(MIN_TIME_MS, min(MAX_TIME_MS, t_ms))

    controller.send(f"#{ch} P{target_pw} T{t_ms}")

    # wait until motion should finish
    time.sleep(t_ms / 1000.0 + gap_ms / 1000.0)

    current_pwm[ch] = target_pw


def move_to_xyz_one_by_one_speed(controller: SSC32U, x_cm, y_cm, z_cm):
    target = np.array([x_cm, y_cm, z_cm], dtype=float)

    theta, err = solve_ik(target)
    if theta is None:
        print(f"❌ IK failed. error={err:.3f} cm")
        return False

    angles_deg = np.degrees(theta)
    pwm_targets = angles_deg_to_pwm(angles_deg)

    print(f"\n🎯 Target ({x_cm},{y_cm},{z_cm}) | IK err={err:.3f} cm")
    print("📡 PWM targets:", pwm_targets)

    # servo-by-servo, with speed rule
    for ch, pw in zip(CH, pwm_targets):
        move_servo_smooth(controller, ch, pw)

    return True


# =============================================================================
# RUN
# =============================================================================
if __name__ == "__main__":
    c = SSC32U(port="COM7", baudrate=9600)

    try:
        # HOME one-by-one using speed rule
        for ch in CH:
            move_servo_smooth(c, ch, 1500)

        # Example points
        move_to_xyz_one_by_one_speed(c, 15, 15, 10)

    finally:
        c.close()