import math
import time
import serial

# =========================================================
# ARM LINK LENGTHS (cm)
# =========================================================
L1 = 14.605
L2 = 18.7325
L3 = 8.5725

# ✅ Wait time after gripping (seconds)
GRIP_WAIT_SEC = 3.0

# =========================================================
# SSC-32U CONTROLLER
# =========================================================
class SSC32U:
    def __init__(self, port="COM7", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port}")

    def send(self, cmd):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()

    def close(self):
        self.ser.close()
        print("🔌 Connection closed")

# =========================================================
# SERVO CHANNELS
# =========================================================
JOINTS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "wrist": 3,
    "gripper": 4,
}
ORDER_4 = ["base", "shoulder", "elbow", "wrist"]

# =========================================================
# PWM SAFETY LIMITS
# =========================================================
PWM_SAFE = {
    "base":     (500, 2500),
    "shoulder": (700, 2033),
    "elbow":    (833, 2000),
    "wrist":    (500, 2500),
    "gripper":  (500, 2500),
}

PWM_MIN = 500
PWM_CENTER = 1500
PWM_MAX = 2500

# =========================================================
# UTILS
# =========================================================
def clamp(x, a, b):
    return max(a, min(b, x))

def angle_to_pwm(angle_deg):
    """Map -90..0..+90 → 500..1500..2500"""
    a = clamp(float(angle_deg), -90.0, 90.0)
    if a < 0:
        pwm = PWM_CENTER + (a / 90.0) * (PWM_CENTER - PWM_MIN)
    else:
        pwm = PWM_CENTER + (a / 90.0) * (PWM_MAX - PWM_CENTER)
    return int(round(pwm))

def yes_no(prompt):
    ans = input(prompt).strip().lower()
    return ans in ("y", "yes")

# =========================================================
# CURRENT PWM (software tracking)
# =========================================================
current_pwm = {
    JOINTS["base"]: 1500,
    JOINTS["shoulder"]: 1500,
    JOINTS["elbow"]: 1500,
    JOINTS["wrist"]: 1500,
    JOINTS["gripper"]: 1500,
}

# =========================================================
# PARALLEL SMOOTH MOVE (base+shoulder+elbow+wrist together)
# =========================================================
def move_all_joints_smooth(ctrl, targets_by_joint, total_time_ms=5000, step_us=3):
    start = [current_pwm[JOINTS[j]] for j in ORDER_4]
    target = [int(targets_by_joint[j]) for j in ORDER_4]

    deltas = [abs(t - s) for s, t in zip(start, target)]
    max_delta = max(deltas)

    if max_delta == 0:
        print("✅ Already at target (no movement).")
        return

    steps = max(1, max_delta // step_us)
    dt = total_time_ms / steps

    print(f"🟢 Parallel smooth move: steps={steps}, dt≈{dt:.1f} ms/step")

    for i in range(1, steps + 1):
        a = i / steps
        pw_step = [int(round(s + a * (t - s))) for s, t in zip(start, target)]

        cmd = ""
        for joint, pw in zip(ORDER_4, pw_step):
            ch = JOINTS[joint]
            cmd += f"#{ch} P{pw} "
        cmd += "T0"

        ctrl.send(cmd)
        time.sleep(dt / 1000.0)

    # update tracking
    for joint, pw in zip(ORDER_4, target):
        current_pwm[JOINTS[joint]] = pw

# =========================================================
# GRIPPER SMOOTH (single channel)
# =========================================================
def move_gripper_smooth(ctrl, target_pwm, total_time_ms=1500, step_us=5):
    ch = JOINTS["gripper"]
    start = current_pwm.get(ch, 1500)
    target = int(target_pwm)

    delta = abs(target - start)
    if delta == 0:
        ctrl.send(f"#{ch} P{target} T0")
        return

    steps = max(1, delta // step_us)
    dt = total_time_ms / steps

    for i in range(1, steps + 1):
        a = i / steps
        pwm = int(round(start + a * (target - start)))
        ctrl.send(f"#{ch} P{pwm} T0")
        time.sleep(dt / 1000.0)

    current_pwm[ch] = target

# =========================================================
# HOME (one command, parallel for joints)
# =========================================================
def go_home(ctrl, home_time_ms=3000):
    print("\n🏠 Going to HOME position...")

    targets = {
        "base": 1500,
        "shoulder": 1500,
        "elbow": 1500,
        "wrist": 1500,
    }
    gripper_pwm = 1090

    # safety check for joints
    for j, pwm in targets.items():
        mn, mx = PWM_SAFE[j]
        if not (mn <= pwm <= mx):
            raise ValueError(f"HOME PWM for {j} unsafe: {pwm}")

    # move 4 joints in parallel
    move_all_joints_smooth(ctrl, targets, total_time_ms=home_time_ms, step_us=3)

    # then gripper
    mn, mx = PWM_SAFE["gripper"]
    if not (mn <= gripper_pwm <= mx):
        raise ValueError(f"HOME PWM for gripper unsafe: {gripper_pwm}")
    move_gripper_smooth(ctrl, gripper_pwm, total_time_ms=1200, step_us=5)

    print("✅ HOME reached\n")

# =========================================================
# PICK_HOME (keep gripper holding)
# =========================================================
def go_pick_home(ctrl, home_time_ms=3000):
    print("\n🏠 Going to PICK_HOME (keep gripper holding)...")

    targets = {
        "base": 1500,
        "shoulder": 1500,
        "elbow": 1500,
        "wrist": 1500,
    }

    for j, pwm in targets.items():
        mn, mx = PWM_SAFE[j]
        if not (mn <= pwm <= mx):
            raise ValueError(f"PICK_HOME PWM for {j} unsafe: {pwm}")

    move_all_joints_smooth(ctrl, targets, total_time_ms=home_time_ms, step_us=3)
    print("✅ PICK_HOME reached (gripper unchanged)\n")

# =========================================================
# Gripper width mapping
# =========================================================
def width_to_pwm(width_cm):
    width_cm = float(width_cm)
    if width_cm >= 3.2:
        pwm = 1090
    elif width_cm <= 0:
        pwm = 2500
    else:
        pwm = int(2500 - ((2500 - 1090) / 3.2) * width_cm)

    mn, mx = PWM_SAFE["gripper"]
    return max(mn, min(mx, pwm))

def move_gripper_width(ctrl, width_cm, total_time_ms=1500, step_us=5):
    pwm = width_to_pwm(width_cm)
    print(f"🖐️ Gripper: width {width_cm:.2f} cm → PWM {pwm}")
    move_gripper_smooth(ctrl, pwm, total_time_ms=total_time_ms, step_us=step_us)

# =========================================================
# INVERSE KINEMATICS
# =========================================================
def ik_from_xyz(x, y, z, alpha_deg):
    base_deg = math.degrees(math.atan2(y, x))

    j = math.sqrt(x*x + y*y)
    k = z
    a = math.radians(alpha_deg)

    m = j - L3 * math.cos(a)
    n = k - L3 * math.sin(a)

    l = math.sqrt(m*m + n*n)
    if l > (L1 + L2) or l < abs(L1 - L2):
        raise ValueError("❌ Target not reachable")

    theta12 = math.atan2(n, m)

    c13 = (L2*L2 - L1*L1 - l*l) / (-2 * L1 * l)
    c13 = clamp(c13, -1.0, 1.0)
    theta13 = math.acos(c13)

    phi1 = theta12 + theta13

    c2 = (l*l - L2*L2 - L1*L1) / (-2 * L2 * L1)
    c2 = clamp(c2, -1.0, 1.0)
    phi2 = math.acos(c2)

    theta2 = phi1 + phi2 - math.pi

    shoulder = math.degrees(phi1) - 90.0
    elbow = -math.degrees(theta2)
    wrist = alpha_deg

    return {"base": base_deg, "shoulder": shoulder, "elbow": elbow, "wrist": wrist}

# =========================================================
# AUTO MOVE IK (PARALLEL)
# =========================================================
def auto_move_ik(ctrl, ik_angles, total_time_ms=5000, step_us=3):
    targets = {}
    for j in ORDER_4:
        pwm = angle_to_pwm(ik_angles[j])
        smin, smax = PWM_SAFE[j]
        if not (smin <= pwm <= smax):
            raise ValueError(f"⛔ {j} PWM out of range [{smin},{smax}] -> {pwm}")
        targets[j] = pwm

    move_all_joints_smooth(ctrl, targets, total_time_ms=total_time_ms, step_us=step_us)

# =========================================================
# MAIN PROGRAM
# =========================================================
def main():
    arm = SSC32U("COM7")

    try:
        go_home(arm)

        print("Enter PICK position (target object position)")
        x = float(input("x (cm) >>> "))
        y = float(input("y (cm) >>> "))
        z = float(input("z (cm) >>> "))
        alpha = float(input("wrist alpha (deg) >>> "))

        pick_angles = ik_from_xyz(x, y, z, alpha)

        while True:
            cmd = input("Command (Enter=auto, exit) >>> ").strip().lower()
            if cmd == "exit":
                break

            if cmd == "":
                auto_move_ik(arm, pick_angles, total_time_ms=5000, step_us=3)

                width = float(input("✅ Reached PICK. Object width (cm) >>> "))
                move_gripper_width(arm, width)

                print(f"⏳ Waiting {GRIP_WAIT_SEC:.1f}s...")
                time.sleep(GRIP_WAIT_SEC)

                go_pick_home(arm)

                if yes_no("Go to PLACE? (yes/no) >>> "):
                    px = float(input("place x (cm) >>> "))
                    py = float(input("place y (cm) >>> "))
                    pz = float(input("place z (cm) >>> "))
                    palpha = float(input("place wrist alpha (deg) >>> "))

                    place_angles = ik_from_xyz(px, py, pz, palpha)
                    auto_move_ik(arm, place_angles)

                    print("✅ Reached PLACE position.")
                    time.sleep(2)

                    move_gripper_smooth(arm, 1090)
                    print("✅ Released.")

                    time.sleep(2)
                    go_home(arm)

    finally:
        arm.close()

if __name__ == "__main__":
    main()