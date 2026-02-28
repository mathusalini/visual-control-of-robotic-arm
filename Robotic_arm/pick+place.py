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
# SERVO CHANNELS  (⚠️ change gripper channel if needed)
# =========================================================
JOINTS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "wrist": 3,
    "gripper": 4,
}

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
# SMOOTH SERVO MOVEMENT (one channel)
# =========================================================
def move_servo_smooth(ctrl, joint, target_pwm, total_time_ms=5000, step_us=5):
    ch = JOINTS[joint]
    start = current_pwm.get(ch, 1500)
    target = int(target_pwm)

    delta = abs(target - start)
    if delta == 0:
        ctrl.send(f"#{ch} P{target} T0")
        print(f"✅ {joint} already at target ({target})")
        return

    steps = max(1, delta // step_us)
    dt = total_time_ms / steps

    print(f"🐢 Moving {joint}: {start} → {target} PWM")

    for i in range(1, steps + 1):
        a = i / steps
        pwm = int(round(start + a * (target - start)))
        ctrl.send(f"#{ch} P{pwm} T0")
        time.sleep(dt / 1000.0)

    current_pwm[ch] = target

# =========================================================
# HOME (one combined command for all channels)
# =========================================================
def go_home(ctrl, home_time_ms=3000):
    print("\n🏠 Going to HOME position...")

    base_pwm = 1500
    shoulder_pwm = 1500
    elbow_pwm = 1500
    wrist_pwm = 1500
    gripper_pwm = 1090

    for joint, pwm in {
        "base": base_pwm,
        "shoulder": shoulder_pwm,
        "elbow": elbow_pwm,
        "wrist": wrist_pwm,
        "gripper": gripper_pwm,
    }.items():
        mn, mx = PWM_SAFE[joint]
        if not (mn <= pwm <= mx):
            raise ValueError(f"HOME PWM for {joint} unsafe: {pwm}")

    cmd = (
        f"#{JOINTS['base']} P{base_pwm} "
        f"#{JOINTS['shoulder']} P{shoulder_pwm} "
        f"#{JOINTS['elbow']} P{elbow_pwm} "
        f"#{JOINTS['wrist']} P{wrist_pwm} "
        f"#{JOINTS['gripper']} P{gripper_pwm} "
        f"T{home_time_ms}"
    )
    ctrl.send(cmd)
    time.sleep(home_time_ms / 1000.0)

    current_pwm[JOINTS["base"]] = base_pwm
    current_pwm[JOINTS["shoulder"]] = shoulder_pwm
    current_pwm[JOINTS["elbow"]] = elbow_pwm
    current_pwm[JOINTS["wrist"]] = wrist_pwm
    current_pwm[JOINTS["gripper"]] = gripper_pwm

    print("✅ HOME reached\n")

# =========================================================
# PICK_HOME (keep gripper holding)
# =========================================================
def go_pick_home(ctrl, home_time_ms=3000):
    print("\n🏠 Going to PICK_HOME (keep gripper holding)...")

    base_pwm = 1500
    shoulder_pwm = 1500
    elbow_pwm = 1500
    wrist_pwm = 1500

    for joint, pwm in {
        "base": base_pwm,
        "shoulder": shoulder_pwm,
        "elbow": elbow_pwm,
        "wrist": wrist_pwm,
    }.items():
        mn, mx = PWM_SAFE[joint]
        if not (mn <= pwm <= mx):
            raise ValueError(f"PICK_HOME PWM for {joint} unsafe: {pwm}")

    cmd = (
        f"#{JOINTS['base']} P{base_pwm} "
        f"#{JOINTS['shoulder']} P{shoulder_pwm} "
        f"#{JOINTS['elbow']} P{elbow_pwm} "
        f"#{JOINTS['wrist']} P{wrist_pwm} "
        f"T{home_time_ms}"
    )
    ctrl.send(cmd)
    time.sleep(home_time_ms / 1000.0)

    current_pwm[JOINTS["base"]] = base_pwm
    current_pwm[JOINTS["shoulder"]] = shoulder_pwm
    current_pwm[JOINTS["elbow"]] = elbow_pwm
    current_pwm[JOINTS["wrist"]] = wrist_pwm

    print("✅ PICK_HOME reached (gripper unchanged)\n")

# =========================================================
# Gripper Control Based on Object Width
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
    pwm = max(mn, min(mx, pwm))
    return pwm

def move_gripper_width(ctrl, width_cm, total_time_ms=1500, step_us=5):
    pwm = width_to_pwm(width_cm)
    print(f"🖐️ Gripper: width {width_cm:.2f} cm → PWM {pwm}")
    move_servo_smooth(ctrl, "gripper", pwm, total_time_ms=total_time_ms, step_us=step_us)

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

    phi1_deg = math.degrees(phi1)
    theta2_deg = math.degrees(theta2)

    shoulder = phi1_deg - 90.0
    elbow    = -theta2_deg
    wrist    = alpha_deg

    print("\n=== IK RESULT (degrees) ===")
    print(f"Base     : {base_deg:.2f}°")
    print(f"Shoulder : {shoulder:.2f}°")
    print(f"Elbow    : {elbow:.2f}°")
    print(f"Wrist    : {wrist:.2f}°")

    return {"base": base_deg, "shoulder": shoulder, "elbow": elbow, "wrist": wrist}

# =========================================================
# AUTO MOVE USING IK (base -> shoulder -> elbow -> wrist)
# =========================================================
def auto_move_ik(ctrl, ik_angles, total_time_ms=5000, step_us=5):
    print("▶ AUTO MOVE: base → shoulder → elbow → wrist")

    for j in ("base", "shoulder", "elbow", "wrist"):
        angle = ik_angles[j]
        pwm = angle_to_pwm(angle)
        safe_min, safe_max = PWM_SAFE[j]

        print(f"{j}: {angle:.2f}° → PWM {pwm}")

        if not (safe_min <= pwm <= safe_max):
            print(f"⛔ {j} PWM out of range [{safe_min},{safe_max}] — skipped")
            continue

        move_servo_smooth(ctrl, j, pwm, total_time_ms=total_time_ms, step_us=step_us)

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

        ik_angles = ik_from_xyz(x, y, z, alpha)

        print("\nControls:")
        print("  Press Enter   -> AUTO pick sequence (move->grip->wait->pick_home->place)")
        print("  Type home     -> go home (resets gripper to 1090)")
        print("  Type pick_home-> go pick_home (keeps gripper holding)")
        print("  Type grip     -> enter object width and move gripper")
        print("  Type exit     -> quit\n")

        while True:
            cmd = input("Command (Enter=auto) >>> ").strip().lower()

            if cmd == "exit":
                break

            if cmd == "home":
                go_home(arm)
                continue

            if cmd == "pick_home":
                go_pick_home(arm)
                continue

            if cmd == "grip":
                width = float(input("Object width (cm) >>> "))
                move_gripper_width(arm, width, total_time_ms=1500, step_us=5)
                continue

            # AUTO: pick -> grip -> wait -> pick_home -> ask reached? -> place
            if cmd == "":
                # 1) Move to pick point
                auto_move_ik(arm, ik_angles, total_time_ms=5000, step_us=5)

                # 2) Grip using width
                width = float(input("✅ Reached PICK. Object width (cm) >>> "))
                move_gripper_width(arm, width, total_time_ms=1500, step_us=5)

                # 3) Wait to stabilize grip
                print(f"⏳ Waiting {GRIP_WAIT_SEC:.1f}s to stabilize grip...")
                time.sleep(GRIP_WAIT_SEC)

                # 4) Go to pick_home (keep holding)
                go_pick_home(arm, home_time_ms=3000)

                # 5) Ask if reached ok, then go to PLACE position
                if yes_no("✅ Reached ? (yes/no) >>> "):
                    print("\nEnter PLACE position")
                    px = float(input("place x (cm) >>> "))
                    py = float(input("place y (cm) >>> "))
                    pz = float(input("place z (cm) >>> "))
                    palpha = float(input("place wrist alpha (deg) >>> "))
                    
                    place_angles = ik_from_xyz(px, py, pz, palpha)
                    auto_move_ik(arm, place_angles, total_time_ms=5000, step_us=5)

                    print("✅ Reached PLACE position.")

                    # ✅ Wait 2 seconds before releasing
                    print("⏳ Waiting 2 seconds before releasing object...")
                    time.sleep(2)

                    # ✅ Fully open gripper (PWM = 1090)
                    print("🖐️ Opening gripper fully (PWM 1090)...")
                    move_servo_smooth(
                        arm,
                        "gripper",
                        1090,
                        total_time_ms=1500,
                        step_us=5
                    )

                    print("✅ Object released successfully.")

                    # ✅ Wait 2 seconds after release
                    print("⏳ Waiting 2 seconds before going HOME...")
                    time.sleep(5)

                    # ✅ Go to full HOME (resets gripper to 1090 again)
                    go_home(arm, home_time_ms=3000)
                    
                else:
                    print("⚠️ Not proceeding to place. (You can run auto again or use commands.)")

                continue

            print("❌ Invalid command")

    finally:
        arm.close()

# =========================================================
if __name__ == "__main__":
    main()