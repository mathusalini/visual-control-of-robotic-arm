import time
import serial

# ============================================================
# SSC-32U Controller
# ============================================================
class SSC32U:
    def __init__(self, port="COM7", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port} @ {baudrate}")

    def send(self, cmd: str):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        # print("➡️", cmd)  # uncomment to see every step

    def close(self):
        self.ser.close()
        print("🔌 Connection closed")


# ============================================================
# SERVO CONFIG (USE YOUR FIRST CODE LIMITS, DO NOT CHANGE)
# ============================================================
JOINTS_CFG = {
    "base":     {"ch": 0, "amin": -90,  "amax": 90},
    "shoulder": {"ch": 1, "amin": -120, "amax": 30},
    "elbow":    {"ch": 2, "amin": -90,  "amax": 90},
    "wrist":    {"ch": 3, "amin": -90,  "amax": 90},
}
JOINT_ORDER = ["base", "shoulder", "elbow", "wrist"]

PWM_MIN = 500
PWM_MAX = 2500

# Track current PWM (start at 1500 for all)
current_pwm = {0: 1500, 1: 1500, 2: 1500, 3: 1500}


def clamp(x, a, b):
    return max(a, min(b, x))


# ============================================================
# Angle → PWM (SAME AS YOUR FIRST CODE, DO NOT CHANGE)
# ============================================================
def angle_to_pwm(angle_deg, amin, amax):
    angle_deg = clamp(angle_deg, amin, amax)
    pwm = PWM_MIN + (angle_deg - amin) * (PWM_MAX - PWM_MIN) / (amax - amin)
    return int(round(pwm))


def degrees_to_pwm_list(angles_deg):
    """
    angles_deg: [base, shoulder, elbow, wrist]
    returns clipped_angles, pwm_targets
    """
    clipped = []
    pwm_targets = []
    for joint, ang in zip(JOINT_ORDER, angles_deg):
        cfg = JOINTS_CFG[joint]
        amin, amax = cfg["amin"], cfg["amax"]
        ang_c = clamp(float(ang), amin, amax)
        clipped.append(ang_c)
        pwm_targets.append(angle_to_pwm(ang_c, amin, amax))
    return clipped, pwm_targets


def parse_angles_4(s: str):
    s = s.replace(",", " ").strip()
    parts = [p for p in s.split() if p]
    if len(parts) != 4:
        raise ValueError("Enter 4 angles: base shoulder elbow wrist")
    return [float(x) for x in parts]


# ============================================================
# SMOOTH MOVE ALL TOGETHER (interpolation steps)
# ============================================================
def move_all_together_smooth(ctrl: SSC32U, pwm_targets, total_time_ms=15000, step_us=5):
    """
    Smooth motion by interpolation (all joints together).

    total_time_ms: total move duration (bigger = slower)
    step_us: max PWM change per step (smaller = smoother)
    """
    start = [current_pwm[JOINTS_CFG[j]["ch"]] for j in JOINT_ORDER]
    target = [int(p) for p in pwm_targets]

    deltas = [abs(t - s) for s, t in zip(start, target)]
    max_delta = max(deltas) if deltas else 0

    if max_delta == 0:
        print("✅ Already at target.")
        return

    steps = max(1, int(max_delta / max(1, int(step_us))))
    dt_ms = total_time_ms / steps

    print(f"🟢 Smooth move: steps={steps}, dt≈{dt_ms:.1f} ms/step, total={total_time_ms} ms")

    for i in range(1, steps + 1):
        a = i / steps  # 0..1
        pw_step = [int(round(s + a * (t - s))) for s, t in zip(start, target)]

        cmd = ""
        for joint, pw in zip(JOINT_ORDER, pw_step):
            ch = JOINTS_CFG[joint]["ch"]
            cmd += f"#{ch} P{pw} "
        cmd += "T0"

        ctrl.send(cmd)
        time.sleep(dt_ms / 1000.0)

    # update current pwm
    for joint, pw in zip(JOINT_ORDER, target):
        ch = JOINTS_CFG[joint]["ch"]
        current_pwm[ch] = int(pw)


# ============================================================
# RUN
# ============================================================
if __name__ == "__main__":
    c = SSC32U(port="COM7", baudrate=9600)

    try:
        print("\nEnter 4 angles (deg): base shoulder elbow wrist")
        print("Example: 0 -60 0 0")
        print("Type 'exit' to quit.\n")

        while True:
            line = input("Degrees >>> ").strip().lower()
            if not line:
                continue
            if line in ("exit", "quit"):
                break

            angles = parse_angles_4(line)
            clipped, pwm_targets = degrees_to_pwm_list(angles)

            print("\n📐 Angles used (deg):", [round(a, 2) for a in clipped])

            print("📡 PWM targets:")
            for joint, pw in zip(JOINT_ORDER, pwm_targets):
                ch = JOINTS_CFG[joint]["ch"]
                print(f"  {joint:8s} (ch {ch}) : {current_pwm[ch]} -> {pw}")

            # Smooth move ALL servos together
            move_all_together_smooth(
                c,
                pwm_targets,
                total_time_ms=15000,  # increase to go slower (e.g., 20000)
                step_us=5             # decrease for smoother (e.g., 3)
            )

    finally:
        c.close()