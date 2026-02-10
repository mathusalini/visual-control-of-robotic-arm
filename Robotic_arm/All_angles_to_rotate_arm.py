import time
import serial

# =============================
# SSC-32U Controller
# =============================
class SSC32U:
    def __init__(self, port="COM7", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port}")

    def send(self, cmd):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        # print("➡️", cmd)

    def close(self):
        self.ser.close()
        print("🔌 Closed connection")


# =============================
# Servo channels
# =============================
JOINTS = {
    "base": 0,
    "shoulder": 1,
    "elbow": 2,
    "wrist": 3,
}
ORDER = ["base", "shoulder", "elbow", "wrist"]

# =============================
# PWM safety limits (HARD)
# =============================
PWM_SAFE = {
    "base":     (500, 2500),
    "shoulder": (700, 2033),
    "elbow":    (833, 2000),
    "wrist":    (500, 2500),
}

# =============================
# Global angle mapping
# =============================
PWM_MIN = 500
PWM_CENTER = 1500
PWM_MAX = 2500


def clamp(x, a, b):
    return max(a, min(b, x))


def angle_to_pwm(angle_deg):
    """Global rule: -90..0..+90 => 500..1500..2500"""
    a = clamp(float(angle_deg), -90.0, 90.0)
    if a < 0:
        pwm = PWM_CENTER + (a / 90.0) * (PWM_CENTER - PWM_MIN)
    else:
        pwm = PWM_CENTER + (a / 90.0) * (PWM_MAX - PWM_CENTER)
    return int(round(pwm))


def parse_4_angles(s: str):
    s = s.replace(",", " ").strip()
    parts = [p for p in s.split() if p]
    if len(parts) != 4:
        raise ValueError("Enter 4 angles: base shoulder elbow wrist")
    return [float(x) for x in parts]


# =============================
# Track current PWM
# =============================
current_pwm = {0: 1500, 1: 1500, 2: 1500, 3: 1500}


# =============================
# Smooth move ALL servos together
# =============================
def move_all_smooth(ctrl, target_pwm_by_ch, total_time_ms=5000, step_us=3):
    start = [current_pwm[ch] for ch in range(4)]
    target = [target_pwm_by_ch[ch] for ch in range(4)]

    deltas = [abs(t - s) for s, t in zip(start, target)]
    max_delta = max(deltas)

    if max_delta == 0:
        print("✅ Already at target (no movement).")
        return

    steps = max(1, max_delta // step_us)
    dt = total_time_ms / steps

    print(f"🟢 Smooth move all: steps={steps}, dt≈{dt:.1f} ms/step")

    for i in range(1, steps + 1):
        a = i / steps
        pw_step = [int(round(s + a * (t - s))) for s, t in zip(start, target)]

        cmd = ""
        for joint, pw in zip(ORDER, pw_step):
            ch = JOINTS[joint]
            cmd += f"#{ch} P{pw} "
        cmd += "T0"

        ctrl.send(cmd)
        time.sleep(dt / 1000.0)

    # update current_pwm
    for ch in range(4):
        current_pwm[ch] = target[ch]


# =============================
# MAIN LOOP
# =============================
def main():
    arm = SSC32U(port="COM7")

    try:
        print("\nEnter 4 angles: base shoulder elbow wrist (deg)")
        print("Rule: -90..0..+90 => 500..1500..2500 (0°=1500)")
        print("Safety: base 500-2500, shoulder 700-2033, elbow 833-2000, wrist 500-2500")
        print("Example: 0 0 0 0")
        print("Type 'exit' to quit\n")

        while True:
            line = input("Angles >>> ").strip().lower()
            if not line:
                continue
            if line in ("exit", "quit"):
                break

            angles = parse_4_angles(line)

            # Convert angles -> PWM (mapped)
            mapped_pwm = {}
            blocked = []

            for joint, ang in zip(ORDER, angles):
                ch = JOINTS[joint]
                pwm = angle_to_pwm(ang)
                smin, smax = PWM_SAFE[joint]

                if smin <= pwm <= smax:
                    mapped_pwm[ch] = pwm
                else:
                    # blocked joint: keep current
                    mapped_pwm[ch] = current_pwm[ch]
                    blocked.append((joint, pwm, smin, smax))

            # Show info
            print("\n📐 Angles:", angles)
            for joint, ang in zip(ORDER, angles):
                ch = JOINTS[joint]
                print(f"  {joint:8s} ch{ch}: angle={ang:6.1f} -> mapped={angle_to_pwm(ang):4d} -> send={mapped_pwm[ch]:4d} safe={PWM_SAFE[joint]}")

            if blocked:
                print("\n⛔ Blocked joints (won't move):")
                for joint, pwm, smin, smax in blocked:
                    print(f"  {joint}: mapped {pwm} outside [{smin},{smax}]")

            # Move all smoothly together
            move_all_smooth(arm, mapped_pwm, total_time_ms=7000, step_us=3)

    finally:
        arm.close()


if __name__ == "__main__":
    main()