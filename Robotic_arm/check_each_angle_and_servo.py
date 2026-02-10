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


# =============================
# Smooth movement (step-by-step)
# =============================
current_pwm = {
    0: 1500,
    1: 1500,
    2: 1500,
    3: 1500,
}


def move_servo_smooth(ctrl, joint, target_pwm,
                      total_time_ms=4000, step_us=5):
    ch = JOINTS[joint]
    start = current_pwm[ch]
    target = int(target_pwm)

    delta = abs(target - start)
    if delta == 0:
        print("✅ Already at target")
        return

    steps = max(1, delta // step_us)
    dt = total_time_ms / steps

    print(f"🐢 Moving {joint} smoothly: {start} → {target} PWM")

    for i in range(1, steps + 1):
        a = i / steps
        pwm = int(round(start + a * (target - start)))
        ctrl.send(f"#{ch} P{pwm} T0")
        time.sleep(dt / 1000.0)

    current_pwm[ch] = target


# =============================
# MAIN LOOP
# =============================
def main():
    arm = SSC32U(port="COM7")

    try:
        print("\nChoose joint: base / shoulder / elbow / wrist")
        print("Angle range: -90 to +90 (0° = 1500 PWM)")
        print("Type 'exit' to quit\n")

        while True:
            joint = input("Joint >>> ").strip().lower()
            if joint == "exit":
                break

            if joint not in JOINTS:
                print("❌ Invalid joint name")
                continue

            try:
                angle = float(input("Angle (deg) >>> "))
            except ValueError:
                print("❌ Invalid angle")
                continue

            mapped_pwm = angle_to_pwm(angle)
            safe_min, safe_max = PWM_SAFE[joint]

            print(f"📐 angle={angle}° → mapped PWM={mapped_pwm}")

            # SAFETY CHECK
            if not (safe_min <= mapped_pwm <= safe_max):
                print(
                    f"⛔ BLOCKED: {joint} PWM {mapped_pwm} outside "
                    f"safe range [{safe_min},{safe_max}]"
                )
                continue

            # Move smoothly
            move_servo_smooth(
                arm,
                joint,
                mapped_pwm,
                total_time_ms=5000,   # slower → increase
                step_us=5             # smoother → decrease
            )

    finally:
        arm.close()


if __name__ == "__main__":
    main()