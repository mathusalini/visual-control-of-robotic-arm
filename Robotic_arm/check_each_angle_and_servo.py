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
        print("➡️", cmd)

    def close(self):
        self.ser.close()
        print("🔌 Closed connection")


# =============================
# Servo configuration
# =============================
JOINTS = {
    "base":     {"ch": 0, "amin": -90,  "amax": 90},
    "shoulder": {"ch": 1, "amin": -120, "amax": 30},
    "elbow":    {"ch": 2, "amin": -90,  "amax": 90},
    "wrist":    {"ch": 3, "amin": -90,  "amax": 90},
}

PWM_MIN = 500
PWM_MAX = 2500
PWM_CENTER = 1500


# =============================
# Angle → PWM
# =============================
def angle_to_pwm(angle_deg, amin, amax):
    angle_deg = max(amin, min(amax, angle_deg))
    pwm = PWM_MIN + (angle_deg - amin) * (PWM_MAX - PWM_MIN) / (amax - amin)
    return int(pwm)


# =============================
# Main interactive control
# =============================
def main():
    arm = SSC32U(port="COM7")

    try:
        print("\nType joint name: base / shoulder / elbow / wrist")
        print("Type 'exit' to quit\n")

        while True:
            joint = input("Joint: ").strip().lower()
            if joint == "exit":
                break

            if joint not in JOINTS:
                print("❌ Invalid joint")
                continue

            try:
                angle = float(input(f"Angle for {joint} (deg): "))
            except ValueError:
                print("❌ Invalid angle")
                continue

            j = JOINTS[joint]
            pwm = angle_to_pwm(angle, j["amin"], j["amax"])

            # move slowly (you can increase T)
            T_MS = 4000

            arm.send(f"#{j['ch']} P{pwm} T{T_MS}")
            time.sleep(T_MS / 1000.0)

    finally:
        arm.close()


if __name__ == "__main__":
    main()