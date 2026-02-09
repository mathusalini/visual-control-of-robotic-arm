import serial
import time

class SSC32U:
    def __init__(self, port="COM7", baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port} @ {baudrate}")

    def send(self, cmd: str):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        print("➡️ Sent:", cmd)

    def move_one(self, channel: int, pwm: int, time_ms: int = 2000):
        # safety clamp (you can change limits)
        pwm = max(500, min(2500, int(pwm)))
        cmd = f"#{channel} P{pwm} T{time_ms}"
        self.send(cmd)

    def close(self):
        self.ser.close()
        print("✅ Connection closed")


if __name__ == "__main__":
    controller = SSC32U(port="COM7", baudrate=9600)

    # channels
    CH_BASE = 0
    CH_SHOULDER = 1
    CH_ELBOW = 2
    CH_WRIST = 3

    joints = [
        ("Base", CH_BASE),
        ("Shoulder", CH_SHOULDER),
        ("Elbow", CH_ELBOW),
        ("Wrist", CH_WRIST),
    ]

    try:
        print("\nEnter PWM (500 to 2500). Type 'q' to quit.\n")

        while True:
            # move joints one-by-one serially
            for joint_name, ch in joints:
                val = input(f"Enter PWM for {joint_name} (ch {ch}): ").strip()
                if val.lower() in ["q", "quit", "exit"]:
                    raise KeyboardInterrupt

                pwm = int(val)

                t = input("Enter move time ms (default 2000): ").strip()
                time_ms = 2000 if t == "" else int(t)

                print(f"\n✅ Moving {joint_name} only...")
                controller.move_one(ch, pwm, time_ms=time_ms)

                # wait until motion completes
                time.sleep(time_ms / 1000.0 + 0.2)

            print("\n🔁 One full cycle done (Base → Shoulder → Elbow → Wrist)\n")

    except (KeyboardInterrupt, SystemExit):
        print("\n👋 Exiting...")

    finally:
        controller.close()