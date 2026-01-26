import serial
import time

class SSC32U:
    def __init__(self, port='COM7', baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print(f"Connected to SSC-32U on {port} at {baudrate} baud")

    def _send(self, cmd: str):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
        print("Sent:", cmd)

    def move_multiple_pw(self, channels, pw=1500, time_ms=2000):
        # Send one command to center all channels
        cmd = ""
        for ch in channels:
            cmd += f"#{ch} P{pw} "
        cmd += f"T{time_ms}"
        self._send(cmd)

    def close(self):
        self.ser.close()
        print("Connection closed")


if __name__ == "__main__":
    controller = SSC32U(port="COM7", baudrate=9600)

    try:
        # Channels used in your arm
        CH_BASE = 0
        CH_SHOULDER = 1
        CH_ELBOW = 2
        CH_WRIST = 3

        all_channels = [CH_BASE, CH_SHOULDER, CH_ELBOW, CH_WRIST]

        # ✅ Step 0: HOME (center all servos to 1500)
        print("\nStep 0) Going to HOME (all servos = 1500)")
        controller.move_multiple_pw(all_channels, pw=1500, time_ms=2000)
        time.sleep(2.2)

        # ✅ Now do your task (example sequence)
        print("\nStep 1) Base move")
        controller._send("#0 P1555 T2000")
        time.sleep(2.2)

        print("\nStep 2) Shoulder move")
        controller._send("#1 P1611 T2000")
        time.sleep(2.2)

        print("\nStep 3) Elbow move")
        controller._send("#2 P1583 T2000")
        time.sleep(2.2)

        print("\nStep 4) Wrist move")
        controller._send("#3 P1611 T2000")
        time.sleep(2.2)

        print("\nDone ✅")

    finally:
        controller.close()




# import serial, time

# PORT = "COM7"
# BAUD = 9600
# CH_BASE = 0

# def send(ser, cmd):
#     ser.write((cmd + "\r").encode("ascii"))
#     ser.flush()
#     print("Sent:", cmd)

# ser = serial.Serial(PORT, BAUD, timeout=1)
# time.sleep(2)
# print(f"Connected on {PORT} @ {BAUD}")

# last_pw = 1500

# try:
#     # Go to center slowly
#     send(ser, f"#{CH_BASE} P1500 T1500")
#     time.sleep(1.7)

#     # Start with a conservative safe window first
#     left_min  = 1200
#     right_max = 1800

#     step = 10       # smaller step = safer
#     move_time = 600 # slower movement = safer

#     print("\n--- LEFT test (decreasing pulse) ---")
#     print("Watch the arm. Press Ctrl+C immediately if it strains / hits stop.")
#     for pw in range(1500, left_min - 1, -step):
#         last_pw = pw
#         send(ser, f"#{CH_BASE} P{pw} T{move_time}")
#         time.sleep((move_time/1000) + 0.1)

#     # Back to center before testing other side
#     send(ser, f"#{CH_BASE} P1500 T1500")
#     time.sleep(1.7)

#     print("\n--- RIGHT test (increasing pulse) ---")
#     print("Watch the arm. Press Ctrl+C immediately if it strains / hits stop.")
#     for pw in range(1500, right_max + 1, step):
#         last_pw = pw
#         send(ser, f"#{CH_BASE} P{pw} T{move_time}")
#         time.sleep((move_time/1000) + 0.1)

#     print("\nFinished conservative range test safely ✅")
#     print("If you need more range, expand left_min/right_max slowly (e.g., 1180, 1820).")

# except KeyboardInterrupt:
#     print(f"\nStopped by you ✅ Last pulse tried: P{last_pw}")
#     # Stop servo outputs immediately
#     try:
#         send(ser, "STOP")
#     except:
#         pass

# finally:
#     # Return to center at the end
#     try:
#         send(ser, f"#{CH_BASE} P1500 T1500")
#         time.sleep(1.7)
#     except:
#         pass
#     ser.close()
#     print("Closed.")
