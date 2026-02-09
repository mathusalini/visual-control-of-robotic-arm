# import serial
# import time
# import numpy as np
# from scipy.optimize import minimize

# # =============================================================================
# # 1) SSC-32U SERIAL CONTROLLER
# # =============================================================================
# class SSC32U:
#     def __init__(self, port="COM7", baudrate=9600, timeout=1):
#         self.ser = serial.Serial(port, baudrate, timeout=timeout)
#         time.sleep(2)
#         print(f"✅ Connected to SSC-32U on {port} @ {baudrate}")

#     def send(self, cmd: str):
#         self.ser.write((cmd + "\r").encode("ascii"))
#         self.ser.flush()
#         print("➡️ Sent:", cmd)

#     def move_servos(self, pwm_list, channels=(0, 1, 2, 3), time_ms=2000):
#         """
#         Send one combined command to move multiple servos together.
#         Example: #0 P1500 #1 P1500 #2 P1500 #3 P1500 T2000
#         """
#         cmd = ""
#         for ch, pw in zip(channels, pwm_list):
#             cmd += f"#{ch} P{int(pw)} "
#         cmd += f"T{int(time_ms)}"
#         self.send(cmd)

#     def close(self):
#         self.ser.close()
#         print("✅ Connection closed")


# # =============================================================================
# # 2) ARM DIMENSIONS (AL5D)
# # =============================================================================
# BASE_HEIGHT = 6.7
# SHOULDER_LENGTH = 14.605
# ELBOW_LENGTH = 18.7325
# WRIST_LENGTH = 8.5725


# # =============================================================================
# # 3) TRANSFORM UTILITIES
# # =============================================================================
# def translate(x, y, z):
#     T = np.eye(4)
#     T[:3, 3] = [x, y, z]
#     return T

# def rotate_z(theta):
#     R = np.eye(4)
#     c, s = np.cos(theta), np.sin(theta)
#     R[0:2, 0:2] = [[c, -s], [s, c]]
#     return R

# def rotate_y(theta):
#     R = np.eye(4)
#     c, s = np.cos(theta), np.sin(theta)
#     R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, s, -s, c
#     return R


# # =============================================================================
# # 4) FORWARD KINEMATICS
# # =============================================================================
# def forward_kinematics(theta):
#     theta1, theta2, theta3, theta4 = theta

#     T_base = translate(0, 0, BASE_HEIGHT)
#     T1 = T_base @ rotate_z(theta1)
#     T2 = T1 @ rotate_y(theta2)
#     T3 = T2 @ translate(SHOULDER_LENGTH, 0, 0) @ rotate_y(theta3)
#     T4 = T3 @ translate(ELBOW_LENGTH, 0, 0) @ rotate_y(theta4)
#     T_end = T4 @ translate(WRIST_LENGTH, 0, 0)

#     return T_end[:3, 3]


# # =============================================================================
# # 5) INVERSE KINEMATICS
# # =============================================================================
# def ik_objective(theta, target):
#     eff_pos = forward_kinematics(theta)
#     return np.linalg.norm(eff_pos - target)

# def solve_ik(target, initial_guess=(0, -np.pi/4, np.pi/4, 0)):
#     bounds = [
#         (-np.pi/2, np.pi/2),      # Base
#         (-2*np.pi/3, np.pi/6),    # Shoulder
#         (-np.pi/2, np.pi/2),      # Elbow
#         (-np.pi/2, np.pi/2)       # Wrist
#     ]

#     result = minimize(
#         ik_objective,
#         np.array(initial_guess, dtype=float),
#         args=(target,),
#         method="L-BFGS-B",
#         bounds=bounds,
#         options={"maxiter": 2000, "disp": False}
#     )

#     theta = result.x
#     final_pos = forward_kinematics(theta)
#     error = np.linalg.norm(final_pos - target)

#     if error < 0.5:
#         return theta, error
#     return None, error


# # =============================================================================
# # 6) PWM CALIBRATION (VERY IMPORTANT FOR REAL ARM)
# # =============================================================================
# # Each joint can be tuned:
# # - center: PWM when angle = 0°
# # - min/max: absolute limits SSC-32U should send
# # - amin/amax: real angle limits in degrees for mapping
# # - invert: True flips direction (use if servo moves opposite way)
# SERVO_CONFIG = [
#     {"center": 1500, "min": 500,  "max": 2500, "amin": -90,  "amax":  90, "invert": False},  # ch0 Base
#     {"center": 1500, "min": 500,  "max": 2500, "amin": -120, "amax":  30, "invert": False},  # ch1 Shoulder
#     {"center": 1500, "min": 500,  "max": 2500, "amin": -90,  "amax":  90, "invert": False},  # ch2 Elbow
#     {"center": 1500, "min": 500,  "max": 2500, "amin": -90,  "amax":  90, "invert": False},  # ch3 Wrist
# ]

# def angles_deg_to_pwm_with_calibration(theta_deg):
#     pwm_out = []
#     for ang, cfg in zip(theta_deg, SERVO_CONFIG):
#         # clamp to angle limits
#         ang = float(np.clip(ang, cfg["amin"], cfg["amax"]))

#         # map angle range -> pwm range
#         if not cfg["invert"]:
#             pwm = cfg["min"] + (ang - cfg["amin"]) * (cfg["max"] - cfg["min"]) / (cfg["amax"] - cfg["amin"])
#         else:
#             # inverted mapping
#             pwm = cfg["max"] - (ang - cfg["amin"]) * (cfg["max"] - cfg["min"]) / (cfg["amax"] - cfg["amin"])

#         pwm_out.append(int(np.clip(pwm, cfg["min"], cfg["max"])))

#     return pwm_out


# # =============================================================================
# # 7) MAIN: COORDINATES -> IK -> PWM -> SEND TO ARM
# # =============================================================================
# def move_arm_to_xyz(controller: SSC32U, x_cm, y_cm, z_cm, time_ms=2000):
#     target = np.array([x_cm, y_cm, z_cm], dtype=float)

#     print(f"\n🎯 Target: ({x_cm:.1f}, {y_cm:.1f}, {z_cm:.1f}) cm")

#     # quick reach check
#     horizontal_dist = np.sqrt(x_cm**2 + y_cm**2)
#     vertical_dist = abs(z_cm - BASE_HEIGHT)
#     total_dist = np.sqrt(horizontal_dist**2 + vertical_dist**2)
#     max_reach = SHOULDER_LENGTH + ELBOW_LENGTH + WRIST_LENGTH

#     if total_dist > max_reach or total_dist < 5.0:
#         print(f"❌ Not reachable (distance {total_dist:.1f} cm, max {max_reach:.1f} cm)")
#         return False

#     theta, error = solve_ik(target)
#     if theta is None:
#         print(f"❌ IK failed. error={error:.2f} cm")
#         return False

#     angles_deg = np.degrees(theta)
#     pwm = angles_deg_to_pwm_with_calibration(angles_deg)

#     print("✅ IK ok | error:", round(error, 3), "cm")
#     print("📐 Angles (deg):", [round(a, 2) for a in angles_deg])
#     print("📡 PWM:", pwm)

#     # SEND ONE COMMAND -> all joints move together
#     controller.move_servos(pwm, channels=(0, 1, 2, 3), time_ms=time_ms)

#     # wait until motion done (little extra)
#     time.sleep(time_ms / 1000.0 + 0.2)
#     return True


# def go_home(controller: SSC32U, time_ms=2000):
#     home_pwm = [1500, 1500, 1500, 1500]
#     print("\n🏠 HOME:", home_pwm)
#     controller.move_servos(home_pwm, channels=(0, 1, 2, 3), time_ms=time_ms)
#     time.sleep(time_ms / 1000.0 + 0.2)


# # =============================================================================
# # 8) RUN
# # =============================================================================
# if __name__ == "__main__":
#     controller = SSC32U(port="COM7", baudrate=9600)

#     try:
#         # Step 0: Home
#         go_home(controller, time_ms=2000)

#         # Example moves (replace with your coordinates)
#         move_arm_to_xyz(controller, 20, 0, 15, time_ms=2000)
#         move_arm_to_xyz(controller, 15, 15, 10, time_ms=2000)

#         print("\n✅ Done")

#     finally:
#         controller.close()

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