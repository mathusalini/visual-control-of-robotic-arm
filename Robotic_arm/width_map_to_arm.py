import math
import time
import serial

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
    "gripper": 4,   # <-- change if your gripper is not channel 4
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

# =========================================================
# Gripper Control Based on Object Width
# =========================================================
def width_to_pwm(width_cm):
    """
    Maps the width of an object (in cm) to a PWM signal for the gripper.
    :param width_cm: Object width in cm
    :return: PWM value for gripper
    """
    # Define the width-to-PWM relationship (linear interpolation)
    if width_cm >= 3.2:
        pwm = 1090  # Max open position
    elif width_cm <= 0:
        pwm = 2500  # Fully closed
    else:
        # Linearly interpolate between 3.2 cm (1090 PWM) and 0 cm (2500 PWM)
        pwm = int(2500 - ((2500 - 1090) / (3.2 - 0)) * (width_cm - 0))
    
    # Clamp the PWM value to ensure it stays within safe limits
    pwm = max(500, min(2500, pwm))
    return pwm

def move_gripper(ctrl, width_cm):
    """
    Move the gripper based on object width.
    :param ctrl: SSC32U controller instance
    :param width_cm: Object width in cm
    """
    # Get the PWM for the given width
    pwm = width_to_pwm(width_cm)
    
    # Send the PWM command to the gripper servo
    ctrl.send(f"#{JOINTS['gripper']} P{pwm} T0")  # Adjust for the gripper channel
    
    print(f"Gripper set to {pwm} PWM for width {width_cm} cm.")

# =========================================================
# MAIN PROGRAM
# =========================================================
def main():
    arm = SSC32U("COM7")

    try:
        print("Enter the object width (in cm) to control the gripper:")
        
        # Ask user for object width
        width = float(input("Width (cm) >>> "))
        
        # Move the gripper to the calculated position based on width
        move_gripper(arm, width)

    finally:
        arm.close()

# =========================================================
if __name__ == "__main__":
    main()