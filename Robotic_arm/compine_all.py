import numpy as np
import serial
import time
from scipy.optimize import minimize

# ============================================================================
# TRANSFORMATION UTILITIES
# ============================================================================
def translate(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def rotate_z(theta):
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0:2, 0:2] = [[c, -s], [s, c]]
    return R

def rotate_y(theta):
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, s, -s, c
    return R

# ============================================================================
# ARM CONFIGURATION (units in simulation space)
# ============================================================================
link1_size = [0.1, 0.2, 0.2]  # Base rotation link
link2_size = [2.0, 0.2, 0.2]  # Shoulder to elbow: 5.75" ≈ 14.6cm
link3_size = [1.5, 0.2, 0.2]  # Elbow to wrist: 7.375" ≈ 18.7cm
link4_size = [1.0, 0.2, 0.2]  # Wrist to gripper tip: 3.375" ≈ 8.6cm

# ============================================================================
# FORWARD KINEMATICS
# ============================================================================
def forward_kinematics(theta):
    """Calculate end-effector position from joint angles"""
    theta1, theta2, theta3, theta4 = theta
    T_base = translate(1.5, 1.5, 1)
    T1 = T_base @ rotate_z(theta1)
    T2 = T1 @ translate(link1_size[0], 0, 0) @ rotate_y(theta2)
    T3 = T2 @ translate(link2_size[0], 0, 0) @ rotate_y(theta3)
    T4 = T3 @ translate(link3_size[0], 0, 0) @ rotate_y(theta4)
    end_effector_pos = (T4 @ np.array([link4_size[0], 0, 0, 1]))[:3]
    return end_effector_pos

# ============================================================================
# INVERSE KINEMATICS
# ============================================================================
def ik_objective(theta, target):
    """Objective function for IK optimization"""
    eff_pos = forward_kinematics(theta)
    return np.linalg.norm(eff_pos - target)

def solve_ik(target, initial_guess):
    """Solve inverse kinematics with joint constraints"""
    bounds = [
        (-np.pi, np.pi),           # Base: full rotation
        (-2*np.pi/3, 0),           # Shoulder: -120° to 0°
        (-np.pi, np.pi),           # Elbow: full range
        (-5*np.pi/6, 5*np.pi/6)    # Wrist: -150° to 150°
    ]
    
    result = minimize(
        ik_objective,
        initial_guess,
        args=(target,),
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 1000, 'disp': False}
    )
    
    theta = result.x
    final_pos = forward_kinematics(theta)
    error = np.linalg.norm(final_pos - target)
    
    if error < 1e-2:
        return theta, error
    else:
        return None, error

# ============================================================================
# SERVO PULSE WIDTH CONVERSION
# ============================================================================
def radians_to_pulse(angle_rad, center=1500, scale=500):
    """
    Convert radians to servo pulse width (microseconds)
    center: neutral position (typically 1500μs)
    scale: μs per radian (500μs ≈ 90° = π/2 rad)
    """
    pulse = center + int(angle_rad * (scale / (np.pi / 2)))
    return np.clip(pulse, 500, 2500)  # Safety limits

# Servo-specific calibration offsets (adjust based on your hardware)
SERVO_OFFSETS = {
    0: 0,      # Base
    1: 0,      # Shoulder
    2: 0,      # Elbow
    3: 0,      # Wrist
    4: 1500    # Gripper (open/close, not angle-based)
}

def theta_to_pulses(theta):
    """Convert joint angles to SSC-32U pulse widths"""
    pulses = []
    for i, angle in enumerate(theta):
        pulse = radians_to_pulse(angle) + SERVO_OFFSETS[i]
        pulses.append(pulse)
    return pulses

# ============================================================================
# SSC-32U HARDWARE CONTROLLER
# ============================================================================
class SSC32U:
    def __init__(self, port='COM7', baudrate=9600):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            print(f"✅ Connected to SSC-32U on {port}")
        except serial.SerialException as e:
            print(f"\n❌ ERROR: Could not open {port}")
            print(f"   {str(e)}")
            print("\n🔧 TROUBLESHOOTING:")
            print("   1. Close any programs using the port (Arduino IDE, other Python scripts, SSC-32 Sequencer)")
            print("   2. Check Device Manager for the correct COM port")
            print("   3. Unplug and replug the USB cable")
            print("   4. Try a different COM port\n")
            self.list_available_ports()
            raise
    
    def list_available_ports(self):
        """List all available COM ports"""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if ports:
            print("📋 Available COM ports:")
            for port in ports:
                print(f"   - {port.device}: {port.description}")
        else:
            print("   No COM ports found!")
    
    def _send(self, cmd: str):
        """Send command to SSC-32U"""
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
    
    def move_servos(self, channels, pulses, time_ms=2000):
        """Move multiple servos simultaneously"""
        cmd = ""
        for ch, pw in zip(channels, pulses):
            cmd += f"#{ch} P{pw} "
        cmd += f"T{time_ms}"
        self._send(cmd)
        print(f"Sent: {cmd}")
    
    def set_gripper(self, state='close', time_ms=1000):
        """Control gripper: 'open', 'close', or 'half'"""
        gripper_positions = {
            'open': 2000,
            'half': 1500,
            'close': 1000
        }
        pulse = gripper_positions.get(state, 1500)
        self._send(f"#4 P{pulse} T{time_ms}")
        print(f"Gripper: {state}")
    
    def home_position(self):
        """Move to home position (all servos centered)"""
        self.move_servos([0, 1, 2, 3], [1500, 1500, 1500, 1500], 2000)
        time.sleep(2.2)
    
    def close(self):
        self.ser.close()
        print("Connection closed")

# ============================================================================
# COORDINATE SYSTEM CONVERSION
# ============================================================================
def cm_to_simulation(x_cm, y_cm, z_cm):
    """
    Convert real-world coordinates (cm) to simulation space
    Adjust scale factor based on your calibration
    """
    # Base is at (1.5, 1.5, 1.0) in simulation
    # Typical scale: 1 simulation unit ≈ 3-5 cm (adjust as needed)
    scale = 0.25  # ADJUST THIS BASED ON YOUR ROBOT
    
    sim_x = 1.5 + x_cm * scale
    sim_y = 1.5 + y_cm * scale
    sim_z = 1.0 + z_cm * scale
    
    return np.array([sim_x, sim_y, sim_z])

# ============================================================================
# MAIN PICK-AND-PLACE CONTROLLER
# ============================================================================
class RoboticArmController:
    def __init__(self, port='COM7'):
        self.controller = SSC32U(port=port)
        self.current_theta = [0, 0, 0, 0]  # Current joint angles
        self.workspace_center = np.array([1.5, 1.5, 1.0])
        self.max_reach = 4.4
    
    def is_reachable(self, target):
        """Check if target is within workspace"""
        dist = np.linalg.norm(target - self.workspace_center)
        return dist <= self.max_reach
    
    def move_to_coordinate(self, x_cm, y_cm, z_cm, time_ms=3000):
        """Move gripper to specified coordinates (in cm)"""
        # Convert to simulation coordinates
        target = cm_to_simulation(x_cm, y_cm, z_cm)
        
        print(f"\n🎯 Target: ({x_cm}, {y_cm}, {z_cm}) cm")
        print(f"   Simulation coords: {target}")
        
        # Check reachability
        if not self.is_reachable(target):
            print("❌ Target is NOT reachable!")
            return False
        
        # Solve inverse kinematics
        theta, error = solve_ik(target, self.current_theta)
        
        if theta is None:
            print(f"❌ IK failed! Error: {error:.4f}")
            return False
        
        print(f"✅ IK solved! Error: {error:.6f}")
        print(f"   Joint angles (rad): {theta}")
        
        # Convert to pulse widths
        pulses = theta_to_pulses(theta)
        print(f"   Servo pulses: {pulses}")
        
        # Send to hardware
        self.controller.move_servos([0, 1, 2, 3], pulses, time_ms)
        
        # Update current state
        self.current_theta = theta
        
        return True
    
    def pick_and_place(self, pickup_cm, place_cm, height_offset=5):
        """
        Complete pick-and-place operation
        pickup_cm: (x, y, z) in cm - object location
        place_cm: (x, y, z) in cm - target location
        height_offset: cm to lift above object before approach
        """
        x_pick, y_pick, z_pick = pickup_cm
        x_place, y_place, z_place = place_cm
        
        print("\n" + "="*60)
        print("🤖 STARTING PICK-AND-PLACE OPERATION")
        print("="*60)
        
        # Step 1: Move above pickup location
        print("\n📍 Step 1: Moving above pickup location...")
        self.controller.set_gripper('open', 1000)
        time.sleep(1.2)
        
        if not self.move_to_coordinate(x_pick, y_pick, z_pick + height_offset, 3000):
            return False
        time.sleep(3.2)
        
        # Step 2: Lower to pickup location
        print("\n📍 Step 2: Lowering to object...")
        if not self.move_to_coordinate(x_pick, y_pick, z_pick, 2000):
            return False
        time.sleep(2.2)
        
        # Step 3: Close gripper
        print("\n🤏 Step 3: Closing gripper...")
        self.controller.set_gripper('close', 1000)
        time.sleep(1.5)
        
        # Step 4: Lift object
        print("\n⬆️  Step 4: Lifting object...")
        if not self.move_to_coordinate(x_pick, y_pick, z_pick + height_offset, 2000):
            return False
        time.sleep(2.2)
        
        # Step 5: Move above place location
        print("\n📍 Step 5: Moving to place location...")
        if not self.move_to_coordinate(x_place, y_place, z_place + height_offset, 3000):
            return False
        time.sleep(3.2)
        
        # Step 6: Lower to place location
        print("\n📍 Step 6: Lowering to place location...")
        if not self.move_to_coordinate(x_place, y_place, z_place, 2000):
            return False
        time.sleep(2.2)
        
        # Step 7: Open gripper
        print("\n🤲 Step 7: Releasing object...")
        self.controller.set_gripper('open', 1000)
        time.sleep(1.2)
        
        # Step 8: Retract
        print("\n⬆️  Step 8: Retracting...")
        if not self.move_to_coordinate(x_place, y_place, z_place + height_offset, 2000):
            return False
        time.sleep(2.2)
        
        print("\n✅ PICK-AND-PLACE COMPLETE!")
        print("="*60 + "\n")
        return True
    
    def close(self):
        self.controller.close()

# ============================================================================
# EXAMPLE USAGE
# ============================================================================
if __name__ == "__main__":
    # Initialize controller
    arm = RoboticArmController(port="COM7")
    
    try:
        # Home position first
        print("Moving to home position...")
        arm.controller.home_position()
        
        # Example 1: Move to single coordinate
        print("\n--- Example 1: Move to coordinate ---")
        arm.move_to_coordinate(x_cm=10, y_cm=5, z_cm=8, time_ms=3000)
        time.sleep(3.5)
        
        # Example 2: Pick and place operation
        print("\n--- Example 2: Pick and Place ---")
        pickup_location = (10, 0, 0)   # (x, y, z) in cm
        place_location = (10, 10, 2)   # (x, y, z) in cm
        
        arm.pick_and_place(
            pickup_cm=pickup_location,
            place_cm=place_location,
            height_offset=5  # Lift 5cm above object
        )
        
        # Return to home
        print("\nReturning to home...")
        arm.controller.home_position()
        
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        arm.close()