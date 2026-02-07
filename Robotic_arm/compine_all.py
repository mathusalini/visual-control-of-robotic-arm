import numpy as np
import serial
import time
from scipy.optimize import minimize

# ============================================================================
# ACTUAL AL5D ARM DIMENSIONS (from manual)
# ============================================================================
# All measurements in CENTIMETERS based on the AL5D specifications:
# - Shoulder to elbow: 5.75" = 14.605 cm
# - Elbow to wrist: 7.375" = 18.7325 cm
# - Wrist to gripper tip: 3.375" = 8.5725 cm
# - Base height: approximately 6.7 cm

BASE_HEIGHT = 6.7          # Height of base rotation point above table (cm)
SHOULDER_LENGTH = 14.605   # Shoulder to elbow length (cm)
ELBOW_LENGTH = 18.7325     # Elbow to wrist length (cm)
WRIST_LENGTH = 8.5725      # Wrist to gripper tip length (cm)

# ============================================================================
# TRANSFORMATION UTILITIES
# ============================================================================
def translate(x, y, z):
    """Create translation matrix"""
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T

def rotate_z(theta):
    """Rotation around Z-axis (base rotation)"""
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0:2, 0:2] = [[c, -s], [s, c]]
    return R

def rotate_y(theta):
    """Rotation around Y-axis (shoulder, elbow, wrist)"""
    R = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    R[0, 0], R[0, 2], R[2, 0], R[2, 2] = c, s, -s, c
    return R

# ============================================================================
# FORWARD KINEMATICS (Using Real Dimensions)
# ============================================================================
def forward_kinematics(theta):
    """
    Calculate end-effector position from joint angles
    
    theta = [theta1, theta2, theta3, theta4]
    theta1: Base rotation (around Z-axis)
    theta2: Shoulder angle (around Y-axis)
    theta3: Elbow angle (around Y-axis)
    theta4: Wrist angle (around Y-axis)
    
    Returns: [x, y, z] position in cm relative to base center at table level
    """
    theta1, theta2, theta3, theta4 = theta
    
    # Start at base (origin is at base center, table level)
    T_base = translate(0, 0, BASE_HEIGHT)
    
    # Base rotation
    T1 = T_base @ rotate_z(theta1)
    
    # Shoulder joint
    T2 = T1 @ rotate_y(theta2)
    
    # Elbow joint (translate by shoulder length first)
    T3 = T2 @ translate(SHOULDER_LENGTH, 0, 0) @ rotate_y(theta3)
    
    # Wrist joint (translate by elbow length)
    T4 = T3 @ translate(ELBOW_LENGTH, 0, 0) @ rotate_y(theta4)
    
    # End effector (translate by wrist length)
    T_end = T4 @ translate(WRIST_LENGTH, 0, 0)
    
    # Extract position
    end_effector_pos = T_end[:3, 3]
    
    return end_effector_pos

# ============================================================================
# INVERSE KINEMATICS
# ============================================================================
def ik_objective(theta, target):
    """Objective function for IK optimization"""
    eff_pos = forward_kinematics(theta)
    return np.linalg.norm(eff_pos - target)

def solve_ik(target, initial_guess=[0, -np.pi/4, np.pi/4, 0]):
    """
    Solve inverse kinematics with joint constraints
    
    Joint limits based on AL5D specifications:
    - Base: ±90° (180° total range)
    - Shoulder: typically 0° to -120° (pointing down to forward)
    - Elbow: ±90° 
    - Wrist: ±90°
    """
    bounds = [
        (-np.pi/2, np.pi/2),      # Base: ±90°
        (-2*np.pi/3, np.pi/6),    # Shoulder: -120° to +30°
        (-np.pi/2, np.pi/2),      # Elbow: ±90°
        (-np.pi/2, np.pi/2)       # Wrist: ±90°
    ]
    
    result = minimize(
        ik_objective,
        initial_guess,
        args=(target,),
        method='L-BFGS-B',
        bounds=bounds,
        options={'maxiter': 2000, 'disp': False}
    )
    
    theta = result.x
    final_pos = forward_kinematics(theta)
    error = np.linalg.norm(final_pos - target)
    
    if error < 0.5:  # 5mm tolerance
        return theta, error
    else:
        return None, error

# ============================================================================
# SERVO PULSE WIDTH CONVERSION
# ============================================================================
def radians_to_pulse(angle_rad, center=1500, scale=500):
    """
    Convert radians to servo pulse width (microseconds)
    
    Standard RC servo:
    - 1500μs = center (0°)
    - 1000μs = -90° (-π/2 rad)
    - 2000μs = +90° (+π/2 rad)
    - Scale: ~500μs per 90° (π/2 rad)
    """
    pulse = center + int(angle_rad * (scale / (np.pi / 2)))
    return np.clip(pulse, 500, 2500)

# Servo calibration offsets (adjust if servos don't center at 1500μs)
SERVO_OFFSETS = {
    0: 0,      # Base
    1: 0,      # Shoulder
    2: 0,      # Elbow
    3: 0,      # Wrist
    4: 0       # Gripper
}

def theta_to_pulses(theta):
    """Convert joint angles to SSC-32U pulse widths"""
    pulses = []
    for i, angle in enumerate(theta):
        pulse = radians_to_pulse(angle, center=1500) + SERVO_OFFSETS[i]
        pulses.append(int(pulse))
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
            self.list_available_ports()
            raise
    
    def list_available_ports(self):
        """List all available COM ports"""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if ports:
            print("\n📋 Available COM ports:")
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
        print(f"   Command: {cmd}")
    
    def set_gripper(self, state='open', time_ms=1000):
        """Control gripper: 'open', 'close', or 'half'"""
        gripper_positions = {
            'open': 1800,    # Fully open
            'half': 1500,    # Half open
            'close': 1200    # Closed
        }
        pulse = gripper_positions.get(state, 1500)
        self._send(f"#4 P{pulse} T{time_ms}")
        print(f"   Gripper: {state}")
    
    def home_position(self):
        """Move to home position (all servos centered)"""
        print("   Moving all servos to center (1500μs)...")
        self.move_servos([0, 1, 2, 3], [1500, 1500, 1500, 1500], 2000)
        time.sleep(2.2)
    
    def close(self):
        self.ser.close()
        print("Connection closed")

# ============================================================================
# ROBOTIC ARM CONTROLLER
# ============================================================================
class AL5DController:
    def __init__(self, port='COM7'):
        self.controller = SSC32U(port=port)
        self.current_theta = [0, -np.pi/4, np.pi/4, 0]  # Default pose
        
        # Calculate workspace based on actual arm dimensions
        max_reach = SHOULDER_LENGTH + ELBOW_LENGTH + WRIST_LENGTH  # ~41.91 cm
        self.max_reach = max_reach
        
        print(f"\n📏 AL5D Arm Specifications:")
        print(f"   Base height: {BASE_HEIGHT} cm")
        print(f"   Shoulder length: {SHOULDER_LENGTH} cm")
        print(f"   Elbow length: {ELBOW_LENGTH} cm")
        print(f"   Wrist length: {WRIST_LENGTH} cm")
        print(f"   Maximum reach: {self.max_reach:.2f} cm")
    
    def is_reachable(self, target):
        """Check if target position is reachable"""
        # Target must be within max reach from base
        x, y, z = target
        horizontal_dist = np.sqrt(x**2 + y**2)
        vertical_dist = abs(z - BASE_HEIGHT)
        total_dist = np.sqrt(horizontal_dist**2 + vertical_dist**2)
        
        # Also check minimum reach (can't be too close to base)
        min_reach = 5.0  # cm
        
        return min_reach <= total_dist <= self.max_reach
    
    def move_to_xyz(self, x_cm, y_cm, z_cm, time_ms=3000):
        """
        Move gripper to specified coordinates in cm
        
        Coordinate system:
        - Origin (0,0,0) = Base center at table level
        - X-axis: Forward
        - Y-axis: Left
        - Z-axis: Up
        """
        target = np.array([x_cm, y_cm, z_cm])
        
        print(f"\n🎯 Target: ({x_cm:.1f}, {y_cm:.1f}, {z_cm:.1f}) cm")
        
        # Check reachability
        if not self.is_reachable(target):
            dist = np.linalg.norm(target - np.array([0, 0, BASE_HEIGHT]))
            print(f"❌ Target NOT reachable! (distance: {dist:.1f} cm, max: {self.max_reach:.1f} cm)")
            return False
        
        # Solve inverse kinematics
        theta, error = solve_ik(target, self.current_theta)
        
        if theta is None:
            print(f"❌ IK failed! Error: {error:.2f} cm")
            return False
        
        print(f"✅ IK solved! Error: {error:.3f} cm")
        print(f"   Joint angles (deg): [{np.degrees(theta[0]):.1f}, {np.degrees(theta[1]):.1f}, {np.degrees(theta[2]):.1f}, {np.degrees(theta[3]):.1f}]")
        
        # Convert to pulse widths
        pulses = theta_to_pulses(theta)
        print(f"   Servo pulses: {pulses}")
        
        # Send to hardware
        self.controller.move_servos([0, 1, 2, 3], pulses, time_ms)
        
        # Update current state
        self.current_theta = theta
        
        return True
    
    def pick_and_place(self, pickup_xyz, place_xyz, approach_height=8):
        """
        Complete pick-and-place operation
        
        pickup_xyz: (x, y, z) in cm - object location
        place_xyz: (x, y, z) in cm - target location
        approach_height: cm above object for approach
        """
        x_pick, y_pick, z_pick = pickup_xyz
        x_place, y_place, z_place = place_xyz
        
        print("\n" + "="*70)
        print("🤖 PICK-AND-PLACE OPERATION")
        print("="*70)
        
        # Open gripper
        print("\n🤲 Opening gripper...")
        self.controller.set_gripper('open', 1000)
        time.sleep(1.2)
        
        # Step 1: Move above pickup
        print(f"\n📍 Step 1: Moving above pickup location...")
        if not self.move_to_xyz(x_pick, y_pick, z_pick + approach_height, 3000):
            print("❌ Failed to reach pickup approach position")
            return False
        time.sleep(3.2)
        
        # Step 2: Lower to pickup
        print(f"\n⬇️  Step 2: Lowering to object...")
        if not self.move_to_xyz(x_pick, y_pick, z_pick, 2000):
            print("❌ Failed to reach pickup position")
            return False
        time.sleep(2.2)
        
        # Step 3: Close gripper
        print(f"\n🤏 Step 3: Closing gripper...")
        self.controller.set_gripper('close', 1000)
        time.sleep(1.5)
        
        # Step 4: Lift object
        print(f"\n⬆️  Step 4: Lifting object...")
        if not self.move_to_xyz(x_pick, y_pick, z_pick + approach_height, 2000):
            print("❌ Failed to lift object")
            return False
        time.sleep(2.2)
        
        # Step 5: Move above place location
        print(f"\n📍 Step 5: Moving to place location...")
        if not self.move_to_xyz(x_place, y_place, z_place + approach_height, 3000):
            print("❌ Failed to reach place approach position")
            return False
        time.sleep(3.2)
        
        # Step 6: Lower to place
        print(f"\n⬇️  Step 6: Lowering to place position...")
        if not self.move_to_xyz(x_place, y_place, z_place, 2000):
            print("❌ Failed to reach place position")
            return False
        time.sleep(2.2)
        
        # Step 7: Open gripper
        print(f"\n🤲 Step 7: Releasing object...")
        self.controller.set_gripper('open', 1000)
        time.sleep(1.2)
        
        # Step 8: Retract
        print(f"\n⬆️  Step 8: Retracting...")
        if not self.move_to_xyz(x_place, y_place, z_place + approach_height, 2000):
            print("❌ Failed to retract")
            return False
        time.sleep(2.2)
        
        print("\n✅ PICK-AND-PLACE COMPLETE!")
        print("="*70)
        return True
    
    def find_zero_position(self):
        """Move to (0,0,0) to mark the origin"""
        print("\n" + "="*70)
        print("🎯 FINDING ZERO POSITION")
        print("="*70)
        print("\nCoordinate System:")
        print("  • (0,0,0) = Base center at table level")
        print("  • +X = Forward")
        print("  • +Y = Left")
        print("  • +Z = Upward")
        
        # (0,0,0) is at table level, which might be unreachable
        # Try (0,0,10) instead - 10cm above table
        print("\nNote: (0,0,0) is at table level and may be unreachable.")
        print("      Moving to (0,0,10) - 10cm above origin instead.\n")
        
        input("Press ENTER to move to position...")
        
        success = self.move_to_xyz(0, 0, 10, 3000)
        
        if success:
            time.sleep(3.5)
            print("\n✅ Gripper is at (0, 0, 10) cm")
            print("   This is 10cm directly above the base center.")
            print("\n📍 Mark this position as your reference point!")
        else:
            print("\n❌ Could not reach (0,0,10)")
        
        return success
    
    def close(self):
        self.controller.close()

# ============================================================================
# MAIN PROGRAM
# ============================================================================
if __name__ == "__main__":
    print("="*70)
    print("🤖 AL5D ROBOTIC ARM CONTROLLER (Real Dimensions)")
    print("="*70)
    
    # List available ports
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if ports:
        print("\n📋 Available COM ports:")
        for port in ports:
            print(f"   - {port.device}: {port.description}")
    
    # Get COM port
    port_input = input("\nEnter COM port (default: COM7): ").strip()
    com_port = port_input if port_input else "COM7"
    
    try:
        # Initialize
        arm = AL5DController(port=com_port)
        
        # Menu
        print("\n" + "="*70)
        print("OPTIONS:")
        print("  1. Find zero position (0,0,10)")
        print("  2. Manual positioning")
        print("  3. Pick and place demo")
        print("  4. Home position only")
        print("="*70)
        
        choice = input("\nEnter choice (1-4): ").strip()
        
        # Home position
        print("\n🏠 Moving to home position...")
        arm.controller.home_position()
        
        if choice == '1':
            arm.find_zero_position()
            
        elif choice == '2':
            print("\n🕹️  Manual Positioning Mode")
            print("Enter coordinates as: x,y,z (in cm)")
            print("Type 'quit' to exit\n")
            
            while True:
                try:
                    coords = input("Coordinates (x,y,z): ").strip()
                    if coords.lower() in ['quit', 'exit', 'q']:
                        break
                    
                    x, y, z = map(float, coords.replace('(', '').replace(')', '').split(','))
                    arm.move_to_xyz(x, y, z, 3000)
                    time.sleep(3.5)
                    
                except ValueError:
                    print("❌ Invalid format. Use: x,y,z")
                except KeyboardInterrupt:
                    break
        
        elif choice == '3':
            print("\n🎯 Pick and Place Demo")
            pickup = (20, 0, 10)    # 20cm forward, 10cm up
            place = (15, 15, 10)    # 15cm forward, 15cm left, 10cm up
            
            arm.pick_and_place(pickup, place, approach_height=8)
        
        elif choice == '4':
            print("✅ At home position")
        
        # Return home
        print("\n🏠 Returning to home...")
        arm.controller.home_position()
        
        arm.close()
        print("\n✅ Program complete!")
        
    except serial.SerialException as e:
        print(f"\n❌ Serial error: {e}")
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        try:
            arm.close()
        except:
            pass
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()