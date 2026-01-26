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
# ARM CONFIGURATION
# ============================================================================
link1_size = [0.1, 0.2, 0.2]
link2_size = [2.0, 0.2, 0.2]
link3_size = [1.5, 0.2, 0.2]
link4_size = [1.0, 0.2, 0.2]

# ============================================================================
# FORWARD KINEMATICS
# ============================================================================
def forward_kinematics(theta):
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
    eff_pos = forward_kinematics(theta)
    return np.linalg.norm(eff_pos - target)

def solve_ik(target, initial_guess):
    bounds = [
        (-np.pi, np.pi),
        (-2*np.pi/3, 0),
        (-np.pi, np.pi),
        (-5*np.pi/6, 5*np.pi/6)
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
# SERVO CONVERSION
# ============================================================================
def radians_to_pulse(angle_rad, center=1500, scale=500):
    pulse = center + int(angle_rad * (scale / (np.pi / 2)))
    return np.clip(pulse, 500, 2500)

SERVO_OFFSETS = {0: 0, 1: 0, 2: 0, 3: 0}

def theta_to_pulses(theta):
    pulses = []
    for i, angle in enumerate(theta):
        pulse = radians_to_pulse(angle) + SERVO_OFFSETS[i]
        pulses.append(pulse)
    return pulses

# ============================================================================
# SSC-32U CONTROLLER
# ============================================================================
class SSC32U:
    def __init__(self, port='COM7', baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)
        print(f"✅ Connected to SSC-32U on {port}")
    
    def _send(self, cmd: str):
        self.ser.write((cmd + "\r").encode("ascii"))
        self.ser.flush()
    
    def move_servos(self, channels, pulses, time_ms=2000):
        cmd = ""
        for ch, pw in zip(channels, pulses):
            cmd += f"#{ch} P{pw} "
        cmd += f"T{time_ms}"
        self._send(cmd)
        print(f"   Sent: {cmd}")
    
    def home_position(self):
        self.move_servos([0, 1, 2, 3], [1500, 1500, 1500, 1500], 2000)
        time.sleep(2.2)
    
    def close(self):
        self.ser.close()

# ============================================================================
# COORDINATE CONVERSION
# ============================================================================
def cm_to_simulation(x_cm, y_cm, z_cm):
    """Convert real-world cm to simulation coordinates"""
    scale = 0.25
    sim_x = 1.5 + x_cm * scale
    sim_y = 1.5 + y_cm * scale
    sim_z = 1.0 + z_cm * scale
    return np.array([sim_x, sim_y, sim_z])

# ============================================================================
# MAIN PROGRAM - FIND ZERO POSITION
# ============================================================================
if __name__ == "__main__":
    print("="*70)
    print("🎯 FINDING ZERO POSITION (0, 0, 0)")
    print("="*70)
    print("\nThis program will move the gripper to the (0,0,0) reference position.")
    print("\nCoordinate System:")
    print("  • X = 0: Base center (forward/backward)")
    print("  • Y = 0: Base center (left/right)")
    print("  • Z = 0: Table surface level")
    print("\n⚠️  Make sure the arm has clear space to move!")
    print("="*70)
    
    # Find available COM ports
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if ports:
        print("\n📋 Available COM ports:")
        for port in ports:
            print(f"   - {port.device}: {port.description}")
    
    # Get COM port from user
    port_input = input("\nEnter COM port (e.g., COM7) or press Enter for COM7: ").strip()
    com_port = port_input if port_input else "COM7"
    
    try:
        # Connect to robot
        print(f"\n🔌 Connecting to {com_port}...")
        controller = SSC32U(port=com_port)
        
        # Move to home position first
        print("\n🏠 Step 1: Moving to HOME position (all servos centered)...")
        controller.home_position()
        print("   ✅ Home position reached\n")
        
        input("Press ENTER to move to (0,0,0) position...")
        
        # Calculate target position
        print("\n🎯 Step 2: Moving to (0, 0, 0) position...")
        target = cm_to_simulation(0, 0, 0)
        print(f"   Target in simulation space: {target}")
        
        # Solve IK
        print("   Solving inverse kinematics...")
        initial_guess = [0, 0, 0, 0]
        theta, error = solve_ik(target, initial_guess)
        
        if theta is None:
            print(f"\n❌ FAILED: Could not reach (0,0,0)")
            print(f"   Error: {error:.4f}")
            print("\n💡 Troubleshooting:")
            print("   1. Try (0,0,5) instead - 5cm above ground")
            print("   2. Adjust the 'scale' factor in cm_to_simulation()")
            print("   3. The position might be too close to the base")
        else:
            print(f"   ✅ IK solved! Error: {error:.6f}")
            print(f"   Joint angles (radians): {np.round(theta, 3)}")
            print(f"   Joint angles (degrees): {np.round(np.degrees(theta), 1)}")
            
            # Convert to pulses
            pulses = theta_to_pulses(theta)
            print(f"   Servo pulses: {pulses}")
            
            # Move robot
            print("\n   Moving robot...")
            controller.move_servos([0, 1, 2, 3], pulses, time_ms=3000)
            time.sleep(3.5)
            
            print("\n" + "="*70)
            print("✅ GRIPPER IS NOW AT (0, 0, 0)")
            print("="*70)
            print("\n📍 MARK THIS POSITION!")
            print("   • Place a piece of tape or marker at the gripper tip")
            print("   • This is your origin/reference point")
            print("   • All future coordinates will be relative to this spot")
            print("\nCoordinate directions from this point:")
            print("   • +X = Forward (away from base)")
            print("   • +Y = Left")
            print("   • +Z = Upward")
            print("   • -X = Backward (toward base)")
            print("   • -Y = Right")
            print("   • -Z = Downward")
            print("="*70)
            
            input("\nPress ENTER to return to home position...")
        
        # Return home
        print("\n🏠 Returning to home position...")
        controller.home_position()
        print("   ✅ Home position reached")
        
        # Close connection
        controller.close()
        print("\n✅ Program complete!")
        
    except serial.SerialException as e:
        print(f"\n❌ Serial connection error: {e}")
        print("\n🔧 Troubleshooting:")
        print("   1. Make sure the SSC-32U is plugged in")
        print("   2. Close other programs using the serial port")
        print("   3. Check the COM port number in Device Manager")
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        try:
            controller.close()
        except:
            pass
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()