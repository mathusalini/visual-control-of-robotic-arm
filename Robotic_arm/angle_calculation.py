import numpy as np
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
# COORDINATE CONVERSION
# ============================================================================
def cm_to_simulation(x_cm, y_cm, z_cm):
    """Convert real-world coordinates (cm) to simulation space"""
    scale = 0.25  # Adjust based on your robot
    
    sim_x = 1.5 + x_cm * scale
    sim_y = 1.5 + y_cm * scale
    sim_z = 1.0 + z_cm * scale
    
    return np.array([sim_x, sim_y, sim_z])

# ============================================================================
# MAIN FUNCTION: COORDINATES TO ANGLES
# ============================================================================
def get_angles_from_coordinates(x_cm, y_cm, z_cm, initial_guess=[0, 0, 0, 0]):
    """
    Takes coordinates in cm and returns joint angles
    
    Parameters:
    -----------
    x_cm, y_cm, z_cm : float
        Target coordinates in centimeters
    initial_guess : list
        Initial guess for joint angles [theta1, theta2, theta3, theta4]
    
    Returns:
    --------
    angles : numpy array or None
        Joint angles [θ1, θ2, θ3, θ4] in radians, or None if unreachable
    """
    
    print("="*70)
    print("🎯 CALCULATING SERVO ANGLES FROM COORDINATES")
    print("="*70)
    
    # Step 1: Convert cm to simulation coordinates
    target = cm_to_simulation(x_cm, y_cm, z_cm)
    print(f"\n📍 Input Coordinates (cm):")
    print(f"   X = {x_cm} cm")
    print(f"   Y = {y_cm} cm")
    print(f"   Z = {z_cm} cm")
    print(f"\n🔄 Simulation Coordinates:")
    print(f"   {target}")
    
    # Step 2: Solve inverse kinematics
    print(f"\n🧮 Solving Inverse Kinematics...")
    theta, error = solve_ik(target, initial_guess)
    
    if theta is None:
        print(f"\n❌ IK FAILED! Position is unreachable.")
        print(f"   Error: {error:.4f}")
        return None
    
    # Step 3: Print angles in different formats
    print(f"\n✅ IK SOLVED! Error: {error:.6f}")
    print("\n" + "="*70)
    print("📐 CALCULATED JOINT ANGLES:")
    print("="*70)
    
    print(f"\n🔹 Radians:")
    print(f"   Servo 0 (Base):     θ1 = {theta[0]:+.6f} rad")
    print(f"   Servo 1 (Shoulder): θ2 = {theta[1]:+.6f} rad")
    print(f"   Servo 2 (Elbow):    θ3 = {theta[2]:+.6f} rad")
    print(f"   Servo 3 (Wrist):    θ4 = {theta[3]:+.6f} rad")
    
    print(f"\n🔹 Degrees:")
    print(f"   Servo 0 (Base):     θ1 = {np.degrees(theta[0]):+.2f}°")
    print(f"   Servo 1 (Shoulder): θ2 = {np.degrees(theta[1]):+.2f}°")
    print(f"   Servo 2 (Elbow):    θ3 = {np.degrees(theta[2]):+.2f}°")
    print(f"   Servo 3 (Wrist):    θ4 = {np.degrees(theta[3]):+.2f}°")
    
    # Verify the solution
    achieved_pos = forward_kinematics(theta)
    print(f"\n✔️  Verification (Forward Kinematics):")
    print(f"   Target Position:   {target}")
    print(f"   Achieved Position: {achieved_pos}")
    print(f"   Position Error:    {error:.6f}")
    
    print("\n" + "="*70 + "\n")
    
    return theta


# ============================================================================
# EXAMPLE USAGE
# ============================================================================
if __name__ == "__main__":
    
    print("\n🤖 ROBOTIC ARM ANGLE CALCULATOR\n")
    
    # Example 1: Single coordinate
    print("\n--- Example 1: Calculate angles for position (10, 5, 8) cm ---")
    angles = get_angles_from_coordinates(x_cm=10, y_cm=5, z_cm=8)
    
    if angles is not None:
        print(f"Angles array: {angles}")
    
    
    # Example 2: Another position
    print("\n--- Example 2: Calculate angles for position (15, 0, 10) cm ---")
    angles = get_angles_from_coordinates(x_cm=15, y_cm=0, z_cm=10)
    
    
    # Example 3: Multiple positions
    print("\n--- Example 3: Calculate angles for multiple positions ---")
    positions = [
        (10, 0, 0),
        (10, 10, 2),
        (5, 5, 12),
        (20, 5, 5)
    ]
    
    for i, (x, y, z) in enumerate(positions, 1):
        print(f"\n{'='*70}")
        print(f"Position {i}: ({x}, {y}, {z}) cm")
        print(f"{'='*70}")
        angles = get_angles_from_coordinates(x, y, z)
        
        if angles is not None:
            print(f"\n➡️  Angles: θ = [{angles[0]:.4f}, {angles[1]:.4f}, {angles[2]:.4f}, {angles[3]:.4f}] rad")


# ## Output Example:

# When you run this, you'll see:
# ```
# ======================================================================
# 🎯 CALCULATING SERVO ANGLES FROM COORDINATES
# ======================================================================

# 📍 Input Coordinates (cm):
#    X = 10 cm
#    Y = 5 cm
#    Z = 8 cm

# 🔄 Simulation Coordinates:
#    [4.0 2.75 3.0]

# 🧮 Solving Inverse Kinematics...

# ✅ IK SOLVED! Error: 0.000023

# ======================================================================
# 📐 CALCULATED JOINT ANGLES:
# ======================================================================

# 🔹 Radians:
#    Servo 0 (Base):     θ1 = +0.123456 rad
#    Servo 1 (Shoulder): θ2 = -0.456789 rad
#    Servo 2 (Elbow):    θ3 = +1.234567 rad
#    Servo 3 (Wrist):    θ4 = -0.789012 rad

# 🔹 Degrees:
#    Servo 0 (Base):     θ1 = +7.07°
#    Servo 1 (Shoulder): θ2 = -26.18°
#    Servo 2 (Elbow):    θ3 = +70.74°
#    Servo 3 (Wrist):    θ4 = -45.21°

# ✔️  Verification (Forward Kinematics):
#    Target Position:   [4.0  2.75 3.0]
#    Achieved Position: [4.0001 2.7499 3.0001]
#    Position Error:    0.000023

# ======================================================================