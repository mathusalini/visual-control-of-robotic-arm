import math

# Link lengths (cm)
L1 = 14.605
L2 = 18.7325
L3 = 8.5725

def ik_from_jk(j, k, alpha_deg=-68.0):
    """
    Inputs:
      j = sqrt(x^2 + y^2)  (horizontal reach, cm)
      k = z coordinate     (height, cm)
      alpha_deg = end-effector angle in degrees (like your Desmos 'a')

    Outputs (degrees):
      shoulder_phi1, elbow_theta2, wrist_phi3
    """

    # Convert alpha to radians for trig
    a = math.radians(alpha_deg)

    # Wrist-center point (m, n)
    m = j - L3 * math.cos(a)
    n = k - L3 * math.sin(a)

    # Distance from shoulder joint to wrist-center
    l = math.sqrt(m*m + n*n)

    # Reachability check (2-link)
    if l > (L1 + L2) or l < abs(L1 - L2):
        raise ValueError(f"Target not reachable: l={l:.3f}, range=[{abs(L1-L2):.3f}, {L1+L2:.3f}]")

    # θ12 = atan(n/m)  (use atan2 for correct quadrant)
    theta_12 = math.atan2(n, m)

    # θ13 = acos((L2^2 - L1^2 - l^2)/(-2*L1*l))
    c13 = (L2*L2 - L1*L1 - l*l) / (-2 * L1 * l)
    c13 = max(-1.0, min(1.0, c13))  # clamp for numeric safety
    theta_13 = math.acos(c13)

    # φ1 = θ12 + θ13
    phi1 = theta_12 + theta_13

    # φ2 = acos((l^2 - L2^2 - L1^2)/(-2*L2*L1))
    c2 = (l*l - L2*L2 - L1*L1) / (-2 * L2 * L1)
    c2 = max(-1.0, min(1.0, c2))
    phi2 = math.acos(c2)

    # θ2 = φ1 + φ2 - 180°  (your Desmos)
    theta2 = phi1 + phi2 - math.pi

    # φ3 = (180° - θ2) + α  (your Desmos)
    phi3 = (math.pi - theta2) + a

    # Convert to degrees
    shoulder_deg = math.degrees(phi1)
    elbow_deg    = math.degrees(theta2)
    wrist_deg    = math.degrees(phi3)

    return shoulder_deg, elbow_deg, wrist_deg


if __name__ == "__main__":
    j = 10
    k = 20
    shoulder, elbow, wrist = ik_from_jk(j, k, alpha_deg=-68)

    print(f"j={j}, k={k}, alpha=-68°")
    print(f"Shoulder (phi1): {shoulder:.4f}°")
    print(f"Elbow (theta2):   {elbow:.4f}°")
    print(f"Wrist (phi3):     {wrist:.4f}°")