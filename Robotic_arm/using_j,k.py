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
      alpha_deg = end-effector angle (deg)

    Returns (degrees):
      shoulder_final, elbow_final, wrist_final
    """

    # Convert alpha to radians
    a = math.radians(alpha_deg)

    # Wrist center
    m = j - L3 * math.cos(a)
    n = k - L3 * math.sin(a)

    # Distance
    l = math.sqrt(m*m + n*n)

    # Reachability check
    if l > (L1 + L2) or l < abs(L1 - L2):
        raise ValueError("Target not reachable")

    # θ12
    theta_12 = math.atan2(n, m)

    # θ13
    c13 = (L2*L2 - L1*L1 - l*l) / (-2 * L1 * l)
    c13 = max(-1.0, min(1.0, c13))
    theta_13 = math.acos(c13)

    # φ1
    phi1 = theta_12 + theta_13

    # φ2
    c2 = (l*l - L2*L2 - L1*L1) / (-2 * L2 * L1)
    c2 = max(-1.0, min(1.0, c2))
    phi2 = math.acos(c2)

    # θ2 (elbow)
    theta2 = phi1 + phi2 - math.pi

    # Convert to degrees
    phi1_deg = math.degrees(phi1)
    theta2_deg = math.degrees(theta2)

    # ✅ FINAL ANGLES (your definition)
    shoulder_final = phi1_deg - 90     # shoulder
    elbow_final =-theta2_deg            # elbow
    wrist_final = alpha_deg             # wrist

    # 🔹 PRINT ALL
    print(f"j={j}, k={k}, alpha={alpha_deg}°")
    print(f"phi1 (raw shoulder) : {phi1_deg:.4f}°")
    print(f"theta2 (elbow raw)  : {theta2_deg:.4f}°")
    print(f"----------------------------------")
    print(f"Shoulder (phi1-90)  : {shoulder_final:.4f}°")
    print(f"Elbow (theta2)      : {elbow_final:.4f}°")
    print(f"Wrist (alpha)       : {wrist_final:.4f}°")

    return shoulder_final, elbow_final, wrist_final


if __name__ == "__main__":
    j = 20
    k = 20
    ik_from_jk(j, k, alpha_deg=-68)