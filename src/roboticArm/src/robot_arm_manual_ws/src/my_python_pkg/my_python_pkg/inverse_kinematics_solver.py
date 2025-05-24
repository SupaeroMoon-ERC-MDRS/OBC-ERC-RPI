#!/usr/bin/env python3
import math

def compute_joint_angles(x, y, z):
    # Longueurs en cm
    L1 = 13.64
    L2 = 17.52
    L3 = 16.089
    L4 = 21.823
    L_eff = 10.0

    x -= L_eff  # reculer à partir de la pince

    try:
        theta1 = math.atan2(y, x)

        r = math.hypot(x, y)
        dz = z - L1
        d = math.hypot(r, dz)

        if d > (L2 + L3 + L4):
            return None

        cos_a = (L2**2 + d**2 - (L3 + L4)**2) / (2 * L2 * d)
        angle_a = math.acos(cos_a)

        cos_b = (L2**2 + (L3 + L4)**2 - d**2) / (2 * L2 * (L3 + L4))
        angle_b = math.acos(cos_b)

        theta2 = math.atan2(dz, r) - angle_a
        theta3 = math.pi - angle_b

        theta4 = 0.0
        theta5 = 0.0

        return [math.degrees(theta1), math.degrees(theta2), math.degrees(theta3), math.degrees(theta4), math.degrees(theta5)]
    except Exception as e:
        print(f"IK failed: {e}")
        return None
