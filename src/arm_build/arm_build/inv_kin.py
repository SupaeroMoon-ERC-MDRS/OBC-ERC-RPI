import numpy as np

def compute_joint_angles(self, positionx, positiony, gamma):
    # Assign inputs
    xe = positionx
    ye = positiony
    g = gamma  # in degrees

    # Position P3
    x3 = xe - (self.l3 * np.cos(np.radians(g)))
    y3 = ye - (self.l3 * np.sin(np.radians(g)))
    C = np.sqrt(x3**2 + y3**2)

    if (self.l1 + self.l2) > C:
        # Angles a and B (law of cosines)
        a = np.degrees(np.arccos((self.l1**2 + self.l2**2 - C**2) / (2 * self.l1 * self.l2)))
        B = np.degrees(np.arccos((self.l1**2 + C**2 - self.l2**2) / (2 * self.l1 * C)))

        # Joint angles for elbow-down configuration
        J1a = np.degrees(np.arctan2(y3, x3)) - B
        J2a = 180 - a
        J3a = g - J1a - J2a

        # Joint angles for elbow-up configuration
        J1b = np.degrees(np.arctan2(y3, x3)) + B
        J2b = -(180 - a)
        J3b = g - J1b - J2b

        print(f'The joint 1, 2 and 3 angles are ({J1a:.2f}, {J2a:.2f}, {J3a:.2f}) respectively for elbow-down configuration.')
        print(f'The joint 1, 2 and 3 angles are ({J1b:.2f}, {J2b:.2f}, {J3b:.2f}) respectively for elbow-up configuration.')
    else:
        print('     Dimension error!')
        print('     End-effector is outside the workspace.')
