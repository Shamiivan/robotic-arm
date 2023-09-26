import math

def forward_kinematics(links1, links2, links3, theta1, theta2, theta3):
    L12 = links1
    L23 = links2
    L34 = links3
    J1 = math.radians(theta1 % 180)  # Apply modulo to constrain angles to [0, 180)
    J2 = math.radians(theta2 % 180)
    J3 = math.radians(theta3 % 180)

    # Joint equations
    x2 = L12 * math.cos(J1)
    x3 = L23 * math.cos(J1 + J2) + x2
    xe = L34 * math.cos(J1 + J2 + J3) + x3
    y2 = L12 * math.sin(J1)
    y3 = L23 * math.sin(J1 + J2) + y2
    ye = L34 * math.sin(J1 + J2 + J3) + y3
    gamma = math.degrees(J1 + J2 + J3) % 180
    
    print(f'Forward Kinematics:')
    print(f'The position of the end-effector is ({xe:.4f}, {ye:.4f}) and orientation is ({gamma:.4f}) degrees')

def inverse_kinematics(links1, links2, links3, positionx, positiony, gamma):
    L12 = links1
    L23 = links2
    L34 = links3
    xe = positionx
    ye = positiony
    g = math.radians(gamma % 180)  # Apply modulo to constrain angles to [0, 180)

    # Calculate position P3
    x3 = xe - (L34 * math.cos(g))
    y3 = ye - (L34 * math.sin(g))
    C = math.sqrt(x3**2 + y3**2)

    if (L12 + L23) > C:
        # Calculate angles a and B
        a = math.degrees(math.acos((L12**2 + L23**2 - C**2) / (2 * L12 * L23)))
        B = math.degrees(math.acos((L12**2 + C**2 - L23**2) / (2 * L12 * C)))

        # Calculate joint angles for elbow-down configuration
        J1a = (math.degrees(math.atan2(y3, x3) - B) % 180)  # Apply modulo to constrain angles
        J2a = (180 - a) % 180
        J3a = (gamma - J1a - J2a) % 180

        # Calculate joint angles for elbow-up configuration
        J1b = (math.degrees(math.atan2(y3, x3) + B) % 180)  # Apply modulo to constrain angles
        J2b = (-(180 - a)) % 180
        J3b = (gamma - J1b - J2b) % 180

        print('Inverse Kinematics:')
        print('Elbow-Down Configuration:')
        print('The joint 1, 2, and 3 angles are ({:.4f}, {:.4f}, {:.4f}) degrees respectively.'.format(J1a, J2a, J3a))
        print('Elbow-Up Configuration:')
        print('The joint 1, 2, and 3 angles are ({:.4f}, {:.4f}, {:.4f}) degrees respectively.'.format(J1b, J2b, J3b))
    else:
        print('Dimension error!')
        print('End-effector is outside the workspace.')

# Example usage:
forward_kinematics(1.0, 2.0, 1.5, 210, 300, 480)
inverse_kinematics(1.0, 2.0, 1.5, 2.0, 2.0, 225)
