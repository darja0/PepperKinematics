from numpy import sin, cos, array, radians, identity, dot, pi
from math_functions import computeT_dh, rx_calc, ry_calc, rz_calc
from hip_knee_pairs import pairs

# Function to get the position of the head end-effector (BotoomCamera)
def get_head_position(t1, t2):
    l1 = -38.0
    l2 = 169.9
    # Bottom camera 
    l3 = 93.6
    l4 = 61.6

    px = l1 + cos(t1)*(l3*cos(t2) + l4*sin(t2))
    py = sin(t1)*(l3*cos(t2) + l4*sin(t2))
    pz = l2 + l4*cos(t2) - l3*sin(t2)

    return array([px, py, pz])

# Function to get the position of the partial arm end-effector (elbow)
def get_elbow_position(t1, t2, arm):
    l1 = -57.0
    l3 = 86.82
    l4 = 181.2
    l6 = 0.13 

    if arm == 'right':
        l2 = -149.74
        l5 = -15.0 
    elif arm == 'left':
        l2 = 149.74
        l5 = 15.0 

    px = l1 + l6*sin(t1) + cos(t1)*(l4*cos(t2) - l5*sin(t2))
    py = l2 + l5*cos(t2) + l4*sin(t2)
    pz = l3 + l6*cos(t1) - sin(t1)*(l4*cos(t2) - l5*sin(t2))

    return array([px, py, pz])
       
# Function to get the position of the full arm end-effector (wrist)
def get_wrist_position(t1, t2, t3, t4, arm):
    l1 = -57.0
    l3 = 86.82
    l4 = 150
    d3 = 181.2
    z3 = 0.13 

    if arm == 'right':
        l2 = -149.74
        a3 = 15.0 
    elif arm == 'left':
        l2 = 149.74
        a3 = -15.0 

    t2 = t2 - pi/2

    a = radians(9)

    px = l1 - cos(t1)*(sin(t2)*(d3 + l4*cos(a)*cos(t4) - l4*sin(a)*sin(t3)*sin(t4)) - cos(t2)*(a3 - l4*cos(t3)*sin(t4))) + \
        sin(t1)*(z3 + l4*sin(a)*cos(t4) + l4*cos(a)*sin(t3)*sin(t4))

    py = l2 + cos(t2)*(d3 + l4*cos(a)*cos(t4) - l4*sin(a)*sin(t3)*sin(t4)) + sin(t2)*(a3 - l4*cos(t3)*sin(t4))

    pz = l3 + sin(t1)*(sin(t2)*(d3 + l4*cos(a)*cos(t4) - l4*sin(a)*sin(t3)*sin(t4)) - cos(t2)*(a3 - l4*cos(t3)*sin(t4))) + \
        cos(t1)*(z3 + l4*sin(a)*cos(t4) + l4*cos(a)*sin(t3)*sin(t4))

    return array([px, py, pz])

# Function to get the position of the torso end-effector (torso)
def get_torso_position(t2, t3, t1=None):
    l3 = 0.02
    l4 = 139.0

    a2 = 268.0
    a3 = 79.0

    if t1 is None:
        t1 = 0.0
    else:
        t2_rounded = round(t2,2)
        t1 = pairs[t2_rounded]

    t1 = t1 + pi/2

    px = cos(t1)*(cos(t2)*(l4*cos(t3) + a3) + l3*sin(t2) + a2) - sin(t1)*(sin(t2)*(l4*cos(t3) + a3) - l3*cos(t2))
    py = l4*sin(t3)
    pz = cos(t1)*(sin(t2)*(l4*cos(t3) + a3) - l3*cos(t2)) + sin(t1)*(cos(t2)*(l4*cos(t3) + a3) + l3*sin(t2) + a2)


    return array([px, py, pz])
       
# Function that returns the FK matrix of the head chain
def head_forward_matrix(theta1, theta2):
    # Lengths of links
    l1 = -38.0
    l2 = 169.9
    # bottom camera 
    l3 = 93.6
    l4 = 61.6

    A0 = array(([1, 0, 0, l1], [0, 1, 0, 0], [0, 0, 1, l2], [0, 0, 0, 1]))

    T0 = computeT_dh(0, 0, theta1, 0)

    T1 = computeT_dh(-pi/2, 0, theta2, 0)

    Rx = rx_calc(pi/2)

    A2 = array(([1, 0, 0, l3], [0, 1, 0, 0], [0, 0, 1, l4], [0, 0, 0, 1]))

    return A0.dot(T0.dot(T1.dot(Rx.dot(A2))))

# Function that returns the FK matrix of the partial arm chain
def arm_partial_forward_matrix(theta1, theta2, arm):
    l1 = -57.0
    l3 = 86.82
    l4 = 181.2
    l6 = 0.13 

    if arm == 'right':
        l2 = -149.74
        l5 = -15.0 
    elif arm == 'left':
        l2 = 149.74
        l5 = 15.0 

    A0 = array(([1, 0, 0, l1], [0, 1, 0, l2], [0, 0, 1, l3], [0, 0, 0, 1]))

    T0 = computeT_dh(-pi/2, 0, theta1, 0)

    T1 = computeT_dh(pi/2, 0, theta2, 0)

    A2 = array(([1, 0, 0, l4], [0, 1, 0, l5], [0, 0, 1, l6], [0, 0, 0, 1]))

    return A0.dot(T0.dot(T1.dot(A2)))

# Function that returns the FK matrix of the arm chain
def arm_forward_matrix(theta1, theta2, theta3, theta4, arm):
    l1 = -57.0
    l3 = 86.82
    d3 = 181.2
    z3 = 0.13 

    if arm == 'right':
        l2 = -149.74
        a3 = -15.0 
    elif arm == 'left':
        l2 = 149.74
        a3 = 15.0 

    A0 = identity(4)
    A0[0,3] = l1
    A0[1,3] = l2
    A0[2,3] = l3

    # ShoulderPitch
    T0 = computeT_dh(-pi/2, 0, theta1, 0)

    # ShoulderRoll
    theta2 = theta2 - pi/2;
    T1 = computeT_dh(pi/2, 0, theta2, 0)

    # ElbowYaw
    T2 = computeT_dh(-pi/2, a3, 0, d3)

    # Adding the 9 degrees offset
    T2_temp = computeT_dh(radians(9), 0, theta3, z3)
    
    # ElbowRoll
    T3 = computeT_dh(pi/2, 0, theta4, 0)

    # pi/2
    Rz = array(([0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]))

    A1 = identity(4)
    A1[0,3] = 150
    return A0.dot(T0.dot(T1.dot(T2.dot(T2_temp.dot(T3.dot(Rz.dot(A1)))))))

# Function that returns the FK matrix of the torso chain
def hip_forward_matrix(theta2, theta3):
    
    t2_rounded = round(theta2,2)

    theta1 = pairs[t2_rounded]

    theta1 = theta1 + pi/2

    T0 = computeT_dh(pi/2, 0, theta1, 0)

    T1 = computeT_dh(0, 268, theta2, 0)

    T2 = computeT_dh(-pi/2, 79, theta3, 0)

    l2 = -0.02
    l3 = 139.0

    A2 = array(([1, 0, 0, l2], [0, 1, 0, 0], [0, 0, 1, l3], [0, 0, 0, 1]))

    # pi/2
    Ry = array(([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]]))

    return T0.dot(T1.dot(T2.dot(Ry.dot(A2))))

