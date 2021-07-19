from numpy import isnan, nan, arctan, arctan2, arcsin, arccos, sqrt, radians, pi, real, zeros, cos, sin
from forward_kinematics import get_torso_position, get_elbow_position
from hip_knee_pairs import pairs

# Function that returns the head angles given the head end-effector position (BottomCamera)
def get_head_angles(px, py, pz):
    l1 = -38.0
    l2 = 169.9
    # bottom camera 
    l3 = 93.6
    l4 = 61.6

    # theta1
    headYaw  = arctan2(py, (px - l1)) 
    # theta2
    headPitch = arcsin((l2 - pz) / sqrt(l4**2 + l3**2)) + arctan(l4/l3)

    # Check if the calculated angles fall within Pepper's range
    # if not set the angles to 0
    if headYaw.imag != 0.0 or headYaw.real < -2.1 or headYaw.real > 2.1:
        headYaw = 0.0
    if headPitch.imag != 0.0 or headPitch.real < -0.71 or headPitch.real > 0.45:
        headPitch = 0.0

    return headYaw, headPitch

# Function that returns all 4 angles of the arm chain given the elbow positon (ex, ey, ez) and wrist position (px, py, pz)
def get_arm_all_angles(ex, ey, ez, px, py, pz, arm):

    [shoulderPitch, shoulderRoll] = get_arm_partial_angles(ex, ey, ez, arm)
    elbowRoll = calculate_theta4(shoulderPitch, shoulderRoll, px, py, pz, arm)
    elbowYaw = calculate_theta3(shoulderPitch, shoulderRoll, elbowRoll, px, py, pz, arm)

    return shoulderPitch, shoulderRoll, elbowYaw, elbowRoll

# Function that returns the ElbowYaw (t3) and ElbowRoll (t4) given the partial angles (t1, t2) and the wrist position (px, py, pz)
def get_arm_t3t4_angles(t1, t2, px, py, pz, arm):

    elbowRoll = calculate_theta4(t1, t2, px, py, pz, arm)
    elbowYaw = calculate_theta3(t1, t2, elbowRoll, px, py, pz, arm)

    return elbowYaw, elbowRoll

# Function that returns the partial angles ShoulderPitch (t1) and ShoulderRoll (t2) given the elbow position (ex, ey, ez)
def get_arm_partial_angles(ex, ey, ez, arm):
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

    # ShoulderRoll or theta2 
    t2_temp = arcsin((ey - l2) / sqrt(l4**2 + l5**2)) - arctan(l5/l4)

    if arm == 'right':
        if t2_temp + arctan(l5/l4) > -pi/2 - arctan(l5/l4):
            shoulderRoll = t2_temp
        else:
            shoulderRoll = -pi - arcsin((ey - l2) / sqrt(l4**2 + l5**2)) - arctan(l5/l4)
            if shoulderRoll < -1.5630:
                shoulderRoll = t2_temp
        # check if solution is within Pepper's range
        if shoulderRoll.imag != 0 or shoulderRoll.real < -1.58 or shoulderRoll.real >= -0.0087:
            shoulderRoll = -0.0087

    elif arm == 'left':
        if t2_temp + arctan(l5/l4) < pi/2 - arctan(l5/l4):
            shoulderRoll = t2_temp
        else:
            shoulderRoll = pi - arcsin((ey - l2) / sqrt(l4**2 + l5**2)) - arctan(l5/l4)
            if shoulderRoll > 1.5630:
                shoulderRoll = t2_temp
        # check if solution is within Pepper's range
        if shoulderRoll.imag != 0 or shoulderRoll.real > 1.58 or shoulderRoll.real <= 0.0087:
            shoulderRoll = 0.0087

    # ShoulderPitch or theta1
    n = l4*cos(shoulderRoll) - l5*sin(shoulderRoll)
    t1_1 = arctan2(ex-l1, ez-l3) - arctan2(sqrt((ex-l1)**2 + (ez-l3)**2 - l6**2), l6)
    t1_2 = arctan2(l6*(ex-l1) - n*(ez-l3), l6*(ez-l3) + n*(ex-l1))
    # check if solution is within Pepper's range
    if t1_1.imag != 0 or t1_1.real < -2.1 or t1_1.real > 2.1:
        t1_1 = nan

    if t1_2.imag != 0 or t1_2.real < -2.1 or t1_2.real > 2.1:
        t1_2 = nan

    sol1 = get_elbow_position(t1_1, shoulderRoll, arm)
    sol2 = get_elbow_position(t1_2, shoulderRoll, arm)
    # check which of the solutions is closer to the position of the elbow 
    dist1 = sqrt((sol1[0]-ex)**2 + (sol1[1]-ey)**2 + (sol1[2]-ez)**2)
    dist2 = sqrt((sol2[0]-ex)**2 + (sol2[1]-ey)**2 + (sol2[2]-ez)**2)

    if dist1 < dist2 or isnan(dist2): 
        shoulderPitch = t1_1
    elif dist1 > dist2 or isnan(dist1):
        shoulderPitch = t1_2

    return shoulderPitch, shoulderRoll

        
# Function that returns the ElbowRoll (t4) given the angles t1, t2 and the wrist position (px, py, pz)
def calculate_theta4(t1, t2, px, py, pz, arm):
    l1 = -57.0
    l3 = 86.82
    l4 = 150
    d3 = 181.2
    z3 = 0.13

    if arm == 'right':
        l2 = -149.74
    elif arm == 'left':
        l2 = 149.74

    t2 = t2 - pi/2
    alpha = radians(9)

    term3 = (px - l1)*sin(t1) + (pz - l3)*cos(t1) - z3
    term4 = (pz-l3)*sin(t1)*sin(t2) + (py-l2)*cos(t2) - d3 - (px-l1)*sin(t2)*cos(t1)
    term2 = sin(alpha)*term3 + cos(alpha)*term4
    term1 = (1.0/l4)*term2      #IT IS IMPORTANT TO PUT .0 otherwise it rounds to 0

    if term1 > 1.0:
        theta4 = nan
    else:
        if arm == 'right':
            theta4 = arccos(term1)
        elif arm == 'left':
            theta4 = -arccos(term1)

    # check if solution is within Pepper's range
    if arm == 'right':
        if theta4.imag != 0.0 or theta4 > 1.58 or theta4 < 0.0087 or isnan(theta4):
            theta4 = 0.0087
    elif arm == 'left':
        if theta4.imag != 0.0 or theta4 > -0.0087 or theta4 < -1.58 or isnan(theta4):
            theta4 = -0.0087

    return theta4

# Function that returns the ElbowYaw (t3) given the angles t1, t2, t4 and the wrist position (px, py, pz)
def calculate_theta3(t1, t2, t4, px, py, pz, arm):
    l1 = -57.0
    l3 = 86.82
    d3 = 181.2
    d5 = 150

    if arm == 'right':
        l2 = -149.74
        a3 = 15.0 
    elif arm == 'left':
        l2 = 149.74
        a3 = -15.0 

    t2 = t2 - pi/2
    alpha = radians(9)

    aterm = (d3 + d5*cos(alpha)*cos(t4) + cos(t1)*sin(t2)*(px-l1) - sin(t1)*sin(t2)*(pz-l3) - cos(t2)*(py-l2)) / (d5*sin(t4)*sin(alpha))
    bterm = (cos(t2)*sin(t1)*(pz-l3) + a3 - cos(t1)*cos(t2)*(px-l1) - sin(t2)*(py-l2)) / (d5*sin(t4))

    if aterm.imag == 0 and bterm.imag == 0:
        theta3 = arctan2(aterm, bterm)
    else:
        theta3 = 0.0

    return theta3


# Function that returns the torso angles given the position of the end-effector (torso)
def get_torso_angles(px, py, pz):
    l3 = 0.02
    l4 = 139.0

    a2 = 268.0
    a3 = 79.0

    # HipRoll or theta3
    hipRoll = arcsin(py/l4)

    n = l4*cos(hipRoll) + a3
    term1 = (px**2 + pz**2 - n**2 - l3**2 - a2**2) / (2*a2 * sqrt(l3**2 + n**2))
    if term1 > 1.0:
        hipPitch = 0.0
        # estimate the KneePitch from HipPitch 
        kneePitch = pairs[hipPitch]
    else:
        # HipPitch or theta2
        t2_1 = arcsin(term1) - arctan2(n,l3)
        t2_2 = pi - arcsin(term1) - arctan2(n,l3)

        # check if solution is within Pepper's range
        if t2_1.imag != 0 or t2_1.real < -1.04 or t2_1.real > 1.04:
            t2_1 = nan

        if t2_2.imag != 0 or t2_2.real < -1.04 or t2_2.real > 1.04:
            t2_2 = nan

        sol1 = get_torso_position(t2_1, hipRoll)
        sol2 = get_torso_position(t2_2, hipRoll)
        # check which of the solutions is closer to the position of the torso 
        dist1 = sqrt((sol1[0]-px)**2 + (sol1[1]-py)**2 + (sol1[2]-pz)**2)
        dist2 = sqrt((sol2[0]-px)**2 + (sol2[1]-py)**2 + (sol2[2]-pz)**2)

        if isnan(dist1) and isnan(dist2):
            hipPitch = 0.0
            # estimate the KneePitch from HipPitch 
            kneePitch = pairs[hipPitch]
        elif dist1 < dist2 or isnan(dist2): 
            hipPitch = t2_1
            t2_rounded = round(t2_1,2)
            # estimate the KneePitch from HipPitch 
            kneePitch = pairs[t2_rounded]

        elif dist1 > dist2 or isnan(dist1):
            hipPitch = t2_2
            t2_rounded = round(t2_2,2)
            # estimate the KneePitch from HipPitch 
            kneePitch = pairs[t2_rounded]


    return kneePitch, hipPitch, hipRoll
