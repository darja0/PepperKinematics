from naoqi import ALProxy
import time
from inverse_kinematics import get_head_angles, get_arm_all_angles, get_torso_angles

IP = '127.0.0.1'

motion = ALProxy("ALMotion", IP, 9559)

# fraction of the maximum speed at which it moves 
fractionMaxSpeed = 0.2

# HEAD CHAIN -------------------------------------------------------------------
# position of the bottom camera in millimeters
bottom_cam = [23.92, 65.36, 235.37]
# calculating the corresponding angles 
[t1, t2] = get_head_angles(bottom_cam[0], bottom_cam[1], bottom_cam[2])

names = ["HeadYaw", "HeadPitch"]
angles = [t1, t2]

# Head sending motion commands
try:
    motion.setAngles(names, angles, fractionMaxSpeed)
except BaseException, err:
    print err

time.sleep(2)

# ARM CHAINS -------------------------------------------------------------------
# positions of the right elbow and wrist in millimeters
relbow = [33.26, -304, 120.17]
rwrist = [115.34, -314.57, 245.27]
# calculating the corresponding angles 
[t1, t2, t3, t4] = get_arm_all_angles(relbow[0], relbow[1], relbow[2], rwrist[0], rwrist[1], rwrist[2], 'right')

names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
angles = [t1, t2, t3, t4]

# Right arm sending motion commands
try:
    motion.setAngles(names, angles, fractionMaxSpeed)
except BaseException, err:
    print err


time.sleep(2)

# positions of the left elbow and wrist in millimeters
lelbow = [65.28, 27.67, 42.15]
lwrist = [169.10, 313.44, 143.98]
# calculating the corresponding angles 
[t1, t2, t3, t4] = get_arm_all_angles(lelbow[0], lelbow[1], lelbow[2], lwrist[0], lwrist[1], lwrist[2], 'left')

names = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
angles = [t1, t2, t3, t4]

# Left arm sending motion commands 
try:
    motion.setAngles(names, angles, fractionMaxSpeed)
except BaseException, err:
    print err


time.sleep(2)

# TORSO CHAIN -------------------------------------------------------------------
# position of the knee in milimeters
knee = [6.88, 22.54, 555.17]
# calculating the corresponding angles 
[t1, t2, t3] = get_torso_angles(knee[0], knee[1], knee[2])

names = ["KneePitch", "HipPitch", "HipRoll"]
angles = [t1, t2, t3]

# Torso sending motion commands
try:
    motion.setAngles(names, angles, fractionMaxSpeed)
except BaseException, err:
    print err

time.sleep(2)

# send Pepper to Standing posture 
postureProxy = ALProxy("ALRobotPosture", IP, 9559)
postureProxy.goToPosture("Stand", 0.4)
