import numpy as np

####### Rotation Matrices R_axis ---------------------------------------------------------------------------------
# function computing the rotation matrix for the X axis for angle and dimension (dim) which default is 4
def rx_calc(angle, dim=4):

    c, s = np.cos(angle), np.sin(angle)
    if dim == 4:
        Rx = np.array(([1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]))
    elif dim == 3:
        Rx = np.array(([1, 0, 0], [0, c, -s], [0, s, c]))

    return Rx

# function computing the rotation matrix for the Y axis for angle and dimension (dim) which default is 4
def ry_calc(angle, dim=4):

    c, s = np.cos(angle), np.sin(angle)
    if dim == 4:
        Ry = np.array(([c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]))
    elif dim == 3:
        Ry = np.array(([c, 0, s], [0, 1, 0], [-s, 0, c]))

    return Ry

# function computing the rotation matrix for the Z axis for angle and dimension (dim) which default is 4
def rz_calc(angle, dim=4):

    c, s = np.cos(angle), np.sin(angle)
    if dim == 4:
        Rz = np.array(([c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]))
    elif dim == 3:
        Rz = np.array(([c, -s, 0], [s, c, 0], [0, 0, 1]))

    return Rz


####### DH Transformation Matrix T_dh ------------------------------------------------------------------------------
# function computing the transformation matrix using the DH parameters alpha, a, theta, and d
def computeT_dh(alpha, a, theta, d):

    Tx, Tz = (np.identity(4), np.identity(4))

    Tx[0, 3] = a
    Tz[2, 3] = d

    Rx = np.array(([1, 0, 0, 0], [0, np.cos(alpha), -np.sin(alpha), 0], [0, np.sin(alpha), np.cos(alpha), 0], [0, 0, 0, 1]))

    Rz = np.array(([np.cos(theta), -np.sin(theta), 0, 0], [np.sin(theta), np.cos(theta), 0, 0], [0, 0, 1, 0], [0, 0, 0,1]))

    return Rx.dot(Tx.dot(Rz.dot(Tz)))





