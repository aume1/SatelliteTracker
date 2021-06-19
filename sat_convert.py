import math
from math import cos, sin
import numpy as np


def servo_angles_to_xyzlines(a1, a2, r=0.5):
    alpha, beta = math.radians(a1), math.radians(a2)
    x1, y1, z1 = r * cos(alpha), 0, r * sin(alpha)

    transform = np.array([[cos(alpha), 0, -sin(alpha), x1],  # DH matrix
                          [0, 1, 0, y1],
                          [sin(alpha), 0, cos(alpha), z1],
                          [0, 0, 0, 1]])
    x2, y2, z2 = r*sin(beta), r*cos(beta), 0
    xyz2 = np.matmul(transform, [x2, y2, z2, 1]).transpose()
    x2, y2, z2, _ = xyz2
    return [[0, x1, x2], [0, y1, y2], [0, z1, z2]]


def xyz_to_servo_angles(x2, y2, z2):
    r = math.sqrt(x2**2 + y2**2 + z2**2)
    if x2 == 0:
        alpha = 90
    else:
        alpha = math.degrees(math.atan(z2/x2))
    beta = math.degrees(math.acos(y2/r))
    if alpha < 0:
        alpha = 180 + alpha
    return alpha, beta


def el_az_to_servo_angles(el, az, r=0.5):
    xyz = el_az_to_xyz(el, az, r=r)
    return xyz_to_servo_angles(*xyz)


def el_az_to_xyz(el, az, r=0.5):
    el = math.radians(el)
    az = math.radians(az)
    x = r * cos(el) * cos(az)
    y = r * cos(el) * sin(az)
    z = r * sin(el)
    return x, y, z


def xyz_to_el_az(x, y, z):
    az = math.atan2(x, y)
    r = math.sqrt(x**2 + y**2)
    el = math.atan2(z, r)

    if el < 0 < z:
        print('elevation is less than 0 and z is greater than 0')
        el += math.pi
        az += math.pi
    return el, az


def rotate(x, y, z, alpha, axis=None):
    alpha = math.radians(alpha)
    if axis == 1 or axis == 'y':
        transform = np.array([[cos(alpha),  0, sin(alpha)],
                              [0,           1, 0],
                              [-sin(alpha), 0, cos(alpha)]])
    elif axis == 2 or axis == 'z':
        transform = np.array([[cos(alpha), -sin(alpha), 0],
                              [sin(alpha),  cos(alpha), 0],
                              [0,           0,          1]])
    else:
        transform = np.array([[1, 0,           0],
                              [0, cos(alpha), -sin(alpha)],
                              [0, sin(alpha),  cos(alpha)]])
    return *np.matmul([x, y, z], transform), 1


def r3(points, rotations):
    if len(rotations) != 3:
        print(f'invalid rotations: <{rotations=}>. should be of the form [rx, ry, rz]')
        return points

    rx, ry, rz = np.radians(rotations)
    rx_mat = np.array([[1, 0,        0],
                       [0, cos(rx), -sin(rx)],
                       [0, sin(rx),  cos(rx)]])
    ry_mat = np.array([[cos(ry),  0, sin(ry)],
                       [0,        1,       0],
                       [-sin(ry), 0, cos(ry)]])
    rz_mat = np.array([[cos(rz), -sin(rz), 0],
                       [sin(rz),  cos(rz), 0],
                       [0,        0,       1]])
    rotate = np.matmul(np.matmul(rz_mat, ry_mat), rx_mat)
    return np.matmul(points, rotate)


def servo_angles_to_camera_line(a1, a2, r=0.5):
    alpha, beta = math.radians(a1), math.radians(a2)
    x2 = r*sin(beta) * cos(alpha)
    y2 = r*cos(beta)
    z2 = sin(alpha) * r * sin(beta)
    return [[0, x2], [0, y2], [0, z2]]


def plot_servo(alpha, beta, ax, color1='--k', color2='--k'):
    nums = servo_angles_to_xyzlines(alpha, beta)
    l1 = [nums[x][:2] for x in range(len(nums))]
    l2 = [nums[x][1:3] for x in range(len(nums))]
    ax.plot3D(*l1, color1)
    ax.plot3D(*l2, color2)


def plot_servo_dir(alpha, beta, ax, color='--k'):
    nums = servo_angles_to_camera_line(alpha, beta)
    l1 = [nums[x][:2] for x in range(len(nums))]
    ax.plot3D(*l1, color)


def setup_plot(ax):
    ax.set_xlim3d(-1, 1)
    ax.set_ylim3d(-1, 1)
    ax.set_zlim3d(0, 2)
    ax.plot3D([-1, 1], [0, 0], [0, 0], '--k')
    ax.plot3D([0, 0], [-1, 1], [0, 0], '--k')


def plot_el_az(el, az, ax, r=0.5, color='--k'):
    x, y, z = el_az_to_xyz(el, az, r=r)
    ax.plot3D([0, x], [0, y], [0, z], color)


def plot_x_y_z(x, y, z, ax, color='--k'):
    ax.plot3D([0, x], [0, y], [0, z], color)


def get_quadrant(x, y, z):
    sign = lambda x: -1 if x < 0 else 1 if x > 0 else 0
    x = sign(x)
    y = sign(y)
    z = sign(z)
    return x, y, z


if __name__ == "__main__":
    import pigpio
    from servo import Servo
    import time
    pi = pigpio.pi('192.168.178.229')
    s1 = Servo(pi, 16, angle_range=[-20, 200], pulse_range=[550,2008])
    s2 = Servo(pi, 12, angle_range=[-20, 200], pulse_range=[550,2008])
    s1.set_angle(90)
    s2.set_angle(90)

    el, az = 43, 87
    el_dir = 10/18

    while True:
        if el_dir > 0 and el >= 80-abs(el_dir):
            el_dir = -el_dir
        if el_dir < 0 and el <= 10+abs(el_dir):
            el_dir = -el_dir
        el += el_dir

        az += 10
        if az > 180:
            az -= 360
        az += 2.5
        if el > 90:
            el = 0

        xyz = el_az_to_xyz(el, az)
        x, y = el_az_to_servo_angles(el, az)

        print()
        s1.set_angle(x)
        s2.set_angle(y)
        time.sleep(0.02)
