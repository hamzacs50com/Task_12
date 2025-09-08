#!/usr/bin/env python3
import numpy as np
from math import sin, cos
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rotation_matrix_from_euler(roll, pitch, yaw):
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rx = np.array([[1,0,0],
                   [0,cos(roll), -sin(roll)],
                   [0,sin(roll),  cos(roll)]])
    Ry = np.array([[cos(pitch),0,sin(pitch)],
                   [0,1,0],
                   [-sin(pitch),0,cos(pitch)]])
    Rz = np.array([[cos(yaw), -sin(yaw),0],
                   [sin(yaw),  cos(yaw),0],
                   [0,0,1]])
    return Rz.dot(Ry).dot(Rx)

def make_htm(tx,ty,tz, roll, pitch, yaw):
    R = rotation_matrix_from_euler(roll,pitch,yaw)
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = [tx,ty,tz]
    return T

def apply_htm_to_points(points, tx,ty,tz, roll, pitch, yaw):
    T = make_htm(tx,ty,tz, roll, pitch, yaw)
    pts = np.array(points)
    ones = np.ones((pts.shape[0],1))
    homo = np.hstack([pts, ones])
    transformed = (T.dot(homo.T)).T
    return transformed[:,:3]

# quaternion helpers
def quaternion_from_euler(roll, pitch, yaw):
    cy = cos(yaw*0.5); sy = sin(yaw*0.5)
    cp = cos(pitch*0.5); sp = sin(pitch*0.5)
    cr = cos(roll*0.5); sr = sin(roll*0.5)
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy
    z = cr*cp*sy - sr*sp*cy
    return np.array([w,x,y,z])

def quaternion_multiply(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w,x,y,z])

def rotate_point_by_quaternion(q, v):
    # v is 3-vector
    qv = np.array([0.0, v[0], v[1], v[2]])
    q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    tmp = quaternion_multiply(q, qv)
    res = quaternion_multiply(tmp, q_conj)
    return res[1:]

def apply_quaternion_to_points(points, tx,ty,tz, roll, pitch, yaw):
    q = quaternion_from_euler(roll,pitch,yaw)
    pts = np.array(points)
    rotated = np.array([rotate_point_by_quaternion(q, p) for p in pts])
    translated = rotated + np.array([tx,ty,tz])
    return translated

def set_axes_equal(ax):
    # make 3D axes have equal scale
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    plot_radius = 0.5*max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

if __name__ == "__main__":
    # example point cloud
    points = [
        [0.2, 0.0, 0.0],
        [0.2, 0.2, 0.0],
        [0.2, -0.2, 0.0],
        [0.0, 0.0, 0.2],
        [-0.2, 0.0, 0.0]
    ]

    # translation and rotation inputs
    tx, ty, tz = 0.5, 0.1, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.785398  # 45 degrees yaw

    # choose method: HTM or quaternion
    transformed_htm = apply_htm_to_points(points, tx,ty,tz, roll, pitch, yaw)
    transformed_quat = apply_quaternion_to_points(points, tx,ty,tz, roll, pitch, yaw)

    # plot original and HTM-transformed
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    orig = np.array(points)
    ax.scatter(orig[:,0], orig[:,1], orig[:,2], marker='o')
    ax.scatter(transformed_htm[:,0], transformed_htm[:,1], transformed_htm[:,2], marker='x')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    set_axes_equal(ax)
    plt.title('original (o) vs transformed (x) using HTM')
    plt.show()
