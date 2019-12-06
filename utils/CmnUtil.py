"""Shared methods, to be loaded in other code.
"""
import numpy as np

ESC_KEYS = [27, 1048603]
MILLION = float(10**6)

def rad_to_deg(rad):
    return np.array(rad) *180./np.pi

def deg_to_rad(deg):
    return np.array(deg) *np.pi/180.

def normalize(v):
    norm=np.linalg.norm(v, ord=2)
    if norm==0:
        norm=np.finfo(v.dtype).eps
    return v/norm

def LPF(data_curr, data_prev, fc, dt):
    return 2*np.pi*fc*dt*data_curr + (1-2*np.pi*fc*dt)*data_prev;

def euler_to_quaternion(rot, unit='rad'):
    if unit=='deg':
        rot = deg_to_rad(rot)

    # for the various angular functions
    z,y,x = rot
    cy = np.cos(z * 0.5);
    sy = np.sin(z * 0.5);
    cp = np.cos(y * 0.5);
    sp = np.sin(y * 0.5);
    cr = np.cos(x * 0.5);
    sr = np.sin(x * 0.5);

    # quaternion
    qw = cy * cp * cr + sy * sp * sr;
    qx = cy * cp * sr - sy * sp * cr;
    qy = sy * cp * sr + cy * sp * cr;
    qz = sy * cp * cr - cy * sp * sr;

    return [qx, qy, qz, qw]

def quaternion_to_eulerAngles(q, unit='rad'):
    qx, qy, qz, qw = q

    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz);
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    roll = np.arctan2(sinr_cosp, cosr_cosp);

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx);
    if (abs(sinp) >= 1):    pitch = np.sign(sinp)*(np.pi/2); # use 90 degrees if out of range
    else:                   pitch = np.arcsin(sinp);

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy);
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    yaw = np.arctan2(siny_cosp, cosy_cosp);

    if unit=='deg':
        [roll, pitch, yaw] = rad_to_deg([roll, pitch, yaw])

    return [roll,pitch,yaw]
