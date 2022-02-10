#!/usr/bin/env python

import rospy
import numpy as np
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from multiprocessing import Process, Value, Manager
import time
import os
from collections import deque
from pynput.keyboard import Listener, Key

delta = Value('d', 0.)
core = Value('b', True)
manager = Manager()
dt = 0.05
L_1 = 0.5
L_2 = 1.4
v = 0.

moveBindings = {
        'w':(1., 0., 0., 0.),
        's':(-1., 0., 0., 0.),
        'a':(0.7, 0., 0., -1.),
        'd':(-0.7, 0., 0., -1.),
    }

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def throttle_update(v, dt, e_buffer, max_v):
    K_P = 1
    K_D = 0.01
    K_I = 0.001
    e = max_v - v
    e_buffer.append(e)

    if len(e_buffer) >= 2:
        de = (e_buffer[-1] - e_buffer[-2]) / dt
        ie = sum(e_buffer) * dt
    else:
        de = 0.0
        ie = 0.0
    
    throttle = np.clip((K_P * e) + (K_D * de / dt) + (K_I * ie * dt), -1.0, 1.0)
    return throttle, e_buffer

def steer_update(s, dt, s_buffer, max_s):
    K_P = 1
    K_D = 0.01
    K_I = 0.001
    e = max_s - s
    s_buffer.append(e)

    if len(s_buffer) >= 2:
        de = (s_buffer[-1] - s_buffer[-2]) / dt
        ie = sum(s_buffer) * dt
    else:
        de = 0.0
        ie = 0.0
    
    steer = np.clip((K_P * e) + (K_D * de / dt) + (K_I * ie * dt), -np.pi/4, np.pi/4)
    return steer, s_buffer

def robot_pose(x, y, th):
    ts = TransformStamped()
    ts.header.frame_id = "odom"
    ts.child_frame_id = "base_link"
    ts.transform.translation.x = x
    ts.transform.translation.y = y
    ts.transform.translation.z = 0.0
    qx, qy, qz, qw = quaternion_from_euler(0., 0., th)
    ts.transform.rotation.x = qx
    ts.transform.rotation.y = qy
    ts.transform.rotation.z = qz
    ts.transform.rotation.w = qw
    return ts

def talker():
    pub_jt = rospy.Publisher('/joint_states', JointState, queue_size=1)
    pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
    rospy.init_node('joint_state_publisher')
    rate = rospy.Rate(10)
    t1 = JointState()
    t1.header.stamp = rospy.Time.now()
    t1.name = ['trailer_pivot']
    
    while core.value:
        t1.header.stamp = rospy.Time.now()
        t1.position = [delta.value]
        pub_jt.publish(t1)
        t = joints[0]
        t.header.stamp = rospy.Time.now()
        tfm = TFMessage([t])
        pub_tf.publish(tfm)
        rate.sleep()

w = Value('b', False)
a = Value('b', False)
d = Value('b', False)
s = Value('b', False)

def on_press(key):
    try:
        if key.char == 'w':
            w.value = True
        if key.char == 'a':
            a.value = True
        if key.char == 'd':
            d.value = True
        if key.char == 's':
            s.value = True
    except Exception as e:
        print(e)

def on_release(key):
    try:
        if key.char == 'w':
            w.value = False
        if key.char == 'a':
            a.value = False
        if key.char == 'd':
            d.value = False
        if key.char == 's':
            s.value = False
    except Exception as e:
        print(e)

if __name__=="__main__":
    # os.system("rosnode kill /joint_state_publisher")
    listener = Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start()
    x, y, th = 0., 0., 0.
    ts = robot_pose(x, y, th)
    joints = manager.list([ts])
    p = Process(target=talker)
    p.start()
    v = 0.
    steer = 0.
    delt = 0.
    alpha = 0.
    e_buffer = deque(maxlen=20)
    s_buffer = deque(maxlen=20)
    try:
        while True:
            time.sleep(dt)
            if w.value == True:
                throttle, e_buffer = throttle_update(v, dt, e_buffer, 1)
            elif s.value == True:
                throttle, e_buffer = throttle_update(v, dt, e_buffer, -1)
            else:
                throttle, e_buffer = throttle_update(v, dt, e_buffer, 0)
            if a.value == True and w.value == True:
                steer, s_buffer = steer_update(delt, dt, s_buffer, -np.pi/4)
            elif d.value == True and w.value == True:
                steer, s_buffer = steer_update(delt, dt, s_buffer, np.pi/4)
            if a.value == True and s.value == True:
                steer, s_buffer = steer_update(delt, dt, s_buffer, -np.pi/4)
            elif d.value == True and s.value == True:
                steer, s_buffer = steer_update(delt, dt, s_buffer, np.pi/4)
            else:
                steer, s_buffer = steer_update(delt, dt, s_buffer, 0)
            delt += steer * dt
            v += throttle * dt
            x += v * np.cos(th) * dt
            y += v * np.sin(th) * dt
            th += v / L_1 * np.tan(delt) * dt
            th = normalize_angle(th)
            alpha += v*np.sin(th-alpha) * dt
            alpha = normalize_angle(alpha)
            ts = robot_pose(x, y, th)
            joints[0] = ts
            delta.value = th - alpha
    except Exception as e:
        print(e)
    finally:
        core.value = False
        p.terminate()
