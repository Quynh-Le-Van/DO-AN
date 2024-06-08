#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


TwistMsg = Twist
PointMsg = Point
BoolMsg = Bool

msg = """
Reading from the keyboard  and Publishing to Twist and Point!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

Gripper control:
g : open gripper
h : close gripper

Control end effector position in XYZ space:
x/X : increase/decrease x by 0.01
y/Y : increase/decrease y by 0.01
z/Z : increase/decrease z by 0.01

anything else : stop

q/Q : increase/decrease max speeds by 10%
w/W : increase/decrease only linear speed by 10%
e/E : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'Q': (.9, .9),
    'w': (1.1, 1),
    'W': (.9, 1),
    'e': (1, 1.1),
    'E': (1, .9),
}

armBindings = {
    'x': (0.01, 0, 0),
    'X': (-0.01, 0, 0),
    'y': (0, 0.01, 0),
    'Y': (0, -0.01, 0),
    'z': (0, 0, 0.01),
    'Z': (0, 0, -0.01),
}

gripperBindings = {
    'g': True,
    'h': False,
}

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher_twist = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        self.publisher_point = rospy.Publisher('arm_position', PointMsg, queue_size=1)
        self.publisher_gripper = rospy.Publisher('gripper_state', BoolMsg, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.arm_x = 0.0
        self.arm_y = 0.0
        self.arm_z = 0.0
        self.gripper_state = False
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher_twist.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher_twist.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn, arm_x, arm_y, arm_z, gripper_state):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.arm_x = arm_x
        self.arm_y = arm_y
        self.arm_z = arm_z
        self.gripper_state = gripper_state
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0, 0, 0, False)
        self.join()

    def run(self):
        twist_msg = TwistMsg()
        if stamped:
            twist = twist_msg.twist
            twist_msg.header.stamp = rospy.Time.now()
            twist_msg.header.frame_id = twist_frame
        else:
            twist = twist_msg

        point_msg = PointMsg()
        gripper_msg = BoolMsg()

        while not self.done:
            if stamped:
                twist_msg.header.stamp = rospy.Time.now()
            self.condition.acquire()
            self.condition.wait(self.timeout)

            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            point_msg.x = self.arm_x
            point_msg.y = self.arm_y
            point_msg.z = self.arm_z

            gripper_msg.data = self.gripper_state

            self.condition.release()

            self.publisher_twist.publish(twist_msg)
            self.publisher_point.publish(point_msg)
            self.publisher_gripper.publish(gripper_msg)

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher_twist.publish(twist_msg)

        point_msg.x = 0
        point_msg.y = 0
        point_msg.z = 0
        self.publisher_point.publish(point_msg)

        gripper_msg.data = False
        self.publisher_gripper.publish(gripper_msg)


def getKey(settings, timeout):
    if sys.platform == 'win32':
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.5)
    stamped = rospy.get_param("~stamped", False)
    twist_frame = rospy.get_param("~frame_id", '')
    if stamped:
        TwistMsg = TwistStamped

    pub_thread = PublishThread(repeat)

    x = 0
    y = 0
    z = 0
    th = 0
    arm_x = 0
    arm_y = 0
    arm_z = 0
    gripper_state = False
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn, arm_x, arm_y, arm_z, gripper_state)

        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key in armBindings.keys():
                arm_x += armBindings[key][0]
                arm_y += armBindings[key][1]
                arm_z += armBindings[key][2]
            elif key in gripperBindings.keys():
                gripper_state = gripperBindings[key]
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if key == '\x03':
                    break

            pub_thread.update(x, y, z, th, speed, turn, arm_x, arm_y, arm_z, gripper_state)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)