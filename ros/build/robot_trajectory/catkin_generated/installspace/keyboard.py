#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

import sys
from select import select

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

TwistMsg = Twist


msg = """
Reading from the keyboard and Publishing to Twist!
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

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

a/d : increase/decrease end-effector's x position
s/w : increase/decrease end-effector's y position

g : close gripper
h : open gripper

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
    'w': (0, 0, 0, 0, 1, 0),  # Increase end-effector's x position
    's': (0, 0, 0, 0, -1, 0), # Decrease end-effector's x position
    'a': (0, 0, 0, 0, 0, 1),  # Increase end-effector's y position
    'd': (0, 0, 0, 0, 0, -1), # Decrease end-effector's y position
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

gripperBindings = {
    'g': True,  # Close gripper
    'h': False  # Open gripper
}

pressed_keys = set()  # Lưu trữ các phím đang được nhấn


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', TwistMsg, queue_size=1)
        self.arm_publisher = rospy.Publisher('arm_cmd', Pose, queue_size=1)
        self.gripper_publisher = rospy.Publisher('gripper_cmd', Bool, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.arm_x = 0.0
        self.arm_y = 0.0
        self.gripper_state = False
        self.condition = threading.Condition()
        self.done = False

        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, th, speed, turn, arm_x, arm_y, gripper_state):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        self.arm_x = arm_x
        self.arm_y = arm_y
        self.gripper_state = gripper_state
        if pressed_keys:
            self.condition.notify()
        self.condition.release()

    def run(self):
        twist_msg = TwistMsg()
        pose_msg = Pose()
        gripper_msg = Bool()

        while not self.done:
            twist_msg.linear.x = self.x * self.speed
            twist_msg.linear.y = self.y * self.speed
            twist_msg.linear.z = self.z * self.speed
            twist_msg.angular.x = 0
            twist_msg.angular.y = 0
            twist_msg.angular.z = self.th * self.turn

            pose_msg.position.x = self.arm_x
            pose_msg.position.y = self.arm_y

            gripper_msg.data = self.gripper_state

            self.publisher.publish(twist_msg)
            self.arm_publisher.publish(pose_msg)
            self.gripper_publisher.publish(gripper_msg)

            rospy.sleep(self.timeout if self.timeout is not None else 0)

        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        twist_msg.linear.z = 0
        twist_msg.angular.x = 0
        twist_msg.angular.y = 0
        twist_msg.angular.z = 0
        self.publisher.publish(twist_msg)
        self.arm_publisher.publish(pose_msg)
        self.gripper_publisher.publish(gripper_msg)


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

def main():
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

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(0, 0, 0, 0, speed, turn, 0, 0, False)  # Khởi tạo các giá trị ban đầu

        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                arm_x = moveBindings[key][4] if len(moveBindings[key]) > 4 else 0
                arm_y = moveBindings[key][5] if len(moveBindings[key]) > 5 else 0
                gripper_state = False
                if len(moveBindings[key]) > 6:
                    gripper_state = moveBindings[key][6]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
            elif key in gripperBindings.keys():
                gripper_state = gripperBindings[key]
                print("Gripper state: {}".format("Closed" if gripper_state else "Opened"))
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0 and z == 0 and th == 0:
                    continue
                x = 0
                y = 0
                z = 0
                th = 0
                arm_x = 0
                arm_y = 0
                gripper_state = False
                if key == '\x03':
                    break

            pressed_keys.clear()  # Xóa tất cả các phím được nhấn
            if key:  # Nếu có phím được nhấn, thêm vào tập hợp
                pressed_keys.add(key)

            pub_thread.update(x, y, z, th, speed, turn, arm_x, arm_y, gripper_state)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)


if __name__ == "__main__":
    main()
