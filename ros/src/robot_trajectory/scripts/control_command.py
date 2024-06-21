#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
import sys
import select
import termios
import tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    pub = rospy.Publisher('control_command', Int32, queue_size=10)
    rospy.init_node('control_command_node', anonymous=True)
    
    # Get the default value from parameter server
    default_value = rospy.get_param('~default_value', 0)
    
    rate = rospy.Rate(100)  # 100 Hz

    # Print instructions to the screen
    print("Press '0' to send 0 to Arduino.")
    print("Press '1' to send 1 to Arduino.")
    print("Press 'Ctrl+C' to exit.")
    print(f"Default value is set to {default_value}.")

    pub.publish(default_value)  # Publish the default value at startup

    while not rospy.is_shutdown():
        key = getKey()
        if key == '0':
            rospy.loginfo("Sending 0 to Arduino")
            pub.publish(0)
        elif key == '1':
            rospy.loginfo("Sending 1 to Arduino")
            pub.publish(1)
        elif key == '\x03':  # Ctrl+C to exit
            break

        rate.sleep()

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)  # Save the current terminal settings
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Restore the terminal settings

# Print some instructions when the script is started
print("Starting control_command_node...")
print("Use the following keys to send commands to Arduino:")
print("Press '0' to send 0")
print("Press '1' to send 1")
print("Press 'Ctrl+C' to exit")
