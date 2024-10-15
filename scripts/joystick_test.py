#!/usr/bin/python3
import intera_interface
from intera_interface import Gripper

import argparse
import rospy
import intera_external_devices
from geometry_msgs.msg import Twist
from relaxed_ik_ros1.msg import EEVelGoals
from sensor_msgs.msg import Joy
from intera_interface import CHECK_VERSION, RobotEnable
from robot import Robot

class JoystickInput:
    def __init__(self, limb):
        # Robot setup
        self.robot = Robot(rospy.get_param('setting_file_path'))
        self.ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=5)

        # Control parameters
        self.pos_stride = 0.05
        self.rot_stride = 0.05
        self.linear = [0, 0, 0]   # x, y, z for linear velocity
        self.angular = [0, 0, 0]  # x, y, z for angular velocity

        self.limb = limb
        self.gripper = None
        try:
            self.gripper = intera_interface.Gripper(limb + '_gripper')
        except:
            rospy.loginfo("Could not detect a connected electric gripper.")

        # Initialize joystick
        rospy.Subscriber("joy", Joy, self.joy_callback)
        rospy.Timer(rospy.Duration(0.033), self.timer_callback)

    def joy_callback(self, joy_msg):
        """
        Maps joystick input to velocity commands.
        - Left joystick controls linear velocities
        - Right joystick controls angular velocities
        - Buttons control the gripper
        """
        self.linear[0] = joy_msg.axes[1] * self.pos_stride  # Forward/backward
        self.linear[1] = joy_msg.axes[0] * self.pos_stride  # Left/right
        self.linear[2] = joy_msg.axes[4] * self.pos_stride  # Up/down

        self.angular[0] = joy_msg.axes[3] * self.rot_stride  # Rotation around x-axis
        self.angular[1] = joy_msg.axes[2] * self.rot_stride  # Rotation around y-axis
        self.angular[2] = joy_msg.buttons[4] * self.rot_stride - joy_msg.buttons[5] * self.rot_stride  # Rotation around z-axis

        # Gripper control
        if joy_msg.buttons[6]:  # Button for closing the gripper
            if self.gripper:
                self.gripper.close()
        elif joy_msg.buttons[7]:  # Button for opening the gripper
            if self.gripper:
                self.gripper.open()

    def timer_callback(self, event):
        """
        Publishes end-effector velocities periodically based on joystick input.
        """
        msg = EEVelGoals()

        for _ in range(self.robot.num_chain):
            twist = Twist()
            twist.linear.x = self.linear[0]
            twist.linear.y = self.linear[1]
            twist.linear.z = self.linear[2]

            twist.angular.x = self.angular[0]
            twist.angular.y = self.angular[1]
            twist.angular.z = self.angular[2]

            tolerance = Twist()
            msg.ee_vels.append(twist)
            msg.tolerances.append(tolerance)

        self.ee_vel_goals_pub.publish(msg)

def main():
    """
    Main function to initialize the joystick control for the robot's
    end-effector and gripper.
    """
    rospy.init_node("joystick_control_node")
    
    # Parse arguments for limb choice and joystick type
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        '-j', '--joystick', required=True, choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    parser.add_argument(
        "-l", "--limb", dest="limb", default="right",
        choices=['right', 'left'],
        help="Limb on which to run the joint position joystick example"
    )
    args = parser.parse_args(rospy.myargv()[1:])
    
    # Setup the joystick
    joystick = None
    if args.joystick == 'xbox':
        joystick = intera_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = intera_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = intera_external_devices.joystick.PS3Controller()

    # Enable the robot
    rs = RobotEnable(CHECK_VERSION)
    rs.enable()

    # Initialize joystick control
    joystick_input = JoystickInput(args.limb)
    rospy.spin()

if __name__ == '__main__':
    main()
