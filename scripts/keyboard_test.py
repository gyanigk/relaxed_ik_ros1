#!/usr/bin/env python3

import rospy
import intera_interface
from intera_interface import Gripper
from relaxed_ik_ros1.msg import EEPoseGoals, EEVelGoals
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
from robot import Robot
from intera_interface import CHECK_VERSION, RobotEnable
import rospkg

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

class KeyboardControl:
    def __init__(self):
        rospy.init_node('keyboard_control', anonymous=True)
        default_setting_file_path = path_to_src + '/configs/settings.yaml'
        setting_file_path = rospy.get_param('setting_file_path', default_setting_file_path)
        self.robot = Robot(setting_file_path)
        self.ee_pose_goals_pub = rospy.Publisher('relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=10)
        self.ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=5)
        self.pose = Pose()
        self.stride = 0.01  # Movement stride in meters
        self.angle_stride = 0.05  # Rotation stride in radians

        # Initialize Sawyer's limb and gripper
        self.rs = intera_interface.RobotEnable(CHECK_VERSION)
        self.init_state = self.rs.state().enabled
        self.rs.enable()
        self.limb = intera_interface.Limb('right')
        try:
            self.gripper = intera_interface.Gripper('right_gripper')
        except:
            rospy.loginfo("Could not detect a connected electric gripper.")
            self.gripper = None

        # Initialize current pose
        current_pose = self.limb.endpoint_pose()
        self.pose.position.x = current_pose['position'].x
        self.pose.position.y = current_pose['position'].y
        self.pose.position.z = current_pose['position'].z
        self.pose.orientation.x = current_pose['orientation'].x
        self.pose.orientation.y = current_pose['orientation'].y
        self.pose.orientation.z = current_pose['orientation'].z
        self.pose.orientation.w = current_pose['orientation'].w

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def update_pose(self):
        ee_pose_goals = EEPoseGoals()
        ee_pose_goals.ee_poses.append(self.pose)
        self.ee_pose_goals_pub.publish(ee_pose_goals)

    def update_velocity(self, linear, angular):
        ee_vel_goals = EEVelGoals()
        ee_vel_goals.ee_vels = linear + angular
        self.ee_vel_goals_pub.publish(ee_vel_goals)

    def print_status(self):
        print("\nCurrent Sawyer Status:")
        print("Enabled:", self.rs.state().enabled)
        print("Stopped:", self.rs.state().stopped)
        print("Error:", self.rs.state().error)
        print("Estop:", self.rs.state().estop_button)
        
        print("\nCurrent Joint Positions:")
        joint_angles = self.limb.joint_angles()
        for joint, angle in joint_angles.items():
            print(f"{joint}: {angle:.2f}")
        
        print("\nCurrent End Effector Pose:")
        current_pose = self.limb.endpoint_pose()
        print(f"Position: x={current_pose['position'].x:.3f}, y={current_pose['position'].y:.3f}, z={current_pose['position'].z:.3f}")
        print(f"Orientation: x={current_pose['orientation'].x:.3f}, y={current_pose['orientation'].y:.3f}, z={current_pose['orientation'].z:.3f}, w={current_pose['orientation'].w:.3f}")

    def map_keyboard(self):
        return {
            'w': (self.stride, 0, 0, 0, 0, 0),
            's': (-self.stride, 0, 0, 0, 0, 0),
            'a': (0, self.stride, 0, 0, 0, 0),
            'd': (0, -self.stride, 0, 0, 0, 0),
            'q': (0, 0, self.stride, 0, 0, 0),
            'e': (0, 0, -self.stride, 0, 0, 0),
            'i': (0, 0, 0, self.angle_stride, 0, 0),
            'k': (0, 0, 0, -self.angle_stride, 0, 0),
            'j': (0, 0, 0, 0, self.angle_stride, 0),
            'l': (0, 0, 0, 0, -self.angle_stride, 0),
            'u': (0, 0, 0, 0, 0, self.angle_stride),
            'o': (0, 0, 0, 0, 0, -self.angle_stride),
        }

    def run(self):
        print("Controlling robot. Press ? for help, Esc to quit.")
        keyboard_map = self.map_keyboard()
        while not rospy.is_shutdown():
            c = self.get_key()
            if c:
                if c in ['\x1b', '\x03']:
                    rospy.signal_shutdown("Exiting...")
                    return
                elif c in keyboard_map:
                    linear = keyboard_map[c][:3]
                    angular = keyboard_map[c][3:]
                    self.update_velocity(linear, angular)
                    self.update_pose()
                elif c == 'g':
                    if self.gripper:
                        self.gripper.close()
                elif c == 'h':
                    if self.gripper:
                        self.gripper.open()
                elif c == 'p':
                    self.print_status()
                elif c == '?':
                    print("\nKeyboard Mapping:")
                    print("w/s: +/- x axis")
                    print("a/d: +/- y axis")
                    print("q/e: +/- z axis")
                    print("i/k: +/- rotation around x")
                    print("j/l: +/- rotation around y")
                    print("u/o: +/- rotation around z")
                    print("g: Close gripper")
                    print("h: Open gripper")
                    print("p: Print current robot status")
                    print("Esc: Quit")
                else:
                    print("Invalid input. Press ? for help.")

if __name__ == '__main__':
    controller = None
    try:
        controller = KeyboardControl()
        controller.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if controller and not controller.init_state:
            controller.rs.disable()
