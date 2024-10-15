#!/usr/bin/env python3

import rospy
import rospkg
from geometry_msgs.msg import Twist
from relaxed_ik_ros1.msg import EEVelGoals
from robot import Robot
from intera_interface import Limb, Gripper
import intera_external_devices

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1') + '/relaxed_ik_core'

class RelaxedIKJoystickControl:
    def __init__(self, joystick_type, limb_name):
        self.robot = Robot(path_to_src + '/configs/settings.yaml')
        self.limb = Limb(limb_name)
        self.gripper = Gripper(limb_name + '_gripper')
        
        self.ee_vel_goals_pub = rospy.Publisher('relaxed_ik/ee_vel_goals', EEVelGoals, queue_size=5)
        
        if joystick_type == 'xbox':
            self.joystick = intera_external_devices.joystick.XboxController()
        elif joystick_type == 'logitech':
            self.joystick = intera_external_devices.joystick.LogitechController()
        elif joystick_type == 'ps3':
            self.joystick = intera_external_devices.joystick.PS3Controller()
        else:
            raise ValueError("Unsupported joystick type")

        self.linear_scale = 0.1
        self.angular_scale = 0.1

    def map_joystick(self):
        twist = Twist()
        
        # Map left stick to linear velocity
        twist.linear.x = self.joystick.stick_value('leftStickVert') * self.linear_scale
        twist.linear.y = self.joystick.stick_value('leftStickHorz') * self.linear_scale
        
        # Map right stick to angular velocity
        twist.angular.x = self.joystick.stick_value('rightStickVert') * self.angular_scale
        twist.angular.y = self.joystick.stick_value('rightStickHorz') * self.angular_scale
        
        # Map triggers to z-axis movement
        try:
            right_trigger_value = self.joystick.stick_value('rightTrigger')
            left_trigger_value = self.joystick.stick_value('leftTrigger')
            twist.linear.z = (right_trigger_value - left_trigger_value) * self.linear_scale
        except KeyError:
            twist.linear.z = 0
        
        # Map bumpers to z-axis rotation
        if self.joystick.button_down('rightBumper'):
            twist.angular.z = self.angular_scale
        elif self.joystick.button_down('leftBumper'):
            twist.angular.z = -self.angular_scale
        
        return twist

    def gripper_control(self):
        if self.joystick.button_down('btnA'):
            self.gripper.close()
        elif self.joystick.button_down('btnB'):
            self.gripper.open()

    def run(self):
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            twist = self.map_joystick()
            
            ee_vel_goals = EEVelGoals()
            ee_vel_goals.ee_vels.append(twist)
            
            self.ee_vel_goals_pub.publish(ee_vel_goals)
            self.gripper_control()
            
            rate.sleep()

def main():
    rospy.init_node("relaxed_ik_joystick_control")
    
    joystick_type = rospy.get_param('~joystick_type', 'xbox')
    limb_name = rospy.get_param('~limb', 'right')
    
    controller = RelaxedIKJoystickControl(joystick_type, limb_name)
    controller.run()

if __name__ == '__main__':
    main()
