#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MecanumController:
    def __init__(self):
        rospy.init_node('mecanum_velocity_controller')

        # Robot dimensions (adjust to match your model)
        self.wheel_radius = 0.045
        self.wheel_base_length = 0.172  # front to back
        self.wheel_base_width = 0.172   # left to right

        # Publishers for each wheel controller
        self.pub_fl = rospy.Publisher('/front_left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_fr = rospy.Publisher('/front_right_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_rl = rospy.Publisher('/rear_left_wheel_velocity_controller/command', Float64, queue_size=10)
        self.pub_rr = rospy.Publisher('/rear_right_wheel_velocity_controller/command', Float64, queue_size=10)

        # Subscribe to velocity command
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        rospy.loginfo("Mecanum velocity controller node started.")

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x     # Forward
        vy = msg.linear.y     # Sideways
        omega = msg.angular.z # Rotation

        # Mecanum wheel inverse kinematics
        L = self.wheel_base_length
        W = self.wheel_base_width
        R = self.wheel_radius

        fl = (1 / R) * (vx - vy - (L + W) * omega)
        fr = (1 / R) * (vx + vy + (L + W) * omega)
        rl = (1 / R) * (vx + vy - (L + W) * omega)
        rr = (1 / R) * (vx - vy + (L + W) * omega)

        # Publish to wheel controllers
        self.pub_fl.publish(fl)
        self.pub_fr.publish(fr)
        self.pub_rl.publish(rl)
        self.pub_rr.publish(rr)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MecanumController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
