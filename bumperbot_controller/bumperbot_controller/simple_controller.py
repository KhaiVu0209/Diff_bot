#!/usr/bin/env python3
import rclpy #  Thu vien ROS2 
from rclpy.node import Node # Thu vien tao mot ROS2 Node 
from rclpy.constants import S_TO_NS # Chuyen doi second sang nanosecond
from rclpy.time import Time # Xu ly thoi gian trong ROS2 
from std_msgs.msg import Float64MultiArray # Gui nhan du lieu dang mang so thuc 64 bit (van toc 2 banh xe)
from geometry_msgs.msg import Twist # Nhan du lieu van toc cua robot (linear va angular)
from sensor_msgs.msg import JointState # Nhan goc quay encoder cua 2 banh xe
from nav_msgs.msg import Odometry # Xuat vi tri va van toc cua robot
import numpy as np 
from tf2_ros import TransformBroadcaster # Phat TF2 
from geometry_msgs.msg import TransformStamped # Cau truc du lieu TF2 
import math
from tf_transformations import quaternion_from_euler # chuyen doi goc quay tu Euler sang Quaternion


class SimpleController(Node):

    def __init__(self):
        super().__init__("simple_controller") # Init node ROS2 ten la "simple_controller"
        self.declare_parameter("wheel_radius", 0.033) # Radius 
        self.declare_parameter("wheel_separation", 0.17) # Khoang cach giua 2 banh xe

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value 
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_)
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_)

        # State variables ( Bien trang thai cua robot)
        self.left_wheel_prev_pos_ = 0.0 # Vi tri góc quay bánh trái ở lần đọc trước
        self.right_wheel_prev_pos_ = 0.0 # Vi tri góc quay bánh phải ở lần đọc trước

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Publishers and Subscribers ( Ket noi Node voi he thong ROS2)
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(Twist, "bumperbot_controller/cmd_vel_unstamped", self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10)  #      
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10) # Xuat odometry 

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_) 

        # Fill the Odometry message with invariant parameters
        # self.odom_msg_ = Odometry()
        # self.odom_msg_.header.frame_id = "odom"
        # self.odom_msg_.child_frame_id = "base_footprint"
        # self.odom_msg_.pose.pose.orientation.x = 0.0
        # self.odom_msg_.pose.pose.orientation.y = 0.0
        # self.odom_msg_.pose.pose.orientation.z = 0.0
        # self.odom_msg_.pose.pose.orientation.w = 1.0

        # # Fill the TF message
        # self.br_ = TransformBroadcaster(self)
        # self.transform_stamped_ = TransformStamped()
        # self.transform_stamped_.header.frame_id = "odom"
        # self.transform_stamped_.child_frame_id = "base_footprint"

        # self.prev_time_ = self.get_clock().now()


    def velCallback(self, msg):
        # Implements the differential kinematic model
        # Given v and w, calculate the velocities of the wheels
        robot_speed = np.array([[msg.linear.x],
                                [msg.angular.z]])
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) 

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]

        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    
    def jointCallback(self, msg):
        # Implements the inverse differential kinematic model
        # Given the position of the wheels, calculates their velocities
        # then calculates the velocity of the robot wrt the robot frame
        # and then converts it in the global frame and publishes the TF

        dp_left = msg.position[1] - self.left_wheel_prev_pos_ # Vi tri goc quay banh trai hien tai tru di vi tri goc quay banh trai lan truoc
        dp_right = msg.position[0] - self.right_wheel_prev_pos_ # Vi tri goc quay banh phai hien tai tru di vi tri goc quay banh phai lan truoc
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ # Thoi gian hien tai tru di thoi gian lan truoc

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[1] # Vi tri goc quay banh trai hien tai luu vao bien luu tru 
        self.right_wheel_prev_pos_ = msg.position[0] # Vi tri goc quay banh phai hien tai luu vao bien luu tru 
        self.prev_time_ = Time.from_msg(msg.header.stamp) # Thoi gian hien tai luu vao bien luu tru

        # Calculate the rotational speed of each wheel (Tinh van toc goc quay cua moi banh xe)
        # Transform nanoseconds to seconds
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS) # Van toc goc quay banh trai
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS) # Van toc goc quay banh phai (rad/s)

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2   # Linear velocity
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_  # Angular velocity 
        self.get_logger().info(f"Linear velocity is {linear} m/s, angular velocity is {angular} rad/s")

        # Calculate the position increment 
        d_s = (
            self.wheel_radius_ * dp_right
            + self.wheel_radius_ * dp_left
        ) / 2

        # Calculate the orientation increment
        d_theta = (
            self.wheel_radius_ * dp_right
            - self.wheel_radius_ * dp_left
        ) / self.wheel_separation_

        # Update the position and orientation 
        self.theta_ += d_theta #
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_) 
        
        # Compose and publish the odom message
        # q = quaternion_from_euler(0, 0, self.theta_)
        # self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        # self.odom_msg_.pose.pose.position.x = self.x_
        # self.odom_msg_.pose.pose.position.y = self.y_
        # self.odom_msg_.pose.pose.orientation.x = q[0]
        # self.odom_msg_.pose.pose.orientation.y = q[1]
        # self.odom_msg_.pose.pose.orientation.z = q[2]
        # self.odom_msg_.pose.pose.orientation.w = q[3]
        # self.odom_msg_.twist.twist.linear.x = linear
        # self.odom_msg_.twist.twist.angular.z = angular
        # self.odom_pub_.publish(self.odom_msg_)

        # # TF
        # self.transform_stamped_.transform.translation.x = self.x_
        # self.transform_stamped_.transform.translation.y = self.y_
        # self.transform_stamped_.transform.rotation.x = q[0]
        # self.transform_stamped_.transform.rotation.y = q[1]
        # self.transform_stamped_.transform.rotation.z = q[2]
        # self.transform_stamped_.transform.rotation.w = q[3]
        # self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        # self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    
    simple_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()