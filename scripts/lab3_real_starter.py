#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from sensor_msgs.msg import JointState  # Changed from SensorState
import tf
import math

class OdometryPublisher:
    def __init__(self):
        rospy.init_node("odometry_publisher", anonymous=True)
        self.odom_pub = rospy.Publisher("/custom_odom", Odometry, queue_size=10)
        
        # Subscribe to joint_states instead of sensor_state
        self.joint_sub = rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_pos = None
        self.last_right_pos = None
        
        self.wheel_radius = 0.033
        self.wheel_separation = 0.160
        
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

    def joint_callback(self, msg):
        # joint_states gives wheel positions in radians directly
        for i, name in enumerate(msg.name):
            if 'left' in name:
                self.left_wheel_pos = msg.position[i]
            elif 'right' in name:
                self.right_wheel_pos = msg.position[i]
        
        if self.last_left_pos is None:
            self.last_left_pos = self.left_wheel_pos
            self.last_right_pos = self.right_wheel_pos

    def update_odometry(self):
        if self.last_left_pos is None or self.last_right_pos is None:
            return
        
        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()
        
        if dt == 0:
            return
        
        # Positions are already in radians, no tick conversion needed
        delta_left_rad = self.left_wheel_pos - self.last_left_pos
        delta_right_rad = self.right_wheel_pos - self.last_right_pos
        
        d_left = delta_left_rad * self.wheel_radius
        d_right = delta_right_rad * self.wheel_radius
        
        delta_d = (d_right + d_left) / 2.0
        delta_theta = (d_right - d_left) / self.wheel_separation
        
        self.x += delta_d * math.cos(self.theta + delta_theta / 2.0)
        self.y += delta_d * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        self.last_left_pos = self.left_wheel_pos
        self.last_right_pos = self.right_wheel_pos
        
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"
        odom.pose.pose = Pose()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*odom_quat)
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist()
        odom.twist.twist.linear.x = delta_d / dt
        odom.twist.twist.angular.z = delta_theta / dt
        self.odom_pub.publish(odom)
        self.last_time = self.current_time

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_odometry()
            rate.sleep()

if __name__ == "__main__":
    try:
        print("Publishing odometry under /custom_odom...")
        odom_pub = OdometryPublisher()
        odom_pub.run()
    except rospy.ROSInterruptException:
        pass
