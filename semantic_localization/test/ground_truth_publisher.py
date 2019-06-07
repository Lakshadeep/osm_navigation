#!/usr/bin/env python


# publishes ground truth for all the test data recorded in HBRS C022
# gazebo environment
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
import tf


class GT(object):

    def __init__(self):
        rospy.init_node('ground_truth_publisher')
        self.pub = rospy.Publisher('/gt', PoseStamped, queue_size=10)
        rospy.Subscriber("/ropod/odom", Odometry, self.odom_callback)

    def odom_callback(self, data):
        pose_msg = PoseStamped()
        pose_msg.pose = data.pose.pose
        pose_msg.pose.position.x = pose_msg.pose.position.x + 60
        pose_msg.pose.position.y = pose_msg.pose.position.y + 25
        pose_msg.header = data.header
        pose_msg.header.frame_id = 'map'
        quaternion = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y,
                      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # roll = euler[0]
        # pitch = euler[1]
        yaw = euler[2]
        rospy.loginfo("$GroundTruth:%.3f,%3f,%3f",
                      pose_msg.pose.position.x, pose_msg.pose.position.y, yaw)
        self.pub.publish(pose_msg)


if __name__ == '__main__':
    gt = GT()
    rospy.spin()
