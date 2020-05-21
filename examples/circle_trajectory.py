#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, Twist
import math
import matplotlib.pyplot as plt
import tf
import numpy as np

def to_trajectory_point_msg_q(x, y, z, qx, qy, qz, qw):
  point_msg = MultiDOFJointTrajectoryPoint()
  
  point_msg.transforms = [Transform()]
  point_msg.velocities = [Twist()]
  point_msg.accelerations = [Twist()]

  point_msg.transforms[0].translation.x = x
  point_msg.transforms[0].translation.y = y
  point_msg.transforms[0].translation.z = z

  point_msg.transforms[0].rotation.w = qw
  point_msg.transforms[0].rotation.x = qx
  point_msg.transforms[0].rotation.y = qy
  point_msg.transforms[0].rotation.z = qz
  
  return point_msg

def to_trajectory_point_msg(x, y, z, yaw):
  point_msg = MultiDOFJointTrajectoryPoint()
  
  point_msg.transforms = [Transform()]
  point_msg.velocities = [Twist()]
  point_msg.accelerations = [Twist()]

  point_msg.transforms[0].translation.x = x
  point_msg.transforms[0].translation.y = y
  point_msg.transforms[0].translation.z = z

  q = quaternion_from_euler(0, 0, yaw, )
  point_msg.transforms[0].rotation.w = q[3]
  point_msg.transforms[0].rotation.x = q[0]
  point_msg.transforms[0].rotation.y = q[1]
  point_msg.transforms[0].rotation.z = q[2]
  
  return point_msg

def publish_circle_once(circle_radus, number_of_points):
  
  odom_msg = Odometry()
  try:
    print("Waiting for Odometry message...")
    odom_msg = rospy.wait_for_message("mavros/global_position/local", Odometry, 5.0)
  except rospy.ROSException:
    print("No odometry message recieved returning...")
    return

  print("Generating circle trajectory...")
  trajectory_msg = MultiDOFJointTrajectory()
  trajectory_msg.header.stamp = rospy.Time.now()
  
  deg_to_rad = math.pi / 180.0
  angle_increment = 360.0 / number_of_points * deg_to_rad

  xs = []
  ys = []
  yaws = np.linspace(0, np.pi, number_of_points + 1)
  for i in range(number_of_points + 1):
    circle_x = odom_msg.pose.pose.position.x - circle_radus + circle_radus * math.cos(i * angle_increment)
    circle_y = odom_msg.pose.pose.position.y + circle_radus * math.sin(i * angle_increment)
    
    yaw_angle = yaws[i]
    print(yaw_angle)
    q = tf.transformations.quaternion_from_euler(0, 0, yaw_angle)
    print(q)
    trajectory_msg.points.append(
      to_trajectory_point_msg_q(
        circle_x,
        circle_y,
        odom_msg.pose.pose.position.z + math.sin(i * angle_increment),
        q[0], q[1], q[2], q[3]
      )
    )
    xs.append(circle_x)
    ys.append(circle_y)
    
  trajectory_pub = rospy.Publisher('topp/input/trajectory', MultiDOFJointTrajectory, queue_size=1)
  while trajectory_pub.get_num_connections() < 1:
    print("Waiting for someone to listen to the circle trajectory...")
    rospy.sleep(1.0)

  trajectory_pub.publish(trajectory_msg)
  print("Trajectory successfully published.")
  

if __name__ == "__main__":
  rospy.init_node("circle_generator_node")
  publish_circle_once(3, 100)

  