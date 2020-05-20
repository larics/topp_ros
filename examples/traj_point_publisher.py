#!/usr/bin/env python

import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


class TrajectoryPointPublisher:

    def __init__(self):        
        self.point_index = 0
        self.trajectory = MultiDOFJointTrajectory()
        self.pose_sub = rospy.Subscriber("input/pose", PoseStamped, self.pose_cb)
        self.carrot_status = String()
        self.carrot_status.data = "HOLD"
        self.status_sub = rospy.Subscriber("carrot/status", String, self.status_cb)
        self.odom_msg = Odometry()
        self.odom_flag = False
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_cb)
        self.point_pub = rospy.Publisher("output/point", MultiDOFJointTrajectoryPoint, queue_size=1)

    def status_cb(self, msg):
        self.carrot_status = msg
        if not self.carrot_status.data == "HOLD" and self.trajectory.points:
            print("TrajectoryPointPublisher - hold disabled, clearing trajectory!")
            self.trajectory.points = []

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.odom_flag = True

    def pose_cb(self, msg):
        print("TrajectoryPointPublisher: Received new pose")
        
        x = [self.odom_msg.pose.pose.position.x, msg.pose.position.x]
        y = [self.odom_msg.pose.pose.position.y, msg.pose.position.y]
        z = [self.odom_msg.pose.pose.position.z, msg.pose.position.z]

        yaw_start = tf.transformations.euler_from_quaternion(
            [self.odom_msg.pose.pose.orientation.x, 
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w])[2]
        yaw_end = tf.transformations.euler_from_quaternion(
            [msg.pose.orientation.x, 
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w])[2]
        yaw = [yaw_start, yaw_end]

         # Create a service request which will be filled with waypoints
        request = GenerateTrajectoryRequest()

        # Add waypoints in request
        waypoint = JointTrajectoryPoint()
        for i in range(0, len(x)):
            # Positions are defined above
            waypoint.positions = [x[i], y[i], z[i], yaw[i]]
            # Also add constraints for velocity and acceleration. These
            # constraints are added only on the first waypoint since the
            # TOPP-RA reads them only from there.
            if i==0:
                waypoint.velocities = [1, 1, 1, 0.5]
                waypoint.accelerations = [0.25, 0.25, 0.25, 0.125]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        # Set up joint names. This step is not necessary
        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        # Set up sampling frequency of output trajectory.
        request.sampling_frequency = 100.0
        # Set up number of gridpoints. The more gridpoints there are, 
        # trajectory interpolation will be more accurate but slower.
        # Defaults to 100
        request.n_gridpoints = 500
        # If you want to plot Maximum Velocity Curve and accelerations you can
        # send True in this field. This is intended to be used only when you
        # have to debug something since it will block the service until plot
        # is closed.
        request.plot = False
        # Request the trajectory
        request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)
        response = request_trajectory_service(request)

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.
        print ("TrajectoryPointPublisher: Converting trajectory to multi dof")
        joint_trajectory = response.trajectory
        self.trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)

    def JointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = 1.0

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[0]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[1]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[2]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[0]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[1]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[2]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)
            temp_point.time_from_start = joint_trajectory.points[i].time_from_start

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

    def run(self):
        while not rospy.is_shutdown():
            
            if not self.odom_flag:
                print("TrajectoryPointPublisher - odometry unavailable")
                rospy.sleep(0.5)
                continue

            if not self.carrot_status.data == "HOLD":
                print("TrajectoryPointPublisher - Position hold disabled")
                rospy.sleep(0.5)
                continue
            
            if not self.trajectory.points:
                print("TrajectoryPointPublisher - No trajectory available")
                rospy.sleep(0.5)
                continue

            # Publish trajectory point
            self.point_pub.publish(self.trajectory.points.pop(0))
            rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("trajectory_point_publisher")   
    tp = TrajectoryPointPublisher()
    tp.run()