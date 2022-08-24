#!/usr/bin/env python3
from __future__ import print_function
import copy, time

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist

class RequestTrajectory():

    def __init__(self):
        # First set up service
        request_trajectory_service = rospy.ServiceProxy(
            "generate_topp_trajectory", GenerateTrajectory)

        trajectory_pub = rospy.Publisher('multi_dof_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)

        # Example of a simple square trajectory for quadcopter. All vectors
        # must be of same length
        x = [0, 1, 1, 0, 0]
        y = [0, 0, 1, 1, 0]
        z = [1, 1, 1, 1, 1]
        yaw = [0, 0, 0, 0, 0]
        # Another example. Same square defined through more points
        x = [0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 0.0, 0.0, 0.0]
        y = [0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 0.5, 0.0]
        z = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        yaw = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        # Create a service request which will be filled with waypoints
        request = GenerateTrajectoryRequest()

        waypoint = JointTrajectoryPoint()
        for i in range(0, len(x)):
            waypoint.positions = [x[i], y[i], z[i], yaw[i]]
            waypoint.velocities = [2, 2, 2, 1]
            waypoint.accelerations = [2, 2, 2, 1]

            request.waypoints.points.append(copy.deepcopy(waypoint))

        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        request.sampling_frequency = 100.0
        response = request_trajectory_service(request)

        print("Converting trajectory to multi dof")
        joint_trajectory = response.trajectory
        multi_dof_trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)
        trajectory_pub.publish(multi_dof_trajectory)

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

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory


if __name__ == "__main__":
    rospy.init_node("topp_trajectory_call_example")
    RequestTrajectory()