#!/usr/bin/env python
from __future__ import print_function
import copy, time, math

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist
from std_msgs.msg import Int32
# Matlab plot and numpy
import matplotlib.pyplot as plt
import numpy as np

class RequestTrajectory():

    def __init__(self):
        # First set up service
        self.request_trajectory_service = rospy.ServiceProxy(
            "generate_toppra_trajectory", GenerateTrajectory)

        self.trajectory_pub = rospy.Publisher('uav_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        self.ugv_trajectory_pub = rospy.Publisher('ugv_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        self.joint_trajectory_pub = rospy.Publisher('joint_trajectory', 
            JointTrajectory, queue_size=1)
        self.manipulator_trajectory_pub = rospy.Publisher('manipulator_trajectory', 
            MultiDOFJointTrajectory, queue_size=1)
        #time.sleep(0.5)

        generate_trajectory_sub = rospy.Subscriber('generate_trajectory', 
            Int32, self.generateTrajectoryCallback, queue_size=1)

    def run(self):
        rospy.spin()

    def jointTrajectory2MultiDofTrajectory(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[0]
            temp_transform.translation.y = joint_trajectory.points[i].positions[1]
            temp_transform.translation.z = joint_trajectory.points[i].positions[2]
            temp_transform.rotation.w = math.cos(joint_trajectory.points[i].positions[3]/2.0)
            temp_transform.rotation.z = math.sin(joint_trajectory.points[i].positions[3]/2.0)

            # Override the rotation
            """
            if (i < (len(joint_trajectory.points)-10)):
                dx = joint_trajectory.points[i+10].positions[0] - joint_trajectory.points[i].positions[0]
                dy = joint_trajectory.points[i+10].positions[1] - joint_trajectory.points[i].positions[1]
                yaw = math.atan2(dy, dx)
                temp_transform.rotation.z = math.sin(yaw/2)
                temp_transform.rotation.w = math.cos(yaw/2)
            elif (i < (len(joint_trajectory.points)-1)):
                dx = joint_trajectory.points[len(joint_trajectory.points)-1].positions[0] - joint_trajectory.points[i].positions[0]
                dy = joint_trajectory.points[len(joint_trajectory.points)-1].positions[1] - joint_trajectory.points[i].positions[1]
                yaw = math.atan2(dy, dx)
                temp_transform.rotation.z = math.sin(yaw/2)
                temp_transform.rotation.w = math.cos(yaw/2)
            else:
                # same as last one
                temp_transform.rotation.z = multi_dof_trajectory.points[i-1].transforms[0].rotation.z
                temp_transform.rotation.w = multi_dof_trajectory.points[i-1].transforms[0].rotation.w
            """
            

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


    def jointTrajectory2MultiDofTrajectoryUgv(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[4]
            temp_transform.translation.y = joint_trajectory.points[i].positions[5]
            temp_transform.translation.z = 0.0
            temp_transform.rotation.z = math.sin(joint_trajectory.points[i].positions[3]/2.0)
            temp_transform.rotation.w = math.cos(joint_trajectory.points[i].positions[3]/2.0)
            

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[4]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[5]
            temp_vel.linear.z = 0.0

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[4]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[5]
            temp_acc.linear.z = 0.0

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory

    def jointTrajectory2MultiDofTrajectoryManipulator(self, joint_trajectory):
        multi_dof_trajectory = MultiDOFJointTrajectory()

        for i in range(0, len(joint_trajectory.points)):
            temp_point = MultiDOFJointTrajectoryPoint()
            temp_transform = Transform()
            temp_transform.translation.x = joint_trajectory.points[i].positions[6]
            temp_transform.translation.y = joint_trajectory.points[i].positions[7]
            temp_transform.translation.z = joint_trajectory.points[i].positions[8]
            temp_transform.rotation.w = math.cos(joint_trajectory.points[i].positions[9]/2.0)
            temp_transform.rotation.z = math.sin(joint_trajectory.points[i].positions[9]/2.0)

            # Override the rotation
            """
            if (i < (len(joint_trajectory.points)-10)):
                dx = joint_trajectory.points[i+10].positions[0] - joint_trajectory.points[i].positions[0]
                dy = joint_trajectory.points[i+10].positions[1] - joint_trajectory.points[i].positions[1]
                yaw = math.atan2(dy, dx)
                temp_transform.rotation.z = math.sin(yaw/2)
                temp_transform.rotation.w = math.cos(yaw/2)
            elif (i < (len(joint_trajectory.points)-1)):
                dx = joint_trajectory.points[len(joint_trajectory.points)-1].positions[0] - joint_trajectory.points[i].positions[0]
                dy = joint_trajectory.points[len(joint_trajectory.points)-1].positions[1] - joint_trajectory.points[i].positions[1]
                yaw = math.atan2(dy, dx)
                temp_transform.rotation.z = math.sin(yaw/2)
                temp_transform.rotation.w = math.cos(yaw/2)
            else:
                # same as last one
                temp_transform.rotation.z = multi_dof_trajectory.points[i-1].transforms[0].rotation.z
                temp_transform.rotation.w = multi_dof_trajectory.points[i-1].transforms[0].rotation.w
            """
            

            temp_vel = Twist()
            temp_vel.linear.x = joint_trajectory.points[i].velocities[6]
            temp_vel.linear.y = joint_trajectory.points[i].velocities[7]
            temp_vel.linear.z = joint_trajectory.points[i].velocities[8]

            temp_acc = Twist()
            temp_acc.linear.x = joint_trajectory.points[i].accelerations[6]
            temp_acc.linear.y = joint_trajectory.points[i].accelerations[7]
            temp_acc.linear.z = joint_trajectory.points[i].accelerations[8]

            temp_point.transforms.append(temp_transform)
            temp_point.velocities.append(temp_vel)
            temp_point.accelerations.append(temp_acc)

            multi_dof_trajectory.points.append(temp_point)

        return multi_dof_trajectory
    def plotTrajectory(self, request, response):
        n = len(response.trajectory.points)
        t = [0]*n
        x_uav = [0]*n
        y_uav = [0]*n
        z_uav = [0]*n
        yaw_uav = [0]*n
        x_ugv = [0]*n
        y_ugv = [0]*n
        for i in range(n):
            t[i] = float(i)*0.01
            x_uav[i] = response.trajectory.points[i].positions[0]
            y_uav[i] = response.trajectory.points[i].positions[1]
            z_uav[i] = response.trajectory.points[i].positions[2]
            yaw_uav[i] = response.trajectory.points[i].positions[3]
            x_ugv[i] = response.trajectory.points[i].positions[4]
            y_ugv[i] = response.trajectory.points[i].positions[5]

        # waypoints
        m = len(request.waypoints.points)
        x_uav_wp = [0]*m
        y_uav_wp = [0]*m
        z_uav_wp = [0]*m
        yaw_uav_wp = [0]*m
        x_ugv_wp = [0]*m
        y_ugv_wp = [0]*m
        for i in range(m):
            x_uav_wp[i] = request.waypoints.points[i].positions[0]
            y_uav_wp[i] = request.waypoints.points[i].positions[1]
            z_uav_wp[i] = request.waypoints.points[i].positions[2]
            yaw_uav_wp[i] = request.waypoints.points[i].positions[3]
            x_ugv_wp[i] = request.waypoints.points[i].positions[4]
            y_ugv_wp[i] = request.waypoints.points[i].positions[5]

        # Finally plot
        fig = plt.figure(1)
        plt.plot(x_uav, y_uav, 'b')
        plt.grid(True)
        plt.hold(True)
        plt.plot(x_ugv, y_ugv, 'r')
        plt.plot(x_uav_wp, y_uav_wp, 'bx', linewidth=3, markersize=10)
        plt.plot(x_ugv_wp, y_ugv_wp, 'rx', linewidth=3, markersize=10)
        plt.xlabel('x[m]')
        plt.ylabel('y[m]')
        plt.xlim([-1.5, 1.5])

        # Plot yaw
        fig = plt.figure(2)
        plt.grid(True)
        plt.plot(t, yaw_uav, 'b')
        plt.xlabel('time[s]')
        plt.ylabel('yaw[rad]')

        # Plot yaw
        fig = plt.figure(3)
        plt.grid(True)
        plt.plot(t, x_uav, 'b')
        plt.xlabel('time[s]')
        plt.ylabel('x_uav[m]')

        # Plot yaw
        fig = plt.figure(4)
        plt.grid(True)
        plt.plot(t, y_uav, 'b')
        plt.xlabel('time[s]')
        plt.ylabel('y_uav[m]')

        plt.show(block=False)
        input()


    def generateTrajectoryCallback(self, msg):
        print("Generating trajectory")
        # Uav + ugv for westpoint, circular trajectory
        """
        x_uav = []
        y_uav = []
        z_uav = []
        yaw_uav = []
        x_ugv = []
        y_ugv = []
        amp_uav = 0.5
        amp_ugv = 0.5
        total_angle = 2*math.pi
        n = 111
        for i in range(0,n):
            angle = i*total_angle/float(n-1)
            x_uav.append(amp_uav*math.cos(angle))
            y_uav.append(amp_uav*math.sin(angle))
            z_uav.append(1.0)
            yaw_uav.append(0.0)
            x_ugv.append(amp_ugv*math.cos(angle))
            y_ugv.append(amp_ugv*math.sin(angle))

        for i in range(0,n-1):
            dx = x_uav[i+1]-x_uav[i]
            dy = y_uav[i+1]-y_uav[i]
            yaw_uav[i] = math.atan2(dy, dx)

        for i in range(1, n-1):
            if yaw_uav[i] < yaw_uav[i-1]:
                yaw_uav[i] = yaw_uav[i] + 2*math.pi
        yaw_uav[n-1] = yaw_uav[n-2]
        """

        # Uav + ugv + manipulator trajectory
        x_uav = []
        y_uav = []
        z_uav = []
        yaw_uav = []
        x_ugv = []
        y_ugv = []
        x_start = -0.5
        x_end = 0.5
        y_start = 0.0
        y_end = 0.0
        x_offset = 0.2597
        y_offset = -0.1777
        z_offset = -0.22
        yaw_offset = -math.pi/2.0
        n = 111

        for i in range(0,n):
            dx = x_end - x_start
            dy = y_end - y_start
            x_uav.append(dx*float(i)/float(n)+x_start)
            y_uav.append(dy*float(i)/float(n)+y_start)
            z_uav.append(1.0)
            yaw_uav.append(0.0)
            x_ugv.append(dx*float(i)/float(n)+x_start)
            y_ugv.append(dy*float(i)/float(n)+y_start)

        for i in range(0,n-1):
            dx = x_uav[i+1]-x_uav[i]
            dy = y_uav[i+1]-y_uav[i]
            yaw_uav[i] = math.atan2(dy, dx)

        for i in range(1, n-1):
            if yaw_uav[i] < yaw_uav[i-1]:
                yaw_uav[i] = yaw_uav[i] + 2*math.pi
        yaw_uav[n-1] = yaw_uav[n-2]
        



        # Create a service request which will be filled with waypoints
        request = GenerateTrajectoryRequest()

        waypoint = JointTrajectoryPoint()
        for i in range(0, n):
            waypoint.positions = [x_uav[i], y_uav[i], z_uav[i], yaw_uav[i], x_ugv[i], y_ugv[i], (x_uav[i]+x_offset), (y_uav[i]+y_offset), (z_uav[i]+z_offset), (yaw_uav[i]+yaw_offset)]
            waypoint.velocities = [1.2, 1.2, 1.2, 4.6, 0.25, 0.25, 1.2, 1.2, 1.2, 4.6]
            waypoint.accelerations = [0.8, 0.8, 0.8, 4.0, 0.04, 0.04, 0.8, 0.8, 0.8, 4.0]

            request.waypoints.points.append(copy.deepcopy(waypoint))

        request.waypoints.joint_names = ["x_uav", "y_uav", "z_uav", "yaw_uav", "x_ugv", "y_ugv", "x_man", "y_man", "z_man", "yaw_man"]
        request.sampling_frequency = 100.0
        response = self.request_trajectory_service(request)
        print("Total trajectory time: ", len(response.trajectory.points)/request.sampling_frequency)

        print("Converting trajectory to multi dof")
        joint_trajectory = response.trajectory
        multi_dof_trajectory = self.jointTrajectory2MultiDofTrajectory(joint_trajectory)
        self.trajectory_pub.publish(multi_dof_trajectory)
        ugv_trajectory = self.jointTrajectory2MultiDofTrajectoryUgv(joint_trajectory)
        self.ugv_trajectory_pub.publish(ugv_trajectory)
        manipulator_trajectory = self.jointTrajectory2MultiDofTrajectoryManipulator(joint_trajectory)
        self.manipulator_trajectory_pub.publish(manipulator_trajectory)

        if msg.data == 1:
            self.plotTrajectory(request, response)

if __name__ == "__main__":
    rospy.init_node("toppra_multiple_robots_call_example_plot")
    a = RequestTrajectory()
    a.run()