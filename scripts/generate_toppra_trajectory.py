#!/usr/bin/env python3
from __future__ import print_function

# Toppra imports
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time
import copy

# Ros imports
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from topp_ros.srv import GenerateTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ToppraTrajectory(Node):

    def __init__(self):
        # Basically we have just one service waiting for request and outputting
        # trajectory
        super().__init__("ToppraTrajectory")

        self.generate_toppra_trajectory_service = self.create_service(
            GenerateTrajectory, 'generate_toppra_trajectory', self.generateToppraTrajectoryCallback)

        self.raw_trajectory_pub = self.create_publisher(JointTrajectory, 'toppra_raw_trajectory', 1)
        self.raw_waypoints_pub = self.create_publisher(JointTrajectory, 'toppra_raw_waypoints', 1)
        
        # Create a timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        # Nothing special, just waiting for service request
        a = 5

    def generateToppraTrajectoryCallback(self, req, res):
        print(" ")
        print("Generating TOPP-RA trajectory.")
        tstart = time.time()
        dof = len(req.waypoints.points[0].positions)
        n = len(req.waypoints.points)

        # If there is not enough waypoints to generate a trajectory return false
        if (n <= 1 or dof == 0):
            print("You must provide at least 2 points to generate a valid trajectory.")
            res.trajectory.success = False
            return res

        # Generate trajectory.
        # First set up waypoints. We know hom many will be from n and dof.
        way_pts = np.zeros([n, dof])
        # Next fill the waypoints with data from request.
        for i in range(0, n):
            for j in range(0, dof):
                way_pts[i][j] = req.waypoints.points[i].positions[j]

        # Part of TOPP-RA is to generate path(s \in [0,1]) from n waypoints.
        # The algorithm then parametrizes the initial path.
        path = ta.SplineInterpolator(np.linspace(0, 1, n), way_pts)

        # Create velocity and acceleration bounds. Supposing symmetrical bounds around zero.
        vlim_ = np.zeros([dof])
        alim_ = np.zeros([dof])
        for i in range(0, dof):
            vlim_[i] = req.waypoints.points[0].velocities[i]
            alim_[i] = req.waypoints.points[0].accelerations[i]
        vlim = np.vstack((-vlim_, vlim_)).T
        alim = np.vstack((-alim_, alim_)).T
        pc_vel = constraint.JointVelocityConstraint(vlim)
        pc_acc = constraint.JointAccelerationConstraint(
            alim, discretization_scheme=constraint.DiscretizationType.Interpolation)

        # Setup a parametrization instance
        if (req.n_gridpoints <= 0):
            num_grid_points = np.max([100, n*2])
            gridpoints = np.linspace(0, path.duration, num_grid_points)
        else:
            gridpoints = np.linspace(0, path.duration, req.n_gridpoints)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, gridpoints=gridpoints, solver_wrapper='seidel')
        print ("instance:", instance.compute_trajectory(0, 0))
        # Retime the trajectory, only this step is necessary.
        t0 = time.time()
        jnt_traj = instance.compute_trajectory(0, 0)
        #print("Parameterization time: {:} secs".format(time.time() - t0))

        # Plot for debugging
        if req.plot == True:
            print("Parameterization time: {:} secs".format(time.time() - t0))
            ts_sample = np.linspace(0, jnt_traj.get_duration(), 100)
            qs_sample = jnt_traj.eval(ts_sample)
            qds_sample = jnt_traj.evald(ts_sample)
            qdds_sample = jnt_traj.evaldd(ts_sample)

            plt.plot(ts_sample, qdds_sample)
            plt.xlabel("Time (s)")
            plt.ylabel("Joint acceleration (rad/s^2)")
            plt.show()

            # Compute the feasible sets and the controllable sets for viewing.
            # Note that these steps are not necessary.
            _, sd_vec, _ = instance.compute_parameterization(0, 0)
            X = instance.compute_feasible_sets()
            K = instance.compute_controllable_sets(0, 0)

            X = np.sqrt(X)
            K = np.sqrt(K)

            plt.plot(X[:, 0], c='green', label="Feasible sets")
            plt.plot(X[:, 1], c='green')
            plt.plot(K[:, 0], '--', c='red', label="Controllable sets")
            plt.plot(K[:, 1], '--', c='red')
            plt.plot(sd_vec, label="Velocity profile")
            plt.title("Path-position path-velocity plot")
            plt.xlabel("Path position")
            plt.ylabel("Path velocity square")
            plt.legend()
            plt.tight_layout()
            plt.show()

            plt.plot(qs_sample[:,0], qs_sample[:,1])
            plt.show()

        # Convert to JointTrajectory message
        res.trajectory = self.TOPPRA2JointTrajectory(jnt_traj, req.sampling_frequency)
        res.success = True
        if len(req.waypoints.joint_names) != 0:
            res.trajectory.joint_names = copy.deepcopy(req.waypoints.joint_names)
        self.raw_trajectory_pub.publish(res.trajectory)
        self.raw_waypoints_pub.publish(req.waypoints)
        print("Time elapsed: ", time.time()-tstart)
        return res

    def TOPPRA2JointTrajectory(self, jnt_traj, f):
        # Sampling frequency is required to get the time samples correctly.
        # The number of points in ts_sample is duration*frequency.
        ts_sample = np.linspace(0, jnt_traj.get_duration(), 
            int(jnt_traj.get_duration()*f))
        # Sampling. This returns a matrix for all DOFs. Accessing specific one is 
        # simple: qs_sample[:, 0]
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)

        n = qs_sample.shape[0]
        dof = qs_sample.shape[1]

        # Transform into JointTrajectory
        joint_trajectory = JointTrajectory()
        for i in range(0, n):
            temp_point = JointTrajectoryPoint()

            for j in range(0, dof):
                temp_point.positions.append(qs_sample[i,j])
                temp_point.velocities.append(qds_sample[i,j])
                temp_point.accelerations.append(qdds_sample[i,j])

            temp_point.time_from_start = Duration(seconds=(i/f)).to_msg()
            joint_trajectory.points.append(temp_point)

        # Add last point with zero velocity and acceleration
        last_point = JointTrajectoryPoint()
        for i in range(0, dof):
            last_point.positions.append(qs_sample[n-1,i])
            last_point.velocities.append(0.0)
            last_point.accelerations.append(0.0)
        last_point.time_from_start = Duration(seconds=(n/f)).to_msg()
        joint_trajectory.points.append(last_point)

        for i in range(0, dof):
            joint_trajectory.joint_names.append("joint"+str(i+1))

        return joint_trajectory

if __name__ == '__main__':
    rclpy.init()    
    toppra_trajectory = ToppraTrajectory()
    rclpy.spin(toppra_trajectory)
