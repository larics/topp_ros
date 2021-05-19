#!/usr/bin/env python
from __future__ import print_function
import string, time, copy
from pylab import *
from numpy import *
from TOPP import TOPPbindings
from TOPP import TOPPpy
from TOPP import Trajectory
from TOPP import Utilities

# Ros imports
import rospy
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ToppTrajectory():

    def __init__(self):
        # setup topics and services
        self.generate_topp_trajectory_service = rospy.Service('generate_topp_trajectory', 
            GenerateTrajectory, self.generateToppTrajectoryCallback)

        self.plot_flag = False

    def run(self):
        rospy.spin()

    def generateToppTrajectoryCallback(self, req):
        print("Generating TOPP trajectory.")
        res = GenerateTrajectoryResponse()
        dof = len(req.waypoints.points[0].positions)
        n = len(req.waypoints.points)

        # If there is not enough waypoints to generate a trajectory return false
        if (n <= 1 or dof == 0):
            print("You must provide at least 2 points to generate a valid trajectory.")
            res.trajectory.success = False
            return res

        # Generate trajectory
        # Path is of dimensions DOF x n_waypoints
        path = np.zeros([dof, n])

        for i in range(0, n):
            for j in range(0, dof):
                path[j][i] = req.waypoints.points[i].positions[j]

        traj0 = Utilities.InterpolateViapoints(path) # Interpolate using splines

        # Constraints
        vmax = zeros(dof)  # Velocity limits
        amax = zeros(dof) # Acceleration limits
        for i in range(0, dof):
            vmax[i] = req.waypoints.points[0].velocities[i]
            amax[i] = req.waypoints.points[0].accelerations[i]

        # Set up the TOPP instance
        trajectorystring = str(traj0)
        discrtimestep = 0.01
        uselegacy = False
        t0 = time.time()
        if uselegacy: #Using the legacy KinematicLimits (a bit faster but not fully supported)
            constraintstring = str(discrtimestep)
            constraintstring += "\n" + string.join([str(v) for v in vmax])
            constraintstring += "\n" + string.join([str(a) for a in amax])
            x = TOPPbindings.TOPPInstance(None,"KinematicLimits",constraintstring,trajectorystring);
        else: #Using the general QuadraticConstraints (fully supported)
            constraintstring = str(discrtimestep)
            constraintstring += "\n" + string.join([str(v) for v in vmax])
            constraintstring += TOPPpy.ComputeKinematicConstraints(traj0, amax, discrtimestep) 
            x = TOPPbindings.TOPPInstance(None,"QuadraticConstraints",constraintstring,trajectorystring);
        
        # Run TOPP
        t1 = time.time()
        ret = x.RunComputeProfiles(0,0)
        x.ReparameterizeTrajectory()
        t2 = time.time()

        print("Using legacy:", uselegacy)
        print("Discretization step:", discrtimestep)
        print("Setup TOPP:", t1-t0)
        print("Run TOPP:", t2-t1)
        print("Total:", t2-t0)

        x.WriteProfilesList()
        x.WriteSwitchPointsList()
        profileslist = TOPPpy.ProfilesFromString(x.resprofilesliststring)
        switchpointslist = TOPPpy.SwitchPointsFromString(x.switchpointsliststring)
        TOPPpy.PlotProfiles(profileslist,switchpointslist,4)
        x.WriteResultTrajectory()
        traj1 = Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
        dtplot = 0.01
        if self.plot_flag == True:
            ion()
            TOPPpy.PlotKinematics(traj0,traj1,dtplot,vmax,amax)

        res.trajectory = self.TOPP2JointTrajectory(traj1, req.sampling_frequency)
        res.success = True
        return res

    def TOPP2JointTrajectory(self, traj, f):
        tvect = arange(0, traj.duration + 1/f, 1/f)
        qvect = array([traj.Eval(t) for t in tvect])
        qdvect = array([traj.Evald(t) for t in tvect])
        qddvect = array([traj.Evaldd(t) for t in tvect])
        n = qvect.shape[0]
        dof = qvect.shape[1]

        joint_trajectory = JointTrajectory()
        for i in range(0,n):
            temp_point = JointTrajectoryPoint()

            for j in range(0, dof):
                temp_point.positions.append(qvect[i][j])
                temp_point.velocities.append(qdvect[i][j])
                temp_point.accelerations.append(qddvect[i][j])

            temp_point.time_from_start = rospy.Duration.from_sec(i/f)
            joint_trajectory.points.append(temp_point)

        last_point = JointTrajectoryPoint()
        for j in range(0, dof):
            last_point.positions.append(qvect[len(tvect)-1][j])
            last_point.velocities.append(qdvect[len(tvect)-1][j])
            last_point.accelerations.append(qddvect[len(tvect)-1][j])
        
        last_point.time_from_start = rospy.Duration.from_sec((len(tvect)-1)/f)
        joint_trajectory.points.append(last_point)

        return joint_trajectory

if __name__=="__main__":
    rospy.init_node("ToppTrajectoryGeneration")
    generator = ToppTrajectory()
    generator.run()