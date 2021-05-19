from __future__ import print_function
import string, time
from pylab import *
from numpy import *
from TOPP import TOPPbindings
from TOPP import TOPPpy
from TOPP import Trajectory
from TOPP import Utilities

# A two-dof path going through 5 viapoints (0,1) - (1,1) - (5,1) - (3,2) - (5,4)
path = array([[-1,0,1,1,1,0,-1,-1,-1],[-1,-1,-1,0,1,1,1,0,-1], [1,1,1,1,1,1,1,1,1], [0,0,0,0,0,0,0,0,0]])
traj0 = Utilities.InterpolateViapoints(path) # Interpolate using splines
print path
# Constraints
vmax = 2*ones(traj0.dimension)  # Velocity limits
amax = 1.5*ones(traj0.dimension) # Acceleration limits

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

# Display results
ion()
x.WriteProfilesList()
x.WriteSwitchPointsList()
profileslist = TOPPpy.ProfilesFromString(x.resprofilesliststring)
switchpointslist = TOPPpy.SwitchPointsFromString(x.switchpointsliststring)
TOPPpy.PlotProfiles(profileslist,switchpointslist,4)
x.WriteResultTrajectory()
traj1 = Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
dtplot = 0.01
TOPPpy.PlotKinematics(traj0,traj1,dtplot,vmax,amax)
print("Trajectory duration before TOPP: ", traj0.duration)
print("Trajectory duration after TOPP: ", traj1.duration)
print("Number of chunks in final trajectory: ", len(traj1.chunkslist))
input()