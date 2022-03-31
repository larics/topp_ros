# topp_ros
This repository provides with TOPP and TOPP-RA interface to ROS.

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_general/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_general/actions) | [![Noetic](https://github.com/larics/topp_ros/actions/workflows/noetic.yml/badge.svg?branch=master)](https://github.com/larics/topp_ros/actions/workflows/noetic.yml) |

## Table Of Contents

- [Installation](#Installation)
- [Running the planner](#Running)
    * [TOPP-RA](#RunningTOPPRA)
        - [Services](#Services)

## <a name="Installation"></a> Installation
This repository depents on following repositories: [TOPP](https://github.com/quangounet/TOPP) and [TOPP-RA](https://github.com/hungpham2511/toppra). Follow the installation instructions on these repositories prior to installation of this package.

**Installing TOPP-RA**
Installing TOPP-RA can be a litle bit tricky. Here are instructions for installing basic functionality(which is enough for usage with topp_ros package) you can type:
``` bash
git clone https://github.com/hungpham2511/toppra && cd toppra/
git checkout 8df858b08175d4884b803bf6ab7f459205e54fa2 #checkout to working python 2.7 version of the toppra package
pip install -r requirements.txt --user
python setup.py install --user
```


To use this repository simply clone it into your ROS workspace and build it.

## <a name="Running"></a> Running the planner
Both TOPP and TOPP-RA algorithms are avaliable for trajectory planning. We recommend the newer version - TOPP-RA since it has better success rate for more constraints.

### <a name="RunningTOPPRA"></a> TOPP-RA
You can run the trajectory generation node with:

```
rosrun topp_ros generate_toppra_trajectory.py
```

#### <a name="Services"></a> Services

**GenerateTrajectory**
```
trajectory_msgs/JointTrajectory     waypoints
float64                             sampling_frequency
bool                                plot
---
trajectory_msgs/JointTrajectory     trajectory
bool                                success
```

The waypoints are sent to the service as a [trajectory\_msgs/JointTrajectory](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectory.html) in request. Each waypoint is n-DOF and the type is [trajectory\_msgs/JointTrajectoryPoint](http://docs.ros.org/melodic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html). Position of each waypoint is passed through ```positions``` field. The first waypoint must also contain constraints for velocity and acceleration for that joint. These constraints can be passed through ```velocities``` and ```accelerations``` fields. The detailed call of this service with comments in code can be found in [examples/toppra\_trajectory\_call\_example](https://github.com/larics/topp_ros/blob/master/examples/topp_trajectory_call_example.py)
