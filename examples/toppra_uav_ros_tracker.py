#!/usr/bin/env python

import rospy
import tf
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Bool
from topp_ros.srv import GenerateTrajectory, GenerateTrajectoryRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint, \
    MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist


class TrackerParameters:
    def __init__(self):
        self.velocity = [5, 5, 5, 2.5]
        self.acceleration = [2.75, 2.75, 2.75, 1.5]
        self.sampling_frequency = 100
        self.n_gridpoints = 500

class UavRosTracker:

    def __init__(self):
        # Load tracker parameters
        self.tracker_params = TrackerParameters()

        self.tracker_params.n_gridpoints = rospy.get_param("~topp_tracker/n_gridpoints")
        self.tracker_params.sampling_frequency = rospy.get_param("~topp_tracker/sampling_frequency")

        self.tracker_params.velocity[0] = rospy.get_param("~topp_tracker/constraints/velocity/x")
        self.tracker_params.velocity[1] = rospy.get_param("~topp_tracker/constraints/velocity/y")
        self.tracker_params.velocity[2] = rospy.get_param("~topp_tracker/constraints/velocity/z")
        self.tracker_params.velocity[3] = rospy.get_param("~topp_tracker/constraints/velocity/yaw")

        self.tracker_params.acceleration[0] = rospy.get_param("~topp_tracker/constraints/acceleration/x")
        self.tracker_params.acceleration[1] = rospy.get_param("~topp_tracker/constraints/acceleration/y")
        self.tracker_params.acceleration[2] = rospy.get_param("~topp_tracker/constraints/acceleration/z")
        self.tracker_params.acceleration[3] = rospy.get_param("~topp_tracker/constraints/acceleration/yaw")

        self.point_index = 0
        self.trajectory = MultiDOFJointTrajectory()
        self.pose_sub = rospy.Subscriber("input/trajectory", MultiDOFJointTrajectory, self.trajectory_cb)
        
        self.carrot_status = String()
        self.carrot_status.data = "HOLD"
        self.status_sub = rospy.Subscriber("carrot/status", String, self.status_cb)
        
        self.odom_msg = Odometry()
        self.odom_flag = False
        self.odom_sub = rospy.Subscriber("odometry", Odometry, self.odom_cb)
        
        self.point_pub = rospy.Publisher("output/point", MultiDOFJointTrajectoryPoint, queue_size=1)
        self.activity_pub = rospy.Publisher("topp/status", Bool, queue_size=1)
        
        self.trajectory_flag_sub = rospy.Subscriber("trajectory_flag", Bool, self.trajectory_flag_cb)
        self.publish_trajectory = True
        
    def status_cb(self, msg):
        self.carrot_status = msg
        if not self.carrot_status.data == "HOLD" and self.trajectory.points:
            print("UavRosTracker - hold disabled, clearing trajectory!")
            self.trajectory.points = []

    def odom_cb(self, msg):
        self.odom_msg = msg
        self.odom_flag = True

    def publish_trajectory_status(self, status):
        msg = Bool()
        msg.data = status
        self.activity_pub.publish(msg)

    def trajectory_flag_cb(self, msg):
        self.publish_trajectory = msg.data

    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            print("UavRosTracker - empty input trajectory recieved, RESET")
            self.trajectory = MultiDOFJointTrajectory()
            return 
        
        x = []
        y = []
        z = []
        yaw = []
        for point in msg.points:
            x.append(point.transforms[0].translation.x)
            y.append(point.transforms[0].translation.y)
            z.append(point.transforms[0].translation.z)
            yaw.append(tf.transformations.euler_from_quaternion(
                [point.transforms[0].rotation.x, 
                point.transforms[0].rotation.y,
                point.transforms[0].rotation.z,
                point.transforms[0].rotation.w])[2])
        
        for x_,y_,z_,yaw_ in zip(x, y, z, yaw):
            print("Recieved point: ", x_, y_, z_, yaw_)
        
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
                waypoint.velocities = [self.tracker_params.velocity[0], self.tracker_params.velocity[1], self.tracker_params.velocity[2], self.tracker_params.velocity[3]]
                waypoint.accelerations = [self.tracker_params.acceleration[0], self.tracker_params.acceleration[1], self.tracker_params.acceleration[2], self.tracker_params.acceleration[3]]

            # Append all waypoints in request
            request.waypoints.points.append(copy.deepcopy(waypoint))

        request.waypoints.joint_names = ["x", "y", "z", "yaw"]
        request.sampling_frequency = self.tracker_params.sampling_frequency
        request.n_gridpoints = self.tracker_params.n_gridpoints
        request.plot = False

        # Request the trajectory
        request_trajectory_service = rospy.ServiceProxy("generate_toppra_trajectory", GenerateTrajectory)
        response = request_trajectory_service(request)

        # Response will have trajectory and bool variable success. If for some
        # reason the trajectory was not able to be planned or the configuration
        # was incomplete or wrong it will return False.
        print ("UavRosTracker: Converting trajectory to multi dof")
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

            quaternion = tf.transformations.quaternion_from_euler(0, 0, joint_trajectory.points[i].positions[3])
            temp_transform.rotation.x = quaternion[0]
            temp_transform.rotation.y = quaternion[1]
            temp_transform.rotation.z = quaternion[2]
            temp_transform.rotation.w = quaternion[3]

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
                rospy.loginfo_throttle(1.0, "UavRosTracker - odometry unavailable")
                self.publish_trajectory_status(False)
                rospy.sleep(0.01)
                continue

            if not self.carrot_status.data == "HOLD":
                rospy.loginfo_throttle(1.0, "UavRosTracker - Position hold disabled")
                self.publish_trajectory_status(False)
                rospy.sleep(0.01)
                continue
            
            if not self.trajectory.points:
                rospy.loginfo_throttle(1.0, "UavRosTracker - No trajectory available")
                self.publish_trajectory_status(False)
                rospy.sleep(0.01)
                continue
                
            if not self.publish_trajectory:
                rospy.loginfo_throttle(1.0, "UavRosTracker - Do not have a permission to publish trajectory.")
                rospy.sleep(0.01)
                continue

            # Publish trajectory point
            self.point_pub.publish(self.trajectory.points.pop(0))
            self.publish_trajectory_status(True)
            rospy.sleep(0.01)

if __name__ == "__main__":
    rospy.init_node("uav_ros_tracker")   
    tracker = UavRosTracker()
    tracker.run()