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
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse


class TrackerParameters:
    def __init__(self):
        self.request_permission = True
        self.velocity = [5, 5, 5, 2.5]
        self.acceleration = [2.75, 2.75, 2.75, 1.5]
        self.sampling_frequency = 100
        self.n_gridpoints = 500

class TrackerStatus:
    off = "OFF"
    accept = "ACCEPT"
    wait = "WAIT"
    active = "ACTIVE"

class UavRosTracker:

    def __init__(self):
        # Load tracker parameters
        self.tracker_params = TrackerParameters()

        self.tracker_params.n_gridpoints = rospy.get_param("~topp_tracker/n_gridpoints")
        self.tracker_params.sampling_frequency = rospy.get_param("~topp_tracker/sampling_frequency")
        self.tracker_params.request_permission = rospy.get_param("~topp_tracker/request_permission")
        self.rate = 1.0 / self.tracker_params.sampling_frequency
        
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
        self.pose_sub = rospy.Subscriber("topp/input_trajectory", MultiDOFJointTrajectory, self.trajectory_cb)
        
        self.carrot_status = String()
        self.carrot_status.data = "HOLD"
        self.status_sub = rospy.Subscriber("carrot/status", String, self.status_cb)
        
        self.carrot_trajectory = MultiDOFJointTrajectoryPoint()
        self.carrot_trajectory_recieved = False
        self.carrot_trajectory_sub = rospy.Subscriber("carrot/trajectory", MultiDOFJointTrajectoryPoint, self.carrot_trajectory_cb)
        
        self.point_pub = rospy.Publisher("output/point", MultiDOFJointTrajectoryPoint, queue_size=1)
        self.activity_pub = rospy.Publisher("topp/status", String, queue_size=1)
        self.path_pub = rospy.Publisher("topp/path", Path, queue_size=1)

        self.enable_trajectory = False
        self.enable_service = rospy.Service("topp/enable", Empty, self.enable_service_cb)
        
    def status_cb(self, msg):
        self.carrot_status = msg
        if not self.carrot_status.data == "HOLD" and self.trajectory.points:
            print("UavRosTracker - hold disabled, clearing trajectory!")
            self.trajectory.points = []

    def carrot_trajectory_cb(self, msg):
        self.carrot_trajectory = msg
        self.carrot_trajectory_recieved = True

    def publish_tracker_status(self, status):
        msg = String()
        msg.data = status
        self.activity_pub.publish(msg)

    def enable_service_cb(self, req):
        self.enable_trajectory = True
        return EmptyResponse()

    def trajectory_cb(self, msg):
        if len(msg.points) == 0:
            print("UavRosTracker - empty input trajectory recieved, RESET")
            self.trajectory = MultiDOFJointTrajectory()
            return 
        
        if (not self.carrot_trajectory_recieved):
            print("UavRosTracker - trajectory recieved but carrot unavailable")
            self.trajectory = MultiDOFJointTrajectory()
            return

        x = []
        y = []
        z = []
        yaw = []

        # Append first point from ROS tracker to avoid jumps
        x.append(self.carrot_trajectory.transforms[0].translation.x)
        y.append(self.carrot_trajectory.transforms[0].translation.y)
        z.append(self.carrot_trajectory.transforms[0].translation.z)
        yaw.append(tf.transformations.euler_from_quaternion(
                [self.carrot_trajectory.transforms[0].rotation.x, 
                self.carrot_trajectory.transforms[0].rotation.y,
                self.carrot_trajectory.transforms[0].rotation.z,
                self.carrot_trajectory.transforms[0].rotation.w])[2])

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

        self.enable_trajectory = False
        self.trajectory = self.JointTrajectory2MultiDofTrajectory(joint_trajectory)

        # Publish the path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = msg.header.frame_id
        for point in self.trajectory.points:
            path_point = PoseStamped()
            path_point.header.stamp = rospy.Time.now()
            path_point.header.frame_id = msg.header.frame_id
            path_point.pose.position.x = point.transforms[0].translation.x
            path_point.pose.position.y = point.transforms[0].translation.y
            path_point.pose.position.z = point.transforms[0].translation.z
            path_point.pose.orientation.x = point.transforms[0].rotation.x
            path_point.pose.orientation.y = point.transforms[0].rotation.y
            path_point.pose.orientation.z = point.transforms[0].rotation.z
            path_point.pose.orientation.w = point.transforms[0].rotation.w
            path_msg.poses.append(path_point)
        self.path_pub.publish(path_msg)
        

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
            
            if not self.carrot_trajectory_recieved:
                rospy.loginfo_throttle(1.0, "UavRosTracker - Carrot trajectory unavailable")
                self.publish_tracker_status(TrackerStatus.off)
                self.enable_trajectory = False
                rospy.sleep(0.01)
                continue

            if not self.carrot_status.data == "HOLD":
                rospy.loginfo_throttle(1.0, "UavRosTracker - Position hold disabled")
                self.publish_tracker_status(TrackerStatus.off)
                self.enable_trajectory = False
                rospy.sleep(0.01)
                continue
            
            if not self.trajectory.points:
                rospy.loginfo_throttle(1.0, "UavRosTracker - No trajectory available")
                self.publish_tracker_status(TrackerStatus.accept)
                self.enable_trajectory = False
                rospy.sleep(0.01)
                continue
                
            if self.tracker_params.request_permission and not self.enable_trajectory:
                rospy.loginfo_throttle(1.0, "UavRosTracker - Do not have a permission to publish trajectory.")
                self.publish_tracker_status(TrackerStatus.wait)
                rospy.sleep(0.01)
                continue

            # Publish trajectory point
            self.point_pub.publish(self.trajectory.points.pop(0))
            self.publish_tracker_status(TrackerStatus.active)
            rospy.sleep(self.rate)

if __name__ == "__main__":
    rospy.init_node("uav_ros_tracker")   
    tracker = UavRosTracker()
    tracker.run()