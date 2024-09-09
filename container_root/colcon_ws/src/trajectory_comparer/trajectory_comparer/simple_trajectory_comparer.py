import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from scipy.spatial.transform import Rotation

class SLAMAccuracyComparer(Node):
    def __init__(self):
        super().__init__('slam_accuracy_comparer')
        
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'robot_0/base_footprint')
        self.declare_parameter('max_time_diff', 2.5)  # maximum allowed time difference in seconds
        
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.max_time_diff = self.get_parameter('max_time_diff').value
        self.cumulative_distance = 0.0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.camera_traj_sub = self.create_subscription(
            Path, '/robot_0/camera_trajectory', self.camera_traj_callback, 10)
        
        self.robot_marker_pub = self.create_publisher(MarkerArray, '/robot_trajectory_markers', 10)
        self.camera_marker_pub = self.create_publisher(MarkerArray, '/camera_trajectory_markers', 10)
        
        self.robot_marker_array = MarkerArray()
        self.camera_marker_array = MarkerArray()
        self.robot_id = 0
        self.camera_id = 0

        self.robot_poses = []
        self.camera_poses = []

        # Timer to periodically check for robot pose
        self.create_timer(0.1, self.robot_pose_timer_callback)  # 10 Hz

    def robot_pose_timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            self.robot_pose_callback(t)
        except (TransformException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f'Could not transform {self.map_frame} to {self.robot_frame}: {ex}')

    def robot_pose_callback(self, t):
        pose_stamped = PoseStamped()
        pose_stamped.header = t.header
        pose_stamped.pose.position.x = t.transform.translation.x
        pose_stamped.pose.position.y = t.transform.translation.y
        pose_stamped.pose.position.z = t.transform.translation.z
        pose_stamped.pose.orientation = t.transform.rotation

        self.publish_marker(pose_stamped, self.robot_id, (1.0, 0.0, 0.0), "robot", self.robot_marker_array, self.robot_marker_pub)
        self.robot_id += 1
        
        if self.robot_poses:
            distance_moved = self.calculate_pose_error(self.robot_poses[-1], pose_stamped)
            self.cumulative_distance += distance_moved
            self.get_logger().info(f"Distance moved: {distance_moved:.4f} m, Cumulative distance: {self.cumulative_distance:.4f} m")
        
        self.robot_poses.append(pose_stamped)
        self.get_logger().info(f"Robot pose {self.robot_id}: {self.pose_to_string(pose_stamped.pose)}")
        
        self.calculate_accuracy()

    def camera_traj_callback(self, msg):
        self.camera_poses = msg.poses
        for pose_stamped in msg.poses:
            self.publish_marker(pose_stamped, self.camera_id, (0.0, 1.0, 0.0), "camera", self.camera_marker_array, self.camera_marker_pub)
            self.camera_id += 1
        self.get_logger().info(f"Received camera trajectory with {len(msg.poses)} poses")
        if msg.poses:
            self.get_logger().info(f"Last camera pose: {self.pose_to_string(msg.poses[-1].pose)}")
        
        self.calculate_accuracy()

    def publish_marker(self, pose_stamped, id, color, ns, marker_array, marker_publisher):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.ns = ns
        marker.id = id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # 0 means never expire

        marker_array.markers.append(marker)
        marker_publisher.publish(marker_array)
        self.get_logger().debug(f"Published {ns} marker: id={id}, frame={pose_stamped.header.frame_id}")

    @staticmethod
    def pose_to_string(pose):
        return f"Position: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}), " \
               f"Orientation: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f})"

    def find_closest_pose_pairs(self):
        paired_poses = []
        for camera_pose in self.camera_poses:
            camera_time = rclpy.time.Time.from_msg(camera_pose.header.stamp).nanoseconds / 1e9
            closest_robot_pose = None
            min_time_diff = float('inf')
            for robot_pose in self.robot_poses:
                robot_time = rclpy.time.Time.from_msg(robot_pose.header.stamp).nanoseconds / 1e9
                time_diff = abs(camera_time - robot_time)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_robot_pose = robot_pose
            if min_time_diff <= self.max_time_diff:
                paired_poses.append((closest_robot_pose, camera_pose))
        return paired_poses

    def calculate_accuracy(self):
        if len(self.robot_poses) == 0 or len(self.camera_poses) == 0:
            self.get_logger().warn("No poses available for accuracy calculation")
            return

        paired_poses = self.find_closest_pose_pairs()

        if len(paired_poses) == 0:
            self.get_logger().warn("No matching pose pairs found within the specified time difference")
            return

        ate_errors = []
        rpe_errors = []
        total_distance = 0

        for i, (robot_pose, camera_pose) in enumerate(paired_poses):
            # Calculate ATE
            ate_error = self.calculate_pose_error(robot_pose, camera_pose)
            ate_errors.append(ate_error)

            # Calculate RPE
            if i > 0:
                rpe_error = self.calculate_relative_pose_error(
                    paired_poses[i-1][0], robot_pose,
                    paired_poses[i-1][1], camera_pose
                )
                rpe_errors.append(rpe_error)

            # Calculate total distance traveled
            if i > 0:
                total_distance = self.cumulative_distance
                
        # Compute statistics
        if ate_errors:
            ate_rmse = np.sqrt(np.mean(np.array(ate_errors)**2))
        else:
            ate_rmse = 0

        if rpe_errors:
            rpe_rmse = np.sqrt(np.mean(np.array(rpe_errors)**2))
        else:
            rpe_rmse = 0

        # Calculate percentage accuracy
        if total_distance > 0:
            ate_percentage = (ate_rmse / total_distance) * 100
            accuracy_percentage = max(0, 100 - ate_percentage)
        else:
            ate_percentage = 0
            accuracy_percentage = 0
            self.get_logger().warn("Total distance traveled is zero")

        self.get_logger().info(f"ATE RMSE: {ate_rmse:.4f} m")
        self.get_logger().info(f"RPE RMSE: {rpe_rmse:.4f} m")
        self.get_logger().info(f"Total distance traveled: {total_distance:.4f} m")
        self.get_logger().info(f"Accuracy: {accuracy_percentage:.2f}%")
        self.get_logger().info(f"Error percentage: {min(100, ate_percentage):.2f}%")

    def calculate_pose_error(self, pose1, pose2):
        p1 = np.array([pose1.pose.position.x, pose1.pose.position.y, pose1.pose.position.z])
        p2 = np.array([pose2.pose.position.x, pose2.pose.position.y, pose2.pose.position.z])
        error = np.linalg.norm(p1 - p2)
        return error  # This is in meters

    def calculate_relative_pose_error(self, pose1_t1, pose1_t2, pose2_t1, pose2_t2):
        def pose_to_matrix(pose):
            t = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
            q = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
            R = Rotation.from_quat(q).as_matrix()
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t
            return T

        T1_t1 = pose_to_matrix(pose1_t1)
        T1_t2 = pose_to_matrix(pose1_t2)
        T2_t1 = pose_to_matrix(pose2_t1)
        T2_t2 = pose_to_matrix(pose2_t2)

        rel_pose1 = np.linalg.inv(T1_t1) @ T1_t2
        rel_pose2 = np.linalg.inv(T2_t1) @ T2_t2

        error = np.linalg.inv(rel_pose1) @ rel_pose2
        return np.linalg.norm(error[:3, 3])  # Only consider translation error for simplicity

def main(args=None):
    rclpy.init(args=args)
    comparer = SLAMAccuracyComparer()
    rclpy.spin(comparer)
    comparer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()