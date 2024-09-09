import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
import tf2_ros

class ImprovedTrajectoryComparer(Node):
    def __init__(self):
        super().__init__('improved_trajectory_comparer')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/robot_0/odom', self.odom_callback, 10)
        
        self.camera_traj_sub = self.create_subscription(
            Path, '/robot_0/camera_trajectory', self.camera_traj_callback, 10)
        
        self.odom_marker_pub = self.create_publisher(MarkerArray, '/odom_trajectory_markers', 10)
        self.camera_marker_pub = self.create_publisher(MarkerArray, '/camera_trajectory_markers', 10)
        
        self.odom_marker_array = MarkerArray()
        self.camera_marker_array = MarkerArray()
        self.odom_id = 0
        self.camera_id = 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.publish_marker(pose_stamped, self.odom_id, (1.0, 0.0, 0.0), "odom", self.odom_marker_array, self.odom_marker_pub)
        self.odom_id += 1
        self.get_logger().info(f"Odometry pose {self.odom_id}: {self.pose_to_string(msg.pose.pose)}")

    def camera_traj_callback(self, msg):
        for pose_stamped in msg.poses:
            transformed_pose = self.transform_pose(pose_stamped, "base_footprint")
            if transformed_pose:
                self.publish_marker(transformed_pose, self.camera_id, (0.0, 1.0, 0.0), "camera", self.camera_marker_array, self.camera_marker_pub)
                self.camera_id += 1
        self.get_logger().info(f"Received camera trajectory with {len(msg.poses)} poses")
        if msg.poses:
            self.get_logger().info(f"Last camera pose: {self.pose_to_string(msg.poses[-1].pose)}")
            
    def transform_pose(self, pose_stamped, target_frame):
        try:
            # Transform from camera_trajectory to camera_link
            camera_link_pose = self.transform_single(pose_stamped, "camera_link")
            if not camera_link_pose:
                print("camera_link_pose is not available")
                return None

            # Transform from camera_link to base_link
            base_link_pose = self.transform_single(camera_link_pose, "base_link")
            if not base_link_pose:
                print("base_link_pose is not available")
                return None

            # Transform from base_link to base_footprint
            base_footprint_pose = self.transform_single(base_link_pose, target_frame)
            if not base_footprint_pose:
                print("base_footprint_pose is not available")
                return None

            return base_footprint_pose

        except (TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Could not transform pose: {ex}")
            return None
        
    def transform_single(self, pose_stamped, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                pose_stamped.header.stamp,
                rclpy.duration.Duration(seconds=1.0))
            return do_transform_pose(pose_stamped, transform)
        except (TransformException, tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().warn(f"Could not transform from {pose_stamped.header.frame_id} to {target_frame}: {ex}")
            return None

    # def publish_marker(self, pose_stamped, id, color, ns):
    #     marker = Marker()
    #     marker.header = pose_stamped.header
    #     marker.ns = ns
    #     marker.id = id
    #     marker.type = Marker.SPHERE
    #     marker.action = Marker.ADD
    #     marker.pose = pose_stamped.pose
    #     marker.scale.x = marker.scale.y = marker.scale.z = 0.1
    #     marker.color.r, marker.color.g, marker.color.b = color
    #     marker.color.a = 1.0

    #     self.marker_array.markers.append(marker)
    #     self.marker_pub.publish(self.marker_array)
    #     self.get_logger().debug(f"Published marker: ns={ns}, id={id}, frame={pose_stamped.header.frame_id}")
    
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

        marker_array.markers.append(marker)
        marker_publisher.publish(marker_array)
        self.get_logger().debug(f"Published {ns} marker: id={id}, frame={pose_stamped.header.frame_id}")

    @staticmethod
    def pose_to_string(pose):
        return f"Position: ({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}), " \
               f"Orientation: ({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, {pose.orientation.z:.4f}, {pose.orientation.w:.4f})"

def main(args=None):
    rclpy.init(args=args)
    comparer = ImprovedTrajectoryComparer()
    rclpy.spin(comparer)
    comparer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()