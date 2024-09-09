import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

class ImprovedTrajectoryComparer(Node):
    def __init__(self):
        super().__init__('improved_trajectory_comparer')
        
        self.odom_sub = self.create_subscription(
            Odometry, '/robot_0/odom', self.odom_callback, 10)
        
        self.camera_traj_sub = self.create_subscription(
            Path, '/robot_0/camera_trajectory', self.camera_traj_callback, 10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)
        
        self.marker_array = MarkerArray()
        self.odom_id = 0
        self.slam_id = 10000

    def odom_callback(self, msg):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        self.publish_marker(pose_stamped, self.odom_id, (1.0, 0.0, 0.0), "odom")
        self.odom_id += 1
        self.get_logger().info(f"Odometry pose {self.odom_id}: {self.pose_to_string(msg.pose.pose)}")

    def camera_traj_callback(self, msg):
        for pose_stamped in msg.poses:
            self.publish_marker(pose_stamped, self.slam_id, (0.0, 1.0, 0.0), "slam")
            self.slam_id += 1
        self.get_logger().info(f"Received SLAM trajectory with {len(msg.poses)} poses")
        if msg.poses:
            self.get_logger().info(f"Last SLAM pose: {self.pose_to_string(msg.poses[-1].pose)}")

    def publish_marker(self, pose_stamped, id, color, ns):
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

        self.marker_array.markers.append(marker)
        self.marker_pub.publish(self.marker_array)
        self.get_logger().debug(f"Published marker: ns={ns}, id={id}, frame={pose_stamped.header.frame_id}")

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