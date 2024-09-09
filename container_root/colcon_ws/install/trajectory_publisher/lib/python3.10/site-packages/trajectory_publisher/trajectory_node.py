import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException, Buffer, TransformListener
from rclpy.time import Time
from rclpy.duration import Duration

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.ground_truth_pub = self.create_publisher(Path, '/ground_truth_path', 10)
        self.slam_path_pub = self.create_publisher(Path, '/slam_path', 10)
        
        self.ground_truth_path = Path()
        self.slam_path = Path()
        
        self.create_timer(0.1, self.timer_callback)

    def get_transform(self, target_frame, source_frame):
        try:
            now = Time()
            trans = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            self.get_logger().debug(f"Successfully got transform from {source_frame} to {target_frame}")
            return trans
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {source_frame} to {target_frame}: {ex}")
            return None

    def timer_callback(self):
        # Ground truth path
        trans = self.get_transform('map', 'robot_0/base_footprint')
        if trans:
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            
            self.ground_truth_path.header = trans.header
            self.ground_truth_path.poses.append(pose)
            self.ground_truth_pub.publish(self.ground_truth_path)
        
        # SLAM path
        trans = self.get_transform('map', 'robot_0/camera_link')
        if trans:
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            
            self.slam_path.header = trans.header
            self.slam_path.poses.append(pose)
            self.slam_path_pub.publish(self.slam_path)

def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    rclpy.spin(trajectory_publisher)
    trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()