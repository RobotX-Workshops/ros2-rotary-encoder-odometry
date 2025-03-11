import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math

def euler_to_quaternion(yaw):
    """Convert yaw angle to quaternion for 2D orientation."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

class EncoderToOdometry(Node):
    def __init__(self):
        super().__init__('encoder_to_odometry')

        # Declare parameters with default values
        self.declare_parameter('encoder_topic', '/encoder')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('distance_per_tick_m', 0.01)  # meters per tick
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        # Retrieve parameter values
        encoder_topic = self.get_parameter('encoder_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.distance_per_tick = self.get_parameter('distance_per_tick_m').get_parameter_value().double_value
        self.odom_frame_id = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # Initialize state variables
        self.count = 0            # Current encoder count
        self.prev_count = 0       # Previous encoder count
        self.x = 0.0              # Robot’s x position (meters)
        self.theta = 0.0          # Fixed orientation (no rotation)
        self.first_run = True     # Flag for initialization
        self.prev_time = self.get_clock().now()  # Last update time

        # Subscribe to encoder counts
        self.encoder_sub = self.create_subscription(
            Int32, encoder_topic, self.encoder_callback, 10)

        # Publish odometry data
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # Timer to process data every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)

    def encoder_callback(self, msg):
        """Store the latest encoder count."""
        self.count = msg.data

    def timer_callback(self):
        """Compute and publish odometry periodically."""
        current_time = self.get_clock().now()

        # Skip computation on the first run (initialize baseline)
        if self.first_run:
            self.prev_count = self.count
            self.prev_time = current_time
            self.first_run = False
            return

        # Calculate time difference (in seconds)
        time_delta = (current_time - self.prev_time).nanoseconds / 1e9
        if time_delta <= 0:
            return  # Avoid division by zero

        # Calculate change in encoder counts
        delta_count = self.count - self.prev_count

        # Convert to distance traveled (meters)
        delta_distance = delta_count * self.distance_per_tick

        # Update position (only x changes)
        self.x += delta_distance

        # Calculate linear velocity (meters/second)
        linear_velocity = delta_distance / time_delta

        # Update previous values for the next iteration
        self.prev_count = self.count
        self.prev_time = current_time

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id     # Parent frame
        odom.child_frame_id = self.child_frame_id  # Robot’s frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0  # No rotation

        # Publish the odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()