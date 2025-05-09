from typing import List, Optional
from collections import deque
import rclpy
from rclpy.node import Node, Timer
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from std_srvs.srv import Trigger
import math
import gc


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
        super().__init__("encoder_to_odometry")

        # Declare parameters with default values
        self.declare_parameter("distance_per_tick_m", 0.0)  # meters per tick
        self.declare_parameter("gain", 1.0)  # Gain for the encoder velocity
        self.declare_parameter("rate", 0.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("odom_tf_id", "encoder_odom")
        self.declare_parameter("publish_transform", False)
        self.declare_parameter("buffer_count", 0)

        # Declare Garbabge Collector parameters
        self.declare_parameter("gc.disable", False)
        self.declare_parameter("gc.thresholds.gen_0", 700)
        self.declare_parameter("gc.thresholds.gen_1", 10)
        self.declare_parameter("gc.thresholds.gen_2", 10)

        # Timer to be created on params init
        self.timer: Optional[Timer] = None

        # Retrieve parameter values
        self.update_config()

        # Callback for parameter changes
        self.add_on_set_parameters_callback(self.param_change_callback)

        # Initialize state variables
        self.initial_count = 0  # Our baseline
        self.count = 0  # Current encoder count
        self.prev_avg_count = 0.0  # Previous averaged count
        self.x = 0.0  # Robot’s x position (meters)
        self.theta = 0.0  # Fixed orientation (no rotation)
        self.first_run = True  # Flag for initialization
        self.got_first_count = False  # Flag to check if first count is received
        self.prev_time = self.get_clock().now()  # Last update time

        # Subscribe to encoder counts
        self.encoder_sub = self.create_subscription(
            Int32, "/encoder", self.encoder_callback, 10
        )

        # Publish odometry data
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Define full 36-element covariance matrices
        # Pose covariance: only x has significant uncertainty, others are fixed
        pose_var_x = 0.1  # Variance for x position (meters^2)
        pose_var_y = 1e-6  # Very small variance since y is fixed at 0
        pose_var_z = 1e-6  # Very small variance since z is fixed at 0
        pose_var_roll = 1e-6  # Very small since orientation is fixed
        pose_var_pitch = 1e-6
        pose_var_yaw = 1e-6  # Very small since theta is fixed at 0

        self.pose_covariance = [0.0] * 36  # Initialize with zeros
        self.pose_covariance[0] = pose_var_x  # x variance
        self.pose_covariance[7] = pose_var_y  # y variance
        self.pose_covariance[14] = pose_var_z  # z variance
        self.pose_covariance[21] = pose_var_roll  # roll variance
        self.pose_covariance[28] = pose_var_pitch  # pitch variance
        self.pose_covariance[35] = pose_var_yaw  # yaw variance

        # Twist covariance: only vx has significant uncertainty, others are zero
        twist_var_vx = 0.05  # Variance for linear velocity x (m/s)^2
        twist_var_vy = 1e-6  # Very small since vy is 0
        twist_var_vz = 1e-6  # Very small since vz is 0
        twist_var_wx = 1e-6  # Very small since no rotation
        twist_var_wy = 1e-6
        twist_var_wz = 1e-6

        self.twist_covariance = [0.0] * 36  # Initialize with zeros
        self.twist_covariance[0] = twist_var_vx  # vx variance
        self.twist_covariance[7] = twist_var_vy  # vy variance
        self.twist_covariance[14] = twist_var_vz  # vz variance
        self.twist_covariance[21] = twist_var_wx  # wx variance
        self.twist_covariance[28] = twist_var_wy  # wy variance
        self.twist_covariance[35] = twist_var_wz

        self.tf_broadcaster = None  # Initialize TransformBroadcaster to None

        self.reset_service = self.create_service(
            Trigger, "reset_odometry", self.reset_odometry_callback
        )

    def update_parameters(self, timer: Timer) -> None:
        """Update parameters from the parameter server."""
        timer.cancel()  # Cancel the timer to avoid multiple calls
        self.update_config()
        # Cancel the timer to make it one-shot

    def update_config(self):
        self.distance_per_tick = (
            self.get_parameter("distance_per_tick_m").get_parameter_value().double_value
        )
        self.gain = (
            self.get_parameter("gain")
            .get_parameter_value()
            .double_value  # Retrieve gain parameter
        )
        self.odom_frame_id = (
            self.get_parameter("odom_frame_id").get_parameter_value().string_value
        )
        self.child_frame_id = (
            self.get_parameter("child_frame_id").get_parameter_value().string_value
        )  # wz variance

        self.odom_tf_id = (
            self.get_parameter("odom_tf_id").get_parameter_value().string_value
        )
        self.publish_transform = (
            self.get_parameter("publish_transform").get_parameter_value().bool_value
        )
        self.buffer_count = (
            self.get_parameter("buffer_count").get_parameter_value().integer_value
        )

        self.count_buffer = deque(maxlen=self.buffer_count)
        self.velocity_buffer = deque(maxlen=self.buffer_count)

        self.rate = self.get_parameter("rate").get_parameter_value().double_value

        if self.get_parameter("gc.disable").get_parameter_value().bool_value:
            self.get_logger().info("Garbage collector disabled")
            gc.disable()
        else:
            self.get_logger().info("Garbage collector enabled")
            gc.enable()
            # Set the garbage collector thresholds
            gc.set_threshold(
                self.get_parameter("gc.thresholds.gen_0")
                .get_parameter_value()
                .integer_value,
                self.get_parameter("gc.thresholds.gen_1")
                .get_parameter_value()
                .integer_value,
                self.get_parameter("gc.thresholds.gen_2")
                .get_parameter_value()
                .integer_value,
            )
            self.get_logger().info("Garbage collector thresholds set")

        if self.timer:
            self.timer.cancel()
        # Timer to process data every x seconds
        self.timer = self.create_timer(self.rate, self.timer_callback)

    def param_change_callback(self, _: List[Parameter]) -> SetParametersResult:
        """Handle dynamic parameter changes."""
        self.get_logger().info("New parameters received")

        # Schedule the update to happen after a short delay
        timer = self.create_timer(0.1, lambda: self.update_parameters(timer))
        return SetParametersResult(successful=True)

    def encoder_callback(self, msg: Int32) -> None:
        """Handle incoming encoder data and update the average."""
        if not self.got_first_count:
            # Set the baseline with the first reading
            self.initial_count = msg.data
            self.got_first_count = True
            self.get_logger().info(f"Initial count set: {self.initial_count}")
            return  # Don’t process this first reading yet

        # Get the relative count and add it to the buffer
        relative_count = msg.data - self.initial_count
        self.count_buffer.append(relative_count)
        self.count = sum(self.count_buffer) / len(self.count_buffer)

    def reset_odometry_callback(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Resetting odometry.")
        self.x = 0.0
        self.prev_avg_count = self.count
        response.success = True
        response.message = "Odometry reset successfully."
        return response

    def timer_callback(self):
        current_time = self.get_clock().now()
        if self.first_run and self.got_first_count:
            self.prev_avg_count = self.count  # Initialize properly
            self.prev_time = current_time
            self.first_run = False
            return

        time_delta = (current_time - self.prev_time).nanoseconds / 1e9
        if time_delta <= 0:
            return

        delta_count = self.count - self.prev_avg_count
        delta_distance = delta_count * self.distance_per_tick
        self.x += delta_distance
        linear_velocity = delta_distance / time_delta

        # Add the new velocity to the buffer
        self.velocity_buffer.append(linear_velocity)

        averaged_velocity = 0.0  # Default if no values yet
        # Compute the rolling average of velocities
        if self.velocity_buffer:  # Check if buffer has values
            averaged_velocity = sum(self.velocity_buffer) / len(self.velocity_buffer)

        # Apply the gain to the averaged velocity
        adjusted_velocity = averaged_velocity * self.gain

        self.prev_avg_count = self.count
        self.prev_time = current_time

        # Create and populate the Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame_id  # Parent frame
        odom.child_frame_id = self.child_frame_id  # Robot’s frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = euler_to_quaternion(self.theta)
        odom.twist.twist.linear.x = adjusted_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0  # No rotation
        # Assign covariances to the message
        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)

        if not self.publish_transform:
            return

        # Publish transform if enabled
        if self.tf_broadcaster is None:
            self.tf_broadcaster = TransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.child_frame_id
        t.child_frame_id = self.odom_tf_id

        # We minus the x position to get the transform from the robot to the odom frame
        t.transform.translation.x = -self.x
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        q = euler_to_quaternion(self.theta)
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
