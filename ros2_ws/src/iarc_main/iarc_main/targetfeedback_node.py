"""
Placeholder
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
import json
import socket
from collections import deque

class TargetFeedbackNode(Node):
    def __init__(self):
        super().__init__("target_feedback_node")
        self._param_init()
        self._ros2_init()
        self.get_logger().info("Target Feedback Node initialized")

    def _param_init(self):
        # main parameters
        self.tag_pose_topic = self.declare_parameter(
            "tag_pose_topic",
            "pose",
            ParameterDescriptor(
                description="Topic name for receiving tag poses"
            )
        ).value
        self.arena_frame_id = self.declare_parameter(
            "arena_frame_id",
            "map",
            ParameterDescriptor(
                description="Frame ID of the arena coordinate system"
            )
        ).value
        self.feedback_ip = self.declare_parameter(
            "feedback_ip",
            "127.0.0.1",
            ParameterDescriptor(
                description="IP address of the feedback receiver"
            )
        ).value
        self.feedback_port = self.declare_parameter(
            "feedback_port",
            6006,
            ParameterDescriptor(
                description="Port number of the feedback receiver"
            )
        ).value
        self.feedback_rate = self.declare_parameter(
            "feedback_rate",
            10.0,
            ParameterDescriptor(
                description="Rate (Hz) at which to send feedback"
            )
        ).value
        self.grid_size_m = self.declare_parameter(
            "grid_size(meters)",
            0.5,
            ParameterDescriptor(
                description="Size of each grid cell in meters"
            )
        ).value
        self.tf_lookup_timeout_s = self.declare_parameter(
            "tf_lookup_timeout(seconds)",
            0.1,
            ParameterDescriptor(
                description="Timeout (seconds) for TF lookups"
            )
        ).value
        self.feedback_retry = self.declare_parameter(
            "feedback_retry",
            3,
            ParameterDescriptor(
                description="Number of times to retry sending feedback if it fails"
            )
        ).value
        self.msg_queue_max_size = self.declare_parameter(
            "msg_queue_max_size",
            100,
            ParameterDescriptor(
                description="Maximum size of the feedback message queue"
            )
        ).value
        self.do_map_pub = self.declare_parameter(
            "do_map_pub",
            False,
            ParameterDescriptor(
                description="Whether to publish occupancy grid maps for visualization"
            )
        ).value
        if self.do_map_pub:
            self.get_logger().info("Occupancy grid map publishing enabled")
            # Auxiliary parameters for map publishing (if enabled)
            self.map_topic = self.declare_parameter(
                "map_topic",
                "map",
                ParameterDescriptor(
                    description="Topic name for receiving occupancy grid maps"
                )
            ).value
            self.map_pub_rate = self.declare_parameter(
                "map_pub_rate",
                1.0,
                ParameterDescriptor(
                    description="Rate (Hz) at which to publish occupancy grid maps"
                )
            ).value

    def _ros2_init(self):
        """
        Placeholder
        """
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.msg_queue = deque(maxlen=self.msg_queue_max_size)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.tag_pose_topic,
            self._pose_callback,
            10
        )
        self.feedback_timer = self.create_timer(
            1.0 / self.feedback_rate,
            self._feedback_timer_callback
        )

        if self.do_map_pub:
            self.grid_set = set()
            self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)
            self.map_pub_timer = self.create_timer(
                1.0 / self.map_pub_rate,
                self._map_pub_timer_callback
            )

    def _feedback_timer_callback(self):
        """
        Placeholder
        """
        while self.msg_queue:
            msg = self.msg_queue[0]  # Peek at the front of the queue
            for _ in range(self.feedback_retry):
                try:
                    self.sock.sendto(
                        msg.encode("utf-8"),
                        (self.feedback_ip, self.feedback_port)
                    )
                    self.msg_queue.popleft()  # Remove the message after successful send
                    break
                except socket.error as e:
                    self.get_logger().error(f"Failed to send feedback: {e}")
            else:
                self.get_logger().error(f"Failed to send feedback after {self.feedback_retry} attempts, dropping message")
                self.msg_queue.popleft()  # Drop the message after exhausting retries

    def _pose_callback(self, msg: PoseStamped):
        """
        Placeholder
        """
        source_frame = msg.header.frame_id
        query_time = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arena_frame_id,
                source_frame,
                query_time,
                Duration(seconds=self.tf_lookup_timeout_s)
            )
        except TransformException as e:
            try:
                self.get_logger().error(f"TF lookup failed: {e}, using latest available transform")
                transform = self.tf_buffer.lookup_transform(
                    self.arena_frame_id,
                    source_frame,
                    rclpy.time.Time(),
                    Duration(seconds=self.tf_lookup_timeout_s)
                ) # How to deal with situations when the transform chain isn't yet established?
            except TransformException as e:
                self.get_logger().error(f"Failed to get any transform: {e}, skipping this pose")
                return
        transformed_pose = do_transform_pose(msg, transform)
        x, y = int(transformed_pose.pose.position.x / self.grid_size_m), int(transformed_pose.pose.position.y / self.grid_size_m)
        if self.do_map_pub:
            self.grid_set.add((x, y))
        feedback_payload = {
            "x": x,
            "y": y
        }
        feedback_json = json.dumps(feedback_payload)
        self.msg_queue.append(feedback_json)

    def _map_pub_timer_callback(self):
        if not self.grid_set:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.arena_frame_id
        msg.info.resolution = self.grid_size_m
        msg.info.width = max(self.grid_set, key=lambda g: g[0])[0] + 1
        msg.info.height = max(self.grid_set, key=lambda g: g[1])[1] + 1
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (msg.info.width * msg.info.height)
        for x, y in self.grid_set:
            if 0 <= x < msg.info.width and 0 <= y < msg.info.height:
                index = y * msg.info.width + x
                msg.data[index] = 100
        self.map_pub.publish(msg)

    def on_exit(self):
        self.sock.close()
        self.get_logger().warn("Target Feedback Node shutting down")

def main(args=None):
    rclpy.init(args=args)
    node = TargetFeedbackNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.on_exit()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()