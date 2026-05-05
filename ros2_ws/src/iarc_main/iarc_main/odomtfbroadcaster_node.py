"""
Placeholder for docstrings
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster
import numpy as np
from collections import deque
from iarc_utils.mathematics import lerp, slerp, stamp2us

class OdomTFBroadcasterNode(Node):
    def __init__(self):
        super().__init__("odom_tf_broadcaster_node")
        self._params_init()
        self._ros2_init()
        self.get_logger().info("OdomTFBroadcasterNode has been started.")

    def _params_init(self):
        """
        Placeholder
        """
        self.px4_odom_topic = self.declare_parameter(
            "px4_odom_topic",
            "",
            ParameterDescriptor(
                description="The topic name for the PX4 odometry messages (VehicleOdometry). This topic is expected to be at a high frequency."
            )
        ).value
        self.px4_ned_frame_id = self.declare_parameter(
            "px4_ned_frame_id",
            "px4_ned",
            ParameterDescriptor(description="The frame ID for the PX4 odometry messages."),
        ).value
        self.base_link_frame_id = self.declare_parameter(
            "base_link_frame_id",
            "base_link",
            ParameterDescriptor(description="The frame ID for the robot's base link."),
        ).value
        self.strategy = self.declare_parameter(
            "strategy",
            "odom_callback",
            ParameterDescriptor(description="The strategy for broadcasting the transform. Options are: 1. zero_order_hold: broadcast the transform with the latest received odometry data at a fixed rate. 2. lerp: linearly interpolate the transform based on the latest two received odometry messages and broadcast at a fixed rate. 3. odom_callback: broadcast the transform only when a new odometry message is received, using the data from that message. 4. ekf: use an extended Kalman filter to estimate the transform based on the received odometry messages and broadcast at a fixed rate."),
        ).value
        assert self.strategy.lower() in ["zero_order_hold", "lerp", "odom_callback", "ekf"], (
            "Invalid strategy parameter. Must be one of 'zero_order_hold', 'lerp', 'odom_callback', or 'ekf'."
        )
        if self.strategy.lower() != "odom_callback":
            self.publish_rate = self.declare_parameter(
                "publish_rate",
                50.0,
                ParameterDescriptor(description="The rate (in Hz) at which to publish the transform when using strategies other than 'odom_callback'."),
            ).value

            self.msg_queue = deque(maxlen=2)  # For storing the latest 2 odometry messages.
            self.t_query_us = -1

    def _ros2_init(self):
        """
        Placeholder
        """
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_callback_dict = {
            "odom_callback": self._broadcast_odom_callback,
            "zero_order_hold": self._broadcast_zero_order_hold,
            "lerp": self._broadcast_lerp,
            "ekf": self._broadcast_ekf,
        }
        px4_qosprofile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            self.px4_odom_topic,
            self.odom_callback_dict[self.strategy.lower()],
            px4_qosprofile,
        )

    def _odom_callback(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.get_logger().warn(f"received odometry message with unsupported pose_frame {msg.pose_frame} or velocity_frame {msg.velocity_frame}. expected ned frames. ignoring this message.")
            return
        self.msg_queue.append((msg, stamp2us(self.get_clock().now())))

    def _odom2tf(self, msg: VehicleOdometry, stamp: int) -> TransformStamped:
        t = TransformStamped()
        if stamp < 0:
            stamp = msg.timestamp_sample
        t.header.stamp.sec = stamp // 1000_000
        t.header.stamp.nanosec = (stamp % 1000_000) * 1000
        t.header.frame_id = self.px4_ned_frame_id
        t.child_frame_id = self.base_link_frame_id
        t.transform.translation.x = msg.position[0]
        t.transform.translation.y = msg.position[1]
        t.transform.translation.z = msg.position[2]
        # Note that PX4 quaternions are of the Hamilton convention (w, x, y, z), while ROS uses (x, y, z, w).
        t.transform.rotation.x = msg.q[1]
        t.transform.rotation.y = msg.q[2]
        t.transform.rotation.z = msg.q[3]
        t.transform.rotation.w = msg.q[0]
        return t

    def _broadcast_odom_callback(self, msg: VehicleOdometry):
        """
        Broadcasts transform on every received odometry message using the data from that message.
        """
        # if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
        #     self.get_logger().warn(f"received odometry message with unsupported pose_frame {msg.pose_frame} or velocity_frame {msg.velocity_frame}. expected ned frames. ignoring this message.")
        #     return
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(f"received odometry message with unsupported pose_frame {msg.pose_frame}. expected ned frame. ignoring this message.")
            return
        t = self._odom2tf(msg, -1) 
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_zero_order_hold(self, msg: VehicleOdometry):
        """
        Broadcasts transform at a fixed rate with zero-order hold strategy.
        """
        if self.t_query_us < 0: # Invalid query timestamp
            self.t_query_us = msg.timestamp_sample # initialize query timestamp with the timestamp of the first received message
        self.msg_queue.append(msg)
        if len(self.msg_queue) == 2:
            while rclpy.ok() and self.t_query_us <= self.msg_queue[-1].timestamp_sample:
                t = self._odom2tf(self.msg_queue[0], self.t_query_us)
                self.tf_broadcaster.sendTransform(t)
                self.t_query_us += int(1e6 / self.publish_rate)

    def _broadcast_lerp(self, msg: VehicleOdometry):
        """
        Broadcasts transform at a fixed rate with linear interpolation strategy.
        """
        if self.t_query_us < 0: # Invalid query timestamp
            self.t_query_us = msg.timestamp_sample # initialize query timestamp with the timestamp of the first received message
        self.msg_queue.append(msg)
        if len(self.msg_queue) == 2:
            while rclpy.ok() and self.t_query_us <= self.msg_queue[-1].timestamp_sample:
                t0 = self._odom2tf(self.msg_queue[0], -1)
                t1 = self._odom2tf(self.msg_queue[1], -1)
                ratio = (self.t_query_us - self.msg_queue[0].timestamp_sample) / (self.msg_queue[1].timestamp_sample - self.msg_queue[0].timestamp_sample)
                t_interp = TransformStamped()
                t_interp.header.stamp.sec = self.t_query_us // 1000_000
                t_interp.header.stamp.nanosec = (self.t_query_us % 1000_000) * 1000
                t_interp.header.frame_id = self.px4_ned_frame_id
                t_interp.child_frame_id = self.base_link_frame_id
                t_interp.transform.translation.x = lerp(self.msg_queue[0].timestamp_sample, t0.transform.translation.x, self.msg_queue[1].timestamp_sample, t1.transform.translation.x, self.t_query_us)
                t_interp.transform.translation.y = lerp(self.msg_queue[0].timestamp_sample, t0.transform.translation.y, self.msg_queue[1].timestamp_sample, t1.transform.translation.y, self.t_query_us)
                t_interp.transform.translation.z = lerp(self.msg_queue[0].timestamp_sample, t0.transform.translation.z, self.msg_queue[1].timestamp_sample, t1.transform.translation.z, self.t_query_us)
                t_interp.transform.rotation.x, t_interp.transform.rotation.y, t_interp.transform.rotation.z, t_interp.transform.rotation.w = slerp(
                    self.msg_queue[0].timestamp_sample,
                    (t0.transform.rotation.w, t0.transform.rotation.x, t0.transform.rotation.y, t0.transform.rotation.z),
                    self.msg_queue[1].timestamp_sample,
                    (t1.transform.rotation.w, t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z),
                    self.t_query_us
                )
                self.tf_broadcaster.sendTransform(t_interp)
                self.t_query_us += int(1e6 / self.publish_rate)

    def _broadcast_ekf(self, msg: VehicleOdometry):
        """
        Broadcasts transform at a fixed rate with extended Kalman filter strategy.
        """
        pass

    def on_exit(self):
        self.get_logger().info("OdomTFBroadcasterNode is shutting down.")

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcasterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.on_exit()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()