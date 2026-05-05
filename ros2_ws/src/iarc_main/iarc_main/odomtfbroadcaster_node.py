"""OdomTFBroadcasterNode: Broadcast odometry-to-base_link transform from PX4 VehicleOdometry messages.

Supports four strategies:
  - odom_callback: publish one transform per incoming odometry message.
  - zero_order_hold: fill transform gaps with the previous pose at a fixed output rate.
  - lerp: linearly interpolate position + slerp orientation between the two latest messages.
  - ekf: placeholder (not implemented — falls through silently).
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
    """
    Converts PX4 VehicleOdometry to tf2 transforms and broadcasts them.

    Depending on the ``strategy`` parameter the node either publishes transforms
    per-message or at a user-specified rate, optionally with interpolation.
    """

    def __init__(self):
        super().__init__("odom_tf_broadcaster_node")
        self._params_init()
        self._ros2_init()
        self.get_logger().info("OdomTFBroadcasterNode has been started.")

    def _params_init(self):
        """
        Declare all ROS parameters and initialise internal state.

        Maps the strategy string to the appropriate subscription callback.
        """
        self.px4_odom_topic = self.declare_parameter(
            "px4_odom_topic",
            "",
            ParameterDescriptor(
                description="The topic name for the PX4 odometry messages (VehicleOdometry). This topic is expected to be at a high frequency."
            ),
        ).value
        self.px4_ned_frame_id = self.declare_parameter(
            "px4_ned_frame_id",
            "px4_ned",
            ParameterDescriptor(
                description="The frame ID for the PX4 odometry messages."
            ),
        ).value
        self.base_link_frame_id = self.declare_parameter(
            "base_link_frame_id",
            "base_link",
            ParameterDescriptor(description="The frame ID for the robot's base link."),
        ).value
        self.strategy = self.declare_parameter(
            "strategy",
            "odom_callback",
            ParameterDescriptor(
                description="The strategy for broadcasting the transform. Options are: 1. zero_order_hold: broadcast the transform with the latest received odometry data at a fixed rate. 2. lerp: linearly interpolate the transform based on the latest two received odometry messages and broadcast at a fixed rate. 3. odom_callback: broadcast the transform only when a new odometry message is received, using the data from that message. 4. ekf: use an extended Kalman filter to estimate the transform based on the received odometry messages and broadcast at a fixed rate."
            ),
        ).value
        assert self.strategy.lower() in [
            "zero_order_hold",
            "lerp",
            "odom_callback",
            "ekf",
        ], (
            "Invalid strategy parameter. Must be one of 'zero_order_hold', 'lerp', 'odom_callback', or 'ekf'."
        )
        if self.strategy.lower() != "odom_callback":
            self.publish_rate = self.declare_parameter(
                "publish_rate",
                50.0,
                ParameterDescriptor(
                    description="The rate (in Hz) at which to publish the transform when using strategies other than 'odom_callback'."
                ),
            ).value

            self.msg_queue = deque(
                maxlen=2
            )  # For storing the latest 2 odometry messages.
            self.t_query_us = -1

    def _ros2_init(self):
        """
        Create TF broadcaster, subscription, and strategy lookup table.

        The subscription callback is selected at construction time based on
        the ``strategy`` parameter, so the node does not use a timer.
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
        """Buffer validated odometry messages for zero_order_hold / lerp strategies."""
        if (
            msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
            or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED
        ):
            self.get_logger().warn(
                f"received odometry message with unsupported pose_frame {msg.pose_frame} or velocity_frame {msg.velocity_frame}. expected ned frames. ignoring this message."
            )
            return
        self.msg_queue.append((msg, stamp2us(self.get_clock().now())))

    def _odom2tf(self, msg: VehicleOdometry, stamp: int) -> TransformStamped:
        """Build a TransformStamped from a VehicleOdometry message and a microsecond stamp.

        When ``stamp < 0`` the message's own ``timestamp_sample`` is used.
        Position and orientation are copied directly; no frame validation is
        performed — the caller must ensure NED frames.
        """
        # TODO: validate msg.pose_frame == POSE_FRAME_NED inside this helper
        #       so that callers (zero_order_hold, lerp) also get the check.
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
        Publish one transform per incoming odometry message.

        Timestamp is taken from the message's PX4 boot-time ``timestamp_sample``,
        which is sufficient for tf2 lookups that only care about relative ordering
        within the same clock domain.
        """
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.get_logger().warn(
                f"received odometry message with unsupported pose_frame {msg.pose_frame}. expected ned frame. ignoring this message."
            )
            return
        t = self._odom2tf(msg, -1)
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_zero_order_hold(self, msg: VehicleOdometry):
        """
        Broadcasts transform at a fixed rate with zero-order hold strategy.
        """
        if self.t_query_us < 0:  # Invalid query timestamp
            self.t_query_us = msg.timestamp_sample  # initialize query timestamp with the timestamp of the first received message
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
        if self.t_query_us < 0:  # Invalid query timestamp
            self.t_query_us = msg.timestamp_sample  # initialize query timestamp with the timestamp of the first received message
        self.msg_queue.append(msg)
        if len(self.msg_queue) == 2:
            while rclpy.ok() and self.t_query_us <= self.msg_queue[-1].timestamp_sample:
                t0 = self._odom2tf(self.msg_queue[0], -1)
                t1 = self._odom2tf(self.msg_queue[1], -1)
                t_interp = TransformStamped()
                t_interp.header.stamp.sec = self.t_query_us // 1000_000
                t_interp.header.stamp.nanosec = (self.t_query_us % 1000_000) * 1000
                t_interp.header.frame_id = self.px4_ned_frame_id
                t_interp.child_frame_id = self.base_link_frame_id
                t_interp.transform.translation.x = lerp(
                    self.msg_queue[0].timestamp_sample,
                    t0.transform.translation.x,
                    self.msg_queue[1].timestamp_sample,
                    t1.transform.translation.x,
                    self.t_query_us,
                )
                t_interp.transform.translation.y = lerp(
                    self.msg_queue[0].timestamp_sample,
                    t0.transform.translation.y,
                    self.msg_queue[1].timestamp_sample,
                    t1.transform.translation.y,
                    self.t_query_us,
                )
                t_interp.transform.translation.z = lerp(
                    self.msg_queue[0].timestamp_sample,
                    t0.transform.translation.z,
                    self.msg_queue[1].timestamp_sample,
                    t1.transform.translation.z,
                    self.t_query_us,
                )
                # slerp returns (w, x, y, z) — PX4 / Hamilton convention.
                # Map back to geometry_msgs (x, y, z, w).
                q_interp = slerp(
                    self.msg_queue[0].timestamp_sample,
                    (
                        t0.transform.rotation.w,
                        t0.transform.rotation.x,
                        t0.transform.rotation.y,
                        t0.transform.rotation.z,
                    ),
                    self.msg_queue[1].timestamp_sample,
                    (
                        t1.transform.rotation.w,
                        t1.transform.rotation.x,
                        t1.transform.rotation.y,
                        t1.transform.rotation.z,
                    ),
                    self.t_query_us,
                )
                t_interp.transform.rotation.w = q_interp[0]
                t_interp.transform.rotation.x = q_interp[1]
                t_interp.transform.rotation.y = q_interp[2]
                t_interp.transform.rotation.z = q_interp[3]
                # TODO: the tight while-loop in the subscription callback can flood
                #       the TF broadcaster during catch-up and may cause excessive
                #       load.  Consider migrating to a timer callback that consumes
                #       the queue at the target publish rate instead.
                self.tf_broadcaster.sendTransform(t_interp)
                self.t_query_us += int(1e6 / self.publish_rate)

    def _broadcast_ekf(self, msg: VehicleOdometry):
        """
        Broadcasts transform at a fixed rate with extended Kalman filter strategy.
        """
        # TODO: implement EKF-based odometry smoothing (body-frame velocity + attitude
        #       integration).  For now this strategy silently does nothing.
        self.get_logger().error(
            "EKF strategy selected but not implemented — no transforms are being published.  "
            "Switch to 'odom_callback', 'zero_order_hold', or 'lerp'."
        )

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
