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
from iarc_utils.messagefilters import PX4MessageClamper
from iarc_msgs.msg import Stamp

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
            ParameterDescriptor(description="The strategy for broadcasting the transform. Options are: 1. zero_order_hold: broadcast the transform with the latest received odometry data at a fixed rate. 2. lerp: linearly extrapolate the transform based on the latest two received odometry messages and broadcast at a fixed rate. 3. odom_callback: broadcast the transform only when a new odometry message is received, using the data from that message. 4. ekf: use an extended Kalman filter to estimate the transform based on the received odometry messages and broadcast at a fixed rate."),
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
            self.msg_and_stamp_queue = deque(maxlen=1)  # For storing the latest odometry message for interpolation or EKF
            if self.strategy.lower() == "lerp":
                self.stamp_topic = self.declare_parameter(
                    "stamp_topic",
                    "stamp",
                    ParameterDescriptor(description="The topic name for the Stamp messages used for triggering interpolation when using the 'lerp' strategy."),
                ).value

    def _ros2_init(self):
        """
        Placeholder
        """
        self.tf_broadcaster = TransformBroadcaster(self)
        self.fcn_dict = {
            "zero_order_hold": self._broadcast_zero_order_hold,
            "lerp": self._lerp_timer_callback,
            "ekf": self._broadcast_ekf,
        }
        px4_qosprofile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self.strategy.lower() == "odom_callback":
            self.odom_sub = self.create_subscription(
                VehicleOdometry,
                self.px4_odom_topic,
                self._broadcast_odom_callback,
                px4_qosprofile,
            )
        else:
            self.odom_sub = self.create_subscription(
                VehicleOdometry,
                self.px4_odom_topic,
                self._odom_callback,
                px4_qosprofile,
            )
            if self.strategy.lower() == "lerp":
                self.px4messageclamper = PX4MessageClamper(
                    self,
                    topics=[self.stamp_topic, self.px4_odom_topic],
                    types=[Stamp, VehicleOdometry],
                    qosprofiles=[px4_qosprofile, px4_qosprofile],
                    bufferdepths=[10, 10],
                    callback=self._lerp_clamped_callback
                )
                self.stamp_pub = self.create_publisher(Stamp, self.stamp_topic, px4_qosprofile)
            self.timer = self.create_timer(1. / self.publish_rate, self.fcn_dict[self.strategy.lower()])

    def _odom_callback(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.get_logger().warn(f"received odometry message with unsupported pose_frame {msg.pose_frame} or velocity_frame {msg.velocity_frame}. expected ned frames. ignoring this message.")
            return
        self.msg_and_stamp_queue.append((msg, stamp2us(self.get_clock().now())))

    def _broadcast_odom_callback(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED or msg.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.get_logger().warn(f"received odometry message with unsupported pose_frame {msg.pose_frame} or velocity_frame {msg.velocity_frame}. expected ned frames. ignoring this message.")
            return
        t = TransformStamped()
        t.header.stamp.sec = msg.timestamp_sample // 1000_000
        t.header.stamp.nanosec = (msg.timestamp_sample % 1000_000) * 1000
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
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_zero_order_hold(self):
        if not self.msg_and_stamp_queue:
            return
        time_now = self.get_clock().now()
        msg, stamp_us = self.msg_and_stamp_queue[-1]
        time_offset_us = msg.timestamp_sample - stamp_us
        t = TransformStamped()
        t.header.stamp.nanosec = (time_now.nanosec + (time_offset_us % 1000_000) * 1000) % (10**9)
        t.header.stamp.sec = time_now.sec + time_offset_us // 1000_000 + (time_now.nanosec + (time_offset_us % 1000_000) * 1000) // (10**9)
        t.header.frame_id = self.px4_ned_frame_id
        t.child_frame_id = self.base_link_frame_id
        t.transform.translation.x = msg.position[0]
        t.transform.translation.y = msg.position[1]
        t.transform.translation.z = msg.position[2]
        t.transform.rotation.x = msg.q[1]
        t.transform.rotation.y = msg.q[2]
        t.transform.rotation.z = msg.q[3]
        t.transform.rotation.w = msg.q[0]
        self.tf_broadcaster.sendTransform(t)

    def _lerp_timer_callback(self):
        if not self.msg_and_stamp_queue:
            return
        msg = Stamp()
        msg.timestamp_sample = stamp2us(self.get_clock().now()) + self.msg_and_stamp_queue[-1][0].timestamp_sample - self.msg_and_stamp_queue[-1][1]
        self.stamp_pub.publish(msg)

    def _lerp_clamped_callback(self, stamp_msg: Stamp, odom_msgs: tuple[VehicleOdometry, VehicleOdometry]):
        """
        Placeholder
        """
        odom_msg1, odom_msg2 = odom_msgs
        t = TransformStamped()
        t.header.frame_id = self.px4_ned_frame_id
        t.child_frame_id = self.base_link_frame_id
        t.header.stamp.nanosec = stamp_msg.timestamp_sample * 1000 % (10**9)
        t.header.stamp.sec = (stamp_msg.timestamp_sample) // 1000_000 + (stamp_msg.timestamp_sample * 1000) // (10**9)
        # Linear extrapolation for translation
        pos_lerp = lerp(odom_msg1.timestamp_sample, np.array(odom_msg1.position), odom_msg2.timestamp_sample, np.array(odom_msg2.position), stamp_msg.timestamp_sample)
        t.transform.translation.x = pos_lerp[0]
        t.transform.translation.y = pos_lerp[1]
        t.transform.translation.z = pos_lerp[2]
        # Spherical linear interpolation (slerp) for rotation
        q = slerp(odom_msg1.timestamp_sample, odom_msg1.q, odom_msg2.timestamp_sample, odom_msg2.q, stamp_msg.timestamp_sample)
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        t.transform.rotation.w = q[0]
        self.tf_broadcaster.sendTransform(t)

    def _broadcast_ekf(self):
        # Placeholder for EKF-based broadcasting implementation
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