"""
SetpointSenderNode: A ROS 2 node that listens for UDP commands and translates them into PX4-compatible messages.

This node receives commands over UDP, processes them, and publishes appropriate PX4 messages for offboard control.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_srvs.srv import Empty
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
import threading
import socket
import json
import math
from iarc_utils.mathematics import euler_to_quaternion, ros2_quaternion_to_euler


class SetpointSenderNode(Node):
    def __init__(self):
        super().__init__("setpoint_sender_node")
        self._params_init()
        self._ros2_init()
        self.get_logger().info("Setpoint Sender Node has been started.")

    def _params_init(self):
        """Initialize parameters."""
        self.cmd_resolution_length = self.declare_parameter(
            "cmd_resolution_length",
            1.0,
            ParameterDescriptor(
                description="Resolution for length commands (meters per unit)."
            ),
        ).value
        self.cmd_resolution_angle = self.declare_parameter(
            "cmd_resolution_angle",
            0.1,
            ParameterDescriptor(
                description="Resolution for angle commands (radians per unit)."
            ),
        ).value
        self.offboard_control_mode_publisher_name = self.declare_parameter(
            "offboard_control_mode_publisher_name",
            "/fmu/in/offboard_control_mode",
            ParameterDescriptor(
                description="Topic name for OffboardControlMode messages."
            ),
        ).value
        self.trajectory_setpoint_publisher_name = self.declare_parameter(
            "trajectory_setpoint_publisher_name",
            "/fmu/in/trajectory_setpoint",
            ParameterDescriptor(
                description="Topic name for TrajectorySetpoint messages."
            ),
        ).value
        self.vehicle_command_publisher_name = self.declare_parameter(
            "vehicle_command_publisher_name",
            "/fmu/in/vehicle_command",
            ParameterDescriptor(description="Topic name for VehicleCommand messages."),
        ).value
        self.timer_period = self.declare_parameter(
            "timer_period",
            0.1,
            ParameterDescriptor(
                description="Timer period for publishing setpoints (seconds)."
            ),
        ).value
        self.udp_port_recv = self.declare_parameter(
            "udp_port_recv",
            5005,
            ParameterDescriptor(description="UDP port for receiving commands."),
        ).value
        self.arena_frame_id = self.declare_parameter(
            "arena_frame_id",
            "map",
            ParameterDescriptor(
                description="Frame ID for the arena coordinate system."
            ),
        ).value
        self.px4_ned_frame_id = self.declare_parameter(
            "px4_ned_frame_id",
            "px4_ned",
            ParameterDescriptor(
                description="Frame ID for the PX4 NED coordinate system."
            ),
        ).value
        self.setpoint_bias = self.declare_parameter(
            "setpoint_bias",
            [0.0, 0.0, 0.0, 0.0],
            ParameterDescriptor(
                description="Bias to be added to setpoints (x, y, z, yaw) in meters and radians to avoid multiple drones colliding at the same point."
            ),
        ).value
        self.tf_lookup_timeout_s = self.declare_parameter(
            "tf_lookup_timeout_s",
            1.0,
            ParameterDescriptor(description="Timeout for TF lookups in seconds."),
        ).value
        self.reset_srv_name = self.declare_parameter(
            "reset_srv_name",
            "reset",
            ParameterDescriptor(
                description="Service name for resetting setpoint node."
            ),
        ).value

    def _ros2_init(self):
        """Create publishers, TF listener, UDP socket, and start the UDP thread.

        Blocks until a valid transform from ``arena_frame_id`` to
        ``px4_ned_frame_id`` is available (default timeout 1 s).
        """
        # TODO: the blocking TF wait in the constructor can deadlock if the
        #       FrameTransformerNode is not running.  Consider moving TF
        #       acquisition into the timer callback with a fallback.
        px4_qosprofile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        while rclpy.ok() and not self.tf_buffer.can_transform(
            self.px4_ned_frame_id,
            self.arena_frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.tf_lookup_timeout_s),
        ):
            self.get_logger().warn(
                f"Waiting for TF transform from {self.arena_frame_id} to {self.px4_ned_frame_id}..."
            )
        self.transform = self.tf_buffer.lookup_transform(
            self.px4_ned_frame_id,
            self.arena_frame_id,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=self.tf_lookup_timeout_s),
        )
        self.get_logger().info(
            f"TF transform from {self.arena_frame_id} to {self.px4_ned_frame_id} is now available. You can control the UAV now."
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(("", self.udp_port_recv))
        self.lock = threading.Lock()
        self.udp_thread = threading.Thread(target=self._udp_listening_task, daemon=True)

        self.target_pos = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw in arena frame
        self.cmd_id_now = -1
        self.offboard_counter = 0  # number of heartbeats published before arming

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            self.offboard_control_mode_publisher_name,
            px4_qosprofile,
        )
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, self.trajectory_setpoint_publisher_name, px4_qosprofile
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, self.vehicle_command_publisher_name, px4_qosprofile
        )

        self.timer = self.create_timer(self.timer_period, self._timer_callback)

        self.reset_srv = self.create_service(
            Empty, self.reset_srv_name, self._reset_callback
        )

        self.udp_thread.start()

    def _reset_callback(self, request, response):
        """Reset setpoint and heartbeat counter to prepared state (response to empty service call)."""
        with self.lock:
            self.target_pos = [0.0, 0.0, 0.0, 0.0]
            self.cmd_id_now = -1
            self.offboard_counter = 0
        self.get_logger().info("Setpoint Sender Node has been reset to initial state.")
        return response

    def _udp_listening_task(self):
        """Blocking thread: receive UDP JSON commands and update target setpoint.

        Protocol: {"s": <seq>, "c": <cmd>, "x": ..., "y": ..., "z": ..., "yaw": ...}
        Commands are ignored if their sequence number is not newer than the last
        processed command.
        """
        # TODO: may need frame transforms for the setpoint — currently only
        #       sets arena-frame target and transforms in the timer callback.
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                self.get_logger().debug(f"Received UDP data: {data.decode()}")
                cmd = json.loads(data.decode())
                with self.lock:
                    if cmd["s"] > self.cmd_id_now:
                        self.cmd_id_now = cmd["s"]
                        match cmd["c"]:
                            case "a":  # arm
                                self._arm()
                            case "d":  # disarm
                                self._disarm()
                            case "m":  # move
                                self.target_pos = [
                                    cmd["x"] * self.cmd_resolution_length,
                                    cmd["y"] * self.cmd_resolution_length,
                                    cmd["z"] * self.cmd_resolution_length,
                                    cmd["yaw"] * self.cmd_resolution_angle,
                                ]
            except Exception as e:
                self.get_logger().error(f"Error processing UDP data: {e}")

    def _transform_setpoint(
        self, setpoint: list[float], tf: TransformStamped
    ) -> list[float]:
        """Transform a setpoint from the arena frame to the PX4 NED frame."""
        pose = PoseStamped()
        pose.header.frame_id = self.arena_frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = setpoint[0] + self.setpoint_bias[0]
        pose.pose.position.y = setpoint[1] + self.setpoint_bias[1]
        pose.pose.position.z = setpoint[2] + self.setpoint_bias[2]
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = euler_to_quaternion(0.0, 0.0, setpoint[3])
        transformed_pose = do_transform_pose(pose, tf)
        return [
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z,
            ros2_quaternion_to_euler(transformed_pose.pose.orientation)[2],  # Yaw
        ]

    def _send_position_command(self):
        """Convert arena-frame target to NED and publish a TrajectorySetpoint."""
        msg = TrajectorySetpoint()
        local_setpoint = self._transform_setpoint(self.target_pos, self.transform)
        msg.position = [
            local_setpoint[0],
            local_setpoint[1],
            local_setpoint[2],
        ]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.jerk = [math.nan] * 3
        msg.yaw = local_setpoint[3]
        msg.yaw_speed = math.nan
        # TODO: msg.timestamp is PX4 boot time (us), but
        #       self.get_clock().now() returns wall time (ns).  PX4
        #       ignores unknown timestamps, so this is not critical, but
        #       the value is semantically wrong.
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        """Publish a VehicleCommand with the given command ID and up to 7 float parameters."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def _publish_offboard_control_heartbeat_signal(self):
        """Send OffboardControlMode (position-only) required to keep the offboard mode active."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def _arm(self):
        """Send an arm command to the vehicle."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
        )
        self.get_logger().info("Arm command sent")

    def _disarm(self):
        """Send a disarm command to the vehicle."""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
        )
        self.get_logger().info("Disarm command sent")

    def _timer_callback(self):
        """Publish heartbeat + optionally switch to offboard or send position setpoints.

        The first 11 timer ticks (~1.0 s at 10 Hz) publish heartbeats only.
        At tick 10 the mode is switched to offboard; from tick 11 onward
        position setpoints are streamed at the timer rate.
        """
        self._publish_offboard_control_heartbeat_signal()
        if self.offboard_counter == 10:  # 1 s at 10 Hz → switch to offboard
            # TODO: hard-coded offboard mode switch after 1 s assumes the
            #       pilot has already armed the vehicle manually.  For fully
            #       autonomous missions this should also arm first (or accept
            #       an 'a' UDP command before sending position setpoints).
            self._publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )

        if self.offboard_counter < 11:
            self.offboard_counter += 1
        else:
            self._send_position_command()

    def on_exit(self):
        """Close the UDP socket and log shutdown."""
        # TODO: the daemon UDP thread is not explicitly joined — it dies
        #       with the process.  For graceful shutdown, signal the thread
        #       to exit before the socket is closed.
        self.sock.close()
        self.get_logger().info("Setpoint Sender Node has been shut down.")


def main():
    rclpy.init()
    node = SetpointSenderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.on_exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()
