import rclpy
from rclpy.node import Node
import threading
import socket
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class UDPServerNode(Node):

    def __init__(self):
        super().__init__("udpserver_node")
        # params
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_resolution_length = self.declare_parameter("cmd_resolution_length", 1.0).value # 1 in cmd means this length in meters in reality
        self.cmd_resolution_angle = self.declare_parameter("cmd_resolution_angle", 0.1).value # 1 in cmd means this angle in radians in reality

        offboard_control_mode_publisher_name = self.declare_parameter("offboard_control_mode_publisher_name", "/fmu/in/offboard_control_mode").value
        trajectory_setpoint_publisher_name = self.declare_parameter("trajectory_setpoint_publisher_name", "/fmu/in/trajectory_setpoint").value
        vehicle_command_publisher_name = self.declare_parameter("vehicle_command_publisher_name", "/fmu/in/vehicle_command").value

        self.timer_period = self.declare_parameter("timer_period", 0.1).value # seconds

        self.udp_port_recv = self.declare_parameter("udp_port_recv", 5005).value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.udp_port_recv))
        self.lock = threading.Lock()
        self.udp_thread = threading.Thread(target=self.udp_listening_task, daemon=True)

        self.target_pos = [0.0, 0.0, 0.0, 0.0] # x, y, z, yaw
        self.cmd_id_now = -1
        self.offboard_counter = 0

        # ros2 components
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, offboard_control_mode_publisher_name, qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, trajectory_setpoint_publisher_name, qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, vehicle_command_publisher_name, qos_profile)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.udp_thread.start()
        self.get_logger().info("UDP Server Node has been started.")

    def udp_listening_task(self):
        while rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                self.get_logger().debug(f"Received UDP data: {data.decode()}")
                cmd = json.loads(data.decode())
                with self.lock:
                    if cmd['s'] > self.cmd_id_now:
                        self.cmd_id_now = cmd['s']
                        match cmd['c']:
                            case 'a': # arm
                                self.arm()
                            case 'd': # disarm
                                self.disarm()
                            case 'm': # move
                                self.target_pos = [
                                    cmd['x'] * self.cmd_resolution_length,
                                    cmd['y'] * self.cmd_resolution_length,
                                    cmd['z'] * self.cmd_resolution_length,
                                    cmd['yaw'] * self.cmd_resolution_angle
                                ]
            except Exception as e:
                self.get_logger().error(f"Error processing UDP data: {e}")

    def send_position_command(self):
        msg = TrajectorySetpoint()
        msg.position = [
            self.target_pos[0],
            - self.target_pos[1],
            - self.target_pos[2]
        ] # px4 uses NED frame, so y and z are inverted
        msg.yaw = self.target_pos[3]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
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

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def timer_callback(self):
        self.publish_offboard_control_heartbeat_signal()
        if self.offboard_counter == 10: # publish heartbeat signal for 1 second before switching to offboard mode, to ensure px4 is ready
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0) # switch to offboard mode
            self.get_logger().info("Switching to offboard mode")

        if self.offboard_counter < 11:
            self.offboard_counter += 1
        else:
            self.send_position_command()

def main():
    rclpy.init()
    node = UDPServerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()