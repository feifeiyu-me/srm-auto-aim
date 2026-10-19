import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from srm_aim_comm.protocol import (
    LowerFrameParser,
    pack_upper_frame,
    unpack_lower_frame,
)


class SerialCommNode(Node):
    def __init__(self):
        super().__init__("serial_comm_node")

        defaults = {
            "port": "/dev/ttyACM0",
            "baudrate": 115200,
            "enable_serial": False,
            "enable_tx": False,
            "reconnect_interval": 2.0,
            "command_timeout_sec": 0.3,
            "feedback_timeout_sec": 0.2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.enable_serial = self.get_parameter("enable_serial").value
        self.enable_tx = self.get_parameter("enable_tx").value
        self.reconnect_interval = float(
            self.get_parameter("reconnect_interval").value
        )
        self.command_timeout = float(
            self.get_parameter("command_timeout_sec").value
        )
        self.feedback_timeout = float(
            self.get_parameter("feedback_timeout_sec").value
        )
        for value in (
            self.reconnect_interval,
            self.command_timeout,
            self.feedback_timeout,
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Timeouts must be finite and positive")

        self.serial_port = None
        self.last_connect_attempt = None
        self.tx_active = False
        self.reset_session()

        for axis in ("yaw", "pitch"):
            self.create_subscription(
                Float32, f"/srm_aim/cmd_{axis}",
                lambda msg, axis=axis: self.command_callback(axis, msg),
                1,
            )

        self.state_pubs = {
            key: self.create_publisher(msg_type, topic, 1)
            for key, msg_type, topic in (
                ("yaw", Float32, "/srm_aim/gimbal_yaw"),
                ("pitch", Float32, "/srm_aim/gimbal_pitch"),
                ("roll", Float32, "/srm_aim/gimbal_roll_feedback"),
                ("bullet_speed", Float32, "/srm_aim/bullet_speed"),
                ("mode", Int32, "/srm_aim/lower_mode"),
                ("color", Int32, "/srm_aim/robot_id"),
            )
        }
        # x/y/z 分别为同一反馈帧的 yaw/pitch/roll，单位为度。
        self.stamped_state_pub = self.create_publisher(
            Vector3Stamped, "/srm_aim/gimbal_state_stamped", 10
        )
        self.connected_pub = self.create_publisher(
            Bool, "/srm_aim/serial_connected", 1
        )
        self.feedback_valid_pub = self.create_publisher(
            Bool, "/srm_aim/feedback_valid", 1
        )

        self.create_timer(0.02, self.timer_callback)
        self.get_logger().info(
            f"Serial enabled={self.enable_serial}, TX enabled={self.enable_tx}; "
            "fire_flag is always 0."
        )

    def reset_session(self):
        self.parser = LowerFrameParser()
        self.last_feedback = None
        self.commands = {"yaw": None, "pitch": None}
        self.command_times = {"yaw": None, "pitch": None}

    def set_tx_active(self, active):
        if active == self.tx_active:
            return
        self.tx_active = active
        if active:
            self.get_logger().info("Fresh feedback and commands: TX started.")
        else:
            self.get_logger().warning(
                "TX stopped. This does not cancel the MCU's stored target."
            )

    def close_serial(self):
        self.set_tx_active(False)
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.reset_session()

    def open_serial(self):
        self.last_connect_attempt = time.monotonic()
        try:
            import serial
            self.serial_port = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0,
                write_timeout=0.02,
                exclusive=True,
            )
            self.reset_session()
            self.get_logger().info(f"Serial opened: {self.port}")
        except Exception as exc:
            self.close_serial()
            self.get_logger().warning(f"Serial open failed: {exc}")

    def command_callback(self, axis, msg):
        now = time.monotonic()
        value = float(msg.data)
        # Do not accept commands before feedback or while feedback is stale.
        if (
            not math.isfinite(value)
            or self.last_feedback is None
            or now - self.last_feedback > self.feedback_timeout
        ):
            self.commands[axis] = None
            self.command_times[axis] = None
            return
        self.commands[axis] = value
        self.command_times[axis] = now

    def publish_state(self, state):
        attitude = Vector3Stamped()
        attitude.header.stamp = self.get_clock().now().to_msg()
        attitude.header.frame_id = "gimbal_feedback_degrees"
        attitude.vector.x = float(state["yaw"])
        attitude.vector.y = float(state["pitch"])
        attitude.vector.z = float(state["roll"])
        self.stamped_state_pub.publish(attitude)
        for key, pub in self.state_pubs.items():
            msg = Int32() if key in ("mode", "color") else Float32()
            msg.data = (
                int(state[key]) if key in ("mode", "color")
                else float(state[key])
            )
            pub.publish(msg)

    def timer_callback(self):
        if self.enable_serial and self.serial_port is None:
            now = time.monotonic()
            if (
                self.last_connect_attempt is None
                or now - self.last_connect_attempt >= self.reconnect_interval
            ):
                self.open_serial()

        connected = (
            self.serial_port is not None and self.serial_port.is_open
        )

        if connected:
            try:
                waiting = self.serial_port.in_waiting
                if waiting:
                    data = self.serial_port.read(min(waiting, 65536))
                    try:
                        frames = self.parser.feed(data)
                    except ValueError as exc:
                        # Parsing failed: invalidate feedback and pending commands.
                        self.reset_session()
                        self.get_logger().warning(f"RX rejected: {exc}")
                        frames = []

                    if frames:
                        # Publish the newest complete state, avoiding ROS backlog.
                        state = unpack_lower_frame(frames[-1])
                        self.last_feedback = time.monotonic()
                        self.publish_state(state)

                now = time.monotonic()
                feedback_fresh = (
                    self.last_feedback is not None
                    and now - self.last_feedback <= self.feedback_timeout
                )
                if not feedback_fresh:
                    self.commands = {"yaw": None, "pitch": None}
                    self.command_times = {"yaw": None, "pitch": None}

                command_fresh = all(
                    self.command_times[axis] is not None
                    and now - self.command_times[axis] <= self.command_timeout
                    for axis in ("yaw", "pitch")
                )
                ready = self.enable_tx and feedback_fresh and command_fresh
                self.set_tx_active(ready)

                if ready:
                    frame = pack_upper_frame(
                        self.commands["yaw"], self.commands["pitch"], 0
                    )
                    if self.serial_port.write(frame) != len(frame):
                        raise IOError("Incomplete serial write")

            except Exception as exc:
                self.get_logger().error(f"Serial I/O failed: {exc}")
                self.close_serial()

        connected = (
            self.serial_port is not None and self.serial_port.is_open
        )
        fresh = (
            connected
            and self.last_feedback is not None
            and time.monotonic() - self.last_feedback <= self.feedback_timeout
        )
        msg = Bool()
        msg.data = connected
        self.connected_pub.publish(msg)
        msg = Bool()
        msg.data = fresh
        self.feedback_valid_pub.publish(msg)

    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialCommNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
