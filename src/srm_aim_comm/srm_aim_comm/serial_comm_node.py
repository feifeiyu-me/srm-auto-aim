import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool

from srm_aim_comm.protocol import (
    pack_upper_frame,
    unpack_lower_frame,
    LowerFrameParser,
)


class SerialCommNode(Node):
    def __init__(self):
        super().__init__("serial_comm_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("enable_serial", False)
        self.declare_parameter("reconnect_interval", 2.0)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.enable_serial = self.get_parameter("enable_serial").value
        self.reconnect_interval = float(
            self.get_parameter("reconnect_interval").value
        )

        self.serial_port = None
        self.parser = LowerFrameParser()
        self.last_connect_attempt = 0.0

        self.yaw = 0.0
        self.pitch = 0.0

        self.create_subscription(
            Float32, "/srm_aim/cmd_yaw", self.yaw_callback, 10
        )
        self.create_subscription(
            Float32, "/srm_aim/cmd_pitch", self.pitch_callback, 10
        )

        self.yaw_pub = self.create_publisher(
            Float32, "/srm_aim/gimbal_yaw_feedback", 10
        )
        self.pitch_pub = self.create_publisher(
            Float32, "/srm_aim/gimbal_pitch_feedback", 10
        )
        self.roll_pub = self.create_publisher(
            Float32, "/srm_aim/gimbal_roll_feedback", 10
        )
        self.speed_pub = self.create_publisher(
            Float32, "/srm_aim/bullet_speed", 10
        )
        self.mode_pub = self.create_publisher(
            Int32, "/srm_aim/lower_mode", 10
        )
        self.color_pub = self.create_publisher(
            Int32, "/srm_aim/robot_id", 10
        )
        self.connected_pub = self.create_publisher(
            Bool, "/srm_aim/serial_connected", 10
        )

        self.create_timer(0.02, self.timer_callback)

        if self.enable_serial:
            self.open_serial()
        else:
            self.get_logger().warning(
                "串口已禁用：当前仅运行安全模式，不发送数据。"
            )

    def open_serial(self):
        self.last_connect_attempt = time.monotonic()

        try:
            import serial

            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.01,
                write_timeout=0.05,
            )
            self.parser = LowerFrameParser()
            self.get_logger().info(f"串口已打开：{self.port}")

        except Exception as exc:
            self.serial_port = None
            self.get_logger().warning(f"打开串口失败：{exc}")

    def yaw_callback(self, msg):
        self.yaw = float(msg.data)

    def pitch_callback(self, msg):
        self.pitch = float(msg.data)

    def timer_callback(self):
        connected = (
            self.serial_port is not None
            and self.serial_port.is_open
        )

        connected_msg = Bool()
        connected_msg.data = connected
        self.connected_pub.publish(connected_msg)

        if not connected:
            if self.enable_serial:
                now = time.monotonic()
                if now - self.last_connect_attempt >= self.reconnect_interval:
                    self.open_serial()
            return

        # 安全要求：始终发送 fire_flag=0。
        frame = pack_upper_frame(
            yaw=self.yaw,
            pitch=self.pitch,
            fire_flag=0,
        )

        try:
            self.serial_port.write(frame)

            data = self.serial_port.read(
                self.serial_port.in_waiting or 1
            )

            if data:
                frames = self.parser.feed(data)

                for frame_data in frames:
                    state = unpack_lower_frame(frame_data)
                    self.publish_state(state)

        except Exception as exc:
            self.get_logger().error(f"串口收发错误，准备重连：{exc}")

            try:
                self.serial_port.close()
            except Exception:
                pass

            self.serial_port = None
            self.parser = LowerFrameParser()

    def publish_state(self, state):
        yaw_msg = Float32()
        yaw_msg.data = float(state["yaw"])
        self.yaw_pub.publish(yaw_msg)

        pitch_msg = Float32()
        pitch_msg.data = float(state["pitch"])
        self.pitch_pub.publish(pitch_msg)

        roll_msg = Float32()
        roll_msg.data = float(state["roll"])
        self.roll_pub.publish(roll_msg)

        speed_msg = Float32()
        speed_msg.data = float(state["bullet_speed"])
        self.speed_pub.publish(speed_msg)

        mode_msg = Int32()
        mode_msg.data = int(state["mode"])
        self.mode_pub.publish(mode_msg)

        color_msg = Int32()
        color_msg.data = int(state["color"])
        self.color_pub.publish(color_msg)

    def destroy_node(self):
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
