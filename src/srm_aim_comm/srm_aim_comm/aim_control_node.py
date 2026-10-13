import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from srm_aim_comm.protocol import (
    pack_upper_frame,
    unpack_upper_frame
)


class AimControlNode(Node):

    def __init__(self):
        super().__init__("aim_control_node")

        self.yaw_error = 0.0
        self.pitch_error = 0.0

        # 数据超时保护：使用单调时钟计算数据新鲜度
        self.last_yaw_time = None
        self.last_pitch_time = None
        self.timeout_sec = 0.5
        self.data_stale = True

        self.yaw_sub = self.create_subscription(
            Float32,
            "/srm_aim/yaw_error",
            self.yaw_callback,
            10
        )

        self.pitch_sub = self.create_subscription(
            Float32,
            "/srm_aim/pitch_error",
            self.pitch_callback,
            10
        )

        self.yaw_pub = self.create_publisher(
            Float32,
            "/srm_aim/cmd_yaw",
            10
        )

        self.pitch_pub = self.create_publisher(
            Float32,
            "/srm_aim/cmd_pitch",
            10
        )

        self.fire_pub = self.create_publisher(
            Int32,
            "/srm_aim/cmd_fire",
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            "aim_control_node started."
        )

    def yaw_callback(self, msg):
        self.yaw_error = msg.data
        self.last_yaw_time = time.monotonic()

    def pitch_callback(self, msg):
        self.pitch_error = msg.data
        self.last_pitch_time = time.monotonic()

    def control_loop(self):
        now = time.monotonic()

        yaw_fresh = (
            self.last_yaw_time is not None
            and now - self.last_yaw_time <= self.timeout_sec
        )
        pitch_fresh = (
            self.last_pitch_time is not None
            and now - self.last_pitch_time <= self.timeout_sec
        )

        # 任一角度数据超时，都不再发布旧角度指令
        if not (yaw_fresh and pitch_fresh):
            fire_msg = Int32()
            fire_msg.data = 0
            self.fire_pub.publish(fire_msg)

            if not self.data_stale:
                self.get_logger().warning(
                    "Angle data timed out; stale commands suppressed."
                )
            self.data_stale = True
            return

        if self.data_stale:
            self.get_logger().info(
                "Fresh yaw and pitch data received; control resumed."
            )
        self.data_stale = False

        cmd_yaw = self.yaw_error
        cmd_pitch = self.pitch_error

        # 暂时禁止开火
        fire_flag = 0

        yaw_msg = Float32()
        yaw_msg.data = cmd_yaw
        self.yaw_pub.publish(yaw_msg)

        pitch_msg = Float32()
        pitch_msg.data = cmd_pitch
        self.pitch_pub.publish(pitch_msg)

        fire_msg = Int32()
        fire_msg.data = fire_flag
        self.fire_pub.publish(fire_msg)

        # 打包成18字节协议帧
        frame = pack_upper_frame(
            cmd_yaw,
            cmd_pitch,
            fire_flag
        )

        self.get_logger().info(
            f"cmd_yaw={cmd_yaw:.2f}, "
            f"cmd_pitch={cmd_pitch:.2f}, "
            f"fire={fire_flag}"
        )

        self.get_logger().info(
            f"frame_len={len(frame)}, "
            f"frame={frame.hex()}"
        )

        # 再把刚刚生成的协议帧解包
        decoded_yaw, decoded_pitch, decoded_fire = \
            unpack_upper_frame(frame)

        self.get_logger().info(
            f"decoded: yaw={decoded_yaw:.2f}, "
            f"pitch={decoded_pitch:.2f}, "
            f"fire={decoded_fire}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = AimControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
