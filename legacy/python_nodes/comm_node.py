import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from srm_aim_comm.protocol import unpack_upper_frame


class CommNode(Node):

    def __init__(self):
        super().__init__("comm_node")

        # 发布 yaw
        self.yaw_pub = self.create_publisher(
            Float32,
            "/srm_aim/yaw",
            10
        )

        # 发布 pitch
        self.pitch_pub = self.create_publisher(
            Float32,
            "/srm_aim/pitch",
            10
        )

        # 发布开火标志
        self.fire_pub = self.create_publisher(
            Int32,
            "/srm_aim/fire_flag",
            10
        )

        # 测试数据
        self.timer = self.create_timer(
            1.0,
            self.timer_callback
        )

        self.get_logger().info(
            "srm_aim_comm node started."
        )

    def timer_callback(self):

        # 模拟收到的18字节数据
        frame = bytes.fromhex(
            "1000010000002841333313c0020001000000"
        )

        try:

            yaw, pitch, fire_flag = unpack_upper_frame(
                frame
            )

            # yaw
            yaw_msg = Float32()
            yaw_msg.data = yaw
            self.yaw_pub.publish(yaw_msg)

            # pitch
            pitch_msg = Float32()
            pitch_msg.data = pitch
            self.pitch_pub.publish(pitch_msg)

            # fire
            fire_msg = Int32()
            fire_msg.data = fire_flag
            self.fire_pub.publish(fire_msg)

            self.get_logger().info(
                f"yaw={yaw:.2f}, "
                f"pitch={pitch:.2f}, "
                f"fire_flag={fire_flag}"
            )

        except ValueError as e:

            self.get_logger().error(
                f"Protocol error: {e}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = CommNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
