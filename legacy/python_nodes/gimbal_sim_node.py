import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class GimbalSimNode(Node):
    """简化云台动态模型：模拟角度响应与反馈。"""

    def __init__(self):
        super().__init__("gimbal_sim_node")

        self.declare_parameter("response_gain", 0.5)
        self.declare_parameter("publish_rate", 10.0)

        self.response_gain = float(
            self.get_parameter("response_gain").value
        )
        publish_rate = float(
            self.get_parameter("publish_rate").value
        )

        if not 0.0 < self.response_gain <= 1.0:
            raise ValueError("response_gain 必须在 (0, 1] 范围内")
        if publish_rate <= 0.0:
            raise ValueError("publish_rate 必须大于 0")

        self.yaw_cmd = 0.0
        self.pitch_cmd = 0.0
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0

        self.create_subscription(
            Float32, "/srm_aim/cmd_yaw",
            self.yaw_cmd_callback, 10
        )
        self.create_subscription(
            Float32, "/srm_aim/cmd_pitch",
            self.pitch_cmd_callback, 10
        )

        self.yaw_state_pub = self.create_publisher(
            Float32, "/srm_aim/gimbal_yaw", 10
        )
        self.pitch_state_pub = self.create_publisher(
            Float32, "/srm_aim/gimbal_pitch", 10
        )

        self.timer = self.create_timer(
            1.0 / publish_rate, self.update
        )

        self.get_logger().info(
            "Gimbal simulation started; fire remains disabled."
        )

    def yaw_cmd_callback(self, msg):
        self.yaw_cmd = msg.data

    def pitch_cmd_callback(self, msg):
        self.pitch_cmd = msg.data

    def update(self):
        # 一阶响应模型：云台角度逐步逼近目标命令。
        self.yaw_angle += self.response_gain * (
            self.yaw_cmd - self.yaw_angle
        )
        self.pitch_angle += self.response_gain * (
            self.pitch_cmd - self.pitch_angle
        )

        yaw_msg = Float32()
        yaw_msg.data = self.yaw_angle
        self.yaw_state_pub.publish(yaw_msg)

        pitch_msg = Float32()
        pitch_msg.data = self.pitch_angle
        self.pitch_state_pub.publish(pitch_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
