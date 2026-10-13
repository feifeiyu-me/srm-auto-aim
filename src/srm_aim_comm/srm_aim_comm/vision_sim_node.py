import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class VisionSimNode(Node):

    def __init__(self):
        super().__init__("vision_sim_node")

        self.image_width = 1280.0
        self.image_height = 720.0
        self.center_x = self.image_width / 2.0
        self.center_y = self.image_height / 2.0

        # 模拟目标的固定世界角度，单位：度
        self.target_world_yaw = 10.0
        self.target_world_pitch = 0.0

        # 模拟云台当前角度反馈，单位：度
        self.gimbal_yaw = 0.0
        self.gimbal_pitch = 0.0

        # 临时模拟参数：每度对应的像素数
        self.pixels_per_degree = 20.0

        self.create_subscription(
            Float32,
            "/srm_aim/gimbal_yaw",
            self.yaw_callback,
            10
        )
        self.create_subscription(
            Float32,
            "/srm_aim/gimbal_pitch",
            self.pitch_callback,
            10
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            "/srm_aim/target",
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info(
            "Vision simulation started; fire remains disabled."
        )

    def yaw_callback(self, msg):
        self.gimbal_yaw = msg.data

    def pitch_callback(self, msg):
        self.gimbal_pitch = msg.data

    def timer_callback(self):
        # 云台转向目标后，目标相对图像中心的偏差随之减小
        dx = (
            self.target_world_yaw - self.gimbal_yaw
        ) * self.pixels_per_degree

        dy = (
            self.target_world_pitch - self.gimbal_pitch
        ) * self.pixels_per_degree

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.point.x = self.center_x + dx
        msg.point.y = self.center_y + dy
        msg.point.z = 1.0

        self.target_pub.publish(msg)

        self.get_logger().info(
            f"target=({msg.point.x:.1f}, {msg.point.y:.1f}), "
            f"gimbal_yaw={self.gimbal_yaw:.2f}, "
            f"gimbal_pitch={self.gimbal_pitch:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisionSimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
