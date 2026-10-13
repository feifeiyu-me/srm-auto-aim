import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32


class AimSolverNode(Node):

    def __init__(self):
        super().__init__("aim_solver_node")

        self.target_x = None
        self.target_y = None
        self.target_valid = False
        self.target_stamp = None
        self.target_timeout_sec = 0.3

        self.gimbal_yaw = 0.0
        self.gimbal_pitch = 0.0
        self.have_yaw_feedback = False
        self.have_pitch_feedback = False

        # 模拟默认值，不代表 CS106 的真实标定结果
        for name, value in [
            ("image_width", 1280.0), ("image_height", 720.0),
            ("fx", 800.0), ("fy", 800.0),
            ("cx", 640.0), ("cy", 360.0),
        ]:
            self.declare_parameter(name, value)

        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)

        if min(self.image_width, self.image_height, self.fx, self.fy) <= 0:
            raise ValueError("Image dimensions and focal lengths must be positive")

        self.target_sub = self.create_subscription(
            PointStamped,
            "/srm_aim/target",
            self.target_callback,
            10
        )

        self.yaw_feedback_sub = self.create_subscription(
            Float32,
            "/srm_aim/gimbal_yaw",
            self.yaw_feedback_callback,
            10
        )

        self.pitch_feedback_sub = self.create_subscription(
            Float32,
            "/srm_aim/gimbal_pitch",
            self.pitch_feedback_callback,
            10
        )

        # 发布目标角度指令，供控制节点使用
        self.yaw_error_pub = self.create_publisher(
            Float32,
            "/srm_aim/yaw_error",
            10
        )

        self.pitch_error_pub = self.create_publisher(
            Float32,
            "/srm_aim/pitch_error",
            10
        )

        self.timer = self.create_timer(0.1, self.solve)

        self.get_logger().info(
            "Aim solver started with simulated gimbal feedback."
        )

    def target_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.target_valid = msg.point.z > 0.5
        self.target_stamp = msg.header.stamp

    def yaw_feedback_callback(self, msg):
        self.gimbal_yaw = msg.data
        self.have_yaw_feedback = True

    def pitch_feedback_callback(self, msg):
        self.gimbal_pitch = msg.data
        self.have_pitch_feedback = True

    def solve(self):
        if (
            self.target_x is None
            or self.target_y is None
            or not self.target_valid
            or self.target_stamp is None
            or not self.have_yaw_feedback
            or not self.have_pitch_feedback
        ):
            return

        stamp_sec = (self.target_stamp.sec +
                     self.target_stamp.nanosec * 1e-9)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        target_age = now_sec - stamp_sec
        if target_age < 0.0 or target_age > self.target_timeout_sec:
            return

        if not (0.0 <= self.target_x < self.image_width
                and 0.0 <= self.target_y < self.image_height):
            return

        dx = self.target_x - self.cx
        dy = self.target_y - self.cy

        # 针孔模型：像素偏差换算为角度（度）
        yaw_offset = math.degrees(math.atan(dx / self.fx))
        # 暂定图像上方对应正俯仰，实机方向需联调确认
        pitch_offset = math.degrees(math.atan(-dy / self.fy))

        # 绝对目标角度 = 当前云台角度 + 相对角度
        target_yaw = self.gimbal_yaw + yaw_offset
        target_pitch = self.gimbal_pitch + pitch_offset

        yaw_msg = Float32()
        yaw_msg.data = target_yaw
        self.yaw_error_pub.publish(yaw_msg)

        pitch_msg = Float32()
        pitch_msg.data = target_pitch
        self.pitch_error_pub.publish(pitch_msg)

        self.get_logger().info(
            f"dx={dx:.1f}, dy={dy:.1f}, "
            f"target_yaw={target_yaw:.2f}, "
            f"target_pitch={target_pitch:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AimSolverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
