import math
import time

from srm_aim_comm.ballistics import solve_ballistic_pitch
from srm_aim_comm.angle_tracker import AngleTracker

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
        self.yaw_tracker = AngleTracker()
        self.tracked_stamp_ns = None
        self.target_gimbal_yaw = None

        self.gimbal_yaw = 0.0
        self.gimbal_pitch = 0.0
        self.have_yaw_feedback = False
        self.have_pitch_feedback = False

        # 模拟默认值，不代表 CS106 的真实标定结果
        for name, value in [
            ("image_width", 1280.0), ("image_height", 720.0),
            ("fx", 800.0), ("fy", 800.0),
            ("cx", 640.0), ("cy", 360.0),
            ("enable_ballistics", False),
            ("horizontal_distance", 5.0),
            ("height_difference", 0.0),
            ("bullet_speed", 22.0),
            ("use_bullet_speed_feedback", False),
            ("bullet_speed_timeout_sec", 0.5),
            ("pitch_up_sign", 1.0),
            ("enable_prediction", False),
            ("actuation_delay_sec", 0.0),
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

        self.feedback_bullet_speed = None
        self.bullet_speed_received_at = None
        self.bullet_speed_sub = self.create_subscription(
            Float32,
            "/srm_aim/bullet_speed",
            self.bullet_speed_callback,
            10
        )

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

    def bullet_speed_callback(self, msg):
        speed = float(msg.data)
        if not math.isfinite(speed) or speed <= 0.0:
            self.feedback_bullet_speed = None
            self.bullet_speed_received_at = None
            return
        self.feedback_bullet_speed = speed
        self.bullet_speed_received_at = time.monotonic()

    def target_callback(self, msg):
        self.target_x = msg.point.x
        self.target_y = msg.point.y
        self.target_valid = msg.point.z > 0.5
        self.target_stamp = msg.header.stamp
        # 暂用目标接收时的云台反馈；实机后续做时间同步。
        self.target_gimbal_yaw = (
            self.gimbal_yaw if self.have_yaw_feedback else None
        )
        if not self.target_valid:
            self.yaw_tracker.reset()
            self.tracked_stamp_ns = None

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
            self.yaw_tracker.reset()
            self.tracked_stamp_ns = None
            return

        if not (0.0 <= self.target_x < self.image_width
                and 0.0 <= self.target_y < self.image_height):
            self.yaw_tracker.reset()
            self.tracked_stamp_ns = None
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

        if self.get_parameter("enable_prediction").value:
            lead = float(self.get_parameter("actuation_delay_sec").value)
            if not math.isfinite(lead) or not 0.0 <= lead <= 0.5:
                return
            if self.target_gimbal_yaw is None:
                return

            observed_yaw = self.target_gimbal_yaw + yaw_offset
            if not math.isfinite(observed_yaw):
                self.yaw_tracker.reset()
                self.tracked_stamp_ns = None
                return

            stamp_ns = (
                self.target_stamp.sec * 1000000000
                + self.target_stamp.nanosec
            )
            if self.tracked_stamp_ns is not None:
                if stamp_ns <= self.tracked_stamp_ns:
                    return

            self.tracked_stamp_ns = stamp_ns
            if not self.yaw_tracker.update(observed_yaw, stamp_sec):
                return




        # 固定场地几何的重力补偿；保留原有视觉瞄准角度。
        # 假设相机与枪管近似平行，安装偏差后续标定。
        if (
            self.get_parameter("enable_ballistics").value
            or self.get_parameter("enable_prediction").value
        ):
            distance = float(
                self.get_parameter("horizontal_distance").value
            )
            height = float(
                self.get_parameter("height_difference").value
            )
            speed = float(self.get_parameter("bullet_speed").value)
            if self.get_parameter("use_bullet_speed_feedback").value:
                timeout = float(
                    self.get_parameter("bullet_speed_timeout_sec").value
                )
                if not math.isfinite(timeout) or timeout <= 0.0:
                    return
                if (
                    self.feedback_bullet_speed is None
                    or self.bullet_speed_received_at is None
                    or time.monotonic() - self.bullet_speed_received_at
                    > timeout
                ):
                    return
                speed = self.feedback_bullet_speed

            up_sign = float(self.get_parameter("pitch_up_sign").value)

            if up_sign not in (-1.0, 1.0):
                self.get_logger().error("pitch_up_sign must be +1 or -1")
                return

            try:
                launch_pitch = solve_ballistic_pitch(
                    distance, height, speed
                )
            except ValueError as exc:
                self.get_logger().error(f"Ballistic solution rejected: {exc}")
                return

            sight_pitch = math.degrees(math.atan2(height, distance))
            compensation = launch_pitch - sight_pitch
            if self.get_parameter("enable_ballistics").value:
                target_pitch += up_sign * compensation

            if self.get_parameter("enable_prediction").value:
                # 水平距离 / 水平弹速，沿用无空气阻力低弹道模型。
                horizontal_speed = speed * math.cos(
                    math.radians(launch_pitch)
                )
                if not math.isfinite(horizontal_speed) or horizontal_speed <= 0:
                    return
                flight_time = distance / horizontal_speed
                horizon = target_age + flight_time + lead
                if not math.isfinite(horizon) or not 0.0 <= horizon <= 0.5:
                    return
                predicted_yaw = self.yaw_tracker.predict(horizon)
                if predicted_yaw is None:
                    return
                target_yaw = observed_yaw + (
                    (predicted_yaw - observed_yaw + 180.0) % 360.0 - 180.0
                )

        if not all(math.isfinite(value) for value in (
            target_yaw, target_pitch
        )):
            return


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
