import math
import struct


# ============================================================
# 协议常量
# ============================================================

UPPER_BODY_LEN = 16
LOWER_BODY_LEN = 28

ID_GIMBAL_RECEIVE = 1
ID_SHOOT_RECEIVE = 2

ID_GIMBAL_SEND = 1
ID_SHOOT_SEND = 2


# ============================================================
# 上位机 -> 下位机：18 字节
# ============================================================

def pack_upper_frame(
    yaw: float,
    pitch: float,
    fire_flag: int
) -> bytes:
    """生成上位机 -> 下位机的完整 18 字节协议帧。"""

    if not math.isfinite(yaw) or not math.isfinite(pitch):
        raise ValueError("yaw 和 pitch 必须是有限数值")

    if not isinstance(fire_flag, int):
        raise TypeError("fire_flag 必须是整数")

    gimbal_data = struct.pack(
        "<hff",
        ID_GIMBAL_RECEIVE,
        yaw,
        pitch
    )

    shoot_data = struct.pack(
        "<hi",
        ID_SHOOT_RECEIVE,
        fire_flag
    )

    body = gimbal_data + shoot_data

    return struct.pack("<H", len(body)) + body


def unpack_upper_frame(frame: bytes):
    """解析完整的 18 字节上行协议帧。"""

    if len(frame) != 18:
        raise ValueError(
            f"帧长度错误: expected 18, got {len(frame)}"
        )

    body_len = struct.unpack("<H", frame[:2])[0]

    if body_len != UPPER_BODY_LEN:
        raise ValueError(f"body_len 错误: {body_len}")

    gimbal_id, yaw, pitch = struct.unpack(
        "<hff",
        frame[2:12]
    )

    if gimbal_id != ID_GIMBAL_RECEIVE:
        raise ValueError(f"ID1 错误: {gimbal_id}")

    shoot_id, fire_flag = struct.unpack(
        "<hi",
        frame[12:18]
    )

    if shoot_id != ID_SHOOT_RECEIVE:
        raise ValueError(f"ID2 错误: {shoot_id}")

    return yaw, pitch, fire_flag


# ============================================================
# 下位机 -> 上位机：30 字节
# ============================================================

def unpack_lower_frame(frame: bytes):
    """解析完整的 30 字节下行协议帧。"""

    if len(frame) != 30:
        raise ValueError(
            f"帧长度错误: expected 30, got {len(frame)}"
        )

    body_len = struct.unpack("<H", frame[:2])[0]

    if body_len != LOWER_BODY_LEN:
        raise ValueError(f"body_len 错误: {body_len}")

    (
        gimbal_id,
        yaw,
        pitch,
        roll,
        mode,
        color
    ) = struct.unpack("<hfffii", frame[2:24])

    shoot_id, bullet_speed = struct.unpack(
        "<hf",
        frame[24:30]
    )

    if gimbal_id != ID_GIMBAL_SEND:
        raise ValueError(f"ID1 错误: {gimbal_id}")

    if shoot_id != ID_SHOOT_SEND:
        raise ValueError(f"ID2 错误: {shoot_id}")

    values = (yaw, pitch, roll, bullet_speed)

    if not all(math.isfinite(value) for value in values):
        raise ValueError("返回帧包含非有限浮点数")

    return {
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "mode": mode,
        "color": color,
        "bullet_speed": bullet_speed
    }


# ============================================================
# 下位机 -> 上位机：流式接收解析器
# ============================================================

class LowerFrameParser:
    """
    下位机返回帧流式解析器。

    支持：
    1. 一帧拆成多次接收
    2. 一次接收多帧
    3. 一次接收半帧或多帧加半帧

    注意：
    协议没有帧头和 CRC。
    遇到非法长度或非法 ID 时，清空缓存并报错。
    """

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        """
        输入本次收到的字节。

        返回：
            本次提取出的完整帧列表。
        """

        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError("data 必须是 bytes 类型数据")

        self.buffer.extend(data)

        frames = []

        while len(self.buffer) >= 2:

            body_len = struct.unpack(
                "<H",
                self.buffer[:2]
            )[0]

            if body_len != LOWER_BODY_LEN:
                self.buffer.clear()

                raise ValueError(
                    f"非法下行帧长度字段: {body_len}，"
                    "已清空接收缓存"
                )

            frame_len = 2 + body_len

            # 数据尚未收完整，保留缓存，等待下一次接收。
            if len(self.buffer) < frame_len:
                break

            frame = bytes(self.buffer[:frame_len])

            try:
                unpack_lower_frame(frame)
            except (ValueError, struct.error):
                self.buffer.clear()
                raise

            frames.append(frame)

            # 删除已处理的帧，保留后续数据。
            del self.buffer[:frame_len]

        return frames
