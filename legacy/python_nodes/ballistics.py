"""忽略空气阻力的低弹道解算，距离单位 m，角度单位度。"""

import math


def solve_ballistic_pitch(horizontal_distance, height_difference,
                          bullet_speed, gravity=9.80665):
    """返回低弹道发射仰角，向上为正。

    horizontal_distance: 枪口到目标的水平距离。
    height_difference: 目标相对枪口的高度，向上为正。
    bullet_speed: 弹丸初速度，m/s。
    无效输入或目标不可达时抛出 ValueError。
    """
    distance, height, speed, gravity = (
        float(value) for value in (
            horizontal_distance, height_difference, bullet_speed, gravity
        )
    )

    if not all(math.isfinite(value) for value in (
        distance, height, speed, gravity
    )):
        raise ValueError("Ballistic inputs must be finite")

    if distance <= 0.0 or speed <= 0.0 or gravity <= 0.0:
        raise ValueError("Distance, speed and gravity must be positive")

    speed_squared = speed * speed
    discriminant = (
        speed_squared * speed_squared
        - gravity * (
            gravity * distance * distance
            + 2.0 * height * speed_squared
        )
    )

    if not math.isfinite(discriminant):
        raise ValueError("Ballistic inputs exceed numerical range")
    if discriminant < 0.0:
        raise ValueError("Target is unreachable at this bullet speed")

    # 低弹道根的有理化形式，避免相近数相减造成精度损失。
    numerator = gravity * distance * distance + 2.0 * height * speed_squared
    denominator = distance * (
        speed_squared + math.sqrt(discriminant)
    )

    if not all(math.isfinite(value) for value in (
        numerator, denominator
    )):
        raise ValueError("Ballistic solution exceeds numerical range")

    return math.degrees(math.atan2(numerator, denominator))
