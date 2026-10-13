import cv2
import numpy as np
import rclpy
from srm_aim_comm.armor_detector_node import ArmorDetectorNode

image_path = '/home/gyqros/srm_aim_ws/test_images/test_target.jpg'

frame = cv2.imread(image_path)
if frame is None:
    raise SystemExit(f'无法读取图片: {image_path}')

rclpy.init()
node = ArmorDetectorNode()

try:
    height, width = frame.shape[:2]
    b, g, r = cv2.split(frame)

    red_mask = (
        (r > 150) & (r > g * 1.5) & (r > b * 1.5)
    ).astype(np.uint8) * 255

    blue_mask = (
        (b > 150) & (b > g * 1.5) & (b > r * 1.5)
    ).astype(np.uint8) * 255

    red_bars = node.find_light_bars(red_mask)
    blue_bars = node.find_light_bars(blue_mask)
    red_pairs = node.match_light_bars(red_bars)
    blue_pairs = node.match_light_bars(blue_bars)
    target = node.select_target(red_pairs, blue_pairs, width, height)

    display = frame.copy()
    cv2.drawMarker(
        display, (width // 2, height // 2),
        (255, 255, 255), cv2.MARKER_CROSS, 24, 1
    )

    for bars, color in [
        (red_bars, (0, 0, 255)),
        (blue_bars, (255, 0, 0))
    ]:
        for bar in bars:
            x, y, w, h = bar['x'], bar['y'], bar['w'], bar['h']
            cv2.rectangle(display, (x, y), (x+w, y+h), color, 2)

    for pairs, color in [
        (red_pairs, (0, 0, 255)),
        (blue_pairs, (255, 0, 0))
    ]:
        for pair in pairs:
            left, right = pair['left'], pair['right']
            x1 = min(left['x'], right['x'])
            y1 = min(left['y'], right['y'])
            x2 = max(left['x']+left['w'], right['x']+right['w'])
            y2 = max(left['y']+left['h'], right['y']+right['h'])
            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)

    print(f'图片尺寸: {width}x{height}')
    print(f'红色灯条: {len(red_bars)}, 红色配对: {len(red_pairs)}')
    print(f'蓝色灯条: {len(blue_bars)}, 蓝色配对: {len(blue_pairs)}')

    if target:
        print(
            f"目标: {target['color']}, "
            f"x={target['center_x']:.1f}, "
            f"y={target['center_y']:.1f}"
        )
        cv2.drawMarker(
            display,
            (int(target['center_x']), int(target['center_y'])),
            (0, 255, 0), cv2.MARKER_TILTED_CROSS, 30, 3
        )
    else:
        print('未检测到有效目标')

    cv2.imshow('Offline Armor Detection', display)
    cv2.imshow('Color Masks', cv2.bitwise_or(red_mask, blue_mask))
    print('按任意键关闭窗口')
    cv2.waitKey(0)
    cv2.destroyAllWindows()

finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
