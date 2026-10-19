import time
import threading
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import yaml


def detect(snapshot):
    frame, received, number = snapshot
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    scale = min(1.0, 800.0 / max(w, h))
    small = cv2.resize(gray, (round(w*scale), round(h*scale)))
    found, points = cv2.findChessboardCorners(
        small, (11, 8),
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    if found:
        points = points.reshape(-1, 1, 2)
        points[:, :, 0] = (points[:, :, 0]+0.5)*w/small.shape[1]-0.5
        points[:, :, 1] = (points[:, :, 1]+0.5)*h/small.shape[0]-0.5
        points = cv2.cornerSubPix(
            gray, points, (7, 7), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
             20, 0.01)
        )
        points = points.reshape(-1, 1, 2)
    return frame, received, number, found, points


def panel(frame, label, points=None):
    h, w = frame.shape[:2]
    scale = min(640/w, 480/h)
    small = cv2.resize(frame, (round(w*scale), round(h*scale)))
    if points is not None:
        scaled = points.copy()
        scaled[:, :, 0] = (scaled[:, :, 0]+0.5)*small.shape[1]/w-0.5
        scaled[:, :, 1] = (scaled[:, :, 1]+0.5)*small.shape[0]/h-0.5
        cv2.drawChessboardCorners(small, (11, 8), scaled, True)
    canvas = np.zeros((480, 640, 3), np.uint8)
    canvas[:small.shape[0], :small.shape[1]] = small
    cv2.rectangle(canvas, (0, 0), (640, 30), (0, 0, 0), -1)
    cv2.putText(canvas, label, (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
    return canvas


def calculate(samples, size, output):
    model = np.zeros((88, 3), np.float32)
    model[:, :2] = np.mgrid[0:11, 0:8].T.reshape(-1, 2)*0.03
    rms, k, d, rotations, translations = cv2.calibrateCamera(
        [model.copy() for _ in samples],
        samples, size, None, None
    )
    if not np.isfinite([rms, *k.flat, *d.flat]).all():
        raise ValueError('Invalid calibration result')

    errors = []
    for observed, r, t in zip(samples, rotations, translations):
        projected, _ = cv2.projectPoints(model, r, t, k, d)
        errors.append(float(np.sqrt(np.mean(np.sum(
            (observed-projected.reshape(-1, 1, 2))**2, axis=2
        )))))

    def matrix(array):
        return dict(
            rows=array.shape[0], cols=array.shape[1],
            data=array.flatten().tolist()
        )

    data = dict(
        image_width=size[0], image_height=size[1],
        camera_name='hik_cs016', distortion_model='plumb_bob',
        camera_matrix=matrix(k),
        distortion_coefficients=matrix(d.reshape(1, -1)),
        rectification_matrix=matrix(np.eye(3)),
        projection_matrix=matrix(np.column_stack((k, np.zeros(3))))
    )
    path = output/'camera.yaml'
    path.write_text(
        yaml.safe_dump(data, sort_keys=False), encoding='utf-8'
    )
    (output/'quality.yaml').write_text(
        yaml.safe_dump(dict(
            rms_px=float(rms), per_view_rms_px=errors,
            views=len(samples)
        ), sort_keys=False), encoding='utf-8'
    )
    return (
        f'RMS={rms:.4f}px; worst={max(errors):.4f}px; '
        f'file={path}'
    )


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    cv2.setNumThreads(1)
    rclpy.init()
    node = rclpy.create_node('hik_calibration_smooth')
    bridge = CvBridge()
    lock = threading.Lock()
    stop = threading.Event()
    latest = [None]
    counter = [0]

    def receive(msg):
        try:
            frame = bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
            with lock:
                counter[0] += 1
                latest[0] = (frame, time.monotonic(), counter[0])
        except Exception as exc:
            print('Image error:', exc)

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT
    )
    subscription = node.create_subscription(
        Image, '/srm_aim/image_raw', receive, qos
    )

    def spin():
        while not stop.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    receiver = threading.Thread(target=spin, daemon=True)
    receiver.start()
    pool = ThreadPoolExecutor(max_workers=2)
    future = computing = result = None
    samples, last_saved, size = [], None, None
    auto = False
    next_capture = next_detect = 0.0
    last_detect = None
    output = (
        Path.home()/'srm_aim_ws'/'calibration'
        /time.strftime('%Y%m%d_%H%M%S')
    )
    output.mkdir(parents=True, exist_ok=False)
    window = 'Calibration | S save | A auto | C calculate | Q quit'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    print('棋盘格：11列×8行内角点，方格30mm')
    print('S手动保存；A自动采集；C计算；Q退出')
    print('自动模式每4秒尝试一次，请保持棋盘格静止。')
    print('保存目录：', output)

    try:
        while rclpy.ok():
            now = time.monotonic()
            with lock:
                snapshot = latest[0]

            if future is not None and future.done():
                try:
                    result = future.result()
                except Exception as exc:
                    result = None
                    print('Detection error:', exc)
                future = None

            if computing is not None and computing.done():
                try:
                    print(computing.result())
                except Exception as exc:
                    print('Calibration error:', exc)
                computing = None

            canvas = np.zeros((480, 1280, 3), np.uint8)
            if snapshot is not None:
                frame, received, number = snapshot
                current_size = (frame.shape[1], frame.shape[0])
                if size is None:
                    size = current_size
                    print('分辨率：', size)
                if size != current_size:
                    raise RuntimeError('分辨率改变，请重新启动标定')

                if (future is None and computing is None
                        and now >= next_detect
                        and number != last_detect):
                    future = pool.submit(detect, snapshot)
                    last_detect = number
                    next_detect = now+0.2

                canvas[:, :640] = panel(
                    frame,
                    f'LIVE | samples={len(samples)} auto={auto}'
                )
                if now-received > 1:
                    cv2.putText(canvas, 'CAMERA STALE',
                                (20, 70), 0, 1, (0, 0, 255), 2)

            if result is not None:
                age = now-result[1]
                canvas[:, 640:] = panel(
                    result[0],
                    f'DETECT | found={result[3]} age={age:.1f}s',
                    result[4] if result[3] else None
                )

            if computing is not None:
                cv2.putText(canvas, 'CALCULATING...',
                            (20, 450), 0, 1, (0, 255, 255), 2)

            cv2.imshow(window, canvas)
            key = cv2.waitKey(20) & 255
            if key in (ord('q'), ord('Q'), 27):
                break

            if key in (ord('a'), ord('A')) and computing is None:
                auto = not auto
                next_capture = now+4
                print('自动采集：', auto)

            capture = key in (ord('s'), ord('S'))
            if auto and now >= next_capture:
                capture = True
                next_capture = now+4

            if capture and computing is None:
                if (result is None or not result[3]
                        or now-result[1] > 1.5):
                    print('未保存：需要近期检测到完整88个角点。')
                elif result[2] != last_saved:
                    points = result[4]
                    if samples and min(
                        float(np.sqrt(np.mean((points-p)**2)))
                        for p in samples
                    ) < 8:
                        print('姿态过于接近，请移动或倾斜棋盘格。')
                    else:
                        path = output/f'view_{len(samples)+1:03d}.png'
                        if not cv2.imwrite(str(path), result[0]):
                            raise OSError('图片保存失败')
                        samples.append(points.copy())
                        last_saved = result[2]
                        np.savez(
                            output/'observations.npz',
                            corners=np.array(samples),
                            image_size=size,
                            inner_size=[11, 8], square_m=0.03
                        )
                        print('已采集：', len(samples))

            if key in (ord('c'), ord('C')) and computing is None:
                auto = False
                if len(samples) < 20:
                    print('至少需要20张不同姿态，目前：', len(samples))
                else:
                    computing = pool.submit(
                        calculate, [p.copy() for p in samples],
                        size, output
                    )
    finally:
        stop.set()
        receiver.join()
        pool.shutdown(wait=False)
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
