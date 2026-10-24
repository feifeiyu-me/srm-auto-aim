"""Bounded exact-frame matching and CPU-only drawing for the YOLO visualizer."""

from collections import OrderedDict, deque

import cv2
import numpy as np


def frame_key(header):
    """Use the full source stamp and frame identity, never an approximate time."""
    if not 0 <= header.stamp.nanosec < 1000000000:
        raise ValueError('Invalid Header nanosec')
    return header.stamp.sec, header.stamp.nanosec, header.frame_id


class ExactFrameCache:
    """Match either arrival order, with receipt-time TTL and strict memory limits."""

    def __init__(self, image_limit=20, detection_limit=20,
                 byte_limit=96 * 1024 * 1024, ttl=1.0):
        if min(image_limit, detection_limit, byte_limit) <= 0 or not np.isfinite(ttl) or ttl <= 0:
            raise ValueError('Cache limits and TTL must be finite and positive')
        self.image_limit = image_limit
        self.detection_limit = detection_limit
        self.byte_limit = byte_limit
        self.ttl = ttl
        self.images = OrderedDict()
        self.detections = OrderedDict()
        self.blocked = OrderedDict()
        self.image_bytes = 0
        self.expired = 0
        self.evicted = 0

    def _remove_image(self, key):
        _, image = self.images.pop(key)
        self.image_bytes -= len(image.data)
        return image

    def prune(self, now):
        """Use monotonic receipt time so historical source stamps remain valid."""
        for key, (arrived, _) in list(self.images.items()):
            if now - arrived >= self.ttl:
                self._remove_image(key)
                self.expired += 1
        for cache in (self.detections, self.blocked):
            for key, (arrived, _) in list(cache.items()):
                if now - arrived >= self.ttl:
                    cache.pop(key)
                    self.expired += 1

    def _block(self, key, now):
        self.blocked[key] = (now, None)
        while len(self.blocked) > self.image_limit + self.detection_limit:
            self.blocked.popitem(last=False)

    def add_image(self, image, now):
        self.prune(now)
        key = frame_key(image.header)
        size = len(image.data)
        if image.width <= 0 or image.height <= 0 or size == 0 or size > self.byte_limit:
            return None
        if key in self.blocked:
            return None
        if key in self.images:
            # Two images with the same identity are ambiguous: never pick one.
            self._remove_image(key)
            self.detections.pop(key, None)
            self._block(key, now)
            return None
        self.images[key] = (now, image)
        self.image_bytes += size
        while len(self.images) > self.image_limit or self.image_bytes > self.byte_limit:
            self._remove_image(next(iter(self.images)))
            self.evicted += 1
        return self._match(key, now)

    def add_detection(self, detection, now):
        self.prune(now)
        key = frame_key(detection.header)
        if key in self.blocked:
            return None
        self.detections.pop(key, None)
        self.detections[key] = (now, detection)
        while len(self.detections) > self.detection_limit:
            self.detections.popitem(last=False)
            self.evicted += 1
        return self._match(key, now)

    def _match(self, key, now):
        if key not in self.images or key not in self.detections:
            return None
        image = self._remove_image(key)
        _, detection = self.detections.pop(key)
        self._block(key, now)
        if (image.width, image.height) != (detection.image_width, detection.image_height):
            return None
        return image, detection


class DetectionRate:
    """Measure detection-message arrival frequency, including unmatched results."""

    def __init__(self):
        self.arrivals = deque(maxlen=120)

    def add(self, now):
        self.arrivals.append(now)

    def fps(self, now):
        while self.arrivals and now - self.arrivals[0] > 2.0:
            self.arrivals.popleft()
        if len(self.arrivals) < 2 or self.arrivals[-1] <= self.arrivals[0]:
            return 0.0
        return (len(self.arrivals) - 1) / (self.arrivals[-1] - self.arrivals[0])


def _text(frame, text, position, color=(255, 255, 255)):
    cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, text, position, cv2.FONT_HERSHEY_SIMPLEX,
                0.6, color, 1, cv2.LINE_AA)


def draw_frame(image, detection, fps, age, stale=False):
    """Draw onto a copy of the matched image; stale views never retain overlays."""
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Expected a BGR8 image')
    height, width = image.shape[:2]
    if (width, height) != (detection.image_width, detection.image_height):
        raise ValueError('Detection/image dimensions differ')
    frame = image.copy()
    drawn = 0
    invalid = 0
    if not stale:
        for armor in detection.detections:
            points = np.array([[getattr(armor, name).x, getattr(armor, name).y]
                               for name in ('lt', 'lb', 'rb', 'rt')], dtype=np.float32)
            if (not np.isfinite(points).all() or np.any(points < 0)
                    or np.any(points[:, 0] >= width) or np.any(points[:, 1] >= height)
                    or not np.isfinite(armor.confidence) or not 0 <= armor.confidence <= 1
                    or not cv2.isContourConvex(points) or abs(cv2.contourArea(points)) < 1):
                invalid += 1
                continue
            pixels = np.clip(np.rint(points), [0, 0], [width - 1, height - 1]).astype(np.int32)
            cv2.polylines(frame, [pixels], True, (0, 255, 0), 2, cv2.LINE_AA)
            for name, (x, y) in zip(('LT', 'LB', 'RB', 'RT'), pixels):
                cv2.circle(frame, (int(x), int(y)), 4, (0, 255, 255), -1)
                _text(frame, name, (min(int(x) + 6, max(0, width - 40)),
                                    max(18, int(y) - 6)), (0, 255, 255))
            color = {armor.COLOR_BLUE: 'Blue', armor.COLOR_RED: 'Red'}.get(
                armor.color, 'Unknown')
            x, y = pixels[0]
            _text(frame, f'{color} {armor.confidence:.3f}',
                  (max(0, min(int(x), width - 180)), max(75, int(y) - 25)), (0, 255, 0))
            drawn += 1
    _text(frame, f'Det FPS (rx): {fps:.1f}  Targets: {drawn}  Invalid: {invalid}', (12, 25))
    stamp = detection.header.stamp
    _text(frame, f'Image: {stamp.sec}.{stamp.nanosec:09d}  '
          f'frame_id={detection.header.frame_id}', (12, 50))
    if stale:
        _text(frame, f'STALE: no fresh matched result ({age:.2f}s). No overlay.',
              (12, 78), (0, 165, 255))
    elif not detection.detections:
        _text(frame, 'No Target', (12, 78), (0, 255, 0))
    elif not drawn:
        _text(frame, 'No valid targets', (12, 78), (0, 165, 255))
    return frame
