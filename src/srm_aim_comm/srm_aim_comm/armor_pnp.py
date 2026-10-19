"""相机光学坐标系：X向右、Y向下、Z向前，位置单位米。"""
from pathlib import Path

import cv2
import numpy as np
import yaml


class ArmorPnP:
    def __init__(self, calibration_file, width_m=0.135, height_m=0.058):
        with Path(calibration_file).expanduser().open(encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if data['distortion_model'] not in (
            'plumb_bob', 'rational_polynomial'
        ):
            raise ValueError('Unsupported distortion model')

        self.size = (
            int(data['image_width']), int(data['image_height'])
        )
        self.k = np.array(
            data['camera_matrix']['data'], dtype=np.float64
        ).reshape(3, 3)
        self.d = np.array(
            data['distortion_coefficients']['data'], dtype=np.float64
        )

        if (
            not np.isfinite([
                *self.k.flat, *self.d.flat, width_m, height_m
            ]).all()
            or min(
                *self.size, width_m, height_m,
                self.k[0, 0], self.k[1, 1]
            ) <= 0
            or self.d.size not in (4, 5, 8, 12, 14)
            or not np.allclose(self.k[2], [0, 0, 1])
        ):
            raise ValueError('Invalid calibration or armor size')

        w, h = width_m / 2, height_m / 2
        # 顺序：左上、左下、右下、右上。
        self.model = np.array([
            [-w, -h, 0], [-w, h, 0],
            [w, h, 0], [w, -h, 0]
        ], dtype=np.float64)

    @staticmethod
    def corners_from_pair(pair):
        endpoints = []
        for name in ('left', 'right'):
            bar = pair[name]
            angle = np.radians(bar['tilt'])
            axis = np.array([np.sin(angle), np.cos(angle)])
            center = np.asarray(bar['center'], dtype=np.float64)
            half = axis * bar['length'] / 2
            endpoints.append((center - half, center + half))

        return np.array([
            endpoints[0][0], endpoints[0][1],
            endpoints[1][1], endpoints[1][0]
        ], dtype=np.float64)

    def solve(self, corners, image_size):
        points = np.ascontiguousarray(corners, dtype=np.float64)
        if (
            tuple(image_size) != self.size
            or points.shape != (4, 2)
            or not np.isfinite(points).all()
        ):
            return None

        if (
            np.any(points < 0)
            or np.any(points[:, 0] >= self.size[0])
            or np.any(points[:, 1] >= self.size[1])
        ):
            return None

        contour = points.astype(np.float32).reshape(-1, 1, 2)
        if (
            not cv2.isContourConvex(contour)
            or cv2.contourArea(contour, oriented=True) >= -1
        ):
            return None

        candidates = []
        try:
            result = cv2.solvePnPGeneric(
                self.model, points, self.k, self.d,
                flags=cv2.SOLVEPNP_IPPE
            )

            for rv, tv in zip(result[1], result[2]):
                if not np.isfinite(rv).all() or not np.isfinite(tv).all():
                    continue

                rv, tv = rv.reshape(3), tv.reshape(3)
                rotation, _ = cv2.Rodrigues(rv)
                depths = (self.model @ rotation.T + tv)[:, 2]

                if not 0.1 <= tv[2] <= 20.0 or np.any(depths <= 0):
                    continue

                projected, _ = cv2.projectPoints(
                    self.model, rv, tv, self.k, self.d
                )
                error = float(np.sqrt(np.mean(np.sum(
                    (projected.reshape(4, 2) - points)**2, axis=1
                ))))

                if not np.isfinite(error) or error > 3.0:
                    continue

                angle = np.linalg.norm(rv)
                quaternion = np.array([0., 0., 0., 1.])
                if angle > 1e-12:
                    quaternion[:3] = rv / angle * np.sin(angle / 2)
                    quaternion[3] = np.cos(angle / 2)

                candidates.append({
                    'position': tv,
                    'quaternion': quaternion,
                    'error_px': error,
                    'distance_m': float(np.linalg.norm(tv)),
                })

        except cv2.error:
            return None

        return min(
            candidates, key=lambda p: p['error_px'], default=None
        )
