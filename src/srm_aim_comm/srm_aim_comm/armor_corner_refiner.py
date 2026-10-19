import cv2
import numpy as np


def refine_light(gray, bar):
    center = np.asarray(bar['center'], dtype=np.float64)
    length, width = float(bar['length']), float(bar['width'])
    if (
        gray.ndim != 2
        or not np.isfinite([
            *center, length, width, bar['tilt']
        ]).all()
        or length < 8 or width <= 0
    ):
        return None

    angle = np.radians(bar['tilt'])
    initial = np.array([np.sin(angle), np.cos(angle)])
    normal = np.array([initial[1], -initial[0]])

    bounds = np.array([
        center + a*initial + b*normal
        for a in (-0.85*length, 0.85*length)
        for b in (-max(4., 2*width), max(4., 2*width))
    ])
    x0, y0 = np.floor(bounds.min(axis=0)).astype(int)
    x1, y1 = np.ceil(bounds.max(axis=0)).astype(int) + 1
    h, w = gray.shape
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)

    if x1-x0 < 3 or y1-y0 < 3:
        return None

    roi = gray[y0:y1, x0:x1]
    if (
        float(np.percentile(roi, 98))
        - float(np.percentile(roi, 10)) < 15
    ):
        return None

    _, mask = cv2.threshold(
        roi, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        mask, connectivity=8
    )

    choices = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < 5:
            continue

        ys, xs = np.nonzero(labels == label)
        pixels = np.column_stack(
            (xs+x0, ys+y0)
        ).astype(np.float64)
        mean = pixels.mean(axis=0)
        delta = mean-center

        if (
            abs(delta @ initial) > 0.3*length
            or abs(delta @ normal) > max(3., width)
        ):
            continue

        # 协方差最大特征值对应的方向，就是PCA长轴。
        values, vectors = np.linalg.eigh(np.cov(pixels.T))
        if values[1] < 4*max(values[0], 0.1):
            continue

        axis = vectors[:, 1]
        if axis[1] < 0:
            axis = -axis
        if axis @ initial < np.cos(np.radians(15)):
            continue

        extent = np.ptp(pixels @ axis)
        if not 0.6*length <= extent <= 1.4*length:
            continue

        cost = (
            np.linalg.norm(delta)/length
            + abs(extent/length-1)
        )
        choices.append((cost, mean, axis))

    if not choices:
        return None

    _, mean, axis = min(choices, key=lambda item: item[0])
    normal = np.array([axis[1], -axis[0]])

    # 沿中心轴每约0.25像素采样，横向取多条采样线平均。
    s = np.linspace(
        -0.85*length, 0.85*length,
        int(np.ceil(6.8*length))+1
    )
    offsets = np.linspace(-0.2*width, 0.2*width, 5)
    coords = (
        mean + s[:, None, None]*axis
        + offsets[None, :, None]*normal
    )

    if (
        coords[:, :, 0].min() < 1
        or coords[:, :, 0].max() > w-2
        or coords[:, :, 1].min() < 1
        or coords[:, :, 1].max() > h-2
    ):
        return None

    sampled = cv2.remap(
        roi.astype(np.float32),
        (coords[:, :, 0]-x0).astype(np.float32),
        (coords[:, :, 1]-y0).astype(np.float32),
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE
    )
    profile = sampled.mean(axis=1)
    profile = cv2.GaussianBlur(
        profile.reshape(-1, 1), (1, 5), 0
    ).ravel()
    gradient = np.gradient(profile, s)

    original_top = (center-initial*length/2-mean) @ axis
    original_bottom = (center+initial*length/2-mean) @ axis

    endpoints = []
    for expected, sign in (
        (original_top, 1), (original_bottom, -1)
    ):
        indices = np.flatnonzero(
            np.abs(s-expected) <= max(3., 0.25*length)
        )
        if indices.size < 3:
            return None

        scores = sign*gradient
        index = indices[np.argmax(scores[indices])]
        if (
            index <= indices[0] or index >= indices[-1]
            or scores[index] < 4
        ):
            return None

        denominator = (
            scores[index-1]-2*scores[index]+scores[index+1]
        )
        delta = 0.0
        if abs(denominator) > 1e-9:
            delta = float(np.clip(
                0.5*(scores[index-1]-scores[index+1])/denominator,
                -0.5, 0.5
            ))
        endpoint = mean + (
            s[index]+delta*(s[1]-s[0])
        )*axis
        endpoints.append(endpoint)

    top, bottom = endpoints
    if not 0.7*length <= np.linalg.norm(bottom-top) <= 1.3*length:
        return None
    return top, bottom


def refine_pair(gray, pair):
    left = refine_light(gray, pair['left'])
    right = refine_light(gray, pair['right'])
    if left is None or right is None:
        return None
    return np.array([
        left[0], left[1], right[1], right[0]
    ], dtype=np.float64)
