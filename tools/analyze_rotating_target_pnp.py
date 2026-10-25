#!/usr/bin/env python3
"""Offline fixed-camera X-Z circle analysis; no ROS, hardware or tracking imports."""
import argparse
from collections import Counter
import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.stats import pearsonr, spearmanr
import yaml


@dataclass
class Observation:
    document: int
    slot: int  # Current array position only, never a physical armor identity.
    stamp_ns: int
    x: float
    y: float
    z: float
    reprojection_error_px: float | None
    confidence: float | None


def finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def read_log(path):
    """Stream YAML documents; reject mixed camera metadata, retain failure counts."""
    observations, counts, reasons = [], Counter(), Counter()
    source, seen = None, set()
    with Path(path).open(encoding='utf-8') as stream:
        for document, frame in enumerate(yaml.safe_load_all(stream), 1):
            if frame is None:
                continue
            counts['documents'] += 1
            if not isinstance(frame, dict) or not isinstance(frame.get('locations'), list):
                raise ValueError(f'Document {document}: expected complete ArmorLocationArray YAML')
            try:
                h = frame['header']
                sec, nsec = h['stamp']['sec'], h['stamp']['nanosec']
                if (type(sec) is not int or type(nsec) is not int or sec < 0
                        or not 0 <= nsec < 10**9):
                    raise ValueError('invalid timestamp')
                metadata = (frame['optical_frame_id'], h['frame_id'],
                            frame['image_width'], frame['image_height'])
                if (not all(isinstance(v, str) and v for v in metadata[:2])
                        or any(type(v) is not int or v <= 0 for v in metadata[2:])):
                    raise ValueError('missing frame identity or image dimensions')
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f'Document {document}: invalid source Header/metadata') from exc
            if source is None:
                source = metadata
            elif metadata != source:
                raise ValueError('Mixed optical/source frames or image dimensions; split logs first')
            stamp = sec*10**9+nsec
            if stamp in seen:
                counts['duplicate_documents_skipped'] += 1
                continue
            seen.add(stamp)
            counts['unique_frames'] += 1
            frame_used = 0
            for slot, item in enumerate(frame['locations']):
                counts['location_entries'] += 1
                if not isinstance(item, dict):
                    raise ValueError(f'Document {document}, slot {slot}: expected location mapping')
                # Never read failure pose coordinates, including the normal zero placeholders.
                if item.get('pose_valid') is not True:
                    counts['pose_invalid_skipped'] += 1
                    reasons[str(item.get('reason', 'unspecified'))] += 1
                    continue
                counts['pose_valid_entries'] += 1
                if item.get('ambiguous') is not False:
                    counts['ambiguous_or_missing_flag_skipped'] += 1
                    continue
                pose = item.get('pose')
                position = pose.get('position') if isinstance(pose, dict) else None
                position = position if isinstance(position, dict) else {}
                xyz = [finite_number(position.get(axis)) for axis in 'xyz']
                if any(value is None for value in xyz) or xyz[2] <= 0:
                    counts['invalid_coordinates_skipped'] += 1
                    continue
                error = finite_number(item.get('reprojection_error_px'))
                detection = item.get('detection')
                confidence = finite_number(detection.get('confidence')) if isinstance(detection, dict) else None
                if error is None or error < 0:
                    error = None
                    counts['missing_reprojection_error'] += 1
                if confidence is None or not 0 <= confidence <= 1:
                    confidence = None
                    counts['missing_confidence'] += 1
                observations.append(Observation(document, slot, stamp, *xyz, error, confidence))
                frame_used += 1
            if not frame_used:
                counts['frames_without_usable_pose'] += 1
            else:
                counts['frames_with_usable_pose'] += 1
                if frame_used > 1:
                    counts['frames_with_multiple_usable_poses'] += 1
    counts['usable_observations'] = len(observations)
    return observations, {
        'counts': dict(counts), 'invalid_pose_reasons': dict(reasons),
        'optical_frame_id': source[0] if source else None,
        'source_frame_id': source[1] if source else None,
        'image_size': list(source[2:]) if source else None,
        'weighting': 'equal weight per valid armor observation, not per image',
    }


def radial_residual(points, circle):
    return np.linalg.norm(points-np.asarray(circle[:2]), axis=1)-circle[2]


def algebraic_center(points):
    origin = points.mean(axis=0)
    q = points-origin
    a = np.column_stack((2*q, np.ones(len(q))))
    solution, _, rank, _ = np.linalg.lstsq(a, np.sum(q*q, axis=1), rcond=None)
    if rank < 3:
        raise ValueError('Circle is unidentifiable: fewer than three non-collinear positions')
    return origin+solution[:2]


def refine(points, center, initial_radius):
    result = least_squares(lambda c: radial_residual(points, c),
                           [*center, initial_radius], bounds=([-np.inf, -np.inf, 1e-6],
                                                           [np.inf, np.inf, np.inf]),
                           max_nfev=2000, xtol=1e-12, ftol=1e-12, gtol=1e-12)
    if not result.success or not np.isfinite(result.x).all():
        raise ValueError('Geometric circle optimization did not converge')
    return result.x


def triplet_circle(points):
    a, b, c = points
    u, v = b-a, c-a
    cross = u[0]*v[1]-u[1]*v[0]
    if abs(cross) <= 1e-4*np.linalg.norm(u)*np.linalg.norm(v) or abs(cross) < 1e-14:
        return None
    center = a+np.linalg.solve(2*np.array([u, v]), [u@u, v@v])
    radius = np.linalg.norm(center-a)
    if radius <= 1e-6 or not np.isfinite([*center, radius]).all():
        return None
    return np.array([*center, radius])


def robust_circle(points, initial_radius, threshold, trials, seed, min_inliers, min_fraction):
    rng = np.random.default_rng(seed)
    best_mask, best_score = None, (-1, -np.inf)
    for _ in range(trials):
        circle = triplet_circle(points[rng.choice(len(points), 3, replace=False)])
        if circle is None:
            continue
        residual = np.abs(radial_residual(points, circle))
        mask = residual <= threshold
        count = int(mask.sum())
        score = (count, -float(np.median(residual[mask]))) if count else (0, -np.inf)
        if score > best_score:
            best_mask, best_score = mask, score
    required = max(min_inliers, math.ceil(len(points)*min_fraction))
    if best_mask is None or int(best_mask.sum()) < required:
        raise ValueError(f'No robust consensus of at least {required} observations; inspect data/threshold')
    # Refine only the consensus; the supplied radius is an initial guess, never a constraint.
    mask = best_mask
    for _ in range(10):
        center = algebraic_center(points[mask])
        circle = refine(points[mask], center, initial_radius)
        next_mask = np.abs(radial_residual(points, circle)) <= threshold
        if int(next_mask.sum()) < required:
            raise ValueError('Robust consensus collapsed during refinement')
        if np.array_equal(mask, next_mask):
            return circle, mask
        mask = next_mask
    raise ValueError('Robust membership did not stabilize; result is uncertain')


def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {'count': 0}
    median = np.median(a)
    return {'count': len(a), 'min': float(a.min()), 'median': float(median),
            'mean': float(a.mean()), 'max': float(a.max()),
            'rms': float(np.sqrt(np.mean(a*a))),
            'mad': float(np.median(np.abs(a-median))),
            'p95_absolute': float(np.quantile(np.abs(a), .95))}


def fit_report(points, circle):
    angles = np.sort(np.mod(np.arctan2(points[:, 1]-circle[1], points[:, 0]-circle[0]), 2*np.pi))
    coverage = float(360-np.max(np.diff(np.r_[angles, angles[0]+2*np.pi]))*180/np.pi)
    offset = points-circle[:2]
    distances = np.linalg.norm(offset, axis=1)
    jac = np.column_stack((-offset/np.maximum(distances[:, None], 1e-12), -np.ones(len(points))))
    condition = float(np.linalg.cond(jac))
    warnings = []
    if coverage < 90:
        warnings.append('Small observed azimuth coverage: center/radius may be weakly constrained')
    if condition > 100:
        warnings.append('Ill-conditioned center/radius fit; no precision claim is supported')
    return {'center_x_m': float(circle[0]), 'center_z_m': float(circle[1]),
            'radius_m': float(circle[2]), 'signed_radial_residual_m': summary(radial_residual(points, circle)),
            'observed_azimuth_coverage_deg': coverage,
            'jacobian_condition_number': condition if math.isfinite(condition) else None,
            'status': 'uncertain' if warnings else 'fit_available_not_ground_truth', 'warnings': warnings}


def relationship(feature, residual):
    mask = np.isfinite(feature) & np.isfinite(residual)
    x, y = np.asarray(feature)[mask], np.asarray(residual)[mask]
    if len(x) < 5 or np.ptp(x) < 1e-12 or np.ptp(y) < 1e-12:
        return {'count': len(x), 'status': 'insufficient_or_constant_data',
                'pearson': None, 'spearman': None}
    return {'count': len(x), 'status': 'descriptive_only',
            'pearson': float(pearsonr(x, y)[0]), 'spearman': float(spearmanr(x, y)[0])}


def analyze(observations, initial_radius=.21, radial_threshold=.02, depth_threshold=.05,
            trials=1000, seed=42, min_inliers=6, min_fraction=.5):
    if (not math.isfinite(initial_radius) or initial_radius <= 1e-6
            or not math.isfinite(radial_threshold) or radial_threshold <= 0
            or not math.isfinite(depth_threshold) or depth_threshold <= 0
            or trials < 1 or min_inliers < 3 or not 0 < min_fraction <= 1):
        raise ValueError('Invalid radius/threshold/RANSAC configuration')
    if len(observations) < max(3, min_inliers):
        raise ValueError('Too few usable PnP observations for robust circle analysis')
    points = np.array([[o.x, o.z] for o in observations])
    # Full-data comparison deliberately includes every valid coordinate, including outliers.
    center = algebraic_center(points)
    try:
        all_circle = refine(points, center, initial_radius)
        all_report = fit_report(points, all_circle)
    except ValueError as exc:
        all_circle, all_report = None, {'status': 'fit_failed', 'reason': str(exc)}
    circle, inlier = robust_circle(points, initial_radius, radial_threshold, trials, seed,
                                   min_inliers, min_fraction)
    residual = radial_residual(points, circle)
    # Nearest of the two possible Z branches at this X; no armor identity is assumed.
    discriminant = circle[2]**2-(points[:, 0]-circle[0])**2
    available = discriminant >= 0
    depth_residual = np.full(len(points), np.nan)
    depth_residual[available] = np.abs(np.abs(points[available, 1]-circle[1])-np.sqrt(discriminant[available]))
    depth_outlier = available & (depth_residual > depth_threshold)
    outlier = ~inlier | depth_outlier
    errors = np.array([o.reprojection_error_px if o.reprojection_error_px is not None else np.nan for o in observations])
    confidence = np.array([o.confidence if o.confidence is not None else np.nan for o in observations])
    report = {
        'model': 'single horizontal X-Z circle, fixed camera; radius is not ground truth',
        'initial_radius_m': initial_radius, 'radial_threshold_m': radial_threshold,
        'depth_threshold_m': depth_threshold, 'ransac_trials': trials, 'seed': seed,
        'all_data_fit': all_report, 'robust_fit': fit_report(points[inlier], circle),
        'robust_residuals_on_all_observations_m': summary(residual),
        'radial_inlier_count': int(inlier.sum()), 'radial_outlier_count': int((~inlier).sum()),
        'combined_outlier_count': int(outlier.sum()),
        'observed_depth_z_m': summary(points[:, 1]),
        'observed_y_m': summary([o.y for o in observations]),
        'depth_z_for_radial_inliers_m': summary(points[inlier, 1]),
        'depth_z_for_radial_outliers_m': summary(points[~inlier, 1]),
        'depth_nearest_circle_branch_residual_m': summary(depth_residual),
        'depth_outlier_count': int(depth_outlier.sum()),
        'depth_branch_unavailable_count': int((~available).sum()),
        'depth_caveat': 'Model consistency, not true depth error; nearest branch and tangent sensitivity; unavailable outside fitted X extent',
        'relationships_to_absolute_radial_residual': {
            name: {'all': relationship(feature, np.abs(residual)),
                   'radial_inliers_only': relationship(feature[inlier], np.abs(residual[inlier]))}
            for name, feature in [('reprojection_error_px', errors), ('yolo_confidence', confidence)]},
        'angular_velocity': {'status': 'not_estimated_uncertain',
                             'reason': 'Four indistinguishable plates have 90-degree phase symmetry; no temporal identity/phase unwrapping is available'},
        'limitations': ['No physical plate IDs or input_index association used',
                       'Same-frame multi-target observations are statistically correlated',
                       'XZ circle assumes rotation axis aligned with optical Y; camera tilt can produce an ellipse rather than PnP outliers',
                       'Correlations are descriptive, not causal or independent accuracy validation',
                       'Azimuth coverage is geometric sample coverage, not swept rotation angle',
                       'Circle residuals do not validate camera calibration, physical radius or metric accuracy'],
    }
    arrays = dict(points=points, circle=circle, all_circle=all_circle, residual=residual,
                  radial_inlier=inlier, depth_residual=depth_residual, outlier=outlier,
                  error=errors, confidence=confidence)
    return report, arrays


def save_outputs(directory, observations, report, arrays):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    t0 = min(o.stamp_ns for o in observations)
    times = np.array([(o.stamp_ns-t0)*1e-9 for o in observations])
    report['time_reference'] = {'stamp_sec': t0//10**9, 'stamp_nanosec': t0 % 10**9,
                                'duration_sec': float(times.max())}
    with (directory/'report.json').open('w', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
    with (directory/'observations.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['document', 'array_slot_not_track_id', 'stamp_sec', 'stamp_nanosec',
                         'relative_time_sec', 'x_m', 'y_m', 'z_m', 'reprojection_error_px',
                         'yolo_confidence', 'radial_residual_m', 'radial_inlier',
                         'nearest_branch_depth_residual_m', 'combined_outlier'])
        for i, o in enumerate(observations):
            depth = arrays['depth_residual'][i]
            writer.writerow([o.document, o.slot, o.stamp_ns//10**9, o.stamp_ns % 10**9,
                             times[i], o.x, o.y, o.z, o.reprojection_error_px, o.confidence,
                             arrays['residual'][i], bool(arrays['radial_inlier'][i]),
                             float(depth) if np.isfinite(depth) else '', bool(arrays['outlier'][i])])
    fig, axes = plt.subplots(2, 2, figsize=(13, 10), constrained_layout=True)
    ax = axes[0, 0]
    points, circle, bad = arrays['points'], arrays['circle'], arrays['outlier']
    scatter = ax.scatter(points[~bad, 0], points[~bad, 1], c=times[~bad], cmap='viridis', s=16,
                         label='Within thresholds')
    if bad.any():
        ax.scatter(points[bad, 0], points[bad, 1], c=times[bad], cmap='viridis',
                   vmin=times.min(), vmax=max(times.max(), 1e-9), marker='x', s=35, label='Flagged observation')
    scatter.set_clim(times.min(), max(times.max(), 1e-9))
    fig.colorbar(scatter, ax=ax, label='Sample time from source t0 (s)')
    angle = np.linspace(0, 2*np.pi, 300)
    ax.plot(circle[0]+circle[2]*np.cos(angle), circle[1]+circle[2]*np.sin(angle), 'g-', label='Robust circle')
    ax.plot(circle[0], circle[1], 'g+', markersize=12)
    full = arrays['all_circle']
    if full is not None:
        ax.plot(full[0]+full[2]*np.cos(angle), full[1]+full[2]*np.sin(angle), '--', color='orange', label='All-data circle')
    # Keep a weak all-data fit from shrinking the actual observations to an unreadable speck.
    low, high = points.min(axis=0), points.max(axis=0)
    span = np.maximum(high-low, .05)
    ax.set_xlim(low[0]-.15*span[0], high[0]+.15*span[0])
    ax.set_ylim(low[1]-.15*span[1], high[1]+.15*span[1])
    for i in np.unique(np.linspace(0, len(observations)-1, min(8, len(observations)), dtype=int)):
        ax.annotate(f'{times[i]:.2f}s', points[i], fontsize=7, xytext=(3, 3), textcoords='offset points')
    ax.set(xlabel='Camera X (m)', ylabel='Camera Z (m)', title='XZ observations; no plate identity assumed')
    ax.set_aspect('equal', adjustable='box'); ax.legend(fontsize=7, loc='upper right')
    for mask, label, marker in [(~bad, 'Within thresholds', '.'), (bad, 'Flagged', 'x')]:
        axes[0, 1].scatter(times[mask], arrays['residual'][mask]*1000, s=12, marker=marker, label=label)
    threshold = report['radial_threshold_m']*1000
    axes[0, 1].axhline(threshold, color='r', linestyle=':'); axes[0, 1].axhline(-threshold, color='r', linestyle=':')
    axes[0, 1].set(xlabel='Source time from t0 (s)', ylabel='Signed radial residual (mm)')
    axes[0, 1].legend(fontsize=7)
    for ax, name, label in [(axes[1, 0], 'error', 'Reprojection RMS (px)'),
                             (axes[1, 1], 'confidence', 'YOLO confidence')]:
        for mask, marker, color in [(~bad, '.', 'tab:blue'), (bad, 'x', 'tab:red')]:
            use = mask & np.isfinite(arrays[name])
            ax.scatter(arrays[name][use], np.abs(arrays['residual'][use])*1000, marker=marker, color=color, s=16)
        ax.set(xlabel=label, ylabel='Absolute robust-circle residual (mm)')
    fig.suptitle('Fixed camera; descriptive model consistency, not measurement accuracy')
    fig.savefig(directory/'analysis.png', dpi=170)
    plt.close(fig)
    # Separate unconnected samples prevent implying identity across frames.
    fig, ax = plt.subplots(figsize=(10, 4), constrained_layout=True)
    ax.scatter(times[~bad], points[~bad, 1], s=12, label='Within thresholds')
    ax.scatter(times[bad], points[bad, 1], s=20, marker='x', label='Flagged')
    ax.axhline(circle[1], color='g', linestyle='--', label='Fitted center Z')
    ax.axhspan(circle[1]-circle[2], circle[1]+circle[2], color='g', alpha=.08, label='Fitted circle Z extent')
    ax.set(xlabel='Source time from t0 (s)', ylabel='Observed optical Z (m)', title='Depth observations; no temporal armor association')
    ax.legend(); fig.savefig(directory/'depth.png', dpi=170); plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--initial-radius', default=.21, type=float, help='Optimization initial guess in metres; not ground truth')
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--radial-threshold', default=.02, type=float, help='RANSAC radial inlier threshold in metres')
    parser.add_argument('--depth-threshold', default=.05, type=float, help='Nearest fitted Z-branch consistency threshold in metres')
    parser.add_argument('--trials', default=1000, type=int)
    parser.add_argument('--seed', default=42, type=int)
    parser.add_argument('--min-inliers', default=6, type=int)
    parser.add_argument('--min-inlier-fraction', default=.5, type=float)
    args = parser.parse_args(argv)
    try:
        observations, source = read_log(args.input)
        report, arrays = analyze(observations, args.initial_radius, args.radial_threshold,
                                 args.depth_threshold, args.trials, args.seed,
                                 args.min_inliers, args.min_inlier_fraction)
        report['source'] = source
        save_outputs(args.output_dir, observations, report, arrays)
    except (OSError, ValueError, yaml.YAMLError, np.linalg.LinAlgError) as exc:
        print(f'Analysis failed: {exc}', file=sys.stderr)
        return 2
    fit = report['robust_fit']
    print(f"Usable poses: {len(observations)}; radial outliers: {report['radial_outlier_count']}")
    print(f"Robust center X,Z: ({fit['center_x_m']:.6f}, {fit['center_z_m']:.6f}) m; radius: {fit['radius_m']:.6f} m")
    print(f"Fit status: {fit['status']}; angular velocity: not estimated (90-degree symmetry/association ambiguity)")
    print(f'Outputs: {args.output_dir.resolve()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
