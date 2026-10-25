"""Synthetic geometry and CLI tests, no ROS/camera/weights; standard-library runner."""
from copy import deepcopy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import analyze_rotating_target_pnp as analysis


def synthetic_frames(frames=48, seed=7, outliers=True):
    rng = np.random.default_rng(seed)
    result = []
    for frame in range(frames):
        stamp = 123_000_000_000+frame*62_500_000
        message = {'header': {'stamp': {'sec': stamp//10**9, 'nanosec': stamp % 10**9},
                              'frame_id': 'source_camera'}, 'image_width': 1440,
                   'image_height': 1080, 'optical_frame_id': 'camera_optical_frame', 'locations': []}
        if frame % 9 != 0:
            # Four 90-degree-separated armor centers; all share a circle, independent of IDs.
            for plate in range(4):
                angle = frame*.037+plate*math.pi/2
                x, z = .35+.21*math.cos(angle), 4.7+.21*math.sin(angle)
                x, z = np.array([x, z])+rng.normal(0, .001, 2)
                bad = outliers and frame % 7 == 0 and plate == 1
                if bad:
                    z += .4
                message['locations'].append({
                    'input_index': int(rng.integers(0, 1000)), 'pose_valid': True,
                    'ambiguous': False, 'reason': 'ok_digit_unverified',
                    'pose': {'position': {'x': float(x), 'y': .02, 'z': float(z)}},
                    'detection': {'confidence': .4 if bad else .9},
                    'reprojection_error_px': 2.5 if bad else .1})
            rng.shuffle(message['locations'])
        # Invalid placeholders are common and must never enter the fit.
        message['locations'].append({'pose_valid': False, 'ambiguous': True,
                                     'reason': 'ambiguous_ippe',
                                     'pose': {'position': {'x': 0., 'y': 0., 'z': 0.}}})
        result.append(message)
    return result


class AnalysisTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'input.yaml'

    def load(self, frames):
        self.path.write_text(yaml.safe_dump_all(frames), encoding='utf-8')
        return analysis.read_log(self.path)

    def test_robust_fit_outliers_and_failed_zero_poses(self):
        observations, source = self.load(synthetic_frames())
        report, arrays = analysis.analyze(observations, trials=300)
        fit = report['robust_fit']
        self.assertAlmostEqual(fit['center_x_m'], .35, delta=.003)
        self.assertAlmostEqual(fit['center_z_m'], 4.7, delta=.003)
        self.assertAlmostEqual(fit['radius_m'], .21, delta=.002)
        self.assertGreater(report['radial_outlier_count'], 0)
        self.assertGreater(report['depth_outlier_count'], 0)
        self.assertEqual(source['counts']['pose_invalid_skipped'], 48)
        self.assertEqual(source['counts']['frames_without_usable_pose'], 6)
        self.assertTrue(np.all(arrays['points'][:, 1] > 4))
        self.assertGreater(report['all_data_fit']['signed_radial_residual_m']['rms'],
                           fit['signed_radial_residual_m']['rms'])
        self.assertGreater(report['relationships_to_absolute_radial_residual']['reprojection_error_px']['all']['pearson'], .8)
        self.assertLess(report['relationships_to_absolute_radial_residual']['yolo_confidence']['all']['pearson'], -.8)
        self.assertEqual(report['angular_velocity']['status'], 'not_estimated_uncertain')

    def test_initial_radius_is_not_ground_truth(self):
        observations, _ = self.load(synthetic_frames(outliers=False))
        report, _ = analysis.analyze(observations, initial_radius=.6, trials=150)
        self.assertAlmostEqual(report['robust_fit']['radius_m'], .21, delta=.002)

    def test_input_index_and_array_order_are_not_identity(self):
        frames = synthetic_frames(outliers=False)
        first, _ = self.load(frames)
        for frame in frames:
            frame['locations'].reverse()
            for item in frame['locations']:
                item['input_index'] = 0
        second, _ = self.load(frames)
        a, _ = analysis.analyze(first, trials=150)
        b, _ = analysis.analyze(second, trials=150)
        for key in ('center_x_m', 'center_z_m', 'radius_m'):
            self.assertAlmostEqual(a['robust_fit'][key], b['robust_fit'][key], places=7)

    def test_invalid_coordinates_flags_and_missing_features(self):
        frame = synthetic_frames(frames=2)[1]
        locations = frame['locations']
        locations[0]['pose']['position']['z'] = float('nan')
        locations[1]['pose']['position']['z'] = -1.
        locations[2]['ambiguous'] = True
        locations[3].pop('detection')
        locations[3]['reprojection_error_px'] = -1.
        observations, source = self.load([frame])
        self.assertEqual(len(observations), 1)
        self.assertIsNone(observations[0].confidence)
        self.assertIsNone(observations[0].reprojection_error_px)
        self.assertEqual(source['counts']['invalid_coordinates_skipped'], 2)
        self.assertEqual(source['counts']['ambiguous_or_missing_flag_skipped'], 1)

    def test_malformed_optional_pose_is_filtered_and_missing_quality_is_not_faked(self):
        frames = synthetic_frames(frames=2)
        frame = frames[1]
        frame['locations'][0]['pose'] = None
        frame['locations'][1]['detection'] = None
        observations, source = self.load([frame])
        self.assertEqual(len(observations), 3)
        self.assertEqual(source['counts']['invalid_coordinates_skipped'], 1)
        self.assertEqual(source['counts']['missing_confidence'], 1)

    def test_duplicate_frames_and_mixed_frames(self):
        frames = synthetic_frames(frames=3)
        observations, source = self.load(frames+[deepcopy(frames[1])])
        self.assertEqual(source['counts']['duplicate_documents_skipped'], 1)
        self.assertEqual(len(observations), 8)
        frames[1]['optical_frame_id'] = 'other_camera'
        with self.assertRaisesRegex(ValueError, 'Mixed'):
            self.load(frames)

    def test_degenerate_missing_and_insufficient_data(self):
        with self.assertRaises(ValueError):
            analysis.analyze([])
        observations = [analysis.Observation(1, i, i, .1*i, 0, 4.7, .1, .9) for i in range(10)]
        with self.assertRaisesRegex(ValueError, 'non-collinear'):
            analysis.analyze(observations)
        with self.assertRaises(ValueError):
            self.load([{'locations': []}])
        self.path.write_text('header: [broken YAML', encoding='utf-8')
        with self.assertRaises(yaml.YAMLError):
            analysis.read_log(self.path)

    def test_short_arc_is_flagged_uncertain(self):
        observations = [analysis.Observation(1, i, i, .35+.21*math.cos(t), 0,
                                             4.7+.21*math.sin(t), .1, .9)
                        for i, t in enumerate(np.linspace(-.15, .15, 40))]
        report, _ = analysis.analyze(observations, trials=150)
        self.assertEqual(report['robust_fit']['status'], 'uncertain')
        self.assertEqual(report['relationships_to_absolute_radial_residual']['yolo_confidence']['all']['pearson'], None)

    def test_cli_saves_standalone_figures_report_and_all_observations(self):
        observations, _ = self.load(synthetic_frames(frames=24))
        output = Path(self.directory.name)/'output'
        result = subprocess.run([sys.executable, str(Path(analysis.__file__)), '--input', str(self.path),
                                 '--initial-radius', '.21', '--output-dir', str(output), '--trials', '200'],
                                capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((output/'report.json').read_text())
        self.assertEqual(report['source']['counts']['usable_observations'], len(observations))
        self.assertEqual(len((output/'observations.csv').read_text().splitlines()), len(observations)+1)
        for name in ('analysis.png', 'depth.png'):
            self.assertGreater((output/name).stat().st_size, 1000)
        self.assertIn('not estimated', result.stdout)


if __name__ == '__main__':
    unittest.main()
