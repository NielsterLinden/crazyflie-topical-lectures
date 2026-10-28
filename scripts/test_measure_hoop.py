"""Offline checks for the supplied survey tool; no radio connection is made."""

import ast
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import measure_hoop


class SurveyToolTests(unittest.TestCase):
    def test_assigned_uri_is_required_and_help_is_radio_free(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch.object(sys, 'argv', ['measure_hoop.py']), patch('sys.stderr'):
                with self.assertRaises(SystemExit) as error:
                    measure_hoop.parse_args()
                self.assertEqual(error.exception.code, 2)
            with patch.object(sys, 'argv', ['measure_hoop.py', '--help']), patch('sys.stdout'):
                with self.assertRaises(SystemExit) as result:
                    measure_hoop.parse_args()
                self.assertEqual(result.exception.code, 0)
        uri = 'radio://0/80/2M/E7E7E7E701'
        with patch.dict(os.environ, {'CF_URI': uri}):
            with patch.object(sys, 'argv', ['measure_hoop.py']):
                self.assertEqual(measure_hoop.parse_args().uri, uri)

    def test_circle_json_retains_measurements_and_does_not_overwrite(self):
        points = np.array([[0, 1, 1], [0, 0, 2], [0, -1, 1], [0, 0, 0]])
        fit = measure_hoop.fit_circle_3d(points, np.array([1, 0, 0]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'survey.json'
            measure_hoop._write_result(path, 'gate-1', fit, list(points), False)
            result = json.loads(path.read_text())
            self.assertEqual(result['name'], 'gate-1')
            self.assertEqual(result['survey']['point_count'], 4)
            self.assertEqual(result['survey']['points_m'], points.tolist())
            np.testing.assert_allclose(result['normal'], [1, 0, 0])
            with self.assertRaises(FileExistsError):
                measure_hoop._write_result(path, 'gate-1', fit, list(points), False)

    def test_rectangle_json_preserves_shape_and_inscribed_aperture(self):
        points = np.array([[-2, -1, 0], [2, -1, 0], [2, 1, 0], [-2, 1, 0]])
        fit = measure_hoop.fit_rectangle_3d(points, np.array([0, 0, 1]))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'rectangle.json'
            measure_hoop._write_rectangle_result(path, 'gate-2', fit, list(points), False)
            result = json.loads(path.read_text())
            self.assertEqual(result['shape'], 'rectangle')
            self.assertAlmostEqual(result['radius_m'], 1)
            self.assertAlmostEqual(result['radius_m'], min(result['width_m'], result['height_m']) / 2)
            self.assertIn('width_axis', result)
            self.assertIn('height_axis', result)

    def test_script_has_no_arming_or_setpoint_path(self):
        tree = ast.parse(Path(measure_hoop.__file__).read_text())
        names = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        self.assertFalse(names.intersection({
            'commander', 'high_level_commander', 'send_arming_request',
            'send_setpoint', 'set_value'}))


if __name__ == '__main__':
    unittest.main()
