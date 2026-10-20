# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""The video audit must count physical crossings, not nearby waypoints."""
import importlib.util
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
spec = importlib.util.spec_from_file_location(
    'sentry_tour', Path(__file__).resolve().parents[1] / 'render-sentry-navigation-video.py')
tour = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tour)


def observe(monitor, y, folded=1., z=.20, x=4.7):
    monitor.observe({'pose': {'translation_m': [x, y, z]}, 'fold_angle_rad': folded}, 128)


class CrossingTests(unittest.TestCase):
    def test_real_under_roof_crossing_in_either_direction(self):
        for ys in [[4.5, 5.2, 5.9], [5.9, 5.2, 4.5]]:
            monitor = tour.Crossings()
            for y in ys:
                observe(monitor, y)
            self.assertIn('Road', monitor.completed)

    def test_above_roof_open_robot_and_missing_entry_do_not_count(self):
        for ys, folded, height in [([4.5, 5.2, 5.9], 0., .20),
                                   ([4.5, 5.2, 5.9], 1., .65),
                                   ([5.2, 5.9], 1., .20)]:
            monitor = tour.Crossings()
            for y in ys:
                observe(monitor, y, folded, height)
            self.assertNotIn('Road', monitor.completed)

    def test_reversing_or_leaving_the_lane_cancels_the_attempt(self):
        monitor = tour.Crossings()
        for y in [4.5, 5.2, 4.5, 5.9]:
            observe(monitor, y)
        self.assertNotIn('Road', monitor.completed)
        monitor = tour.Crossings()
        observe(monitor, 4.5)
        observe(monitor, 5.2)
        observe(monitor, 5.4, x=6.)
        observe(monitor, 5.9)
        self.assertNotIn('Road', monitor.completed)


if __name__ == '__main__':
    unittest.main()
