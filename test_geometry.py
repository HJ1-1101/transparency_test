"""Checks which catch silent depth/frame mistakes without requiring RTX hardware."""
import unittest
import numpy as np
from geometry import camera_pose, intrinsics, target_box_depth, dip_to_points


class GeometryTests(unittest.TestCase):
    def test_camera_faces_target_and_is_right_handed(self):
        for angle in (-60, 0, 20, 60):
            target = np.array([0, 0.55, 1.35])
            t = camera_pose(target, 0.8, angle)
            np.testing.assert_allclose(t[:3, 3] + 0.8 * t[:3, 2], target, atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(t[:3, :3]), 1.0)
            np.testing.assert_allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-12)

    def test_box_front_not_center_distance(self):
        k = intrinsics((3, 3), 60)
        t = camera_pose([0, 0, 1], 1, 0)
        z = target_box_depth((3, 3), k, t, [0, 0, 1], [.006, .4, .4])
        self.assertAlmostEqual(float(z[1, 1]), .997, places=6)
        self.assertTrue(np.isnan(z[0, 0]))

    def test_oblique_center_distance_and_radial_difference(self):
        k = intrinsics((5, 5), 60)
        t = camera_pose([0, 0, 1], 1, 30)
        z = target_box_depth((5, 5), k, t, [0, 0, 1], [.006, .5, .5])
        self.assertAlmostEqual(float(z[2, 2]), 1 - .003 / np.cos(np.deg2rad(30)), places=6)
        points = dip_to_points(np.ones((5, 5)), k)
        self.assertAlmostEqual(float(points[0, 0, 2]), 1)
        self.assertGreater(np.linalg.norm(points[0, 0]), 1)

    def test_world_translation_does_not_change_reference(self):
        k = intrinsics((12, 16), 60)
        offset = np.array([3, -2, 5])
        t = camera_pose([0, 0, 1], 1, 15)
        moved = t.copy()
        moved[:3, 3] += offset
        a = target_box_depth((12, 16), k, t, [0, 0, 1], [.006, .5, .5])
        b = target_box_depth((12, 16), k, moved, np.array([0, 0, 1]) + offset, [.006, .5, .5])
        np.testing.assert_allclose(a, b, equal_nan=True)

    def test_object_behind_camera_is_not_a_hit(self):
        k = intrinsics((3, 3), 60)
        t = camera_pose([0, 0, 1], 1, 0)
        z = target_box_depth((3, 3), k, t, [-2, 0, 1], [.01, .4, .4])
        self.assertTrue(np.isnan(z).all())


if __name__ == "__main__":
    unittest.main()
