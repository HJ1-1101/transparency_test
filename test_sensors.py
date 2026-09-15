"""Exercise RGB/depth routing without starting Isaac Sim."""
import unittest
import numpy as np

from sensors import DepthSensorModule


class FakeSensor:
    def __init__(self, path, data):
        self.render_product = path
        self.data = data

    def get_data(self, annotator):
        # Reject requests not configured on this render product.
        return self.data[annotator], {"source": self.render_product}


class SensorReadTests(unittest.TestCase):
    def test_dsd_reads_rgb_from_standard_camera_and_preserves_native_points(self):
        module = DepthSensorModule.__new__(DepthSensorModule)
        module.selected = "dsd"
        module.resolution = (2, 3)
        module.k = np.eye(3)
        module.pose = np.eye(4)
        module.cfg = {"dsd": {"min_distance_m": 0.4, "max_distance_m": 10.0}}
        module.depth_range = {"dsd": (0.4, 10.0)}
        module.vendor_parameters = {}
        module.intrinsics = {"dsd": np.eye(3)}
        module.pixel_aligned = {"dsd": True}
        module.dsd_scale = 1.0
        depth = np.array([[1, 0, np.nan], [0.3, 10, 2]], dtype=np.float32)
        points = np.arange(18, dtype=np.float32).reshape(2, 3, 3)
        rgb = np.full((2, 3, 3), 123, dtype=np.uint8)
        module.backends = {"dsd": FakeSensor("/depth", {
            "depth_sensor_distance": depth,
            "depth_sensor_point_cloud_position": points,
        })}
        module.rgb_backends = {"dsd": FakeSensor("/rgb", {"rgb": rgb})}

        frame = module.read()

        np.testing.assert_array_equal(frame.depth_m, depth)
        np.testing.assert_array_equal(frame.points_native, points)
        np.testing.assert_array_equal(frame.rgb, rgb)
        np.testing.assert_array_equal(frame.valid, [[True, False, False], [False, False, True]])
        self.assertEqual(frame.metadata["render_product"], "/depth")
        self.assertEqual(frame.metadata["rgb_render_product"], "/rgb")
        self.assertEqual(frame.metadata["rgb_annotator_info"], {"source": "/rgb"})

    def test_dsd_output_is_rescaled_out_of_the_renderer_near_squared_inflation(self):
        """near=0.05 inflates depth 400x; unscaled, every pixel falls outside the range."""
        module = DepthSensorModule.__new__(DepthSensorModule)
        module.selected = "dsd"
        module.resolution = (1, 3)
        module.k = np.eye(3)
        module.pose = np.eye(4)
        module.cfg = {"dsd": {"min_distance_m": 0.4, "max_distance_m": 10.0}}
        module.depth_range = {"dsd": (0.4, 10.0)}
        module.vendor_parameters = {}
        module.intrinsics = {"dsd": np.eye(3)}
        module.pixel_aligned = {"dsd": True}
        module.dsd_scale = 0.05**2
        true_m = np.array([[0.6, 0.8, 1.2]], dtype=np.float32)
        raw = true_m / module.dsd_scale
        module.backends = {"dsd": FakeSensor("/depth", {
            "depth_sensor_distance": raw,
            "depth_sensor_point_cloud_position": np.repeat(raw[..., None], 3, axis=-1),
        })}
        module.rgb_backends = {"dsd": FakeSensor("/rgb", {"rgb": np.zeros((1, 3, 3), np.uint8)})}

        frame = module.read()

        np.testing.assert_allclose(frame.depth_m, true_m, rtol=1e-6)
        np.testing.assert_allclose(frame.points_native[..., 0], true_m, rtol=1e-6)
        np.testing.assert_array_equal(frame.valid, [[True, True, True]])
        self.assertEqual(frame.metadata["dsd_depth_scale"], module.dsd_scale)


if __name__ == "__main__":
    unittest.main()
