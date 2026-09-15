"""Switchable DSD/DIP outputs with explicit measurement semantics."""
from dataclasses import dataclass
import numpy as np
from geometry import intrinsics, dip_to_points

# The 6.0.1 depth sensor post-process reports true_depth / near**2 instead of
# metres, and culls against min/max distance using that inflated value. Measured
# on RTX 3070 / 6.0.1: near 0.05 turns a 0.797 m panel into 318.8, past any sane
# max distance, so every pixel is culled and the output is uniformly zero.
# A small far plane distorts the same output non-linearly, so rendering uses a
# far plane well past the scene and cfg far_m stays a validity bound only.
RENDER_FAR_M = 1.0e6


@dataclass
class SensorFrame:
    mode: str
    depth_m: np.ndarray
    valid: np.ndarray
    rgb: np.ndarray
    points_native: np.ndarray
    metadata: dict


@dataclass
class LidarFrame:
    """Sparse returns. Deliberately not reshaped into an image."""
    points_m: np.ndarray        # (N, 3) world
    range_m: np.ndarray         # (N,) native range
    azimuth_deg: np.ndarray
    elevation_deg: np.ndarray
    intensity: np.ndarray
    material_id: np.ndarray
    object_id: np.ndarray
    echo_id: np.ndarray
    valid: np.ndarray
    metadata: dict


class DepthSensorModule:
    """Separate render products prevent DSD post-processing from configuring DIP.

    select_mode('dip'|'dsd') switches an already attached output. Future physics,
    acoustic and radar adapters should preserve their native (non-image) data.
    """
    def __init__(self, cfg, modes=("dip", "dsd"), asset_root=None):
        from isaacsim.sensors.experimental.rtx import CameraSensor, RtxCamera, SingleViewDepthCameraSensor
        from pxr import UsdGeom
        import omni.usd

        self.cfg = cfg
        self.resolution = tuple(cfg["camera"]["resolution_hw"])
        self.k = intrinsics(self.resolution, cfg["camera"]["horizontal_fov_deg"])
        self.dsd_scale = float(cfg["camera"]["near_m"]) ** 2
        self.backends = {}
        self.rgb_backends = {}
        self.camera_prims = {}
        self.depth_range = {}
        self.mount_roots = {}
        self.vendor_parameters = {}
        self.selected = modes[0]
        stage = omni.usd.get_context().get_stage()
        for mode in modes:
            if mode not in ("dip", "dsd", "realsense"):
                raise ValueError(f"Unsupported mode: {mode}")
            if mode == "realsense":
                camera, authored = self._build_vendor_camera(cfg, asset_root, stage)
            else:
                path = f"/World/SensorRig/{mode}/Camera"
                authored = RtxCamera(path)
                camera = UsdGeom.Camera(stage.GetPrimAtPath(path))
                camera.CreateFocalLengthAttr(20.0)
                camera.CreateHorizontalApertureAttr(20.0 * self.resolution[1] / self.k[0, 0])
                camera.CreateVerticalApertureAttr(20.0 * self.resolution[0] / self.k[1, 1])
            # The near plane drives the near**2 correction, so every camera shares it.
            camera.CreateClippingRangeAttr((cfg["camera"]["near_m"], RENDER_FAR_M))
            if mode == "dip":
                sensor = CameraSensor(authored, resolution=self.resolution,
                                      annotators=["rgb", "distance_to_image_plane"])
            else:
                sensor = SingleViewDepthCameraSensor(authored, resolution=self.resolution,
                    annotators=["depth_sensor_distance", "depth_sensor_point_cloud_position"])
                if mode == "realsense":
                    # The asset ships a template render product, already copied onto
                    # this one by SingleViewDepthCameraSensor. Keep those numbers.
                    self.vendor_parameters[mode] = self._depth_parameters(sensor)
                    low, high = sensor.get_sensor_distance_cutoffs()
                else:
                    dsd = cfg["dsd"]
                    sensor.set_sensor_baseline(dsd["baseline_mm"])
                    sensor.set_sensor_focal_length(float(self.k[0, 0]))
                    sensor.set_sensor_size(float(self.resolution[1]))
                    sensor.set_sensor_maximum_disparity(dsd["max_disparity_px"])
                    sensor.set_sensor_disparity_confidence(dsd["confidence_threshold"])
                    sensor.set_sensor_noise_parameters(dsd["noise_mean"], dsd["noise_sigma"])
                    sensor.set_sensor_disparity_noise_downscale(dsd["noise_downscale"])
                    sensor.set_enabled_outlier_removal(dsd["outlier_removal"])
                    low, high = dsd["min_distance_m"], dsd["max_distance_m"]
                # Cutoffs are compared against the inflated output, so express
                # the intended metre range in the same inflated units.
                self.depth_range[mode] = (float(low), float(high))
                sensor.set_sensor_distance_cutoffs(low / self.dsd_scale, high / self.dsd_scale)
                sensor.set_enabled_post_processing(True)
            self.backends[mode] = sensor
            # DSD excludes rgb. Use the same camera prim with a separate standard
            # render product so RGB does not inherit DSD post-processing.
            self.rgb_backends[mode] = sensor if mode == "dip" else CameraSensor(
                authored, resolution=self.resolution, annotators=["rgb"])
            self.camera_prims[mode] = camera
        # Keeps the DSD depth input at full resolution: with DLSS upscaling the
        # renderer feeds it a half-resolution depth texture. That mismatch only
        # costs depth-map resolution; it does not empty the output, which the
        # near**2 inflation above does.
        for sensor in set(self.backends.values()) | set(self.rgb_backends.values()):
            self._configure_native_resolution(sensor.render_product.GetPrim())

    def _build_vendor_camera(self, cfg, asset_root, stage):
        """Reference a vendor depth-camera asset and wrap its depth camera prim."""
        from isaacsim.sensors.experimental.rtx import RtxCamera
        from pxr import Usd, UsdGeom

        relative = cfg["realsense"]["asset"]
        url = relative if "://" in relative else str(asset_root).rstrip("/") + "/" + relative.lstrip("/")
        mount = "/World/SensorRig/realsense/Body"
        authored = RtxCamera.create(mount, usd_path=url)
        camera_path = authored.paths[0]
        expected = cfg["realsense"].get("camera_prim_name")
        if expected and not str(camera_path).endswith(expected):
            raise RuntimeError(f"Vendor asset resolved to {camera_path}, expected a prim named "
                               f"{expected}; the depth template targets a different camera")
        self.mount_roots["realsense"] = mount
        # The depth camera sits inside the body, so the mount has to be offset for
        # the optical centre to land on the requested pose.
        relative, _ = UsdGeom.XformCache(Usd.TimeCode.Default()).ComputeRelativeTransform(
            stage.GetPrimAtPath(camera_path), stage.GetPrimAtPath(mount))
        self.camera_in_mount = np.array(relative, dtype=float).T
        return UsdGeom.Camera(stage.GetPrimAtPath(camera_path)), authored

    @staticmethod
    def _depth_parameters(sensor):
        prim = sensor.render_product.GetPrim()
        return {attr.GetName().split(":")[-1]: attr.Get() for attr in prim.GetAttributes()
                if "depthSensor" in attr.GetName()}

    @staticmethod
    def _configure_native_resolution(prim):
        from pxr import Sdf
        prim.ApplyAPI("OmniRtxPostDebugSettingsAPI_1")
        prim.CreateAttribute("omni:rtx:post:aa:limitedOps", Sdf.ValueTypeNames.Bool).Set(False)
        prim.CreateAttribute("omni:rtx:post:aa:op", Sdf.ValueTypeNames.Token).Set("none")

    def render_settings(self):
        """Read authored/effective USD settings rather than just CLI intent."""
        result = {}
        for label, sensor in [(f"{m}_depth", s) for m, s in self.backends.items()] + [
                (f"{m}_rgb", s) for m, s in self.rgb_backends.items()]:
            prim = sensor.render_product.GetPrim()
            result[label] = {"path": str(prim.GetPath()), "attributes": {
                attr.GetName(): str(attr.Get()) for attr in prim.GetAttributes()
                if attr.GetName().startswith("omni:rtx:") or attr.GetName() == "resolution"}}
        return result

    def select_mode(self, mode):
        if mode not in self.backends:
            raise ValueError(f"Mode {mode} was not attached; attached={list(self.backends)}")
        self.selected = mode

    def set_pose(self, optical_to_world):
        from pxr import Gf, UsdGeom
        import omni.usd
        # USD cameras look down -Z with +Y up. Matrix4d uses row-vector convention.
        usd_to_optical = np.diag([1.0, -1.0, -1.0, 1.0])
        usd_to_world = optical_to_world @ usd_to_optical
        stage = omni.usd.get_context().get_stage()
        for mode, camera in self.camera_prims.items():
            if mode in self.mount_roots:
                # Move the whole sensor body so its depth camera lands on the pose.
                target = usd_to_world @ np.linalg.inv(self.camera_in_mount)
                prim = stage.GetPrimAtPath(self.mount_roots[mode])
            else:
                target = usd_to_world
                prim = camera.GetPrim()
            xform = UsdGeom.Xformable(prim)
            xform.ClearXformOpOrder()
            xform.AddTransformOp().Set(Gf.Matrix4d(target.T.tolist()))
        self.pose = np.array(optical_to_world, copy=True)

    @staticmethod
    def _array(sensor, annotator):
        data, info = sensor.get_data(annotator)
        if data is None:
            raise RuntimeError(f"No data yet: {annotator}; increase warmup or check renderer log")
        array = data.numpy() if hasattr(data, "numpy") else np.asarray(data)
        if array.size == 0:
            raise RuntimeError(f"Empty data: {annotator}")
        return np.array(array, copy=True), info

    def read(self, mode=None):
        mode = mode or self.selected
        sensor = self.backends[mode]
        annotator = "distance_to_image_plane" if mode == "dip" else "depth_sensor_distance"
        depth, info = self._array(sensor, annotator)
        depth = depth.reshape(self.resolution).astype(np.float32)
        rgb_sensor = self.rgb_backends[mode]
        rgb, rgb_info = self._array(rgb_sensor, "rgb")
        if mode == "dip":
            valid = np.isfinite(depth) & (depth > 0)
            valid &= depth < self.cfg["camera"]["far_m"]
            points = dip_to_points(depth, self.k)
            semantics = "axial_z_m; optical camera frame (+X right, +Y down, +Z forward)"
        else:
            # Undo the renderer's near**2 inflation; see RENDER_FAR_M above.
            depth = (depth * self.dsd_scale).astype(np.float32)
            low, high = self.depth_range[mode]
            valid = np.isfinite(depth) & (depth > 0)
            valid &= (depth >= low) & (depth < high)
            points, _ = self._array(sensor, "depth_sensor_point_cloud_position")
            points = (points * self.dsd_scale).astype(np.float32)
            semantics = ("DSD metres after dividing out the renderer's near**2 inflation; "
                         "axial/radial convention must be calibrated before comparison")
        return SensorFrame(mode, depth, valid, rgb, points, {
            "annotator": annotator, "depth_semantics": semantics,
            "points_frame": "optical camera" if mode == "dip" else "native left imager; validate axes and offset",
            "annotator_info": info,
            "depth_all_zero": bool(np.all(depth == 0)),
            "valid_pixel_count": int(valid.sum()),
            "rgb_annotator_info": rgb_info,
            "rgb_render_product": str(rgb_sensor.render_product),
            "rgb_source": "standard CameraSensor; same camera prim and optics as depth",
            "K": self.k.tolist(), "optical_to_world": self.pose.tolist(),
            "render_product": str(sensor.render_product),
            # Raw renderer output is recoverable as depth_m / dsd_depth_scale.
            "dsd_depth_scale": None if mode == "dip" else self.dsd_scale,
            "render_far_m": RENDER_FAR_M,
            "depth_range_m": self.depth_range.get(mode),
            "vendor_parameters": self.vendor_parameters.get(mode),
        })


def lidar_pose(optical_to_world, aim_point_m, mount_height_m=0.0):
    """Lidar frame: +X forward, +Y left, +Z up, aimed at aim_point_m.

    Mounted above the camera and tilted down, because at camera height the sensor
    sits millimetres above the bench and its lower rings rake along the surface at
    grazing incidence instead of sampling the objects standing on it.
    """
    position = np.asarray(optical_to_world)[:3, 3] + np.array([0.0, 0.0, mount_height_m])
    forward = np.asarray(aim_point_m, dtype=float) - position
    forward = forward / np.linalg.norm(forward)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    up /= np.linalg.norm(up)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((forward, left, up))
    transform[:3, 3] = position
    return transform


class LidarModule:
    """RTX Lidar returns, kept sparse and native.

    Unlike the cameras this is ray traced against the scene and its returns carry
    a non-visual material id, so glass and plastic are distinguishable here even
    though they are not in the depth annotators.
    """
    VALID_FLAG = 64          # ElementFlags.VALID
    PATH = "/World/SensorRig/lidar/Lidar"

    def __init__(self, cfg):
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor

        self.cfg = cfg
        settings = cfg.get("lidar", {})
        self.aux_output_level = settings.get("aux_output_level", "FULL")
        self.mount_height_m = float(settings.get("mount_height_m", 0.0))
        # farRangeM defaults to 200 m, which scans the whole room: 80% of returns
        # land on walls and floor and the table is a sliver of the cloud.
        self.far_range_m = float(settings.get("far_range_m", 2.0))
        self.lidar = Lidar(self.PATH, aux_output_level=self.aux_output_level,
                           attributes={"omni:sensor:Core:farRangeM": self.far_range_m})
        self.sensor = LidarSensor(self.lidar, annotators=["generic-model-output"])
        self.pose = np.eye(4)

    def set_pose(self, optical_to_world, aim_point_m):
        from pxr import Gf, UsdGeom
        import omni.usd

        self.pose = lidar_pose(optical_to_world, aim_point_m, self.mount_height_m)
        prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.PATH)
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(Gf.Matrix4d(self.pose.T.tolist()))

    @staticmethod
    def _field(gmo, name, count):
        array = np.asarray(getattr(gmo, name))
        return array[:count] if array.size >= count else np.zeros(count, dtype=array.dtype)

    def read(self):
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data

        data, info = self.sensor.get_data("generic-model-output")
        gmo = parse_generic_model_output_data(data)
        count = int(gmo.numElements)
        if not count:
            raise RuntimeError("Lidar returned no elements; increase warmup frames")
        # Elements are spherical: x=azimuth deg, y=elevation deg, z=range m.
        azimuth = self._field(gmo, "x", count).astype(np.float32)
        elevation = self._field(gmo, "y", count).astype(np.float32)
        ranges = self._field(gmo, "z", count).astype(np.float32)
        az, el = np.deg2rad(azimuth), np.deg2rad(elevation)
        local = np.stack((ranges * np.cos(el) * np.cos(az),
                          ranges * np.cos(el) * np.sin(az),
                          ranges * np.sin(el)), axis=-1)
        points = (local @ self.pose[:3, :3].T + self.pose[:3, 3]).astype(np.float32)
        flags = self._field(gmo, "flags", count)
        valid = ((flags & self.VALID_FLAG) != 0) & np.isfinite(ranges) & (ranges > 0)
        return LidarFrame(
            points_m=points, range_m=ranges, azimuth_deg=azimuth, elevation_deg=elevation,
            intensity=self._field(gmo, "scalar", count).astype(np.float32),
            material_id=self._field(gmo, "matId", count).astype(np.int32),
            object_id=self._field(gmo, "objId", count).astype(np.int64),
            echo_id=self._field(gmo, "echoId", count).astype(np.int32),
            valid=valid,
            metadata={
                "sensor": "rtx_lidar",
                "annotator": "generic-model-output",
                "elements": count,
                "aux_output_level": self.aux_output_level,
                "scan_complete": int(gmo.scanComplete),
                "timestamp_ns": int(gmo.timestampNs),
                "native_semantics": "azimuth deg, elevation deg, range m in the sensor frame",
                "points_frame": "world metres, converted from the native spherical returns",
                "material_id_semantics": "non-visual material base index; 11=plastic, 19=clear_glass",
                "mount_height_m": self.mount_height_m,
                "far_range_m": self.far_range_m,
                "coverage": "360 deg rotary clipped by far_range_m to keep the scan on the table",
                "sensor_to_world": self.pose.tolist(),
                "annotator_info": info,
                "valid_count": int(valid.sum()),
            })
