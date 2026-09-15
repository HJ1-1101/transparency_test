"""Pure geometry. Optical frame: +X right, +Y down, +Z forward."""
import numpy as np


def camera_pose(target, distance, angle_deg):
    if distance <= 0 or not np.isfinite(distance):
        raise ValueError("distance must be finite and positive")
    angle = np.deg2rad(angle_deg)
    target = np.asarray(target, dtype=float)
    eye = target + [-distance * np.cos(angle), distance * np.sin(angle), 0]
    forward = (target - eye) / distance
    right = np.cross(forward, [0, 0, 1])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    transform = np.eye(4)
    transform[:3, :3] = np.column_stack((right, down, forward))
    transform[:3, 3] = eye
    return transform


def intrinsics(resolution_hw, hfov_deg):
    height, width = resolution_hw
    if height <= 0 or width <= 0 or not 0 < hfov_deg < 180:
        raise ValueError("Invalid resolution or field of view")
    focal = width / (2 * np.tan(np.deg2rad(hfov_deg) / 2))
    return np.array([[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]])


def pixel_rays(resolution_hw, k):
    height, width = resolution_hw
    v, u = np.mgrid[:height, :width]
    return np.stack(((u + 0.5 - k[0, 2]) / k[0, 0],
                     (v + 0.5 - k[1, 2]) / k[1, 1], np.ones_like(u)), axis=-1)


def target_box_depth(resolution_hw, k, optical_to_world, center, size):
    """First box intersection in axial Z metres; NOT an environment occlusion test."""
    rays = pixel_rays(resolution_hw, k) @ optical_to_world[:3, :3].T
    origin = optical_to_world[:3, 3]
    low = np.asarray(center) - np.asarray(size) / 2
    high = np.asarray(center) + np.asarray(size) / 2
    parallel = np.abs(rays) < 1e-12
    divisor = np.where(parallel, 1.0, rays)
    t0, t1 = (low - origin) / divisor, (high - origin) / divisor
    entry = np.where(parallel, -np.inf, np.minimum(t0, t1)).max(axis=-1)
    leave = np.where(parallel, np.inf, np.maximum(t0, t1)).min(axis=-1)
    outside_parallel = (parallel & ((origin < low) | (origin > high))).any(axis=-1)
    t = np.where(entry > 0, entry, leave)
    hit = (~outside_parallel) & (leave >= np.maximum(entry, 0)) & (t > 0)
    return np.where(hit, t, np.nan).astype(np.float32)


def dip_to_points(depth_z, k):
    """Only for axial depth such as DIP; never silently apply this to DSD."""
    return (pixel_rays(depth_z.shape, k) * depth_z[..., None]).astype(np.float32)
