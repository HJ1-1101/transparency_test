#!/usr/bin/env python3
"""Isaac Sim 6.0.1 standalone capture. Run with the Isaac Python environment."""
import argparse
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime, timezone

HERE = Path(__file__).resolve().parent


def preflight():
    try:
        version = importlib.metadata.version("isaacsim")
    except importlib.metadata.PackageNotFoundError:
        version = None
    try:
        gpu = subprocess.run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=15)
        gpu_ok = gpu.returncode == 0
        gpu_detail = (gpu.stdout + gpu.stderr).strip()
    except (OSError, subprocess.TimeoutExpired) as exc:
        gpu_ok, gpu_detail = False, str(exc)
    report = {"python": sys.executable, "isaacsim_package": version,
              "version_ok": bool(version and version.startswith("6.0.1")),
              "gpu_query_ok": gpu_ok, "gpu_details": gpu_detail,
              "asset_loading_tested": False, "rendering_tested": False}
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return report


def validate_config(cfg):
    import math
    def positive(value, label):
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{label} must be finite and positive")
    known = {"opaque", "omniglass", "transparent", "omniglass_water"}
    if not cfg["material_order"] or set(cfg["material_order"]) - known:
        raise ValueError(f"material_order entries must come from {sorted(known)}")
    if len(set(cfg["material_order"])) != len(cfg["material_order"]):
        raise ValueError("material_order must not repeat a condition")
    for key in ("bench_origin_m", "room_translation_m"):
        if len(cfg[key]) != 3 or not all(math.isfinite(x) for x in cfg[key]):
            raise ValueError(f"{key} must have three finite coordinates")
    if not str(cfg["cup_asset"]).endswith(".usd"):
        raise ValueError("cup_asset must be a .usd path")
    for key in ("radius_m", "height_m"):
        positive(cfg["water"][key], f"water {key}")
    mount = cfg.get("lidar", {}).get("mount_height_m", 0.0)
    if not math.isfinite(mount) or mount < 0:
        raise ValueError("lidar mount_height_m must be finite and non-negative")
    far = cfg.get("lidar", {}).get("far_range_m", 2.0)
    positive(far, "lidar far_range_m")
    if far <= max(cfg["capture"]["distances_m"]):
        raise ValueError("lidar far_range_m must exceed the largest capture distance")
    for x in cfg["camera"]["resolution_hw"]:
        positive(x, "resolution")
        if type(x) is not int:
            raise ValueError("resolution must contain integers")
    if len(cfg["camera"]["resolution_hw"]) != 2:
        raise ValueError("resolution_hw must be (height, width)")
    if not 0 < cfg["camera"]["horizontal_fov_deg"] < 180:
        raise ValueError("Invalid FOV")
    if not 0 < cfg["camera"]["near_m"] < cfg["camera"]["far_m"]:
        raise ValueError("Invalid camera clipping range")
    if not 0 <= cfg["transparent_opacity"] <= 1:
        raise ValueError("Opacity must be in [0, 1]")
    if not 0 < cfg["dsd"]["min_distance_m"] < cfg["dsd"]["max_distance_m"]:
        raise ValueError("Invalid DSD range")
    for key in ("baseline_mm", "max_disparity_px", "noise_downscale"):
        positive(cfg["dsd"][key], key)
    for key in ("noise_mean", "noise_sigma", "confidence_threshold"):
        if not math.isfinite(cfg["dsd"][key]) or cfg["dsd"][key] < 0:
            raise ValueError(f"Invalid {key}")
    positive(cfg["slot_spacing_m"], "slot_spacing_m")
    positive(cfg["background_offset_m"], "background_offset_m")
    for key in ("warmup_frames", "samples_per_pose", "frames_between_samples"):
        value = cfg["capture"][key]
        if type(value) is not int or value < 1:
            raise ValueError(f"{key} must be a positive integer")
    if not cfg["capture"]["distances_m"] or not cfg["capture"]["angles_deg"]:
        raise ValueError("Capture pose lists must not be empty")
    for distance in cfg["capture"]["distances_m"]:
        positive(distance, "capture distance")
    for angle in cfg["capture"]["angles_deg"]:
        if not math.isfinite(angle) or abs(angle) >= 80:
            raise ValueError("Initial experiment restricts angles to (-80, 80) degrees")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=HERE / "config.json")
    parser.add_argument("--mode", choices=("dip", "dsd", "realsense", "both", "all"),
                        default="all", help="all = dip + dsd + realsense")
    parser.add_argument("--no-lidar", action="store_true", help="Skip the RTX Lidar scan")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--preflight", action="store_true")
    parser.add_argument("--validate-config", action="store_true")
    parser.add_argument("--smoke", action="store_true", help="One pose/sample")
    parser.add_argument("--stay-open", action="store_true", help="Keep GUI open after capture")
    parser.add_argument("--output", type=Path, help="A new, non-existing output directory")
    comments = parser.add_mutually_exclusive_group()
    comments.add_argument("--comment", help="Run comment (Markdown), saved to comment.md")
    comments.add_argument("--ask-comment", action="store_true",
                          help="Prompt for a run comment (the default)")
    args = parser.parse_args()
    cfg = json.loads(args.config.read_text())
    validate_config(cfg)
    if args.validate_config:
        print("Configuration valid")
        return 0
    report = preflight()
    if args.preflight:
        return 0 if report["version_ok"] and report["gpu_query_ok"] else 2
    if not report["version_ok"] or not report["gpu_query_ok"]:
        print("Capture not started. Check the Isaac Python environment and GPU availability.", file=sys.stderr)
        return 2
    if args.headless and args.stay_open:
        parser.error("--stay-open requires a GUI")
    comment = args.comment
    if comment is None:
        try:
            comment = input("Run comment (Enter to skip): ")
        except EOFError:
            comment = ""
    if args.smoke:
        cfg["capture"].update(distances_m=[0.8], angles_deg=[0.0], samples_per_pose=1)
    output = args.output or HERE / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    output.mkdir(parents=True, exist_ok=False)
    # Persist notes before simulator startup, including runs that later fail.
    if comment and comment.strip():
        (output / "comment.md").write_text(
            "# Run comment\n\n" + comment + "\n", encoding="utf-8")
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": args.headless, "multi_gpu": False,
                         "renderer": "RaytracedLighting", "anti_aliasing": 0})
    status = {"state": "started", "preflight": report}
    try:
        import numpy as np
        import carb
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from scene import build_scene
        from sensors import DepthSensorModule, LidarModule
        from geometry import camera_pose
        extensions = omni.kit.app.get_app().get_extension_manager()
        for extension in ("isaacsim.core.experimental.materials", "isaacsim.sensors.experimental.rtx"):
            extensions.set_extension_enabled_immediate(extension, True)
        carb.settings.get_settings().set("/rtx/post/aa/op", 0)
        carb.settings.get_settings().set("/rtx-transient/post/aa/limitedOps", False)
        omni.usd.get_context().new_stage()
        scene = build_scene(cfg)
        # Reload after non-visual material authoring, before sensor initialization.
        stage_path = output / "scene.usda"
        omni.usd.get_context().get_stage().GetRootLayer().Export(str(stage_path))
        if not omni.usd.get_context().open_stage(str(stage_path)):
            raise RuntimeError("Could not reload authored scene")
        for _ in range(5):
            app.update()
        modes = {"both": ("dip", "dsd"), "all": ("dip", "dsd", "realsense")}.get(
            args.mode, (args.mode,))
        module = DepthSensorModule(cfg, modes, asset_root=scene["asset_root"])
        lidar = None if args.no_lidar else LidarModule(cfg)
        if not args.headless:
            from isaacsim.core.rendering_manager import ViewportManager
            origin = np.asarray(cfg["bench_origin_m"])
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=origin + [-2.3, -2.0, 1.2], target=origin)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        def advance(count):
            for _ in range(count):
                if not app.is_running():
                    raise RuntimeError("Application closed before capture completed")
                app.update()

        # Inspect and persist the real camera matrices after square-pixel enforcement.
        camera_settings = {}
        for mode, camera in module.camera_prims.items():
            camera_settings[mode] = {a.GetName(): str(a.Get()) for a in camera.GetPrim().GetAttributes()}
        metadata = {"config": cfg, "scene": scene, "camera_attributes": camera_settings,
                    "renderer": "RaytracedLighting", "anti_aliasing": 0,
                    "render_products": module.render_settings(),
                    "notes": ["No real sensor calibration has been performed.",
                              "reference_depth_m is the DIP output, a sensor reading rather than "
                              "independent geometry; it carries DIP's own errors.",
                              "The water volume is an approximating cylinder, not the cup interior.",
                              "DSD is a single-view post-process, not a LiDAR measurement.",
                              "One scan covers all targets at once; every frame contains every condition.",
                              "Lidar returns are ray traced and carry a non-visual material id, so unlike "
                              "the depth annotators they can distinguish glass from plastic."]}
        # One scan of the whole table: aim at the centroid of the cups, not at one of them.
        centres = np.array([t["center_m"] for t in scene["targets"].values()], dtype=float)
        aim = centres.mean(axis=0)
        # Framing is set by the widest offset from the aim point, not by the span,
        # because the handles make the cup bounds asymmetric about their slots.
        lo = min(t["bbox_min_m"][1] for t in scene["targets"].values())
        hi = max(t["bbox_max_m"][1] for t in scene["targets"].values())
        half_width = max(abs(hi - aim[1]), abs(aim[1] - lo))
        fits_from_m = half_width / np.tan(np.deg2rad(cfg["camera"]["horizontal_fov_deg"]) / 2)
        metadata["scene_scan"] = {"aim_point_m": aim.tolist(), "targets": list(scene["targets"]),
                                  "target_span_m": float(hi - lo),
                                  "required_half_width_m": float(half_width),
                                  "all_targets_fit_beyond_m": float(fits_from_m)}
        (output / "run.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False))
        for distance in cfg["capture"]["distances_m"]:
            if distance < fits_from_m:
                print(f"WARNING: at {distance:.2f} m the row of cups ({hi - lo:.2f} m wide) is clipped "
                      f"by the camera field of view; all of it fits beyond {fits_from_m:.2f} m.",
                      file=sys.stderr, flush=True)
        index = 0
        empty_frames = []
        for distance in cfg["capture"]["distances_m"]:
            for angle in cfg["capture"]["angles_deg"]:
                pose = camera_pose(aim, distance, angle)
                module.set_pose(pose)
                if lidar is not None:
                    lidar.set_pose(pose, aim)
                advance(cfg["capture"]["warmup_frames"])
                for sample in range(cfg["capture"]["samples_per_pose"]):
                    advance(cfg["capture"]["frames_between_samples"])
                    # Every sensor is read after the same update, with no step in between.
                    # Keep the per-annotator metadata: this alone does not prove hardware synchronization.
                    frames = {}
                    for mode in modes:
                        module.select_mode(mode)
                        frames[mode] = module.read()
                    # Cup meshes have no analytic ray intersection, so DIP is the
                    # geometric reference: it returned the surface for every material
                    # in the panel runs. Without DIP attached there is no reference.
                    if "dip" in frames:
                        reference = np.where(frames["dip"].valid, frames["dip"].depth_m, np.nan)
                        reference_source = "dip distance_to_image_plane, same pose and update"
                    else:
                        reference = np.full(module.resolution, np.nan, dtype=np.float32)
                        reference_source = "unavailable; attach dip (--mode both) for a reference"
                    common = {"distance_to_aim_m": distance, "angle_deg": angle, "sample": sample,
                              "simulation_time_s": timeline.get_current_time(),
                              "targets_in_view": list(scene["targets"]),
                              "aim_point_m": aim.tolist()}
                    for mode, frame in frames.items():
                        stem = f"{index:05d}_scene_{mode}"
                        np.savez_compressed(output / f"{stem}.npz", depth_m=frame.depth_m,
                            valid=frame.valid, rgb=frame.rgb, points_native=frame.points_native,
                            reference_depth_m=reference)
                        item = {**common, "valid_fraction_full_image": float(frame.valid.mean()),
                                "reference_source": reference_source, **frame.metadata}
                        (output / f"{stem}.json").write_text(json.dumps(item, indent=2, default=str))
                        if not frame.valid.any():
                            empty_frames.append(stem)
                            print(f"WARNING: {stem} has no valid depth pixels; no usable point cloud. "
                                  "Check renderer warnings and sensor filtering before "
                                  "interpreting this as a material effect.",
                                  file=sys.stderr, flush=True)
                    if lidar is not None:
                        scan = lidar.read()
                        stem = f"{index:05d}_scene_lidar"
                        np.savez_compressed(output / f"{stem}.npz", points_m=scan.points_m,
                            range_m=scan.range_m, azimuth_deg=scan.azimuth_deg,
                            elevation_deg=scan.elevation_deg, intensity=scan.intensity,
                            material_id=scan.material_id, object_id=scan.object_id,
                            echo_id=scan.echo_id, valid=scan.valid)
                        item = {**common, "valid_fraction_returns": float(scan.valid.mean()),
                                **scan.metadata}
                        (output / f"{stem}.json").write_text(json.dumps(item, indent=2, default=str))
                        if not scan.valid.any():
                            empty_frames.append(stem)
                            print(f"WARNING: {stem} has no valid lidar returns.",
                                  file=sys.stderr, flush=True)
                    index += 1
                print(f"Captured scene: distance={distance:.3f} m, angle={angle:.1f} deg", flush=True)
        # Save all sensor/render-product settings for replay and GUI inspection.
        omni.usd.get_context().get_stage().GetRootLayer().Export(str(output / "scene_with_sensors.usda"))
        status.update(state="complete", pose_samples=index, modes=list(modes),
                      empty_depth_frames=empty_frames,
                      data_quality="needs_review" if empty_frames else "nonempty_depth")
        # --stay-open must not delay the completion/quality record until GUI exit.
        (output / "status.json").write_text(json.dumps(status, indent=2))
        print(f"Capture complete: {output}", flush=True)
        while args.stay_open and app.is_running():
            app.update()
    except Exception as exc:
        status.update(state="failed", error=repr(exc))
        raise
    finally:
        (output / "status.json").write_text(json.dumps(status, indent=2))
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
