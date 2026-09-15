"""Place the same cup asset in every slot; only the visual materials differ."""
import numpy as np


def build_scene(cfg):
    import omni.client
    import omni.usd
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.core.experimental.materials import OmniGlassMaterial, NonVisualMaterial
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    root = cfg.get("asset_root") or get_assets_root_path()
    if not root:
        raise RuntimeError("No Isaac asset root. Set asset_root in config.json.")

    def resolve(relative):
        url = relative if "://" in relative else root.rstrip("/") + "/" + relative.lstrip("/")
        result, _ = omni.client.stat(url)
        if result != omni.client.Result.OK:
            raise RuntimeError(f"Cannot access asset: {url} ({result})")
        return url

    url = resolve(cfg["environment"])
    cup_url = resolve(cfg["cup_asset"])
    room = UsdGeom.Xform.Define(stage, "/World/Room")
    if not room.GetPrim().GetReferences().AddReference(url):
        raise RuntimeError(f"Failed to reference {url}")
    room.AddTranslateOp().Set(Gf.Vec3d(*cfg["room_translation_m"]))
    # Keep the referenced room read-only; all experiment geometry lives in this layer.
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    light = UsdLux.DomeLight.Define(stage, "/World/LabLight")
    light.CreateIntensityAttr(600.0)

    def preview(name, opacity, color):
        mat = UsdShade.Material.Define(stage, f"/World/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/World/Looks/{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        shader.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        return mat

    materials = {
        "opaque": preview("Opaque", 1.0, (0.65, 0.65, 0.65)),
        # Alpha/opacity control, NOT a physical acrylic material.
        "transparent": preview("TransparentOpacity", cfg["transparent_opacity"], (0.65, 0.65, 0.65)),
    }
    glass = OmniGlassMaterial("/World/Looks/OmniGlass")
    shader = glass.shaders[0]
    for name, kind, value in [
        ("glass_ior", Sdf.ValueTypeNames.Float, cfg["glass_ior"]),
        ("frosting_roughness", Sdf.ValueTypeNames.Float, cfg["glass_roughness"]),
        ("thin_walled", Sdf.ValueTypeNames.Bool, False),
        ("glass_color", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1, 1, 1)),
    ]:
        shader.CreateInput(name, kind).Set(value)
    materials["omniglass"] = glass.materials[0]
    water = OmniGlassMaterial("/World/Looks/Water")
    water_shader = water.shaders[0]
    for name, kind, value in [
        ("glass_ior", Sdf.ValueTypeNames.Float, cfg["water"]["ior"]),
        ("frosting_roughness", Sdf.ValueTypeNames.Float, 0.0),
        ("thin_walled", Sdf.ValueTypeNames.Bool, False),
        ("glass_color", Sdf.ValueTypeNames.Color3f, Gf.Vec3f(1, 1, 1)),
    ]:
        water_shader.CreateInput(name, kind).Set(value)
    materials["water"] = water.materials[0]
    # The water condition is a glass cup; the water is separate geometry inside it.
    materials["omniglass_water"] = materials["omniglass"]
    # Non-visual labels are for future RTX acoustic/radar experiments, not camera DSD.
    for name in ("opaque", "transparent", "omniglass", "water"):
        path = str(materials[name].GetPath())
        NonVisualMaterial(path, bases="clear_glass" if name in ("omniglass", "water") else "plastic")

    def box(path, center, size, material):
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        geom.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
        return geom

    # bench_origin_m is the centre of the bench top surface; cups stand on it.
    origin = np.array(cfg["bench_origin_m"], dtype=float)
    box("/World/Bench", origin + [0, 0, -0.025], [0.7, 1.9, 0.05], materials["opaque"])

    def cup(path, base_centre, material):
        slot = UsdGeom.Xform.Define(stage, path)
        slot.AddTranslateOp().Set(Gf.Vec3d(*map(float, base_centre)))
        # The asset root carries its own xform ops, so it gets its own prim.
        asset = UsdGeom.Xform.Define(stage, path + "/Cup")
        if not asset.GetPrim().GetReferences().AddReference(cup_url):
            raise RuntimeError(f"Failed to reference {cup_url}")
        # The asset binds its own material on the mesh, so this must outrank it.
        UsdShade.MaterialBindingAPI.Apply(slot.GetPrim()).Bind(
            material, bindingStrength=UsdShade.Tokens.strongerThanDescendants)
        for prim in Usd.PrimRange(asset.GetPrim()):
            if prim.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(prim)
        return asset

    names = list(cfg["material_order"])
    offset = (len(names) - 1) / 2.0
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    targets = {}
    for slot, name in enumerate(names):
        base = origin + [0, (slot - offset) * cfg["slot_spacing_m"], 0]
        path = f"/World/Targets/{name}"
        geom = cup(path, base, materials[name])
        if name == "omniglass_water":
            water_cfg = cfg["water"]
            fill = UsdGeom.Cylinder.Define(stage, path + "/Water")
            fill.CreateAxisAttr(UsdGeom.Tokens.z)
            fill.CreateRadiusAttr(water_cfg["radius_m"])
            fill.CreateHeightAttr(water_cfg["height_m"])
            fill.AddTranslateOp().Set(Gf.Vec3d(
                0.0, 0.0, water_cfg["base_offset_m"] + water_cfg["height_m"] / 2))
            UsdShade.MaterialBindingAPI.Apply(fill.GetPrim()).Bind(materials["water"])
        extent = bounds.ComputeWorldBound(geom.GetPrim()).ComputeAlignedRange()
        size = np.array(extent.GetMax()) - np.array(extent.GetMin())
        # Aim at the middle of the cup body, not the base it stands on.
        centre = base + [0, 0, size[2] / 2]
        targets[name] = {"prim_path": path, "center_m": centre.tolist(),
                         "size_m": size.tolist(), "base_m": base.tolist(),
                         # The handle makes the bounds asymmetric about the slot.
                         "bbox_min_m": list(extent.GetMin()), "bbox_max_m": list(extent.GetMax())}
        if size[1] >= cfg["slot_spacing_m"]:
            raise ValueError(f"slot_spacing_m {cfg['slot_spacing_m']} is smaller than the "
                             f"cup footprint {size[1]:.3f} m; slots would overlap")

    # Known background with texture cues for the DSD confidence model.
    white = preview("BackgroundWhite", 1.0, (0.8, 0.8, 0.8))
    dark = preview("BackgroundDark", 1.0, (0.08, 0.08, 0.08))
    background_x = origin[0] + cfg["background_offset_m"]
    # Centred on the cups rather than on the bench surface they stand on.
    background_z = float(np.mean([t["center_m"][2] for t in targets.values()]))
    box("/World/Backboard", [background_x, origin[1], background_z], [0.02, 3.2, 1.2], white)
    for row in range(12):
        for col in range(32):
            if (row + col) % 2 == 0:
                box(f"/World/Pattern/tile_{row}_{col}",
                    [background_x - 0.012, origin[1] + (col - 15.5) * 0.1,
                     background_z + (row - 5.5) * 0.1], [0.002, 0.098, 0.098], dark)
    return {"environment_url": url, "targets": targets, "cup_asset_url": cup_url,
            "asset_root": root,
            "transparent_semantics": "UsdPreviewSurface opacity control; not physical acrylic",
            "water_semantics": "cylinder approximating the interior fill; not measured from the cup mesh"}
