from pxr import UsdGeom, Gf, Sdf
import omni.usd
import asyncio

stage = omni.usd.get_context().get_stage()

# Define available shape types
def define_shape(stage, shape_type, path):
    if shape_type == "cube":
        return UsdGeom.Cube.Define(stage, path)
    elif shape_type == "sphere":
        return UsdGeom.Sphere.Define(stage, path)
    elif shape_type == "capsule":
        return UsdGeom.Capsule.Define(stage, path)
    elif shape_type == "cone":
        return UsdGeom.Cone.Define(stage, path)
    elif shape_type == "cylinder":
        return UsdGeom.Cylinder.Define(stage, path)
    else:
        raise ValueError(f"Unsupported shape type: {shape_type}")

# Animate a shape back and forth between two points
async def animate_shape(shape_type, start_pos, end_pos, prim_path_str, duration):
    prim_path = Sdf.Path(prim_path_str)
    shape_prim = define_shape(stage, shape_type, prim_path)
    xform = UsdGeom.Xformable(shape_prim)

    # Setup scale and offsets
    if shape_type == "cube":
        xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 2.0))
        offset_z = 1.0
    elif shape_type == "sphere":
        shape_prim.GetRadiusAttr().Set(1.0)
        offset_z = 1.0
    elif shape_type == "capsule":
        shape_prim.GetRadiusAttr().Set(1.0)
        shape_prim.GetHeightAttr().Set(2.0)
        offset_z = 1.0
    elif shape_type == "cone":
        shape_prim.GetRadiusAttr().Set(1.0)
        shape_prim.GetHeightAttr().Set(2.0)
        offset_z = 1.0
    elif shape_type == "cylinder":
        shape_prim.GetRadiusAttr().Set(1.0)
        shape_prim.GetHeightAttr().Set(2.0)
        offset_z = 1.0
    else:
        offset_z = 0.0

    translate_op = xform.AddTranslateOp()

    steps = 60
    dt = duration / steps

    while True:
        # A → B
        for step in range(steps + 1):
            alpha = step / steps
            interp = Gf.Vec3f(
                start_pos[0] * (1 - alpha) + end_pos[0] * alpha,
                start_pos[1] * (1 - alpha) + end_pos[1] * alpha,
                (start_pos[2] + offset_z) * (1 - alpha) + (end_pos[2] + offset_z) * alpha,
            )
            translate_op.Set(interp)
            await asyncio.sleep(dt)

        # B → A
        for step in range(steps + 1):
            alpha = step / steps
            interp = Gf.Vec3f(
                end_pos[0] * (1 - alpha) + start_pos[0] * alpha,
                end_pos[1] * (1 - alpha) + start_pos[1] * alpha,
                (end_pos[2] + offset_z) * (1 - alpha) + (start_pos[2] + offset_z) * alpha,
            )
            translate_op.Set(interp)
            await asyncio.sleep(dt)

# Launch shapes with different paths and speeds
asyncio.ensure_future(animate_shape("cube", (-4, -4, 0), (-16, -4, 0), "/World/Shape_0_cube", duration=10.0))
asyncio.ensure_future(animate_shape("sphere", (-4, 4, 0), (4, -12, 0), "/World/Shape_1_sphere", duration=15.0))
asyncio.ensure_future(animate_shape("capsule", (-8, -10, 0), (-8, -4, 0), "/World/Shape_2_capsule", duration=14.0))
asyncio.ensure_future(animate_shape("cone", (-10, -10, 0), (-2, -2, 0), "/World/Shape_3_cone", duration=6.0))
asyncio.ensure_future(animate_shape("cylinder", (-14, -14, 0), (-2, -14, 0), "/World/Shape_4_cylinder", duration=12.5))

