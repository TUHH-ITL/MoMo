from pxr import UsdGeom, Gf, Sdf
import omni.usd
import asyncio

stage = omni.usd.get_context().get_stage()

# Define (shape_type, start_pos, end_pos)
shape_specs = [
    ("cube", (-2, -2, 0), (2, 2, 0)),
    ("sphere", (-4, -4, 0), (-1, -1, 0)),
    ("capsule", (1, 1, 0), (3, 0, 0)),
    ("cone", (-3, -5, 0), (-1, -2, 0)),
    ("cylinder", (-6, -2, 0), (-3, 1, 0)),
    ("cube", (1, -2, 0), (0, 2, 0)),
]

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

async def animate_shape(shape_type, start_pos, end_pos, prim_path_str):
    prim_path = Sdf.Path(prim_path_str)
    shape_prim = define_shape(stage, shape_type, prim_path)
    xform = UsdGeom.Xformable(shape_prim)

    # Set shape size and ground it (Z offset)
    if shape_type == "cube":
        xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 2.0))
        offset_z = 1.0
    elif shape_type in {"sphere", "cone", "cylinder", "capsule"}:
        shape_prim.GetRadiusAttr().Set(1.0)
        if hasattr(shape_prim, "GetHeightAttr"):
            shape_prim.GetHeightAttr().Set(2.0)
        offset_z = 1.0
    else:
        offset_z = 0.0

    translate_op = xform.AddTranslateOp()

    # Movement loop: back and forth forever
    duration = 3.0
    steps = 60
    dt = duration / steps

    while True:
        # Forward: A -> B
        for step in range(steps + 1):
            alpha = step / steps
            interp = Gf.Vec3f(
                start_pos[0] * (1 - alpha) + end_pos[0] * alpha,
                start_pos[1] * (1 - alpha) + end_pos[1] * alpha,
                (start_pos[2] + offset_z) * (1 - alpha) + (end_pos[2] + offset_z) * alpha,
            )
            translate_op.Set(interp)
            await asyncio.sleep(dt)

        # Backward: B -> A
        for step in range(steps + 1):
            alpha = step / steps
            interp = Gf.Vec3f(
                end_pos[0] * (1 - alpha) + start_pos[0] * alpha,
                end_pos[1] * (1 - alpha) + start_pos[1] * alpha,
                (end_pos[2] + offset_z) * (1 - alpha) + (start_pos[2] + offset_z) * alpha,
            )
            translate_op.Set(interp)
            await asyncio.sleep(dt)

# Start all shapes in parallel
for i, (shape_type, start_pos, end_pos) in enumerate(shape_specs):
    prim_path_str = f"/World/Shape_{i}_{shape_type}"
    asyncio.ensure_future(animate_shape(shape_type, start_pos, end_pos, prim_path_str))

