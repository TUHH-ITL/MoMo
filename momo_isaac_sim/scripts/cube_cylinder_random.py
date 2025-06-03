from pxr import UsdGeom, Gf, Sdf
import omni.usd
import asyncio
import random

stage = omni.usd.get_context().get_stage()

# === CONFIGURATION ===
NUM_CUBES = 10
NUM_CYLINDERS = 10
# =====================

# Rectangle bounds
main_bounds = {"x_min": -25, "x_max": 4.5, "y_min": -22, "y_max": 29.7}
exclusion_zones = [
    {"x_min": 3.1, "x_max": 4.5, "y_min": 8.3, "y_max": 26},
    {"x_min": -2.3, "x_max": 0.4, "y_min": 8.3, "y_max": 26},
    {"x_min": -6.7, "x_max": -4.4, "y_min": 8.3, "y_max": 26},
    {"x_min": -11.7, "x_max": -9.2, "y_min": 8.3, "y_max": 26},
    {"x_min": -17, "x_max": -14, "y_min": 8.3, "y_max": 26},
    {"x_min": -22, "x_max": -19, "y_min": 8.3, "y_max": 26},
    {"x_min": -25, "x_max": -24, "y_min": 8.3, "y_max": 26},
]

def is_in_exclusion_zones(x, y):
    for zone in exclusion_zones:
        if zone["x_min"] <= x <= zone["x_max"] and zone["y_min"] <= y <= zone["y_max"]:
            return True
    return False

def generate_valid_position():
    while True:
        x = random.uniform(main_bounds["x_min"], main_bounds["x_max"])
        y = random.uniform(main_bounds["y_min"], main_bounds["y_max"])
        if not is_in_exclusion_zones(x, y):
            return (x, y, 0)

def define_shape(stage, shape_type, path):
    if shape_type == "cube":
        return UsdGeom.Cube.Define(stage, path)
    elif shape_type == "cylinder":
        return UsdGeom.Cylinder.Define(stage, path)
    else:
        raise ValueError(f"Unsupported shape type: {shape_type}")

async def animate_shape(shape_type, prim_path_str):
    prim_path = Sdf.Path(prim_path_str)
    shape_prim = define_shape(stage, shape_type, prim_path)
    xform = UsdGeom.Xformable(shape_prim)

    # Moderate size
    if shape_type == "cube":
        xform.AddScaleOp().Set(Gf.Vec3f(1, 1, 0.8))
        offset_z = 0.3
    elif shape_type == "cylinder":
        shape_prim.GetRadiusAttr().Set(0.7)
        shape_prim.GetHeightAttr().Set(0.8)
        offset_z = 0.3
    else:
        offset_z = 0.0

    translate_op = xform.AddTranslateOp()
    current_pos = generate_valid_position()

    while True:
        target_pos = generate_valid_position()
        duration = random.uniform(10.0, 16.0)
        steps = 100
        dt = duration / steps

        for step in range(steps + 1):
            alpha = step / steps
            interp = Gf.Vec3f(
                current_pos[0] * (1 - alpha) + target_pos[0] * alpha,
                current_pos[1] * (1 - alpha) + target_pos[1] * alpha,
                (current_pos[2] + offset_z) * (1 - alpha) + (target_pos[2] + offset_z) * alpha,
            )
            translate_op.Set(interp)
            await asyncio.sleep(dt)

        current_pos = target_pos

# Launch cubes
for i in range(NUM_CUBES):
    prim_path_str = f"/World/Cube_{i}"
    asyncio.ensure_future(animate_shape("cube", prim_path_str))

# Launch cylinders
for i in range(NUM_CYLINDERS):
    prim_path_str = f"/World/Cylinder_{i}"
    asyncio.ensure_future(animate_shape("cylinder", prim_path_str))



