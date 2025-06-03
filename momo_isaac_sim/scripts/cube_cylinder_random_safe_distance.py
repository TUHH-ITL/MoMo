from pxr import UsdGeom, Gf, Sdf
import omni.usd
import asyncio
import random
import math

stage = omni.usd.get_context().get_stage()

# === CONFIGURATION ===
NUM_CUBES = 20
NUM_CYLINDERS = 20
MIN_CENTER_DIST = 0.7  # Minimum distance between obstacle surface and robot center
ROBOT_PRIM_PATH = "/World/robot"
# =====================

main_bounds = {"x_min": -25, "x_max": 4.5, "y_min": -22, "y_max": 29.7}

def get_robot_position():
    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if not robot_prim.IsValid():
        raise RuntimeError(f"[ERROR] Robot prim not found at {ROBOT_PRIM_PATH}")

    xform = UsdGeom.Xformable(robot_prim)
    transforms = xform.GetOrderedXformOps()
    for op in transforms:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            return op.Get()
    return Gf.Vec3f(0, 0, 0)

def generate_valid_position():
    x = random.uniform(main_bounds["x_min"], main_bounds["x_max"])
    y = random.uniform(main_bounds["y_min"], main_bounds["y_max"])
    return (x, y, 0)

def is_path_safe(start, end, obstacle_radius):
    robot_pos = get_robot_position()
    steps = 100
    for i in range(steps + 1):
        alpha = i / steps
        interp_x = start[0] * (1 - alpha) + end[0] * alpha
        interp_y = start[1] * (1 - alpha) + end[1] * alpha
        dx = interp_x - robot_pos[0]
        dy = interp_y - robot_pos[1]
        dist = math.hypot(dx, dy)
        if dist < (MIN_CENTER_DIST + obstacle_radius):
            return False
    return True

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

    if shape_type == "cube":
        size = 0.6
        xform.AddScaleOp().Set(Gf.Vec3f(size, size, 0.8))
        obstacle_radius = size / 2
        offset_z = 0.3
    elif shape_type == "cylinder":
        radius = 0.7
        height = 0.8
        shape_prim.GetRadiusAttr().Set(radius)
        shape_prim.GetHeightAttr().Set(height)
        obstacle_radius = radius
        offset_z = 0.3
    else:
        obstacle_radius = 0.0
        offset_z = 0.0

    translate_op = xform.AddTranslateOp()
    current_pos = generate_valid_position()
    translate_op.Set(Gf.Vec3f(current_pos[0], current_pos[1], offset_z))

    while True:
        found_valid_path = False
        for retry in range(20):
            target_pos = generate_valid_position()
            if not is_path_safe(current_pos, target_pos, obstacle_radius):
                continue

            duration = random.uniform(10.0, 16.0)
            steps = 100
            dt = duration / steps

            for step in range(steps + 1):
                alpha = step / steps
                interp_x = current_pos[0] * (1 - alpha) + target_pos[0] * alpha
                interp_y = current_pos[1] * (1 - alpha) + target_pos[1] * alpha
                interp_z = (current_pos[2] + offset_z) * (1 - alpha) + (target_pos[2] + offset_z) * alpha

                robot_pos = get_robot_position()
                dx = interp_x - robot_pos[0]
                dy = interp_y - robot_pos[1]
                dist = math.hypot(dx, dy)

                if dist < (MIN_CENTER_DIST + obstacle_radius):
                    print(f"[BLOCKED] {prim_path_str}: step {step} too close to robot (dist = {dist:.2f}), breaking")
                    current_pos = (interp_x, interp_y, interp_z - offset_z)
                    break

                translate_op.Set(Gf.Vec3f(interp_x, interp_y, interp_z))
                await asyncio.sleep(dt)
            else:
                current_pos = target_pos
                found_valid_path = True
                break

        if not found_valid_path:
            print(f"[SKIP] {prim_path_str}: could not find valid path after retries")
            await asyncio.sleep(1.0)

# Launch cubes
for i in range(NUM_CUBES):
    prim_path_str = f"/World/Cube_{i}"
    asyncio.ensure_future(animate_shape("cube", prim_path_str))

# Launch cylinders
for i in range(NUM_CYLINDERS):
    prim_path_str = f"/World/Cylinder_{i}"
    asyncio.ensure_future(animate_shape("cylinder", prim_path_str))


