from pxr import UsdGeom, Gf, Sdf
import omni.usd
import asyncio

stage = omni.usd.get_context().get_stage()

# Define (shape_type, position) pairs
shape_specs = [
    ("cube", (-2, -2, 0)),
    ("sphere", (-4, -4, 0)),
    ("capsule", (1, 1, 0)),
    ("cone", (-3, -5, 0)),
    ("cylinder", (-6, -2, 0)),
    ("cube", (1, -2, 0)),  # reusing cube for final shape
]

# Function to define the shape based on type
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

# Async function to spawn and remove shapes
async def spawn_shapes_sequentially():
    for i, (shape_type, pos) in enumerate(shape_specs):
        prim_path_str = f"/World/Shape_{i}_{shape_type}"
        prim_path = Sdf.Path(prim_path_str)

        # Define shape
        shape_prim = define_shape(stage, shape_type, prim_path)
        xform = UsdGeom.Xformable(shape_prim)

        # Scale and translate per shape to keep base at Z=0
        if shape_type == "cube":
            xform.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 2.0))  # height = 2m
            adjusted_z = pos[2] + 1.0
        elif shape_type == "sphere":
            shape_prim.GetRadiusAttr().Set(1.0)
            adjusted_z = pos[2] + 1.0
        elif shape_type == "capsule":
            shape_prim.GetRadiusAttr().Set(0.5)
            shape_prim.GetHeightAttr().Set(2.0)
            adjusted_z = pos[2] + 1.0
        elif shape_type == "cone":
            shape_prim.GetRadiusAttr().Set(1.0)
            shape_prim.GetHeightAttr().Set(2.0)
            adjusted_z = pos[2] + 1.0
        elif shape_type == "cylinder":
            shape_prim.GetRadiusAttr().Set(1.0)
            shape_prim.GetHeightAttr().Set(2.0)
            adjusted_z = pos[2] + 1.0
        else:
            adjusted_z = pos[2]  # fallback, shouldn't happen

        xform.AddTranslateOp().Set(Gf.Vec3f(pos[0], pos[1], adjusted_z))

        print(f"✅ Spawned {shape_type} at {pos}")

        await asyncio.sleep(3)

        stage.RemovePrim(prim_path)
        print(f"❌ Removed {shape_type} at {pos}")

# Run the async task
asyncio.ensure_future(spawn_shapes_sequentially())

