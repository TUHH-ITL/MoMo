from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, UsdShade
import omni.usd
import asyncio
import random

stage = omni.usd.get_context().get_stage()

# Parameters
num_spheres = 20
sphere_radius = 0.7
low_density = 2.0
speed = 3.0
material_path = "/World/PhysicsMaterial"
area_bounds = {
    "x_min": -26.0,
    "x_max": 4.42,
    "y_min": -22.0,
    "y_max": 8.0
}

# Remove old
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
if stage.GetPrimAtPath(material_path).IsValid():
    stage.RemovePrim(material_path)

# Create shared material
material = UsdShade.Material.Define(stage, Sdf.Path(material_path))
physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
physx_api.GetPrim().CreateAttribute("physxMaterial:staticFriction", Sdf.ValueTypeNames.Float).Set(0.5)
physx_api.GetPrim().CreateAttribute("physxMaterial:dynamicFriction", Sdf.ValueTypeNames.Float).Set(0.5)
physx_api.GetPrim().CreateAttribute("physxMaterial:restitution", Sdf.ValueTypeNames.Float).Set(0.1)

# Spawn spheres
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    pos = Gf.Vec3f(
        random.uniform(area_bounds["x_min"], area_bounds["x_max"]),
        random.uniform(area_bounds["y_min"], area_bounds["y_max"]),
        1.0
    )

    sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    sphere.GetRadiusAttr().Set(sphere_radius)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set(pos)

    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateDensityAttr().Set(low_density)

    binding_rel = sphere.GetPrim().CreateRelationship("physics:material:binding")
    binding_rel.SetTargets([material.GetPath()])

# Continuous motion loop
async def keep_moving_sphere(prim_path, bounds, speed):
    while True:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return

        # Choose a random target within the area
        target_x = random.uniform(bounds["x_min"], bounds["x_max"])
        target_y = random.uniform(bounds["y_min"], bounds["y_max"])
        pos = prim.GetAttribute("xformOp:translate").Get()

        # Compute direction vector and normalize
        direction = Gf.Vec3f(target_x - pos[0], target_y - pos[1], 0.0)
        length = direction.GetLength()
        if length < 0.1:
            await asyncio.sleep(0.1)
            continue
        direction.Normalize()

        # Apply velocity
        velocity = direction * speed
        UsdPhysics.RigidBodyAPI(prim).GetVelocityAttr().Set(velocity)

        # Move for a while then pick a new direction
        await asyncio.sleep(random.uniform(2.0, 4.0))

# Launch async motion for each sphere
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    asyncio.ensure_future(keep_moving_sphere(path, area_bounds, speed))

print("✅ Spheres spawned and continuously moving within the specified area.")

