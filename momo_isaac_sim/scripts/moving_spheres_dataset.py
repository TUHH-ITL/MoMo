from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, UsdShade
import omni.usd
import asyncio
import random

stage = omni.usd.get_context().get_stage()

# Parameters
num_spheres = 25
sphere_radius = 0.5
low_density = 1.0
max_speed = 5.0
material_path = "/World/PhysicsMaterial"
z_height = 0.8

# Define 4 corner points of the motion region
corner_targets = [
    Gf.Vec3f(4.27, 8, z_height),
    Gf.Vec3f(-25.3, 8, z_height),
    Gf.Vec3f(4.42, -22, z_height),
    Gf.Vec3f(-26, -22, z_height)
]

# Remove old spheres and material if they exist
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)
if stage.GetPrimAtPath(material_path).IsValid():
    stage.RemovePrim(material_path)

# Create shared physics material
material = UsdShade.Material.Define(stage, Sdf.Path(material_path))
physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
physx_api.GetPrim().CreateAttribute("physxMaterial:staticFriction", Sdf.ValueTypeNames.Float).Set(0.5)
physx_api.GetPrim().CreateAttribute("physxMaterial:dynamicFriction", Sdf.ValueTypeNames.Float).Set(0.5)
physx_api.GetPrim().CreateAttribute("physxMaterial:restitution", Sdf.ValueTypeNames.Float).Set(0.1)

# Spawn spheres at random corners
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    start = random.choice(corner_targets)
    sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    sphere.GetRadiusAttr().Set(sphere_radius)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set(start)

    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateDensityAttr().Set(low_density)

    # Bind material
    binding_rel = sphere.GetPrim().CreateRelationship("physics:material:binding")
    binding_rel.SetTargets([material.GetPath()])

# Async motion with smooth velocity changes
async def keep_moving_sphere(prim_path, corners, speed):
    while True:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return

        # Choose a target different from current corner
        current_pos = prim.GetAttribute("xformOp:translate").Get()
        target = random.choice([c for c in corners if (abs(c[0] - current_pos[0]) > 1.0 or abs(c[1] - current_pos[1]) > 1.0)])

        direction = Gf.Vec3f(target[0] - current_pos[0], target[1] - current_pos[1], 0.0)
        direction.Normalize()
        target_velocity = direction * speed

        rb_api = UsdPhysics.RigidBodyAPI(prim)
        velocity_attr = rb_api.GetVelocityAttr()

        # Smooth transition
        for _ in range(30):  # ~3s smooth change
            current_vel = velocity_attr.Get()
            alpha = 0.1
            new_vel = (1 - alpha) * current_vel + alpha * target_velocity
            velocity_attr.Set(new_vel)
            await asyncio.sleep(0.1)

        # Let it travel before picking new target
        await asyncio.sleep(random.uniform(2.0, 3.0))

# Launch motion for each sphere
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    asyncio.ensure_future(keep_moving_sphere(path, corner_targets, max_speed))

print("✅ 8 spheres spawned with long-range corner-to-corner motion and smooth velocity transitions.")

