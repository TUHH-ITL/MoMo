import asyncio
import random

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade

stage = omni.usd.get_context().get_stage()

# Sphere group configuration
sphere_groups = {
    "small": {
        "count": 20,
        "min_radius": 0.26,  # d_eff = 0.2
        "max_radius": 0.3125,  # d_eff = 0.5
    },
    "medium": {
        "count": 10,
        "min_radius": 0.3125,  # d_eff = 0.5
        "max_radius": 0.5,  # # d_eff = 1.0
    },
    "large": {
        "count": 5,
        "min_radius": 0.5,  # d_eff = 1.0
        "max_radius": 0.8125,  # d_eff = 1.5
    },
}

low_density = 1.0
max_speed = 5.0
material_path = "/World/PhysicsMaterial"
z_height = 0.8

# Define 4 corner points of the motion region
corner_targets = [
    Gf.Vec3f(1, -1, z_height),
    Gf.Vec3f(19, -1, z_height),
    Gf.Vec3f(19, -23, z_height),
    Gf.Vec3f(1, -23, z_height),
]

# Compute total number of spheres
num_spheres = sum(group["count"] for group in sphere_groups.values())

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
physx_api.GetPrim().CreateAttribute(
    "physxMaterial:staticFriction",
    Sdf.ValueTypeNames.Float,
).Set(0.5)
physx_api.GetPrim().CreateAttribute(
    "physxMaterial:dynamicFriction",
    Sdf.ValueTypeNames.Float,
).Set(0.5)
physx_api.GetPrim().CreateAttribute(
    "physxMaterial:restitution",
    Sdf.ValueTypeNames.Float,
).Set(0.1)

# Spawn spheres with group-based radius
sphere_index = 0
for group_name, config in sphere_groups.items():
    for _ in range(config["count"]):
        path = f"/World/BouncingSphere_{sphere_index}"
        start = random.choice(corner_targets)
        radius = random.uniform(config["min_radius"], config["max_radius"])

        sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
        sphere.GetRadiusAttr().Set(radius)
        UsdGeom.Xformable(sphere).AddTranslateOp().Set(start)

        UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
        UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateDensityAttr().Set(
            low_density,
        )

        # Bind material
        binding_rel = sphere.GetPrim().CreateRelationship(
            "physics:material:binding",
        )
        binding_rel.SetTargets([material.GetPath()])

        sphere_index += 1


# Async motion with smooth velocity changes
async def keep_moving_sphere(prim_path, corners, speed):
    while True:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return

        current_pos = prim.GetAttribute("xformOp:translate").Get()
        target = random.choice(
            [
                c
                for c in corners
                if (
                    abs(c[0] - current_pos[0]) > 1.0
                    or abs(c[1] - current_pos[1]) > 1.0
                )
            ],
        )

        direction = Gf.Vec3f(
            target[0] - current_pos[0],
            target[1] - current_pos[1],
            0.0,
        )
        direction.Normalize()
        target_velocity = direction * speed

        rb_api = UsdPhysics.RigidBodyAPI(prim)
        velocity_attr = rb_api.GetVelocityAttr()

        for _ in range(30):  # ~3s smooth transition
            current_vel = velocity_attr.Get()
            alpha = 0.1
            new_vel = (1 - alpha) * current_vel + alpha * target_velocity
            velocity_attr.Set(new_vel)
            await asyncio.sleep(0.1)

        await asyncio.sleep(random.uniform(2.0, 3.0))


# Launch motion for each sphere
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    asyncio.ensure_future(keep_moving_sphere(path, corner_targets, max_speed))

print(f"✅ Spawned {num_spheres} spheres in 3 size groups with smooth motion.")
