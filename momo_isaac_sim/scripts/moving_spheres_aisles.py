from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, UsdShade
import omni.usd
import asyncio
import random

stage = omni.usd.get_context().get_stage()

# Parameters
num_spheres = 20
sphere_radius = 0.5
low_density = 1.0
max_speed = 5.0
material_path = "/World/PhysicsMaterial"
z_height = 0.8

# Define warehouse boundaries and aisle coordinates
warehouse_bounds = [(4.6, 29.8), (-25, 29.8), (-25, -22), (4.6, -22)]
aisles = [
    [(5.5, 26), (3.5, 26), (3.2, 8), (4.5, 8)],  # Aisle 1
    [(0.6, 26), (-1.3, 26), (-1.6, 8), (0.4, 8)],  # Aisle 2
    [(-4.7, 26), (-6.5, 26), (-6.4, 8), (-4.8, 8)],  # Aisle 3
    [(-9.3, 26), (-11.4, 26), (-11.4, 8), (-9.6, 8)],  # Aisle 4
    [(-14.5, 26), (-16.4, 26), (-16.4, 8), (-14.5, 8)],  # Aisle 5
    [(-19.5, 26), (-21.4, 26), (-21.4, 8), (-19.5, 8)],  # Aisle 6
    [(-24.4, 26), (-25, 26), (-25, 8), (-24.5, 8)]  # Aisle 7
]

def point_in_polygon(x, y, polygon):
    """Check if a point is inside a polygon using ray casting algorithm"""
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def is_valid_position(x, y, buffer=0.5):
    """Check if a position is valid (within warehouse but not inside actual aisles)"""
    # Check if within warehouse bounds
    if x < -24.5 or x > 4.2 or y < -21.5 or y > 29.3:
        return False
    
    # Check if inside any aisle structure (the actual shelving areas)
    for aisle in aisles:
        # Get the aisle boundaries
        min_x = min(point[0] for point in aisle) - buffer
        max_x = max(point[0] for point in aisle) + buffer
        min_y = min(point[1] for point in aisle) - buffer
        max_y = max(point[1] for point in aisle) + buffer
        
        # Only block if actually inside the aisle rectangle
        if min_x <= x <= max_x and min_y <= y <= max_y:
            return False
    
    return True

def generate_valid_positions(count, buffer=0.5):
    """Generate valid random positions for spheres"""
    positions = []
    max_attempts = 500
    
    for _ in range(count):
        attempts = 0
        while attempts < max_attempts:
            # Generate random position within warehouse bounds
            x = random.uniform(-24.0, 4.0)
            y = random.uniform(-21.0, 29.0)
            
            if is_valid_position(x, y, buffer):
                positions.append(Gf.Vec3f(x, y, z_height))
                break
            attempts += 1
        
        if attempts >= max_attempts:
            # Fallback to pathway positions (between aisles)
            pathway_positions = [
                # Main pathways between aisles
                Gf.Vec3f(2.0, 17.0, z_height),   # Between aisle 1 and edge
                Gf.Vec3f(-0.5, 17.0, z_height),  # Between aisle 1 and 2
                Gf.Vec3f(-3.0, 17.0, z_height),  # Between aisle 2 and 3
                Gf.Vec3f(-7.8, 17.0, z_height),  # Between aisle 3 and 4
                Gf.Vec3f(-12.5, 17.0, z_height), # Between aisle 4 and 5
                Gf.Vec3f(-17.5, 17.0, z_height), # Between aisle 5 and 6
                Gf.Vec3f(-22.5, 17.0, z_height), # Between aisle 6 and 7
                # Open areas
                Gf.Vec3f(0.0, 28.0, z_height),   # Top area
                Gf.Vec3f(-12.0, 28.0, z_height), # Top area
                Gf.Vec3f(0.0, -15.0, z_height),  # Bottom area
                Gf.Vec3f(-12.0, -15.0, z_height) # Bottom area
            ]
            positions.append(random.choice(pathway_positions))
    
    return positions

def get_valid_target_position(current_pos, buffer=0.5):
    """Get a valid target position anywhere in the warehouse including pathways"""
    max_attempts = 150
    attempts = 0
    
    while attempts < max_attempts:
        # Generate completely random position within warehouse bounds
        x = random.uniform(-24.0, 4.0)
        y = random.uniform(-21.0, 29.0)
        
        # Check if position is valid
        if is_valid_position(x, y, buffer):
            return Gf.Vec3f(x, y, z_height)
        
        attempts += 1
    
    # Fallback to pathway and open area positions throughout warehouse
    pathway_positions = [
        # Pathways between aisles (main traffic areas)
        Gf.Vec3f(2.5, 17.0, z_height),   # Right side pathway
        Gf.Vec3f(-0.5, 17.0, z_height),  # Between aisle 1 and 2
        Gf.Vec3f(-3.0, 17.0, z_height),  # Between aisle 2 and 3
        Gf.Vec3f(-7.8, 17.0, z_height),  # Between aisle 3 and 4
        Gf.Vec3f(-12.5, 17.0, z_height), # Between aisle 4 and 5
        Gf.Vec3f(-17.5, 17.0, z_height), # Between aisle 5 and 6
        Gf.Vec3f(-22.5, 17.0, z_height), # Between aisle 6 and 7
        
        # Top open area (above aisles)
        Gf.Vec3f(2.0, 27.5, z_height), Gf.Vec3f(-5.0, 27.5, z_height),
        Gf.Vec3f(-12.0, 27.5, z_height), Gf.Vec3f(-20.0, 27.5, z_height),
        
        # Bottom open area (below aisles)  
        Gf.Vec3f(2.0, 0.0, z_height), Gf.Vec3f(-5.0, 0.0, z_height),
        Gf.Vec3f(-12.0, 0.0, z_height), Gf.Vec3f(-20.0, 0.0, z_height),
        
        # Very bottom area
        Gf.Vec3f(2.0, -15.0, z_height), Gf.Vec3f(-12.0, -15.0, z_height), 
        Gf.Vec3f(-22.0, -15.0, z_height),
        
        # Vertical pathways (ends of aisles)
        Gf.Vec3f(2.0, 7.5, z_height), Gf.Vec3f(-12.0, 7.5, z_height), 
        Gf.Vec3f(-22.0, 7.5, z_height)
    ]
    
    return random.choice(pathway_positions)

# Remove old spheres and material if they exist
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

if stage.GetPrimAtPath(material_path).IsValid():
    stage.RemovePrim(material_path)

# Create shared physics material with high bounciness
material = UsdShade.Material.Define(stage, Sdf.Path(material_path))
physx_api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())

# Set friction and high restitution (bounciness)
physx_api.GetPrim().CreateAttribute("physxMaterial:staticFriction", Sdf.ValueTypeNames.Float).Set(0.1)
physx_api.GetPrim().CreateAttribute("physxMaterial:dynamicFriction", Sdf.ValueTypeNames.Float).Set(0.1)
physx_api.GetPrim().CreateAttribute("physxMaterial:restitution", Sdf.ValueTypeNames.Float).Set(0.99)  # ↑ More bounce!

# Define the four corners of the warehouse
warehouse_corners = [
    Gf.Vec3f(4.0, 29.0, z_height),   # Top right corner
    Gf.Vec3f(-24.0, 29.0, z_height), # Top left corner
    Gf.Vec3f(4.0, -21.0, z_height),  # Bottom right corner
    Gf.Vec3f(-24.0, -21.0, z_height) # Bottom left corner
]

# Generate starting positions from corners
def generate_corner_positions(count):
    """Generate starting positions distributed among the four warehouse corners"""
    positions = []
    corner_index = 0
    
    for i in range(count):
        # Cycle through corners
        base_corner = warehouse_corners[corner_index % 4]
        
        # Add small random offset from exact corner (±1.5 units)
        offset_x = random.uniform(-1.5, 1.5)
        offset_y = random.uniform(-1.5, 1.5)
        
        # Ensure the offset position is still valid
        new_x = base_corner[0] + offset_x
        new_y = base_corner[1] + offset_y
        
        # Clamp to warehouse bounds and ensure not in aisles
        new_x = max(-24.0, min(4.0, new_x))
        new_y = max(-21.0, min(29.0, new_y))
        
        if is_valid_position(new_x, new_y, buffer=0.8):
            positions.append(Gf.Vec3f(new_x, new_y, z_height))
        else:
            # Fallback to exact corner if offset position is invalid
            positions.append(base_corner)
        
        corner_index += 1
    
    return positions

# Generate starting positions from corners
valid_positions = generate_corner_positions(num_spheres)

# Spawn spheres at valid positions
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    start_pos = valid_positions[i]
    
    sphere = UsdGeom.Sphere.Define(stage, Sdf.Path(path))
    sphere.GetRadiusAttr().Set(sphere_radius)
    UsdGeom.Xformable(sphere).AddTranslateOp().Set(start_pos)
    
    UsdPhysics.RigidBodyAPI.Apply(sphere.GetPrim())
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    UsdPhysics.MassAPI.Apply(sphere.GetPrim()).CreateDensityAttr().Set(low_density)
    
    # Bind material
    binding_rel = sphere.GetPrim().CreateRelationship("physics:material:binding")
    binding_rel.SetTargets([material.GetPath()])

# Async motion with collision avoidance
async def keep_moving_sphere(prim_path, speed):
    while True:
        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            return
        
        # Get current position
        current_pos = prim.GetAttribute("xformOp:translate").Get()
        
        # Get valid target position
        target = get_valid_target_position(current_pos, buffer=0.5)
        
        # Calculate direction and velocity
        direction = Gf.Vec3f(target[0] - current_pos[0], target[1] - current_pos[1], 0.0)
        direction.Normalize()
        target_velocity = direction * speed
        
        rb_api = UsdPhysics.RigidBodyAPI(prim)
        velocity_attr = rb_api.GetVelocityAttr()
        
        # Smooth velocity transition
        for _ in range(30):  # ~3s smooth change
            current_vel = velocity_attr.Get()
            alpha = 0.1
            new_vel = (1 - alpha) * current_vel + alpha * target_velocity
            velocity_attr.Set(new_vel)
            await asyncio.sleep(0.1)
        
        # Let it travel before picking new target
        await asyncio.sleep(random.uniform(2.0, 4.0))

# Launch motion for each sphere
for i in range(num_spheres):
    path = f"/World/BouncingSphere_{i}"
    asyncio.ensure_future(keep_moving_sphere(path, max_speed))

print(f"✅ {num_spheres} spheres spawned with warehouse-aware movement, avoiding aisles.")

