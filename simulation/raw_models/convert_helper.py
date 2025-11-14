# Author: Anders Bäckelie
"""
Run with `blender -b --python convert_helper.py -- [.dae] [mass in kg]`
Example: `blender -b --python convert_helper.py -- ./objects/eur-pallet.dae 25`
"""
import sys
import bpy
import phobos

# from https://blender.stackexchange.com/a/8405
argv = sys.argv
args = argv[argv.index("--") + 1 :]  # get all args after "--"

# Import .dae into blender
bpy.ops.wm.collada_import(
    filepath=args[0], auto_connect=True, find_chains=True, fix_orientation=True
)

mesh_obj = bpy.context.object

# Create a copy of the mesh that will be used for collision
bpy.ops.object.duplicate()
coll_obj = bpy.context.object

# Clean the collision mesh
bpy.ops.object.editmode_toggle()
bpy.ops.mesh.select_all(action="SELECT")
bpy.ops.mesh.remove_doubles(threshold=0.01, use_sharp_edge_from_normals=True)
bpy.ops.object.editmode_toggle()

# Set phobostype on imported object to 'visual'
mesh_obj.select_set(True)
coll_obj.select_set(False)
bpy.ops.phobos.set_phobostype(phobostype="visual")

# Set phobostype on copied object to 'collision'
mesh_obj.select_set(False)
coll_obj.select_set(True)
bpy.ops.phobos.set_phobostype(phobostype="collision")

mesh_obj.select_set(False)
coll_obj.select_set(False)

# Select object and set geometry type
mesh_obj["geometry/type"] = "mesh"
coll_obj["geometry/type"] = "mesh"

# Object Apply Rotation&Scale
mesh_obj.select_set(True)
bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
mesh_obj.select_set(False)

coll_obj.select_set(True)
bpy.ops.object.transform_apply(location=False, rotation=True, scale=True)
coll_obj.select_set(False)

# Select object and Create Link(s)
mesh_obj.select_set(True)
coll_obj.select_set(False)
bpy.ops.phobos.create_links(
    location="selected objects", parent_link=True, parent_objects=True
)
link_obj = bpy.context.object
link_obj.name = "model_link"
# Select the collision mesh and Create Collision
mesh_obj.select_set(False)
coll_obj.select_set(True)
link_obj.select_set(True)
# bpy.ops.phobos.create_collision_objects(property_colltype='mesh')
bpy.ops.phobos.parent()

# Select object and Create Inertials
mesh_obj.select_set(False)
coll_obj.select_set(True)
link_obj.select_set(False)
bpy.ops.phobos.generate_inertial_objects(
    mass=float(args[1]),
    derive_inertia_from_geometry=True,
    visuals=False,
    collisions=True,
)

# Save & Export
bpy.context.scene.export_mesh_dae = True
bpy.context.scene.export_entity_sdf = True
bpy.context.scene.export_entity_urdf = False
bpy.context.scene.export_entity_smurf = False
bpy.context.scene.phobosexportsettings.export_sdf_mesh_type = "dae"

bpy.ops.phobos.select_model()
bpy.ops.phobos.export_model(modelname="*model_link")
