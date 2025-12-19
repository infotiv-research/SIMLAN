# Converting meshes to SDF models

Here are the steps to convert FreeCAD meshes to SDF models (for Gazebo):

## Script step-by-step

- Install FreeCAD `sudo apt-get install freecad=0.19.2+dfsg1-3ubuntu1` (Tested with version 0.19)
- Install Blender v3.3 LTS
- Download phobos.zip (tested with [phobos 2.0 "Perilled Pangolin".](https://github.com/dfki-ric/phobos/releases/tag/2.0.0) )
- Install phobos by following [the guide on their GitHub](https://github.com/dfki-ric/phobos/tree/2.0.0#installation)
- In your phobos settings (easiest accessed through the Blender GUI), change the models folder to where you want the output to be. You can add a relative path here so that the output folder depends on where you run the script, e.g., setting the model folder to `./phobos_out/` would create a `./phobos_out/` directory as a sub-directory to wherever you run the script and save the exported SDFs there.
- Enter FreeCAD and load your `.FCstd`, then select the body in the tree view and select `File`->`Export` and choose to export it as a Collada (`.dae`) file.
- Run `./build.sh`

You should now be able to load the model/directory created by the reformat script directly through Gazebo.

```bash
wget https://download.blender.org/release/Blender3.3/blender-3.3.0-linux-x64.tar.xz
tar -xf blender-3.3.0-linux-x64.tar.xz
mv blender-3.3.0-linux-x64 blender
alias blender="`pwd`/blender/blender"
blender --version
git clone git@github.com:dfki-ric/phobos.git
cd phobos
git checkout 2.0.0
blender -b --python install_requirements.py
cd ..
./build.sh
```

## In GUI

- Import `.dae` file
- Set phobostype to `visual`
- Select the object and set the `geometry type`
- [Guide](https://github.com/dfki-ric/phobos/wiki/Modeling-Walkthrough) says to `Object->Apply->Rotation & Scale` here, but I'm not sure what it actually accomplishes
  - This will apply the scaling of objects to their mesh information (vertices) and applies the current orientation to the mesh as well. After this option, all objects have zero orientation. If this is not desired, one can also only apply the `Scale`.
- Create a "visual" collection and put the model into there; this lets us create a tree structure. (unsure of necessity when we only have one model)
- Select the object and press `Create Link(s)`, make sure `selected objects` mode is chosen.
- Select the visual and then `Create Collision`.
- Select the object and then `Create Inertials`.
  - Here we need to manually enter our mass; we should check if that can be adjusted somehow.
- Save and export.
- Then we need to move around the resulting files so they're structured in the way Gazebo wants them, i.e.:

```text
model
├── meshes
│   └── filename.dae
├── model.config
└── model.sdf
```

In order to accomplish this, we'll have to create the model.config file; however, this is relatively simple as it doesn't contain any unique data between different models aside from the model name.

Example of a successful build:

```
simulation/raw_models$ ./build_models.sh

../simlan_gazebo_environment/models/aruco/materials/textures/0.png
../simlan_gazebo_environment/models/aruco/materials/textures/1.png
../simlan_gazebo_environment/models/aruco/materials/textures/2.png
../simlan_gazebo_environment/models/aruco/materials/textures/3.png



Color management: using fallback mode for management
Color management: Error could not find role data role.
Blender 3.0.1
Read prefs: [HOME]/.config/blender/3.0/config/userpref.blend
Color management: scene view "Filmic" not found, setting default "Standard".
/usr/lib/python3/dist-packages/scipy/__init__.py:146: UserWarning: A NumPy version >=1.17.3 and <1.25.0 is required for this version of SciPy (detected version 1.26.2
  warnings.warn(f"A NumPy version >={np_minversion} and <{np_maxversion}"
Checking requirements:
ensurepip is disabled in Debian/Ubuntu for the system python.

Python modules for the system python are usually handled by dpkg and apt-get.

    apt install python3-<module name>

Install the python3-pip package to use pip itself.  Using pip together
with the system python might have unexpected results for any system-installed
module, so use it at your own risk, or make sure to only use it in virtual
environments.

WARNING: We couldn't do ensurepip; we try to continue anyway
  Checking yaml
  Checking numpy
  Checking scipy
  Checking collada
  Checking pydot
  Checking lxml
  Checking networkx
  Checking trimesh
  Checking PIL
Importing phobos
IMPORT:  phobos.blender.defs
Parsing definitions from: [HOME]/.config/blender/3.0/scripts/addons/phobos/data/blender/definitions
  defaultControllers.yml
  defaultSensors.yml
  defaultMaterials.yml
  defaultJoints.yml
  defaultSubmechanisms.yml
  defaultMotors.yml
Creating new definition type: joints
Creating new definition type: motors
IMPORT:  phobos.blender.display
IMPORT:  phobos.blender.io
RELOAD:  phobos.blender.model
IMPORT:  phobos.blender.operators
RELOAD:  phobos.blender.phobosgui
RELOAD:  phobos.blender.phoboslog
RELOAD:  phobos.blender.phobossystem
RELOAD:  phobos.blender.reserved_keys
RELOAD:  phobos.blender.utils
Registering operators.selection...
Registering operators.io...
Registering operators.editing...
TypeError: EnumProperty(..., default='mechanism'): not found in enum members
ValueError: bpy_struct "PHOBOS_OT_define_submodel" registration error: 'submodeltype' EnumProperty could not register (see previous error)
Registering operators.naming...
Registering operators.misc...

Registering phobosgui...
  ... successful.
+-- Collada Import parameters------
| input file      : ./objects/meshes/eur-pallet.dae
| use units       : no
| autoconnect     : yes
+-- Armature Import parameters ----
| find bone chains: yes
| min chain len   : 0
| fix orientation : yes
| keep bind info  : no
| IOR of negative value is not allowed for materials (using Blender default value instead)+-- Import Scene --------
| NODE  id='node0', name='node0'
+----------------------------------
| Collada Import : OK
+----------------------------------
Error in sys.excepthook:

Original exception was:
File "[HOME]/.config/blender/3.0/scripts/addons/phobos/blender/operators/editing.py", line 1048, in toggleVisual
Error in sys.excepthook:

Original exception was:
File "[HOME]/.config/blender/3.0/scripts/addons/phobos/blender/operators/editing.py", line 1051, in toggleCollision
[20231211_08:52:53] WARNING No text file README.md found. (phobos/blender/utils/blender.py - readTextFile (l259))
Collada export to: [PATH]simulation/raw_models/phobos_out/unnamed/meshes/dae/Body.dae
Info: Exported 1 Object
Info: Exported 1 Object
Collada export to: [PATH]simulation/raw_models/phobos_out/unnamed/meshes/dae/Body.dae
Info: Exported 1 Object
Info: Exported 1 Object
Info: Phobos exported to: phobos_out/unnamed
Info: Export successful.
Info: Phobos exported to: phobos_out/unnamed
Info: Export successful.

----------------------------------------------------------------------------------------------------
Unregistering Phobos...
Unregistering phobosgui...
Unregistering display...
Unregistering icons...
Unregistering classes...
Unregistering manuals...
  ... successful.

Blender quit
Error: Not freed memory blocks: 14, total unfreed memory 0.002426 MB
```
