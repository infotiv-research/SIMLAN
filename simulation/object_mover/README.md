Dyno-robotics/Infotiv delivery

Setting `self.MODE = "abnormal/normal"` in the main script causes the object movement to deviate from the normal distribution specified above.

To reproduce

```
git checkout ....
# In three separate terminals:
./control.sh clean ; ./control.sh build ; ./control.sh sim
./control.sh move_object
./control.sh camera_dump
```

## Random, In Distribution Objects Movement (normal mode)

Data collection from cameras according to [requirements](resources/20240522-req.png)

- Random self.objects are placed within the position range self.grid_x, self.grid_y: REQ.ID.1
- Random rotation along the z axis.
- You can use `camera_config ID.xacro` for placement of several cameras in the intersection. You can alternatively use `camera_config ID_noise.xacro`  to slightly changes the camera settings (REQ.ID.2) as below:
  - at most +-10 degree (+-0.1) rotation around one of the axis
  - at most +-10cm (+-0.1) change in x,y,z coordinates

## Random, Out of Distribution Objects Movement (abnormal mode)

Data collection from cameras according to [requirements](resources/20240610-req.png).

- Addition of other objects (spotlight, support pole, traffic cone): REQ.OD.1, REQ.OD.2.
- With some probability, the objects don't leave the scene and may coexist and collide with new objects: REQ.OD.3.
- Objects are at least rotated by π/4 (45 degrees, upside-down object): REQ.OD.3, REQ.OD.4.
- The spotlight is tilted by π/6 (30 degrees): REQ.OD.1.
- Objects are not placed on the ground and instead are dropped from a height of 2-3 meters: REQ.OD.4.

### Deterministic, Disentanglement One-parameter Object Movements (one_object_deterministic mode)

- Object: forklift
- Camera: 1
- (#Parameter:  orientation of forklift)
- Annotation: Position and rotation of the forklift. Annotation in filename.
- 256x256 RGB non compression

./control.sh sim
./control.sh move_object
./control.sh camera_dump
