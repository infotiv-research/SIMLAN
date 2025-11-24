# Gazebo Worlds

```xml
    <physics type="ode">
      <ode>
        <solver>
          <type>world</type>
        </solver>
        <constraints>
          <contact_max_correcting_vel>0.1</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size>
    </physics>
```

Two solvers:
world step gives an accurate solution if it is able to solve the problem, while quick step depends on the number of iterations to reach an accurate enough solution.

### contact/collision parameters

dampingFactor 	double 	Exponential velocity decay of the link velocity - takes the value and multiplies the previous link velocity by (1-dampingFactor).
maxVel 	double 	maximum contact correction velocity truncation term.
minDepth 	double 	minimum allowable depth before contact correction impulse is applied
maxContacts 	int 	Maximum number of contacts allowed between two entities. This value overrides the max_contacts element defined in physics.

contact_max_correcting_vel : contact_max_correcting_vel This is the same parameter as the max_vel under collision->surface->contact. contact_max_correcting_vel sets max_vel globally.
contact_surface_layer : contact_surface_layer This is the same parameter as the min_depth under collision->surface->contact.

*Note*: We had issue getting global settings (`contact_max_correcting_vel` and `contact_surface_layer`) working. We therefore define object level `minDepth` and `maxVel` for each object

"quick" solver parameters
min_step_size The minimum time duration which advances with each time step of a variable time step solver.
iters The number of iterations for the solver to run for each time step.

### objects:

As of now there are many objects in the world file nad if you want to move them all it's quite cumbersome. Therefore the move_objects.py was added to be able to increment the poses by x amount if needed.
See move_objects.py

### reference:

- [Physics Parameters](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics)
- [SDF format](http://sdformat.org/spec)
