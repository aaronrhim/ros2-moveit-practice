# Here is the additional parameters we need that the URDF does not provide

### Joint parameters
Joint Torque Range | range="lower upper"
- Range of motion (prevents motion beyond that range)
Viscous Damping Coefficient | damping="float"
- Adds velocity-proportional resistance — smooths motion, prevents oscillation.
Rotational/Translational Inertia at Joint | armature="float"
- Adds inertia to the motor which helps with massless/stiff joints
Friction | frictionloss="float"
- Force/torque that must be exceeded before joint moves

### Geometry parameters
Friction | friction="slide spin roll"
- sliding, tortorsional, and rolling friction

### Actuator parameters
Control Torque Range | ctrlrange="lower upper"
- Controls the range of voltage you can send to each individual actuator