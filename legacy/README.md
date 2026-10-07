# Archived ROS / Gazebo prototype

These files preserve the original prototype for reference. They are not part of
installation, CI, or the MuJoCo simulation. Controllers and perception contain
placeholders; the old hover node publishes gravity as velocity and is not a valid
flight controller. Old setup instructions mix Gazebo versions and are historical.

The broken root gitlinks were removed (there was no .gitmodules). Their original
references remain recoverable in Git history:
- ArduPilot: 453d99d1c98c21867b1e0c4c428a31c0880a4fe5
- ardupilot_gazebo: 685e3e3d10c0ba33324df2b81474f9ed5c01289f

Run any future ROS integration as a separate adapter to the Python simulator.
