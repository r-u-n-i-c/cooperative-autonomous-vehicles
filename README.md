# Coorperative Autonomous Vehicles

## Overview
Senior design project focused on V2V (vehicle to vehicle) communication for small autonomous RC cars. The vehicles will coordinate their motion to display collision avoidance, vehicle following, and autonomous intersection control

The control and coordination algoithms are first to be developed and tested in simulation.

### Project Goals
-  Autonomous Intersection control of 3 vehicles

### Planned Arcitecture
Python will be primarily used for simulation, allgorithm, development, and testing.

MATLAB might be used for verification of any mathematics.

C/C++ will be used for final embedded implementation.

### Current Status
- [ ] 3 Vehicles. Car 2 follows Car 1 using leader position and velocity, and Car 3 follows Car 2 by maintaining fixed distance
- [ ] Include braking comparison for different leader brake message hops at 5 ms, 10 ms, 20 ms, and 50 ms
- [ ] 2 Vehicle automatic interestion crossing without stopping. This is a first proof of concept before meeting with professor to define solution for 3 vehicle crossing.