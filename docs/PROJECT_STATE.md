# Project State

## Project Goal

Build three autonomous RC vehicles capable of V2V communication and
cooperative control.

First major demo:
- Three-vehicle cascading braking


## Current Development Stage

Currently developing the singlevehicle control simulation in Python.

Completed:
- [x] Single-vehicle model
- [x] PI velocity control
- [x] Drag/friction model
- [x] Actuator saturation
- [x] Integral anti-windup
- [x] Two independent vehicle objects
- [x] Separate PI controller for each vehicle
- [x] Following controller for Car 2
- [x] Car 2 calculates its own velocity reference from Car 1 state
- [x] Planned braking test with two vehicles
- [x] Inter-vehicle distance logging and plotting

Current vehicle behavior:
- Car 2 follows Car 1 using leader position and velocity
- Desired following distance is 2.0m
- Following control is not tuned yet. Lags behind and then overshoots slightly

Next:
- [ ] Tune/Test following controller
- [ ] Add Car 3
- [ ] Add Car 2 -> Car 3 following relationship
- [ ] Implement three-vehicle cascading braking
- [ ] Add simulated V2V message layer