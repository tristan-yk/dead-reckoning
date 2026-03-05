# dead-reckoning

## Simulation Quick Start

### Prerequisites
- MATLAB with Simulink installed.

### First-time setup (one time)
1. Open MATLAB.
2. Set the current folder to `Simulation`.
3. Run:

```matlab
configure_plant_project
```

This registers project startup/shutdown hooks so opening `plant.prj` adds all `Plant/` and `EKF/` subfolders to the MATLAB path, and closing the project removes them.

### Run the simulation
1. In MATLAB, open:
- `Simulation/plant.prj`
2. Open model:
- `Simulation/Plant/plant.slx`
3. Click **Run** in Simulink.

### Path behavior
- On project open: recursively adds `Simulation/Plant/**` and `Simulation/EKF/**` to path.
- On project close: removes those same paths.

### Optional MATLAB setting
If you do not want paths to come back automatically after restarting MATLAB, disable auto-reopen of the last project in MATLAB Preferences (`MATLAB > Projects`).
