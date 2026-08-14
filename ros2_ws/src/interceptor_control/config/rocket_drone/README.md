# Rocket Drone CFD Aerodynamic Model

Gazebo AdvancedLiftDrag plugin configuration for the Rocket Drone.

## CFD Conditions

ANSYS CFD results based on the CATIA Rocket Drone geometry.

- Air density: 1.225 kg/m^3
- Reference area: 0.05890 m^2
- CFD velocity: 5 m/s, 10 m/s
- Angle of attack: 0, 5, 10 deg

## CFD Results

| Velocity | AoA | CD | CL |
|---|---:|---:|---:|
| 5 m/s | 0 deg | 0.186 | -0.00243 |
| 5 m/s | 5 deg | 0.1895 | 0.1710 |
| 5 m/s | 10 deg | 0.203 | 0.328 |
| 10 m/s | 0 deg | 0.179 | -0.0011 |
| 10 m/s | 5 deg | 0.182 | 0.172 |
| 10 m/s | 10 deg | 0.195 | 0.331 |

## Gazebo Parameters

- CL0 = 0.0
- CD0 = 0.186
- CLa = 1.89 rad^-1
- Reference area = 0.05890 m^2
- CP z = 0.027097 m

These coefficients were applied to the Gazebo AdvancedLiftDrag plugin
and validated using the High-Speed Mission.
