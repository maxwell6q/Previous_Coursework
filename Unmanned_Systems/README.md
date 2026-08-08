# Unmanned Systems: Hexarotor Modeling, Control, and Visual Interception
This repository contains MATLAB and Simulink implementations of a high-fidelity dynamic model for a hexarotor UAV, along with custom-designed flight controllers and a vision-based target interception system (videos).

Scene Camera             |  Interceptor Mounted
:-------------------------:|:-------------------------:
![Scene Camera](https://github.com/maxwell6q/Previous_Coursework/blob/main/Unmanned_Systems/scene.mp4) | ![Interceptor Mounted](https://github.com/maxwell6q/Previous_Coursework/blob/main/Unmanned_Systems/husky.mp4)

## Dynamic Modeling
The platform model is based on a custom hexarotor developed by the University of Bologna. The Simulink environment incorporates high-fidelity subsystems to reflect real-world flight dynamics:

* **Power Electronics & Powertrain:** A coupled model integrating a 6S LiPo battery, DJI OPTO 30A ESCs, and DJI 2212 brushless motors. It features a quadratic efficiency mapping based on motor angular velocity and torque, alongside an exponential PWM-to-RPM mapping identified via least-squares fitting on experimental testbed data.

* **Aerodynamics:** Rotor thrust and torque are modeled using Blade Element Momentum Theory (BEMT) based on [NASA Technical report TM84281](https://ntrs.nasa.gov/citations/19830001781), parameterized using physical measurements from the actual propellers.

* **Rigid Body Dynamics:** Mass, inertia, and aerodynamic drag coefficients derived from physical experiments on the real drone frame.

* **Environmental Model:** Includes the International Standard Atmosphere (ISA) model, configurable wind conditions, and a gravity model based on WGS84.

## Control Architecture
The baseline flight controller mirrors the architecture of the PX4 / Pixhawk autopilot, using a cascaded control topology for velocity, attitude, and angular rates. Controller parameters were fine-tuned interactively using real-time joystick flight input.

## Visual Interception Pipeline
To evaluate vision-based target tracking, the simulation is co-simulated with an Unreal Engine 3D environment.

* **Setup:** A multi-UAV scene where a pursuit hexarotor is equipped with an onboard camera feed.

* **Perception & Control:** The pursuing drone detects the adversary target via corner detection features, estimating its relative bearing to generate velocity setpoints for autonomous interception.


## Simulation Results
The figure shows the tracking behaviour of the body velocities when the interception mode is enabled. It can be seen that the velocity follows the target well.


![Body velocities during interception](https://github.com/maxwell6q/Previous_Coursework/blob/main/Unmanned_Systems/VelocityInterception.png)