# DC Servo Motor Digital Control Strategy Evaluation
This repository contains the implementation, simulation, and hardware deployment of various digital control strategies for a Quanser Rotary Servo Base Unit (SRV-02). The primary objective of this project is to explore digital controller design methodologies and evaluate the critical impact of discretization methods and sampling times ($T_s$) on closed-loop performance and stability. 

![State-Space Control by Emulation on Hardware](https://github.com/maxwell6q/Previous_Coursework/blob/main/Control_Engineering_Lab/Figures/control_system.png)
The algorithms developed here are fundamental to precise actuator control in complex non-linear applications, such as balancing robots. 

## Methodologies

* **System Modeling:** Implementation of a validated Simulink model of the plant, accommodating realistic non-linearities such as viscous and static friction, back EMF and estimated inertia parameters.
* **Digital PID by Emulation:**
    + Design of continuous-time PID controllers discretized using the Emulation method comparing the Backward/Forward Euler, Tustin, and Zero-Order Hold (ZOH) intrgration.
    + Implementation of an integrator Anti-Windup Scheme to mitigate overshoot due to actuator saturation.
* **State-Space Control by Emulation:**
    + Design of both nominal and robust (integral action) state-space controllers using continuous-time pole placement, subsequently discretized using the Emulation method.
    + To estimate the load angular velocity a reduced-order state observer was implemented.
* **State-Space Control by Direct Digital Design:**
    + Exact mapping of the continuous time plant model to the discrete-time domain.
    + Direct design of nominal and robust state-space controllers via z-plane pole placement.

## Key Results
* **The Sim-to-Real Gap:** While simulations showed near-identical responses across discretization methods at small sampling times (1 ms, 10 ms), the real hardware exhibited higher overshoot and exposed unmodeled dynamics.
* **Sampling Time Sensitivity:** Decreasing the sampling time does not universally improve performance (see Fig. (a), $T_s=1$ ms increases overshoot and settling time wrt. $T_s=10$ ms). At $T_s = 50$ ms, controllers utilizing Forward Euler discretization became severely unstable on both the model and the real plant, illustrating the strict stability bounds of the method (see Fig. (b)).
* **Anti-Windup Efficacy:** Implementing an anti-windup loop successfully mitigated excessive overshoot caused by actuator saturation during large reference steps.
* **Robust Tracking:** The addition of an integrator state to the state-space controllers successfully eliminated steady-state error, though it introduced slightly higher overshoot due to increased overall system gain.

(a) State-Space Control by Direct Digital Design on Hardware            |  (b) State-Space Control by Emulation on Hardware
:-------------------------:|:-------------------------:
![State-Space Control by Direct Digital Design on Hardware](https://github.com/maxwell6q/Previous_Coursework/blob/main/Control_Engineering_Lab/Figures/hw_ssDD_nom.png) | ![State-Space Control by Emulation on Hardware](https://github.com/maxwell6q/Previous_Coursework/blob/main/Control_Engineering_Lab/Figures/hw_ssEm_nom.png)