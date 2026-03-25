# Independent Research Report
## ME 4991 — Undergraduate Research

**Title:** Hierarchical Flight Controller Design for Aggressive Quadrotor Trajectory Tracking: Sliding Mode Control with Differential Flatness Feedforward

**Author:** Hojun Kim
**Advisor:** Prof. Rifat Sipahi
**Department of Mechanical and Industrial Engineering, Northeastern University**
**Summer 1, 2025**

---

## Abstract

This report documents the design, derivation, and simulation of a hierarchical flight controller for a 6DOF quadrotor performing aggressive trajectory tracking. The work was conducted as an independent undergraduate research project (ME 4991) prior to formal coursework in nonlinear control theory, requiring the author to independently survey relevant literature and derive control formulations from first principles. The proposed architecture combines Sliding Mode Control (SMC) for both position and attitude loops with a Differential Flatness (DF) feedforward term that analytically computes desired angular rates and accelerations from the jerk and snap of the reference trajectory. Lyapunov-based stability is proven for each control loop. Simulation results on a C⁴ Lemniscate trajectory demonstrate stable and accurate tracking. A subsequent stage extending the outer loop to Incremental Nonlinear Dynamic Inversion (INDI) is also outlined as ongoing work.

---

## 1. Introduction

In recent years, quadrotors have garnered significant attention, proving their potential in various fields such as search and rescue, surveillance, and logistics. To successfully perform these missions, the ability to quickly and accurately track reference trajectories that require dynamic and aggressive maneuvers in complex 3D environments is essential.

However, precisely controlling the autonomous flight of a quadrotor involves considerable technical challenges. The quadrotor system has complex characteristics, including strong nonlinear dynamics, coupling between rotors, unpredictable aerodynamic effects and external disturbances. While the system can be controlled using a linearized model under the small-angle assumption for slow movements like hovering, this assumption is no longer valid during high-speed, aggressive maneuvers, where nonlinear effects become a major, non-negligible factor.

To overcome this nonlinear control problem, various approaches have been attempted over the past decades. Model-based controllers such as Backstepping, Feedback Linearization, and Nonlinear Model Predictive Control (NMPC) have shown high performance by explicitly utilizing the system's dynamic model. However, these methods have limitations, as they require accurate model parameters, or in the case of NMPC, a high computational cost to solve optimization problems in real-time. On the other hand, data-driven methods like Neural Networks and Reinforcement Learning have been proposed as alternatives to reduce model dependency, but they generally require a vast amount of training data and long training times.

In this work, we propose a hierarchical controller combining Sliding Mode Control (SMC) for both position and attitude loops with analytically derived Differential Flatness (DF) feedforward terms computed from the jerk and snap of the reference trajectory. Rather than numerically differentiating SMC-commanded desired angles — which amplifies noise during aggressive maneuvers — the feedforward terms are derived algebraically from the flat outputs, providing exact and clean angular rate and acceleration references. Lyapunov-based stability is proven independently for each control loop. The controller is validated in MATLAB/Simulink simulation on a C⁴ Lemniscate reference trajectory.

This research was conducted as an independent undergraduate research project (ME 4991) at Northeastern University under the supervision of Prof. Rifat Sipahi, undertaken prior to the author's formal coursework in nonlinear control theory. The work was driven by a self-directed literature survey, independent mathematical derivation, and hands-on implementation — with the goal of understanding, from first principles, what architectural choices determine controller performance on aggressive trajectories.

### 1.1 Research Objectives

1. Model the full 6DOF nonlinear dynamics of a quadrotor in a simulation environment.
2. Design and implement an SMC position and attitude controller with Lyapunov-proven stability.
3. Augment the attitude controller with a Differential Flatness feedforward term derived analytically from trajectory higher-order derivatives.
4. Validate the architecture through simulation on an aggressive C⁴ reference trajectory.
5. Design a subsequent stage replacing the SMC position loop with INDI as an ongoing extension.

---

## 2. System Modeling

### 2.1 Coordinate Frames and Notation

The quadrotor is modeled in an inertial North-East-Down (NED) frame with body-frame angular velocities $(p, q, r)$ and Euler angles $(\phi, \theta, \psi)$ representing roll, pitch, and yaw respectively.

### 2.2 Translational Dynamics

$$\dot{\mathbf{x}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = g\mathbf{i}_z + \mathbf{F}_{thr} + m^{-1}\mathbf{F}_{ext}$$

where $\mathbf{F}_{ext}$ captures aerodynamic drag and external disturbances.

### 2.3 Rotational Dynamics

The angular acceleration equations, derived from the Newton-Euler formulation for a cross-configuration quadrotor with aerodynamic drag terms $K_2$, are:

$$\dot{p} = \frac{1}{J_x}\left[\frac{\sqrt{2}}{2}(T_2+T_3-T_1-T_4)\cdot l - qr(J_z-J_y) - K_2 p|p|\right] + d$$

$$\dot{q} = \frac{1}{J_y}\left[\frac{\sqrt{2}}{2}(T_1+T_3-T_2-T_4)\cdot l - pr(J_x-J_z) - K_2 q|q|\right] + d$$

$$\dot{r} = \frac{1}{J_z}\left[(Q_1+Q_2-Q_3-Q_4)\cdot l - pq(J_y-J_x) - K_2 r|r|\right] + d$$

The kinematic relations between Euler angle rates and body angular rates are:

$$\dot{\phi} = p + q\tan\theta\sin\phi + r\tan\theta\cos\phi$$
$$\dot{\theta} = q\cos\phi - r\sin\phi$$
$$\dot{\psi} = q\frac{\sin\phi}{\cos\theta} + r\frac{\cos\phi}{\cos\theta}$$

### 2.4 Physical Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass $m$ | 0.8 | kg |
| Arm length $l$ | 0.165 | m |
| $J_x, J_y$ | 0.005 | kg·m² |
| $J_z$ | 0.009 | kg·m² |
| $T_{max}$ | 4 | N |
| $Q_{max}$ | 0.05 | N·m |

---

## 3. Differential Flatness

It has been proven that the quadrotor is a **differentially flat system**, where all states and control inputs can be expressed as functions of a set of *flat outputs* and their time derivatives, without inverting the complex dynamic equations (Mellinger & Kumar, 2011; Faessler et al., 2017). The flat outputs for a quadrotor are defined as the position of its center of mass and its yaw angle:

$\mathbf{r}_T = [x_T,\ y_T,\ z_T,\ \psi_T]^T$

This property is highly significant for controller design. Given a differentiable reference trajectory $\mathbf{r}_T$, all necessary state variables (e.g., attitude, angular rates) and control inputs (e.g., total thrust) required to follow that trajectory can be calculated algebraically. In this work, this property is leveraged to generate feedforward angular rate and acceleration references for the attitude controller, thereby improving tracking performance on aggressive trajectories.

### 3.1 Reference Attitude

Given the reference acceleration $\mathbf{a}_T = [\ddot{x}_T, \ddot{y}_T, \ddot{z}_T]^T$, the required thrust direction defines the desired body z-axis. The desired reference rotation matrix ${}^W R_{B,ref}$ is computed from the flat outputs (full expression in **Appendix A**). The desired roll and pitch angles are then extracted as:

$\phi_{ref} = \text{atan2}(s_{\psi_T}\ddot{x}_T - c_{\psi_T}\ddot{y}_T,\ \ddot{z}_T + g)$

$\theta_{ref} = \text{atan2}(s_{\psi_T}\ddot{y}_T + c_{\psi_T}\ddot{x}_T,\ \eta)$

where $\eta = \sqrt{(\ddot{z}_T+g)^2 + (s_{\psi_T}\ddot{x}_T - c_{\psi_T}\ddot{y}_T)^2}$, and $s_{\psi_T}, c_{\psi_T}$ denote $\sin\psi_T, \cos\psi_T$.

### 3.2 Feedforward Angular Rates from Jerk

The desired body angular rates are obtained from the jerk $\mathbf{j}_T = [j_{x,T}, j_{y,T}, j_{z,T}]^T$ of the reference trajectory. With total thrust $u_1 = m\sqrt{\ddot{x}_T^2 + \ddot{y}_T^2 + (\ddot{z}_T+g)^2}$:

$p_{ref} = -\frac{m}{u_1}\left({}^W R_B(1,2)\,j_{x,T} + {}^W R_B(2,2)\,j_{y,T} + {}^W R_B(3,2)\,j_{z,T}\right)$

$q_{ref} = \frac{m}{u_1}\left({}^W R_B(1,1)\,j_{x,T} + {}^W R_B(2,1)\,j_{y,T} + {}^W R_B(3,1)\,j_{z,T}\right)$

The Euler angle rate references $\dot{\phi}_{ref}$, $\dot{\theta}_{ref}$, $r_{ref}$ follow from the kinematic relations between body angular rates and Euler angle rates (see **Appendix A**).

### 3.3 Feedforward Angular Accelerations from Snap

The desired angular accelerations $\dot{p}_{ref}$, $\dot{q}_{ref}$ are derived from the snap $\mathbf{s}_T = \mathbf{r}_T^{(4)}$, and $\ddot{\phi}_{ref}$, $\ddot{\theta}_{ref}$, $\dot{r}_{ref}$ are obtained through auxiliary matrices $A$ and $B$ constructed from the flat outputs. Full derivations are provided in **Appendix A**.

### 3.4 Design Rationale

A central architectural decision in this work was to derive all feedforward terms **analytically** from the flat outputs, rather than by numerically differentiating the controller-commanded desired angles $\phi_{des}$ and $\theta_{des}$.

Numerical differentiation of desired angles introduces two problems. First, it amplifies high-frequency noise in the commanded signals, which is particularly harmful during aggressive maneuvers where desired angles change rapidly. Second, it introduces a one-step delay that degrades feedforward accuracy.

The analytic DF approach eliminates both issues, providing exact feedforward signals at the cost of requiring C⁴ (four-times continuously differentiable) reference trajectories — a constraint that is readily satisfied by minimum-snap polynomial trajectories. This trade-off was identified through early simulation experiments, where noise amplification in numerically differentiated feedforward terms produced visible degradation in attitude tracking.

---

## 4. Controller Design

### 4.1 Overall Architecture

The controller follows a cascaded structure:

```
Reference Trajectory (pos, vel, acc, jerk, snap)
            │
   [SMC Position Controller]
     → desired attitude (φ_des, θ_des)
            │
   [SMC Attitude Controller]
     + Disturbance Observer
     + DF Feedforward (jerk/snap → φ̇_ref, φ̈_ref, θ̇_ref, θ̈_ref)
            │
   [Motor Mixer] → rotor commands
```

### 4.2 SMC Position Controller

Position, velocity, and acceleration error terms:

$$\mathbf{e}_p = \mathbf{r} - \mathbf{r}_T, \quad \mathbf{e}_v = \dot{\mathbf{r}} - \dot{\mathbf{r}}_T$$

A sliding surface is defined independently for each translational axis. The control law computes a commanded acceleration $\mathbf{a}_c$, from which the desired body z-axis direction (i.e., the required attitude) is extracted.

### 4.3 SMC Attitude Controller

#### 4.3.1 Roll Controller

Error terms incorporating DF feedforward:

$$e_\phi = \phi - \phi_{des}, \quad \dot{e}_\phi = \dot{\phi} - \dot{\phi}_{ref}, \quad \ddot{e}_\phi = \ddot{\phi} - \ddot{\phi}_{ref}$$

Sliding surface:

$$s_\phi = a_3 e_\phi + \dot{e}_\phi$$

Control law:

$$\tau_R = k_5 \cdot \text{sat}\!\left(\frac{s_\phi}{\epsilon}\right) + k_6 \dot{e}_\phi + (\text{cross-coupling compensation}) + \frac{J_x}{2\sqrt{2}\,l\,T_{max}}(f_\phi - \ddot{\phi}_{ref})$$

**Stability proof.** With Lyapunov candidate $V(s_\phi) = \frac{1}{2}s_\phi^2 \geq 0$, substituting the control law yields:

$$\dot{V}(s_\phi) = -\frac{2\sqrt{2}\,l\,T_{max}}{J_x} k_5 \left(s_\phi \cdot \text{sat}\!\left(\frac{s_\phi}{\epsilon}\right)\right) \leq 0$$

under the gain condition $a_3 = \frac{2\sqrt{2}\,l\,T_{max}}{J_x} k_6$, with $k_5 > 0$.

#### 4.3.2 Pitch and Yaw Controllers

Structurally identical derivations apply to pitch and yaw, with appropriate substitution of inertia terms ($J_y$, $J_z$) and control coefficients. Stability is proven independently for each axis by the same Lyapunov argument. The saturation function $\text{sat}(\cdot)$ replaces the discontinuous $\text{sign}(\cdot)$ function throughout to suppress chattering.

### 4.4 Disturbance Observer

A first-order nonlinear disturbance observer is applied on each rotational axis to estimate and compensate for unknown disturbances $d$:

$$d_{ob} = z_{in} + k_{ob}\,\omega$$

$$\dot{z}_{in} = -k_{ob}\,d_{ob} - k_{ob}\cdot f(\tau, \omega)$$

The estimated disturbance is fed back to augment the control torque:

$$\tau_{new} = \tau_{orig} + d_{ob} \cdot \frac{J}{c_{control}}$$

---

## 5. Implementation

The controller was implemented in MATLAB/Simulink (R2024b) as a 6DOF nonlinear simulation. Key implementation details:

- **Solver**: Fixed-step `ode4` (Runge-Kutta 4th order), step size 1 ms
- **Trajectory**: C⁴ Lemniscate-type trajectory defined symbolically; derivatives up to 4th order computed automatically via MATLAB Symbolic Math Toolbox
- **Gains**: SMC attitude gains ($a_3, k_5, k_6$ for roll/pitch; $a_4, k_7, k_8$ for yaw) tuned via Particle Swarm Optimization (PSO) following the approach of Jing et al. (2022)
- **Code**: Publicly available at [https://github.com/HojunKim00920/6DOF-quadrotor-SMC-controller](https://github.com/HojunKim00920/6DOF-quadrotor-SMC-controller)

---

## 6. Simulation Results

Simulation was conducted on a Lemniscate-type C⁴ reference trajectory without external disturbances. An initial transient of approximately 5 seconds reflects the controller starting from rest at the origin.

**Figure 1** shows the 3D trajectory tracking result. The quadrotor successfully converges to and tracks the reference trajectory.

**Figure 2** shows position tracking in X, Y, and Z axes. After the initial transient, tracking error converges to near zero across all axes.

**Figure 3** shows the position tracking error over time, confirming stable convergence.

*(Figures available at the GitHub repository linked above.)*

---

## 7. Discussion

### 7.1 Key Design Choices and Lessons

Two architectural decisions proved particularly consequential:

**Analytic vs. numeric feedforward.** Computing $\dot{\phi}_{ref}, \ddot{\phi}_{ref}, \dot{\theta}_{ref}, \ddot{\theta}_{ref}$ analytically via Differential Flatness — rather than by numerically differentiating SMC-commanded desired angles — resulted in noticeably cleaner feedforward signals. This was not anticipated at the outset and emerged from observing noise amplification in early simulations.

**Separation of position and attitude loops.** The cascaded structure, in which the position controller outputs a desired attitude rather than direct torque commands, simplifies stability analysis and allows the attitude loop to be tuned independently. This separation also makes it straightforward to substitute the position controller in the next stage.

### 7.2 Limitations

| Assumption | Implication |
|---|---|
| No aerodynamic drag ($K_2 = 0$) | Performance under real flight conditions will degrade |
| Ideal actuators | Motor dynamics and latency not captured |
| Exact model parameters | Robustness to parameter uncertainty not yet evaluated |
| No sensor noise | Feedforward terms assume clean state measurements |

---

## 8. Ongoing Work and Future Directions

A second stage of the research replaces the SMC position controller with an **Incremental Nonlinear Dynamic Inversion (INDI)** outer loop. INDI's sensor-based incremental control law implicitly compensates for external forces without requiring a disturbance observer, and has been shown to improve robustness against model uncertainty and aerodynamic disturbances in the literature.

The INDI outer loop has been designed and implemented. The inner-loop SMC attitude controller and DF feedforward structure remain identical across both stages, allowing a controlled comparison of the two position controller approaches. Gain optimization for the INDI stage is ongoing.

Beyond this, the research trajectory points toward a deeper question surfaced during the project: the persistent residual between DF-predicted and SMC-observed control inputs — which grows with trajectory frequency — suggests that control effort in input space may carry information about system uncertainty beyond what tracking error in state space already reveals. This motivates future exploration of uncertainty-aware control formulations, including connections to Bayesian estimation and stochastic optimal control.

---

## References

[1] Jing, Y., Wang, X., Heredia-Juesas, J., Fortner, C., Giacomo, C., Sipahi, R., & Martinez-Lorenzo, J. (2022). *PX4 Simulation Results of a Quadcopter with a Disturbance-Observer-Based and PSO-Optimized Sliding Mode Surface Controller.*

[2] Mellinger, D., & Kumar, V. (2011). *Minimum snap trajectory generation and control for quadrotors.* ICRA.

[3] Tal, E., & Karaman, S. (2020). *Accurate tracking of aggressive quadrotor trajectories using incremental nonlinear dynamic inversion and differential flatness.* IEEE Transactions on Control Systems Technology.

[4] Faessler, M., Franchi, A., & Scaramuzza, D. (2017). *Differential flatness of quadrotor dynamics subject to rotor drag for accurate tracking of high-speed trajectories.* IEEE RA-L.

[5] Wang, X., et al. (2019). *Stability analysis for incremental nonlinear dynamic inversion control.* Journal of Guidance, Control, and Dynamics.

---

## Appendix A: Full Differential Flatness Feedforward Derivations

*(To be populated with equations from Section 4 of the technical derivation document.)*
