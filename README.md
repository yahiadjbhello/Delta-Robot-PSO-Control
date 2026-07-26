# Delta Robot PSO Control

<div align="center">

# 🤖 Optimal Tracking Control of a 3-DOF Delta Parallel Robot
### PSO-Tuned Backstepping and Sliding Mode Controllers via Simscape Co-Simulation

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b%2B-orange?logo=mathworks&logoColor=white)](https://www.mathworks.com/)
[![Simulink](https://img.shields.io/badge/Simulink-Multibody-blue?logo=mathworks)](https://www.mathworks.com/products/simscape-multibody.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Windows-lightgrey?logo=windows)](https://www.mathworks.com/)

</div>

---

## 📋 Table of Contents

- [Abstract](#-abstract)
- [Robot Specifications](#-robot-specifications)
- [Project Structure](#-project-structure)
- [Controllers Implemented](#-controllers-implemented)
- [Kinematics & Dynamics](#-kinematics--dynamics)
- [Trajectory Planning](#-trajectory-planning)
- [Requirements](#-requirements)
- [Getting Started](#-getting-started)
- [Simulation Models](#-simulation-models)
- [Results](#-results)
- [Citation](#-citation)

---

## 📄 Abstract

This project presents the design, modelling, and optimal tracking control of a **3-DOF Delta Parallel Robot** using a **MATLAB/Simulink + Simscape Multibody co-simulation** framework. The 3D CAD assembly (SolidWorks → STEP) is imported directly into Simscape Multibody to provide a high-fidelity, physics-based plant model.

Two nonlinear controllers are designed and compared:

1. **PSO-Tuned Backstepping Controller** — A Lyapunov-based recursive design that guarantees asymptotic stability; gains are optimally tuned using **Particle Swarm Optimization (PSO)**.
2. **PSO-Tuned PD + Sliding Mode Controller (SMC)** — Combines a proportional-derivative pre-filter with a robust sliding surface; chattering is attenuated via PSO-optimized parameters.

Both controllers track quintic polynomial reference trajectories (circular and square paths) in the robot's **Cartesian task space**. Performance is evaluated in terms of tracking error, control effort, and robustness.

---

## 🔩 Robot Specifications

| Parameter | Value |
|---|---|
| Architecture | 3-DOF Delta Parallel Robot |
| Base triangle side (`sb`) | 412.68 mm |
| Moving platform triangle side (`sp`) | 86 mm |
| Upper arm length (`L`) | 150 mm |
| Lower arm length (`l`) | 300 mm |
| Number of actuators | 3 × Revolute joints (motors) |
| Passive joints | 12 × Spherical joints (ball joints) |
| Total mass (estimated) | ~1.5 kg |
| Workspace | Cylindrical, centred below base |

---

## 📁 Project Structure

```
Delta-Robot-PSO-Control/
│
├── README.md                        ← You are here
├── LICENSE                          ← MIT License
├── .gitignore                       ← Excludes caches, autosaves, large MAT files
│
├── models/                          ← Simulink / Simscape models
│   ├── backstepping25072026.slx          ← PSO-tuned Backstepping controller
│   ├── DELTA_ROBOT_X_PDSMC25072026.slx  ← PSO-tuned PD + SMC controller
│   ├── PDplusSMC.slx                    ← PD + SMC (baseline)
│   ├── PD_SMC_control.slx               ← Standalone control subsystem block
│   ├── SlidingModeControllBasedOnlLinereasition.slx ← SMC via linearization
│   ├── inversegeo.slx                   ← Inverse geometry verification model
│   └── DELTA_ROBOT_XD_original.slx      ← Original imported Simscape model
│
├── kinematics/                      ← Kinematic model functions
│   ├── FGM.m                        ← Forward Geometry Model
│   ├── IGM.m                        ← Inverse Geometry Model (analytical)
│   ├── rot_0Z.m                     ← Z-axis rotation matrix R(θ)
│   ├── dir_kin.m                    ← Direct kinematics script
│   └── inv_kin.m                    ← Inverse kinematics script
│
├── dynamics/                        ← Dynamic model
│   └── ineriamatrix.m               ← Mass matrix M(q), Jacobian J, gravity G
│
├── trajectory/                      ← Reference trajectory generation
│   ├── quintic.m                    ← Quintic polynomial planner (position, velocity, acceleration)
│   ├── trajectories.m               ← Trajectory generation (circle / square)
│   └── trajectorieslitim.m          ← Trajectory with workspace limits
│
├── simscape/                        ← Simscape Multibody import data
│   ├── DELTA_ROBOT_X_DataFile.m     ← Rigid body transforms, inertia tensors, joint states
│   └── DELTA_ROBOT_X.xml            ← SolidWorks-to-Simscape XML export
│
├── cad/                             ← CAD geometry files
│   └── meshes/                      ← STEP files for each robot link
│       ├── FIXED-BASE_Défaut_sldprt.STEP
│       ├── Mouving-Platform_Défaut_sldprt.STEP
│       ├── UPPER-ARME_Default_sldprt.STEP
│       ├── LOWER-ARME_Défaut_sldprt.STEP
│       ├── MOTOR_Défaut_sldprt.STEP
│       ├── H1_Défaut_sldprt.STEP
│       ├── H2_Défaut_sldprt.STEP
│       ├── Spherical-Joint_Défaut_sldprt.STEP
│       ├── END-EFFECTOR_Défaut_sldprt.STEP
│       └── O_Défaut_sldprt.STEP
│
└── scripts/                         ← Entry-point and utility scripts
    ├── run_simulation.m             ← ⭐ Main simulation entry-point
    ├── density.m                    ← Robot physical & material parameters
    ├── dplot.m                      ← 3D Delta robot visualizer
    └── aubic.m                      ← Cubic polynomial utility
```

---

## 🎮 Controllers Implemented

### 1. PSO-Tuned Backstepping Controller
**File:** `models/backstepping25072026.slx`

A recursive Lyapunov-based nonlinear controller designed for the robot's joint-space dynamics. The control law is derived by successively stabilising each subsystem layer (backstep). **Particle Swarm Optimization** is used to tune the control gains, minimising the Integral of Squared Error (ISE) across all three joints.

```
z1 = q - qd           (tracking error)
z2 = dq - α(z1, qd)   (virtual input error)
τ = M(q)[α̇ - K2·z2 - z1] + C(q,dq)·dq + G(q)
```

### 2. PSO-Tuned PD + Sliding Mode Controller
**File:** `models/DELTA_ROBOT_X_PDSMC25072026.slx`

Combines a PD feedforward term with a sliding mode discontinuous term. The sliding surface is defined as:
```
s = ė + λ·e       (e = q - qd)
τ = τ_PD + K·sign(s)
```
PSO optimises `{Kp, Kd, λ, K}` jointly to minimise tracking error and chattering.

### 3. Baseline PD + SMC (No PSO)
**File:** `models/PDplusSMC.slx` — manually tuned reference implementation.

### 4. SMC via Linearization
**File:** `models/SlidingModeControllBasedOnlLinereasition.slx` — SMC designed on a linearised plant model.

---

## 📐 Kinematics & Dynamics

### Forward Geometry Model (FGM)
`kinematics/FGM.m` solves the position of the end-effector given three joint angles:

```matlab
P = FGM(theta1, theta2, theta3)   % Returns [x; y; z] in metres
```

Uses the **intersection of three spheres** approach (geometric solution) with Z-axis rotational symmetry.

### Inverse Geometry Model (IGM)
`kinematics/IGM.m` computes the three joint angles for a desired Cartesian position:

```matlab
q = IGM(x, y, z)   % Returns [theta1; theta2; theta3] in radians
```

Analytical closed-form solution using the half-angle tangent substitution.

### Dynamic Model
`dynamics/ineriamatrix.m` derives (symbolically) the **mass matrix M(q)**, **Coriolis/centrifugal matrix C(q,dq)**, and **gravity vector G(q)** using the Lagrangian formulation and the analytical Jacobian J(q):

```
M(q) = Ib + m_total · Jᵀ·J
G(q) = Jᵀ · m_eff · g_vec − G_bg
τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)
```

---

## 📈 Trajectory Planning

`trajectory/quintic.m` implements a **quintic (5th-order) polynomial** trajectory planner that ensures smooth position, velocity, and acceleration profiles with zero boundary conditions:

```matlab
[q, dq, ddq] = quintic(q0, qf, dq0, dqf, ddq0, ddqf, t0, tf, t)
```

Both **circular** and **square** task-space paths are supported, converted to joint space via IGM at each timestep.

---

## ⚙️ Requirements

| Software | Version | Notes |
|---|---|---|
| MATLAB | R2023b or later | Tested on R2023b |
| Simulink | Included with MATLAB | Core simulation engine |
| Simscape Multibody | Toolbox | For physics-based plant model |
| Optimization Toolbox | Optional | For re-running PSO tuning |

---

## 🚀 Getting Started

### 1. Clone the Repository
```bash
git clone https://github.com/yahiadjbhello/Delta-Robot-PSO-Control.git
cd Delta-Robot-PSO-Control
```

### 2. Open MATLAB
Set the working directory to the repository root.

### 3. Run the Main Simulation
```matlab
>> run_simulation
```

Or open `scripts/run_simulation.m`, select your controller and trajectory at the top of the file, and press **Run**.

### 4. Open a Specific Model
```matlab
>> open('models/backstepping25072026.slx')
>> open('models/DELTA_ROBOT_X_PDSMC25072026.slx')
```

> **Note:** The Simscape data file `simscape/DELTA_ROBOT_X_DataFile.m` must be run (or called) before opening the main Simscape models so that the `smiData` structure is populated in the workspace.

---

## 📦 Simulation Models

| Model File | Controller | PSO Tuned | Description |
|---|---|---|---|
| `backstepping25072026.slx` | Backstepping | ✅ Yes | Primary backstepping model |
| `DELTA_ROBOT_X_PDSMC25072026.slx` | PD + SMC | ✅ Yes | Primary PD+SMC model |
| `PDplusSMC.slx` | PD + SMC | ❌ No | Baseline PD+SMC |
| `PD_SMC_control.slx` | PD + SMC | ❌ No | Standalone control block |
| `SlidingModeControllBasedOnlLinereasition.slx` | SMC | ❌ No | Linearization-based SMC |
| `inversegeo.slx` | — | — | Inverse geometry verification |

---

## 📊 Results

Performance metrics are evaluated over a **12-second circular trajectory** (R = 0.2 m, ω = 2 rad/s):

| Metric | Backstepping (PSO) | PD+SMC (PSO) |
|---|---|---|
| Max tracking error | — | — |
| RMS tracking error | — | — |
| Max control torque | — | — |
| Settling time | — | — |

> ⚠️ *Fill in your results after running the simulations. Figures and plots can be saved to `docs/images/` and embedded here.*

---

## 📖 Citation

If you use this work in your research, please cite:

```bibtex
@misc{yahia2026delta,
  author       = {Yahia Djebali, Menad Abdelouaheb, Attarsia Taki Eddine},
  title        = {Optimal Tracking Control of a 3-DOF Delta Parallel Robot
                  Using PSO-Tuned Backstepping and Sliding Mode Controllers
                  via Simscape Co-Simulation},
  year         = {2026},
  howpublished = {\url{https://github.com/yahiadjbhello/Delta-Robot-PSO-Control}},
}
```

---

## 📝 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

---

<div align="center">
Made with ❤️ using MATLAB, Simulink & Simscape Multibody
</div>
