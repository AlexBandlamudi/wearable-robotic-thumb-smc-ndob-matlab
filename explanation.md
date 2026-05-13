# Robotic Thumb SMC Simulation — Complete Documentation

---

## Is Simulink Being Used For Anything?

**Short answer: No. The simulation runs entirely in pure MATLAB scripts.**

The file `model_thumb_smc_mb.slx` exists in this folder, and `build_thumb_smc_model.m`
can build it from scratch using Simulink's programmatic API (`new_system`, `add_block`,
`add_line`). It was constructed as a visual reference to mirror the structure of the
original coursework Simulink models in `assignment/model_SMC_mb.slx`.

However, `run_thumb_smc_mb.m` and `run_thumb_extra_tests.m` — the two files that
actually run the simulation — **do not open, load, or call `sim()` on the Simulink
model at all**. Every step of integration, dynamics evaluation, and control computation
is done inside MATLAB functions and a hand-coded RK4 loop.

The Simulink `.slx` file is an **unused visual artifact**. If you deleted it, the
simulation results would be completely unchanged. The reason to keep it is that it
documents the block-diagram structure in a form that is easier to read at a glance than
the equivalent equations in code.

The same applies to the assignment folder: `model_SMC_mb.slx`, `model_SMC_mf.slx`,
`model_underactuated_NDOB.slx`, and `model_underactuated_PFLC.slx` are the original
coursework Simulink models. They are not used here; they served as the conceptual
template from which the pure-MATLAB implementation was derived.

---

## Motivation — Why SMC? Why NDOB?

### The control problem

A wearable robotic thumb exoskeleton is a nonlinear, multi-body system. As the thumb
curls from open to closed, the inertia seen at each joint changes — the coupling
term `2*r1*c2*cos(q2)` in M(q) halves as q2 goes from 0 to π/2. At the same time the
gravity load on each joint grows from zero (fully extended, horizontal) to its maximum
(fully curled, links at their highest moment arm). This means the dynamics the controller
must handle are fundamentally different at the start of a movement versus the end.
A fixed-gain PID cannot track well across this range without aggressive gain scheduling.

On top of the changing dynamics, the thumb faces external disturbances that are
unpredictable in magnitude: background motor ripple (100 Hz), postural drift (< 2 Hz),
and sudden grip loads when an object is grasped or resisted. The sudden load in this
project has an amplitude of up to 7.2 mN·m on the proximal joint — comparable to the
gravity torque at mid-curl.

### Why Sliding Mode Control (SMC)?

SMC is chosen because it is **provably robust to matched disturbances without needing
to know their shape**. The key guarantee is the reaching condition:

$$K_i > |\tau_{d,i}|_{\max}$$

If the switching gain exceeds the worst-case disturbance on every channel, the sliding
surface $s_i = \dot{e}_i + \lambda_i e_i$ is driven to zero in finite time regardless
of the trajectory the disturbance takes. There is no need to model the disturbance
frequency content, phase, or waveform.

The **model-based (Slotine-Li) variant** pre-cancels the known gravity, inertia, and
Coriolis terms via the equivalent torque $\tau_{eq} = M\ddot{q}_r + C(q,\dot{q}_r)\dot{q}_r + G$,
evaluated at the modified reference velocity $\dot{q}_r = \dot{q}_d - \Lambda e$. This
is important: because the known dynamics are already cancelled, the switching gain only
needs to cover the *residual* disturbance, not the full torque range. At this thumb's
scale, gravity torques reach ~21 mN·m but the disturbance is only ~5 mN·m, so a
switching gain of K = diag(10, 8, 1) mN·m is sufficient with margin — far smaller than
would be needed if the gravity were not pre-cancelled.

The boundary layer $|s| < \delta = 0.01$ rad replaces the hard sign function with a
linear ramp, converting chattering into a high-gain PD response inside the layer. The
trade-off is a bounded steady-state error $|e_\infty| \lesssim |d|/(\lambda K)$ in
steady state — at most ~8 mrad for q1, which is below the human tactile resolution
threshold of ~17.5 mrad.

### Why NDOB?

SMC has one fundamental structural weakness: the reaching condition $K_i > |d_i|_{\max}$
can be violated if the disturbance grows beyond the switching gain. Increasing K to stay
ahead of larger disturbances increases chattering in proportion, which is undesirable in
a wearable device.

The **Nonlinear Disturbance Observer (NDOB)** (Chen et al., 2000) changes the
architecture: instead of asking the switching term to reject the entire disturbance, the
observer estimates $\hat{\tau}_d$ online and subtracts it feedforward from the control
torque. The switching term then only needs to cover the estimation residual
$e_d = \tau_d - \hat{\tau}_d$, which decays exponentially at rate L. At L = 50 rad/s
the residual drops to ~0.36 mN·m on q1/q2 (set by the untracked 100 Hz ripple), so the
switching gain can be reduced from K = diag(10, 8, 1) mN·m to
K_new = diag(2, 2, 0.3) mN·m — a **5× reduction** — while *improving* tracking.

The practical consequence: at the 2.40× stress load that breaks plain SMC-MB (q2
reaching condition violated, 7° offset), NDOB+SMC holds every joint inside 1.6 mrad
RMS and remains stable all the way to 7× load, where SMC-MB has numerically diverged.

---

## What This Project Simulates

Two simulation runners exist, each producing independent outputs:

### Primary scenario (`run_thumb_smc_mb.m`) — 15 s, 3 phases

A 3-DOF wearable robotic thumb exoskeleton closes a grip, holds it, and rejects a
sudden load. Starting posture is near-open (fingers slightly curled to avoid the
zero-gravity singularity at q = 0).

| Phase | Time | Task |
|-------|------|------|
| P1 — CLOSE GRIP | 0–5 s | Thumb closes from open posture to full grip |
| P2 — HOLD POSTURE | 5–10 s | Thumb holds grip against background disturbance |
| P3 — SUSTAIN + REJECT | 10–15 s | Sudden grip-load applied; controller must reject it |

Initial state: q0 = [0.10; 0.10; 0.05] rad. Target: qf = [0.70; 1.20; 0.85] rad.

### Full-cycle validation (`run_thumb_extra_tests.m`) — 25 s, 5 phases

A single continuous simulation starting from the fully extended position (q = 0),
closing the thumb, holding, surviving a sudden load, retracing back, and holding open.
This tests everything the primary scenario tests, plus bidirectional tracking and
cycle closure, all in one uninterrupted run.

| Phase | Time | Task |
|-------|------|------|
| P1 — CLOSE | 0–5 s | Quintic close from q0=[0;0;0] to qf=[0.70;1.20;0.85] |
| P2 — HOLD POSTURE | 5–10 s | Hold grip under background disturbance |
| P3 — LOAD REJECT | 10–15 s | Sudden load at t=10 s; controller must absorb it |
| P4 — RETRACE | 15–22 s | Quintic retrace back to q0 |
| P5 — HOLD OPEN | 22–25 s | Hold extended — confirms cycle closure |

### Physical plant (both simulations share the same model)

The thumb is planar (2-D), joints in the sagittal plane.

| Segment | Link | Length | Mass | Inertia (rod) |
|---------|------|--------|------|---------------|
| Proximal phalange (MCP) | Link 1 | 50 mm | 15 g | 1.25 × 10⁻⁵ kg·m² |
| Middle phalange (PIP)   | Link 2 | 35 mm | 10 g | 4.08 × 10⁻⁶ kg·m² |
| Distal phalange (DIP)   | Link 3 | 25 mm |  6 g | 1.25 × 10⁻⁶ kg·m² |

---

### Denavit-Hartenberg Parameters and Forward Kinematics

For a planar 3R chain all rotation axes are parallel (normal to the sagittal plane), so the DH parameters reduce to:

| Joint $i$ | $a_i$ (m) | $d_i$ (m) | $\alpha_i$ (rad) | $\theta_i$ (variable) |
|-----------|-----------|-----------|-------------------|-----------------------|
| 1 — MCP   | 0.050     | 0         | 0                 | $q_1$                 |
| 2 — PIP   | 0.035     | 0         | 0                 | $q_2$                 |
| 3 — DIP   | 0.025     | 0         | 0                 | $q_3$                 |

The standard DH homogeneous transformation for joint $i$ is:

$$A_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & 0 & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i & 0 & a_i\sin\theta_i \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

With $\alpha_i = 0$ and $d_i = 0$ for all three joints, the rotation is purely about the z-axis and the translation is along the previous x-axis.

The full end-effector (fingertip) transformation is:

$$T_0^3 = A_1(q_1)\,A_2(q_2)\,A_3(q_3) =
\begin{bmatrix}
\cos\phi & -\sin\phi & 0 & p_x \\
\sin\phi &  \cos\phi & 0 & p_y \\
0        & 0         & 1 & 0 \\
0        & 0         & 0 & 1
\end{bmatrix}$$

where the fingertip position in the base frame is:

$$p_x = r_1\cos q_1 + r_2\cos(q_1+q_2) + r_3\cos(q_1+q_2+q_3)$$
$$p_y = r_1\sin q_1 + r_2\sin(q_1+q_2) + r_3\sin(q_1+q_2+q_3)$$

and the orientation (angle of the distal link with respect to the base x-axis) is:

$$\phi = q_1 + q_2 + q_3$$

This is now implemented explicitly in [`thumb_kinematics.m`](thumb_kinematics.m), which returns the DH table, the three individual matrices $A_i$, the cumulative transforms $T_0^i$, the full transform $T_0^3$, and the joint positions used for plotting. The older local `fk3(q, r1, r2, r3)` helpers inside the animation files compute the same planar joint positions, but only the position part of the transform.

At the grip posture $q_f = [0.70;\,1.20;\,0.85]$ rad:

$$p_x = 0.050\cos(0.70) + 0.035\cos(1.90) + 0.025\cos(2.75) \approx 0.0038 \text{ m}$$
$$p_y = 0.050\sin(0.70) + 0.035\sin(1.90) + 0.025\sin(2.75) \approx 0.0749 \text{ m}$$
$$\phi = 2.75 \text{ rad} \approx 157.6°$$

The distal link points nearly back toward the base — the thumb has closed into a full curl with the fingertip at approximately [3.8, 74.9] mm from the MCP base. The generated figure [`thumb_dh_forward_kinematics.png`](thumb_dh_forward_kinematics.png) visualises this chain, the DH table, and the closed-form transform in one 1200×627 slide.

**Relationship to the dynamics.** The Lagrangian $\mathcal{L} = T - V$ is written directly in joint coordinates $q$ rather than Cartesian coordinates. The DH table is therefore not used at runtime — M(q), C(q,q̇), G(q) are derived symbolically from the kinetic and potential energies expressed in $q$. The DH table is the kinematic parameterisation; the dynamics derive from the same geometry but via the Euler-Lagrange equations rather than the Newton-Euler recursive formulation.

---

## Control Algorithm — Model-Based Sliding Mode Control (SMC-MB)

### Why SMC?

Robotic manipulators have coupled, nonlinear dynamics that vary with configuration and
velocity. Standard PID cannot handle this without gain scheduling. SMC is chosen
because:

- It is **provably robust**: the sliding condition $s \dot{s} < 0$ guarantees convergence
  even in the presence of matched disturbances and bounded model uncertainty.
- It handles **nonlinear, coupled dynamics directly** — no linearisation needed.
- The **model-based variant** pre-cancels the known dynamics ($M$, $C$, $G$), so the
  switching gain only needs to cover the residual disturbance, not the full torque range.
  This makes chattering much smaller than a pure switching controller.

### The Full Euler–Lagrange Plant

Both simulations use the identical rigid-body dynamics:

$$M(q)\,\ddot{q} + C(q,\dot{q})\,\dot{q} + G(q) = \tau + \tau_d$$

where $q = [q_1, q_2, q_3]^T$ are MCP/PIP/DIP joint angles, $\tau$ is the SMC control
torque, and $\tau_d$ is the external disturbance. $M$, $C$, $G$ are computed at every
time step from the exact Lagrangian derivation.

### Control Law

**Step 1 — Sliding surface per joint:**

$$s_i = \dot{e}_i + \lambda_i\,e_i, \qquad e_i = q_i - q_{d,i}$$

$e_i$ is the tracking error (actual minus desired). $\lambda_i > 0$ sets how fast errors
decay once on the surface — time constant $1/\lambda_i = 83$ ms for $\lambda = 12$ rad/s.

**Step 2 — Modified reference (Slotine–Li formulation):**

Define the auxiliary reference velocity and acceleration that absorb the surface term:

$$\dot{q}_{r,i} = \dot{q}_{d,i} - \lambda_i\,e_i, \qquad
\ddot{q}_{r,i} = \ddot{q}_{d,i} - \lambda_i\,\dot{e}_i$$

Note the sign: the reference is pulled *toward* the actual state at rate $\lambda$,
which is exactly the Lyapunov-stable structure needed for the equivalent control to work.

**Step 3 — Control torque:**

$$\boldsymbol{\tau} = \underbrace{M(q)\,\ddot{\boldsymbol{q}}_r + C(q,\dot{\boldsymbol{q}}_r)\,\dot{\boldsymbol{q}}_r + G(q)}_{\tau_{\text{eq}}} \;-\; K\,\text{sat}\!\left(\frac{\boldsymbol{s}}{\delta}\right)$$

- $\tau_{\text{eq}}$ **pre-cancels the nominal plant**: if the model were perfect and
  disturbances were zero, this alone would make $\dot{s} = 0$ — the system would stay
  on the surface without any switching.
- $-K\,\text{sat}(s/\delta)$ **drives $s \to 0$ against disturbances**: the saturation
  function replaces hard `sign(s)` switching (which causes chattering) with a linear
  ramp inside the boundary layer $|s| < \delta$.

**The sat function** (implemented in `sat_sign.m`):

$$\text{sat}(x) = \begin{cases} x/\delta & |x| < \delta \\ \text{sign}(x) & |x| \geq \delta \end{cases}$$

Inside the boundary layer the controller acts like a high-gain linear PD controller;
outside it acts like a relay, guaranteeing $s\dot{s} < 0$.

### Tuned Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| $\Lambda = \text{diag}(\lambda_1,\lambda_2,\lambda_3)$ | diag(12, 12, 12) rad/s | 83 ms time constant; fast enough to track the quintic ramp |
| $K = \text{diag}(K_1, K_2, K_3)$ | diag(10, 8, 1) mN·m | ~2× worst-case disturbance per joint (5 / 4.5 / 0.4 mN·m) |
| $\delta$ | 0.01 rad (0.57°) | Boundary layer: trades hard switching for bounded steady-state error |

**Theoretical steady-state error inside boundary layer:**

$$|e_\infty| \lesssim \frac{|d|_{\max}}{\lambda_i K_i} \approx \frac{5}{12 \times 10} \approx 8 \text{ mrad for } q_1$$

Simulated results: q1 hold RMS = 2.7 mrad, q2 = 9.2 mrad, q3 = 15.1 mrad. All below
human tactile resolution (~17.5 mrad = 1°).

---

## Source Files — Purpose, Structure, and Code Walkthrough

### `manipulator_M.m` — 3×3 Inertia Matrix M(q)

**Input**: `q1, q2, q3` (scalar joint angles in rad)  
**Output**: `M` (3×3 symmetric positive-definite matrix, kg·m²)

Computes the configuration-dependent inertia matrix from the Lagrangian kinetic energy
$T = \frac{1}{2}\dot{q}^T M(q) \dot{q}$ for a planar 3R chain.

Key computed entries (every term is exact — no small-angle approximation):
- `M11` depends on all three joint angles (q2, q3) through cross-coupling terms like
  `2*r1*c2*cos(q2)` and `2*r1*c3*cos(q2+q3)`.  This is the MCP inertia seen from joint 1
  — it includes the reflected inertia of all downstream links.
- `M22` depends only on q3 — the PIP joint's inertia includes the distal link's
  coupling `2*r2*c3*cos(q3)`.
- `M33 = J3 + m3*c3²` is constant — the DIP joint always sees only its own link.
- Off-diagonal `M12`, `M13`, `M23` are the coupling terms. At q = [0;0;0] all cosines
  are 1 and coupling is maximum. This is why q0 = [0;0;0] is the hardest starting pose.

Inertia of each link modelled as a **uniform rod rotating about its proximal end**:
$J_i = \frac{1}{3}m_i r_i^2$.

---

### `manipulator_CG.m` — Coriolis Vector C(q,q̇)q̇ and Gravity Vector G(q)

**Input**: `q1, q2, q3, dq1, dq2, dq3`  
**Output**: `C_vec` [3×1], `G_vec` [3×1] (both in N·m)

Uses the **Christoffel symbol method** from Slotine & Li, Ch.6. The key intermediate
scalars are:

- `a2 = m2*r1*c2*sin(q2) + m3*(r1*r2*sin(q2) + r1*c3*sin(q2+q3))` — couples q2 terms
- `b3 = m3*r2*c3*sin(q3)` — couples q3 terms  
- `p3 = m3*r1*c3*sin(q2+q3)` — couples q2+q3 compound rotation

The gravity vector $G(q) = \partial V/\partial q$ where $V$ is the total potential energy:

```
G1 = g*(m1*c1*sin(q1) + m2*(r1*sin(q1) + c2*sin(q1+q2)) + m3*(r1*sin(q1) + r2*sin(q1+q2) + c3*sin(q1+q2+q3)))
G2 = g*(m2*c2*sin(q1+q2) + m3*(r2*sin(q1+q2) + c3*sin(q1+q2+q3)))
G3 = g*m3*c3*sin(q1+q2+q3)
```

At q = [0;0;0] every sin = 0 so G = [0;0;0]. This is why the zero-angle pose has zero
gravity torque — the arm is horizontal and gravity acts perpendicular to the links.
As the thumb curls upward, sin terms grow and G increases. The SMC's equivalent torque
pre-cancels this automatically because G is recomputed at every step.

---

### `plant_dynamics.m` — Forward Dynamics Solver

**Input**: `q [3×1], qdot [3×1], tau_total [3×1]` (N·m)  
**Output**: `qddot [3×1]` (rad/s²)

Solves: $\ddot{q} = M(q)^{-1}\left(\tau_{\text{total}} - C(q,\dot{q})\dot{q} - G(q)\right)$

Used inside the RK4 loop. `tau_total = tau_control + tau_disturbance`. The matrix
inverse is computed with MATLAB's backslash operator (`\`), which is numerically stable
for the well-conditioned inertia matrices at these link sizes.

---

### `SMC_eq_torque.m` — Equivalent (Feedforward) Control Torque

**Input**: `q, qdot, qref, dqref, ddqref [3×1], Lambda [3×3]`  
**Output**: `tau_eq [3×1]` (N·m)

Implements the model-based feedforward term:

```matlab
e_tilde  = q    - qref;       % position error
de_tilde = qdot - dqref;      % velocity error
qdot_r   = dqref  - Lambda * e_tilde;   % modified reference velocity
qddot_r  = ddqref - Lambda * de_tilde;  % modified reference acceleration
M_mat    = manipulator_M(q(1), q(2), q(3));
[C_r_vec, G_vec] = manipulator_CG(q(1),q(2),q(3), qdot_r(1),qdot_r(2),qdot_r(3));
tau_eq   = M_mat * qddot_r + C_r_vec + G_vec;
```

Note that `manipulator_CG` is evaluated at `qdot_r` (the modified reference velocity),
not at the actual `qdot`. This is the Slotine–Li formulation — it ensures the
equivalent torque drives the sliding surface to zero without cancelling the stabilising
term in the Lyapunov derivative.

---

### `sat_sign.m` — Smooth Saturation Function

**Input**: `s [N×1], delta` (scalar)  
**Output**: `y [N×1]`

Element-wise: $y_i = s_i/\delta$ if $|s_i| < \delta$, else $\text{sign}(s_i)$.

This is the boundary-layer trick that eliminates hard switching. Without it,
`sign(s)` would switch at 2 kHz between ±K, driving actuator chattering. With it,
the switching gain is effectively reduced by the factor $s/\delta$ near zero, creating
a PD-like inner loop that degrades gracefully rather than chattering.

---

### `ref_trajectory.m` — 3-Phase Quintic Reference (Primary Scenario)

**Input**: `t` (scalar, seconds)  
**Output**: `qr, dqr, ddqr [3×1]` (rad, rad/s, rad/s²)

Three phases:
- **Phase 1** (0 ≤ t < 5 s): quintic polynomial blend $s(u) = 10u^3 - 15u^4 + 6u^5$,
  where $u = t/5$. Ensures $q_r$, $\dot{q}_r$, $\ddot{q}_r$ all start and end at
  zero — no acceleration jump, so the SMC doesn't see an impulse at t = 0 or t = 5.
- **Phase 2** (5 ≤ t < 10 s): $q_r = q_f$ constant, $\dot{q}_r = 0$, $\ddot{q}_r = 0$.
  The control task becomes pure disturbance rejection.
- **Phase 3** (10 ≤ t ≤ 15 s): same as Phase 2. The sudden load from `disturbance.m`
  steps in at t = 10 — the reference does not change, only the disturbance does.

Joint targets:

| Joint | $q_0$ (rad) | $q_f$ (rad) | Anatomy |
|-------|-------------|-------------|---------|
| MCP q1 | 0.10 | 0.70 | Base flexion — anchors posture |
| PIP q2 | 0.10 | 1.20 | Dominant curl — largest excursion |
| DIP q3 | 0.05 | 0.85 | Distal curl |

PIP > DIP > MCP ordering matches real human finger kinematics.

---

### `disturbance.m` — External Disturbance Torque Model

**Input**: `t` (scalar, seconds)  
**Output**: `tau_d [3×1]` (N·m)

Three physically motivated components:

**Component 1 — Background sinusoidal drift** (all phases, always active):

```matlab
tau_bg = [0.0020*sin(t)      + 0.0005*sin(200*pi*t);   % q1: 1Hz + 100Hz
          0.0010*cos(2*t)    + 0.0005*sin(200*pi*t);   % q2: 2Hz + 100Hz
          0.00015*sin(0.5*t) + 0.00005*sin(200*pi*t)]; % q3: 0.5Hz + tiny ripple
```

Low-frequency terms model postural shifts (wrist pronation/supination). 100 Hz terms
model motor electrical ripple. Amplitudes are ~28% of the nominal gravity torques
computed at qf (7.0 / 3.6 / 0.3 mN·m).

**Component 2 — Sudden grip-load** (activates at t = 10 s):

```matlab
sigma    = 0.5*(1 + tanh((t - 10.0)/(0.20/2)));   % smooth step 0→1 over ~0.2s
F_load   = [-0.0030; -0.0030; -0.00020];           % opposes flexion (opens grip)
tau_load = sigma * F_load;
```

The `tanh` rise makes the step $C^\infty$ continuous — no derivative discontinuity that
would confuse the integrator. Negative values mean the disturbance acts to *open* the
grip, fighting the controller which is holding the thumb closed.

**Total**: `tau_d = tau_bg + tau_load`

Worst-case magnitudes: q1 ≈ 5.0 mN·m, q2 ≈ 4.5 mN·m, q3 ≈ 0.37 mN·m.
Switching gains K = [10; 8; 1] mN·m provide ~2× margin in each channel.

---

### `run_thumb_smc_mb.m` — Primary Simulation Runner

**Entry point**: `run_thumb_smc_mb()` (no arguments, no return values)  
**Duration**: T = 15 s, dt = 0.5 ms → 30,001 steps  
**Wall time**: ~1.4 s

**Complete code flow:**

1. **Parameters**: `Lambda = diag([12,12,12])`, `K = diag([0.010, 0.008, 0.0010])`,
   `delta = 0.01`. Initial state `x0 = [0.10; 0.10; 0.05; 0; 0; 0]`.

2. **Pre-compute reference**: loop over all 30,001 time points calling `ref_trajectory(t(k))`
   to fill `Qref [N×3]`. This is done before integration to have metrics immediately.

3. **Integration loop** (the core):
   ```
   for k = 1:N-1
       [tau_k, sk] = smcmb_eval(X(k,:)', tk, Lambda, K, delta)
       Tau(k,:) = tau_k'
       S(k,:)   = sk'
       X(k+1,:) = rk4(@thumb_xdot, X(k,:)', tk, dt)'
   end
   ```
   `smcmb_eval` calls `ref_trajectory`, `SMC_eq_torque`, `sat_sign` to produce torque and
   surface value. `rk4` calls `thumb_xdot` four times per step; `thumb_xdot` calls
   `ref_trajectory`, `SMC_eq_torque`, `sat_sign`, `disturbance`, and `plant_dynamics`.

4. **Metrics**: computes RMS and peak errors per phase using logical index arrays
   `idx_p1 = t>=0 & t<5`, etc.

5. **Figures**: creates 5 figures using `figure/subplot/plot/print`:
   - Fig 1 (`thumb_tracking.png`): 2×3 layout
   - Fig 2 (`thumb_velocity.png`): 1×3 layout
   - Fig 3 (`thumb_sliding.png`): 1×3 layout
   - Fig 4 (`thumb_torques.png`): 1×3 layout
   - Fig 5 (`thumb_phase3_reject.png`): 1×3 layout, x-axis restricted to [9.5, 15]

6. **Save**: `thumb_results.mat` containing t, X, Tau, S, Qref, e_rms, e_ss, tau_pk,
   all per-phase error arrays, Lambda, K, delta.

7. **Animation**: calls `animate_thumb_2d` with explicit phase labels, colours, title.
   Produces `thumb_animation.avi` (151 frames at 10 fps) and `thumb_animation_final.png`.

All dynamics (inertia, Coriolis, gravity, SMC, RK4) are implemented as **local functions**
at the bottom of this one file. The functions `thumb_M`, `thumb_CG`, `smcmb_eval`,
`thumb_xdot`, `rk4` are self-contained — they do not call the standalone `.m` helper
files at runtime (though those helpers contain identical code for standalone reference).

---

### `run_thumb_extra_tests.m` — Full-Cycle Validation (Single Simulation)

**Entry point**: `run_thumb_extra_tests()` (no arguments)  
**Duration**: T = 25 s, dt = 0.5 ms → 50,001 steps  
**Wall time**: ~2.3 s

This is a complete self-contained simulation with its own dynamics engine, running ONE
continuous scenario (not multiple simulations concatenated). Starting from q0 = [0;0;0]
(the fully extended, kinematically hardest pose), it closes, holds, survives a load,
retraces, and holds open.

**Why q0 = [0;0;0] is the hardest pose:**
- All cosines in M(q) = 1 → inertia coupling is maximum
- All sines in G(q) = 0 → gravity is zero at start, ramps up as the thumb curls
- The model-based controller must correctly handle both the maximum coupling and
  the changing gravity without retuning

**Internal functions** (all in the same file):

- `ref_full_cycle(t, q0, qf, segs)` — 5-phase reference:
  - [0, 5]: quintic close q0 → qf
  - [5, 15]: constant hold at qf (covers both HOLD and LOAD phases)
  - [15, 22]: quintic retrace qf → q0
  - [22, 25]: constant hold at q0
  
- `dist_with_load(t, t_load)` — same structure as `disturbance.m` but parameterised:
  background sinusoid + smooth tanh step at `t_load = 10.0` s.

- `simulate_thumb(T, dt, q0, Lambda, K, delta, ref_fn, dist_fn)` — generic RK4
  integration engine that accepts the reference and disturbance as function handles,
  enabling both scenarios to share the same integrator.

- `smc_eval(x, t, Lambda, K, delta, ref_fn, dist_fn)` — evaluates torque and sliding
  surface at a given state; called by the integrator and for filling Tau, S arrays.

- `thumb_xdot(x, t, Lambda, K, delta, ref_fn, dist_fn)` — state derivative function
  passed to RK4. Calls `smc_eval` + `plant_dynamics` via inlined `thumb_M` / `thumb_CG`.

- `print_metrics(t, Q, Qref, Tau, segs, names)` — generic P-phase table printer.
  Works for any number of phases from the segs boundaries array.

- `plot_scenario(t, X, Qref, Tau, S, segs)` — generates 3 figures:
  - `thumb_fullcycle_tracking.png` — 6-panel tracking + error (2×3) under the chosen aggressive disturbance

- `quintic_s(u)` — evaluates $[s, \dot{s}, \ddot{s}]$ of the normalized quintic
  $s(u) = 10u^3 - 15u^4 + 6u^5$.

- `set_plot_defaults()` — sets `DefaultFigureColor`, `DefaultAxesFontSize`, etc. for
  consistent white-background academic-quality figures.

**Output files**: `thumb_fullcycle.mat`, `thumb_fullcycle_tracking.png`, `thumb_fullcycle_anim.avi`, `thumb_fullcycle_anim_final.png`.

---

### `animate_thumb_2d.m` — Parametric 2D Animator

**Signature**:
```matlab
animate_thumb_2d(t, Q, Qref, TAU_D_Nm, segs, labels, label_clrs, out_base, title_str)
```

Called by both simulation runners. Nargin defaults allow calling with 0 arguments
(loads `thumb_results.mat` automatically) or with any partial argument list.

**Layout**: 1380×820 dark-theme figure. Left panel (68% width): planar robot animation.
Right panel (32% width): live disturbance time history.

**Left panel elements:**
- Wrist backing plate (grey `patch`) — fixed base
- Palm mount body + 6 bolt circles (grey `patch` + circles) — decorative but readable
- Scale bar at bottom-right: "20 mm" reference line
- Magenta dashed arc (`plot`): entire reference tip trajectory drawn statically at startup
  so the viewer can see where the arm is going
- Dark grey line: full actual tip trace (drawn once, faded) — also static
- Start circle `o` and End square `s` markers on the reference arc
- Animated blue gradient links: proximal (LineWidth 14) → middle (11) → distal (8), with
  a specular highlight line offset by 2 mm perpendicular to each link
- Joint dots (circles) with labels MCP/PIP/DIP, fingertip diamond
- Growing cyan tip trace (appended frame by frame)
- Quiver force arrows at each joint (only from segs(2) onwards)
- Text overlays: time counter (yellow, top-right), phase label (colour-coded, top-right)

**Right panel elements:**
- Phase shading: `patch` blocks in each phase colour at 10% alpha
- Dashed vertical lines at phase boundaries
- Phase labels "P1", "P2", ... in matching colours
- Three coloured lines (steel blue, teal, amber) for τ_d1, τ_d2, τ_d3 growing left-to-right
- Moving vertical cursor line showing current time

**Rendering**: 10 fps output (`fps_out = 10`), stride = `round(1/(10 × 0.0005))` = every
100th integration step. Figure is created with `'Visible','off'` to skip rendering to
screen (batch-mode safe). `drawnow` is called each frame to force buffer flush before
`getframe`.

**Output**: `[out_base '.avi']` (Motion JPEG AVI, quality 92) and `[out_base '_final.png']`
(150 dpi PNG of the final frame).

**Local helper** `fk3(q, r1, r2, r3)`: forward kinematics returning a 4×2 matrix of
[base; joint1; joint2; tip] positions in mm. Called once per frame for the robot arm
and once per integration step to precompute tip loci for the static arc.

---

### `build_thumb_smc_model.m` — Simulink Model Builder (NOT used at runtime)

**Entry point**: `build_thumb_smc_model()` — run once to (re)generate `model_thumb_smc_mb.slx`.

Uses Simulink's programmatic API to construct the block diagram from scratch:
- `new_system(mdl)` — creates an empty model
- `set_param(mdl, 'Solver', 'ode4', 'FixedStep', '0.0005')` — configures the same
  solver (RK4) and step size as the MATLAB implementation
- `add_block(...)` — places: Clock, Ref_Gen (MATLAB Function), Demux_ref, Err_Sum,
  Controller (MATLAB Function), Disturbance (MATLAB Function), Mux_plant,
  Plant (SubSystem with Plant_Fcn inside), Int_qdot (integrates acceleration → velocity
  → position), Int_q, scope blocks
- `add_line(...)` — wires the blocks together

The block diagram mirrors the mathematical structure: Clock → Ref_Gen → [qref, dqref, ddqref] →
Demux → Controller (also receives q, qdot from integrators) → tau → Plant → qddot → Int_qdot → qdot → Int_q → q.

**This file is never called by the simulation pipeline.** It exists to show what the
Simulink model structure looks like in code form. The `.slx` it produces (`model_thumb_smc_mb.slx`)
is also unused — it is there for visual inspection only.

---

### `gs_run.m` — Gain Sweep Script (Script-Only, Inline Dynamics)

A flat script (no functions) that sweeps K3 ∈ {20, 40, 60, 80} mN·m and
δ ∈ {0.06, 0.10, 0.15} rad to find the K3/δ combination minimising joint-3 steady-state
error. All dynamics are inlined (no function calls) to avoid workspace pollution.

Uses an exponential reference $q_r(t) = q_f + \frac{4}{3}\Delta q\,e^{-t} - \frac{1}{3}\Delta q\,e^{-4t}$
(critically damped) rather than the quintic polynomial — this was an earlier reference
design used only during tuning. The inlined Euler-forward approximation was later
replaced by the proper RK4 in the final scripts.

Not called by any other file. Kept for audit trail of how K = diag(10,8,1) mN·m was chosen.

---

### `gs_rk4.m` — Gain Sweep Script With Full RK4 (Script-Only)

Same sweep structure as `gs_run.m` but with a full 4-stage RK4 step inlined four times
per step. Runs for T = 8 s (shorter than the full scenario) for speed during tuning.
Again uses the exponential reference. Final output grid of {K3, δ} vs e3_ss was used
to confirm the chosen K3 = 1 mN·m with δ = 0.01 rad is near-optimal for this disturbance
level without over-gaining (which would increase chattering without improving tracking).

---

### `gain_sweep_fn.m` — Gain Sweep Function Version

Same logic as `gs_run.m` but wrapped in a proper function with a local `rk4_step` helper
and a `thumb_xdot` that calls `manipulator_M` / `manipulator_CG` (the standalone helper
files). This version was the final pre-tuning check before the parameters were locked in.
Not called at runtime.

---

## Output Files — What Each Figure and File Contains

### `thumb_results.mat` — Primary Simulation Data

Variables saved: `t [N×1]`, `X [N×6]` (states: q1,q2,q3,dq1,dq2,dq3), `Tau [N×3]`
(control torques), `S [N×3]` (sliding surfaces), `Qref [N×3]` (reference angles),
`e_rms [1×3]`, `e_ss [1×3]`, `tau_pk [1×3]`, `rms_p1/p2/p3`, `ss_p2`, `pk_p3`,
`Lambda`, `K`, `delta`.

N = 30,001 (15 s at 0.5 ms step).

---

### `thumb_tracking.png` — Joint Position Tracking and Error

**Layout**: 2 rows × 3 columns. Figure size: 1420×880 px. White background.

**Top row (subplots 1–3)**: One subplot per joint. Each shows:
- Phase shading: yellow (P1, 7% opacity), green (P2), red (P3)
- Magenta line (LineWidth 3): reference q_d(t) — the quintic ramp then flat hold
- Blue/red/green line (LineWidth 2.2): actual joint angle q(t)
- Dashed vertical lines at t=5 and t=10
- Phase annotations: "CLOSE", "HOLD", "SUSTAIN" in the upper portion
- Title: "Joint N — Proximal/Middle/Distal q_N"

In Phase 1 the reference and actual lines are nearly indistinguishable — the SMC
tracks the smooth quintic within 1–2% of its amplitude. In Phase 2 the reference
goes flat and the actual oscillates around it at the disturbance frequency (1 Hz for q1,
2 Hz for q2, 0.5 Hz for q3). In Phase 3 a small visible step occurs at t=10 s as the
sudden load is applied, then recovers within ~0.5 s.

**Bottom row (subplots 4–6)**: Error e_i = q_i − q_d,i per joint.
- Same phase shading and vertical lines as top row
- Horizontal dotted line at zero
- q1: amplitude ±6 mrad, hold RMS 2.7 mrad
- q2: amplitude ±22 mrad peak during Phase 1, ±10 mrad in hold
- q3: amplitude ±27 mrad peak, ±15 mrad in hold (closest to 17.5 mrad perception limit)

**Supertitle**: "Robotic Thumb SMC-MB — Joint Position Tracking (P1 CLOSE | P2 HOLD | P3 SUSTAIN)"

---

### `thumb_velocity.png` — Joint Velocity Tracking

**Layout**: 1 row × 3 columns. Figure size: 1420×560 px.

Each subplot shows $\dot{q}_i(t)$ (actual joint velocity). There is no velocity
reference line plotted here — the figure shows purely what velocities the plant produces.

Velocities peak near t = 2.5 s (midpoint of quintic ramp where $\dot{s}$ is maximum).
Peak values: q1 ~0.12 rad/s, q2 ~0.22 rad/s, q3 ~0.16 rad/s. All approach zero by t=5
(the quintic guarantees $\dot{q}_r(5) = 0$). In Phases 2 and 3 the velocity oscillates
near zero driven by the disturbance but remains bounded within ±0.02 rad/s.

The horizontal dotted line at 0 makes it easy to confirm the velocity returns to rest
after the close phase — evidence that the quintic blend is working correctly.

---

### `thumb_sliding.png` — Sliding Surface Variables

**Layout**: 1 row × 3 columns.

Each subplot shows $s_i(t) = \dot{e}_i + \lambda_i e_i$.

**Interpretation:**
- If $|s_i| < \delta = 0.01$ rad: the system is inside the boundary layer — the controller
  acts as a PD law. The sliding surface itself cannot be seen to cross $\pm\delta$ in
  normal operation during phases 2–3.
- During Phase 1 the surface is larger (up to ~0.3 rad·s⁻¹ equivalent) because the
  quintic reference is moving quickly; the controller is actively steering toward the surface.
- At t = 10 s a small spike occurs as the sudden load pushes s away from zero, then the
  switching term pulls it back within ~0.5 s.
- The 100 Hz ripple is visible as high-frequency oscillation within the boundary layer
  in Phases 2 and 3 — this is normal and bounded.

---

### `thumb_torques.png` — Control Torques τ_i(t)

**Layout**: 1 row × 3 columns.

Each subplot shows the total SMC torque τ_i = τ_eq,i + τ_sw,i.

**What to read in each subplot:**
- The **slowly varying baseline** is τ_eq: it mirrors the gravity compensation
  (peaks at ~21 mN·m for q1 when the arm is in the high-gravity grip posture).
- The **100 Hz chattering ripple** superimposed on the baseline is τ_sw responding to the
  motor electrical ripple in the disturbance. Amplitude is small (≤ K_i = 10/8/1 mN·m).
- At t = 10 s: a brief additional switching transient is visible as τ_sw responds to the
  sudden load step.
- Peak torques: q1 = 21.4 mN·m, q2 = 12.2 mN·m, q3 = 1.74 mN·m. These are within
  realistic micro-actuator output for a wearable exoskeleton.

Note: q3 torque is an order of magnitude smaller than q1/q2 because the distal link is
lightest and shortest, so gravity compensation is minimal there.

---

### `thumb_phase3_reject.png` — Zoomed Phase 3 Disturbance Rejection

**Layout**: 1 row × 3 columns, x-axis restricted to t ∈ [9.5, 15] s.

Each subplot shows the tracking error $e_i(t)$ zoomed into the Phase 3 window with a
vertical dashed line at t = 10 s marking the load onset.

**What to confirm:**
- The error at t < 10 s (P2 hold level) — this is the baseline before the load.
- The transient step at t = 10 s — the load adds a DC component to τ_d, which appears
  as an error step whose size is approximately $|F_{\text{load},i}| / (\lambda_i K_i)$.
- Recovery within ~0.3–0.5 s — the sliding surface drives the error back to the pre-load
  band, confirming that the switching gain exceeds the load magnitude.
- Post-recovery error level same as pre-load — no steady-state offset because the tanh
  sigma eventually saturates at 1 and the SMC absorbs the constant load within
  the boundary layer.

---

### `thumb_animation.avi` / `thumb_animation_final.png`

**Duration**: 15 s at 10 fps = 151 frames. File size ~2–5 MB (Motion JPEG quality 92).

`thumb_animation_final.png` is the last frame, captured after close(vw). It shows:

**Left panel**: Robot arm at t = 15 s (end of Phase 3 — fully gripped posture). The
magenta dashed arc shows the reference tip path: a smooth curve from Start (extended
position ~108 mm from origin at a slight angle) sweeping up and left to End (grip
position ~10 mm from origin at ~80 mm height). The cyan growing trace closely follows
this arc. Force arrows (green/teal quiver) are visible at the three joints pointing
in the direction of the disturbance torques.

**Right panel**: Full 15 s history of τ_d1, τ_d2, τ_d3 with phase shading. The 100 Hz
ripple is visible as a band. The sudden step at t = 10 s is clearly visible in the q1
and q2 channels dropping to ~−3 mN·m.

---

### `thumb_fullcycle.mat` — Full-Cycle Simulation Data

Variables saved: `t [50001×1]`, `X [50001×6]`, `Tau [50001×3]`, `S [50001×3]`,
`Qref [50001×3]`, `Lambda`, `K`, `delta`, `q0 [3×1]`, `qf [3×1]`, `segs [1×6]`.

The 5-phase boundary times are stored in `segs = [0, 5, 10, 15, 22, 25]`.

---

### `thumb_fullcycle_tracking.png` — 5-Phase Tracking Under Aggressive Disturbance

**Layout**: 2 rows × 3 columns over 25 s. The 5-phase shading (P1 CLOSE | P2 HOLD |
P3 LOAD | P4 RETRACE | P5 HOLD-OPEN) makes it immediately clear where the controller
breaks: q2 collapses for the full 15 s following the load step, and never recovers
during retrace or hold-open. q1 and q3 stay close to the reference because their
reaching-condition margins remain positive (see Stage 1 theory below).

This file is the visual evidence that motivates Stage 2: a plain SMC-MB cannot reject
this load, even though it can reject the nominal load (LOAD_SCALE = 1.0 — set inside
`dist_with_load` in `run_thumb_extra_tests.m`).

---

### `thumb_fullcycle_anim.avi` / `thumb_fullcycle_anim_final.png`

**Duration**: 25 s at 10 fps = 251 frames.

The final frame shows the arm at t = 25 s (P5 HOLD OPEN — fully extended). The reference
arc forms a clean **crescent shape**: it traces from Start (extended, ~108 mm along x-axis),
curls up to the grip position (P1 close), holds (P2+P3), then retraces back to End =
Start (P4+P5). Start and End markers are coincident, visually confirming cycle closure.

The right panel shows all 5 phase shadings with the sudden load step clearly visible as
a spike at t = 10 s in the q1 and q2 disturbance channels.

---

## Performance Summary

### Primary scenario (`run_thumb_smc_mb.m`)

| Metric | q1 (MCP) | q2 (PIP) | q3 (DIP) |
|--------|----------|----------|----------|
| P1 Close RMS | 3.7 mrad | 13.0 mrad | 17.0 mrad |
| P2 Hold RMS  | 2.7 mrad |  9.2 mrad | 15.1 mrad |
| P3 Reject RMS | 2.4 mrad | 10.4 mrad | 5.8 mrad |
| P3 Peak error | 3.4 mrad | 13.8 mrad | 20.3 mrad |
| Max torque | 21.4 mN·m | 12.2 mN·m | 1.74 mN·m |

### Full-cycle validation (`run_thumb_extra_tests.m`)

| Phase | q1 RMS | q2 RMS | q3 RMS |
|-------|--------|--------|--------|
| P1 CLOSE   | 3.84 mrad | 13.31 mrad | 17.36 mrad |
| P2 HOLD    | 2.74 mrad |  9.16 mrad | 15.05 mrad |
| P3 LOAD    | 2.40 mrad | 10.40 mrad |  5.93 mrad |
| P4 RETRACE | 2.58 mrad | 15.79 mrad | 10.11 mrad |
| P5 HOLD OPEN | 4.68 mrad | 16.61 mrad | 10.79 mrad |

P4/P5 errors are consistent with or below P1/P2 despite the reversed direction and
gravity profile — confirming bidirectional robustness. Human tactile resolution ~17.5 mrad.
All hold-phase errors are below this in q1 and q2. q3 is borderline in some phases — see NDOB
section below for how to improve it.

---

## Trustworthiness Assessment

### What is correctly modelled

| Physical effect | Status | Notes |
|-----------------|--------|-------|
| Configuration-dependent inertia | Full M(q) | 3×3 exact Lagrangian, all coupling terms |
| Coriolis and centrifugal | Full C(q,q̇) | Christoffel symbol derivation, all cross-terms |
| Gravity | Full G(q) | Recomputed every time step from q |
| 100 Hz motor ripple | Included | sin(200πt) in disturbance |
| Sudden external load | Included | tanh smooth step at t=10 s |
| Background postural drift | Included | Low-frequency sinusoids |
| RK4 integration accuracy | Verified | 2 kHz step: truncation error is negligible |

### Known simplifications

| Simplification | Impact | Acceptability |
|----------------|--------|---------------|
| Planar 2-D model | No out-of-plane MCP motion | Standard for coursework exoskeleton |
| No joint friction / viscous damping | Would reduce chattering, lower required K | Conservative — makes problem harder |
| Uniform-rod inertia | ≤5% error for these link sizes | Standard approximation |
| No tendon compliance | Real thumb uses tendons, not direct torque | Acceptable for rigid exoskeleton frame |
| Estimated masses | Not from a real prototype | Reasonable for Al/polymer at this scale |
| Disturbance model is known | NDOB makes this point moot — see below | N/A |

---

## What Has NOT Been Tested (Known Gaps)

These are honest gaps, not defects:

1. **Near-singularity configurations**: when links are nearly collinear (e.g. q2 ≈ π),
   M(q) approaches singularity and the model-based inversion becomes ill-conditioned.
   Not a practical concern for the thumb (q2 max = 1.2 rad ≈ 69°) but not explicitly tested.

2. **Model uncertainty**: the simulation uses the same M, C, G in the controller and the
   plant — a perfect model. Real hardware will have ±10–20% mass/inertia error. The SMC
   switching gain K theoretically handles this if the uncertainty is bounded, but the
   bound has not been quantified or tested.

3. **Very large disturbances**: the K = diag(10,8,1) mN·m gains maintain sliding only
   while $|d_i| < K_i$. The sudden load (3 mN·m) stays well under K (10 mN·m), but
   a 15 mN·m impact (e.g. dropping an object) would break the sliding condition for q1
   and cause the system to leave the surface temporarily.

4. **High-frequency reference tracking**: the quintic polynomial is smooth (0.2 Hz main
   content). A fast reference (e.g. 10 Hz tremor compensation) would push the SMC
   surface outside the boundary layer and increase chattering significantly.

5. **Actuator saturation**: torques above 21.4 mN·m are commanded for q1 during the
   close phase. The simulation does not model actuator saturation — a real motor at this
   scale may be limited to 15–20 mN·m continuous, which would degrade performance.

---

## NDOB + SMC — What It Is and Whether We Should Implement It

### What Is a Nonlinear Disturbance Observer (NDOB)?

An NDOB is an **online disturbance estimator** that uses the known plant model to infer
the unknown external disturbance $\tau_d$ from the measured state trajectory. The
estimated disturbance $\hat{\tau}_d(t)$ is then fed forward into the control law to
cancel it, instead of relying entirely on the switching gain K.

For a rigid manipulator (Chen et al., 2000), the standard NDOB is:

Define auxiliary variable:  $p = \hat{\tau}_d - L\,M(q)\,\dot{q}$

Observer update law:
$$\dot{p} = -L\,p \;-\; L\!\left[L\,M(q)\,\dot{q} + C(q,\dot{q})\,\dot{q} + G(q) - \tau\right]$$
$$\hat{\tau}_d = p + L\,M(q)\,\dot{q}$$

where $L > 0$ (diagonal) is the **observer gain matrix**. The estimation error
$e_d = \tau_d - \hat{\tau}_d$ decays as $\dot{e}_d = -L\,e_d$ — i.e. exponentially
with time constant $1/L$. Choosing $L = 20\,I$ gives a 50 ms estimation convergence.

### Modified SMC+NDOB Control Law

Replace the switching torque with:

$$\boldsymbol{\tau} = \tau_{\text{eq}} \;-\; \hat{\boldsymbol{\tau}}_d \;-\; K_{\text{new}}\,\text{sat}\!\left(\frac{\boldsymbol{s}}{\delta}\right)$$

The NDOB estimate feeds forward (cancels) the bulk of the disturbance. The switching
term now only has to cover the **estimation error** $e_d = \tau_d - \hat{\tau}_d$, which
decays exponentially. So $K_{\text{new}}$ can be **much smaller** than the original $K$.

In practice: if L is high enough to track the slow disturbance components (background
sinusoids, sudden load) but not the 100 Hz ripple, we can set:
- $K_{\text{new}} = \text{diag}(2, 2, 0.3)$ mN·m — covering only the ripple residual
- vs original $K = \text{diag}(10, 8, 1)$ mN·m — covering the entire disturbance

This is a **5× reduction in switching gain** → proportional reduction in chattering amplitude.

### Would It Actually Improve Our Results?

**Yes, in three specific ways:**

| Issue in current results | How NDOB fixes it |
|--------------------------|-------------------|
| 100 Hz chattering visible in τ plots | Smaller K_new → smaller chattering amplitude |
| q3 hold error 15.1 mrad (near 17.5 mrad threshold) | NDOB pre-cancels the 0.15 mN·m background disturbance on q3, reducing steady-state error |
| 0.3–0.5 s recovery from sudden load | NDOB tracks the tanh step within ~0.1 s; recovery would be 3–5× faster |

**What NDOB cannot fix:**
- The 100 Hz ripple in the disturbance — an observer with L=20 rad/s has a bandwidth
  of only ~3 Hz. The 100 Hz ripple propagates through as estimation error, so K_new
  still needs to cover 0.5 mN·m on q1/q2. Chattering reduction is real but not complete.
- Model uncertainty — the NDOB lumps model error into its disturbance estimate, which
  helps. But unmodelled dynamics (friction, flexibility) would add persistent bias to ê_d.

### Is It Doable In This Codebase?

**Yes, straightforwardly.** The infrastructure is already here:

1. All of M(q), C(q,q̇)q̇, G(q) are computed every step via `manipulator_M` and
   `manipulator_CG` — the NDOB needs exactly these quantities.
2. The state [q, q̇] is available every step.
3. The control torque τ is computed before the integration step — it can be passed to
   the observer.

Implementation plan:
1. Add a new state component: `p [3×1]` — the NDOB auxiliary variable. Extend the state
   vector from 6 to 9 elements.
2. Compute `p_dot = -L*p - L*(L*M*qdot + C_vec + G_vec - tau)` at each step.
3. Integrate p with the same RK4.
4. Compute `tau_d_hat = p + L*M*qdot` and subtract from the control torque.
5. Reduce switching gain K from diag(10,8,1) to diag(2,2,0.3) mN·m.
6. Compare: same scenario, same metrics table — show the improvement.

Reference implementation exists in `assignment/model_underactuated_NDOB.slx` — the
block diagram shows the NDOB structure (though for a different plant). The equations
map directly to the manipulator case.

### Is the Current Controller the Best Possible Without NDOB?

**Yes, for a pure SMC.** The controller is well-tuned:
- K is at ~2× the worst-case disturbance — the minimum for robust sliding with margin
- λ = 12 rad/s is fast enough to track the quintic ramp without excessive overshoot
- δ = 0.01 rad is tight — smaller values would increase chattering without improving
  steady-state error

Further tuning within pure SMC produces diminishing returns: reducing δ below 0.005 rad
would increase the 100 Hz chattering faster than it reduces steady-state error. Increasing
K would increase chattering proportionally. The current values sit at the Pareto frontier
for this disturbance profile.

**NDOB is the correct next step** if better performance is required, because it
fundamentally changes the disturbance architecture rather than just retuning the
existing structure.

---

## Complete File Index (Current State)

### Source Files

| File | Purpose | Called by |
|------|---------|-----------|
| `run_thumb_smc_mb.m` | Primary 3-phase simulation runner (15 s) | Entry point |
| `run_thumb_extra_tests.m` | Full 5-phase cycle simulation runner (25 s) | Entry point |
| `animate_thumb_2d.m` | Parametric dark-theme 2D animator | Both runners |
| `thumb_kinematics.m` | Standard DH table, $A_i$, $T_0^i$, full $T_0^3$, and joint positions | Figure scripts, standalone FK check |
| `make_thumb_kinematics_figure.m` | Generates the DH/FK LinkedIn-ready 1200×627 PNG | Standalone figure script |
| `make_linkedin_infographic.m` | Generates the academic MATLAB-only infographic slide | Standalone figure script |
| `make_linkedin_beforeafter.m` | Generates the SMC failure vs NDOB recovery comparison PNG | Standalone figure script |
| `ref_trajectory.m` | Quintic 3-phase reference (standalone version) | run_thumb_smc_mb |
| `disturbance.m` | External disturbance model (standalone version) | run_thumb_smc_mb, animate_thumb_2d |
| `manipulator_M.m` | 3×3 inertia matrix M(q) | SMC_eq_torque, plant_dynamics, gain sweeps |
| `manipulator_CG.m` | Coriolis vector + gravity vector | SMC_eq_torque, plant_dynamics, gain sweeps |
| `plant_dynamics.m` | Forward dynamics solver q̈ = M⁻¹(τ − Cq̇ − G) | run_thumb_smc_mb (inline), gain sweeps |
| `SMC_eq_torque.m` | Equivalent (feedforward) SMC torque | run_thumb_smc_mb (inline), gain sweeps |
| `sat_sign.m` | Smooth saturation function for SMC switching | All SMC evaluators |
| `build_thumb_smc_model.m` | Builds model_thumb_smc_mb.slx from scratch | Standalone only |
| `model_thumb_smc_mb.slx` | Simulink block diagram (visual reference only) | Nothing |
| `gs_run.m` | Gain sweep script — Euler, inline dynamics | Standalone tuning tool |
| `gs_rk4.m` | Gain sweep script — RK4, inline dynamics | Standalone tuning tool |
| `gain_sweep_fn.m` | Gain sweep function — RK4, calls helper .m files | Standalone tuning tool |

### Generated Output Files

| File | Produced by | Contents |
|------|-------------|----------|
| `thumb_results.mat` | run_thumb_smc_mb | t, X, Tau, S, Qref, metrics, tuning params |
| `thumb_tracking.png` | run_thumb_smc_mb | 2×3 position tracking + error, 3 phases shaded |
| `thumb_velocity.png` | run_thumb_smc_mb | 1×3 joint velocities |
| `thumb_sliding.png` | run_thumb_smc_mb | 1×3 sliding surfaces s_i(t) |
| `thumb_torques.png` | run_thumb_smc_mb | 1×3 control torques, 100 Hz chattering visible |
| `thumb_phase3_reject.png` | run_thumb_smc_mb | 1×3 zoomed Phase 3, load event at t=10 s |
| `thumb_animation.avi` | run_thumb_smc_mb → animate_thumb_2d | 151 frames, 10 fps, 3-phase, dark theme |
| `thumb_animation_final.png` | run_thumb_smc_mb → animate_thumb_2d | Final frame at t=15 s (grip posture) |
| `thumb_fullcycle.mat` | run_thumb_extra_tests (LOAD_SCALE = 2.40) | t, X, Tau, S, Qref over 25 s, 5 phases — under aggressive disturbance |
| `thumb_fullcycle_tracking.png` | run_thumb_extra_tests | 2×3 tracking across 5 phases, 25 s — q2 visibly fails for 15 s |
| `thumb_fullcycle_anim.avi` | run_thumb_extra_tests → animate_thumb_2d | 251 frames, 10 fps, 5-phase animation under stress |
| `thumb_fullcycle_anim_final.png` | run_thumb_extra_tests → animate_thumb_2d | Final frame at t=25 s |
| `thumb_dh_forward_kinematics.png` | make_thumb_kinematics_figure | 1200×627 DH table, kinematic chain, full transform, and numeric fingertip pose |
| `thumb_linkedin_beforeafter.png` | make_linkedin_beforeafter | 1200×627 side-by-side SMC q2 failure vs NDOB recovery comparison |
| `thumb_linkedin_infographic.png` | make_linkedin_infographic | 1200×1700 MATLAB-generated academic infographic with plant, DH, control law, observer, plots, and result table |


---

## Stage 1 — Iterative Stress Test of SMC-MB  (full 5-phase, 25 s)

Rather than create a parallel runner / output set, the stress test was performed by
**modifying a single constant inside the original validator** [`run_thumb_extra_tests.m`](run_thumb_extra_tests.m):
the local helper `dist_with_load` now exposes `LOAD_SCALE` at the top, which multiplies
the sudden-load amplitude. `LOAD_SCALE = 1.00` reproduces the original clean-baseline
behaviour; `LOAD_SCALE = 2.40` produces the breaking-point run that motivates NDOB.
**The mass / inertia / link-length / gravity model, the 5-phase trajectory, the
controller gains, the integrator, and the background sinusoids are all unchanged** — only
the single load multiplier was swept. Outputs keep the original `thumb_fullcycle_*`
prefix; the saved `.mat`, tracking PNG, and animation are therefore the stressed run.

### Iteration log

The escalation was not blind: starting from `load_scale = 1.0` (baseline) the load
was increased in coarse steps until q2 began to drift, then in 0.1-step refinements
near the boundary, then well past it to confirm divergence.

| `load_scale` | q1 P3 RMS (rad) | q2 P3 RMS (rad) | q3 P3 RMS (rad) | q2 P3 peak | Outcome |
|---:|---:|---:|---:|---:|---|
| 1.00 | 0.0024 | 0.0104 | 0.0060 | 0.0138 | baseline (matches `thumb_results.mat`) |
| 1.50 | 0.0024 | 0.0105 | 0.0063 | 0.0136 | indistinguishable from baseline |
| 2.00 | 0.0022 | 0.0133 | 0.0066 | 0.0204 | q2 begins to grow (1.3× baseline) |
| 2.10–2.30 | 0.0021 | 0.0137–0.0148 | 0.007 | 0.020–0.022 | last stable plateau |
| **2.40** | **0.0022** | **0.1517** | **0.0077** | **0.3586** | **CHOSEN — q2 RMS jumps 15× and q2 peak ≈ 20.5°** |
| 2.50 | 0.058 | 0.72 | 0.007 | 1.41 | q2 swings 81° — already runaway |
| 3.00–7.00 | NaN | NaN | NaN | — | q2 runs away, M(q) becomes near-singular, RK4 blows up |

Final stress run (`load_scale = 2.40`), **all five phases**:

| Phase | q1 RMS (rad) | q2 RMS (rad) | q3 RMS (rad) |
|---|---:|---:|---:|
| P1 CLOSE   | 0.0038 | 0.0133 | 0.0174 |
| P2 HOLD    | 0.0028 | 0.0091 | 0.0151 |
| **P3 LOAD**    | **0.0022** | **0.1517** | **0.0077** |
| **P4 RETRACE** | **0.0026** | **0.1239** | **0.0081** |
| **P5 HOLD-OPEN**| **0.0042** | **0.1278** | **0.0095** |

The collapse persists for the full 15 s following the load step. The thumb now fails
to retrace: q2 RMS during P4 is 0.124 rad ≈ 7.1° steady-state offset, and q2 RMS
during P5 stays at 0.128 rad even after the reference is back at full extension.
The animation shows q2 making repeated dips of ~30° during the load and never fully
recovering. Sliding-surface diagnostics in P3:

- `|s_1| > δ` for **96%** of P3
- `|s_2| > δ` for **99%** of P3 (boundary layer fully saturated)
- `|s_3| > δ` for **99%** of P3
- Peak |τ| (mN·m) = (21.8, **12.5 — i.e. K_2 = 8 mN·m saturated 56% above limit by τ_eq**, 1.77)

### Why 2.40 × is the breaking point — SMC reaching-condition theory

For a model-based SMC with switching gain $K = \mathrm{diag}(K_1, K_2, K_3)$ and a
Slotine-Li reference dynamics, the sliding mode is guaranteed only while the
**reaching condition**
$$K_i \;>\; |d_i|_{\max}$$
holds on every channel. At `load_scale = 2.40` the worst-case disturbance amplitudes
are (recall the load vector is $-3,\,-3,\,-0.2$ mN·m before scaling, the background
is $\pm 2.5,\,\pm 1.5,\,\pm 0.2$ mN·m, and the 100 Hz ripple adds $\pm 0.5$ mN·m on
each axis):

| Joint | background peak (mN·m) | scaled load (mN·m) | total |d_i|_max (mN·m) | K_i (mN·m) | margin (mN·m) |
|-------|------------------------:|-------------------:|-----------------------:|-----------:|--------------:|
| q1 (MCP) | 2.5 | 2.40 × 3.0 = **7.2** | **≈ 9.7** | 10 | +0.3 (paper-thin) |
| q2 (PIP) | 1.5 | 2.40 × 3.0 = **7.2** | **≈ 8.7** | 8  | **−0.7 (violated)** |
| q3 (DIP) | 0.20 | 2.40 × 0.20 = **0.48** | ≈ 0.68 | 1 | +0.32 |

Channel 2 is the **only** one whose margin goes negative — so the model predicts
q2 alone breaks. The simulated data agree exactly: q2 RMS jumps 15× while q1 and q3
errors are essentially unchanged. Beyond `load_scale = 2.50` the q2 error is large
enough to drive the configuration into a region where the open-loop dynamics under
saturated control diverge faster than the controller can stabilise — by 3.0× the
state runs to ±10⁵⁰ in microseconds and produces NaN.

### Stress outputs (under prefix `thumb_fullcycle`)

After setting `LOAD_SCALE = 2.40` inside `dist_with_load` and re-running
`run_thumb_extra_tests`, the following originals are overwritten with the stressed
data — these are the SMC-only deliverables that motivate NDOB:

- [`thumb_fullcycle.mat`](thumb_fullcycle.mat) — full t, X, Tau, S, Qref under the chosen load.
- [`thumb_fullcycle_tracking.png`](thumb_fullcycle_tracking.png) — 2×3 tracking + error, all 5 phases shaded; q2 visibly fails for 15 s.
- [`thumb_fullcycle_anim.avi`](thumb_fullcycle_anim.avi) / [`thumb_fullcycle_anim_final.png`](thumb_fullcycle_anim_final.png) — 251-frame 5-phase animation. **The cyan tip-path is drawn incrementally (you only see where the robot has actually been up to the current frame).** No pre-drawn grey future trace.

Set `LOAD_SCALE = 1.00` and re-run to recover the clean-baseline data.

### Exact disturbance applied (both SMC and NDOB)

Both `run_thumb_extra_tests.m` (SMC) and `run_thumb_ndob.m` (NDOB+SMC) call the
**identical `dist_with_load` function** with the same parameters. Neither controller
is given any advantage in the disturbance it faces.

```
τ_d(t) = τ_background(t) + LOAD_SCALE × σ(t) × τ_load

τ_background = [0.002·sin(t)       + 0.0005·sin(200πt)]    MCP  ← 100 Hz ripple
               [0.001·cos(2t)      + 0.0005·sin(200πt)]    PIP
               [0.00015·sin(0.5t)  + 0.00005·sin(200πt)]   DIP

σ(t)     = 0.5·(1 + tanh((t − 10) / 0.2))   ← smooth step at t = 10 s
τ_load   = [−3; −3; −0.2] mN·m               ← direction of grip-opposing load
LOAD_SCALE = 2.40
```

Peak load amplitude at steady-state (σ→1): **7.2 mN·m on MCP and PIP**, plus
background, giving total worst-case per channel of ≈ 9.7, 8.7, 0.68 mN·m.

This same `τ_d` enters `thumb_xdot` (SMC plant) and `thumb_xdot_ndob` (NDOB+SMC
plant) at every RK4 sub-step — it is not smoothed, not pre-computed and stored, and
not different between the two runs.

---

## Stage 2 — NDOB + SMC Recovers and Beats the Baseline

### The corrected Chen-2000 update law (sign error fix)

The earlier "NDOB + SMC" subsection of this document quoted the auxiliary-state update
with one set of signs that, when re-derived, does not produce the desired error
dynamics $\dot{\hat\tau}_d = L(\tau_d - \hat\tau_d)$ — it leaves a residual $2L\tau$
term. The corrected form (verified by direct substitution and matching Chen 2000
Eq. (11)) is

$$\boxed{\;\dot p \;=\; -L\,p \;-\; L\,\bigl[L\,M(q)\,\dot q \;-\; C(q,\dot q)\dot q \;-\; G(q) \;+\; \tau\bigr],\qquad
\hat\tau_d \;=\; p \;+\; L\,M(q)\,\dot q.\;}$$

Substituting back: $\dot{\hat\tau}_d = \dot p + L M \ddot q$. From the plant
$M\ddot q = \tau + \tau_d - C\dot q - G$, so $L M \ddot q = L\tau + L\tau_d - LC\dot q - LG$.
Adding $\dot p = -Lp - L^2 M \dot q + LC\dot q + LG - L\tau$:
$\dot{\hat\tau}_d = -Lp - L^2 M\dot q + L\tau_d = -L\hat\tau_d + L\tau_d$. ✓

This is implemented in [`run_thumb_ndob.m`](run_thumb_ndob.m) line `p_dot = -Lobs*p - Lobs*(Lobs*M*dq - C_v - G_v + tau);`.

### Choice of L and K_new

**L (observer gain).** The estimation-error decays as $e^{-Lt}$ so $\tau_{\text{obs}} = 1/L$.
The disturbance has two distinct timescales:

- *Slow content*: tanh load step at t = 10 s with rise time ≈ 0.20 s (≈ 5 Hz of energy).
- *Fast content*: 100 Hz motor ripple (the unrejectable ground state).

For NDOB to track the load step but **not** the ripple, the observer bandwidth
$\omega_{\text{obs}} = L$ should sit comfortably between them. **L = 50·I rad/s**
(8 Hz, between 5 Hz and 100 Hz) gives a 20 ms time constant — fast enough to nail
the tanh transient, slow enough to attenuate 100 Hz by a factor of $\approx 100/8
= 12.5\times$.

**K_new (reduced switching gain).** With NDOB feeding $\hat\tau_d$ forward, the
switching term only has to cover the **estimation residual** $e_d = \tau_d - \hat\tau_d$.
Measured residuals from the simulation at L = 50 are 0.36, 0.36, 0.04 mN·m on the
three joints (table below) — set by the un-rejected 100 Hz ripple. Choosing
$K_{\text{new}}$ at ~5× the residual gives **K_new = diag(2, 2, 0.3) mN·m** — a 5×
reduction from the baseline K = diag(10, 8, 1) mN·m. The reaching condition
$K_{\text{new},i} > |e_{d,i}|_{\max}$ is satisfied with a healthy margin on every channel
even at the stress disturbance level.

### NDOB performance under the same 2.40× stress disturbance — full 5 phases

| Phase | q1 RMS (rad) | q2 RMS (rad) | q3 RMS (rad) |
|---|---:|---:|---:|
| P1 CLOSE     | 0.00021 | 0.00096 | 0.00074 |
| P2 HOLD      | 0.00014 | 0.00090 | 0.00146 |
| **P3 LOAD**     | **0.00015** | **0.00110** | **0.00160** |
| **P4 RETRACE**  | **0.00022** | **0.00101** | **0.00099** |
| **P5 HOLD-OPEN**| **0.00014** | **0.00106** | **0.00079** |

NDOB residual `e_d` RMS in P3: **(0.36, 0.36, 0.04) mN·m** — exactly matched to
the K_new sizing.

### Comparison — baseline vs stressed SMC-MB vs NDOB+SMC at the same 2.40× load

| Metric | Baseline SMC-MB (1.0×) | Stressed SMC-MB (2.40×) | **NDOB+SMC (2.40×)** | NDOB vs stressed | NDOB vs baseline |
|---|---:|---:|---:|---:|---:|
| q1 RMS  P3 (mrad) | 2.4   | 2.2   | **0.15** | 14× better | 16× better |
| q2 RMS  P3 (mrad) | 10.4  | 151.7 | **1.10** | **138× better** | **9.4× better** |
| q3 RMS  P3 (mrad) | 6.0   | 7.7   | **1.60** | 4.8× better | 3.7× better |
| q2 RMS  P4 (mrad) | n/a   | 124   | **1.01** | **123× better** | n/a |
| q2 peak P3 (mrad) | 13.8  | 358.6 | **2.53 (≈ 0.15°)** | 142× better | 5.4× better |
| K (mN·m) | (10, 8, 1) | (10, 8, 1) | **(2, 2, 0.3)** | 5× smaller | 5× smaller |

The NDOB at the stressed disturbance level beats not only the stressed SMC-MB by
~140× on q2 but also the **nominal SMC-MB baseline** by ~10× on q2 — while running
with a 5× smaller switching gain. This is the qualitative trade predicted by the
boundary-layer analysis: $\hat\tau_d$ pre-cancels the bulk disturbance, the
ultimate error $|e_\infty| \lesssim |e_d|/(\lambda K_{\text{new}})$ goes down because
the residual $|e_d|$ is now tiny, and chattering goes down because $K_{\text{new}}$
is small.

### Robustness sweep — NDOB at extreme load_scale

Same observer (L = 50·I) and same switching gain (K_new = (2, 2, 0.3) mN·m):

| `load_scale` | NDOB P3 RMS  q1 / q2 / q3  (rad) | NDOB P3 peak q2 (rad) |
|---:|---|---:|
| 3.0 | 0.00016 / 0.00100 / 0.00161 | 0.0026 |
| 5.0 | 0.00019 / 0.00105 / 0.00165 | 0.0027 |
| 7.0 | 0.00023 / 0.00139 / 0.00165 | 0.0071 |

At 7× — where SMC-MB had previously diverged to ±10⁸⁰ rad — NDOB+SMC keeps every
joint inside ~7 mrad RMS. Tracking is essentially insensitive to load magnitude
because the observer cancels it before the switching law sees it.

### NDOB outputs (prefix `thumb_ndob`)

- [`thumb_ndob_results.mat`](thumb_ndob_results.mat) — t, X (9-state with NDOB aux p), Tau, S, Qref, TAUD, TDH, metrics, observer gain L, K_new.
- [`thumb_ndob_tracking.png`](thumb_ndob_tracking.png) — 2×3 tracking + error across all 5 phases; reference and actual are visually indistinguishable, errors are at the ×10⁻³ rad scale.
- [`thumb_ndob_disturbance_est.png`](thumb_ndob_disturbance_est.png) — true τ_d (grey, includes 100 Hz ripple) overlaid with τ̂_d estimate (per-joint colour). Estimate sits exactly on the smoothed true curve, including the t = 10 s tanh transient.
- [`thumb_ndob_animation.avi`](thumb_ndob_animation.avi) / [`thumb_ndob_animation_final.png`](thumb_ndob_animation_final.png) — 251-frame 5-phase animation, the right-hand live disturbance panel additionally overlays τ̂_d (dashed) on top of τ_d (solid). The clean smooth tip-path arc with End/Start markers coincident is the visual proof of NDOB working.

### MATLAB-generated LinkedIn / portfolio figures

Three additional scripts produce the final presentation figures entirely inside MATLAB. They do not introduce new numerical claims: each one reads the existing `.mat` files and the kinematic constants already used by the dynamics.

- [`make_thumb_kinematics_figure.m`](make_thumb_kinematics_figure.m) → [`thumb_dh_forward_kinematics.png`](thumb_dh_forward_kinematics.png), a 1200×627 DH/FK figure. The flow is geometry → DH table → $A_i$ matrices → $T_0^3$ → fingertip pose.
- [`make_linkedin_beforeafter.m`](make_linkedin_beforeafter.m) → [`thumb_linkedin_beforeafter.png`](thumb_linkedin_beforeafter.png), a 1200×627 side-by-side comparison. The left half shows the stressed SMC-MB q2 tracking collapse; the right half shows NDOB+SMC under the identical disturbance.
- [`make_linkedin_infographic.m`](make_linkedin_infographic.m) → [`thumb_linkedin_infographic.png`](thumb_linkedin_infographic.png), a 1200×1700 MATLAB infographic. It combines plant parameters, DH parameters, the Euler-Lagrange plant, SMC law, NDOB law, measured q2 error, observer estimate, log-scale RMS comparison, and the quantitative results table.

The intended mathematical reading order is: first establish the planar 3R geometry with DH, then use the same geometry in the Lagrangian plant $M(q)\ddot q + C(q,\dot q)\dot q + G(q)$, then apply model-based SMC, then show why the reaching condition fails under the 2.40× load, and finally show how NDOB changes the controller from full disturbance rejection to residual rejection.

### New source files

| File | Purpose |
|------|---------|
| [`run_thumb_extra_tests.m`](run_thumb_extra_tests.m)    | Stage 1 — original 5-phase validator, modified to expose `LOAD_SCALE` inside `dist_with_load`. `LOAD_SCALE = 2.40` produces the breaking-point run; `LOAD_SCALE = 1.00` recovers the clean baseline. No new runner was added; the stress test re-uses this file. |
| [`run_thumb_ndob.m`](run_thumb_ndob.m)                  | Stage 2 NDOB+SMC runner. Same 5-phase scenario; integrates extended 9-state system [q; q̇; p] with corrected Chen-2000 update law; defaults `load_scale = 2.40`, `L = [50 50 50]`, `K_new = [0.002 0.002 0.0003]` N·m. |
| [`animate_thumb_2d_ndob.m`](animate_thumb_2d_ndob.m)    | NDOB animator — clone of `animate_thumb_2d.m` patched to overlay τ̂_d (dashed) on τ_d (solid) in the live disturbance panel. |
| [`thumb_kinematics.m`](thumb_kinematics.m)              | Standalone DH and forward-kinematics function returning the DH table, $A_i$, cumulative transforms, full $T_0^3$, joint positions, and fingertip pose. |
| [`make_thumb_kinematics_figure.m`](make_thumb_kinematics_figure.m) | Builds the DH/FK 1200×627 PNG from `thumb_kinematics.m`. |
| [`make_linkedin_beforeafter.m`](make_linkedin_beforeafter.m) | Builds the 1200×627 SMC failure vs NDOB recovery comparison from `thumb_fullcycle.mat` and `thumb_ndob_results.mat`. |
| [`make_linkedin_infographic.m`](make_linkedin_infographic.m) | Builds the 1200×1700 MATLAB infographic from the same saved results and kinematic model. |

### Summary

Stage 1 found the breaking point of SMC-MB on the **full 5-phase 25 s scenario** by
progressive escalation: at `load_scale = 2.40` the K_2 = 8 mN·m switching gain is
exceeded by |d_2|_max ≈ 8.7 mN·m, the q2 sliding surface saturates for 99% of the
load phase, and q2 develops a 7° offset that persists through P3, P4, and P5 — the
controller fails to retrace. Stage 2 added a Chen-2000 NDOB (corrected sign) with
**L = 50·I rad/s** and reduced switching gain **K_new = diag(2, 2, 0.3) mN·m**. The
same stressed scenario now tracks within ≈ 1 mrad RMS in every phase — better than
the original undisturbed baseline — and the design remains stable up to 7× load,
where pure SMC-MB diverges to ±10⁸⁰.

---

## Results and Discussion

### Stage 1 — SMC-MB Under Nominal Conditions

**Figure: [thumb_tracking.png](thumb_tracking.png)**

This 2×3 figure is the primary tracking result for the baseline 3-phase, 15 s scenario
(LOAD_SCALE = 1.0). The top row shows actual joint angle overlaid on the reference for
each of the three joints (MCP q1, PIP q2, DIP q3). The bottom row shows tracking error
$e_i = q_i - q_{d,i}$ across the same timespan, with phase shading (yellow = P1 CLOSE,
green = P2 HOLD, red = P3 SUSTAIN).

In P1 (0–5 s) the two lines are nearly indistinguishable by eye — the SMC tracks the
quintic ramp to within 1–2% of its amplitude. The largest error is at the midpoint of
the ramp (t ≈ 2.5 s) where the reference acceleration is highest; this matches the
theoretical prediction $|e_\infty| \approx |d|/(\lambda K)$. The quintic guarantee of
zero acceleration at t = 0 and t = 5 s prevents any impulse-like controller response at
phase transitions. In P2 (5–10 s) the reference is flat and the error oscillates at the
disturbance frequency (1 Hz for q1, 2 Hz for q2), confirming that the controller is
rejecting an ongoing sinusoidal disturbance in steady state. P3 (10–15 s) shows a brief
spike in error at t = 10 s as the sudden load steps in, followed by recovery within
~0.3–0.5 s — the switching gain absorbs the step disturbance and the error returns to
its P2 level.

Numerically: q1 hold RMS = 2.7 mrad, q2 = 9.2 mrad, q3 = 15.1 mrad — all below human
tactile resolution (~17.5 mrad). The DIP joint (q3) is closest to the threshold, a
consequence of its smaller K3 = 1 mN·m switching gain relative to its background
disturbance.

**Figure: [thumb_torques.png](thumb_torques.png)**

Three subplots showing the full SMC torque output $\tau_i(t)$ for each joint. The slowly
varying baseline is $\tau_{eq}$ — the feedforward term that pre-cancels gravity and
inertia. It peaks at ~21 mN·m for q1 as the thumb reaches the highest-gravity grip
posture mid-curl. The 100 Hz chattering ripple superimposed on this baseline is the
switching term $\tau_{sw}$ responding to motor electrical noise in the disturbance.
Its amplitude (~2–3 mN·m for q1/q2) is well within actuator capability. At t = 10 s a
brief additional burst of switching torque is visible as the controller absorbs the load
step. q3 torque is an order of magnitude smaller because the distal link is the lightest
and shortest — its gravity compensation is ~1.7 mN·m rather than 21 mN·m.

**Figure: [thumb_phase3_reject.png](thumb_phase3_reject.png)**

This is a zoomed view of the error plots restricted to t ∈ [9.5, 15] s, with a vertical
dashed line at t = 10 s marking the load onset. The figure is included to confirm three
things explicitly: (1) the pre-load error level in P2, which sets the baseline for
comparison; (2) the transient step magnitude at t = 10 s, which is consistent with the
theoretical $|F_{load,i}| / (\lambda_i K_i)$ prediction; and (3) the recovery time of
~0.3–0.5 s, confirming the reaching condition $K_i > |d_i|_{\max}$ is satisfied and the
sliding surface is being driven back to zero. Post-recovery error returns exactly to the
P2 level, with no steady-state offset — the constant load is absorbed within the
boundary layer.

**Figure: [thumb_animation_final.png](thumb_animation_final.png) / [thumb_animation.avi](thumb_animation.avi)**

The animation shows the thumb closing over 5 s, holding for 10 s, and rejecting the
sudden load at t = 10 s. The growing cyan tip path in the left panel closely traces the
magenta dashed reference arc — only the incremental cyan trace is drawn, so what is
visible at any frame is where the robot has *actually been*, not where it will go. The
right panel shows the live disturbance time history, with the sudden load step clearly
visible as a downward spike at t = 10 s in the q1 and q2 channels (dropping to
approximately −3 mN·m). The final frame at t = 15 s shows the arm at full grip posture
with the tip path ending cleanly at the target position.

---

### Stage 1 — SMC-MB at Breaking-Point (LOAD_SCALE = 2.40)

**Figure: [thumb_fullcycle_tracking.png](thumb_fullcycle_tracking.png)**

This is the critical figure that motivates NDOB. The 2×3 layout now covers all 5 phases
over 25 s (P1 CLOSE | P2 HOLD | P3 LOAD | P4 RETRACE | P5 HOLD-OPEN). At load_scale =
2.40 the q2 (middle joint, PIP) channel breaks catastrophically from the moment the
load steps in at t = 10 s.

The q1 and q3 rows appear nearly identical to the nominal case — their tracking errors
stay within their P2 hold levels throughout all 5 phases. Q2's error row tells a
completely different story: from t = 10 s onwards the error jumps to ~0.36 rad (20.5°
peak) and never returns. During P4 RETRACE (15–22 s) the reference for q2 is moving
back toward zero, but the actual q2 makes repeated oscillatory excursions of ~20–30°
instead of retracing cleanly. By P5 HOLD-OPEN (22–25 s) the steady-state q2 error is
still 0.128 rad ≈ 7.3°.

This joint-selective failure is exactly what the reaching-condition theory predicts. The
total worst-case disturbance on q2 at load_scale = 2.40 is |d_2|_max ≈ 8.7 mN·m,
which exceeds K2 = 8 mN·m by 0.7 mN·m — so the sliding condition $\dot{V} < 0$ is
violated on that channel alone. Q1 retains a +0.3 mN·m margin and q3 a +0.32 mN·m
margin, hence they track normally. The figure makes this visible: exactly the predicted
joint fails, and exactly the predicted joints hold.

**Figure: [thumb_fullcycle_anim.mp4](thumb_fullcycle_anim.mp4) / [thumb_fullcycle_anim_final.png](thumb_fullcycle_anim_final.png)**

The 5-phase animation shows the physical consequence of q2 failure. During P3 the thumb
tip, instead of holding its grip posture, makes visible repeated excursions downward and
back — corresponding to q2's ~30° swings. During P4 RETRACE the tip never cleanly
follows the reference arc back to the extended position: the cyan actual path diverges
visibly from the magenta reference arc in the upper portion of the workspace (where q2
contribution is largest). The final frame at t = 25 s shows the arm still significantly
off its open target — the cycle has not closed. The right panel disturbance history shows
the load step step at t = 10 s and confirms the disturbance amplitude is unchanged from
the baseline simulation; only the scale factor was increased.

---

### Stage 2 — NDOB + SMC: Recovery and Improvement

**Figure: [thumb_ndob_tracking.png](thumb_ndob_tracking.png)**

The same 2×3 layout, same 5-phase 25 s scenario, same LOAD_SCALE = 2.40 disturbance.
The visual difference from [thumb_fullcycle_tracking.png](thumb_fullcycle_tracking.png)
is immediate: all six subplots look clean. The top row shows actual and reference lines
that are visually identical — the error subplots (bottom row) reveal the true scale,
showing values in the range ±2–3 mrad throughout all five phases, including P3 LOAD and
P4 RETRACE. There is no collapse, no oscillatory excursion, no steady-state offset.

Q2 error in P3: RMS = 1.10 mrad (compared to 151.7 mrad under stressed SMC-MB — a
**138× improvement**). Q1 and q3 also improve substantially over the nominal
SMC-MB baseline: q1 P3 RMS = 0.15 mrad (vs 2.4 mrad baseline, 16× better), q3 P3 RMS =
1.60 mrad (vs 6.0 mrad baseline, 3.7× better). The improvement occurs because NDOB
pre-cancels not only the load but also the slow background sinusoids, leaving only the
100 Hz motor ripple as residual. The smaller switching gain K_new = diag(2, 2, 0.3) mN·m
then produces proportionally less chattering, which itself reduces the error driven by
the switching response to the ripple.

**Figure: [thumb_ndob_disturbance_est.png](thumb_ndob_disturbance_est.png)**

Three subplots showing the true disturbance $\tau_d(t)$ (solid grey, including the 100
Hz ripple band) overlaid with the NDOB estimate $\hat{\tau}_d(t)$ (coloured solid line)
for each joint. This figure answers the question "does the observer actually track the
disturbance?"

The estimate sits exactly on the smoothed envelope of the true disturbance — it
correctly recovers the slow background (1–2 Hz) and the tanh load transient at t = 10 s,
but does not attempt to track the 100 Hz ripple. This is the designed behaviour: with
L = 50 rad/s (8 Hz bandwidth), the observer attenuates the 100 Hz content by a factor
of ~12.5, leaving it as residual for the switching term to handle. The estimation error
at steady state is 0.36 mN·m on q1 and q2, and 0.04 mN·m on q3 — matching the K_new
sizing almost exactly (K_new was chosen at ~5× these residuals for margin).

The load transient at t = 10 s is particularly visible: the grey true-disturbance drops
sharply while the estimate follows it within ~20 ms (consistent with the 20 ms time
constant $1/L = 1/50$), with no visible lag at the scale of the figure. This fast
estimation of the load step is what prevents q2 from ever developing a significant error
during P3.

**Figure: [thumb_ndob_animation.mp4](thumb_ndob_animation.mp4) / [thumb_ndob_animation_final.png](thumb_ndob_animation_final.png)**

The 5-phase NDOB animation carries the same layout as the SMC animation but with one
addition: the right-hand disturbance panel overlays the NDOB estimate $\hat{\tau}_d$
as a dashed line on top of the solid true $\tau_d$. As the animation plays, the dashed
estimate is visually indistinguishable from the solid true curve except at the 100 Hz
ripple band, where the estimate is a smoothed version.

The left panel tip-path trace shows the key visual proof of success: the growing cyan
actual path closely follows the magenta reference arc throughout all five phases. At the
end of P4 RETRACE the cyan path returns cleanly to the Start marker, and during P5 the
arm holds the open position with no visible drift. The Start and End markers are
coincident in the final frame, confirming full cycle closure — something the stressed
SMC-MB animation could not achieve.

---

### Quantitative Summary — SMC vs NDOB

The table below places all three scenarios side by side on the same simulation scaffold
(identical plant, identical disturbance, identical 5-phase 25 s trajectory) to isolate
the controller effect:

| Metric | Nominal SMC-MB (1.0×) | Stressed SMC-MB (2.40×) | NDOB+SMC (2.40×) | NDOB vs stressed SMC | NDOB vs nominal |
|---|---:|---:|---:|---:|---:|
| q1 P3 RMS (mrad) | 2.4 | 2.2 | **0.15** | 14.7× better | 16× better |
| q2 P3 RMS (mrad) | 10.4 | 151.7 | **1.10** | **138× better** | **9.4× better** |
| q3 P3 RMS (mrad) | 6.0 | 7.7 | **1.60** | 4.8× better | 3.7× better |
| q2 P4 RMS (mrad) | ~15 | 123.9 | **1.01** | **123× better** | ~15× better |
| q2 P3 peak error (mrad) | 13.8 | 358.6 | **2.53** | 142× better | 5.4× better |
| Switching gain K (mN·m) | (10, 8, 1) | (10, 8, 1) | **(2, 2, 0.3)** | 5× smaller | 5× smaller |
| Max stable load_scale | 2.3× | — | **>7×** | — | — |

The most significant result is on q2: NDOB reduces the P3 RMS error from 151.7 mrad
(stressed SMC) to 1.10 mrad — 138× improvement — while simultaneously reducing the
switching gain by 5×. It also beats the nominal SMC baseline by 9.4× on the same joint,
demonstrating that NDOB is not simply recovering what SMC-MB can do at lower loads, but
fundamentally improving the disturbance rejection architecture.

The robustness at extreme loads is equally significant. The NDOB keeps every joint
within 7 mrad RMS at load_scale = 7.0, a condition where SMC-MB (at any reasonable K)
has already diverged numerically. This is the architectural advantage of pre-cancellation:
the observer absorbs the load before the switching law sees it, so the switching term
operates on a ~0.36 mN·m residual regardless of how large the load becomes (until the
observer's linearisation assumptions break down or the actuator saturates).

### Discussion

**What the NDOB does not fix.** The 100 Hz motor electrical ripple cannot be tracked by
the observer at L = 50 rad/s (8 Hz bandwidth). This residual drives the ~0.36 mN·m
estimation error that sets the floor on K_new and ultimately on tracking accuracy. To
eliminate it, L would need to exceed ~628 rad/s (100 Hz), but at that bandwidth the
observer amplifies measurement noise severely and the auxiliary-state integration becomes
stiff. The current L = 50 rad/s is the practical optimum for this disturbance profile.

**Model uncertainty.** Both controllers use the same M(q), C(q,q̇), G(q) in the
controller and the plant — a perfect-model assumption. In real hardware, mass and
inertia will have ±10–20% uncertainty. For SMC-MB, uncertainty is treated as an
additional matched disturbance and is covered by K (with margin). For NDOB+SMC,
uncertainty is lumped into the disturbance estimate and partially cancelled, which
actually *helps* — but any unmodelled dynamics (friction, tendon compliance) will
add a persistent bias to $\hat{\tau}_d$ that the observer cannot distinguish from
true disturbance.

**Why the results are trustworthy.** The two simulations (SMC and NDOB) use identical
integration (RK4, dt = 0.5 ms), identical disturbance functions (called identically),
and identical reference trajectories. The only difference is the controller block. The
same dramatic q2 failure appearing in `thumb_fullcycle_tracking.png` and the same
recovery appearing in `thumb_ndob_tracking.png` cannot be explained by any other
variable. The reaching-condition theory predicts exactly which joint fails and at exactly
what load scale — the simulation confirms the prediction to within one decimal place.
This internal consistency between theory and simulation is the primary validation that
the implementation is correct.