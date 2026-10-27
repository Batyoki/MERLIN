# MERLIN — Math fixes, control novelty and comparison (List 2)

Each equation below is implemented in `ros_ws/src/controller/controller/`, and the file is named.
Numbers come from `experiments/bilateral_sim/results/results.md` (MuJoCo, RX200 → VX300s).
**They are simulation results.** The hardware protocol in §4 is what produces the paper's numbers.

Notation: n = 5 active joints [waist, shoulder, elbow, wrist_angle, wrist_rotate]; Δt = 20 ms;
dz_δ(x) = sign(x)·max(|x|−δ, 0); F_α{x}[k] = α x[k] + (1−α) F_α{x}[k−1].

---

## 1. Equation-by-equation fixes

### Eq. 1 — state and motor torque (`arm_interface.py`)
Old: τ_raw = k_t ⊙ I. That is wrong by 10³ (xs_sdk reports mA) and by ×2 on joints with shadow motors.
The filter coefficient is also printed as α_v = 0.3 while the code used 0.15.

$$
\dot q[k] = F_{\alpha}\!\left\{\tfrac{q[k]-q[k-1]}{t_k-t_{k-1}}\right\},\qquad
\tau_{mot}[k] = n_m \odot k_t \odot 10^{-3}\, I_{mA}[k],\qquad \alpha = 0.3
$$
(use the message timestamps, not a nominal Δt).

### Eq. 2/3 — external torque (`observer.py`)
Old: τ̂_ext = τ_m − C(q, q̇) + G(q). The sign of G is wrong, C is a matrix used as a vector,
and the omitted M q̈ dominates on a position-controlled arm (±12 N·m RMS in free motion in simulation).

Dynamics convention, used everywhere:
$$ M(q)\ddot q + C(q,\dot q)\dot q + G(q) = \tau_{mot} + \tau_{ext} $$

Slave estimate, a generalised-momentum observer (no q̈; De Luca & Mattone 2003):
$$
p = M(q)\dot q,\qquad
r[k] = K_O\Big(p[k]-p[0]-\sum_{i<k}\big(\tau_{mot}+C^\top\dot q - G + r\big)[i]\,\Delta t\Big)
\;\Rightarrow\; \dot r = K_O(\tau_{ext}-r)
$$
with K_O = 15 rad/s. The observer takes unfiltered q̇ and τ_mot (filtered q̇ caused a limit cycle).
Put the static residual τ̂ = G + C q̇ − τ_mot in the paper only as the quasi-static special case.

**Master:** no estimate. In current mode the measured current equals the command, so a residual
from it is not an external torque (this is the deferred novelty item 1: an observer driven by the
commanded torque).

### Eq. 4 — master impedance (`misa_mpc_controller.py`)
Old: τ_m,cmd = G_m − D_md q̇_m + τ^tot_ext,m with τ^tot = τ̂_ext,m + τ_f + τ_p.
Adding τ̂_ext,m (the master's own current) is unity positive feedback: τ[k] = τ[k−1] + …

$$
M_m\ddot q_m + C_m\dot q_m + G_m = \tau_{m,cmd} + J_m^\top F_h,\qquad
\tau_{m,cmd} = \hat G_m(q_m) - D_{md}\,\dot q_m + \gamma\,\tau_f
$$
where Ĝ_m is evaluated at the raw master angles (URDF convention), γ ∈ [0, 1] is the tank gate (Eq. 8),
and τ_m,cmd is saturated at about 50 % of stall torque, then converted to current per motor.

### Eq. 5 — slave admittance (`admittance.py`)
Old: explicit Euler with M_sd equal to the robot inertia, plus an ε = 10⁻⁴ regulariser (unstable at the wrist).
The text also mixes M_s / M_sd.

$$
M_{sd}\ddot q_e + D_{sd}\dot q_e + K_{sd} q_e = \mathrm{dz}_\delta(\hat\tau_{ext,s}),\qquad
q_{s,ref} = S\,q_m + o + q_e
$$
M_sd, D_sd, K_sd are diagonal design values (ω_n = √(K/M) ≈ 10 rad/s, ζ = 1), discretised exactly:
$$
x[k+1] = e^{A\Delta t}x[k] + \Big(\int_0^{\Delta t}e^{As}ds\,B\Big)\,\mathrm{dz}_\delta(\hat\tau_{ext,s}[k]),\quad
A=\begin{bmatrix}0&1\\-K/M&-D/M\end{bmatrix},\;B=\begin{bmatrix}0\\1/M\end{bmatrix}
$$
This is stable for any Δt. The operator-force term −k_of τ̂_ext,m is removed until the master observer exists.

### Eq. 6 — force/position feedback (`force_mapping.py`, `misa_mpc_controller.py`)
$$
\tau_f = k_{of}\,\Gamma(q_m,q_s)\,\mathrm{dz}_\delta(\hat\tau_{ext,s}) \;-\; k_{om}K_{md}\,(S q_m + o - q_s),\qquad
\Gamma = J_m^\top S_f\,(J_s^\top)^{+}_{\lambda}
$$
with (J_sᵀ)⁺_λ = J_s(J_sᵀJ_s + λ²I)⁻¹.

**Channel count:** with the master estimate removed, the implemented scheme is **3-channel**
(position forward, position back, force back). Call it "four-channel" only once the operator-force
channel exists.

### Eq. 7 — slave MPC (`mpc_optimizer.py`)
Old: torque-input MPC for a position-controlled arm. The τ₀* the paper says is applied was never applied.
The velocity decision variables don't enter the dynamics, so dq* ≡ 0 (verified on the original code).
The cost terms also differ between paper and code.

Per joint i, identified position-loop model (`tools/identify_position_loop.py`):
$$
\ddot q = \omega_i^2(u-q) - 2\zeta_i\omega_i\dot q \;\xrightarrow{\text{ZOH}}\; x[k+1]=A_ix[k]+B_iu[k]
$$
$$
\min_{u_{0:N-1}}\sum_{k=1}^{N-1}Q\,(\hat q_k + \hat d - r_k)^2 + P\,(\hat q_N+\hat d-r_N)^2 + R\sum_{k=0}^{N-1}(u_k-u_{k-1})^2
$$
$$
\text{s.t. } q_{min}\le u_k\le q_{max},\quad |u_k-u_{k-1}|\le\Delta u_{max},\quad |\hat{\dot q}_k|\le\dot q_{max}
$$
- r_k = q_s,ref + k Δt S q̇_m (velocity preview).
- d̂ = LPF(q − q̂) is the offset-free disturbance (it absorbs the static gravity sag).
- N = 10. Solved by ADMM (ρ = 0.3, 60 iterations): 0.9 ms mean per tick for five joints; the worst single solve in a 24 s run was ≈ 12 ms (shared CPU, includes OS jitter), still inside the 20 ms period.

### Eq. 8 — energy tank, new (`passivity.py`)
$$
P_{out}=\tau_f^\top\dot q_m,\quad P_{in}=\eta\,(\dot q_m^\top D_{md}\dot q_m + \dot q_e^\top D_{sd}\dot q_e),\quad
T[k+1]=\min\!\big(T_{max},\,T[k]+\Delta t(P_{in}-\gamma P_{out})\big)
$$
$$
\gamma=\begin{cases}1 & T+\Delta t(P_{in}-P_{out})\ge T_{min}\\[2pt] \max\!\Big(0,\dfrac{T+\Delta tP_{in}-T_{min}}{\Delta t\,P_{out}}\Big) & \text{otherwise}\end{cases}
$$
**Proposition:** T[k] ≥ T_min for all k, so
Σ_k γP_out Δt ≤ T[0] − T_min + Σ_k P_in Δt.
The force-feedback port is therefore passive with respect to the energy the controller dissipates,
whatever the estimation error or the non-power-preserving Γ. The proof is one line from the update law.

---

## 2. Control novelty (items 2–5; item 1, the master observer, deferred)

| # | Contribution | What exists now | Evidence (simulation) | What's still needed for a strong claim |
|---|---|---|---|---|
| 2 | Passivity of sensorless force reflection via an energy tank | `passivity.py`, Eq. 8 + proposition | k_of = 1.5 without tank: RMSE 0.197 rad, wall peak 58 N. With tank: 0.054 rad, 59 N. At the default k_of = 0.3 the tank never activates (identical results). | **The single master-side tank doesn't give delay robustness.** At 60 ms with k_of = 0.6 it fails with or without the tank (0.30 rad). Needs a two-port TDPA / energy exchange between master and slave tanks (Artigas 2011; Ferraguti 2015). |
| 3 | Force reflection between kinematically different arms, Γ = J_mᵀ S_f (J_sᵀ)⁺ | `force_mapping.py` | At k_of = 0.6: identity map peaks 37 N vs 12 N with the Jacobian map (earlier run). **At k_of = 0.3: no measurable difference** (force corr 0.69 vs 0.76). | A Cartesian task (pushing along a direction that loads joints differently on RX200 vs VX300s) and S_f = position-scale ratio (impedance-transparent). Otherwise claim it as a design choice, not a result. |
| 4 | Correct constrained, offset-free MPC for a position-controlled slave | `mpc_optimizer.py` | RMSE 0.011 vs 0.026 rad without MPC. Lag **0 ms** (< 10 ms) vs 20 ms (no MPC) and 32 ms (unilateral/P-P/P-F). Lag-compensated RMSE 0.007. Solve 0.9 ms. | Identify ω, ζ on the real arm. Show the constraint activity (Δu, q̇) in a figure. |
| 5 | Tank-aware MPC (the passivity constraint inside the QP) | not implemented | — | Would turn #2 + #4 into one formulation. Optional; only if time allows. |

**Answer to "infrastructure, not control" (reviewer):** the contribution becomes:
- a sensorless bilateral controller for a *current-controlled master* and *position-controlled heterogeneous slave*;
- a stated passivity property (Eq. 8);
- an MPC formulated for the actuator interface that actually exists (position loop, with offset-free compensation of its gravity sag);
- an ablation that isolates each part.

The testbed is how it is evaluated, not the contribution itself.

---

## 3. Comparison (simulation, `experiments/bilateral_sim/results/results.md`)

Scenario: 12 s free motion (all joints, sinusoids), then 12 s with two pushes into a 2000 N/m wall.
"Felt" is the non-gravity force the master applies to the operator's hand.
All controllers use k_of = 0.3 unless stated.

| Controller | RMSE pooled | RMSE free | max | lag | lag-comp RMSE | felt/true peak | force corr |
|---|---|---|---|---|---|---|---|
| **Original code (as in repo)** | 0.161 | 0.221 | 0.530 | 336 ms | 0.105 | 1.3 / 1.3 N | −0.55 |
| Original, units fixed only | 0.147 | 0.204 | 0.491 | 332 ms | 0.085 | 2.2 / 1.8 N | 0.70 |
| B1 Unilateral (xsarm_puppet) | 0.031 | 0.025 | 0.154 | 32 ms | 0.018 | 0.2 / 13.2 N | −0.40 |
| B2 Position–position | 0.026 | 0.025 | 0.135 | 32 ms | 0.018 | 0.4 / 8.2 N | 0.39 |
| B3 Position–force (2-ch) | 0.024 | 0.026 | 0.157 | 32 ms | 0.018 | 1.3 / 3.4 N | **0.94** |
| **Proposed** | **0.011** | **0.007** | **0.123** | **0 ms** (< 10 ms) | **0.007** | 1.2 / 4.3 N | 0.73 |

Ablations of the proposed controller: no MPC 0.026 rad / 20 ms; no tank (identical at k_of = 0.3); identity map 0.011 rad, corr 0.70; N = 3 0.011 rad. (Seed 0, ADMM ρ = 0.3; see `EVALUATION_METHODOLOGY.md` for the 5-seed table.)

Delay (both directions):

| Delay | Proposed | P–P | P–F | Proposed, k_of = 0.6 |
|---|---|---|---|---|
| 20 ms | 0.024 | – | 0.032 | – |
| 60 ms | **0.040** | 0.048 | 0.056 | 0.303 (unstable) |

What this shows:
1. **The paper's Table I is explained by code bugs.** The original controller's 0.39 s smoothing filter gives about 0.16–0.22 rad RMSE and about 0.34 s lag in simulation. It's the same mechanism as the 0.36 rad measured on hardware (the hardware motion was faster).
2. **The fixed system tracks about 10× better than the original and 1.5–2× better than standard baselines.** The gain comes from the MPC's preview and lag compensation.
3. **The force channel now reproduces the contact.** Correlation with the true wall force is 0.76 (0.94 for plain P-F at the same gain). The operator stops the slave at about 4 N instead of 13 N (unilateral).
4. **Honest negatives:**
   - P-F has better force correlation than the proposed controller. Its position-coupling term adds a velocity-dependent component.
   - The Jacobian map shows no benefit at k_of = 0.3.
   - The tank alone doesn't fix delay.

   Report these. Reviewers trust a paper more when it shows where the method doesn't help.

---

## 4. Hardware protocol that answers each reviewer point

1. **"Force path never measured."**
   - Ground truth without a force/torque sensor: hang known masses (0.1–0.5 kg) from the slave gripper (τ_true = J_sᵀ m g), and push against a kitchen scale or spring scale.
   - Report τ̂_ext,s against τ_true (RMSE, bias, bandwidth), and the force rendered on the master (log τ_f, map with (J_mᵀ)⁺) against the scale reading.
   - This also calibrates k_t (`CODE_FIXES.md`, hardware step 2).
2. **"No baseline/ablation."** Same task, same operator, ≥ 5 trials each:
   - controllers: unilateral, P-P, P-F, proposed, plus ablations (no MPC, no tank, identity map);
   - all selectable with `control_mode:=…` and YAML flags; nothing else changes.
3. **"RMSE issue"** (`tools/evaluate_tracking.py`):
   - report the pooled RMSE, √(1/(KJ) Σ e²), per-joint RMSE/MAE/max, lag by cross-correlation, and lag-compensated RMSE;
   - state that the error is w.r.t. the mapped master angle;
   - report free-motion and contact segments separately (in contact, q_e is intended compliance; use `--ref q_ref` for that).
4. **Figures:**
   - replace the Fig. 3 scatter with per-joint time series (`joint_tracking.png` style);
   - replace Fig. 4 (PD on a toy plant) with the force-transparency plot (`comparison.png`, middle panel: felt vs true) and the tank-energy trace;
   - Fig. 2 caption: say which pairs were run.
5. **Prior work to add:**
   - De Luca & Mattone 2003 and Haddadin 2017 (momentum observer);
   - Ohnishi / Murakami (DOB/RTOB; the latter is already cited);
   - Hannaford & Ryu 2002, Artigas 2011 (TDPA);
   - Franken 2011, Ferraguti 2015 (energy tanks);
   - Lawrence 1993 (four-channel transparency);
   - one MPC-for-teleoperation reference;
   - the third-party code you build on (manipulator_teleop, mink).

## 5. Text fixes in the draft
- Conclusion: 0.3418 → the pooled value. 0.3418 is the mean of per-joint RMSEs; pooled is 0.3583. MAE 0.2267 is a correct pooled mean.
- Drop "impressive" and "tightly synchronized" for 0.36 rad (≈ 20°) RMSE and 1.56 rad maximum.
- "Kinematic error accumulates along successive links" applies to Cartesian error, not joint-space error. The wrist error came from rate limits and the lag filter.
- "Delays … inherent to constrained MPC … within 20 ms": the delay came from the 0.39 s filter. The MPC solve was 17–48 ms, i.e. over budget.
- "Only τ₀* is applied": false for the old code; true for u₀* in the new formulation.
- §3.4 / Fig. 1: HDF5, ray embeddings, tactile/haptic, collision checks, E-stop. Either implement them or soften them to "supports"/"planned". The recorder currently writes synthetic joint data (CODE_FIXES D1).
