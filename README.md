# Obstacle Avoidance in Self‑Driving Cars using Nonlinear Model Predictive Control (Python)

A clean, end‑to‑end Nonlinear Model Predictive Control (NMPC) notebook that plans and controls a vehicle modeled as a kinematic bicycle to follow a lane and avoid static obstacles. Implemented from scratch in Python with CasADi and IPOPT, using direct collocation for fast, reliable optimization.

> **Notebook**: `Model_NMPC.ipynb`

---

##  What’s inside

* **Kinematic bicycle model** with states `[x, y, ψ, v]` and inputs `[a, δ]`.
* **Direct collocation (Legendre, degree d = 3)** transcription of dynamics.
* **NMPC with soft obstacle avoidance** via distance penalties + safety margin.
* **Input rate smoothing** to produce physically plausible acceleration and steering profiles.
* **Multiple scenarios** ("Tasks") from basic motion to single/multi‑obstacle avoidance.
* **Plots** of trajectories, headings, input histories, and lane/obstacle geometry.

---

## Problem formulation (high level)

**Dynamics.** Kinematic bicycle model

* States: `x` (m), `y` (m), `ψ` (rad), `v` (m/s)
* Controls: `a` (m/s²), `δ` (rad)
* Slip angle: `β = atan( lr/(lf+lr) * tan(δ) )`
* Continuous‑time RHS:
  $\dot x = v\cos(ψ+β),\quad \dot y = v\sin(ψ+β),\quad \dot ψ = \frac{v}{lr} \sin β,\quad \dot v = a$

**Constraints (examples used in the notebook):**

* Lane bounds on `y` (keeps the vehicle within the 1‑lane corridor).
* Input bounds on `a` and `δ`.
* Optional heading/speed limits.
* Soft obstacle clearance using a **safety radius** (obstacle radius + vehicle circumscribed radius + margin).

**Cost (tunable):** combination of

* Lateral tracking to a lane centerline (e.g., `y → y_ref`),
* Progress term (encouraging speed),
* Steering magnitude regularization,
* **Obstacle penalty** when inside safety radius,
* **Rate penalties** for `Δa` and `Δδ` to smooth actuation.

All terms are accumulated at collocation points and weighted; see the notebook cells for the exact weights per task.

---

## Scenarios (Tasks) implemented

The notebook is organized into small, self‑contained sections:

1. **Task 1 — Kinematic bicycle rollout**
   Simple forward simulation & visualization on a straight highway cross‑section, showing ego heading arrows and lane boundaries.

2. **Task 2 — NMPC: go‑fast while staying feasible**
   NMPC with objective dominated by progress (maximize speed) subject to dynamics and bounds.

3. **Task 3 — NMPC: lane tracking**
   Adds lateral tracking (`y → y_ref`) and steering regularization for smoothness, still no obstacles.

4. **Task 4 — NMPC: obstacle avoidance**

   * **N = 3**: short horizon; demonstrates late but feasible avoidance with soft penalties and rate constraints.
   * **N = 10**: longer horizon; anticipatory behavior with earlier, gentler steering.
   * **Multiple obstacles**: list of circles `(x, y, r)` (e.g., `(100, 2.0, 1.0)`, `(130, 5.5, 1.2)`, `(170, 3.0, 0.8)`)—vehicle threads a safe path while staying near the lane center.

Each task section plots: trajectory in **(x, y)**, obstacle/safety circles, lane centerline, headings over time, and the control inputs `a(t)` and `δ(t)`.

---

## Key parameters & where to change them

Open `Model_NMPC.ipynb` and look near the top of each task cell.

* **Vehicle geometry**: `lf`, `lr`, `car_length = lf + lr`, `car_width`.
  Circumscribed radius `r_car = sqrt(car_length² + car_width²)/2`.
* **Sampling & horizon**: `dt = 0.05 s`, `Nsim` (simulation steps), `N` (prediction horizon), `d = 3` (collocation degree).
* **Lane**: `y_min = 0.0`, `y_max ≈ lane_width − r_car`, `lane_center`.
* **Obstacles**: list of `(x, y, r)` plus **margin** `obs_margin` to tune conservativeness.
* **Bounds**: acceleration `a ∈ [a_min, a_max]`, steering `δ ∈ [δ_min, δ_max]`, plus optional `v`/`ψ` limits.
* **Weights** (typical starting point in obstacle tasks):
  `w_y = 20`, `w_delta = 20`, `w_obs = 1000`, `w_acc_rate = 100`, `w_steer_rate = 50`.
  Increase `w_obs`/`obs_margin` to be more conservative; increase rate weights to reduce jerky actuation.
* **Initial state**: e.g., `x0 = 0`, `y0 = 2.5`, `ψ0 = 0`, `v0 = 120/3.6` (m/s).

> Tip: Horizon `N` trades computational load vs. look‑ahead. Try `N = 3, 5, 10`. Longer horizons generally yield gentler avoidance but cost more CPU.

---

## Tech stack

* **Python**
* **CasADi** (symbolic and automatic differentiation)
* **IPOPT** (nonlinear solver via CasADi: `ca.nlpsol('solver', 'ipopt', ...)`)
* **NumPy**, **Matplotlib** (simulation & plotting)

---

##  Quick start

1. **Create an environment** (recommended)

   ```bash
   python -m venv .venv
   source .venv/bin/activate   # Windows: .venv\\Scripts\\activate
   ```
2. **Install dependencies**

   ```bash
   pip install casadi numpy matplotlib
   ```
3. **Run the notebook**

   * Open `Model_NMPC.ipynb` in Jupyter / VS Code.
   * Run cells in order. Each *Task* section is independent and produces its own plots.

> If you prefer a script: export the task cell to a `.py` and wrap the simulation in a `main()`; keep solver creation *outside* the sim loop and warm‑start with the previous solution.

---

##  What the plots show

* **Trajectory (x–y)** with the lane, obstacle(s), and safety margins.
* **Heading & speed** over time.
* **Inputs** `a(t)` and `δ(t)` and their smoothness.
* For multiple obstacles, selected snapshots of the vehicle’s circumscribed circle to visualize clearance.

---

## Implementation notes

* **Direct collocation** with Legendre points (degree `d = 3`): state polynomials per interval, continuity constraints, and collocation residuals are added to the NLP.
* **Soft obstacle constraints**: cost increases inside the safety radius (`r_obs + r_car + margin`). This avoids infeasibility; raise `w_obs` if the optimizer tries to “cut corners”.
* **Rate constraints**: bound/penalize `Δa`, `Δδ` between consecutive control moves to avoid aggressive commands.
* **Warm starts**: the solution vector is shifted forward in time for faster convergence at each MPC step.

---

## Reproducibility & tuning

* Solves use IPOPT’s defaults via CasADi. For tighter convergence, tune IPOPT options (tolerances, max\_iters) where the solver is constructed.
* Start with smaller `N` and modest weights; then increase `N` and obstacle weights once basic tracking works.
* If you see oscillations, increase rate penalties and/or reduce lateral weight `w_y`.

---

##  Status & roadmap

* [x] Kinematic bicycle rollout
* [x] NMPC with lane tracking
* [x] Single obstacle avoidance (short/long horizons)
* [x] Multiple obstacles

---

##  Acknowledgments

* **CasADi** and **IPOPT** for the optimization backbone.
* Classic references on the kinematic bicycle model and MPC.

---

## 💬 Questions / feedback

Issues and PRs are welcome! If something doesn’t run on your setup, please open an issue with your OS, Python version, and full error message.

