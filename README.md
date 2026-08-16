# kin_humanoid

MuJoCo-based **kinematic whole-body walking control** for the DYROS Tocabi v2 humanoid.

This project is a restructuring of the kinematics-based humanoid control framework
originally developed in 2019, rebuilt around MuJoCo.

Keyboard-teleoperated walking built as a clean, swappable pipeline:

```
FootstepGenerator ─▶ PreviewController ─▶ WholeBodyIK ─▶ joint targets ─▶ robot
 (footsteps+ZMP)    (real-time 1s window)  (priority DLS)   (position servo)
```

Model information (kinematics/dynamics) and robot I/O are both behind abstract
base classes so the backend can be swapped without touching the controller:

- **Model**: `RobotModel` ← `MujocoModel` (active) / `RbdlModel` (skeleton, for later)
- **I/O**:  `RobotIO`   ← `SimIO` (active, MuJoCo) / `RealIO` (skeleton, real robot)
- **ROS**:  interface *location* only (`io/RosInterface.h`), implementation excluded.

Simulation runs in MuJoCo; linear algebra uses Eigen; RBDL remains linked for the
future RBDL model backend.

### Controls (viewer)

| Key | Action |
|-----|--------|
| `H` | go to the walk-ready posture, then start the whole-body controller |
| `Space` | toggle walking on/off (only after `H`) |
| `W` / `S` | walk forward / backward |
| `A` / `D` | strafe left / right (게걸음, crab walk) |
| `Q` / `E` | turn left / right |
| `X` | stop |
| `G` | toggle the real-time walking graphs |

### Startup flow (no jump at start)

The controller boots in **Idle**: it just holds the initial pose stiffly (the
footstep generator / preview / IK are **not** running), so there is no start-up
transient. Press **`H`** to smoothly crouch into the walk-ready posture (~2.5 s);
on arrival the whole-body controller starts and balances in place (**Active**).
Then **`Space`** begins walking. This staged bring-up removes the program-start
and walk-start jolts.

### Walking graphs (`G`)

A built-in real-time plot overlay (MuJoCo `mjvFigure`, no external deps) shows two
time-series panels — **Lateral (Y)** and **Sagittal (X)** — each overlaying five
signals so the walk-start transient is easy to see: **footstep** (support), **ZMP
reference**, **COM reference**, **measured ZMP** (ground CoP), **measured COM**.

> **Status**: stands stably and walks (forward / strafe) at conservative speed.
> This is an **open-loop kinematic** walker (no ZMP/FT balance feedback yet), so
> the lateral inverted-pendulum mode is only marginally stable — long continuous
> walks eventually tip. A balance stabilizer (DCM/ZMP or FT-based ankle strategy)
> is the documented next step; a hook is present (`HumanoidController::setStabilizer`,
> experimental/off by default). See §10.

---

## 1. Overview

| Item | Details |
|------|---------|
| Simulator | MuJoCo 3.3.x (`/opt/mujoco/mujoco`) |
| Robot | [DYROS Tocabi v2](https://github.com/saga0619/dyros_tocabi_v2) — **MuJoCo model only** |
| Kinematics / dynamics | [RBDL](https://github.com/rbdl/rbdl) + urdfreader addon |
| Linear algebra | Eigen3 |
| Language / standard | C++17, g++ |
| Rendering | GLFW + GLEW + OpenGL |

**Robot specs (from the compiled model):** `nq=40`, `nv=39`, `nu=33` (torque
motors), total mass ≈ 95.6 kg, timestep 0.5 ms, gravity `[0, 0, -9.81]`.

---

## 2. Directory layout

```
kin_humanoid_ws/
├── src/
│   ├── main.cpp                     # entrypoint: wire SimIO + MujocoModel + controller, run loop
│   ├── config/
│   │   └── WalkingConfig.{h,cpp}    # runtime config struct + JSON loader (values live in config/walking_config.json)
│   ├── util/
│   │   ├── MathUtil.h               # rotations/quaternions, cycloid/cubic, DARE, DLS pseudo-inverse
│   │   └── RobotDefs.h              # joint/body indices, Side enum, model dims (nq/nv/nu)
│   ├── model/
│   │   ├── RobotModel.h             # abstract: FK / body Jacobian / COM / COM-Jacobian
│   │   ├── MujocoModel.{h,cpp}      # MuJoCo backend (active)  — analytic model queries
│   │   └── RbdlModel.{h,cpp}        # RBDL backend (skeleton, TODO)
│   ├── io/
│   │   ├── RobotIO.h                # abstract: read state / write joint targets / velocity cmd
│   │   ├── SimIO.{h,cpp}            # simulator backend (active) — wraps MujocoEnv
│   │   ├── RealIO.{h,cpp}           # real-robot backend (skeleton, TODO)
│   │   └── RosInterface.h           # ROS location only (implementation excluded)
│   ├── controller/
│   │   ├── WalkingState.h           # gait phase / pose types
│   │   ├── FootstepGenerator.{h,cpp}# velocity cmd -> footsteps + ZMP preview window
│   │   ├── PreviewController.h      # real-time 1-second-window LIPM ZMP preview
│   │   ├── WholeBodyIK.{h,cpp}      # Nakamura priority DLS + support-consistent base slaving
│   │   └── HumanoidController.{h,cpp}# orchestrator (footstep->preview->WBIK->joints)
│   └── simulator/
│       └── MujocoEnv.{h,cpp}        # MuJoCo load/step/render + mouse camera + keyboard teleop
├── test/
│   └── headless_walk_test.cpp       # headless verification harness (no viewer)
├── model/dyros_tocabi_v2/           # submodule (mujoco_model + meshes only)
├── prj/ , .vscode/ , README.md
```

### Dependency management

- **Robot model**: only the *MuJoCo model* of Tocabi is needed, not the whole
  repository. It is added as a submodule with sparse-checkout limited to
  `tocabi_description/mujoco_model` (MJCF) and `tocabi_description/meshes` (STL).
- **Kinematics / linear algebra**: RBDL (`librbdl`, `librbdl_urdfreader`) and
  Eigen are used from their system installs (`/usr/local`, `/usr/include/eigen3`),
  so there is no in-tree dependency build step.

---

## 3. Build

### 3.1 One-time setup

```bash
# (1) Fetch submodules
git submodule update --init --recursive

# (2) Keep only the robot's MuJoCo model (sparse-checkout, if a full clone came in)
cd model/dyros_tocabi_v2
git sparse-checkout set tocabi_description/mujoco_model tocabi_description/meshes
cd ../..
```

### 3.2 Build the project

- **VS Code**: run the `build (kin_humanoid)` task (`Ctrl+Shift+B`)
- **Terminal**: run the same g++ command as in `.vscode/tasks.json` to produce `./main`

Key link flags: `-lmujoco -lglfw -lGLEW -lGL -lrbdl -lrbdl_urdfreader`
(includes: `/opt/mujoco/mujoco/include`, `/usr/include/eigen3`, `/usr/local/include`)

---

## 4. Run

```bash
./main            # default model: model/.../mujoco_model/dyros_tocabi.xml
./main <model.xml>  # or specify another MJCF
```

On launch:
1. Load the Tocabi MuJoCo model.
2. Reset to **keyframe 0 (`front`)** — a standing pose with pelvis at 0.92983 m
   and slightly bent knees.
3. Run an Eigen/RBDL self-test once to verify the libraries link and run.
4. Switch all joint actuators to **position control mode** and hold the current
   (keyframe) pose.
5. Open the GLFW viewer and run the simulation loop.

> **Note**: every joint is position-controlled, so the robot **holds its standing
> pose** in place (verified: ~6 mm pelvis drift, < 1.5° joint error over 2 s).
> Motion is produced later by the kinematics-based controller writing desired
> joint angles into `d->ctrl`.

### Note on the initial pose

With the default `qpos0` (all zeros) the pelvis spawns at the origin (z=0), so
the feet penetrate the ground by about 0.96 m. MuJoCo then pushes the penetration
out with a huge instantaneous contact force and launches the robot upward. To
avoid this, the state is reset to the keyframe (`front`) right after loading
(`MujocoEnv::load` -> `resetToKeyframe(0)`).

---

## 5. Viewer (mouse / rendering)

### Mouse controls

| Input | Action |
|-------|--------|
| Left-drag | Rotate camera (`Shift`: horizontal orbit) |
| Right-drag | Pan / translate (`Shift`: horizontal pan) |
| Middle-drag / Wheel | Zoom |

### Geometry group visibility

MuJoCo's default option is `geomgroup = [1,1,1,0,0,0]`, so group 3 is off and the
ground was invisible by default. `MujocoEnv::initViewer` adjusts the render set:

| Group | Contents | Shown |
|-------|----------|-------|
| 1 | Robot visual meshes (class `viz`) | yes |
| 2 | Collision primitives (class `cls`, gray boxes) | hidden |
| 3 | Ground plane (`geom "ground"`, 10x10 m) | shown |

Toggle any group at runtime with `MujocoEnv::setGeomGroupVisible(group, visible)`.

---

## 6. Actuation & control mode

The Tocabi MJCF ships with 33 torque `motor` actuators (one per joint). For
**kinematics-level control** those are switched, on the loaded model in memory
(no XML/submodule edit), into **position servos** by `MujocoEnv::setJointPositionMode(kp, kv)`:

```
force = kp * (ctrl - q) - kv * qdot     // ctrl becomes the desired joint angle [rad]
```

- The original per-joint `ctrlrange` (torque limits) is moved to `forcerange`, so
  realistic torque saturation is preserved while `ctrl` now carries a position.
- `MujocoEnv::holdCurrentPose()` seeds `d->ctrl` with the current joint angles, so
  the robot holds its pose from the first step.
- Default gains: `kp = 2000`, `kv = 100` (uniform). Tune in `main.cpp`.

This is the interface every kinematics-level controller uses: compute desired
joint angles `q_des` and write them to `d->ctrl` each control cycle; MuJoCo's
built-in servo turns them into joint torques.

---

## 7. Debugging (VS Code)

Open `prj/kinematics_humanoid.code-workspace` to load the project:

| Configuration | Description |
|---------------|-------------|
| Debug kin_humanoid (gdb) | Debug this project |

The configuration runs its build task first and injects the MuJoCo and RBDL
library paths into `LD_LIBRARY_PATH`.

---

## 8. Note on RBDL for kinematics

| Use case | Fit |
|----------|-----|
| Forward/inverse kinematics, Jacobians | Ideal. RBDL provides fast recursive FK, point/body Jacobians, and CoM kinematics out of the box. |
| Inverse/forward dynamics, mass matrix | Very suitable. RNEA / CRBA / ABA are built in, useful for gravity compensation and dynamics-consistent kinematic control. |
| Model source | The MuJoCo MJCF is the simulation source of truth; the RBDL model is built separately (URDF via the urdfreader addon, or constructed in code) and kept consistent with it. |

Adopted for the kinematics and (later) dynamics-consistent control paths. The MuJoCo
model remains authoritative for simulation; RBDL supplies the analytical
kinematics/dynamics the controller queries.

---

## 9. Control pipeline (how it works)

Each control cycle (500 Hz), `HumanoidController::update()`:

1. **FootstepGenerator** — from the velocity command it maintains a rolling,
   alternating footstep plan (only the current step + swing target are committed;
   the rest is regenerated every tick so the plan always reflects the latest
   command). It emits the current support/swing feet, the swing-foot cycloid
   target, and a **ZMP reference sampled over the next 1 second**.
2. **PreviewController** — LIPM ZMP-preview (Kajita). The optimal gains (DARE) are
   computed **once**; each tick it consumes the sliding 1-second ZMP window and the
   current state and outputs the COM reference (position + velocity). This is the
   "real-time 1-second-window" preview (requirement 2).
3. **WholeBodyIK** — Nakamura **priority DLS**. The stance foot is enforced as the
   top priority *by construction*: the base velocity is slaved to keep the support
   foot fixed (`v_base = -Jb⁻¹ Jj v_joint`, *support-consistent reduction*), so all
   lower tasks become functions of joint velocity only. Then, in priority order:
   **support foot > COM > swing foot > hands > pelvis orientation**, solved by
   successive null-space projection with damped least squares. COM uses the
   **COM Jacobian** (requirement 5). When the support/swing feet swap, only the
   stance/swing Jacobians change, so priorities adjust automatically (requirement 4).
4. **Internal feed-forward integration** — the resulting joint velocities integrate
   an internal desired state (the base propagated by the same support-foot
   constraint), which keeps the stance foot exactly planted with no kinematic drift.
   The joint part is sent to the position servos.

Model queries (FK, body/COM Jacobians) go through `RobotModel`; today they are
served by MuJoCo (`mj_jac`, `mj_jacSubtreeCom`) and were finite-difference
verified. The Jacobian column convention is `[base_lin(3), base_ang(3), joints(33)]`.

## 10. Status / roadmap

**Done**
- [x] MuJoCo environment wrapper (load / step / render / mouse camera / keyboard teleop)
- [x] Joint position-servo mode; stands stably
- [x] `RobotModel` abstraction + MuJoCo backend (RBDL backend = skeleton)
- [x] `RobotIO` abstraction + Sim backend (Real backend = skeleton); ROS location noted
- [x] Footstep generator with real-time replanning + ZMP preview window
- [x] Real-time 1-second-window LIPM preview controller
- [x] Priority-DLS whole-body IK (support-consistent, swap-on-foot-change)
- [x] Keyboard walking: forward/back, strafe (게걸음), turn
- [x] Headless verification harness (`test/headless_walk_test.cpp`)

**Next (walking robustness)**
- [ ] **Balance stabilizer** — the current walker is open-loop kinematic, so lateral
      balance is only marginally stable. Add DCM/ZMP feedback or FT-based ankle
      strategy at the hook in `HumanoidController` (measured state is already read).
- [ ] RBDL model backend (fill `RbdlModel` once a Tocabi URDF is available)
- [ ] Real-robot I/O backend (`RealIO`) and the ROS interface layer
- [ ] Gains/gait auto-tuning; joint-limit & self-collision avoidance in null space

### Tuning

**Motor control mode**: MuJoCo applies a direct torque law
`τ = kp·(q_des − q) − kv·q̇` (an **impedance-style PD**, *not* a classic
position→velocity→current cascade; there is no current loop). Pure PD leaves a
gravity-load steady-state droop (≈ load/kp); an **integral term** (`servoKi`, added
via a target offset `+(ki/kp)∫e` with anti-windup `servoIClampRad`) trims it
(≈2.5°→2.2° at the loaded knee). An optional **gravity-compensation feedforward**
(`gravityComp`, via `qfrc_applied`) exists but defaults off — naive `qfrc_bias`
over-compensates for a robot in double contact; make it support-consistent before enabling.

**All tunables live in one JSON file — [`config/walking_config.json`](config/walking_config.json)** —
loaded at startup (no recompile needed; edit the JSON and re-run). It holds:
motor position-servo gains (`servoKp/servoKv`; `servoKi` integral; `gravityComp`),
gait pattern (`stepPeriod`, plus **separate `stepPeriodStart`/`stepPeriodEnd`** that
make the first/last steps slower to soften the walk-start/stop ZMP transient,
`doubleSupportRatio`, `stepHeight`, stride/sway limits),
preview (`previewSec`, `comHeight`, `previewQe`/`previewR`), CLIK task gains
(`kpCom/kpSwing/...` = the closed-loop IK error gains) and DLS damping, and teleop
velocity limits. `main`/`headless_test` call `kin::config::loadFromJson()` before
constructing anything, so every module's defaults read the loaded `gConfig` (a missing
file is auto-created from safe defaults). Each module's values come from here;
runtime overrides exist too (`SimIO::setServoGains`, `HumanoidController::setGains`/`setGaitParams`).

The headless harness `test/headless_walk_test.cpp` reads env vars for quick sweeps:
`KIN_KP`/`KIN_KV`/`KIN_KI`/`KIN_GRAV` (servo), `KIN_VX`/`KIN_VY`/`KIN_VYAW` (walk speed),
`KIN_TSTEP`/`KIN_TSTART`/`KIN_DS`/`KIN_H` (gait), `KIN_COMZ` (COM height),
`KIN_STAB` (experimental COM feedback — off by default; naive gains destabilize),
`KIN_WALKSEC`, `KIN_COM/SWING/HAND/PELVIS` (task on/off), and diagnostics
`KIN_JUMP`(+`KIN_JTHR`) — per-tick joint-command spikes with the culprit task —
and `KIN_WSTART` — walk-start ZMP/COM/foot-height trace. Example:

```bash
KIN_KP=8000 KIN_VX=0.04 KIN_WALKSEC=6 ./headless_test   # stiffer servo, forward walk
```
