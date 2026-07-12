# kin_humanoid

MuJoCo-based **kinematics whole-body control** for the DYROS Tocabi v2 humanoid.

The goal is to develop kinematics-based controllers in C++ for balancing and
motion generation. Simulation runs in MuJoCo, rigid-body kinematics/dynamics are
computed with RBDL, and linear algebra uses Eigen.

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
│   ├── main.cpp                 # entrypoint (load model -> RBDL self-test -> sim loop)
│   ├── simulator/
│   │   └── MujocoEnv.{h,cpp}     # MuJoCo load/step/render + mouse camera control
│   ├── kinematics/             # (planned) robot kinematics/dynamics interface
│   ├── controller/             # (planned) whole-body controller
│   ├── model/                  # (planned)
│   └── util/                   # (planned)
├── model/
│   └── dyros_tocabi_v2/        # submodule (sparse-checkout: mujoco_model + meshes only)
├── prj/
│   └── kinematics_humanoid.code-workspace  # VS Code workspace
├── .vscode/                    # tasks / launch / c_cpp_properties
└── README.md
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
4. Open the GLFW viewer and run the simulation loop.

> **Note**: there is no controller yet, so actuator torques are zero.
> The robot therefore collapses in place under gravity (-9.81) — this is expected.
> Standing and motion generation are the job of the upcoming kinematics-based
> whole-body controller.

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

## 6. Debugging (VS Code)

Open `prj/kinematics_humanoid.code-workspace` to load the project:

| Configuration | Description |
|---------------|-------------|
| Debug kin_humanoid (gdb) | Debug this project |

The configuration runs its build task first and injects the MuJoCo and RBDL
library paths into `LD_LIBRARY_PATH`.

---

## 7. Note on RBDL for kinematics

| Use case | Fit |
|----------|-----|
| Forward/inverse kinematics, Jacobians | Ideal. RBDL provides fast recursive FK, point/body Jacobians, and CoM kinematics out of the box. |
| Inverse/forward dynamics, mass matrix | Very suitable. RNEA / CRBA / ABA are built in, useful for gravity compensation and dynamics-consistent kinematic control. |
| Model source | The MuJoCo MJCF is the simulation source of truth; the RBDL model is built separately (URDF via the urdfreader addon, or constructed in code) and kept consistent with it. |

Adopted for the kinematics and (later) dynamics-consistent control paths. The MuJoCo
model remains authoritative for simulation; RBDL supplies the analytical
kinematics/dynamics the controller queries.

---

## 8. Status / roadmap

**Done**
- [x] MuJoCo environment wrapper (load / step / render / mouse camera)
- [x] Keyframe init pose, ground + collision-mesh render cleanup
- [x] Dependency wiring (Eigen / RBDL / Tocabi model)
- [x] VS Code build/debug environment

**Next**
- [ ] Robot kinematics/dynamics interface (`src/kinematics`)
- [ ] Import + modify the kinematics-based whole-body controller (`src/controller`)
- [ ] Posture / CoM / contact tasks -> balancing and motion generation
