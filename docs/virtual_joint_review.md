# Virtual joint(가상 관절 / floating base) 도입 검토

> 상태: **검토 중, 미구현.** 이 문서는 다른 세션이 이어받기 위한 인수인계 노트다.
> 작성 시점의 작업 트리 기준이며, 여기 적힌 수치는 전부 실측/유한차분 검증된 값이다.

---

## 1. 무엇을 바꾸려는가

현재 `HumanoidController` 는 **fixed-foot 재앵커링**으로 base 를 매 tick 역산한다.
검토 대상은 이것을 **IK 가 직접 적분하는 가상 floating base**(옛 RBDL 프로젝트 방식)로
바꾸는 것이다.

### 현재 방식 — fixed-foot 재앵커링

`src/controller/HumanoidController.cpp` `update()` 2) 단계:

```cpp
Pose2 sp = footstep_.supportFootPose();      // 계획된 지지발 포즈
Vector3d p_pl(sp.x, sp.y, footRefZ_);
Matrix3d R_pl = rotZ(sp.yaw);                // 발바닥이 "평평하다"고 가정 (roll=pitch=0)
// base=단위로 FK → 지지발 기준점의 base-상대 포즈 → base 역산
Matrix3d R_base = R_pl * R_bf.transpose();
Vector3d p_base = p_pl - R_base * p_bf;
```

- base 를 적분하지 않으므로 **키네마틱 드리프트가 없다**(이게 이 방식의 장점).
- 대신 내부 모델의 "월드"는 **지지발에 고정된 계획 프레임**이 된다.
- 지지발 제약은 태스크가 아니라 `WholeBodyIK::baseSlave()` 로 base 를 종속시켜 처리한다
  (`S = -dampedPinv(Jb) * Jj`, 이후 모든 태스크 자코비안을 `reduce()` 로 축약).

### 옛 RBDL 프로젝트 — virtual joint 방식

`/home/user1/Downloads/RBDL_test1/RBDL_test1/RBDL_test1.cpp` (V-REP 연동, **실제로 잘 걸었던 코드**)

```cpp
// 585행: COM 이 최우선 태스크, 발은 그 null space
_dq.block(12,0,6,1) = (J_RFoot_inv * Perr_RFoot * 1000.).block(12,0,6,1);   // 오른다리→오른발
_dq.block(6,0,6,1)  = (J_LFoot_inv * Perr_LFoot * 1000.).block(6,0,6,1);    // 왼다리→왼발
_dq0 = _dq;
_dq = J_COM_inv * COM * 1000. + (I28 - J_COM_inv * J_COM) * _dq0;

// 596행: base 를 포함한 전체 좌표를 IK 로 적분
q_cur += (SYSTEM_DT * _dq);          // q_cur(0..5) = 가상 base 도 포함

// 240-242행: base 자세만 고정, 위치는 자유
q_cur(3) = 0.; q_cur(4) = 0.; q_cur(5) = 0.;

// 278행: J_COM 의 base 6열 마스킹이 주석 처리 = base 열을 살려 둠
//J_COM.block(0, 0, 3, 6);
```

**핵심**: `J_COM` 의 base 병진 3열은 **정확히 단위행렬**이다 (`Σ mᵢ·I / M = I`).
`CRBDL.cpp` 의 augmented-body 재귀에서도 루트 바디는 `M = m_aug/mass = 1`, `E = I` 로 확인된다.
따라서 COM 태스크는 조건수 1, 특이점 없음, **로봇 전체를 평행이동시켜 COM 을 맞춘다.**
발은 그 null space 에서 다리 관절로 따라간다.

관절각은 매 tick 측정값으로 덮어쓴다 (226-229행):
```cpp
for (int i = 0; i < MOTORNUM; i++) q_cur(VREPIdxToRBDL[i]) = g_Qcur(i);
```
→ 내부 상태는 **base 6 DOF 뿐**이고 나머지는 전부 실측.

이득: `_dq = J⁺·e·1000`, `q += 0.001·_dq` → **kp·dt = 1.0 (deadbeat 위치레벨 IK)**.
현재 프로젝트는 `kpCom·dt = 6.0×0.002 = 0.012` 로 약 **80배 느리다**.

---

## 2. 왜 검토하는가 — 현재 방식의 측정된 한계

재앵커링이 "지지발이 **계획 위치에, 평평하게** 있다"고 가정하는 데서 오는 프레임 오차.
(검증 워크플로우에서 반증 단계까지 거친 수치)

| 성분 | 크기 | 근거 |
|---|---|---|
| 지지발 tilt 무시 (`R_pl = rotZ(yaw)`) | 최대 **113 mm** | 실제 발바닥 7.57° 기울어짐 × COM 높이 0.85 m = 0.112 m (기하 예측과 일치) |
| 지지발 위치를 계획값에 고정 | 최대 **56 mm** | 실제 발바닥이 계획 대비 최대 57.5 mm 이탈, z 도 21 mm 뜸 |
| 지령 vs 측정 관절각 | 최대 **34 mm** | 관절 추종오차 최대 0.046 rad |

결과: **제어기가 믿는 COM 오차 ≤ 8.3 mm, 실제 물리 오차 최대 92.8 mm (11.2배).**

골반 자세도 같은 구조다 — 내부 오차 rms **0.00°** 인데 실제 world roll/pitch 는
rms 0.43°/0.47°, max 0.89°/1.32° (전진 15 s 실측).

---

## 3. 검증 완료된 사실 (다시 의심하지 말 것)

유한차분 + 반증 에이전트까지 통과한 항목들.

- **`MujocoModel::comJacobian()` 은 정확하다.** 13개 자세 × 39 velocity DOF, `mj_integratePos`
  기반 중심차분, 독립 계산한 전신 COM 대비 **최대 잔차 4.42e-10**.
- **`reduce(comJacobian(), baseSlave(Jsup))` 도 정확하다.** 재앵커링 사상의 참 미분 대비
  상대잔차 **6.5e-10** (exact inverse), damping 켜도 3.3e-6.
- **MuJoCo dof 순서 = `RobotDefs.h` 의 `JointIdx`.** 33개 전부 hinge, `qposadr=7+j`,
  `dofadr=6+j`, 불일치 **0건**. 액추에이터도 1:1.
- **`base_link` 이 world 의 유일한 자식**, subtree CoM = 전신 COM (일치 5.55e-16 m),
  `body_subtreemass = 95.62191 kg` = 전체 질량 합.
- **`mj_comPos` 가 `cdof` 를 채운다.** `updateKinematics()`(mj_kinematics + mj_comPos) 로 충분.
- **`lamSupport = 1e-3` 은 무해하다.** `Jb` 특이값 {1.56,1.56,1.00,1.00,0.63,0.63}, cond 2.4 →
  8 s 보행 누적 왜곡 0.02 mm.
- **지지발은 미끄러지지 않는다.** `reduce(Jsup,S)*dq` 실주행 최대 2.77e-07 m/s.
- **Preview controller 구현이 정확하다.** Tocabi `preview_Parameter()`/옛 프로젝트
  `SolveDynamicEquation()` 과 게인 구조·부호가 대수적으로 일치.
  COM 진폭이 LIPM 이론 `d(1−sech(ω·Tstep/2))` 와 일치 (Tstep=1.0 → 0.0710 vs 0.0713).

---

## 4. 참고 구현 — DYROS Tocabi

- `saga0619/dyros_tocabi` `tocabi_controller/src/walking_pattern.cpp` → **Capture Point 기반**
  (`cpReferencePatternGeneration`, `setCpPosition`, `cptoComTrajectory`). preview 아님.
- `saga0619/tocabi_avatar` `src/avatar.cpp` → **preview control 구현**
  - `preview_Parameter()`: `Qe = 1.0`, `R = 1e-6`, `C(0,2) = -0.71/9.81` → **zc = 0.71**
    (우리 발목 기준 `zc = 0.7096` 과 거의 일치 → 발목 기준 선택이 옳았다는 방증)
  - `preview_horizon_ = 1.6 s`, `preview_hz_ = 2000`
  - K 는 **하드코딩된 상수** (온라인 DARE 를 안 푼다)
  - `previewcontroller()` 는 **증분형(Δ)**: `Δu = −Gi·e − Gx·Δx − ΣGd(i)·Δp_ref`, `u += Δu`
  - **CP 피드백 있음**: `del_zmp = 1.4·(cp_meas − cp_des)` (x), `1.3·` (y)
    → `CLIPM_ZMP_compen_MJ()` 로 반영. 즉 순수 개루프가 아니다.
  - `CP_compen_MJ_FT()` 에 **`zmp_offset = 0.02`** — ZMP 레퍼런스를 발 중심에서 **2 cm 안쪽**으로.
    COM 요구 진폭을 직접 줄이는 장치. 우리는 아직 미적용.
- 로컬 사본: `/tmp/.../scratchpad/tocabi_avatar.cpp`, `tocabi_walking_pattern.cpp`
  (세션 스크래치패드이므로 사라질 수 있음 — 필요하면 `gh api` 로 재취득)

---

## 5. Virtual joint 도입 시 검토할 지점

### 5.1 얻는 것

- **COM 태스크가 자명해진다.** base 병진 열이 단위행렬이라 조건수 1. 현재는 base 를 소거한
  뒤 지지다리+허리로만 COM 을 움직여야 해서 특이값이 `[0.895, 0.781, 0.139]` 이고
  최소 특이방향이 **98.8% 수직**이다(수직 COM 제어가 사실상 안 됨).
- 옛 프로젝트가 실제로 이 구조로 잘 걸었다는 실적.

### 5.2 잃는 것 / 위험

- **키네마틱 드리프트.** 가상 base 를 적분하면 지지발이 내부 모델에서 고정되지 않는다.
  옛 프로젝트는 발 목표를 **절대 월드 좌표**(`Pdes_LFoot(1)=0.11` 등)로 주고 양발을
  **동시에** 위치제어해서 이를 상쇄했다. 우리 구조(단일지지 + 스윙 궤적)와는 다르다.
- **가상 base 이동은 실제 로봇을 움직이지 않는다.** 옛 방식의 COM "추종"은 일부 허구다.
  실제로 넘어지지 않았던 이유는 별개로 **발 F/T 어드미턴스**가 있었기 때문
  (§6 참조). virtual joint 만 도입하고 그건 빼면 같은 결과가 안 나온다.
- base 자세를 어떻게 다룰지 결정 필요. 옛 프로젝트는 `q_cur(3..5)=0` 으로 **고정**했다
  (골반 항상 수평·yaw 0). 회전 보행을 하려면 이 부분을 다시 설계해야 한다.

### 5.3 절충안 (검토 가치 있음)

지지발 제약을 **태스크로 올리고**(현재는 `baseSlave` 로 소거) base 를 자유 DOF 로 두는 방식.
- 우선순위 1 = 지지발 6D 고정, 우선순위 2 = COM(전신 자코비안, base 열 포함), …
- 드리프트는 지지발 태스크가 억제하고, COM 은 base 열의 좋은 조건수를 그대로 누린다.
- `WholeBodyIK::solve()` 를 nJoints 대신 nv 공간으로 확장하면 되고, `reduce()`/`baseSlave()`
  는 필요 없어진다.

---

## 6. 옛 프로젝트가 걸었던 진짜 이유 (virtual joint 만이 아니다)

`RBDL_test1.cpp` 470-494행 — **발 F/T 어드미턴스**. 이게 유일한 실측 피드백이었다.

```cpp
pre_torque_L = pre_torque_L * 0.98 + g_FT_LFoot * 0.02;   // 1차 LPF (τ≈50 ms)
Pdes_LFoot(2) += pre_torque_L(2) * 0.00001;   // z     ← Fz  (수직 컴플라이언스)
Pdes_LFoot(3) -= pre_torque_L(3) * 0.0001;    // roll  ← Mx
Pdes_LFoot(4) -= pre_torque_L(4) * 0.0001;    // pitch ← My
```

- 적용 지점이 **발의 목표 자세**(`Pdes_LFoot`)이고, IK 가 그걸 **다리 전체로** 추종했다.
- 우리 구조에서 발목 관절에 직접 더하면 안 된다 — 지지발은 접촉으로 고정돼 있어
  **몸통이 회전**하고, 재앵커링(`R_pl` 평평 가정)과 골반 태스크가 그걸 되돌리려 싸운다.
  (실측: 이득을 올릴수록 악화, `|CoPx|` 0.048 → 0.119, 전도)
- **수직축(Fz→z)** 이 빠져 있었는데, 착지 충격 흡수/양발 하중 분배는 사실상 그 축이 한다.

`src/controller/AnkleAdmittance.h` 는 구현돼 있고 CoP 계산·필터·부호(실측 기반)·하중
스케일링까지 검증됐다. **적용 지점만** 바꾸면 된다:
스윙발 → 목표 자세 `Rdes`, 지지발 → 재앵커링의 `R_pl`.

---

## 7. 이번 세션에서 확인된 다른 사실 / 수정

### 반영 완료
- `zeroCols` 제거 (COM 태스크가 전신 축약 자코비안 사용) — COM 오차 8.83/14.54 → 1.19/2.74 mm
- 발 기준점을 **발목**으로 (`footRefAnkle=1`) — `zc` 0.868 → 0.7096, ω 3.36 → 3.72
- 양팔을 관절각 유지 posture 태스크로 (골반보다 상위) — 이탈 지령 1e-6 rad
- F/T 센서 배선 + `SimIO::measuredZmp()` 를 **F/T 기반**으로
  (접촉 기반과 직립 구간 rms 2.6 mm 일치, 검증용 `measuredZmpContact()` 유지)
- `FootstepGenerator::currentZmp()` 버그 수정 (정지 중 `zmpPreview()` 와 y 로 102.5 mm 어긋남)
- DS 를 스텝 앞뒤로 절반씩 분할 (Tstep=1.0/ds=0.5 → 0.25/0.50/0.25 실측 확인)
- ZMP 선형보간 제거 → 계단형
- `FootstepGenerator::torsoYaw()` 추가 — 골반 yaw 계단 회전 해결
  (yaw 각가속도 max **419 → 4.7 rad/s²**)
- `servoKvFF` (속도 피드포워드) 구현 — **단 실측이 나빠서 기본 off**

### 이번 세션에서 내가 틀렸던 것 (반복하지 말 것)
프로브에 `setJointLimits()` 를 빠뜨려 헤드리스 하네스를 충실히 재현하지 못한 결과다.
**성능 비교는 반드시 `test/headless_walk_test.cpp` 와 동일 조건으로 할 것.**

- ~~"발목 kp 를 10000 으로 낮추면 좋다"~~ → 공정 A/B 결과 **매 조건 전도**.
  전역 20000 유지가 맞다 (15 s: dx +0.547 안 넘어짐 vs kp=10000 dx +0.096 전도).
- ~~"kv 를 올리면 나빠진다"~~ → 제대로 재면 kv 300→600 중립 (dx 0.547 vs 0.546).
  kv=1000 에서야 전도.
- 속도 피드포워드: 이론상 옳지만 실측 해로움 (dx +0.547 → +0.105, 전도).
  `−kv·q̇` 의 절대속도 감쇠가 실제로 진동을 눌러 주고 있었다.

### 현재 동작 상태
전진 15 s, **전도 없음, dx +0.547 m**.

---

## 8. IMU 관련 (별도 검토 중)

- 모델에 있음: `Pelvis_quat`(framequat), `Gyro_Pelvis_IMU`(site 로컬 각속도),
  `Acc_Pelvis_IMU`, `Magnet_Pelvis_IMU`. **`src/` 에서 아무도 안 읽는다.**
- `Pelvis_IMU` site 는 `base_link` 에 `quat` 없이 붙어 있음 → **IMU 프레임 = 골반 바디 프레임**.
- 바디 프레임 각자코비안 `J_body = R_pel_model^T · J_ang_world` 는 **월드 프레임 선택과 무관**하다
  (`R_off` 가 소거됨). 그래서 IMU 오차(진짜 월드)와 자코비안(계획 프레임)을 섞어도 정합이 맞는다.
- **yaw 는 IMU 로 잡으면 안 된다** — 드리프트하고, 계획 heading 과 어긋나 다리가 비틀린다.
  표준은 roll/pitch = IMU, yaw = 계획(`torsoYaw()`).
- 더 근본적인 적용점: 재앵커링의 `R_pl` 에 측정 발바닥 기울기를 넣는 것.
  `R_sole_true = R_pel_imu · (R_pel_model^T · R_sole_model)` → 내부 모델 전체가 진짜 월드와
  정렬되고 COM 오차 신호도 같이 고쳐진다.

---

## 9. 미해결 / 다음 후보

1. 재앵커링 `R_pl` 에 IMU 기반 실측 발바닥 기울기 반영 (효과 최대 추정)
2. 어드미턴스 적용 지점 이동 (스윙발 `Rdes` / 지지발 `R_pl`) + 수직축(Fz→z) 추가
3. Tocabi 의 **ZMP 2 cm 인보드 오프셋** 적용 — COM 요구 진폭 직접 감소
4. 태스크 우선순위 재정리 (현재 팔이 골반보다 위 — 균형 관점에서 부자연스러움)
5. `stepPeriod` 를 1.0 으로 내리면 넘어지는 문제 (사용자 보고, 미분석)
6. virtual joint 도입 여부 결정 (§5)

### 참고: stepPeriod 와 COM 진폭
계단형 ZMP + LIPM 정상상태 COM 진폭 = `d·(1 − sech(ω·Tstep/2))`, `d = halfWidth = 0.1025`

| Tstep | COM 이 도달해야 하는 y | 발 중심까지 여유 |
|---|---|---|
| 1.0 s | ±0.0713 m | 31 mm |
| 2.0 s | ±0.0975 m | 5 mm |
| 3.0 s | ±0.1017 m | **0.8 mm** |

주기가 길수록 준정적에 가까워져 체중을 완전히 한 발로 옮겨야 한다.
옛 프로젝트와 Tocabi 가 둘 다 1.0 s 를 쓴 이유.
