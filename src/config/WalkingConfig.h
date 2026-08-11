#pragma once
// ============================================================================
//  보행 제어 통합 설정 (single source of truth)
// ----------------------------------------------------------------------------
//  모터 제어 이득 / 보행 패턴 / preview / WBIK task 이득 / 텔레옵 속도 등
//  튜닝 대상 값을 여기 한 곳에 모은다. 각 모듈의 기본값은 모두 이 상수를 참조한다.
//  (런타임 오버라이드도 가능: SimIO::setServoGains, HumanoidController::setGains 등)
// ============================================================================
#include "util/RobotDefs.h"

namespace kin {
namespace config {

// ── 제어 주기 ───────────────────────────────────────────────────────────────
inline constexpr double kControlDt = 0.002;      // 제어 주기 [s] (500 Hz)

// ── 모터 위치 서보 제어 이득 ─────────────────────────────────────────────────
//   force = kp*(q_des - q) - kv*qdot.
//   보행 시 관절이 목표 궤적을 못 따라가 넘어지면 kp 를 키워 추종 강성을 높인다.
//   (SimIO 가 MuJoCo 액추에이터에 적용. 기존 ctrlrange 는 forcerange 로 이전되어
//    실제 토크 포화는 유지됨.)
inline constexpr double kServoKp = 20000.0;        // 위치 이득
inline constexpr double kServoKv = 500.0;         // 속도(감쇠) 이득

// ── 보행 패턴(발걸음 생성) ───────────────────────────────────────────────────
inline constexpr double kStepPeriod          = 2.0;    // 정상 스텝(스윙) 주기 Tstep [s]
//  보행 시작/끝 스텝 주기(별도). 시작 시 ZMP 가 갑자기 한 발 밑으로 쏠려 로봇이
//  크게 흔들리므로, 첫 몇 스텝을 더 길게(느리게) 밟아 ZMP 변화율을 낮춘다.
//  시작: kStepPeriodStart → kStartRampSteps 스텝에 걸쳐 kStepPeriod 로 램프.
//  끝  : 정지 요청 후 마지막 스텝을 kStepPeriodEnd 로.
inline constexpr double kStepPeriodStart     = 5.;    // 보행 시작 첫 스텝 주기 [s]
inline constexpr double kStepPeriodEnd       = 5.;    // 보행 정지 마지막 스텝 주기 [s]
inline constexpr int    kStartRampSteps      = 2;      // 시작 주기 → 정상 주기 램프 스텝 수
inline constexpr double kDoubleSupportRatio  = 0.4;    // 스텝 내 양발지지 비율(앞부분)
inline constexpr double kStepHeight          = 0.05;  // 스윙발 최고 높이 [m]
inline constexpr double kHalfStanceWidth     = kin::kHalfStanceWidth;  // 좌우 발 간격 절반 [m]
inline constexpr double kMaxStridePerStep    = 0.20;   // 스텝당 최대 전후 이동 [m]
inline constexpr double kMaxSwayPerStep      = 0.12;   // 스텝당 최대 좌우 이동 [m]

// ── COM / preview controller ────────────────────────────────────────────────
inline constexpr double kComHeight    = 0.0;     // COM 목표 높이 [m] (0 = 초기 자세 높이 사용)
inline constexpr double kPreviewSec   = 1.0;     // 미리보기 창 길이 [s]
inline constexpr double kGravity      = 9.81;    // 중력 [m/s^2]
inline constexpr double kPreviewQe    = 1.0;     // ZMP 추종 오차 가중
inline constexpr double kPreviewR     = 1.0e-6;  // 입력(jerk) 가중

// ── CLIK 제어 이득 (WBIK task 비례 이득 P) ───────────────────────────────────
//   Closed-Loop Inverse Kinematics 이득. 각 task 오차를 목표 task 속도로 바꾼다:
//     xdot = kp * (x_des - x_cur)   (COM/스윙발은 피드포워드 속도 항이 추가됨)
//   이 xdot 을 WholeBodyIK 의 우선순위 DLS 가 관절속도로 푼다:
//     dq = Σ pinv(J_i · N) (xdot_i - J_i · dq).
//   (내부 피드포워드 모델 기준의 closed-loop — IK 적분 드리프트 방지/수렴 보장.
//    실측 로봇에 대한 균형 피드백은 별도의 안정화기 몫이다.)
//   우선순위: 고정발 > COM > 스윙발 > 손 > 골반자세 > 허리자세(최하위).
inline constexpr double kpCom    = 6.0;
inline constexpr double kpSwing  = 12.0;
inline constexpr double kpHand   = 4.0;
inline constexpr double kpPelvis = 3.0;
inline constexpr double kpWaist  = 2.0;   // 허리축(관절 0 유지), 최하위 우선순위

// ── WBIK DLS 감쇠 계수(lambda) ──────────────────────────────────────────────
inline constexpr double kLamSupport = 1.0e-3;
inline constexpr double kLamCom     = 1.0e-3;
inline constexpr double kLamSwing   = 1.0e-3;
inline constexpr double kLamHand    = 1.0e-2;
inline constexpr double kLamPelvis  = 1.0e-2;
inline constexpr double kLamWaist   = 1.0e-2;

// ── 텔레옵(키보드) 속도 한계/램프 ────────────────────────────────────────────
inline constexpr double kVfwdMax  = 0.06;        // 전후 [m/s]
inline constexpr double kVlatMax  = 0.04;        // 좌우 게걸음 [m/s]
inline constexpr double kVyawMax  = 0.20;        // 회전 [rad/s]
inline constexpr double kVaccel   = 0.3;         // 명령 램프 [m/s^2]
inline constexpr double kYawAccel = 1.0;         // 회전 램프 [rad/s^2]

}  // namespace config
}  // namespace kin
