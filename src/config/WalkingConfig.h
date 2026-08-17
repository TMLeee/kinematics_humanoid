#pragma once
// ============================================================================
//  보행 제어 통합 설정 (런타임 로드)
// ----------------------------------------------------------------------------
//  모든 튜닝 값은 config/walking_config.json 에 있고, 시작 시 이 파일을 읽어
//  전역 설정 kin::config::gConfig 에 반영한다(재컴파일 없이 값만 바꿔 재실행).
//  각 모듈의 기본값은 gConfig 의 필드를 참조한다.
//    - 파일 로드: kin::config::loadFromJson(path)  (main/test 시작 시 호출)
//    - 파일 저장: kin::config::saveToJson(path)     (없으면 기본값으로 생성)
//  런타임 오버라이드도 여전히 가능: SimIO::setServoGains, HumanoidController::setGains 등.
// ============================================================================
#include <string>

#include "util/RobotDefs.h"

namespace kin {
namespace config {

// 보행 제어 설정값(전부 JSON 키와 1:1 대응). 기본값 = 안전한 초기값.
struct WalkingConfig {
    // 제어 주기
    double controlDt = 0.002;             // [s] (500 Hz)

    // 모터 위치 서보 이득: force = kp*(q_des - q) - kv*qdot  (임피던스형 PD, 전류루프 없음)
    double servoKp = 2000.0;
    double servoKv = 100.0;
    // 적분 I텀: 목표에 (ki/kp)*∫e 를 더해 중력 부하 sag(정상상태 오차)를 없앤다.
    //   anti-windup: 적분 위치 오프셋을 ±servoIClampRad 로 클램프.
    double servoKi        = 100.0;
    double servoIClampRad = 0.10;
    // 발목 2축 서보 이득(별도). 0 이면 전역 servoKp/Kv 사용. pitch/roll 축별 독립 설정.
    //   발목은 접촉/균형 부하가 커서 몸통과 다른 이득이 필요할 때가 많다.
    double anklePitchKp = 0.0, anklePitchKv = 0.0;
    double ankleRollKp  = 0.0, ankleRollKv  = 0.0;
    // 중력보상 피드포워드(qfrc_applied = gravityComp * qfrc_bias). 기본 off(0).
    //   naive qfrc_bias 는 양발 접촉(닫힌 체인)에서 접촉 반력과 중복돼 과보상되므로
    //   support-consistent 보상으로 고쳐야 정확. 그때 1 로 켠다.
    double gravityComp = 0.0;
    // 속도 피드포워드 계수(0=off, 1=완전보상).
    //   MuJoCo 위치서보는 force = kp*(ctrl−q) − kv*q̇ 로 q̇_des 항이 없다. +kv·q̇_des 를
    //   실어 주면 force = kp*e − kv*(q̇ − q̇_des) 가 되어 kv 가 "추종오차 속도"만 감쇠한다.
    //   이론적으로는 그게 맞지만, **실측은 반대다**(15 s 전진, 전역 kv=300):
    //       FF off → dx +0.547 m, 안 넘어짐
    //       FF on  → dx +0.105 m, 전도
    //   이 로봇은 개루프 키네마틱 보행이라 −kv·q̇ 의 "절대속도 감쇠"가 실제로 진동을
    //   눌러 주는 역할을 하고 있었고, FF 가 그걸 상쇄해 버린다. 또 q̇_des 를 수치미분해
    //   쓰므로 지지 교체 tick 의 dq 스파이크가 kv 배로 증폭돼 토크 포화를 일으킨다.
    //   기능은 남기되 기본 off. 켜려면 q̇_des 평활화와 kv 재튜닝이 함께 필요하다.
    double servoKvFF = 0.0;

    // 보행 패턴(발걸음 생성)
    double stepPeriod         = 1.0;      // 정상 스텝 주기 Tstep [s]
    //  초기/종료 ZMP shift 시간(스텝이 아니라 "체중 이동" 페이즈):
    //    시작: 이 시간 동안 ZMP 를 양발중앙 → 첫 지지발 로 천천히 이동(스텝 없음).
    //    종료: 이 시간 동안 ZMP 를 지지발 → 양발중앙 으로 이동 후 정지.
    //  (개루프 균형 한계상 너무 길면 편측지지 드리프트로 넘어짐 → start≤2, end≤1 권장.)
    double stepPeriodStart    = 2.0;      // 초기 ZMP shift 시간 [s]
    double stepPeriodEnd      = 1.0;      // 종료 ZMP shift 시간 [s]
    int    startRampSteps     = 2;        // (미사용)
    double doubleSupportRatio = 0.4;      // 스텝 내 양발지지 비율
    double stepHeight         = 0.035;    // 스윙발 최고 높이 [m]
    double halfWidth          = kHalfStanceWidth;  // 좌우 발 간격 절반 [m]
    double maxStridePerStep   = 0.20;     // 스텝당 최대 전후 이동 [m]
    double maxSwayPerStep     = 0.12;     // 스텝당 최대 좌우 이동 [m]

    // COM / preview controller
    double comHeight  = 0.0;              // COM 목표 높이 [m] (0 = 초기 자세 높이)
    double previewSec = 1.0;              // 미리보기 창 길이 [s]
    double gravity    = 9.81;             // [m/s^2]
    double previewQe  = 1.0;              // ZMP 추종 오차 가중
    double previewR   = 1.0e-6;           // 입력(jerk) 가중
    // ZMP 계산 시 COM 관성항(c̈om 계수) 배율. 1.0=이론(질량 소거), 1.3=계산상 30% 무겁게.
    //   계산 ZMP 와 실제 동작이 다를 때 보정용. C=[1,0,−comMassScale·zc/g].
    double comMassScale = 1.0;
    // 폐루프 IK: 측정 관절각으로 기구학(COM/자코비안)을 풀어 "실제 COM 오차"를 보정한다.
    //   1=on → 이득이 실제 거동에 반영됨. 단 naive COM 피드백은 에너지를
    //   더해 불안정 → 안정하려면 DCM/캡처포인트 피드백 필요. 0=off(개루프, 현재 안정 default).
    double closedLoop = 0.0;
    // LIPM/ZMP·지지제약·스윙발 목표의 발 기준점.
    //   1 = 발목(AnkleRoll 링크 원점) — 기준면이 지면 위 0.1585 m 로 올라가
    //       LIPM 높이 zc 가 그만큼 줄고 ω=√(g/zc) 가 커진다.
    //   0 = 발바닥(지면 z=0, 기존 방식).
    double footRefAnkle = 1.0;
    // COM task 의 오차 신호로 쓸 COM 을 무엇으로 볼 것인가.
    //   1 = "지지발이 지면에 고정" 가정 + **측정 관절각** FK (실기에서도 성립하는 실측 COM).
    //   0 = 내부 지령(jointsInt_) 기준 COM — 순수 개루프.
    //   주의: 1 은 COM 위치 피드백을 그대로 닫는 것이라 LIPM 에 에너지를 더해 불안정해질 수
    //   있다(실측: 전진 8 s 에서 dx +0.02 → −1.02 m). 안정화하려면 DCM/캡처포인트 형태로
    //   바꿔야 한다. A/B 용으로 노브를 남겨 둔다.
    double comMeasPlanted = 1.0;

    // ── IK 정식화(formulation) ────────────────────────────────────────────
    //  0 = reduced   : 최적화 변수 = 관절 33. 지지발 구속을 baseSlave()/reduce() 로 소거.
    //  1 = floating  : 최적화 변수 = nv 39(가상 base 6 포함). 지지발 구속을 최상위 task 로.
    //  단일지지에서 두 형태는 **달성 task 속도가 기계정밀도로 동일**하다(ikCompare=1 로
    //  상시 검증: λ→0 에서 COM 속도차 9e-15 m/s). 남는 차이는 여유 자유도 배분뿐이다
    //  (최소노름이 재는 노름이 다름: reduced ‖dq_joint‖ vs floating ‖[dq_base;dq_joint]‖).
    //  floating 만이 양발지지에서 두 발을 동시에 구속할 수 있다(12행 > base 6 → 소거 불가).
    //  0 으로 두면 기존 reduced 동작으로 정확히 되돌아간다(A/B 용).
    double floatingBase = 1.0;
    //  양발지지/정지 구간에서 양발을 모두 접촉 구속할지. floatingBase=1 에서만 유효.
    //  0 이면 반대발은 (기존처럼) 하위 우선순위 스윙 task 로만 잡힌다 = 접촉이 soft.
    //  실측: DS 접촉 잔차 1.13e-3 → 3.5e-6 m/s (약 320배). 대신 DS 구간 COM 제어 권한이
    //  절반으로 줄어든다(sigComMin 0.128 → 0.072) — 이는 "양발이 땅에 있다" 는 물리를
    //  정직하게 반영한 결과이지 성능 저하가 아니다.
    double dsBothFeet = 1.0;
    //  접촉 구속 task 의 DLS 감쇠. 0 이면 lamSupport 를 그대로 쓴다.
    //  1e-6 이면 접촉이 사실상 완전 구속이 된다(지령 발속도 ~1e-12 m/s). Jsup 조건수가
    //  2.4 수준이라 이렇게 작게 잡아도 안전하다(양발 12행에서도 확인).
    double lamContact = 1.0e-6;
    //  1 = 매 tick 두 정식화를 모두 풀어 dq 상대잔차를 계측(동치성 회귀 검증용, 2배 느림).
    double ikCompare = 0.0;

    // ── 골반(pelvis) 자세 task 좌표 ────────────────────────────────────────
    //  0 = world(=계획) 프레임 각자코비안/오차 (기존).
    //  1 = 골반 바디 프레임: J_body = R_pel^T·J_ang,  e_body = R_pel^T·e.
    //      등방 DLS + 3행 전부 사용 시 dq 는 world 형태와 **정확히 동일**하다(회전 불변).
    //      의미가 생기는 건 (a) roll/pitch 오차를 IMU(진짜 world)에서 받을 때,
    //      (b) 축별 이득을 다르게 줄 때. J_body 는 내부 world 프레임 선택과 무관하므로
    //      계획 프레임 자코비안과 진짜 world 의 IMU 오차를 섞어도 정합이 맞는다.
    //  검증: 15 s 보행 7500 tick 전체에서 world↔body 의 dq 상대차 max 1.0e-15
    //  (ikCompare=2). 즉 지금 이 값을 1 로 켜는 것 자체는 거동을 바꾸지 않는다 —
    //  IMU/축별이득으로 가는 구조적 준비다.
    double pelvisBodyFrame = 1.0;
    //  골반 yaw 전용 이득(0 = kpPelvis 와 동일). pelvisBodyFrame=1 에서만 분리 적용.
    double kpPelvisYaw = 0.0;
    //  1 = 골반 roll/pitch 오차를 **측정 base 자세**(IMU 등가)에서 취한다. yaw 는 계획 유지.
    //      pelvisBodyFrame=1 필요. 실측 기울기를 균형에 되먹이는 첫 경로.
    //  **실측은 해롭다 → 기본 off.** 9 조건 스윕에서 전도 2/9 → 3/9 로 늘고(vx=0.08 에서
    //  A/C 는 버티는데 이것만 전도), pitch rms 0.47° → 0.74°, ZMP x 오차 rms 2 배,
    //  내부 COM 오차 rms +78%. roll rms 만 0.46° → 0.31° 로 좋아진다.
    //  이유: 순수 P 자세 피드백이라 위상지연이 그대로 LIPM 에 에너지를 넣는다. 제대로 쓰려면
    //  DCM/캡처포인트 또는 ankle strategy 형태로 감쇠를 포함해 설계해야 한다.
    double pelvisImuRollPitch = 0.0;

    // CLIK task 비례 이득 (우선순위: 고정발>COM>스윙발>손>골반>허리)
    double kpCom    = 6.0;
    double kpSwing  = 12.0;
    double kpHand   = 4.0;
    double kpPelvis = 3.0;
    double kpWaist  = 2.0;

    // WBIK DLS 감쇠 계수(lambda)
    double lamSupport = 1.0e-4;
    double lamCom     = 1.0e-3;
    double lamSwing   = 1.0e-3;
    double lamHand    = 1.0e-2;
    double lamPelvis  = 1.0e-2;
    double lamWaist   = 1.0e-2;

    // ── 발목 어드미턴스(임피던스) ──────────────────────────────────────────
    //  발 F/T 로 발바닥 CoP 를 읽어 발목 pitch/roll 을 순응시키는 국소 컴플라이언스.
    //  전역 균형 제어기가 아니다(그건 DCM/캡처포인트 루프의 몫). 발 모서리 들림 방지 +
    //  착지 충격 흡수가 목적이라 이득은 작게 두고 1차 지연·클램프를 건다.
    //  실측 감도: dCoP_x/dPitch = -0.75 m/rad, dCoP_y/dRoll = -2.5 m/rad
    //             → 완전보상 이득은 각각 1.33 / 0.40. 기본값은 그 20~35%.
    double ankleAdmEnable = 1.0;    // 0 = off
    double ankleAdmKPitch = 0.05;   // [rad/m] (목표 CoP = 발목 원점)
    double ankleAdmKRoll  = 0.015;  // [rad/m]
    double ankleAdmTau    = 0.05;   // 1차 지연 [s]
    double ankleAdmClamp  = 0.10;   // 보정 한계 [rad]
    double ankleAdmFtTau  = 0.02;   // F/T 저역통과 [s]
    double ankleAdmFzMin  = 30.0;   // 접지 판정 하중 [N]
    //  하중 스케일 기준(체중 절반). 보정량에 |Fz|/이 값 을 곱해 모멘트 기반과 등가로 만든다.
    //  0 이면 스케일링 off(고정 이득 — 가볍게 실린 발에서 과보정하므로 권장하지 않음).
    double ankleAdmFzNom  = 469.0;

    // 텔레옵 속도 한계/램프
    double vFwdMax  = 0.06;
    double vLatMax  = 0.04;
    double vYawMax  = 0.20;
    double vAccel   = 0.3;
    double yawAccel = 1.0;
};

// 전역 설정(기본값으로 초기화). loadFromJson 으로 시작 시 덮어쓴다.
inline WalkingConfig gConfig;

// 기본 JSON 경로(작업 디렉터리 기준).
inline constexpr const char* kDefaultConfigPath = "config/walking_config.json";

// JSON 파일을 읽어 gConfig 에 반영한다. 파일에 있는 키만 덮어쓴다(나머지는 기본값 유지).
// 성공(파일 존재/파싱) 시 true.
bool loadFromJson(const std::string& path);

// 현재 gConfig 를 JSON 파일로 저장한다.
bool saveToJson(const std::string& path);

}  // namespace config
}  // namespace kin
