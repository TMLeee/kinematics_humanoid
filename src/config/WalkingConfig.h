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

    // 모터 위치 서보 이득: force = kp*(q_des - q) - kv*qdot
    double servoKp = 2000.0;
    double servoKv = 100.0;

    // 보행 패턴(발걸음 생성)
    double stepPeriod         = 1.0;      // 정상 스텝 주기 Tstep [s]
    double stepPeriodStart    = 1.5;      // 시작 첫 스텝 주기 [s]
    double stepPeriodEnd      = 1.5;      // 정지 마지막 스텝 주기 [s]
    int    startRampSteps     = 2;        // 시작→정상 램프 스텝 수
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
