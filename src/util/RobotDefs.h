#pragma once
// DYROS Tocabi v2 모델의 일반화 좌표/관절/바디 규약 정의.
//
// MuJoCo 컴파일 결과(프로브로 확인):
//   nq = 40 : free joint(base) 7 (pos3 + quat4) + 관절 33
//   nv = 39 : free joint(base) 6 (lin3 + ang3)  + 관절 33
//   nu = 33 : 관절 모터 33개
//
// 인덱싱 규약(중요):
//   - qpos:  base = qpos[0..6],  관절 j(0..32) = qpos[7 + j]
//   - qvel:  base = qvel[0..5],  관절 j(0..32) = qvel[6 + j]   (= dof index)
//   - actuator i(0..32) 는 관절 i 를 1:1 로 구동한다.
//   즉 액추에이터 순서 = 관절 순서 = qpos[7+i] = dof[6+i]. (아주 깔끔)
//
// 본 코드 전체는 이 nv(=39) 속도공간 규약을 "정본"으로 사용한다.
//   Jacobian 열: [ base_lin(3), base_ang(3), joint(33) ] (world 프레임)
//   Jacobian 행: [ linear(3); angular(3) ]
#include <string>
#include <array>

namespace kin {

// ---- 모델 차원 ----
constexpr int kNq       = 40;   // 일반화 위치 차원
constexpr int kNv       = 39;   // 일반화 속도 차원
constexpr int kNjoints  = 33;   // 구동 관절 수
constexpr int kBaseQ    = 7;    // base qpos 개수 (pos3 + quat4)
constexpr int kBaseV    = 6;    // base qvel 개수 (lin3 + ang3)

// qpos/qvel 내 관절 j(0-based) 의 주소
constexpr int jointQposAdr(int j) { return kBaseQ + j; }   // = 7 + j
constexpr int jointDofAdr (int j) { return kBaseV + j; }   // = 6 + j

// ---- 좌/우 구분 ----
enum class Side { Left = 0, Right = 1 };
inline Side other(Side s) { return s == Side::Left ? Side::Right : Side::Left; }

// ---- 관절 인덱스(0-based, 액추에이터/관절 공통) ----
// 순서: L다리6, R다리6, 허리3, L팔8, 목2, R팔8  (총 33)
enum JointIdx {
    L_HipYaw = 0, L_HipRoll, L_HipPitch, L_Knee, L_AnklePitch, L_AnkleRoll,   // 0..5
    R_HipYaw,     R_HipRoll, R_HipPitch, R_Knee, R_AnklePitch, R_AnkleRoll,   // 6..11
    Waist1, Waist2, Upperbody,                                                // 12..14
    L_Shoulder1, L_Shoulder2, L_Shoulder3, L_Armlink,                         // 15..18
    L_Elbow, L_Forearm, L_Wrist1, L_Wrist2,                                   // 19..22
    Neck, Head,                                                               // 23..24
    R_Shoulder1, R_Shoulder2, R_Shoulder3, R_Armlink,                         // 25..28
    R_Elbow, R_Forearm, R_Wrist1, R_Wrist2                                    // 29..32
};

// 다리 관절 그룹의 시작 인덱스(6개 연속).
constexpr int legJointStart(Side s) { return s == Side::Left ? L_HipYaw : R_HipYaw; }

// 양팔 관절(각 8 DOF, 목/머리는 제외). 팔 자세 유지 task 의 대상.
inline constexpr std::array<int, 16> kArmJoints = {
    L_Shoulder1, L_Shoulder2, L_Shoulder3, L_Armlink, L_Elbow, L_Forearm, L_Wrist1, L_Wrist2,
    R_Shoulder1, R_Shoulder2, R_Shoulder3, R_Armlink, R_Elbow, R_Forearm, R_Wrist1, R_Wrist2};

// ---- 주요 바디 이름(MuJoCo body / RBDL body 공통 명명) ----
struct BodyNames {
    static constexpr const char* Pelvis    = "base_link";
    static constexpr const char* Upperbody = "Upperbody_Link";
    static constexpr const char* LFoot     = "L_AnkleRoll_Link";  // 발목 롤 링크 = 발 프레임
    static constexpr const char* RFoot     = "R_AnkleRoll_Link";
    static constexpr const char* LHand     = "L_Wrist2_Link";
    static constexpr const char* RHand     = "R_Wrist2_Link";

    static const char* foot(Side s) { return s == Side::Left ? LFoot : RFoot; }
    static const char* hand(Side s) { return s == Side::Left ? LHand : RHand; }
};

// 발목 롤 링크 원점에서 발바닥 접촉 중심까지의 로컬 오프셋(m).
// 서있는 자세에서 발목 롤 링크가 z≈0.16 에 있으므로 발바닥은 약 -0.16.
constexpr double kSoleOffsetZ = -0.1585;

// 기본 보행 파라미터(기하)
constexpr double kHalfStanceWidth = 0.1025;  // 골반 중심 기준 발 좌우 오프셋(m)

}  // namespace kin
