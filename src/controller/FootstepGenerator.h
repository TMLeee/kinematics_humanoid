#pragma once
// 발걸음 생성기(Footstep Generator).
//
// 요구사항 2 파이프라인의 첫 단계. 사용자 속도 명령(vx, vy 게걸음, vyaw)을 받아
//   - 교대 발 착지 계획(footstep plan)을 실시간으로 생성/갱신하고,
//   - preview controller 가 소비할 ZMP 레퍼런스(1초 윈도우)를 만들며,
//   - 현재 지지발/스윙발과 스윙발 목표 궤적(cycloid)을 제공한다.
//
// 실시간성(요구사항 2): 현재 진행 중인 스텝(지지발/스윙 목표)만 "확정"하고,
//   그 이후 미래 발걸음은 매 tick 현재 속도 명령으로 다시 계산한다. 따라서
//   1초 preview 윈도우가 항상 최신 명령을 반영한다.
#include <vector>

#include "config/WalkingConfig.h"
#include "controller/WalkingState.h"
#include "io/RobotIO.h"

namespace kin {

class FootstepGenerator {
public:
    // 기본값은 모두 전역 설정 gConfig(=JSON 로드) 에서 온다.
    struct Params {
        double Tstep      = config::gConfig.stepPeriod;
        double TstepStart = config::gConfig.stepPeriodStart;   // 시작 스텝 주기
        double TstepEnd   = config::gConfig.stepPeriodEnd;     // 정지 스텝 주기
        int    startRamp  = config::gConfig.startRampSteps;    // 시작→정상 램프 스텝 수
        double dsRatio    = config::gConfig.doubleSupportRatio;
        double stepHeight = config::gConfig.stepHeight;
        double halfWidth  = config::gConfig.halfWidth;
        double maxStride  = config::gConfig.maxStridePerStep;
        double maxSway    = config::gConfig.maxSwayPerStep;
    };

    void init(const Pose2& leftFoot, const Pose2& rightFoot, double comZ, const Params& p);
    void init(const Pose2& leftFoot, const Pose2& rightFoot, double comZ) {
        init(leftFoot, rightFoot, comZ, Params{});
    }

    // 한 제어주기 진행. cmd.walk 이 false 면 정지 상태로 유지.
    void update(double dt, const VelocityCommand& cmd);

    GaitPhase phase() const { return phase_; }
    bool walking() const { return walking_; }
    Side supportSide() const { return support_side_; }
    Side swingSide() const { return other(support_side_); }

    // 지지발(피벗) 착지 포즈.
    Pose2 supportFootPose() const { return support_pose_; }
    // 현재 스윙발 목표(cycloid + 높이). 정지/양발지지 중엔 착지 포즈 유지(z=0).
    FootPose swingFootTarget() const { return swing_target_; }

    // 이번 tick 에 지지발이 교체되었는가(요구사항 4: 우선순위 스왑 트리거).
    bool supportJustSwitched() const { return support_switched_; }

    // preview 용 ZMP 레퍼런스 윈도우(N 샘플, 간격 dt) 채우기.
    void zmpPreview(std::vector<double>& refx, std::vector<double>& refy,
                    int N, double dt) const;

    double comHeight() const { return com_z_; }

    // 디버그: 현재 desired ZMP.
    void currentZmp(double& zx, double& zy) const { zmpAt(t_, zx, zy); }

private:
    // 하나의 지지구간(이 발이 ZMP 를 담당하는 [t0, t1)).
    struct Support {
        Side side;
        double fx, fy, fyaw;      // 발 착지 포즈(world)
        double tx, ty, tyaw;      // 이 스텝의 토르소 앵커(재생성용)
        double t0, t1;
    };

    void extendPlan();                     // 미래 지지구간을 필요한 만큼 확장
    void regenerateTail();                 // 명령 변화 시 미확정 미래를 재생성
    int  currentIndex() const;             // t_ 를 포함하는 지지구간 인덱스
    void appendSupport(Side side);         // 앵커에서 다음 지지구간 하나 추가
    void zmpAt(double t, double& zx, double& zy) const;
    double stepPeriod(int stepIdx) const;  // 스텝 인덱스(1=첫스텝)별 주기(시작/끝 램프)

    Params p_;
    std::vector<Support> plan_;
    double t_ = 0.0;
    double com_z_ = 0.85;
    bool   walking_ = false;
    bool   support_switched_ = false;

    GaitPhase phase_ = GaitPhase::Stand;
    Side      support_side_ = Side::Right;
    Pose2     support_pose_;
    FootPose  swing_target_;

    // 정지 시 사용할 현재 양발 포즈.
    Pose2 left_foot_, right_foot_;

    // 재생성 앵커(마지막으로 append 된 스텝의 토르소/사이드/시간).
    double anchor_tx_ = 0, anchor_ty_ = 0, anchor_tyaw_ = 0, anchor_t1_ = 0;
    Side   anchor_side_ = Side::Left;

    VelocityCommand cmd_;
    int prev_cur_ = -1;                    // 지지 교체 검출용
    int  step_count_ = 0;                  // append 된 스텝 인덱스(시작 램프용)
    bool stopping_ = false;                // 정지 요청 후 마지막 스텝 마무리 중
};

}  // namespace kin
