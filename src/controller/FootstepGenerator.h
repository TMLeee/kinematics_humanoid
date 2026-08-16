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
        // 한 스텝 주기에서 양발지지가 차지하는 비율. Tds = dsRatio·Tstep 을 스텝의
        // 앞뒤로 절반씩(Tds/2) 나눠 배치한다 → DS(Tds/2) / SS(1−dsRatio) / DS(Tds/2).
        //   예) Tstep=1.0, dsRatio=0.5 → 0.25 / 0.50 / 0.25.
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

    // 현재 desired ZMP. zmpPreview() 의 k=0 샘플과 반드시 같은 값이어야 한다.
    //   [버그 수정] 정지(!walking_ || plan_ 비어있음) 중에는 zmpPreview() 가 양발 중점을
    //   쓰는데 zmpAt() 는 support_pose_(지지발)를 돌려주어 y 로 halfWidth(=102.5 mm)
    //   만큼 어긋났다. 그래프의 가짜 ZMP 오차이자, 이 값을 목표로 쓰는 쪽에서는
    //   실제 제어 오차였다. 두 경로를 하나로 맞춘다.
    void currentZmp(double& zx, double& zy) const {
        if (!walking_ || plan_.empty()) {
            zx = 0.5 * (left_foot_.x + right_foot_.x);
            zy = 0.5 * (left_foot_.y + right_foot_.y);
            return;
        }
        zmpAt(t_, zx, zy);
    }

private:
    // 하나의 지지구간(이 발이 ZMP 를 담당하는 [t0, t1)).
    struct Support {
        Side side;
        double fx, fy, fyaw;      // 발 착지 포즈(world)
        double tx, ty, tyaw;      // 이 스텝의 토르소 앵커(재생성용)
        double t0, t1;
        bool   seed = false;      // 보행 시작 시드(plan_[0] = 첫 스윙발의 출발 포즈)
                                  //   ZMP 가 계단형이 된 뒤로는 분기에 쓰이지 않는다(계보 표시용).
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
    int  step_count_ = 0;                  // append 된 스텝 인덱스
    bool stopping_ = false;                // 정지 요청 후 마지막 스텝 마무리 중

    // ── 초기/종료 ZMP shift 페이즈 ──────────────────────────────────────────────
    //  시작: [0, T_start_) 동안 ZMP 를 양발중앙 → 첫 지지발 로 천천히 이동(스텝 없음).
    //  종료: 마지막 스텝 후 [.., T_end_) 동안 ZMP 를 지지발 → 양발중앙 으로 이동.
    double T_start_ = 0.0, T_end_ = 0.0;   // shift 지속시간(= TstepStart/TstepEnd)
    double center_x_ = 0.0, center_y_ = 0.0;   // 양발 중앙(시작 shift 출발점)
    double sshift_to_x_ = 0.0, sshift_to_y_ = 0.0;  // 시작 shift 도착점(첫 지지발)
    Side   first_stance_ = Side::Right;
    // 종료 shift
    bool   final_shift_ = false;
    double final_t0_ = 0.0;
    double fshift_from_x_ = 0.0, fshift_from_y_ = 0.0;  // 지지발(출발)
    double fshift_to_x_ = 0.0, fshift_to_y_ = 0.0;      // 양발 중앙(도착)
    Pose2  final_left_, final_right_;                   // 종료 시 확정 양발
};

}  // namespace kin
