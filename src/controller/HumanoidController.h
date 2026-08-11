#pragma once
// 전신 키네마틱 보행 제어기(오케스트레이터).
//
// 요구사항 2 파이프라인을 연결한다:
//   FootstepGenerator → PreviewController → WholeBodyIK(우선순위 DLS) → 관절 목표.
//
// 매 제어주기 update():
//   1) 모델을 측정 상태로 세팅(FK/Jacobian/COM).
//   2) 발걸음 생성기 갱신(속도명령 → 지지/스윙발, ZMP 1초 윈도우).
//   3) preview 로 COM 레퍼런스(실시간) 계산.
//   4) task(고정발 제약 → COM → 스윙발 → 손 → 골반) 구성, 우선순위 DLS 로 dq.
//   5) q_des = q_meas + dq*dt (resolved-velocity closed-loop IK).
#include <string>

#include "config/WalkingConfig.h"
#include "controller/FootstepGenerator.h"
#include "controller/PreviewController.h"
#include "controller/WholeBodyIK.h"
#include "model/RobotModel.h"
#include "io/RobotIO.h"

namespace kin {

class HumanoidController {
public:
    // 기본값은 모두 전역 설정 gConfig(=JSON 로드) 에서 온다.
    struct Gains {
        double kpCom    = config::gConfig.kpCom;
        double kpSwing  = config::gConfig.kpSwing;
        double kpHand   = config::gConfig.kpHand;
        double kpPelvis = config::gConfig.kpPelvis;
        double kpWaist  = config::gConfig.kpWaist;
        double lamSupport = config::gConfig.lamSupport;
        double lamCom     = config::gConfig.lamCom;
        double lamSwing   = config::gConfig.lamSwing;
        double lamHand    = config::gConfig.lamHand;
        double lamPelvis  = config::gConfig.lamPelvis;
        double lamWaist   = config::gConfig.lamWaist;
    };

    // task on/off (튜닝/디버그용). 기본 전부 on.
    struct Enable { bool com = true, swing = true, hand = true, pelvis = true, waist = true; };
    void setEnable(const Enable& e) { en_ = e; }

    void init(RobotModel* model, double dt, const RobotState& s0);

    // 동작 모드:
    //   Idle      : 프로그램 시작 직후. WBC(footstep/preview/IK) 를 돌리지 않고 초기
    //               자세를 그대로 유지만 한다 → 시작 시 튐 없음. 'h' 로 Preparing 진입.
    //   Preparing : 보행 준비 자세로 관절을 부드럽게 보간(WBC 아직 off).
    //   Active    : 준비 자세에서 WBC 시작(정지 균형 유지). Space 로 보행 on/off.
    enum class Mode { Idle, Preparing, Active };
    Mode mode() const { return mode_; }

    // 관절 목표각(nJoints) 계산.
    VectorXd update(const RobotState& s, const VelocityCommand& cmd);

    // 관절 하한/상한(nJoints) 설정 시 q_des 클램프.
    void setJointLimits(const VectorXd& lo, const VectorXd& hi) { qlo_ = lo; qhi_ = hi; }

    // 보행 파라미터(init 전에 설정). 튜닝용.
    void setGaitParams(const FootstepGenerator::Params& p) { gait_ = p; }
    void setGains(const Gains& g) { g_ = g; }
    // COM 목표 높이 override(0 이면 초기 자세 높이 사용). 낮출수록 측면 안정성 ↑.
    void setComHeight(double z) { comHeightOverride_ = z; }
    // [실험적] 균형 안정화(측정 COM 피드백) 게인 alpha in [0,1]. 0=순수 피드포워드(기본).
    //   naive COM 피드백은 발산하기 쉽다. 실사용에는 DCM/ZMP 피드백 또는 FT 기반
    //   ankle strategy 를 여기에 제대로 설계해 넣어야 한다(향후 과제).
    void setStabilizer(double alpha) { stabAlpha_ = alpha; }

    const std::string& statusText() const { return status_; }

    // 그래프/디버그용 내부 신호(제어기가 매 tick 산출).
    struct DebugSignals {
        Eigen::Vector2d footstep = Eigen::Vector2d::Zero();  // 지지발(footstep) 중심 (x,y)
        Eigen::Vector2d zmpRef   = Eigen::Vector2d::Zero();  // 목표 ZMP (footstep 생성기)
        Eigen::Vector2d comRef   = Eigen::Vector2d::Zero();  // preview COM 목표
        bool walking = false;
    };
    const DebugSignals& debug() const { return dbg_; }

private:
    void startWBC(const RobotState& s);   // 준비 자세에서 WBC(footstep/preview/IK) 초기화·시작

    RobotModel* model_ = nullptr;
    double dt_ = 0.002;
    int nJoints_ = kNjoints;

    // 모드 상태 머신
    Mode     mode_ = Mode::Idle;
    VectorXd holdPose_;              // Idle 유지 자세(관절)
    VectorXd readyPose_;             // 보행 준비 자세(관절 목표)
    VectorXd prepStart_;             // Preparing 시작 시 관절
    double   prepT_ = 0.0;           // Preparing 경과 시간 [s]
    double   prepDuration_ = 2.5;    // 준비 자세 이동 시간 [s]
    bool     walkEnabled_ = false;   // Active 에서 Space 로 토글되는 유효 보행 상태

    FootstepGenerator footstep_;
    PreviewController preview_;
    FootstepGenerator::Params gait_;
    Gains g_;
    Enable en_;

    // 바디 id 캐시
    int idPelvis_ = -1, idLFoot_ = -1, idRFoot_ = -1, idLHand_ = -1, idRHand_ = -1;

    double comRefZ_ = 0.88;                       // 유지할 COM 높이(world z)
    double comHeightOverride_ = config::gConfig.comHeight; // >0 이면 이 높이로 override
    Vector3d soleOffset_{0, 0, kSoleOffsetZ};

    // 내부(피드포워드) 모델 상태: 지지발을 정확히 고정한 채 base 를 지지발 제약으로
    // 적분한다. 측정값에 앵커링하지 않으므로 키네마틱 드리프트가 없다.
    Vector3d   basePos_ = Vector3d::Zero();
    Quaterniond baseQuat_ = Quaterniond::Identity();
    VectorXd   jointsInt_;           // 내부 관절 지령(nJoints, 적분)
    VectorXd   qInt_;                // 조립된 내부 전체 좌표(nq)
    VectorXd   qlo_, qhi_;           // 관절 한계(선택)

    // 손을 골반 프레임에 고정(자연스러운 팔 유지)하기 위한 상대 변환.
    Matrix3d handRrel_[2];
    Vector3d handPrel_[2];

    // 스윙발 피드포워드용 이전 목표.
    FootPose prevSwing_;
    bool prevWalking_ = false;
    bool first_ = true;

    // 균형 안정화 상태
    double stabAlpha_ = 0.0;
    Eigen::Vector2d prevComMeas_ = Eigen::Vector2d::Zero();
    bool haveComMeas_ = false;

    std::string status_;
    DebugSignals dbg_;

    int footId(Side s) const { return s == Side::Left ? idLFoot_ : idRFoot_; }
    int handId(Side s) const { return s == Side::Left ? idLHand_ : idRHand_; }
};

}  // namespace kin
