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
#include "controller/AnkleAdmittance.h"
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

    // 발목 어드미턴스 파라미터. init 전에 호출.
    void setAnkleAdmittance(const AnkleAdmittance::Params& p) { admParams_ = p; }
    const AnkleAdmittance& ankleAdmittance() const { return adm_; }

    const std::string& statusText() const { return status_; }

    // 그래프/디버그용 내부 신호(제어기가 매 tick 산출).
    struct DebugSignals {
        Eigen::Vector2d footstep = Eigen::Vector2d::Zero();  // 지지발(footstep) 중심 (x,y)
        Eigen::Vector2d zmpRef   = Eigen::Vector2d::Zero();  // 목표 ZMP (footstep 생성기, step 입력)
        Eigen::Vector2d comRef   = Eigen::Vector2d::Zero();  // preview COM 목표
        // 로봇이 스스로 계산한 현재 COM: "지지발이 지면에 고정" 가정 + 측정 관절각 FK.
        //   시뮬레이터의 자유베이스(글로벌 정답)를 쓰지 않으므로 실기에서도 동일하게 얻는다.
        //   → 그래프/마커의 "현재 COM" 은 이 값을 써야 실제 로봇과 같은 것을 보게 된다.
        Eigen::Vector3d comMeas  = Eigen::Vector3d::Zero();
        bool comMeasValid = false;
        // --- 발목 어드미턴스 진단 ---
        Eigen::Vector2d copFoot[2] = {Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()}; // 발 로컬 CoP (L,R)
        double footFz[2] = {0.0, 0.0};        // 발 수직 하중 [N] (L,R)
        double ankleDPitch[2] = {0.0, 0.0};   // 발목 보정 [rad] (L,R)
        double ankleDRoll [2] = {0.0, 0.0};
        bool   admActive = false;
        Eigen::Vector2d zmpFromCom = Eigen::Vector2d::Zero();// LIPM: COM ref 에서 나오는 ZMP(=C·x)
        double comRefZ = 0.0;                                // COM ref 높이(그래픽 구 표시용)
        bool walking = false;

        // --- 점프 진단용 ---
        double dqMax = 0.0;         // 이 tick 최대 |dq_i| [rad/s]
        int    dqMaxJoint = -1;     // 그 관절 인덱스
        double qStep = 0.0;         // 이 tick 최대 관절 지령 변화 |Δq_i| [rad]
        // 각 task 의 목표속도(xdot) 노름 — 어느 task 가 점프를 유발하는지 특정
        double xdotCom = 0, xdotSwing = 0, xdotHand = 0, xdotPelvis = 0, xdotWaist = 0;
        // 각 task 오차 노름
        double errCom = 0, errSwing = 0, errPelvis = 0;
        int    supportSide = 0;     // 0=L, 1=R
        bool   supportSwitched = false;
        double swingZ = 0.0;        // 스윙발 지령 높이 [m]
        int    phase = 0;           // 0=stand,1=DS,2=SS
    };
    const DebugSignals& debug() const { return dbg_; }

private:
    void startWBC(const RobotState& s);   // 준비 자세에서 WBC(footstep/preview/IK) 초기화·시작

    // "지지발이 지면에 고정되어 있다"고 가정하고, 주어진 관절각으로 전신 COM 을 푼다.
    //   floating-base 추정에 의존하지 않으므로 실제 로봇에서도 그대로 성립한다.
    //   주의: 내부 모델 상태를 덮어쓴다(호출 후 반드시 다시 setState 할 것).
    Vector3d comWithPlantedFoot(const VectorXd& qJoints, int supId);

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

    double comRefZ_  = 0.88;                      // 유지할 COM 높이(world z) — COM task 목표
    double comLipmZ_ = 0.88;                      // LIPM 높이 zc = 기준면 위 COM 높이(preview 용)
    double comHeightOverride_ = config::gConfig.comHeight; // >0 이면 이 높이로 override

    // 발 기준점(footRef): LIPM/ZMP·지지 제약·스윙발 목표가 공유하는 발 위의 점.
    //   ANKLE 모드: AnkleRoll 링크 원점, 지면 위 +0.1585 m.
    //   SOLE  모드(기존): 발바닥, 지면 z=0.
    //   기준면이 올라가면 LIPM 의 zc 가 그만큼 줄고 ω=√(g/zc) 가 커진다.
    static bool useAnkleRef() { return config::gConfig.footRefAnkle > 0.5; }
    Vector3d footRefOffset_{0, 0, 0};             // 발 링크 원점 기준 로컬 오프셋
    double   footRefZ_ = 0.0;                     // 발 평지 접지 시 기준점의 world z
    Vector3d soleOffset_{0, 0, kSoleOffsetZ};     // (발바닥 — 진단/외부용)

    // 내부(피드포워드) 모델 상태: 지지발을 정확히 고정한 채 base 를 지지발 제약으로
    // 적분한다. 측정값에 앵커링하지 않으므로 키네마틱 드리프트가 없다.
    Vector3d   basePos_ = Vector3d::Zero();
    Quaterniond baseQuat_ = Quaterniond::Identity();
    VectorXd   jointsInt_;           // 내부 관절 지령(nJoints, 적분)
    VectorXd   qInt_;                // 조립된 내부 전체 좌표(nq)
    VectorXd   qlo_, qhi_;           // 관절 한계(선택)

    // 양팔 유지 목표 관절각(startWBC 시점 = 보행 준비 자세). kArmJoints 성분만 사용.
    VectorXd armHold_;

    // 스윙발 피드포워드용 이전 목표 + 스윙발 side(정체성 변경 검출).
    FootPose prevSwing_;
    Side prevSwingSide_ = Side::Right;
    bool haveSwingSide_ = false;
    bool prevWalking_ = false;
    bool first_ = true;

    // 균형 안정화 상태
    double stabAlpha_ = 0.0;
    Eigen::Vector2d prevComMeas_ = Eigen::Vector2d::Zero();
    bool haveComMeas_ = false;

    // 발목 어드미턴스(F/T 기반 발바닥 CoP 순응).
    AnkleAdmittance adm_;
    AnkleAdmittance::Params admParams_{
        config::gConfig.ankleAdmEnable, config::gConfig.ankleAdmKPitch,
        config::gConfig.ankleAdmKRoll,  config::gConfig.ankleAdmTau,
        config::gConfig.ankleAdmClamp,  config::gConfig.ankleAdmFtTau,
        config::gConfig.ankleAdmFzMin,  config::gConfig.ankleAdmFzNom, 0.0, 0.0};

    std::string status_;
    DebugSignals dbg_;

    int footId(Side s) const { return s == Side::Left ? idLFoot_ : idRFoot_; }
    int handId(Side s) const { return s == Side::Left ? idLHand_ : idRHand_; }
};

}  // namespace kin
