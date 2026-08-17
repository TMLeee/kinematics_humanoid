#include "controller/HumanoidController.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace kin {

namespace {
double yawOf(const Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }
}  // namespace

void HumanoidController::init(RobotModel* model, double dt, const RobotState& s0) {
    model_ = model;
    dt_ = dt;
    nJoints_ = model_->nJoints();

    idPelvis_ = model_->bodyId(BodyNames::Pelvis);
    idLFoot_  = model_->bodyId(BodyNames::LFoot);
    idRFoot_  = model_->bodyId(BodyNames::RFoot);
    idLHand_  = model_->bodyId(BodyNames::LHand);
    idRHand_  = model_->bodyId(BodyNames::RHand);

    // 바디 이름이 하나라도 없으면(-1) 이후 MuJoCo 버퍼를 음수 인덱스로 접근해
    // UB/세그폴트가 난다. 여기서 명확히 걸러낸다.
    struct { int id; const char* name; } req[] = {
        {idPelvis_, BodyNames::Pelvis}, {idLFoot_, BodyNames::LFoot},
        {idRFoot_, BodyNames::RFoot}, {idLHand_, BodyNames::LHand}, {idRHand_, BodyNames::RHand}};
    for (auto& r : req) {
        if (r.id < 0) {
            std::fprintf(stderr, "[HumanoidController] FATAL: body '%s' not found in model.\n", r.name);
            std::abort();
        }
    }

    // Idle 유지 자세 = 측정 초기 자세(관절). WBC 는 아직 돌리지 않는다.
    holdPose_  = s0.q.segment(kBaseQ, nJoints_);
    jointsInt_ = holdPose_;
    basePos_   = s0.q.head(3);
    baseQuat_  = Quaterniond(s0.q(3), s0.q(4), s0.q(5), s0.q(6));  // (w,x,y,z)
    baseQuat_.normalize();
    qInt_ = s0.q;

    // 보행 준비 자세: 무릎을 조금 더 굽힌 안정적 스탠스(발바닥은 수평 유지:
    // hipPitch+knee+anklePitch = 0). COM 이 낮아져 측면 안정성이 좋아진다.
    readyPose_ = holdPose_;
    const double hp = -0.40, kn = 0.80, ap = -0.40;   // flat-foot: -0.4+0.8-0.4=0
    for (Side sd : {Side::Left, Side::Right}) {
        int b = legJointStart(sd);                    // 0(L)/6(R): HipYaw..AnkleRoll
        readyPose_(b + 0) = 0.0;   // HipYaw
        readyPose_(b + 1) = 0.0;   // HipRoll
        readyPose_(b + 2) = hp;    // HipPitch
        readyPose_(b + 3) = kn;    // Knee
        readyPose_(b + 4) = ap;    // AnklePitch
        readyPose_(b + 5) = 0.0;   // AnkleRoll
    }
    readyPose_(Waist1) = 0.0; readyPose_(Waist2) = 0.0; readyPose_(Upperbody) = 0.0;

    mode_ = Mode::Idle;
    walkEnabled_ = false;
    prevWalking_ = false;
    first_ = true;
}

// 준비 자세(현재 측정 상태)에서 WBC(footstep/preview/IK)를 초기화하고 Active 로 진입.
void HumanoidController::startWBC(const RobotState& s) {
    // 내부 모델을 현재 측정 상태(준비 자세)로 시드.
    basePos_  = s.q.head(3);
    baseQuat_ = Quaterniond(s.q(3), s.q(4), s.q(5), s.q(6));
    baseQuat_.normalize();
    jointsInt_ = s.q.segment(kBaseQ, nJoints_);
    qInt_ = s.q;

    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();

    // 발 기준점(footRef): LIPM/ZMP/지지 제약이 공유하는 발 위의 점.
    //   footRefAnkle=1 → AnkleRoll 링크 원점(발목).
    //   footRefAnkle=0 → 발바닥(기존).
    //   footRefZ_ = 발이 지면에 평평할 때 이 점의 world z. 발목이면 +0.1585.
    footRefOffset_ = useAnkleRef() ? Vector3d::Zero() : Vector3d(0, 0, kSoleOffsetZ);
    footRefZ_      = useAnkleRef() ? -kSoleOffsetZ : 0.0;

    Vector3d com = model_->com();
    comRefZ_ = (comHeightOverride_ > 0.0) ? comHeightOverride_ : com.z();
    // LIPM 높이 zc = 기준면 위 COM 높이. 기준이 발목이면 발목 높이를 뺀다.
    //   (zc 가 바뀌면 ω=√(g/zc) 와 preview 게인이 함께 바뀐다 — 의도된 변경.)
    comLipmZ_ = std::max(0.05, comRefZ_ - footRefZ_);

    Vector3d lf = model_->bodyPos(idLFoot_, footRefOffset_);
    Vector3d rf = model_->bodyPos(idRFoot_, footRefOffset_);
    Pose2 left{lf.x(), lf.y(), yawOf(model_->bodyRot(idLFoot_))};
    Pose2 right{rf.x(), rf.y(), yawOf(model_->bodyRot(idRFoot_))};
    footstep_.init(left, right, comLipmZ_, gait_);

    preview_.init(dt_, comLipmZ_, config::gConfig.previewSec, config::gConfig.gravity,
                  config::gConfig.previewQe, config::gConfig.previewR,
                  config::gConfig.comMassScale);
    preview_.reset(Eigen::Vector2d(com.x(), com.y()));

    std::printf("[HumanoidController] footRef=%s  footRefZ=%.4f  comZ(world)=%.4f  "
                "zc(LIPM)=%.4f  omega=%.3f\n",
                useAnkleRef() ? "ANKLE" : "SOLE", footRefZ_, comRefZ_, comLipmZ_,
                std::sqrt(config::gConfig.gravity / comLipmZ_));

    // 발목 어드미턴스 초기화(F/T 필터·보정량 리셋).
    adm_.init(dt_, admParams_);

    // 양팔 유지 목표 = 현재(보행 준비) 자세의 팔 관절각.
    armHold_ = jointsInt_;

    prevSwing_ = footstep_.swingFootTarget();
    haveSwingSide_ = false;   // 첫 Active tick 에서 prevSwing_ 을 현재 목표로 재설정
    prevWalking_ = false;
    walkEnabled_ = false;
    haveComMeas_ = false;
    first_ = true;
}

// 지지발 기준점을 계획 포즈(평지, yaw 만)에 고정한 채 관절각으로 base 를 역산하고 COM 을 푼다.
Vector3d HumanoidController::comWithPlantedFoot(const VectorXd& qJoints, int supId) {
    const Pose2 sp = footstep_.supportFootPose();
    const Matrix3d R_pl = rotZ(sp.yaw);
    const Vector3d p_pl(sp.x, sp.y, footRefZ_);

    // 1) base = 단위 로 두고 지지발 기준점의 base-상대 포즈를 얻는다.
    qInt_.head(3).setZero(); qInt_(3) = 1; qInt_(4) = 0; qInt_(5) = 0; qInt_(6) = 0;
    qInt_.segment(kBaseQ, nJoints_) = qJoints;
    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();
    const Vector3d p_bf = model_->bodyPos(supId, footRefOffset_);
    const Matrix3d R_bf = model_->bodyRot(supId);

    // 2) foot_world = base ⊕ foot_base 를 만족하는 base 로 다시 세팅 후 COM.
    const Matrix3d R_base = R_pl * R_bf.transpose();
    const Vector3d p_base = p_pl - R_base * p_bf;
    const Quaterniond qb = Quaterniond(R_base).normalized();
    qInt_.head(3) = p_base;
    qInt_(3) = qb.w(); qInt_(4) = qb.x(); qInt_(5) = qb.y(); qInt_(6) = qb.z();
    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();
    return model_->com();
}

VectorXd HumanoidController::update(const RobotState& s, const VelocityCommand& cmd) {
    // ===== 모드 상태 머신 =====
    // 'h' : 보행 준비 자세로 전환 시작(Preparing). Preparing 중이 아니면 언제든.
    if (cmd.prepareEdge && mode_ != Mode::Preparing) {
        mode_ = Mode::Preparing;
        prepStart_ = jointsInt_;
        prepT_ = 0.0;
        walkEnabled_ = false;
    }

    // Idle: WBC off, 초기 자세 그대로 유지(시작 시 튐 없음).
    if (mode_ == Mode::Idle) {
        jointsInt_ = holdPose_;
        dbg_.walking = false;
        status_ = "mode:IDLE   [H] go to walk-ready pose";
        return jointsInt_;
    }

    // Preparing: 준비 자세로 관절을 smoothstep 보간(경계 속도 0 → 부드럽게). WBC off.
    if (mode_ == Mode::Preparing) {
        prepT_ += dt_;
        double a = std::min(1.0, prepT_ / prepDuration_);
        double s3 = a * a * (3.0 - 2.0 * a);            // smoothstep
        jointsInt_ = prepStart_ + (readyPose_ - prepStart_) * s3;
        dbg_.walking = false;
        char b[96];
        std::snprintf(b, sizeof(b), "mode:PREPARING  %.0f%%  (moving to walk-ready pose)", a * 100.0);
        status_ = b;
        if (a >= 1.0) { startWBC(s); mode_ = Mode::Active; }  // 준비 완료 → WBC 시작
        return jointsInt_;
    }

    // ===== Active: WBC 실행 =====
    // Space: 보행 시작/정지 토글, x: 즉시 정지.  유효 보행 상태 walkEnabled_ 를 footstep 에 전달.
    if (cmd.spaceEdge) walkEnabled_ = !walkEnabled_;
    if (cmd.stop)      walkEnabled_ = false;
    VelocityCommand fcmd = cmd;
    fcmd.walk = walkEnabled_;

    // 0) 계산 기준(qBasis): 내부 지령 jointsInt_ 과 측정 관절각을 stabAlpha(γ)로 섞는다.
    //    γ=0 → 순수 개루프(내부), γ=1 → 완전 폐루프(측정). fixed-foot 로 base 를 역산하므로
    //    측정을 섞어도 드리프트가 없다. 지령은 항상 내부 적분(jointsInt_)으로 부드럽게 유지.
    VectorXd qMeas = (s.valid && s.q.size() == model_->nq())
                     ? s.q.segment(kBaseQ, nJoints_) : jointsInt_;
    // 폐루프: 측정 관절각으로 기구학을 풀어 "실제 COM/발 오차"를 보정한다.
    //   개루프(closedLoop=0)면 내부 지령 jointsInt_ 기준(+ 선택적 γ 블렌드).
    bool closedLoop = (config::gConfig.closedLoop > 0.5) && s.valid
                      && s.q.size() == model_->nq();
    double gamma = stabAlpha_;
    VectorXd qBasis = closedLoop ? qMeas
                    : (gamma <= 0.0) ? jointsInt_
                    : ((1.0 - gamma) * jointsInt_ + gamma * qMeas);

    // 1) 발걸음 생성기(지지발/스윙 결정 — 모델 상태 불필요).
    footstep_.update(dt_, fcmd);
    Side sup = footstep_.supportSide();
    Side sw  = footstep_.swingSide();
    int supId = footId(sup), swId = footId(sw);

    // 1-b) 발목 어드미턴스: 발 F/T → 발바닥 CoP → 발목 pitch/roll 순응.
    //   FK 가 필요 없다(센서 프레임에서 바로 CoP 를 푼다). IK 와 독립적으로 돌고,
    //   결과는 아래 7) 에서 출력 지령에만 더한다.
    if (s.valid && s.ftValid) adm_.update(s.ftLeft, s.ftRight);

    // 2) 고정발(fixed-foot) 재앵커링:
    //    base 를 적분하지 않고, 매 tick "지지발 발바닥을 계획 포즈에 고정"하도록 관절각으로부터
    //    FK 로 역산한다. 이렇게 하면 내부 모델이 정확히 "지지발이 지면에 고정된 매니퓰레이터"가
    //    되어(지지발이 루트) 적분 2차 드리프트가 사라진다.
    {
        Pose2 sp = footstep_.supportFootPose();
        Vector3d p_pl(sp.x, sp.y, footRefZ_);      // 지지발 기준점(발목이면 z=+0.1585)
        Matrix3d R_pl = rotZ(sp.yaw);
        // base=단위 로 두고 지지발 기준점의 base-상대 포즈를 구한다.
        qInt_.head(3).setZero(); qInt_(3) = 1; qInt_(4) = 0; qInt_(5) = 0; qInt_(6) = 0;
        qInt_.segment(kBaseQ, nJoints_) = qBasis;
        model_->setState(qInt_, VectorXd::Zero(model_->nv()));
        model_->updateKinematics();
        Vector3d p_bf = model_->bodyPos(supId, footRefOffset_);   // 기준점(base 프레임)
        Matrix3d R_bf = model_->bodyRot(supId);
        // foot_world = base ⊕ foot_base = (p_pl, R_pl) 를 만족하는 base:
        Matrix3d R_base = R_pl * R_bf.transpose();
        Vector3d p_base = p_pl - R_base * p_bf;
        basePos_  = p_base;
        baseQuat_ = Quaterniond(R_base).normalized();
    }

    // 2-b) COM "측정값": 지지발이 지면에 고정되어 있다는 가정 하에 **측정 관절각**으로 푼다.
    //    floating-base 추정(sim 의 자유베이스 / 실기의 상태추정)에 의존하지 않으므로 실제
    //    로봇에서도 그대로 성립한다. 이 값만 COM task 의 오차 신호로 쓰고, 나머지 태스크와
    //    자코비안은 아래 3) 의 내부 기준(qBasis)을 그대로 유지한다 — dq 의 선형화점과
    //    적분점을 어긋나게 하지 않기 위해서다(closedLoop=1 로 전체를 바꾸면 그 불일치가 생긴다).
    //    주의: 이 호출은 내부 모델 상태를 덮어쓰므로 반드시 3) 이전에 한다.
    //    그래프/뷰어 마커의 "현재 COM" 도 이 값을 쓰므로, comMeasPlanted 노브와 무관하게
    //    측정이 유효하면 항상 계산한다(노브는 "COM task 오차에 쓸지"만 정한다).
    const bool measOk = s.valid && s.q.size() == model_->nq();
    Vector3d comMeas = measOk ? comWithPlantedFoot(qMeas, supId) : Vector3d::Zero();
    const bool haveMeas = measOk && (config::gConfig.comMeasPlanted > 0.5);

    // 3) 내부 모델을 (재앵커된 base, 계산 기준 관절각) 로 세팅 → 발/자코비안 기준.
    qInt_.head(3) = basePos_;
    qInt_(3) = baseQuat_.w(); qInt_(4) = baseQuat_.x();
    qInt_(5) = baseQuat_.y(); qInt_(6) = baseQuat_.z();
    qInt_.segment(kBaseQ, nJoints_) = qBasis;
    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();
    // COM task 오차용 COM: 측정이 있으면 "지지발 지면고정 + 측정 관절각" 값을 쓴다.
    Vector3d com = haveMeas ? comMeas : model_->com();

    if (footstep_.walking() != prevWalking_)
        preview_.reset(Eigen::Vector2d(com.x(), com.y()));   // 상태 전환 시 점프 방지
    prevWalking_ = footstep_.walking();

    // 4) preview: 앞으로 1초 ZMP 윈도우 → COM 레퍼런스(순수 feedforward).
    int N = preview_.previewSize();
    std::vector<double> zx, zy;
    footstep_.zmpPreview(zx, zy, N, dt_);
    preview_.update(zx, zy);
    Eigen::Vector2d comRef = preview_.comPos();
    Eigen::Vector2d comVel = preview_.comVel();
    Vector3d comRefW(comRef(0), comRef(1), comRefZ_);

    // 5) ── IK: 접촉 구속 + 우선순위 태스크 ────────────────────────────────────
    //  정식화 두 가지를 같은 코드로 지원한다(config: floatingBase).
    //   (a) reduced  : 변수 = 관절 33. 지지발 구속을 baseSlave()/reduce() 로 소거.
    //   (b) floating : 변수 = nv 39(가상 base 6 포함). 지지발 구속을 최상위 task 로,
    //                  task 자코비안은 축약 없이 nv 열 그대로. 해의 관절 성분만 사용.
    //  단일지지에서 두 형태의 **달성 task 속도는 같다** (ikCompare=1 로 매 tick 계측:
    //  λ→0 에서 COM 속도차 1e-14 m/s). 관절 dq 는 3e-4 남는데 이는 여유 자유도 배분
    //  차이다 — 최소노름이 재는 노름이 reduced ‖dq_joint‖ vs floating ‖[dq_base;dq_joint]‖.
    //  (b) 만이 양발지지에서 두 발을 동시에 구속할 수 있다(12행 > base 6 → 소거 불가능).
    const int  nv       = model_->nv();
    const bool useFb    = config::gConfig.floatingBase > 0.5;
    const bool dsBoth   = config::gConfig.dsBothFeet > 0.5;
    const bool bodyFrm  = config::gConfig.pelvisBodyFrame > 0.5;
    const bool imuRp    = config::gConfig.pelvisImuRollPitch > 0.5;
    const double lamCon = (config::gConfig.lamContact > 0.0) ? config::gConfig.lamContact
                                                            : g_.lamSupport;
    const double kpPelYaw = (config::gConfig.kpPelvisYaw > 0.0) ? config::gConfig.kpPelvisYaw
                                                                : g_.kpPelvis;
    // 물리적으로 두 발이 접지 중인가(DS 또는 정지). 진단은 항상 이 집합으로 잰다.
    const bool bothDown = (footstep_.phase() != GaitPhase::SingleSupport);
    // 양발 구속 여부는 buildAndSolve 안에서 **호출별 fb** 로 판정한다. 바깥에서 useFb 로
    // 고정하면, ikCompare 의 반대편 호출이 reduced 일 때 양발 구속을 물려받아
    // "스윙 태스크도 없고 접촉 구속도 없는" 존재하지 않는 정식화를 비교하게 된다.

    // 골반 자세: 목표 heading yaw + (선택) 측정 base 자세 = IMU 등가.
    //   지지발 yaw 는 지지구간마다 계단식으로 점프하므로 시간 보간된 토르소 yaw 를 쓴다.
    const double headingYaw = footstep_.torsoYaw();
    Matrix3d R_pel_true = Matrix3d::Identity();
    bool imuOk = false;
    if (measOk) {
        Quaterniond qm(s.q(3), s.q(4), s.q(5), s.q(6));
        qm.normalize();
        R_pel_true = qm.toRotationMatrix();   // 골반 = base 바디이므로 base quat = 골반 자세
        imuOk = true;
    }

    // 스윙발 목표/피드포워드는 태스크 구성 전에 한 번만 계산한다(상태 갱신 부작용 분리 —
    // ikCompare 로 태스크를 두 번 구성해도 prevSwing_ 이 두 번 갱신되지 않게).
    FootPose swT = footstep_.swingFootTarget();
    // 스윙발 정체성(side)이 바뀌면(WBC 시작 tick, Stand↔walk, 지지 교체) 목표가 반대발로
    // 불연속 점프하므로 prevSwing_ 을 현재 목표로 맞춰 FF 스파이크를 없앤다.
    if (!haveSwingSide_ || sw != prevSwingSide_) { prevSwing_ = swT; haveSwingSide_ = true; }
    prevSwingSide_ = sw;
    const Vector3d swFfPos((swT.x - prevSwing_.x) / dt_, (swT.y - prevSwing_.y) / dt_,
                           (swT.z - prevSwing_.z) / dt_);
    const Vector3d swFfOri = orientationError(rotZ(swT.yaw), rotZ(prevSwing_.yaw)) / dt_;

    MatrixXd Jsup = model_->bodyJacobian(supId, footRefOffset_);          // 6×nv
    MatrixXd S = WholeBodyIK::baseSlave(Jsup, nJoints_, g_.lamSupport);   // reduced 전용

    VectorXd dqFull = VectorXd::Zero(nv);       // 실제 사용하는 해의 nv 벡터(접촉 잔차 진단용)
    VectorXd dqFullOther = VectorXd::Zero(nv);  // ikCompare 용 반대 정식화의 해

    // 두 정식화를 하나의 빌더로. 반환값은 항상 "관절 dq(nJoints)".
    //   fb        : floating(nv) / reduced(nJoints) 정식화
    //   bodyFrame : 골반 자세 task 를 골반 바디 프레임으로 표현할지
    auto buildAndSolve = [&](bool fb, bool bodyFrame) -> VectorXd {
        const int  nx      = fb ? nv : nJoints_;
        const int  c0      = fb ? kBaseV : 0;      // 관절 j 의 열 인덱스 = c0 + j
        const bool primary = (fb == useFb) && (bodyFrame == bodyFrm);
        // 이 호출이 양발을 구속하는가. reduced 정식화는 구조적으로 불가능(12행 > base 6).
        const bool two = fb && dsBoth && bothDown;
        // Cartesian task 자코비안: floating 은 nv 열 그대로, reduced 는 support-consistent 축약.
        auto mapC = [&](const MatrixXd& J) -> MatrixXd {
            return fb ? J : WholeBodyIK::reduce(J, S);
        };

        std::vector<WholeBodyIK::Task> tasks;
        int idxCom = -1, idxPel = -1;   // task 별 특이값 진단을 뽑아낼 위치

        // --- (0) 접촉 구속(최우선, xdot = 0) : floating 정식화에서만 명시적 task ---
        //   reduced 에서는 이 구속이 S 안에 소거되어 이미 반영돼 있다(단, 지지발 1개 한정).
        //   xdot=0 이고 dq 가 0 에서 시작하므로 이 task 는 dq 에 기여하지 않는다 —
        //   역할은 오직 하위 태스크가 쓸 null-space 투영자 N 을 만드는 것이다.
        MatrixXd Jcon;
        if (fb) {
            Jcon.resize(two ? 12 : 6, nx);
            Jcon.topRows(6) = Jsup;
            if (two) Jcon.bottomRows(6) = model_->bodyJacobian(swId, footRefOffset_);
            tasks.push_back({Jcon, VectorXd::Zero(Jcon.rows()), lamCon, "Contact"});
        }

        // --- (1) COM ---
        //   [주의] 열을 0 으로 만들면 안 된다. 예전에는 팔/목과 스윙 다리 열을 zeroCols 로
        //   지웠으나 그것은 "이 관절들은 COM 을 움직이지 않는다"는 거짓 모델이라
        //     (1) 스윙발/손 태스크가 그 관절을 크게 움직이는데 COM 태스크가 그 기여를 모르고
        //     (2) null-space 투영자 N 이 "지워진" Jc 로 만들어져 하위 태스크가 COM 을 흔든다.
        //   자코비안은 진실을 유지한 채, 사용 관절을 제한하려면 관절 가중을 써야 한다.
        MatrixXd Jcom = mapC(model_->comJacobian());   // 3×nx
        {
            // closedLoop=on 이면 com 은 측정 관절각 기준(실측 COM) → e 가 곧 실제 COM 오차.
            Vector3d e = comRefW - com;
            VectorXd xdot(3);
            xdot << comVel(0) + g_.kpCom * e(0),
                    comVel(1) + g_.kpCom * e(1),
                    g_.kpCom * e(2);
            if (primary) { dbg_.errCom = e.norm(); dbg_.xdotCom = xdot.norm(); }
            if (en_.com) { idxCom = static_cast<int>(tasks.size());
                           tasks.push_back({Jcom, xdot, g_.lamCom, "COM"}); }
        }

        // --- (2) 스윙발 : cycloid 목표 추종(피드포워드 + P) ---
        //   양발을 접촉 구속한 구간(DS/정지)에서는 이 태스크를 뺀다. 반대발은 이미 접촉
        //   구속으로 정지해 있고, 같은 발에 "속도 0" 과 "목표 추종"을 겹쳐 걸 이유가 없다.
        if (!two) {
            MatrixXd Jr = mapC(model_->bodyJacobian(swId, footRefOffset_));
            Vector3d pcur = model_->bodyPos(swId, footRefOffset_);
            Matrix3d Rcur = model_->bodyRot(swId);
            // footstep 의 swing z 는 "지면 위 높이"(0..stepHeight) → 기준점 높이를 더해 world z.
            Vector3d pdes(swT.x, swT.y, footRefZ_ + swT.z);
            Matrix3d Rdes = rotZ(swT.yaw);
            VectorXd xdot(6);
            xdot.head(3) = swFfPos + g_.kpSwing * (pdes - pcur);
            xdot.tail(3) = swFfOri + g_.kpSwing * orientationError(Rdes, Rcur);
            if (primary) { dbg_.errSwing = (pdes - pcur).norm(); dbg_.xdotSwing = xdot.norm(); }
            if (en_.swing) tasks.push_back({Jr, xdot, g_.lamSwing, "Swing"});
        } else if (primary) {
            dbg_.errSwing = 0.0; dbg_.xdotSwing = 0.0;
        }

        // --- (3) 양팔 : 초기(보행 준비) 자세의 관절각 유지 — 골반 자세보다 상위 우선순위 ---
        //   Cartesian 손 태스크(골반 프레임 고정) 대신 관절공간 posture 로 잡는다.
        //   손을 골반에 고정하면 허리가 움직일 때 팔이 그것을 보상하려 계속 흔들리는데,
        //   관절각 유지는 팔을 말 그대로 가만히 둔다(COM 교란이 작고 예측 가능).
        {
            const int nArm = static_cast<int>(kArmJoints.size());
            MatrixXd J = MatrixXd::Zero(nArm, nx);
            VectorXd xdot(nArm);
            for (int k = 0; k < nArm; ++k) {
                const int j = kArmJoints[k];
                J(k, c0 + j) = 1.0;
                xdot(k) = g_.kpHand * (armHold_(j) - qBasis(j));
            }
            if (primary) dbg_.xdotHand = xdot.norm();
            if (en_.hand) tasks.push_back({J, xdot, g_.lamHand, "Arms"});
        }

        // --- (4) 골반 자세 : 수평 유지 + 진행방향 yaw ---
        //   좌표 선택(pelvisBodyFrame):
        //     world  : e, J 를 내부 world(= 지지발에 앵커된 계획 프레임)에서 그대로 쓴다.
        //     body   : J_body = R_pel^T·J_ang,  e_body = R_pel^T·e.
        //   등방 DLS + 3행 전부 + 동일 이득이면 두 좌표의 dq 는 **정확히 같다**
        //   (λ²I 가 회전 불변이라 R^T 가 소거된다). 좌표 변경이 값을 바꾸는 건
        //     (a) roll/pitch 를 측정(IMU, 진짜 world)에서 받을 때,
        //     (b) 축별 이득을 다르게 줄 때 뿐이다.
        //   J_body 는 내부 world 프레임 선택과 무관하다: 계획 프레임이 진짜 world 와
        //   R_off 만큼 어긋나 있어도 R_pel^T·J_ang = R_true^T·J_true 로 R_off 가 소거된다.
        //   그래서 "계획 프레임 자코비안 + 진짜 world 의 IMU 오차" 조합이 정합적이다.
        {
            Matrix3d Rpel = model_->bodyRot(idPelvis_);
            MatrixXd Jori = mapC(model_->bodyJacobian(idPelvis_)).bottomRows(3);   // angular
            Vector3d oErr = orientationError(rotZ(headingYaw), Rpel);
            VectorXd xdot(3);
            if (bodyFrame) {
                Jori = Rpel.transpose() * Jori;
                oErr = Rpel.transpose() * oErr;
                if (imuRp && imuOk) {
                    // 골반 바디 z 축을 "진짜 수직" 으로. 축이 z_body 에 수직이므로
                    // 바디 프레임으로 옮기면 z 성분이 정확히 0 → roll/pitch 만 건드린다.
                    const Vector3d zb = R_pel_true.col(2);
                    const Vector3d ax = zb.cross(Vector3d::UnitZ());
                    const double sn = ax.norm();
                    Vector3d tiltW = Vector3d::Zero();
                    if (sn > 1e-12) tiltW = ax / sn * std::atan2(sn, zb.z());
                    const Vector3d tiltB = R_pel_true.transpose() * tiltW;
                    oErr.x() = tiltB.x();   // roll/pitch = 실측
                    oErr.y() = tiltB.y();
                    // yaw(z) 는 계획 유지 — IMU yaw 는 드리프트하고 계획 heading 과 어긋난다.
                }
                xdot << g_.kpPelvis * oErr.x(), g_.kpPelvis * oErr.y(), kpPelYaw * oErr.z();
            } else {
                xdot = g_.kpPelvis * oErr;
            }
            if (primary) { dbg_.errPelvis = oErr.norm(); dbg_.xdotPelvis = xdot.norm(); }
            if (en_.pelvis) { idxPel = static_cast<int>(tasks.size());
                              tasks.push_back({Jori, xdot, g_.lamPelvis, "PelvisOri"}); }
        }

        // --- (5) 허리 자세(최하위) : 허리축(Waist1/Waist2/Upperbody) 각 0 유지 ---
        {
            MatrixXd J = MatrixXd::Zero(3, nx);
            J(0, c0 + Waist1) = 1.0; J(1, c0 + Waist2) = 1.0; J(2, c0 + Upperbody) = 1.0;
            VectorXd xdot(3);
            xdot << g_.kpWaist * (0.0 - qBasis(Waist1)),
                    g_.kpWaist * (0.0 - qBasis(Waist2)),
                    g_.kpWaist * (0.0 - qBasis(Upperbody));
            if (primary) dbg_.xdotWaist = xdot.norm();
            if (en_.waist) tasks.push_back({J, xdot, g_.lamWaist, "Waist"});
        }

        // 우선순위 DLS 풀이. primary 일 때만 task 별 특이값 진단을 함께 뽑는다.
        std::vector<WholeBodyIK::TaskDiag> td;
        VectorXd x = WholeBodyIK::solve(tasks, nx, primary ? &td : nullptr);
        VectorXd dqj = fb ? VectorXd(x.segment(kBaseV, nJoints_)) : x;

        if (primary) {
            // task 가 상위 우선순위의 null space 안에서 실제로 쓸 수 있는 방향의 세기.
            //   COM/골반 각각 "제어 권한" 을 나타내는 결정론적 지표 — 카오스에 안 흔들린다.
            auto pick = [&](int i, double& mn, double& mx) {
                if (i >= 0 && i < static_cast<int>(td.size())) { mn = td[i].sigMin; mx = td[i].sigMax; }
            };
            pick(idxCom, dbg_.sigComMin, dbg_.sigComMax);
            pick(idxPel, dbg_.sigPelMin, dbg_.sigPelMax);
            // 접촉 구속을 넣기 **전** 의 전신 COM 자코비안 특이값.
            //   floating 정식화의 3×nv 자코비안은 base 병진 3열이 정확히 I₃ 라 조건수가
            //   1 에 가깝다. 하지만 그 좋은 조건수는 "base 를 자유롭게 평행이동시킬 수 있다"는
            //   전제에서 나오고, 접촉 구속을 걸면 사라진다(위 sigComMin). 두 값을 함께 봐야
            //   "가상 base 덕에 COM 제어가 쉬워진다"는 기대가 왜 성립하지 않는지 보인다.
            Eigen::JacobiSVD<MatrixXd> svdRaw(model_->comJacobian());
            const VectorXd svr = svdRaw.singularValues();
            dbg_.sigComRawMax = svr(0);
            dbg_.sigComRawMin = svr(svr.size() - 1);
            dbg_.nContactRows = fb ? static_cast<int>(Jcon.rows()) : 6;
        }
        // 해를 nv 로 복원(reduced 는 base 를 S 로 되살린다) → 접촉/동치성 진단에 사용.
        {
            VectorXd& out = primary ? dqFull : dqFullOther;
            if (fb) {
                out = x;
            } else {
                out.head(kBaseV) = S * dqj;
                out.tail(nJoints_) = dqj;
            }
        }
        return dqj;
    };

    VectorXd dq = buildAndSolve(useFb, bodyFrm);
    prevSwing_ = swT;

    // 5-b) 동치성 회귀. ikCompare 로 무엇을 비교할지 고른다:
    //   1 = 정식화(reduced ↔ floating). 단일지지에서 **달성 task 속도**는 동일해야 하고,
    //       관절 dq 는 여유 자유도 배분(최소노름이 재는 노름)이 달라 O(1e-4) 차이가 남는다.
    //   2 = 골반 task 좌표(world ↔ body). 3행 전부 + 동일 이득 + 등방 DLS 이면
    //       dq 가 **정확히** 같아야 한다(λ²I 가 회전 불변이라 R^T 가 소거됨).
    const int cmpMode = static_cast<int>(config::gConfig.ikCompare + 0.5);
    dbg_.ikResidual = 0.0;
    dbg_.ikComDiff = 0.0;
    dbg_.ikSwingDiff = 0.0;
    dbg_.ikContactDiff = 0.0;
    if (cmpMode > 0) {
        VectorXd other = (cmpMode == 2) ? buildAndSolve(useFb, !bodyFrm)
                                        : buildAndSolve(!useFb, bodyFrm);
        const double den = std::max(1e-12, std::max(dq.norm(), other.norm()));
        dbg_.ikResidual = (dq - other).norm() / den;
        // 관절 dq 가 달라도 **달성되는 task 속도**가 같다면, 차이는 전부 null space
        //   (= 여유 자유도 배분)에서 온 것이다. 두 정식화의 진짜 차이는 여기에 있다:
        //   감쇠/최소노름이 재는 노름이 다르다(reduced: ‖dq_joint‖, floating: ‖[dq_base;dq_joint]‖).
        const VectorXd d39 = dqFull - dqFullOther;
        dbg_.ikComDiff     = (model_->comJacobian() * d39).norm();
        dbg_.ikSwingDiff   = (model_->bodyJacobian(swId, footRefOffset_) * d39).head(3).norm();
        dbg_.ikContactDiff = (Jsup * d39).head(3).norm();
    }

    // 5-c) 접촉 잔차 진단: **물리적으로 접지 중인 모든 발**의 속도를 잰다.
    //   솔버가 그 발을 구속했는지와 무관하게 같은 잣대로 재야 두 정식화를 비교할 수 있다.
    //   reduced 는 DS 에서 반대발을 구속하지 않으므로 여기서 큰 값이 나오는 것이 정상이며,
    //   그 값이 곧 "지령이 접촉과 모순되는 정도"(발 미끄러짐/들림 요구량)다.
    {
        dbg_.slipLin = 0.0; dbg_.slipAng = 0.0; dbg_.slipSwingLin = 0.0;
        auto footVel = [&](int fid) {
            return VectorXd(model_->bodyJacobian(fid, footRefOffset_) * dqFull);
        };
        VectorXd vs = footVel(supId);
        dbg_.slipLin = vs.head(3).norm();
        dbg_.slipAng = vs.tail(3).norm();
        if (bothDown) {
            VectorXd vw = footVel(swId);
            dbg_.slipSwingLin = vw.head(3).norm();
            dbg_.slipLin = std::max(dbg_.slipLin, dbg_.slipSwingLin);
            dbg_.slipAng = std::max(dbg_.slipAng, vw.tail(3).norm());
            // 반대발이 계획 착지 포즈에서 얼마나 벗어나 있는가(내부 모델 기준).
            Vector3d pdes(swT.x, swT.y, footRefZ_ + swT.z);
            dbg_.dsFootDev = (model_->bodyPos(swId, footRefOffset_) - pdes).norm();
        } else {
            dbg_.dsFootDev = 0.0;
        }
    }
    // 점프 진단: 최대 |dq_i| 와 그 관절, 이 tick 관절 지령 변화량.
    {
        dbg_.dqMax = 0.0; dbg_.dqMaxJoint = -1;
        for (int i = 0; i < nJoints_; ++i)
            if (std::fabs(dq(i)) > dbg_.dqMax) { dbg_.dqMax = std::fabs(dq(i)); dbg_.dqMaxJoint = i; }
        dbg_.qStep = dbg_.dqMax * dt_;
        dbg_.supportSide = (sup == Side::Left) ? 0 : 1;
        dbg_.supportSwitched = footstep_.supportJustSwitched();
    }

    // 6) 지령 적분: jointsInt_ += dq*dt (항상 누적 — 위치서보가 지령을 앞세워 로봇을 끌게).
    //    폐루프는 "오차"만 측정 기준(qBasis=qMeas)으로 닫는다(위 COM/발 태스크). 지령을
    //    측정에서 매번 리셋하면 위치서보 힘이 안 실려 로봇이 못 움직인다.
    //    base 는 다음 tick 에 지지발 고정 조건으로부터 FK 로 재계산 → 드리프트 없음.
    jointsInt_ += dq * dt_;

    // 7) 출력 지령 = IK 결과 + 발목 어드미턴스 보정.
    //    보정을 jointsInt_ 에 되먹이지 않는다. 되먹이면 다음 tick 의 재앵커링/골반자세
    //    태스크가 이를 "관절 오차"로 보고 되돌리려 해서 와인드업이 생긴다. 패턴 생성기는
    //    그대로 두고 출력단에서만 발목을 겹쳐 쓰는 것이 ankle strategy 의 표준 배치다.
    VectorXd qCmd = jointsInt_;
    if (adm_.enabled()) {
        qCmd(L_AnklePitch) += adm_.pitch(Side::Left);
        qCmd(L_AnkleRoll)  += adm_.roll (Side::Left);
        qCmd(R_AnklePitch) += adm_.pitch(Side::Right);
        qCmd(R_AnkleRoll)  += adm_.roll (Side::Right);
    }

    // 관절 한계 클램프(설정된 경우).
    if (qlo_.size() == nJoints_ && qhi_.size() == nJoints_)
        for (int i = 0; i < nJoints_; ++i) {
            jointsInt_(i) = std::max(qlo_(i), std::min(qhi_(i), jointsInt_(i)));
            qCmd(i)       = std::max(qlo_(i), std::min(qhi_(i), qCmd(i)));
        }

    // 상태 텍스트.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "mode:ACTIVE  walk:%s  support:%s  phase:%s  cmd(vx=%.2f vy=%.2f wz=%.2f)  com=(%.3f,%.3f) ref=(%.3f,%.3f)  [Space]walk",
        footstep_.walking() ? "ON " : "off",
        sup == Side::Left ? "L" : "R",
        footstep_.phase() == GaitPhase::SingleSupport ? "SS"
            : footstep_.phase() == GaitPhase::DoubleSupport ? "DS" : "stand",
        fcmd.vx, fcmd.vy, fcmd.vyaw, com.x(), com.y(), comRef(0), comRef(1));
    status_ = buf;

    // 그래프/디버그용 신호 기록.
    {
        double zx, zy; footstep_.currentZmp(zx, zy);
        Pose2 sp = footstep_.supportFootPose();
        dbg_.footstep = Eigen::Vector2d(sp.x, sp.y);
        dbg_.zmpRef   = Eigen::Vector2d(zx, zy);
        dbg_.comRef   = comRef;
        dbg_.comMeas      = measOk ? comMeas : model_->com();
        dbg_.comMeasValid = measOk;
        dbg_.admActive    = adm_.enabled();
        for (int i = 0; i < 2; ++i) {
            Side sd = (i == 0) ? Side::Left : Side::Right;
            dbg_.copFoot[i]     = adm_.cop(sd);
            dbg_.footFz[i]      = adm_.fz(sd);
            dbg_.ankleDPitch[i] = adm_.pitch(sd);
            dbg_.ankleDRoll[i]  = adm_.roll(sd);
        }
        dbg_.zmpFromCom = preview_.zmp();   // LIPM: COM ref 로부터의 ZMP(=C·x)
        dbg_.comRefZ  = comRefZ_;
        dbg_.walking  = footstep_.walking();
        dbg_.swingZ   = footstep_.swingFootTarget().z;
        dbg_.phase    = footstep_.phase() == GaitPhase::SingleSupport ? 2
                      : footstep_.phase() == GaitPhase::DoubleSupport ? 1 : 0;
    }

    first_ = false;
    return qCmd;
}

}  // namespace kin
