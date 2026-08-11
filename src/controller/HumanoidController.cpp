#include "controller/HumanoidController.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

namespace kin {

namespace {
double yawOf(const Matrix3d& R) { return std::atan2(R(1, 0), R(0, 0)); }

// 지정 열들을 0 으로(하체만 COM 제어 → 팔/목 열 제거).
void zeroCols(MatrixXd& J, int c0, int c1) {
    for (int c = c0; c <= c1 && c < J.cols(); ++c) J.col(c).setZero();
}
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

    // 내부 상태를 측정 초기자세로 시드.
    basePos_  = s0.q.head(3);
    baseQuat_ = Quaterniond(s0.q(3), s0.q(4), s0.q(5), s0.q(6));  // (w,x,y,z)
    baseQuat_.normalize();
    jointsInt_ = s0.q.segment(kBaseQ, nJoints_);
    qInt_ = s0.q;

    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();

    Vector3d com = model_->com();
    comRefZ_ = (comHeightOverride_ > 0.0) ? comHeightOverride_ : com.z();

    // 초기 양발 착지 포즈(발바닥 기준).
    Vector3d lf = model_->bodyPos(idLFoot_, soleOffset_);
    Vector3d rf = model_->bodyPos(idRFoot_, soleOffset_);
    Pose2 left{lf.x(), lf.y(), yawOf(model_->bodyRot(idLFoot_))};
    Pose2 right{rf.x(), rf.y(), yawOf(model_->bodyRot(idRFoot_))};
    footstep_.init(left, right, comRefZ_, gait_);

    // preview: dt·COM높이에만 의존하는 게인 1회 계산 후, 현재 COM 으로 리셋.
    preview_.init(dt_, comRefZ_, config::kPreviewSec, config::kGravity,
                  config::kPreviewQe, config::kPreviewR);
    preview_.reset(Eigen::Vector2d(com.x(), com.y()));

    // 손을 골반 프레임에 고정하기 위한 상대 변환 저장.
    Matrix3d Rp = model_->bodyRot(idPelvis_);
    Vector3d pp = model_->bodyPos(idPelvis_);
    for (int i = 0; i < 2; ++i) {
        int hId = (i == 0) ? idLHand_ : idRHand_;
        Matrix3d Rh = model_->bodyRot(hId);
        Vector3d ph = model_->bodyPos(hId);
        handRrel_[i] = Rp.transpose() * Rh;
        handPrel_[i] = Rp.transpose() * (ph - pp);
    }

    prevSwing_ = footstep_.swingFootTarget();
    prevWalking_ = false;
    first_ = true;
}

VectorXd HumanoidController::update(const RobotState& s, const VelocityCommand& cmd) {
    // 0) (안정화 사용 시) 측정 상태에서 실제 COM 을 먼저 구한다.
    Eigen::Vector2d comMeas(0, 0), comVelMeas(0, 0);
    bool useStab = (stabAlpha_ > 0.0) && s.valid && (s.q.size() == model_->nq());
    if (useStab) {
        model_->setState(s.q, VectorXd::Zero(model_->nv()));
        model_->updateKinematics();
        Vector3d cm = model_->com();
        comMeas = Eigen::Vector2d(cm.x(), cm.y());
        if (haveComMeas_) comVelMeas = (comMeas - prevComMeas_) / dt_;
        prevComMeas_ = comMeas;
        haveComMeas_ = true;
    }

    // 1) 모델을 내부(피드포워드) 상태로 세팅.
    //    지지발을 정확히 고정한 일관된 전신 궤적을 내부에서 만들고, 관절만 지령한다.
    qInt_.head(3) = basePos_;
    qInt_(3) = baseQuat_.w(); qInt_(4) = baseQuat_.x();
    qInt_(5) = baseQuat_.y(); qInt_(6) = baseQuat_.z();
    qInt_.segment(kBaseQ, nJoints_) = jointsInt_;
    model_->setState(qInt_, VectorXd::Zero(model_->nv()));
    model_->updateKinematics();
    Vector3d com = model_->com();

    // 2) 발걸음 생성기.
    footstep_.update(dt_, cmd);
    if (footstep_.walking() != prevWalking_)
        preview_.reset(Eigen::Vector2d(com.x(), com.y()));   // 상태 전환 시 점프 방지
    prevWalking_ = footstep_.walking();

    // 지지발이 교체되는 tick 에는 스윙발 목표가 반대발로 불연속 점프하므로,
    // 스윙 피드포워드가 거대한 스파이크를 만들지 않도록 prevSwing_ 을 현재 목표로 맞춘다.
    if (footstep_.supportJustSwitched())
        prevSwing_ = footstep_.swingFootTarget();

    // 3) preview: 앞으로 1초 ZMP 윈도우 → COM 레퍼런스.
    int N = preview_.previewSize();
    std::vector<double> zx, zy;
    footstep_.zmpPreview(zx, zy, N, dt_);
    if (useStab) preview_.updateWithFeedback(zx, zy, comMeas, comVelMeas, stabAlpha_);
    else         preview_.update(zx, zy);
    Eigen::Vector2d comRef = preview_.comPos();
    Eigen::Vector2d comVel = preview_.comVel();
    Vector3d comRefW(comRef(0), comRef(1), comRefZ_);

    // 4) 지지발 제약(최상위) → 베이스 종속 사상 S.
    Side sup = footstep_.supportSide();
    Side sw  = footstep_.swingSide();
    int supId = footId(sup), swId = footId(sw);
    MatrixXd Jsup = model_->bodyJacobian(supId, soleOffset_);   // 6×nv
    MatrixXd S = WholeBodyIK::baseSlave(Jsup, nJoints_, g_.lamSupport);

    std::vector<WholeBodyIK::Task> tasks;

    // --- (1) COM : 하체(다리+허리)로만 제어 ---
    {
        MatrixXd Jc = WholeBodyIK::reduce(model_->comJacobian(), S);  // 3×nJoints
        zeroCols(Jc, L_Shoulder1, R_Wrist2);   // 팔+목 열 제거 (15..32)
        Vector3d e = comRefW - com;
        VectorXd xdot(3);
        xdot << comVel(0) + g_.kpCom * e(0),
                comVel(1) + g_.kpCom * e(1),
                g_.kpCom * e(2);
        if (en_.com) tasks.push_back({Jc, xdot, g_.lamCom, "COM"});
    }

    // --- (2) 스윙발 : cycloid 목표 추종(피드포워드 + P) ---
    {
        FootPose t = footstep_.swingFootTarget();
        MatrixXd Jr = WholeBodyIK::reduce(model_->bodyJacobian(swId, soleOffset_), S);
        Vector3d pcur = model_->bodyPos(swId, soleOffset_);
        Matrix3d Rcur = model_->bodyRot(swId);
        Vector3d pdes(t.x, t.y, t.z);
        Matrix3d Rdes = rotZ(t.yaw);
        Vector3d ffPos((t.x - prevSwing_.x) / dt_, (t.y - prevSwing_.y) / dt_,
                       (t.z - prevSwing_.z) / dt_);
        Vector3d ffOri = orientationError(rotZ(t.yaw), rotZ(prevSwing_.yaw)) / dt_;
        VectorXd xdot(6);
        xdot.head(3) = ffPos + g_.kpSwing * (pdes - pcur);
        xdot.tail(3) = ffOri + g_.kpSwing * orientationError(Rdes, Rcur);
        if (en_.swing) tasks.push_back({Jr, xdot, g_.lamSwing, "Swing"});
        prevSwing_ = t;
    }

    // --- (3) 손 : 골반 프레임에 고정(자연스러운 팔 유지) ---
    {
        Matrix3d Rp = model_->bodyRot(idPelvis_);
        Vector3d pp = model_->bodyPos(idPelvis_);
        for (int i = 0; i < 2; ++i) {
            int hId = (i == 0) ? idLHand_ : idRHand_;
            MatrixXd Jr = WholeBodyIK::reduce(model_->bodyJacobian(hId), S);
            Matrix3d Rdes = Rp * handRrel_[i];
            Vector3d pdes = pp + Rp * handPrel_[i];
            Vector3d pcur = model_->bodyPos(hId);
            Matrix3d Rcur = model_->bodyRot(hId);
            VectorXd xdot(6);
            xdot.head(3) = g_.kpHand * (pdes - pcur);
            xdot.tail(3) = g_.kpHand * orientationError(Rdes, Rcur);
            if (en_.hand) tasks.push_back({Jr, xdot, g_.lamHand, "Hand"});
        }
    }

    // --- (4) 골반 자세 : 수평 유지 + 진행방향 yaw ---
    {
        double headingYaw = footstep_.supportFootPose().yaw;
        Matrix3d Rpel = model_->bodyRot(idPelvis_);
        MatrixXd Jr = WholeBodyIK::reduce(model_->bodyJacobian(idPelvis_), S);
        MatrixXd Jori = Jr.bottomRows(3);   // angular
        VectorXd xdot = g_.kpPelvis * orientationError(rotZ(headingYaw), Rpel);
        if (en_.pelvis) tasks.push_back({Jori, xdot, g_.lamPelvis, "PelvisOri"});
    }

    // 5) 우선순위 DLS → 관절 dq.
    VectorXd dq = WholeBodyIK::solve(tasks, nJoints_);

    // 6) 내부 모델 적분: 관절은 dq, base 는 지지발 고정 제약(v_base = S dq).
    //    지지발이 내부 모델에서 정확히 고정되므로 키네마틱 드리프트가 없다.
    // MuJoCo free-joint qvel 규약: 선속도는 world, 각속도는 base-local 프레임.
    // 따라서 위치는 world 로 적분(좌표 그대로), 자세는 local 각속도 → 우곱으로 적분.
    VectorXd vbase = S * dq;                  // 6: [lin(world,3), ang(local,3)]
    basePos_ += vbase.head(3) * dt_;
    Vector3d omega = vbase.tail(3) * dt_;     // base-local 각속도 * dt
    double ang = omega.norm();
    if (ang > 1e-12) {
        Quaterniond dQ(Eigen::AngleAxisd(ang, omega / ang));
        baseQuat_ = (baseQuat_ * dQ).normalized();   // local 프레임 → 우곱
    }
    jointsInt_ += dq * dt_;

    // 관절 한계 클램프(설정된 경우).
    if (qlo_.size() == nJoints_ && qhi_.size() == nJoints_)
        for (int i = 0; i < nJoints_; ++i)
            jointsInt_(i) = std::max(qlo_(i), std::min(qhi_(i), jointsInt_(i)));

    // 상태 텍스트.
    char buf[256];
    std::snprintf(buf, sizeof(buf),
        "walk:%s  support:%s  phase:%s  cmd(vx=%.2f vy=%.2f wz=%.2f)  com=(%.3f,%.3f) ref=(%.3f,%.3f)",
        footstep_.walking() ? "ON " : "off",
        sup == Side::Left ? "L" : "R",
        footstep_.phase() == GaitPhase::SingleSupport ? "SS"
            : footstep_.phase() == GaitPhase::DoubleSupport ? "DS" : "stand",
        cmd.vx, cmd.vy, cmd.vyaw, com.x(), com.y(), comRef(0), comRef(1));
    status_ = buf;

    first_ = false;
    return jointsInt_;
}

}  // namespace kin
