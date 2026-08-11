#include "controller/FootstepGenerator.h"

#include <algorithm>
#include <cmath>

namespace kin {

namespace {
constexpr double kPreviewHorizon = 1.2;   // 계획 확장 여유(>1초 preview 윈도우)
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
}  // namespace

void FootstepGenerator::init(const Pose2& leftFoot, const Pose2& rightFoot,
                             double comZ, const Params& p) {
    p_ = p;
    com_z_ = comZ;
    left_foot_ = leftFoot;
    right_foot_ = rightFoot;
    walking_ = false;
    phase_ = GaitPhase::Stand;
    support_side_ = Side::Right;
    support_pose_ = rightFoot;
    swing_target_ = FootPose{leftFoot.x, leftFoot.y, 0.0, leftFoot.yaw};
    plan_.clear();
    t_ = 0.0;
    prev_cur_ = -1;
}

void FootstepGenerator::appendSupport(Side side) {
    const double T = p_.Tstep;
    // 토르소 앵커를 속도 명령만큼 전진(스텝당). yaw 먼저 갱신.
    anchor_tyaw_ += cmd_.vyaw * T;
    double c = std::cos(anchor_tyaw_), s = std::sin(anchor_tyaw_);
    double dx = clampd(cmd_.vx * T, -p_.maxStride, p_.maxStride);
    double dy = clampd(cmd_.vy * T, -p_.maxSway,   p_.maxSway);
    anchor_tx_ += c * dx - s * dy;
    anchor_ty_ += s * dx + c * dy;

    double ly = (side == Side::Left ? p_.halfWidth : -p_.halfWidth);
    Support sp;
    sp.side = side;
    sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
    sp.fx = anchor_tx_ - std::sin(anchor_tyaw_) * ly;
    sp.fy = anchor_ty_ + std::cos(anchor_tyaw_) * ly;
    sp.fyaw = anchor_tyaw_;
    sp.t0 = anchor_t1_;
    sp.t1 = anchor_t1_ + T;
    anchor_t1_ = sp.t1;
    anchor_side_ = side;
    plan_.push_back(sp);
}

void FootstepGenerator::extendPlan() {
    while (plan_.empty() || plan_.back().t1 < t_ + kPreviewHorizon + 2 * p_.Tstep)
        appendSupport(other(anchor_side_));
}

int FootstepGenerator::currentIndex() const {
    if (plan_.empty()) return 0;
    if (t_ < plan_.front().t0) return 0;
    for (int i = 0; i < (int)plan_.size(); ++i)
        if (t_ >= plan_[i].t0 && t_ < plan_[i].t1) return i;
    return (int)plan_.size() - 1;
}

void FootstepGenerator::regenerateTail() {
    int cur = currentIndex();
    int keep = std::min<int>((int)plan_.size(), cur + 2);   // [0 .. cur+1] 확정 유지
    if (keep < 1) return;
    plan_.resize(keep);
    const Support& last = plan_.back();
    anchor_tx_ = last.tx; anchor_ty_ = last.ty; anchor_tyaw_ = last.tyaw;
    anchor_t1_ = last.t1; anchor_side_ = last.side;
    extendPlan();
}

void FootstepGenerator::zmpAt(double t, double& zx, double& zy) const {
    if (plan_.empty()) { zx = support_pose_.x; zy = support_pose_.y; return; }
    int i = (int)plan_.size() - 1;
    if (t < plan_.front().t0) i = 0;
    else for (int k = 0; k < (int)plan_.size(); ++k)
        if (t >= plan_[k].t0 && t < plan_[k].t1) { i = k; break; }

    const double Tds = p_.dsRatio * p_.Tstep;
    double local = t - plan_[i].t0;
    if (i > 0 && local < Tds) {   // 양발지지: 이전 지지발 -> 현재 지지발로 ZMP 이동
        double f = clampd(local / Tds, 0.0, 1.0);
        zx = plan_[i - 1].fx + (plan_[i].fx - plan_[i - 1].fx) * f;
        zy = plan_[i - 1].fy + (plan_[i].fy - plan_[i - 1].fy) * f;
    } else {
        zx = plan_[i].fx;
        zy = plan_[i].fy;
    }
}

void FootstepGenerator::update(double dt, const VelocityCommand& cmd) {
    support_switched_ = false;

    // --- 보행 시작(엣지) ---
    if (cmd.walk && !walking_) {
        walking_ = true;
        cmd_ = cmd;
        t_ = 0.0;
        plan_.clear();
        // 첫 스윙발: 게걸음 방향에 맞춰 선택(발 교차 방지).
        Side firstSwing = (cmd.vy < -1e-4) ? Side::Right : Side::Left;
        Side firstStance = other(firstSwing);

        // 초기 토르소 앵커 = 양발 중점.
        anchor_tx_ = 0.5 * (left_foot_.x + right_foot_.x);
        anchor_ty_ = 0.5 * (left_foot_.y + right_foot_.y);
        anchor_tyaw_ = 0.5 * (left_foot_.yaw + right_foot_.yaw);

        auto footOf = [&](Side s) { return s == Side::Left ? left_foot_ : right_foot_; };
        // plan_[0]: 첫 스윙발의 현재 포즈(swing-from), [-Tstep, 0]
        {
            Support sp; sp.side = firstSwing;
            Pose2 f = footOf(firstSwing);
            sp.fx = f.x; sp.fy = f.y; sp.fyaw = f.yaw;
            sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
            sp.t0 = -p_.Tstep; sp.t1 = 0.0;
            plan_.push_back(sp);
        }
        // plan_[1]: 첫 지지발의 현재 포즈(current stance), [0, Tstep]
        {
            Support sp; sp.side = firstStance;
            Pose2 f = footOf(firstStance);
            sp.fx = f.x; sp.fy = f.y; sp.fyaw = f.yaw;
            sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
            sp.t0 = 0.0; sp.t1 = p_.Tstep;
            plan_.push_back(sp);
        }
        anchor_side_ = firstStance;
        anchor_t1_ = p_.Tstep;
        extendPlan();
        prev_cur_ = 1;
    }

    // --- 보행 정지(엣지) ---
    if (!cmd.walk && walking_) {
        walking_ = false;
        phase_ = GaitPhase::Stand;
        // 현재 지지/스윙발을 착지 상태로 확정(스윙발은 수평 현재위치에 내림).
        Pose2 sup = support_pose_;
        Pose2 sw{swing_target_.x, swing_target_.y, swing_target_.yaw};
        if (support_side_ == Side::Left) { left_foot_ = sup; right_foot_ = sw; }
        else                             { right_foot_ = sup; left_foot_ = sw; }
        swing_target_ = FootPose{sw.x, sw.y, 0.0, sw.yaw};
        plan_.clear();
        return;
    }

    // --- 정지 상태 유지 ---
    if (!walking_) {
        cmd_ = cmd;
        phase_ = GaitPhase::Stand;
        // 피벗 = 왼발, ZMP = 양발 중점(COM 중앙 유지).
        support_side_ = Side::Left;
        support_pose_ = left_foot_;
        swing_target_ = FootPose{right_foot_.x, right_foot_.y, 0.0, right_foot_.yaw};
        return;
    }

    // --- 보행 진행 ---
    t_ += dt;
    cmd_ = cmd;
    regenerateTail();      // 최신 명령으로 미확정 미래 재생성(1초 윈도우 실시간 반영)

    int cur = currentIndex();
    if (cur < 1) cur = 1;
    support_switched_ = (prev_cur_ >= 0 && cur != prev_cur_);
    prev_cur_ = cur;

    support_side_ = plan_[cur].side;
    support_pose_ = Pose2{plan_[cur].fx, plan_[cur].fy, plan_[cur].fyaw};

    // 스윙발: cur-1(이전 착지) -> cur+1(다음 착지), 단일지지 구간에서 이동.
    const Support& from = plan_[cur - 1];
    const Support& to   = plan_[std::min<int>(cur + 1, (int)plan_.size() - 1)];
    const double Tstep = p_.Tstep;
    const double Tds = p_.dsRatio * Tstep;
    double tau = t_ - plan_[cur].t0;

    if (tau < Tds) {
        phase_ = GaitPhase::DoubleSupport;
        swing_target_ = FootPose{from.fx, from.fy, 0.0, from.fyaw};
    } else {
        phase_ = GaitPhase::SingleSupport;
        double x = cycloidXY(tau, Tds, Tstep, from.fx, to.fx);
        double y = cycloidXY(tau, Tds, Tstep, from.fy, to.fy);
        double yaw = cycloidXY(tau, Tds, Tstep, from.fyaw, to.fyaw);
        double z = swingHeight(tau, Tds, Tstep, p_.stepHeight);
        swing_target_ = FootPose{x, y, z, yaw};
    }

    // plan_ 앞부분(더 이상 필요 없는 과거 지지구간)을 잘라 무한 증가를 막는다.
    // 현재 스텝의 swing-from(plan_[cur-1])은 남겨야 하므로 그 앞까지만 제거하고,
    // 지지 교체 검출용 인덱스(prev_cur_)를 시프트만큼 보정한다. (from/to 참조는 위에서 소진됨)
    int drop = cur - 1;
    if (drop > 0) {
        plan_.erase(plan_.begin(), plan_.begin() + drop);
        prev_cur_ -= drop;
    }
}

void FootstepGenerator::zmpPreview(std::vector<double>& refx, std::vector<double>& refy,
                                   int N, double dt) const {
    refx.resize(N);
    refy.resize(N);
    if (!walking_ || plan_.empty()) {
        // 정지: ZMP = 양발 중점 고정.
        double mx = 0.5 * (left_foot_.x + right_foot_.x);
        double my = 0.5 * (left_foot_.y + right_foot_.y);
        std::fill(refx.begin(), refx.end(), mx);
        std::fill(refy.begin(), refy.end(), my);
        return;
    }
    for (int k = 0; k < N; ++k)
        zmpAt(t_ + k * dt, refx[k], refy[k]);
}

}  // namespace kin
