#include "controller/FootstepGenerator.h"

#include <algorithm>
#include <cmath>

namespace kin {

namespace {
constexpr double kPreviewHorizon = 1.2;   // 계획 확장 여유(>1초 preview 윈도우)
double clampd(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
// 부드러운 0→1 보간(경계 속도 0). ZMP shift 를 부드럽게.
double smoothstep01(double x) { x = clampd(x, 0.0, 1.0); return x * x * (3.0 - 2.0 * x); }
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

// 모든 스텝은 동일 주기 Tstep. (초기/종료의 느린 체중이동은 별도 ZMP shift 페이즈가 담당.)
double FootstepGenerator::stepPeriod(int /*stepIdx*/) const {
    return p_.Tstep;
}

void FootstepGenerator::appendSupport(Side side) {
    const double T = stepPeriod(++step_count_);
    // 토르소 앵커를 속도 명령만큼 전진(스텝당). yaw 먼저 갱신.
    anchor_tyaw_ += cmd_.vyaw * T;
    double c = std::cos(anchor_tyaw_), s = std::sin(anchor_tyaw_);
    double dx = clampd(cmd_.vx * T, -p_.maxStride, p_.maxStride);
    double dy = clampd(cmd_.vy * T, -p_.maxSway,   p_.maxSway);

    // ── 게걸음 발 겹침 방지 ──────────────────────────────────────────────────
    // 좌우 이동 중에는 매 스텝 앵커가 dy 만큼 옆으로 가고 두 발이 그 앵커에서 ±halfWidth
    // 로 놓인다. 그래서 **진행 방향의 반대발**이 착지할 때 두 발 중심 간격이
    // 2·halfWidth − dy 로 줄어든다(halfWidth=0.1025, 발 반폭 0.065 → dy>0.075 면 겹침).
    //
    // 고치는 방법: 좌우 이동분을 **진행 방향 쪽 발의 스텝에만 몰아서** 준다.
    //   진행쪽 발이 2·dy 만큼 크게 벌려 나가고, 반대발은 옆으로 움직이지 않고
    //   (앵커가 그대로이므로) 새 진행쪽 발에서 정확히 2·halfWidth 떨어진 자리에 놓인다.
    //   → 최소 간격이 항상 2·halfWidth 로 유지되고, 사이클당 이동량(2·dy)은 그대로다.
    // 전후(dx) 는 두 발이 좌우로 이미 떨어져 있어 겹치지 않으므로 손대지 않는다.
    const double kVyEps = 1e-4;
    const bool leadSide = (dy > kVyEps  && side == Side::Left)
                       || (dy < -kVyEps && side == Side::Right);
    const bool trailSide = (dy > kVyEps  && side == Side::Right)
                        || (dy < -kVyEps && side == Side::Left);
    double dyStep = dy;
    if (p_.sidestepLeadOnly > 0.5) {
        if (leadSide)  dyStep = clampd(2.0 * dy, -p_.maxSway, p_.maxSway);
        if (trailSide) dyStep = 0.0;
    }

    anchor_tx_ += c * dx - s * dyStep;
    anchor_ty_ += s * dx + c * dyStep;

    double ly = (side == Side::Left ? p_.halfWidth : -p_.halfWidth);
    Support sp;
    sp.side = side;
    sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
    sp.fx = anchor_tx_ - std::sin(anchor_tyaw_) * ly;
    sp.fy = anchor_ty_ + std::cos(anchor_tyaw_) * ly;
    sp.fyaw = anchor_tyaw_;

    // 안전망: 바로 앞 스텝(이 스텝의 지지발이 될 발)과의 **횡방향** 간격이 최소치보다
    // 좁으면 밖으로 밀어낸다. 회전 보행처럼 위 보정으로 안 잡히는 조합까지 막는다.
    if (!plan_.empty() && p_.minFootClearance > 0.0) {
        const Support& prev = plan_.back();
        const double cy = std::cos(prev.fyaw), sy = std::sin(prev.fyaw);
        // 이전 발 프레임에서 본 이번 발의 횡방향 좌표.
        const double dxw = sp.fx - prev.fx, dyw = sp.fy - prev.fy;
        double lat = -sy * dxw + cy * dyw;                  // prev 발 기준 y
        const double want = (side == Side::Left) ? p_.minFootClearance : -p_.minFootClearance;
        if ((side == Side::Left && lat < want) || (side == Side::Right && lat > want)) {
            const double push = want - lat;                 // prev 발 프레임에서 밀 양
            sp.fx += -sy * push;
            sp.fy +=  cy * push;
        }
    }

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

// ZMP 레퍼런스. 첫 주기(초기 shift)·마지막 주기(종료 shift)만 천천히(smoothstep) 이동하고,
// 중간 스테핑 구간은 보간 없이 현재 지지발에 고정한다(계단형 ZMP).
//   지지 교체 경계를 중심으로 앞뒤 반쪽 DS 가 이어져 길이 Tds 의 연속 양발지지 구간이
//   만들어지므로, 그 안에서 ZMP 가 한 발 → 다른 발로 계단 형태로 건너뛰어도 항상
//   지지 다각형 안에 있다. preview controller 는 계단형 ZMP 입력을 전제로 설계된 것이라
//   이 불연속을 미리보기로 흡수한다.
void FootstepGenerator::zmpAt(double t, double& zx, double& zy) const {
    // 종료 ZMP shift(마지막 주기): 지지발 → 양발중앙 (T_end_ 동안 천천히).
    if (final_shift_) {
        double f = smoothstep01((t - final_t0_) / std::max(1e-6, T_end_));
        zx = fshift_from_x_ + (fshift_to_x_ - fshift_from_x_) * f;
        zy = fshift_from_y_ + (fshift_to_y_ - fshift_from_y_) * f;
        return;
    }
    // 초기 ZMP shift(첫 주기): 양발중앙 → 첫 지지발 (T_start_ 동안 천천히).
    if (t < T_start_) {
        double f = smoothstep01(T_start_ > 1e-6 ? t / T_start_ : 1.0);
        zx = center_x_ + (sshift_to_x_ - center_x_) * f;
        zy = center_y_ + (sshift_to_y_ - center_y_) * f;
        return;
    }
    // 중간 스테핑: 보간 없이 현재 지지발에 고정.
    if (plan_.empty()) { zx = support_pose_.x; zy = support_pose_.y; return; }
    int i = (int)plan_.size() - 1;
    if (t < plan_.front().t0) i = 0;
    else for (int k = 0; k < (int)plan_.size(); ++k)
        if (t >= plan_[k].t0 && t < plan_[k].t1) { i = k; break; }
    zx = plan_[i].fx;
    zy = plan_[i].fy;
}

// 토르소 yaw: 접지 중인 두 발의 중간 yaw 를 지지구간 안에서 선형 보간.
//   지지구간 cur 의 시작에는 (cur-1, cur) 두 발이, 끝에는 (cur, cur+1) 두 발이 접지한다.
//     시작값 = (tyaw[cur-1] + tyaw[cur]) / 2
//     끝값   = (tyaw[cur]   + tyaw[cur+1]) / 2
//   연속 앵커가 vyaw·Tstep 씩 차이나므로 구간마다 기울기가 같아 전체가 등속 회전이 되고,
//   구간 경계에서 값도 이어진다(끝값[cur] == 시작값[cur+1]).
double FootstepGenerator::torsoYaw() const {
    if (!walking_ || plan_.empty())
        return 0.5 * (left_foot_.yaw + right_foot_.yaw);

    // 초기 ZMP shift 중에는 아직 스텝이 없다 → 시작 앵커 yaw 유지.
    if (t_ < T_start_) return plan_.front().tyaw;

    int cur = currentIndex();
    if (cur < 1) cur = 1;
    const int prev = cur - 1;
    const int next = std::min<int>(cur + 1, (int)plan_.size() - 1);

    const double y0 = 0.5 * (plan_[prev].tyaw + plan_[cur].tyaw);
    const double y1 = 0.5 * (plan_[cur].tyaw  + plan_[next].tyaw);

    const double T = plan_[cur].t1 - plan_[cur].t0;
    if (T <= 1e-9) return y1;
    const double f = clampd((t_ - plan_[cur].t0) / T, 0.0, 1.0);
    return y0 + (y1 - y0) * f;
}

void FootstepGenerator::update(double dt, const VelocityCommand& cmd) {
    support_switched_ = false;

    // --- 보행 시작(엣지) ---
    if (cmd.walk && !walking_) {
        walking_ = true;
        stopping_ = false;
        final_shift_ = false;
        step_count_ = 1;
        cmd_ = cmd;
        t_ = 0.0;
        plan_.clear();
        T_start_ = std::max(0.0, p_.TstepStart);   // 초기 체중이동 시간
        T_end_   = std::max(0.0, p_.TstepEnd);      // 종료 체중이동 시간

        // 첫 스윙발: 게걸음 방향에 맞춰 선택(발 교차 방지).
        Side firstSwing = (cmd.vy < -1e-4) ? Side::Right : Side::Left;
        Side firstStance = other(firstSwing);
        first_stance_ = firstStance;

        // 초기 토르소 앵커 = 양발 중점.
        anchor_tx_ = 0.5 * (left_foot_.x + right_foot_.x);
        anchor_ty_ = 0.5 * (left_foot_.y + right_foot_.y);
        anchor_tyaw_ = 0.5 * (left_foot_.yaw + right_foot_.yaw);

        auto footOf = [&](Side s) { return s == Side::Left ? left_foot_ : right_foot_; };
        // 초기 ZMP shift: 양발 중앙 → 첫 지지발.
        center_x_ = anchor_tx_; center_y_ = anchor_ty_;
        { Pose2 fs = footOf(firstStance); sshift_to_x_ = fs.x; sshift_to_y_ = fs.y; }

        // 스테핑 계획은 t = T_start_ 부터 시작(초기 shift 이후).
        // plan_[0]: 첫 스윙발의 현재 포즈(swing-from) — 시드(첫 스텝 ZMP 램프 없음)
        {
            Support sp; sp.side = firstSwing;
            Pose2 f = footOf(firstSwing);
            sp.fx = f.x; sp.fy = f.y; sp.fyaw = f.yaw;
            sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
            sp.t0 = T_start_ - p_.Tstep; sp.t1 = T_start_;
            sp.seed = true;
            plan_.push_back(sp);
        }
        // plan_[1]: 첫 지지발(current stance), [T_start_, T_start_+Tstep]
        {
            Support sp; sp.side = firstStance;
            Pose2 f = footOf(firstStance);
            sp.fx = f.x; sp.fy = f.y; sp.fyaw = f.yaw;
            sp.tx = anchor_tx_; sp.ty = anchor_ty_; sp.tyaw = anchor_tyaw_;
            sp.t0 = T_start_; sp.t1 = T_start_ + p_.Tstep;
            plan_.push_back(sp);
        }
        anchor_side_ = firstStance;
        anchor_t1_ = T_start_ + p_.Tstep;
        extendPlan();
        prev_cur_ = 1;
    }

    // --- 보행 정지 요청(엣지) → 마지막 스텝을 마무리한 뒤 정지(graceful stop) ---
    //   즉시 멈추면 흔들리므로, 전진을 멈추고(속도 0) 현재 스텝을 TstepEnd 로 끝낸 뒤
    //   다음 지지 교체 시점에 정지한다.
    if (!cmd.walk && walking_ && !stopping_) {
        stopping_ = true;
        cmd_.vx = cmd_.vy = cmd_.vyaw = 0.0;   // 더 이상 전진하지 않음
        regenerateTail();                       // 남은 미래를 제자리·TstepEnd 로
        // (return 하지 않고 보행 진행 분기로 내려가 스텝을 마무리)
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

    // (A) 종료 ZMP shift 진행 중: 지지발 → 양발중앙. 끝나면 정지(Stand).
    if (final_shift_) {
        if (t_ >= final_t0_ + T_end_) {
            walking_ = false; final_shift_ = false;
            phase_ = GaitPhase::Stand;
            left_foot_ = final_left_; right_foot_ = final_right_;
            support_side_ = Side::Left; support_pose_ = left_foot_;
            swing_target_ = FootPose{right_foot_.x, right_foot_.y, 0.0, right_foot_.yaw};
            plan_.clear();
            return;
        }
        phase_ = GaitPhase::DoubleSupport;   // support/swing 은 shift 시작 때 고정. ZMP 는 zmpAt.
        support_switched_ = false;
        return;
    }

    // (B) 초기 ZMP shift: 양발중앙 → 첫 지지발. 스텝 없이 체중만 천천히 옮긴다.
    if (t_ < T_start_) {
        phase_ = GaitPhase::DoubleSupport;
        support_side_ = first_stance_;
        Pose2 f  = (first_stance_ == Side::Left) ? left_foot_ : right_foot_;
        support_pose_ = f;
        Pose2 fw = (first_stance_ == Side::Left) ? right_foot_ : left_foot_;  // 스윙(반대발) 유지
        swing_target_ = FootPose{fw.x, fw.y, 0.0, fw.yaw};
        support_switched_ = false;
        return;
    }

    // (C) 스테핑.
    regenerateTail();      // 최신 명령으로 미확정 미래 재생성(1초 윈도우 실시간 반영)

    int cur = currentIndex();
    if (cur < 1) cur = 1;
    support_switched_ = (prev_cur_ >= 0 && cur != prev_cur_);
    prev_cur_ = cur;

    support_side_ = plan_[cur].side;
    support_pose_ = Pose2{plan_[cur].fx, plan_[cur].fy, plan_[cur].fyaw};

    // graceful stop: 정지 요청 후 한 스텝(지지 교체)이 마무리되면 → 종료 ZMP shift 시작.
    if (stopping_ && support_switched_) {
        Pose2 supF{plan_[cur].fx, plan_[cur].fy, plan_[cur].fyaw};
        Pose2 othF{plan_[cur - 1].fx, plan_[cur - 1].fy, plan_[cur - 1].fyaw};
        if (support_side_ == Side::Left) { final_left_ = supF; final_right_ = othF; }
        else                             { final_right_ = supF; final_left_ = othF; }
        final_shift_ = true; stopping_ = false;
        final_t0_ = t_;
        fshift_from_x_ = supF.x; fshift_from_y_ = supF.y;                 // 지지발
        fshift_to_x_ = 0.5 * (final_left_.x + final_right_.x);            // 양발 중앙
        fshift_to_y_ = 0.5 * (final_left_.y + final_right_.y);
        support_pose_ = supF;                                            // 피벗 유지
        swing_target_ = FootPose{othF.x, othF.y, 0.0, othF.yaw};
        phase_ = GaitPhase::DoubleSupport;
        return;
    }

    // 스윙발: cur-1(이전 착지) -> cur+1(다음 착지).
    //
    // 한 지지구간 [t0, t1) 의 구성 — DS 를 앞뒤로 "절반씩" 나눈다:
    //     │◀ Tds/2 (DS) ▶│◀── (1−dsRatio)·Tstep (SS) ──▶│◀ Tds/2 (DS) ▶│
    //     t0            이륙                           착지            t1
    //   이렇게 두면 지지 교체 경계(t1)를 중심으로 "앞 구간의 뒤쪽 반쪽 DS"와
    //   "뒤 구간의 앞쪽 반쪽 DS"가 이어져 길이 Tds 의 연속된 양발지지 구간이 된다.
    //   예) Tstep=1.0, dsRatio=0.5 → 0.25(DS) / 0.50(SS) / 0.25(DS).
    // 스텝마다 주기가 다를 수 있으므로 해당 구간의 실제 길이를 쓴다.
    const Support& from = plan_[cur - 1];
    const Support& to   = plan_[std::min<int>(cur + 1, (int)plan_.size() - 1)];
    const double Tstep = plan_[cur].t1 - plan_[cur].t0;
    const double Th    = 0.5 * clampd(p_.dsRatio, 0.0, 1.0) * Tstep;  // 앞/뒤 반쪽 DS
    const double tLift = Th;                                          // 이륙 시각
    const double tLand = Tstep - Th;                                  // 착지 시각
    double tau = t_ - plan_[cur].t0;

    if (tau < tLift) {              // 앞쪽 DS: 스윙발은 아직 이전 착지점에 접지
        phase_ = GaitPhase::DoubleSupport;
        swing_target_ = FootPose{from.fx, from.fy, 0.0, from.fyaw};
    } else if (tau < tLand) {       // SS: 들기 → 이동 → 내리기 (양 끝 속도 0)
        phase_ = GaitPhase::SingleSupport;
        double x   = cycloidXY(tau, tLift, tLand, from.fx, to.fx);
        double y   = cycloidXY(tau, tLift, tLand, from.fy, to.fy);
        double yaw = cycloidXY(tau, tLift, tLand, from.fyaw, to.fyaw);
        double z   = swingHeight(tau, tLift, tLand, p_.stepHeight);
        swing_target_ = FootPose{x, y, z, yaw};
    } else {                        // 뒤쪽 DS: 이미 착지 완료, 새 착지점에 접지 유지
        phase_ = GaitPhase::DoubleSupport;
        swing_target_ = FootPose{to.fx, to.fy, 0.0, to.fyaw};
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
