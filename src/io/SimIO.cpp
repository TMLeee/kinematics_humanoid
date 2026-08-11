#include "io/SimIO.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace kin {

namespace {
// 목표값(target)으로 rate 만큼 램프.
double ramp(double cur, double target, double rate, double dt) {
    double d = target - cur;
    double step = rate * dt;
    if (std::fabs(d) <= step) return target;
    return cur + (d > 0 ? step : -step);
}
}  // namespace

bool SimIO::init() {
    if (!env_.load(model_path_)) {
        std::fprintf(stderr, "[SimIO] model load failed: %s\n", model_path_.c_str());
        return false;
    }
    mjModel* m = env_.model();

    // 물리는 네이티브 timestep(0.5ms) 유지, 제어는 control_dt(2ms) 마다.
    double phys_dt = m->opt.timestep;
    substeps_ = std::max(1, (int)std::lround(control_dt_ / phys_dt));
    std::printf("[SimIO] physics dt=%.4f, control dt=%.4f, substeps=%d\n",
                phys_dt, control_dt_, substeps_);

    base_body_ = mj_name2id(m, mjOBJ_BODY, "base_link");
    if (base_body_ < 0) base_body_ = 1;

    // 모든 관절 위치제어 모드 + 현재(키프레임) 자세 유지.
    env_.setJointPositionMode(servo_kp_, servo_kv_);
    env_.holdCurrentPose();
    std::printf("[SimIO] servo gains: kp=%.0f kv=%.0f\n", servo_kp_, servo_kv_);

    if (with_viewer_) {
        if (!env_.initViewer("kin_humanoid — walking control")) {
            std::fprintf(stderr, "[SimIO] viewer init failed (headless?)\n");
            return false;
        }
    }
    return true;
}

bool SimIO::read(RobotState& out) {
    mjModel* m = env_.model();
    mjData*  d = env_.data();
    if (!m || !d) return false;

    if (out.q.size() != m->nq) out.q = VectorXd::Zero(m->nq);
    if (out.dq.size() != m->nv) out.dq = VectorXd::Zero(m->nv);
    for (int i = 0; i < m->nq; ++i) out.q(i)  = d->qpos[i];
    for (int i = 0; i < m->nv; ++i) out.dq(i) = d->qvel[i];

    // F/T 센서가 모델에 없으면 0 유지(키네마틱 제어에는 불필요).
    out.ftLeft.setZero();
    out.ftRight.setZero();

    out.valid = true;
    return true;
}

void SimIO::writeJointTargets(const VectorXd& qDesJoints) {
    mjModel* m = env_.model();
    mjData*  d = env_.data();
    if (!m || !d) return;
    // actuator i(0..nu-1) 는 관절 i 를 1:1 구동 → ctrl[i] = 목표 관절각.
    const int n = std::min<int>(m->nu, (int)qDesJoints.size());
    for (int i = 0; i < n; ++i) d->ctrl[i] = qDesJoints(i);
}

void SimIO::step() {
    for (int i = 0; i < substeps_; ++i) env_.step();
}

bool SimIO::running() {
    return !env_.viewerShouldClose();
}

void SimIO::render() {
    if (!with_viewer_) return;
    // sim 시간 1/60초마다 렌더(그 사이 물리는 여러 번 스텝됨). vsync 로 실시간 페이싱.
    double t = env_.data()->time;
    if (t - last_render_sim_ >= 1.0 / 60.0) {
        env_.render();
        last_render_sim_ = t;
    }
}

Eigen::Vector2d SimIO::measuredCom() const {
    mjData* d = const_cast<MujocoEnv&>(env_).data();
    const mjtNum* c = d->subtree_com + 3 * base_body_;
    return Eigen::Vector2d(c[0], c[1]);
}

bool SimIO::measuredZmp(Eigen::Vector2d& zmp) const {
    mjModel* m = const_cast<MujocoEnv&>(env_).model();
    mjData*  d = const_cast<MujocoEnv&>(env_).data();
    double fz_tot = 0.0, sx = 0.0, sy = 0.0;
    mjtNum f6[6];
    for (int i = 0; i < d->ncon; ++i) {
        mj_contactForce(m, d, i, f6);            // 접촉 프레임 기준 wrench
        const mjContact& c = d->contact[i];
        // 접촉 프레임 축(world)로 힘을 world 좌표로 변환: fw = Σ f_k * frame_row_k
        mjtNum fw2 = c.frame[0 * 3 + 2] * f6[0]  // world z 성분만 필요(수직력)
                   + c.frame[1 * 3 + 2] * f6[1]
                   + c.frame[2 * 3 + 2] * f6[2];
        if (fw2 <= 0.0) continue;
        fz_tot += fw2;
        sx += c.pos[0] * fw2;
        sy += c.pos[1] * fw2;
    }
    if (fz_tot < 1e-6) return false;             // 공중(접촉 없음)
    zmp = Eigen::Vector2d(sx / fz_tot, sy / fz_tot);
    return true;
}

VelocityCommand SimIO::velocityCommand() {
    MujocoEnv::KeyInput k = env_.pollKeys();

    if (k.space) cmd_.walk = !cmd_.walk;   // 보행 on/off 토글
    if (k.x)     cmd_.walk = false;

    // 목표 속도(키 홀드 기준).
    double tvx = (k.w ? v_fwd_max_ : 0.0) + (k.s ? -v_fwd_max_ : 0.0);
    double tvy = (k.a ? v_lat_max_ : 0.0) + (k.d ? -v_lat_max_ : 0.0);
    double tvyaw = (k.q ? v_yaw_max_ : 0.0) + (k.e ? -v_yaw_max_ : 0.0);
    if (!cmd_.walk || k.x) { tvx = tvy = tvyaw = 0.0; }

    // 부드럽게 램프.
    cmd_.vx   = ramp(cmd_.vx,   tvx,   v_accel_,   control_dt_);
    cmd_.vy   = ramp(cmd_.vy,   tvy,   v_accel_,   control_dt_);
    cmd_.vyaw = ramp(cmd_.vyaw, tvyaw, yaw_accel_, control_dt_);
    return cmd_;
}

}  // namespace kin
