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
