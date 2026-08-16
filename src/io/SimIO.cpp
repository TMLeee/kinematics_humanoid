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

    // 발 F/T 센서(모델 XML 의 <force>/<torque> + 부착 site) 해석.
    {
        const char* sname[4] = {"LF_Force_sensor", "LF_Torque_sensor",
                                "RF_Force_sensor", "RF_Torque_sensor"};
        const char* tname[2] = {"LF_FT", "RF_FT"};
        ft_ok_ = true;
        for (int i = 0; i < 4; ++i) {
            int sid = mj_name2id(m, mjOBJ_SENSOR, sname[i]);
            ft_adr_[i] = (sid >= 0) ? m->sensor_adr[sid] : -1;
            if (ft_adr_[i] < 0) ft_ok_ = false;
        }
        for (int i = 0; i < 2; ++i) {
            ft_site_[i] = mj_name2id(m, mjOBJ_SITE, tname[i]);
            if (ft_site_[i] < 0) ft_ok_ = false;
        }
        std::printf("[SimIO] foot F/T: %s (measuredZmp %s)\n",
                    ft_ok_ ? "OK" : "MISSING",
                    ft_ok_ ? "= F/T CoP" : "unavailable → 접촉 CoP 로 대체");
    }

    // 모든 관절 위치제어 모드 + 현재(키프레임) 자세 유지.
    env_.setJointPositionMode(servo_kp_, servo_kv_);

    // 발목 2축(pitch/roll) 별도 서보 이득(설정 시). 관절 이름으로 액추에이터를 찾아 개별 적용.
    //   kp<=0 이면 전역 사용, kv<=0 이면 전역 감쇠(servo_kv_) 사용.
    auto applyAnkle = [&](const char* jname, double kp, double kv) {
        if (kp <= 0.0) return;
        if (kv <= 0.0) kv = servo_kv_;
        int jid = mj_name2id(m, mjOBJ_JOINT, jname);
        if (jid < 0) return;
        for (int a = 0; a < m->nu; ++a)
            if (m->actuator_trnid[2 * a] == jid) {
                env_.setActuatorServoGain(a, kp, kv);
                std::printf("[SimIO] ankle gain: %-20s kp=%.0f kv=%.0f\n", jname, kp, kv);
                break;
            }
    };
    applyAnkle("L_AnklePitch_Joint", config::gConfig.anklePitchKp, config::gConfig.anklePitchKv);
    applyAnkle("R_AnklePitch_Joint", config::gConfig.anklePitchKp, config::gConfig.anklePitchKv);
    applyAnkle("L_AnkleRoll_Joint",  config::gConfig.ankleRollKp,  config::gConfig.ankleRollKv);
    applyAnkle("R_AnkleRoll_Joint",  config::gConfig.ankleRollKp,  config::gConfig.ankleRollKv);

    env_.holdCurrentPose();
    eint_ = VectorXd::Zero(m->nu);
    std::printf("[SimIO] servo: kp=%.0f kv=%.0f ki=%.0f (Iclamp=%.2f) gravComp=%.2f\n",
                servo_kp_, servo_kv_, servo_ki_, i_clamp_, grav_comp_);

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

    // 발 F/T (센서 site 프레임, 센서 원점 기준 모멘트). 센서가 없으면 0 유지.
    out.ftLeft.setZero();
    out.ftRight.setZero();
    out.ftValid = ft_ok_;
    if (ft_ok_) {
        for (int i = 0; i < 3; ++i) {
            out.ftLeft(i)      = d->sensordata[ft_adr_[0] + i];
            out.ftLeft(3 + i)  = d->sensordata[ft_adr_[1] + i];
            out.ftRight(i)     = d->sensordata[ft_adr_[2] + i];
            out.ftRight(3 + i) = d->sensordata[ft_adr_[3] + i];
        }
    }

    out.valid = true;
    return true;
}

void SimIO::writeJointTargets(const VectorXd& qDesJoints) {
    mjModel* m = env_.model();
    mjData*  d = env_.data();
    if (!m || !d) return;
    if (eint_.size() != m->nu) eint_ = VectorXd::Zero(m->nu);
    const int n = std::min<int>(m->nu, (int)qDesJoints.size());

    // 속도 피드포워드용 q̇_des = (q_des(k) − q_des(k−1)) / dt.
    //   지령은 jointsInt_ += dq·dt 로 만들어져 C1 연속이라 수치미분이 안전하다.
    if (qdes_prev_.size() != qDesJoints.size()) {
        qdes_prev_ = qDesJoints; have_qdes_prev_ = false;
    }
    for (int i = 0; i < n; ++i) {
        int jid = m->actuator_trnid[2 * i];
        if (jid < 0) { d->ctrl[i] = qDesJoints(i); continue; }
        int qadr = m->jnt_qposadr[jid];
        int dadr = m->jnt_dofadr[jid];

        // 적분 I텀: 목표 위치에 (ki/kp)*∫e 를 더한다.
        //   → MuJoCo PD 가 force = kp*(ctrl-q) - kv*qd = kp*e + ki*∫e - kv*qd 를 적용.
        //   중력 부하로 인한 정상상태 sag(≈부하/kp)를 서서히 0 으로 만든다.
        double e = qDesJoints(i) - d->qpos[qadr];
        double offset = 0.0;
        if (servo_ki_ > 0.0 && servo_kp_ > 0.0) {
            eint_(i) += e * control_dt_;
            offset = (servo_ki_ / servo_kp_) * eint_(i);
            const double omax = i_clamp_;              // anti-windup
            if (offset >  omax) { offset =  omax; eint_(i) =  omax * servo_kp_ / servo_ki_; }
            if (offset < -omax) { offset = -omax; eint_(i) = -omax * servo_kp_ / servo_ki_; }
        }
        d->ctrl[i] = qDesJoints(i) + offset;

        if (dadr < 0) continue;
        // ── 피드포워드 항 두 개를 qfrc_applied 에 함께 싣는다 ──
        double ff = 0.0;

        // (1) 중력보상: qfrc_bias(중력+코리올리)를 더한다(즉시 sag 제거).
        ff += grav_comp_ * d->qfrc_bias[dadr];

        // (2) 속도 피드포워드.
        //   MuJoCo 위치서보는 force = kp*(ctrl − q) − kv*q̇ 로, q̇_des 항이 없다.
        //   즉 kv 가 외란만이 아니라 **지령된 움직임 자체를 반대로 밀어낸다**.
        //   그래서 kv 를 올리면 감쇠가 아니라 추종 방해가 되어 보행이 나빠졌다(실측).
        //   여기서 +kv·q̇_des 를 더해 실효 법칙을 force = kp*e − kv*(q̇ − q̇_des) 로 만든다.
        //   → 이제 kv 는 "추종오차 속도"만 감쇠하므로 자유롭게 올릴 수 있다.
        if (kv_ff_ != 0.0 && have_qdes_prev_ && control_dt_ > 1e-9) {
            const double kv = -m->actuator_biasprm[i * mjNBIAS + 2];   // 액추에이터별 실제 kv
            const double qdotDes = (qDesJoints(i) - qdes_prev_(i)) / control_dt_;
            ff += kv_ff_ * kv * qdotDes;
        }

        // 토크 한계 안으로 클램프(액추에이터 포화와 이중으로 어긋나지 않게).
        if (m->actuator_forcelimited[i]) {
            const double lo = m->actuator_forcerange[2 * i], hi = m->actuator_forcerange[2 * i + 1];
            ff = std::max(lo, std::min(hi, ff));
        }
        d->qfrc_applied[dadr] = ff;
    }
    qdes_prev_ = qDesJoints;
    have_qdes_prev_ = true;
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

Eigen::Vector3d SimIO::measuredCom3() const {
    mjData* d = const_cast<MujocoEnv&>(env_).data();
    const mjtNum* c = d->subtree_com + 3 * base_body_;
    return Eigen::Vector3d(c[0], c[1], c[2]);
}

bool SimIO::measuredZmp(Eigen::Vector2d& zmp) const {
    mjModel* m = const_cast<MujocoEnv&>(env_).model();
    mjData*  d = const_cast<MujocoEnv&>(env_).data();
    if (!m || !d) return false;
    // 센서가 없는 모델이면 접촉 기반으로 대체(기능 정지보다 낫다).
    if (!ft_ok_) return measuredZmpContact(zmp);

    // 양발 F/T wrench 를 world 원점 기준으로 합산 → 지면(z=0) 위 CoP.
    //   ZMP 는 "합력의 작용점"이므로 발별 CoP 를 평균내지 않고 wrench 를 합산한다.
    Vector3d fSum = Vector3d::Zero(), MoSum = Vector3d::Zero();
    for (int s = 0; s < 2; ++s) {
        Vector6d ft;
        for (int i = 0; i < 3; ++i) {
            ft(i)     = d->sensordata[ft_adr_[2 * s] + i];
            ft(3 + i) = d->sensordata[ft_adr_[2 * s + 1] + i];
        }
        const int sid = ft_site_[s];
        Matrix3d R;
        const mjtNum* x = d->site_xmat + 9 * sid;      // row-major
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) R(r, c) = x[3 * r + c];
        Vector3d p(d->site_xpos[3 * sid], d->site_xpos[3 * sid + 1], d->site_xpos[3 * sid + 2]);
        accumulateWrench(ft, R, p, fSum, MoSum);
    }
    return copOnPlane(fSum, MoSum, /*z0=*/0.0, /*fzMin=*/10.0, zmp);
}

bool SimIO::measuredZmpContact(Eigen::Vector2d& zmp) const {
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

    // 엣지/트리거는 그대로 제어기로 전달(보행 토글/준비 자세는 제어기가 상태로 관리).
    cmd_.spaceEdge   = k.space;
    cmd_.prepareEdge = k.h;
    cmd_.stop        = k.x;
    cmd_.walk        = false;   // 유효 보행 상태는 HumanoidController 가 설정한다.

    // 목표 속도(키 홀드 기준). 정지(x) 면 0.
    double tvx   = (k.w ? v_fwd_max_ : 0.0) + (k.s ? -v_fwd_max_ : 0.0);
    double tvy   = (k.a ? v_lat_max_ : 0.0) + (k.d ? -v_lat_max_ : 0.0);
    double tvyaw = (k.q ? v_yaw_max_ : 0.0) + (k.e ? -v_yaw_max_ : 0.0);
    if (k.x) { tvx = tvy = tvyaw = 0.0; }

    // 부드럽게 램프.
    cmd_.vx   = ramp(cmd_.vx,   tvx,   v_accel_,   control_dt_);
    cmd_.vy   = ramp(cmd_.vy,   tvy,   v_accel_,   control_dt_);
    cmd_.vyaw = ramp(cmd_.vyaw, tvyaw, yaw_accel_, control_dt_);
    return cmd_;
}

}  // namespace kin
