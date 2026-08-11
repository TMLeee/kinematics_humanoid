// 헤드리스 검증 하네스(뷰어 없이 제어 루프를 돌려 안정성/보행을 확인).
//   - 3초 정지: 자세 유지(넘어지지 않음, NaN 없음) 확인
//   - 이후 전진 보행 명령: base 가 실제로 전진하는지 확인
#include "io/SimIO.h"
#include "model/MujocoModel.h"
#include "controller/HumanoidController.h"

#include <cstdio>
#include <cmath>

int main(int argc, char** argv) {
    const char* model_path =
        (argc > 1) ? argv[1]
                   : "model/dyros_tocabi_v2/tocabi_description/mujoco_model/dyros_tocabi.xml";

    kin::SimIO io(model_path, /*with_viewer=*/false);
    // 모터 서보 이득 오버라이드(KIN_KP / KIN_KV) — init 전에 설정.
    {
        double kp = getenv("KIN_KP") ? atof(getenv("KIN_KP")) : kin::config::kServoKp;
        double kv = getenv("KIN_KV") ? atof(getenv("KIN_KV")) : kin::config::kServoKv;
        io.setServoGains(kp, kv);
    }
    if (!io.init()) { std::printf("SimIO init failed\n"); return 1; }

    kin::MujocoModel model;
    if (!model.load(model_path)) { std::printf("model load failed\n"); return 1; }

    kin::RobotState state;
    io.read(state);
    kin::HumanoidController controller;
    {   // 보행 파라미터(환경변수로 튜닝): KIN_TSTEP, KIN_DS, KIN_H
        kin::FootstepGenerator::Params gp;
        if (getenv("KIN_TSTEP")) gp.Tstep = atof(getenv("KIN_TSTEP"));
        if (getenv("KIN_DS"))    gp.dsRatio = atof(getenv("KIN_DS"));
        if (getenv("KIN_H"))     gp.stepHeight = atof(getenv("KIN_H"));
        controller.setGaitParams(gp);
        if (getenv("KIN_COMZ")) controller.setComHeight(atof(getenv("KIN_COMZ")));
        if (getenv("KIN_STAB")) controller.setStabilizer(atof(getenv("KIN_STAB")));
        std::printf("gait: Tstep=%.2f dsRatio=%.2f stepH=%.3f comz=%s\n",
                    gp.Tstep, gp.dsRatio, gp.stepHeight, getenv("KIN_COMZ") ? getenv("KIN_COMZ") : "auto");
    }
    controller.init(&model, io.controlDt(), state);

    // 환경변수로 task on/off (디버그): KIN_COM/KIN_SWING/KIN_HAND/KIN_PELVIS (기본 1)
    {
        auto onoff = [](const char* n, bool d) {
            const char* v = getenv(n); return v ? (v[0] != '0') : d; };
        kin::HumanoidController::Enable e;
        e.com = onoff("KIN_COM", true); e.swing = onoff("KIN_SWING", true);
        e.hand = onoff("KIN_HAND", true); e.pelvis = onoff("KIN_PELVIS", true);
        controller.setEnable(e);
        std::printf("tasks: com=%d swing=%d hand=%d pelvis=%d\n",
                    e.com, e.swing, e.hand, e.pelvis);
    }

    // 관절 한계
    {
        mjModel* m = io.env().model();
        kin::VectorXd lo(m->nu), hi(m->nu);
        for (int i = 0; i < m->nu; ++i) {
            int jid = m->actuator_trnid[2 * i];
            if (jid >= 0 && m->jnt_limited[jid]) { lo(i) = m->jnt_range[2*jid]; hi(i) = m->jnt_range[2*jid+1]; }
            else { lo(i) = -1e9; hi(i) = 1e9; }
        }
        controller.setJointLimits(lo, hi);
    }

    double dt = io.controlDt();
    int nStand = (int)std::lround(2.0 / dt);
    int nWalk  = (int)std::lround((getenv("KIN_WALKSEC") ? atof(getenv("KIN_WALKSEC")) : 8.0) / dt);

    double base_x0 = state.q(0), base_z0 = state.q(2);
    std::printf("start: base=(%.3f,%.3f,%.3f) com=(%.3f,%.3f,%.3f)\n",
                state.q(0), state.q(1), state.q(2),
                model.com().x(), model.com().y(), model.com().z());

    bool ok = true;
    auto tick = [&](kin::VelocityCommand cmd, int k, const char* tag) {
        io.read(state);
        kin::VectorXd qDes = controller.update(state, cmd);
        for (int i = 0; i < qDes.size(); ++i)
            if (std::isnan(qDes(i)) || std::isinf(qDes(i))) { ok = false; }
        io.writeJointTargets(qDes);
        io.step();
        if (k % (int)std::lround(0.5 / dt) == 0) {
            std::printf("[%s t=%.2f] base=(%.3f,%.3f,%.3f) %s\n",
                        tag, k * dt, state.q(0), state.q(1), state.q(2),
                        controller.statusText().c_str());
        }
    };

    // --- 정지 3초 ---
    for (int k = 0; k < nStand && io.running(); ++k) tick(kin::VelocityCommand{}, k, "STAND");
    io.read(state);
    double z_after_stand = state.q(2);
    std::printf(">> after stand: base_z=%.3f (start %.3f), fell=%s, NaN=%s\n",
                z_after_stand, base_z0, z_after_stand < 0.6 ? "YES" : "no",
                ok ? "no" : "YES");

    // --- 전진 보행 4초 (KIN_VX 로 속도 지정, 기본 0.08; 0 이면 제자리 걸음) ---
    kin::VelocityCommand walk; walk.walk = true;
    walk.vx = getenv("KIN_VX") ? atof(getenv("KIN_VX")) : 0.08;
    walk.vy = getenv("KIN_VY") ? atof(getenv("KIN_VY")) : 0.0;
    walk.vyaw = getenv("KIN_VYAW") ? atof(getenv("KIN_VYAW")) : 0.0;
    for (int k = 0; k < nWalk && io.running(); ++k) tick(walk, k, "WALK");
    io.read(state);
    std::printf(">> after walk: base=(%.3f,%.3f,%.3f) dx=%.3f, fell=%s, NaN=%s\n",
                state.q(0), state.q(1), state.q(2), state.q(0) - base_x0,
                state.q(2) < 0.6 ? "YES" : "no", ok ? "no" : "YES");

    std::printf("RESULT: %s\n", (ok && state.q(2) > 0.6) ? "PASS (no NaN, stayed upright)"
                                                          : "CHECK (see above)");
    return 0;
}
