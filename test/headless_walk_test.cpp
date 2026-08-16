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

    // 설정 로드(객체 생성 전): config/walking_config.json → gConfig.
    kin::config::loadFromJson(kin::config::kDefaultConfigPath);
    // 이득/폐루프 오버라이드(제어기 생성 전이어야 g_ 기본값에 반영).
    if (getenv("KIN_KPCOM")) kin::config::gConfig.kpCom = atof(getenv("KIN_KPCOM"));
    if (getenv("KIN_CLOOP")) kin::config::gConfig.closedLoop = atof(getenv("KIN_CLOOP"));

    kin::SimIO io(model_path, /*with_viewer=*/false);
    // 모터 서보 이득 오버라이드(KIN_KP / KIN_KV) — init 전에 설정.
    {
        double kp = getenv("KIN_KP") ? atof(getenv("KIN_KP")) : kin::config::gConfig.servoKp;
        double kv = getenv("KIN_KV") ? atof(getenv("KIN_KV")) : kin::config::gConfig.servoKv;
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
        if (getenv("KIN_TSTART")) gp.TstepStart = atof(getenv("KIN_TSTART"));
        if (getenv("KIN_TEND"))   gp.TstepEnd   = atof(getenv("KIN_TEND"));
        if (getenv("KIN_DS"))    gp.dsRatio = atof(getenv("KIN_DS"));
        if (getenv("KIN_H"))     gp.stepHeight = atof(getenv("KIN_H"));
        controller.setGaitParams(gp);
        if (getenv("KIN_COMZ")) controller.setComHeight(atof(getenv("KIN_COMZ")));
        if (getenv("KIN_STAB")) controller.setStabilizer(atof(getenv("KIN_STAB")));
        // 폐루프(측정 관절각 + 지지발 지면고정으로 COM 계산) / 발 기준점 토글
        if (getenv("KIN_CLOSED"))  kin::config::gConfig.closedLoop  = atof(getenv("KIN_CLOSED"));
        if (getenv("KIN_ANKLEREF")) kin::config::gConfig.footRefAnkle = atof(getenv("KIN_ANKLEREF"));
        if (getenv("KIN_COMMEAS"))  kin::config::gConfig.comMeasPlanted = atof(getenv("KIN_COMMEAS"));
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
        e.waist = onoff("KIN_WAIST", true);
        controller.setEnable(e);
        std::printf("tasks: com=%d swing=%d hand=%d pelvis=%d waist=%d\n",
                    e.com, e.swing, e.hand, e.pelvis, e.waist);
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
        // 점프 진단: 한 tick 관절 지령 변화(qStep)가 임계 이상이면 유발 task 와 함께 출력.
        if (getenv("KIN_JUMP")) {
            const auto& d = controller.debug();
            double thr = getenv("KIN_JTHR") ? atof(getenv("KIN_JTHR")) : 0.004;  // rad/tick
            if (d.qStep > thr) {
                std::printf("JUMP %s t=%.3f qStep=%.4f dqMax=%.2f@j%d | xdot: com=%.2f sw=%.2f hand=%.2f pel=%.2f wst=%.2f | err: com=%.3f sw=%.3f pel=%.3f | sup=%s%s\n",
                    tag, k * dt, d.qStep, d.dqMax, d.dqMaxJoint,
                    d.xdotCom, d.xdotSwing, d.xdotHand, d.xdotPelvis, d.xdotWaist,
                    d.errCom, d.errSwing, d.errPelvis,
                    d.supportSide == 0 ? "L" : "R", d.supportSwitched ? " SWITCH" : "");
            }
        }
        // walk-start 상세 궤적: 발이 뜨는 원인(COM이 지지발 위로 오기 전에 스윙 리프트?).
        if (getenv("KIN_WSTART") && (std::string(tag) == "WALK" || std::string(tag) == "STOP")
            && k * dt < 5.0 && k % (int)std::lround(0.2 / dt) == 0) {
            const auto& d = controller.debug();
            io.read(state);
            Eigen::Vector2d cm = io.measuredCom();
            Eigen::Vector2d zm; bool zv = io.measuredZmp(zm);
            model.setState(state.q, kin::VectorXd::Zero(model.nv()));
            model.updateKinematics();
            kin::Vector3d off(0, 0, kin::kSoleOffsetZ);
            double Lz = model.bodyPos(model.bodyId(kin::BodyNames::LFoot), off).z();
            double Rz = model.bodyPos(model.bodyId(kin::BodyNames::RFoot), off).z();
            const char* ph = d.phase == 2 ? "SS" : d.phase == 1 ? "DS" : "st";
            std::printf("WS %s t=%.2f %s sup=%s | zmpRef.y=%+.3f comRef.y=%+.3f | comMeas.y=%+.3f zmpMeas.y=%+.3f | swZcmd=%.3f | Lz=%.3f Rz=%.3f\n",
                tag, k * dt, ph, d.supportSide == 0 ? "L" : "R",
                d.zmpRef.y(), d.comRef.y(), cm.y(), zv ? zm.y() : 9.99, d.swingZ, Lz, Rz);
        }
        // LIPM 검증: step ZMP ref 를 preview COM 이 잘 따라가는지.
        //   zmpRef(step 입력) vs comRef(preview 출력) vs zmpLIPM(=C·x, COM 에서 나오는 ZMP).
        //   preview 가 옳으면 zmpLIPM ≈ zmpRef 이고 comRef 는 부드럽게 zmpRef 로 수렴.
        if (getenv("KIN_LIPM") && std::string(tag) == "WALK" && k * dt < 6.0
            && k % (int)std::lround(0.1 / dt) == 0) {
            const auto& d = controller.debug();
            Eigen::Vector2d zmMeas; bool zvMeas = io.measuredZmp(zmMeas);
            Eigen::Vector2d cmMeas = io.measuredCom();
            // 비교: zmpRef(입력) | comRef(preview) | zmpLIPM(=C·x, 모델 예측 ZMP)
            //       | comMeas(실제 COM) | zmpMeas(실제 측정 ZMP, 접촉 CoP)
            std::printf("LIPM t=%.2f Y| zmpRef=%+.3f comRef=%+.3f zmpLIPM=%+.3f || comMeas=%+.3f zmpMeas=%+.3f\n",
                k * dt, d.zmpRef.y(), d.comRef.y(), d.zmpFromCom.y(),
                cmMeas.y(), zvMeas ? zmMeas.y() : 9.99);
        }
        // 신호 계측(그래프 오프셋 진단): ref(제어기) vs 측정(sim).
        if (getenv("KIN_SIG") && k % (int)std::lround(0.2 / dt) == 0) {
            const auto& d = controller.debug();
            io.read(state);
            Eigen::Vector2d cm = io.measuredCom();          // sim subtree_com (측정)
            // 동일 측정 상태에서 모델(MujocoModel)이 계산한 COM.
            model.setState(state.q, kin::VectorXd::Zero(model.nv()));
            model.updateKinematics();
            kin::Vector3d mc = model.com();
            std::printf("SIG %s t=%.2f | simCOM=(%.4f,%.4f,%.4f) modelCOM=(%.4f,%.4f,%.4f) diff=(%.1e,%.1e)\n",
                tag, k * dt, cm.x(), cm.y(), 0.0, mc.x(), mc.y(), mc.z(),
                cm.x() - mc.x(), cm.y() - mc.y());
        }
        io.step();
        if (!getenv("KIN_SIG") && k % (int)std::lround(0.5 / dt) == 0) {
            std::printf("[%s t=%.2f] base=(%.3f,%.3f,%.3f) %s\n",
                        tag, k * dt, state.q(0), state.q(1), state.q(2),
                        controller.statusText().c_str());
        }
    };

    // --- IDLE 1.5초: 아무 입력 없이 유지(시작 튐 확인) ---
    for (int k = 0; k < (int)std::lround(1.5 / dt) && io.running(); ++k)
        tick(kin::VelocityCommand{}, k, "IDLE");
    io.read(state);
    std::printf(">> after idle: base=(%.3f,%.3f,%.3f) drift=(%.4f,%.4f)\n",
                state.q(0), state.q(1), state.q(2), state.q(0) - base_x0, state.q(1));

    // --- 'h' 준비 자세 이동(엣지 한 번) 후 WBC 시작까지 대기(~3초) ---
    { kin::VelocityCommand c; c.prepareEdge = true; tick(c, 0, "PREP"); }
    for (int k = 1; k < (int)std::lround(3.0 / dt) && io.running(); ++k)
        tick(kin::VelocityCommand{}, k, "PREP");
    io.read(state);
    std::printf(">> after prepare: base_z=%.3f mode=%s\n", state.q(2),
                controller.mode() == kin::HumanoidController::Mode::Active ? "ACTIVE" : "NOT-ACTIVE");

    // --- Space 로 보행 시작(엣지 한 번) + 속도 유지 ---
    double vx = getenv("KIN_VX") ? atof(getenv("KIN_VX")) : 0.04;
    double vy = getenv("KIN_VY") ? atof(getenv("KIN_VY")) : 0.0;
    double vyaw = getenv("KIN_VYAW") ? atof(getenv("KIN_VYAW")) : 0.0;
    double bx_walk0 = state.q(0);
    for (int k = 0; k < nWalk && io.running(); ++k) {
        kin::VelocityCommand c; c.vx = vx; c.vy = vy; c.vyaw = vyaw;
        if (k == 0) c.spaceEdge = true;   // 보행 시작 토글
        tick(c, k, "WALK");
    }
    io.read(state);
    std::printf(">> after walk: base=(%.3f,%.3f,%.3f) dx=%.3f, fell=%s, NaN=%s\n",
                state.q(0), state.q(1), state.q(2), state.q(0) - bx_walk0,
                state.q(2) < 0.6 ? "YES" : "no", ok ? "no" : "YES");

    // --- Space 로 정지(엣지 한 번) → graceful stop, 5초 유지 ---
    int nStop = (int)std::lround(5.0 / dt);
    for (int k = 0; k < nStop && io.running(); ++k) {
        kin::VelocityCommand c;
        if (k == 0) c.spaceEdge = true;   // 보행 정지 토글
        tick(c, k, "STOP");
    }
    io.read(state);
    std::printf(">> after stop: base=(%.3f,%.3f,%.3f), fell=%s, NaN=%s\n",
                state.q(0), state.q(1), state.q(2),
                state.q(2) < 0.6 ? "YES" : "no", ok ? "no" : "YES");

    std::printf("RESULT: %s\n", (ok && state.q(2) > 0.6) ? "PASS (no NaN, stayed upright)"
                                                          : "CHECK (see above)");
    return 0;
}
