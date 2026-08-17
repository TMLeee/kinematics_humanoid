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

    // ── IK 정식화(floating base) / 골반 좌표 A/B 노브 ──────────────────────
    //  lamSupport 는 HumanoidController::Gains 의 기본값으로 **생성 시점에** 복사되므로
    //  반드시 컨트롤러 생성 전에 덮어써야 한다(여기). 나머지 노브는 update() 에서 매 tick
    //  gConfig 를 읽으므로 시점이 자유롭지만, 한곳에 모아 두는 편이 혼동이 없다.
    if (getenv("KIN_FB"))       kin::config::gConfig.floatingBase   = atof(getenv("KIN_FB"));
    if (getenv("KIN_DSBOTH"))   kin::config::gConfig.dsBothFeet     = atof(getenv("KIN_DSBOTH"));
    if (getenv("KIN_LAMCON"))   kin::config::gConfig.lamContact     = atof(getenv("KIN_LAMCON"));
    if (getenv("KIN_LAMSUP"))   kin::config::gConfig.lamSupport     = atof(getenv("KIN_LAMSUP"));
    if (getenv("KIN_IKCMP"))    kin::config::gConfig.ikCompare      = atof(getenv("KIN_IKCMP"));
    if (getenv("KIN_PELBODY"))  kin::config::gConfig.pelvisBodyFrame = atof(getenv("KIN_PELBODY"));
    if (getenv("KIN_PELIMU"))   kin::config::gConfig.pelvisImuRollPitch = atof(getenv("KIN_PELIMU"));
    if (getenv("KIN_KPPELYAW")) kin::config::gConfig.kpPelvisYaw    = atof(getenv("KIN_KPPELYAW"));
    // 발목 어드미턴스 A/B (admParams_ 도 컨트롤러 생성 시점에 gConfig 에서 복사된다).
    if (getenv("KIN_ADM"))    kin::config::gConfig.ankleAdmEnable = atof(getenv("KIN_ADM"));
    if (getenv("KIN_ADMKP"))  kin::config::gConfig.ankleAdmKPitch = atof(getenv("KIN_ADMKP"));
    if (getenv("KIN_ADMKR"))  kin::config::gConfig.ankleAdmKRoll  = atof(getenv("KIN_ADMKR"));
    // KIN_LAMALL: 모든 DLS 감쇠를 한 값으로. 동치성 검증 전용.
    //   두 정식화의 차이는 **감쇠가 재는 노름**이다: reduced 는 ‖dq_joint‖ 을,
    //   floating 은 ‖[dq_base; dq_joint]‖ 을 벌점한다. λ→0 이면 그 차이도 0 이 되어야 한다.
    if (getenv("KIN_LAMALL")) {
        const double L = atof(getenv("KIN_LAMALL"));
        auto& c = kin::config::gConfig;
        c.lamSupport = c.lamCom = c.lamSwing = c.lamHand = c.lamPelvis = c.lamWaist = L;
        c.lamContact = L;
    }
    std::printf("IK: floatingBase=%.0f dsBothFeet=%.0f lamSup=%.1e lamCon=%.1e ikCompare=%.0f"
                " | pelvis: bodyFrame=%.0f imuRollPitch=%.0f kpYaw=%.1f\n",
                kin::config::gConfig.floatingBase, kin::config::gConfig.dsBothFeet,
                kin::config::gConfig.lamSupport, kin::config::gConfig.lamContact,
                kin::config::gConfig.ikCompare, kin::config::gConfig.pelvisBodyFrame,
                kin::config::gConfig.pelvisImuRollPitch, kin::config::gConfig.kpPelvisYaw);

    kin::SimIO io(model_path, /*with_viewer=*/false);
    // 모터 서보 이득 오버라이드(KIN_KP / KIN_KV) — init 전에 설정.
    {
        double kp = getenv("KIN_KP") ? atof(getenv("KIN_KP")) : kin::config::gConfig.servoKp;
        double kv = getenv("KIN_KV") ? atof(getenv("KIN_KV")) : kin::config::gConfig.servoKv;
        io.setServoGains(kp, kv);
        if (getenv("KIN_KI"))   io.setServoIntegral(atof(getenv("KIN_KI")), 0.10);
        if (getenv("KIN_GRAV")) io.setGravityComp(atof(getenv("KIN_GRAV")));
        if (getenv("KIN_KVFF")) io.setVelFeedforward(atof(getenv("KIN_KVFF")));
        std::printf("servo override: kp=%.0f kv=%.0f kvFF=%s\n", kp, kv,
                    getenv("KIN_KVFF") ? getenv("KIN_KVFF") : "(config)");
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

    // ── 정량 계측(A/B 비교용) ────────────────────────────────────────────────
    //  두 정식화를 같은 잣대로 비교하기 위한 누적기. WALK 구간만 집계한다.
    //  "측정" 신호는 전부 시뮬레이터 ground truth (제어기가 믿는 값이 아니라 실제 물리).
    struct Acc {
        double s2 = 0.0, mx = 0.0;
        long n = 0;
        void add(double v) { s2 += v * v; mx = std::max(mx, std::fabs(v)); ++n; }
        double rms() const { return n ? std::sqrt(s2 / (double)n) : 0.0; }
    };
    struct Metrics {
        Acc zmpEx, zmpEy;        // 측정 ZMP − 계획 ZMP(계단형) [m]
        Acc zmpLx, zmpLy;        // 측정 ZMP − LIPM 함의 ZMP(preview COM ref 에서 나오는 값) [m]
                                 //   계단형 레퍼런스와 달리 연속이라 추종 품질 지표로 적합.
        Acc comEx, comEy;        // 측정 COM − preview COM ref [m]
        Acc comEint;             // 제어기 내부 COM 오차 [m]
        Acc pelRoll, pelPitch;   // 진짜 world 골반 roll/pitch [deg]
        Acc slipSS, slipDS;      // 접지 발 지령 속도 [m/s] (SS 구간 / DS 구간)
        Acc qStep;               // tick 당 관절 지령 변화 [rad]
        double dsDevMax = 0.0, slipAngMax = 0.0, ikResMax = 0.0;
        double ikComMax = 0.0, ikSwMax = 0.0, ikConMax = 0.0;   // task 속도 차 [m/s]
        double sigMinMin = 1e9, sigMinSum = 0.0;
        double sigRawMinMin = 1e9, sigRawMaxMax = 0.0;   // 접촉 구속 전 COM 자코비안
        double sigPelMinMin = 1e9, sigPelSum = 0.0;      // 골반 task 제어 권한
        // SS / DS 를 나눠 평균: 양발 구속의 효과는 DS 구간에만 나타난다.
        double sigComSsSum = 0.0, sigComDsSum = 0.0, sigPelSsSum = 0.0, sigPelDsSum = 0.0;
        long   sigSsN = 0, sigDsN = 0;
        long   sigN = 0;
        double zmpFootX = 0.0, zmpFootY = 0.0;   // 진짜 SS 중 |측정 ZMP − 지지발 중심| 최대 [m]
        long   ssPlan = 0, ssTrue = 0;           // 계획상 SS tick / 실제로 한 발만 접지한 tick
    } M;
    bool collect = false;

    bool ok = true;
    kin::VectorXd lastQDes;
    auto tick = [&](kin::VelocityCommand cmd, int k, const char* tag) {
        io.read(state);
        kin::VectorXd qDes = controller.update(state, cmd);
        lastQDes = qDes;
        for (int i = 0; i < qDes.size(); ++i)
            if (std::isnan(qDes(i)) || std::isinf(qDes(i))) { ok = false; }
        io.writeJointTargets(qDes);

        if (collect) {
            const auto& d = controller.debug();
            Eigen::Vector2d cm = io.measuredCom();
            Eigen::Vector2d zm;
            const bool zv = io.measuredZmp(zm);
            if (zv) {
                M.zmpEx.add(zm.x() - d.zmpRef.x());
                M.zmpEy.add(zm.y() - d.zmpRef.y());
                M.zmpLx.add(zm.x() - d.zmpFromCom.x());
                M.zmpLy.add(zm.y() - d.zmpFromCom.y());
            }
            M.comEx.add(cm.x() - d.comRef.x());
            M.comEy.add(cm.y() - d.comRef.y());
            M.comEint.add(d.errCom);
            {   // 진짜 world 골반 자세(계획 프레임이 아님) = 측정 base quat.
                kin::Quaterniond qp(state.q(3), state.q(4), state.q(5), state.q(6));
                qp.normalize();
                const kin::Vector3d rpy = kin::quatToRpy(qp);
                M.pelRoll.add(rpy(0) * 57.2957795);
                M.pelPitch.add(rpy(1) * 57.2957795);
            }
            if (d.phase == 2) {           // SS
                M.slipSS.add(d.slipLin);
                // ZMP 지지다각형 여유는 "정말로 한 발만 접지" 인 tick 에서만 의미가 있다.
                //   계획상 SS 라도 스윙발이 아직 하중을 받고 있으면 ZMP 가 두 발 사이에
                //   있을 수 있으므로(물리적으로 정상) 그 tick 은 제외한다.
                const double fzSwing = (d.supportSide == 0) ? state.ftRight(2) : state.ftLeft(2);
                if (zv && std::fabs(fzSwing) < 30.0) {
                    M.zmpFootX = std::max(M.zmpFootX, std::fabs(zm.x() - d.footstep.x()));
                    M.zmpFootY = std::max(M.zmpFootY, std::fabs(zm.y() - d.footstep.y()));
                    ++M.ssTrue;
                }
                ++M.ssPlan;
            } else {                      // DS / stand
                M.slipDS.add(d.slipLin);
            }
            M.slipAngMax = std::max(M.slipAngMax, d.slipAng);
            M.dsDevMax   = std::max(M.dsDevMax, d.dsFootDev);
            M.ikResMax   = std::max(M.ikResMax, d.ikResidual);
            M.ikComMax   = std::max(M.ikComMax, d.ikComDiff);
            M.ikSwMax    = std::max(M.ikSwMax, d.ikSwingDiff);
            M.ikConMax   = std::max(M.ikConMax, d.ikContactDiff);
            M.qStep.add(d.qStep);
            M.sigMinMin  = std::min(M.sigMinMin, d.sigComMin);
            M.sigMinSum += d.sigComMin;
            M.sigRawMinMin = std::min(M.sigRawMinMin, d.sigComRawMin);
            M.sigRawMaxMax = std::max(M.sigRawMaxMax, d.sigComRawMax);
            M.sigPelMinMin = std::min(M.sigPelMinMin, d.sigPelMin);
            M.sigPelSum   += d.sigPelMin;
            if (d.phase == 2) { M.sigComSsSum += d.sigComMin; M.sigPelSsSum += d.sigPelMin; ++M.sigSsN; }
            else              { M.sigComDsSum += d.sigComMin; M.sigPelDsSum += d.sigPelMin; ++M.sigDsN; }
            ++M.sigN;
        }
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
    // sag(추종오차) 계측: 준비자세 유지 중 max |qDes - q_meas| (부하로 인한 처짐).
    {
        double sag = 0.0; int jmax = -1;
        for (int i = 0; i < lastQDes.size(); ++i) {
            double e = std::fabs(lastQDes(i) - state.q(kin::kBaseQ + i));
            if (e > sag) { sag = e; jmax = i; }
        }
        std::printf(">> after prepare: base_z=%.3f mode=%s | SAG max|qDes-q|=%.4f rad (%.2f deg) @j%d\n",
                    state.q(2),
                    controller.mode() == kin::HumanoidController::Mode::Active ? "ACTIVE" : "NOT-ACTIVE",
                    sag, sag * 57.2958, jmax);
    }

    // --- Space 로 보행 시작(엣지 한 번) + 속도 유지 ---
    double vx = getenv("KIN_VX") ? atof(getenv("KIN_VX")) : 0.04;
    double vy = getenv("KIN_VY") ? atof(getenv("KIN_VY")) : 0.0;
    double vyaw = getenv("KIN_VYAW") ? atof(getenv("KIN_VYAW")) : 0.0;
    double bx_walk0 = state.q(0);
    collect = true;
    for (int k = 0; k < nWalk && io.running(); ++k) {
        kin::VelocityCommand c; c.vx = vx; c.vy = vy; c.vyaw = vyaw;
        if (k == 0) c.spaceEdge = true;   // 보행 시작 토글
        tick(c, k, "WALK");
    }
    collect = false;
    io.read(state);
    std::printf(">> after walk: base=(%.3f,%.3f,%.3f) dx=%.3f, fell=%s, NaN=%s\n",
                state.q(0), state.q(1), state.q(2), state.q(0) - bx_walk0,
                state.q(2) < 0.6 ? "YES" : "no", ok ? "no" : "YES");

    // ── 계측 요약 ──────────────────────────────────────────────────────────
    //  전부 WALK 구간 집계. 접두어 M| 로 grep/diff 하기 쉽게.
    {
        const bool fell = state.q(2) < 0.6;
        std::printf("M| gait     dx=%+.4f fell=%d nan=%d walksec=%.1f\n",
                    state.q(0) - bx_walk0, fell ? 1 : 0, ok ? 0 : 1, nWalk * dt);
        std::printf("M| zmpErr   rms=(%.5f,%.5f) max=(%.5f,%.5f)   [측정ZMP − 계획ZMP, m]\n",
                    M.zmpEx.rms(), M.zmpEy.rms(), M.zmpEx.mx, M.zmpEy.mx);
        std::printf("M| comErr   rms=(%.5f,%.5f) max=(%.5f,%.5f)   [측정COM − previewCOM, m]\n",
                    M.comEx.rms(), M.comEy.rms(), M.comEx.mx, M.comEy.mx);
        std::printf("M| comInt   rms=%.5f max=%.5f                 [제어기 내부 COM 오차, m]\n",
                    M.comEint.rms(), M.comEint.mx);
        std::printf("M| pelvis   rms=(%.4f,%.4f) max=(%.4f,%.4f)   [진짜 world roll,pitch, deg]\n",
                    M.pelRoll.rms(), M.pelPitch.rms(), M.pelRoll.mx, M.pelPitch.mx);
        std::printf("M| slip     SS rms=%.3e max=%.3e | DS rms=%.3e max=%.3e | ang max=%.3e"
                    "   [접지발 지령속도, m/s]\n",
                    M.slipSS.rms(), M.slipSS.mx, M.slipDS.rms(), M.slipDS.mx, M.slipAngMax);
        std::printf("M| contact  dsFootDev max=%.5f m | sigComRaw(구속전) min=%.4f max=%.4f\n",
                    M.dsDevMax, M.sigRawMinMin, M.sigRawMaxMax);
        // 제어 권한(Ji·N 최소특이값). 결정론적 지표 — 정식화 비교의 본선.
        std::printf("M| author   sigComMin  SS=%.4f DS=%.4f all(min=%.4f mean=%.4f)\n",
                    M.sigSsN ? M.sigComSsSum / (double)M.sigSsN : 0.0,
                    M.sigDsN ? M.sigComDsSum / (double)M.sigDsN : 0.0,
                    M.sigMinMin, M.sigN ? M.sigMinSum / (double)M.sigN : 0.0);
        std::printf("M| author   sigPelMin  SS=%.4f DS=%.4f all(min=%.4f mean=%.4f)\n",
                    M.sigSsN ? M.sigPelSsSum / (double)M.sigSsN : 0.0,
                    M.sigDsN ? M.sigPelDsSum / (double)M.sigDsN : 0.0,
                    M.sigPelMinMin, M.sigN ? M.sigPelSum / (double)M.sigN : 0.0);
        std::printf("M| ik       dqRel max=%.3e | taskVelDiff com=%.3e swing=%.3e contact=%.3e m/s"
                    "   [ikCompare=1 일 때만]\n",
                    M.ikResMax, M.ikComMax, M.ikSwMax, M.ikConMax);
        std::printf("M| zmpLipm  rms=(%.5f,%.5f) max=(%.5f,%.5f)   [측정ZMP − LIPM함의ZMP, m]\n",
                    M.zmpLx.rms(), M.zmpLy.rms(), M.zmpLx.mx, M.zmpLy.mx);
        std::printf("M| margin   zmpVsSupFoot max=(%.4f,%.4f) m (발 반치수 x=0.150 y=0.065)"
                    " | trueSS=%ld/%ld tick\n",
                    M.zmpFootX, M.zmpFootY, M.ssTrue, M.ssPlan);
        std::printf("M| smooth   qStep rms=%.5f max=%.5f rad\n", M.qStep.rms(), M.qStep.mx);
    }

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
