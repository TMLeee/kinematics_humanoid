// kin_humanoid — MuJoCo 기반 휴머노이드(DYROS Tocabi) 키네마틱 보행 제어 엔트리포인트.
//
// 파이프라인(요구사항 2):
//   FootstepGenerator → PreviewController(실시간 1초 윈도우) → WholeBodyIK(우선순위 DLS)
//   → 관절 목표 → SimIO(위치서보) → MuJoCo.
//
// 구성(요구사항 7,8):
//   - 모델 정보: RobotModel(추상) ← MujocoModel(현재) / RbdlModel(틀만)
//   - 로봇 I/O : RobotIO(추상)   ← SimIO(현재)      / RealIO(틀만)
//
// 조작(요구사항 6): [Space] 보행 on/off, W/S 전후, A/D 좌우 게걸음, Q/E 회전, X 정지.
#include "io/SimIO.h"
#include "model/MujocoModel.h"
#include "controller/HumanoidController.h"

#include <cstdio>
#include <string>

namespace {
constexpr const char* kDefaultModel =
    "model/dyros_tocabi_v2/tocabi_description/mujoco_model/dyros_tocabi.xml";
}

int main(int argc, char** argv) {
    const std::string model_path = (argc > 1) ? argv[1] : kDefaultModel;
    std::printf("[kin_humanoid] model: %s\n", model_path.c_str());

    // --- 출력(시뮬레이터) I/O ---
    kin::SimIO io(model_path, /*with_viewer=*/true);
    if (!io.init()) {
        std::fprintf(stderr, "[kin_humanoid] SimIO init failed\n");
        return 1;
    }

    // --- 모델 정보(현재는 MuJoCo 백엔드) ---
    kin::MujocoModel model;
    if (!model.load(model_path)) {
        std::fprintf(stderr, "[kin_humanoid] model load failed\n");
        return 1;
    }
    std::printf("[kin_humanoid] model: nq=%d nv=%d nJoints=%d mass=%.2f kg\n",
                model.nq(), model.nv(), model.nJoints(), model.mass());

    // --- 초기 상태 읽기 ---
    kin::RobotState state;
    io.read(state);

    // --- 제어기 초기화 ---
    kin::HumanoidController controller;
    controller.init(&model, io.controlDt(), state);

    // 관절 한계(위치서보 안정용) 설정 — 액추에이터→관절 매핑에서 가져온다.
    {
        mjModel* m = io.env().model();
        kin::VectorXd lo(m->nu), hi(m->nu);
        for (int i = 0; i < m->nu; ++i) {
            int jid = m->actuator_trnid[2 * i];       // 액추에이터 i 가 구동하는 관절
            if (jid >= 0 && m->jnt_limited[jid]) {
                lo(i) = m->jnt_range[2 * jid];
                hi(i) = m->jnt_range[2 * jid + 1];
            } else {
                lo(i) = -1e9; hi(i) = 1e9;
            }
        }
        controller.setJointLimits(lo, hi);
    }

    std::printf("[kin_humanoid] ready. Press SPACE to walk, WASD to move.\n");

    // --- 제어 루프 ---
    while (io.running()) {
        io.read(state);
        kin::VelocityCommand cmd = io.velocityCommand();
        kin::VectorXd qDes = controller.update(state, cmd);
        io.setStatus(controller.statusText());
        io.writeJointTargets(qDes);

        // 실시간 그래프 데이터: 목표(제어기) + 측정(시뮬레이터).
        const auto& dbg = controller.debug();
        Eigen::Vector2d zmpMeas;
        bool zmpValid = io.measuredZmp(zmpMeas);
        io.env().walkPlotter().push(io.env().data()->time,
            dbg.footstep, dbg.zmpRef, dbg.comRef, io.measuredCom(), zmpMeas, zmpValid);

        io.step();
        io.render();
    }
    return 0;
}
