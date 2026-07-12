// kin_humanoid — MuJoCo 기반 휴머노이드(DYROS Tocabi) 키네마틱스 제어 엔트리포인트.
//
// 현재는 스켈레톤이다:
//   1) tocabi MuJoCo 모델을 로드하고 뷰어를 띄운다.
//   2) Eigen / RBDL 링크를 소규모 자기진단으로 확인한다.
//   3) 시뮬레이션 루프를 돌며 렌더링한다.
// 실제 키네마틱스 기반 whole-body 제어 로직은 이후 src/kinematics, src/controller
// 에서 확장한다(제어 라이브러리는 별도로 가져와 모디파이).
#include "simulator/MujocoEnv.h"

#include <rbdl/rbdl.h>

#include <cstdio>
#include <string>

namespace {
constexpr const char* kDefaultModel =
    "model/dyros_tocabi_v2/tocabi_description/mujoco_model/dyros_tocabi.xml";

// Eigen + RBDL 링크 및 동작을 확인하는 소규모 자기진단.
// 3-링크 회전 체인을 만들고 순운동학(CoM)을 한 번 계산한다.
void rbdlSelfTest() {
    namespace R = RigidBodyDynamics;
    namespace M = RigidBodyDynamics::Math;

    R::Model model;
    model.gravity = M::Vector3d(0.0, 0.0, -9.81);

    R::Body   body(1.0, M::Vector3d(0.0, 0.0, 0.5), M::Vector3d(0.1, 0.1, 0.1));
    R::Joint  joint(R::JointTypeRevolute, M::Vector3d(0.0, 1.0, 0.0));

    unsigned int id = 0;
    for (int i = 0; i < 3; ++i)
        id = model.AddBody(id, M::Xtrans(M::Vector3d(0.0, 0.0, 1.0)), joint, body);

    M::VectorNd q = M::VectorNd::Zero(model.dof_count);
    double mass = 0.0;
    M::Vector3d com = M::Vector3d::Zero();
    R::Utils::CalcCenterOfMass(model, q, q, nullptr, mass, com);

    std::printf("[rbdl] self-test OK (dof=%u, mass=%.3f, com_z=%.3f)\n",
                model.dof_count, mass, com[2]);
}
}  // namespace

int main(int argc, char** argv) {
    const std::string model_path = (argc > 1) ? argv[1] : kDefaultModel;
    std::printf("[kin_humanoid] loading model: %s\n", model_path.c_str());

    MujocoEnv env;
    if (!env.load(model_path)) {
        std::fprintf(stderr, "[kin_humanoid] failed to load model\n");
        return 1;
    }
    mjModel* m = env.model();
    std::printf("[kin_humanoid] model loaded: nq=%d nv=%d nu=%d\n",
                m->nq, m->nv, m->nu);

    // Eigen / RBDL 링크 검증.
    rbdlSelfTest();

    if (!env.initViewer("kin_humanoid — DYROS Tocabi")) {
        std::fprintf(stderr, "[kin_humanoid] viewer init failed (headless?)\n");
        return 1;
    }

    std::printf("[kin_humanoid] entering sim loop (close window to quit)\n");
    while (!env.viewerShouldClose()) {
        // TODO: 여기서 키네마틱스 기반 whole-body 제어를 매 제어주기마다 호출하고
        //       env.data()->ctrl 에 명령을 기록한다.
        env.step();
        env.render();
    }
    return 0;
}
