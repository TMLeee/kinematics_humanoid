#pragma once
// RobotModel 의 MuJoCo 백엔드 구현.
//
// 요구사항 7: "우선은 MuJoCo 로부터 모델 키네마틱/다이나믹 정보를 받아 사용".
//   시뮬레이터(물리 스텝을 도는 MujocoEnv 의 mjData)와는 별개로, 이 클래스는
//   자체 mjModel/mjData 를 로드해 "해석용"으로만 쓴다. 따라서 임의의 q 를 넣어
//   순운동학/Jacobian/COM 을 계산할 수 있으며 물리 시뮬레이션에 영향을 주지 않는다.
#include <mujoco/mujoco.h>

#include "model/RobotModel.h"

namespace kin {

class MujocoModel : public RobotModel {
public:
    MujocoModel() = default;
    ~MujocoModel() override;

    bool load(const std::string& path) override;

    int nq() const override { return m_ ? m_->nq : 0; }
    int nv() const override { return m_ ? m_->nv : 0; }
    int nJoints() const override { return m_ ? m_->nu : 0; }

    void setState(const VectorXd& q, const VectorXd& dq) override;
    void updateKinematics() override;

    int bodyId(const std::string& name) const override;

    Vector3d bodyPos(int bodyId, const Vector3d& localOffset = Vector3d::Zero()) const override;
    Matrix3d bodyRot(int bodyId) const override;
    MatrixXd bodyJacobian(int bodyId,
                          const Vector3d& localOffset = Vector3d::Zero()) const override;

    Vector3d com() const override;
    double mass() const override;
    MatrixXd comJacobian() const override;

private:
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;   // 해석 전용 데이터(물리 스텝 안 함)
    int base_body_ = 1;      // 전신 subtree 루트(base_link)
};

}  // namespace kin
