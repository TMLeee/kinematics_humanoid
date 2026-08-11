#pragma once
// RobotModel 의 RBDL 백엔드 — [스켈레톤/틀만].
//
// 요구사항 7: 최종 목표는 RBDL 사용이나, 지금은 MujocoModel 을 쓴다.
//   이 클래스는 "RBDL 로 교체 가능"함을 보장하기 위한 인터페이스 틀이며,
//   실제 RBDL 구현은 tocabi URDF 가 준비되면 채운다(아래 각 메서드의 TODO 참조).
//
// 구현 시 참고(기존 RBDL_test1/CRBDL.cpp):
//   - 순운동학:      RigidBodyDynamics::UpdateKinematics
//   - 바디 6D Jaco:  RigidBodyDynamics::CalcPointJacobian6D
//   - COM:           RigidBodyDynamics::Utils::CalcCenterOfMass
//   - COM Jacobian:  CRBDL::_ComputeCOMJacobian (augmented-body 재귀) 이식
//   - 좌표 규약:     본 인터페이스의 nv 열 순서 [base_lin, base_ang, joints] 에 맞게
//                    RBDL floating-base(q: pos3+quat, v: lin3+ang3) 를 매핑.
#include "model/RobotModel.h"

namespace RigidBodyDynamics { class Model; }

namespace kin {

class RbdlModel : public RobotModel {
public:
    RbdlModel() = default;
    ~RbdlModel() override;

    // URDF 경로 로드. [미구현] tocabi URDF 준비 후 RBDL URDFReadFromFile 로 채운다.
    bool load(const std::string& path) override;

    int nq() const override { return nq_; }
    int nv() const override { return nv_; }
    int nJoints() const override { return nJoints_; }

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
    RigidBodyDynamics::Model* model_ = nullptr;   // RBDL 모델(로드 후 소유)
    int nq_ = 0, nv_ = 0, nJoints_ = 0;
    VectorXd q_, dq_;
};

}  // namespace kin
