#include "model/RbdlModel.h"

#include <cstdio>

// [스켈레톤] RBDL 백엔드. 현재는 컴파일만 되는 틀이며 실제 동작은 MujocoModel 을 쓴다.
// 각 메서드의 TODO 위치에 RBDL API 호출을 채워 넣으면 완성된다.
namespace kin {

RbdlModel::~RbdlModel() {
    // TODO: delete model_;  (RBDL Model 소유 시)
}

bool RbdlModel::load(const std::string& /*path*/) {
    std::fprintf(stderr,
        "[RbdlModel] not implemented yet — use MujocoModel for now.\n"
        "            (RBDL URDF 로드/FK/Jacobian/COM-Jacobian 구현 예정)\n");
    // TODO: RigidBodyDynamics::Addons::URDFReadFromFile(path, model_, true);
    //       nq_/nv_/nJoints_ 설정, q_/dq_ 크기 초기화.
    return false;
}

void RbdlModel::setState(const VectorXd& q, const VectorXd& dq) {
    q_ = q; dq_ = dq;
    // TODO: RBDL q(pos3+quat) / v(lin3+ang3) 규약으로 매핑 저장.
}

void RbdlModel::updateKinematics() {
    // TODO: RigidBodyDynamics::UpdateKinematics(*model_, q_, dq_, ddq_zero);
    //       + COM/augmented-body 갱신(CRBDL::_UpdateCOM 참고).
}

int RbdlModel::bodyId(const std::string& /*name*/) const {
    // TODO: return model_->GetBodyId(name.c_str());
    return -1;
}

Matrix3d RbdlModel::bodyRot(int /*bodyId*/) const {
    // TODO: model_->X_base[id].E (transpose 주의).
    return Matrix3d::Identity();
}

Vector3d RbdlModel::bodyPos(int /*bodyId*/, const Vector3d& /*localOffset*/) const {
    // TODO: CalcBodyToBaseCoordinates(*model_, q_, id, localOffset, false).
    return Vector3d::Zero();
}

MatrixXd RbdlModel::bodyJacobian(int /*bodyId*/, const Vector3d& /*localOffset*/) const {
    // TODO: CalcPointJacobian6D(...); 열/행 순서를 본 인터페이스 규약에 맞게 재배열.
    return MatrixXd::Zero(6, nv_ > 0 ? nv_ : kNv);
}

Vector3d RbdlModel::com() const {
    // TODO: Utils::CalcCenterOfMass(...).
    return Vector3d::Zero();
}

double RbdlModel::mass() const {
    return 0.0;   // TODO
}

MatrixXd RbdlModel::comJacobian() const {
    // TODO: CRBDL::_ComputeCOMJacobian (augmented-body 재귀) 이식.
    return MatrixXd::Zero(3, nv_ > 0 ? nv_ : kNv);
}

}  // namespace kin
