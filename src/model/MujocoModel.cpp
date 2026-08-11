#include "model/MujocoModel.h"

#include <cstdio>

namespace kin {

MujocoModel::~MujocoModel() {
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
}

bool MujocoModel::load(const std::string& path) {
    char error[1000] = "";
    m_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!m_) {
        std::fprintf(stderr, "[MujocoModel] load failed: %s\n", error);
        return false;
    }
    d_ = mj_makeData(m_);
    if (!d_) return false;

    base_body_ = mj_name2id(m_, mjOBJ_BODY, BodyNames::Pelvis);
    if (base_body_ < 0) base_body_ = 1;

    // 초기 자세: 키프레임 0(서기)로 맞춰 놓는다.
    if (m_->nkey > 0) mj_resetDataKeyframe(m_, d_, 0);
    updateKinematics();
    return true;
}

void MujocoModel::setState(const VectorXd& q, const VectorXd& dq) {
    if (!m_) return;
    const int nq = m_->nq, nv = m_->nv;
    for (int i = 0; i < nq && i < q.size(); ++i) d_->qpos[i] = q(i);
    for (int i = 0; i < nv && i < dq.size(); ++i) d_->qvel[i] = dq(i);
}

void MujocoModel::updateKinematics() {
    if (!m_) return;
    // Jacobian(mj_jac / mj_jacSubtreeCom)은 xpos(kinematics)와 cdof/subtree_com(comPos)을
    // 필요로 한다. 물리 스텝/충돌 없이 위치 파생량만 갱신한다.
    mj_kinematics(m_, d_);
    mj_comPos(m_, d_);
}

int MujocoModel::bodyId(const std::string& name) const {
    return m_ ? mj_name2id(m_, mjOBJ_BODY, name.c_str()) : -1;
}

Matrix3d MujocoModel::bodyRot(int bodyId) const {
    Matrix3d R;
    const mjtNum* x = d_->xmat + 9 * bodyId;   // row-major
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R(i, j) = x[3 * i + j];
    return R;
}

Vector3d MujocoModel::bodyPos(int bodyId, const Vector3d& localOffset) const {
    Vector3d p(d_->xpos[3 * bodyId], d_->xpos[3 * bodyId + 1], d_->xpos[3 * bodyId + 2]);
    if (!localOffset.isZero())
        p += bodyRot(bodyId) * localOffset;
    return p;
}

MatrixXd MujocoModel::bodyJacobian(int bodyId, const Vector3d& localOffset) const {
    const int nv = m_->nv;
    Vector3d point = bodyPos(bodyId, localOffset);

    // MuJoCo 는 3×nv 를 row-major flat 으로 채운다.
    std::vector<mjtNum> jp(3 * nv), jr(3 * nv);
    mjtNum pt[3] = {point(0), point(1), point(2)};
    mj_jac(m_, d_, jp.data(), jr.data(), pt, bodyId);

    MatrixXd J(6, nv);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < nv; ++c) {
            J(r, c)     = jp[r * nv + c];   // linear
            J(r + 3, c) = jr[r * nv + c];   // angular
        }
    return J;
}

Vector3d MujocoModel::com() const {
    const mjtNum* c = d_->subtree_com + 3 * base_body_;
    return Vector3d(c[0], c[1], c[2]);
}

double MujocoModel::mass() const {
    return m_ ? m_->body_subtreemass[base_body_] : 0.0;
}

MatrixXd MujocoModel::comJacobian() const {
    const int nv = m_->nv;
    std::vector<mjtNum> jc(3 * nv);
    mj_jacSubtreeCom(m_, d_, jc.data(), base_body_);
    MatrixXd J(3, nv);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < nv; ++c)
            J(r, c) = jc[r * nv + c];
    return J;
}

}  // namespace kin
