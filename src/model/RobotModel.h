#pragma once
// 로봇 모델(키네마틱/다이나믹) 추상 베이스 클래스.
//
// 목적(요구사항 7): 제어기는 "모델 정보"를 이 인터페이스로만 질의한다.
//   현재 백엔드는 MuJoCo(MujocoModel) 로부터 키네마틱/다이나믹 정보를 받아 쓰고,
//   추후 RBDL(RbdlModel) 로 교체 가능하도록 동일 인터페이스를 상속해 구현한다.
//
// 좌표/프레임 규약 (RobotDefs.h 참조):
//   - setState(q, dq): q 는 nq(=40), dq 는 nv(=39) 차원.
//       q  = [ base_pos(3), base_quat(w,x,y,z)(4), joints(33) ]
//       dq = [ base_lin(3), base_ang(3), joints(33) ]
//   - 모든 Jacobian 은 nv(=39) 속도공간, world 프레임 기준.
//       열: [ base_lin(3), base_ang(3), joint(33) ]
//       6D Jacobian 행: [ linear(3); angular(3) ]
//
// 중요: 이 모델은 시뮬레이터(물리 스텝)와 독립적인 "해석용" 상태를 가진다.
//   즉 임의의 q 를 setState 로 넣어 FK/Jacobian 을 계산할 수 있다.
#include <Eigen/Dense>
#include <string>

#include "util/MathUtil.h"
#include "util/RobotDefs.h"

namespace kin {

class RobotModel {
public:
    virtual ~RobotModel() = default;

    // 모델(URDF/MJCF)을 로드한다. 성공 시 true.
    virtual bool load(const std::string& path) = 0;

    // 차원 정보
    virtual int nq() const = 0;
    virtual int nv() const = 0;
    virtual int nJoints() const = 0;

    // 해석용 상태 설정 후 순운동학/COM/Jacobian 준비.
    virtual void setState(const VectorXd& q, const VectorXd& dq) = 0;
    virtual void updateKinematics() = 0;

    // 바디 이름 -> 인덱스(백엔드 내부 id). 없으면 -1.
    virtual int bodyId(const std::string& name) const = 0;

    // 바디의 world 위치/자세. localOffset 은 바디 로컬 프레임 오프셋(m).
    virtual Vector3d bodyPos(int bodyId, const Vector3d& localOffset = Vector3d::Zero()) const = 0;
    virtual Matrix3d bodyRot(int bodyId) const = 0;
    Quaterniond bodyQuat(int bodyId) const { return Quaterniond(bodyRot(bodyId)); }

    // 바디 점(localOffset)에 대한 6×nv world Jacobian. 행 [linear(3); angular(3)].
    virtual MatrixXd bodyJacobian(int bodyId,
                                  const Vector3d& localOffset = Vector3d::Zero()) const = 0;

    // 전신 무게중심(world) 및 전신 질량.
    virtual Vector3d com() const = 0;
    virtual double mass() const = 0;

    // 전신 COM Jacobian (3×nv, world). 요구사항 5: COM Jacobian 기반 제어에 사용.
    virtual MatrixXd comJacobian() const = 0;

    // (선택) 관성행렬 등 다이나믹 질의 — 키네마틱 제어에는 불필요하지만
    //        추후 다이나믹 일관 제어를 위해 인터페이스만 열어둔다. 기본은 미지원.
    virtual bool supportsDynamics() const { return false; }
};

}  // namespace kin
