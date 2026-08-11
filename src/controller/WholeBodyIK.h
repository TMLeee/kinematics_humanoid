#pragma once
// Whole-Body Inverse Kinematics — 나카무라(Nakamura) 우선순위 DLS.
//
// 요구사항 3/4:
//   우선순위: 고정발 > COM > 스윙발 > 손 > 골반 자세.
//   - "고정발(지지발)"은 지면에 고정되어 있으므로, 최상위 우선순위를 "제약"으로
//     정확히 반영한다: 지지발 속도 0 이 되도록 베이스 속도를 관절속도에 종속시킨다
//       v_base = -Jb^{-1} Jj v_joint
//     (support-consistent reduction). 이렇게 하면 모든 하위 task Jacobian 이
//     "관절속도(nJoints)"만의 함수 Ĵ 로 축약되고, 지지발은 자동으로 고정된다.
//   - 나머지(COM, 스윙발, 손, 골반)는 관절공간에서 successive null-space projection
//     (감쇠 최소자승, DLS)으로 우선순위대로 푼다.
//   - 발이 바뀌면 지지발/스윙발 Jacobian 만 교체하면 되어 우선순위가 자동 조정된다.
#include <vector>
#include <Eigen/Dense>

#include "util/MathUtil.h"
#include "util/RobotDefs.h"

namespace kin {

class WholeBodyIK {
public:
    struct Task {
        MatrixXd J;        // 관절공간 task Jacobian (m × nJoints), support-consistent
        VectorXd xdot;     // 목표 task 속도 (= kp * 오차), 크기 m
        double lambda;     // DLS 감쇠
        const char* name = "";
    };

    // 지지발 6×nv Jacobian 으로 베이스 종속 사상 S(6×nJoints)를 만든다.
    //   v_base = S v_joint  (지지발 속도 0 제약).
    static MatrixXd baseSlave(const MatrixXd& Jsupport, int nJoints, double lambda = 1e-6);

    // task 6/3×nv Jacobian 을 support-consistent 관절공간 Jacobian(m×nJoints)으로 축약.
    //   Ĵ = J_joint + J_base * S.
    static MatrixXd reduce(const MatrixXd& Jtask, const MatrixXd& S);

    // 우선순위 DLS 풀이. tasks 는 우선순위 높은 순서. dq(nJoints) 반환.
    static VectorXd solve(const std::vector<Task>& tasks, int nJoints);
};

}  // namespace kin
