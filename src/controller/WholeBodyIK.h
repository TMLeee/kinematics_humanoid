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
//
// 두 가지 정식화(config: floatingBase)를 모두 지원한다. solve() 는 변수 차원에 무관하다.
//   (a) reduced  변수 = 관절 nJoints(33). 지지발 구속을 baseSlave()/reduce() 로 **소거**.
//   (b) floating 변수 = nv(39, 가상 base 6 포함). 지지발 구속을 **최상위 task** 로 넣고
//       task 자코비안은 축약 없이 nv 열을 그대로 쓴다. 해에서 관절 성분만 로봇에 보낸다.
//   단일지지에서 (a)≡(b): J_c q̇=0 을 [Jb|Jj] 로 나누면 Jb 가 6×6 가역이라
//   q̇_b = −Jb⁻¹Jj q̇_j 로 소거되고, 그것을 대입한 것이 곧 reduce() 다.
//   (b) 만이 **양발지지에서 두 발을 동시에** 구속할 수 있다(구속 12행 > base 6 → 소거 불가).
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

    // 각 task 가 "상위 task 의 null space 안에서" 실제로 쓸 수 있는 방향의 세기.
    //   Ji·N 의 특이값. sigMin 이 작을수록 그 task 는 그 방향으로 무력하다
    //   (= 상위 task 와 접촉 구속이 그 자유도를 이미 다 써 버렸다는 뜻).
    //   카오스에 흔들리지 않는 결정론적 지표라 정식화 비교에 적합하다.
    struct TaskDiag {
        double sigMin = 0.0, sigMax = 0.0;
        int rows = 0;
    };

    // 우선순위 DLS 풀이. tasks 는 우선순위 높은 순서. dq(nJoints) 반환.
    //   diag != nullptr 이면 task 별 Ji·N 특이값을 기록한다(SVD 비용 발생).
    static VectorXd solve(const std::vector<Task>& tasks, int nJoints,
                          std::vector<TaskDiag>* diag = nullptr);
};

}  // namespace kin
