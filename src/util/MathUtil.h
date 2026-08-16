#pragma once
// 수학 유틸리티: 회전/쿼터니언 변환, 보간(cubic/cycloid), DARE(이산 리카티),
// DLS(damped least squares) 의사역행렬 등 제어에 필요한 소도구 모음.
//
// 참고: 기존 RBDL_test1 프로젝트의 Convertor.h / math_util.h 를 이식하되,
//       원본의 GetRotationMatrixX/Y/Z 가 모두 단위행렬을 반환하던 버그를 바로잡았다.
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace kin {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Quaterniond;
using Vector6d = Eigen::Matrix<double, 6, 1>;

// ---------------------------------------------------------------------------
// 회전 표현 변환
// ---------------------------------------------------------------------------

inline Matrix3d rotX(double a) {
    Matrix3d R;
    R << 1, 0, 0,
         0, std::cos(a), -std::sin(a),
         0, std::sin(a),  std::cos(a);
    return R;
}
inline Matrix3d rotY(double a) {
    Matrix3d R;
    R <<  std::cos(a), 0, std::sin(a),
          0, 1, 0,
         -std::sin(a), 0, std::cos(a);
    return R;
}
inline Matrix3d rotZ(double a) {
    Matrix3d R;
    R << std::cos(a), -std::sin(a), 0,
         std::sin(a),  std::cos(a), 0,
         0, 0, 1;
    return R;
}

// roll-pitch-yaw (X-Y-Z, world 고정축 기준) -> 회전행렬. R = Rz(yaw) Ry(pitch) Rx(roll).
inline Matrix3d rpyToRot(double roll, double pitch, double yaw) {
    return rotZ(yaw) * rotY(pitch) * rotX(roll);
}
inline Matrix3d rpyToRot(const Vector3d& rpy) {
    return rpyToRot(rpy(0), rpy(1), rpy(2));
}

// 회전행렬 -> roll-pitch-yaw.
inline Vector3d rotToRpy(const Matrix3d& R) {
    Vector3d rpy;
    rpy(1) = std::atan2(-R(2, 0), std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0)));
    rpy(0) = std::atan2(R(2, 1), R(2, 2));
    rpy(2) = std::atan2(R(1, 0), R(0, 0));
    return rpy;
}

inline Quaterniond rpyToQuat(const Vector3d& rpy) {
    return Quaterniond(rpyToRot(rpy));
}

inline Vector3d quatToRpy(const Quaterniond& q) {
    return rotToRpy(q.toRotationMatrix());
}

// ---------------------------------------------------------------------------
// 각도(orientation) 오차 -> world 프레임 각속도 오차 (small-angle 근사)
//   R_des 를 향하도록 R_cur 를 회전시키는 축*각 벡터. MuJoCo 의 angular
//   Jacobian(jacr: qvel -> world angular velocity) 과 프레임이 일치한다.
// ---------------------------------------------------------------------------
inline Vector3d orientationError(const Matrix3d& R_des, const Matrix3d& R_cur) {
    Matrix3d Re = R_des * R_cur.transpose();
    Quaterniond qe(Re);
    if (qe.w() < 0) qe.coeffs() *= -1.0;   // 최단 회전
    Vector3d v(qe.x(), qe.y(), qe.z());
    double n = v.norm();
    if (n < 1e-9) return Vector3d::Zero();
    double angle = 2.0 * std::atan2(n, qe.w());
    return v / n * angle;
}
inline Vector3d orientationError(const Quaterniond& q_des, const Quaterniond& q_cur) {
    return orientationError(q_des.toRotationMatrix(), q_cur.toRotationMatrix());
}

// skew-symmetric 행렬.
inline Matrix3d skew(const Vector3d& v) {
    Matrix3d S;
    S <<     0, -v(2),  v(1),
          v(2),     0, -v(0),
         -v(1),  v(0),     0;
    return S;
}

// ---------------------------------------------------------------------------
// DLS(damped least squares) 의사역행렬:  J^+ = J^T (J J^T + λ² I)^{-1}
//   특이점 근처에서도 수치적으로 안정. λ 는 감쇠 계수.
// ---------------------------------------------------------------------------
inline MatrixXd dampedPinv(const MatrixXd& J, double lambda) {
    const int m = static_cast<int>(J.rows());
    MatrixXd JJt = J * J.transpose() + lambda * lambda * MatrixXd::Identity(m, m);
    return J.transpose() * JJt.inverse();
}

// ---------------------------------------------------------------------------
// 접촉 wrench → 지면(z = z0) 위 CoP(= 측정 ZMP)
// ---------------------------------------------------------------------------
//  f  : 지면반력 합력 (world 축)
//  Mo : 그 반력의 world **원점** 기준 모멘트 합 (= Σ (m_i + p_i × f_i))
//  z0 : CoP 를 구할 평면의 높이(보통 지면 0)
//
//  ZMP 정의: M_O = p_cop × f (수평성분).  p_cop = (x, y, z0) 로 두면
//      M_O_x = y·f_z − z0·f_y ,   M_O_y = z0·f_x − x·f_z
//  이므로 아래 두 식이 나온다.
//
//  부호 규약 무관: F/T 센서가 "지면→발"이 아니라 그 반작용을 내더라도
//  (f, Mo) 가 함께 −1 배 되어 분자·분모가 같이 뒤집히므로 결과는 동일하다.
//  (MuJoCo 의 force/torque 센서는 본 모델에서 fz<0 으로 나온다.)
inline bool copOnPlane(const Vector3d& f, const Vector3d& Mo,
                       double z0, double fzMin, Eigen::Vector2d& cop) {
    if (std::fabs(f.z()) < fzMin) return false;      // 하중 없음(공중)
    cop.x() = (z0 * f.x() - Mo.y()) / f.z();
    cop.y() = (Mo.x() + z0 * f.y()) / f.z();
    return true;
}

// 한 발 F/T 센서 측정치를 world 원점 기준 wrench 로 누적한다.
//   ft : (fx,fy,fz, mx,my,mz) — **센서 site 프레임**, 센서 원점 기준 모멘트
//   R, p : 센서 site 의 world 회전/위치
inline void accumulateWrench(const Vector6d& ft, const Matrix3d& R, const Vector3d& p,
                             Vector3d& fSum, Vector3d& MoSum) {
    const Vector3d fw = R * ft.head(3);
    const Vector3d mw = R * ft.tail(3);
    fSum  += fw;
    MoSum += mw + p.cross(fw);       // 센서 원점 기준 → world 원점 기준으로 이동
}

// ---------------------------------------------------------------------------
// 보간
// ---------------------------------------------------------------------------

// 3차 다항 보간 위치 (경계 속도 0). t in [t0, tf].
inline double cubic(double t, double t0, double tf, double x0, double xf) {
    if (t <= t0) return x0;
    if (t >= tf) return xf;
    double s = (t - t0) / (tf - t0);
    double h = 3 * s * s - 2 * s * s * s;   // Hermite (0->1, 경계 속도 0)
    return x0 + (xf - x0) * h;
}

// cycloid 수평 이동: 부드러운 가감속(경계 속도/가속 0에 가까움).
inline double cycloidXY(double t, double t0, double tf, double x0, double xf) {
    if (t <= t0) return x0;
    if (t >= tf) return xf;
    double theta = 2.0 * M_PI * (t - t0) / (tf - t0);
    return x0 + (xf - x0) / (2.0 * M_PI) * (theta - std::sin(theta));
}

// cycloid 수직(스윙발 높이): t0/tf 에서 0, 중간에서 최고점 h.
inline double swingHeight(double t, double t0, double tf, double h) {
    if (t <= t0 || t >= tf) return 0.0;
    double theta = 2.0 * M_PI * (t - t0) / (tf - t0);
    return h * 0.5 * (1.0 - std::cos(theta));
}

// ---------------------------------------------------------------------------
// 이산 대수 리카티 방정식(DARE) 풀이 — 심플렉틱 행렬 고윳값 분해 방식.
//   preview controller 의 게인 계산에 사용. (기존 math_util.h 이식/정리)
//   A: n×n, B: n×m, Q: n×n, R: m×m  ->  P (n×n)
// ---------------------------------------------------------------------------
inline MatrixXd dare(const MatrixXd& A, const MatrixXd& B,
                     const MatrixXd& Q, const MatrixXd& R) {
    const int n = static_cast<int>(A.rows());

    MatrixXd Ainv = A.inverse();
    MatrixXd Rinv = R.inverse();
    MatrixXd BRB = B * Rinv * B.transpose();

    // 심플렉틱 행렬 Z (2n×2n)
    MatrixXd Z(2 * n, 2 * n);
    Z.topLeftCorner(n, n)     = Ainv;
    Z.topRightCorner(n, n)    = Ainv * BRB;
    Z.bottomLeftCorner(n, n)  = Q * Ainv;
    Z.bottomRightCorner(n, n) = A.transpose() + Q * Ainv * BRB;

    Eigen::EigenSolver<MatrixXd> es(Z);
    Eigen::MatrixXcd evec = es.eigenvectors();
    Eigen::VectorXcd eval = es.eigenvalues();

    // 단위원 밖(|λ|>1)의 고유벡터를 모은다.
    Eigen::MatrixXcd U(2 * n, n);
    int c = 0;
    for (int i = 0; i < 2 * n && c < n; ++i) {
        if (std::norm(eval(i)) > 1.0) {
            U.col(c++) = evec.col(i);
        }
    }

    Eigen::MatrixXcd U11 = U.topRows(n);
    Eigen::MatrixXcd U21 = U.bottomRows(n);
    Eigen::MatrixXcd X = U21 * U11.inverse();
    return X.real();
}

}  // namespace kin
