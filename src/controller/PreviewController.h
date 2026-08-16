#pragma once
// Preview Controller (LIPM ZMP-preview, Kajita 방식) — 실시간 슬라이딩 윈도우 형.
//
// 요구사항 2: 기존 프로젝트는 전체 ZMP 궤적을 미리 넣고 오프라인처럼 계산했으나,
//   여기서는 매 제어주기마다 "앞으로 1초"의 ZMP 레퍼런스 윈도우를 입력받아
//   그에 맞는 COM 을 실시간으로 계산한다.
//   - 게인(Gi, Gx, 미리보기 게인 Gd[·])은 dt·COM높이·가중치에만 의존하므로
//     시작 시 DARE 로 1회만 계산한다.
//   - update() 는 현재 상태 + 최신 1초 ZMP 윈도우로 온라인 점화식 1스텝만 돈다.
//
// 모델: 3중적분기(cart-table). 상태 x=[c, c_dot, c_ddot], 입력 u=jerk,
//       ZMP p = C x,  C=[1, 0, -Zc/g].
#include <vector>
#include <Eigen/Dense>

#include "util/MathUtil.h"

namespace kin {

class PreviewController {
public:
    // dt: 제어주기, comHeight: COM 높이 Zc, previewSec: 미리보기 창 길이(기본 1초).
    //   comMassScale: ZMP 관계식(zmp = com − (zc/g)·c̈om)의 관성항(c̈om 계수) 배율.
    //     LIPM 점질량 ZMP 식에서 질량은 소거되므로, "com 무게 N배"는 관성 반력을 N배로
    //     보는 것과 같다 → C=[1,0,−comMassScale·zc/g]. 1.0=이론값, >1=계산상 더 무겁게.
    void init(double dt, double comHeight, double previewSec = 1.0,
              double g = 9.81, double Qe = 1.0, double R = 1.0e-6,
              double comMassScale = 1.0) {
        dt_ = dt; zc_ = comHeight; g_ = g;
        N_ = std::max(1, (int)std::lround(previewSec / dt));

        // 연속 3중적분기의 이산화.
        A_ << 1, dt, dt * dt / 2.0,
              0, 1,  dt,
              0, 0,  1;
        B_ << dt * dt * dt / 6.0, dt * dt / 2.0, dt;
        C_ << 1.0, 0.0, -comMassScale * zc_ / g_;

        // ZMP-오차 적분을 포함한 확장 시스템(4상태).
        Eigen::Matrix4d Atil = Eigen::Matrix4d::Zero();
        Eigen::Vector4d Btil = Eigen::Vector4d::Zero();
        Eigen::Vector4d Itil = Eigen::Vector4d::Zero();
        Eigen::RowVector3d CA = C_ * A_;
        double CB = (C_ * B_)(0, 0);
        Atil(0, 0) = 1.0;  Atil.block<1, 3>(0, 1) = CA;
        Atil.block<3, 3>(1, 1) = A_;
        Btil(0) = CB;      Btil.segment<3>(1) = B_;
        Itil(0) = 1.0;

        Eigen::Matrix4d Qtil = Eigen::Matrix4d::Zero();
        Qtil(0, 0) = Qe;
        Eigen::MatrixXd Rm(1, 1); Rm(0, 0) = R;

        Eigen::MatrixXd P = dare(Atil, Btil, Qtil, Rm);

        double r = (Rm + Btil.transpose() * P * Btil)(0, 0);
        Eigen::RowVector4d K = (Btil.transpose() * P * Atil) / r;   // [Gi, Gx(3)]
        Gi_ = K(0);
        Gx_ = K.tail<3>();

        // 미리보기 게인 Gd(l) = (1/r) Btil^T (Ac^T)^{l-1} P Itil,  l=1..N.
        Eigen::Matrix4d Ac = Atil - Btil * K;
        Gp_.assign(N_, 0.0);
        Eigen::Vector4d v = P * Itil;   // (Ac^T)^0 P Itil
        for (int l = 0; l < N_; ++l) {
            Gp_[l] = (Btil.transpose() * v)(0) / r;   // Gp_[l] = Gd(l+1)
            v = Ac.transpose() * v;
        }
        reset(Eigen::Vector2d::Zero());
    }

    void reset(const Eigen::Vector2d& com0,
               const Eigen::Vector2d& comVel0 = Eigen::Vector2d::Zero()) {
        xx_ << com0(0), comVel0(0), 0.0;
        xy_ << com0(1), comVel0(1), 0.0;
        eint_x_ = eint_y_ = 0.0;
    }

    // 한 스텝 진행. zmpRefX/Y: 앞으로 N 샘플의 desired ZMP(윈도우). COM ref 갱신.
    void update(const std::vector<double>& zmpRefX, const std::vector<double>& zmpRefY) {
        stepAxis(xx_, eint_x_, zmpRefX);
        stepAxis(xy_, eint_y_, zmpRefY);
    }

    // 측정 COM 피드백을 반영한 한 스텝(관측기+LQR 피드백, 균형 안정화).
    //   내부 상태의 위치/속도를 측정값 쪽으로 alpha 만큼 당긴 뒤 표준 스텝을 밟는다.
    //   측정 COM 이 발산하면 preview 가 이를 되돌리는 COM 궤적을 지령 → 안정화.
    void updateWithFeedback(const std::vector<double>& refx, const std::vector<double>& refy,
                            const Eigen::Vector2d& comMeas, const Eigen::Vector2d& comVelMeas,
                            double alpha) {
        xx_(0) = (1 - alpha) * xx_(0) + alpha * comMeas(0);
        xx_(1) = (1 - alpha) * xx_(1) + alpha * comVelMeas(0);
        xy_(0) = (1 - alpha) * xy_(0) + alpha * comMeas(1);
        xy_(1) = (1 - alpha) * xy_(1) + alpha * comVelMeas(1);
        stepAxis(xx_, eint_x_, refx);
        stepAxis(xy_, eint_y_, refy);
    }

    Eigen::Vector2d comPos() const { return {xx_(0), xy_(0)}; }
    Eigen::Vector2d comVel() const { return {xx_(1), xy_(1)}; }
    Eigen::Vector2d comAcc() const { return {xx_(2), xy_(2)}; }
    Eigen::Vector2d zmp() const { return {(C_ * xx_)(0), (C_ * xy_)(0)}; }
    int previewSize() const { return N_; }

private:
    void stepAxis(Eigen::Vector3d& x, double& eint, const std::vector<double>& pref) {
        int n = std::min<int>(N_, (int)pref.size());
        if (n == 0) return;
        double p = (C_ * x)(0);              // 현재 ZMP 추정
        eint += (p - pref[0]);               // ZMP 오차 적분
        // 미리보기 항: u 에 +Σ Gd(l)·p_ref[k+l] 로 들어간다(예견하여 COM 을
        // ZMP 진행 방향으로 미리 이동). 부호는 참고 구현(보행 검증됨)과 일치.
        double preview = 0.0;
        for (int l = 1; l < n; ++l) preview += Gp_[l - 1] * pref[l];
        double u = -Gi_ * eint - (Gx_ * x)(0) + preview;   // 최적 jerk
        x = A_ * x + B_ * u;
    }

    double dt_ = 0.002, zc_ = 0.85, g_ = 9.81;
    int N_ = 500;
    Eigen::Matrix3d A_;
    Eigen::Vector3d B_;
    Eigen::RowVector3d C_;
    double Gi_ = 0.0;
    Eigen::RowVector3d Gx_ = Eigen::RowVector3d::Zero();
    std::vector<double> Gp_;
    Eigen::Vector3d xx_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d xy_ = Eigen::Vector3d::Zero();
    double eint_x_ = 0.0, eint_y_ = 0.0;
};

}  // namespace kin
