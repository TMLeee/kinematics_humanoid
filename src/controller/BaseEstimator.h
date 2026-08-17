#pragma once
// 가상 floating base(virtual joint)의 pose 추정기.
//
// ── 원리 ────────────────────────────────────────────────────────────────────
// "접촉 중인 발은 world 에 고정되어 있다" 는 가정 하나로 base pose 를 정한다
// (kinematic odometry). 발마다 접지 순간의 world pose 를 **anchor** 로 붙잡아 두고,
// 매 주기 관절각으로 FK 를 풀어 "그 anchor 를 만족하는 base pose" 를 역산한다.
//
//   R_base = R_anchor · R_footInBase^T ,   p_base = p_anchor − R_base · p_footInBase
//
// - 한 지지구간 안에서는 적분이 없다 → 드리프트 없음(FK 정확).
// - 지지 교체 시 새 지지발의 anchor 를 **직전 추정**에서 물려받는다 → base 가 점프하지 않는다.
//   (계획 포즈로 매 tick 스냅하던 기존 방식은 스텝마다 스윙발 추종오차만큼 점프했다.)
// - 스윙발 목표가 절대 world 계획 포즈이므로 anchor 오차는 누적되지 않는다:
//   anchor_n = plan_n + (그 스텝의 추종오차) 로 매 스텝 리셋된다.
//
// ── 평지 가정과 그 대체 지점 ─────────────────────────────────────────────────
// anchor 의 z 와 roll/pitch 는 "발이 지면에 평평하게 닿는다" 로 고정하고 x/y/yaw 만
// 누적한다(flatten). 이 가정이 곧 프레임 오차의 주범이며(docs/virtual_joint_review.md §2:
// 발 tilt 무시로 최대 113 mm), **그것을 대체하는 자리가 아래 Correction 인터페이스**다.
//
// ── 상위 레벨 보정 (IMU / SLAM) ──────────────────────────────────────────────
// base pose 는 anchor 에서 유일하게 결정되므로, base 를 보정하는 것은 곧 **anchor 를
// 보정하는 것**이다. applyCorrection() 이 그 변환을 해 준다:
//   IMU  → attitude(roll/pitch). yaw 는 드리프트하고 계획 heading 과 어긋나므로 쓰지 않는다.
//   SLAM → position, yaw.
// 현재 호출자는 없다(seam 만 준비). 붙이는 방법은 applyCorrection() 주석 참조.
#include <Eigen/Dense>

#include "util/MathUtil.h"

namespace kin {

class BaseEstimator {
public:
    // 상위 레벨(IMU/SLAM)이 주는 base pose 관측. have* 가 true 인 성분만 반영되고,
    // weight(0..1)로 기존 추정과 블렌딩된다(1 = 관측을 그대로 신뢰).
    struct Correction {
        bool     haveAttitude = false;   // IMU: base 자세. roll/pitch 만 쓴다.
        Matrix3d attitude = Matrix3d::Identity();
        double   attitudeWeight = 1.0;

        bool   haveYaw = false;          // SLAM/자기계: base yaw
        double yaw = 0.0;
        double yawWeight = 1.0;

        bool     havePosition = false;   // SLAM: base 위치
        Vector3d position = Vector3d::Zero();
        double   positionWeight = 1.0;
    };

    // ── anchor 관리 ─────────────────────────────────────────────────────────
    // 발이 "공중 → 접지" 로 바뀌는 순간 호출한다. flat=true 면 평지로 투영한다.
    void setAnchor(Side s, const Vector3d& pWorld, const Matrix3d& RWorld,
                   bool flat, double groundZ) {
        Anchor& a = anchor(s);
        a.p = pWorld;
        a.R = RWorld;
        if (flat) flatten(a.p, a.R, groundZ);
        a.valid = true;
    }
    bool valid(Side s) const { return anchor(s).valid; }
    void invalidate(Side s) { anchor(s).valid = false; }
    const Vector3d& anchorPos(Side s) const { return anchor(s).p; }
    const Matrix3d& anchorRot(Side s) const { return anchor(s).R; }

    // anchor 의 z / roll·pitch 를 평지 접지로 고정하고 yaw 만 남긴다.
    static void flatten(Vector3d& p, Matrix3d& R, double groundZ) {
        p.z() = groundZ;
        R = rotZ(std::atan2(R(1, 0), R(0, 0)));
    }

    // ── base pose 산출 ──────────────────────────────────────────────────────
    // (p_bf, R_bf) = base 를 단위로 두고 FK 로 얻은 지지발 기준점의 base-상대 pose.
    // tilt 는 anchor 자세에 곱할 추가 회전(어드미턴스 순응 등). 없으면 단위.
    void solveBase(Side sup, const Vector3d& p_bf, const Matrix3d& R_bf,
                   const Matrix3d& tilt, Vector3d& p_base, Matrix3d& R_base) const {
        const Anchor& a = anchor(sup);
        const Matrix3d Ra = a.R * tilt;
        R_base = Ra * R_bf.transpose();
        p_base = a.p - R_base * p_bf;
    }

    // ── 상위 레벨 보정 ──────────────────────────────────────────────────────
    // 관측된 base pose 를 anchor 보정으로 환산해 반영한다.
    //   (p_bf, R_bf) 는 solveBase 에 넣는 것과 같은 값이어야 한다.
    //
    // 붙이는 방법(예):
    //   BaseEstimator::Correction c;
    //   c.haveAttitude = true;  c.attitude = imuBaseRot();  c.attitudeWeight = 1.0;
    //   est_.applyCorrection(sup, c, p_bf, R_bf);      // solveBase 직전에 호출
    // IMU 자세만 주면 anchor 의 roll/pitch 가 실측 발바닥 기울기로 대체된다
    // (docs/virtual_joint_review.md §8 의 R_sole_true 와 같은 식).
    void applyCorrection(Side sup, const Correction& c,
                         const Vector3d& p_bf, const Matrix3d& R_bf) {
        Anchor& a = anchor(sup);
        if (!a.valid) return;

        if (c.haveAttitude) {
            // 관측 base 자세 → 그것을 만족하는 anchor 자세: R_a' = R_base_meas · R_bf.
            // yaw 는 관측에서 취하지 않는다(드리프트). 기존 anchor yaw 를 유지한 채
            // roll/pitch 만 갈아 끼운다.
            const Matrix3d Ra_obs = c.attitude * R_bf;
            const double yawKeep = std::atan2(a.R(1, 0), a.R(0, 0));
            const Vector3d rpyObs = rotToRpy(Ra_obs);
            const Matrix3d Ra_mix = rpyToRot(rpyObs(0), rpyObs(1), yawKeep);
            a.R = blendRot(a.R, Ra_mix, c.attitudeWeight);
        }
        if (c.haveYaw) {
            const double yawCur = std::atan2(a.R(1, 0), a.R(0, 0));
            const Vector3d rpy = rotToRpy(a.R);
            // 관측 yaw 는 base 의 것이므로 anchor yaw 로 옮긴다: Δ = yaw_obs − yaw_base.
            const Matrix3d Rb = a.R * R_bf.transpose();
            const double yawBase = std::atan2(Rb(1, 0), Rb(0, 0));
            const double dyaw = wrapPi(c.yaw - yawBase) * clamp01(c.yawWeight);
            a.R = rpyToRot(rpy(0), rpy(1), yawCur + dyaw);
        }
        if (c.havePosition) {
            // 관측 base 위치 → anchor 위치: p_a' = p_base_meas + R_base · p_bf.
            const Matrix3d R_base = a.R * R_bf.transpose();
            const Vector3d pa_obs = c.position + R_base * p_bf;
            const double w = clamp01(c.positionWeight);
            a.p = (1.0 - w) * a.p + w * pa_obs;
        }
    }

private:
    struct Anchor {
        Vector3d p = Vector3d::Zero();
        Matrix3d R = Matrix3d::Identity();
        bool     valid = false;
    };
    Anchor aL_, aR_;

    Anchor&       anchor(Side s)       { return s == Side::Left ? aL_ : aR_; }
    const Anchor& anchor(Side s) const { return s == Side::Left ? aL_ : aR_; }

    static double clamp01(double v) { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }
    static double wrapPi(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
    static Matrix3d blendRot(const Matrix3d& A, const Matrix3d& B, double w) {
        const double t = clamp01(w);
        if (t <= 0.0) return A;
        if (t >= 1.0) return B;
        return Quaterniond(A).slerp(t, Quaterniond(B)).toRotationMatrix();
    }
};

}  // namespace kin
