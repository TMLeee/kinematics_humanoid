#pragma once
// 발목 어드미턴스(임피던스) 제어 — 발 F/T 로 발바닥 CoP 를 읽어 발목 pitch/roll 을 순응시킨다.
//
// 역할(중요): 이것은 "전역 ZMP 를 배치하는 균형 제어기"가 아니라 **발을 지면에 순응시키는
//   국소 컴플라이언스**다. 각 발이 자기 CoP 를 자기 발의 중립점(발 box 중심) 쪽으로
//   부드럽게 되돌려, 발이 모서리로 들리는 것(tipping)을 막고 착지 충격을 흡수한다.
//   - 전역 균형(COM/DCM)을 이 루프로 잡으려 하면 안 된다. CoP 를 고정 목표에 붙들면
//     COM 은 LIPM 고유모드로 그대로 발산한다(ω 로 지수 발산). 그건 상위 루프의 몫이다.
//   - 그래서 이득을 작게 두고 1차 지연 + 클램프를 건다. 정적 평형을 이기려 들지 않고
//     과도 성분(충격·기울어짐)에만 반응하게 만드는 것이 목적이다.
//
// 부호는 추측하지 않고 실측했다(무릎 굽힌 양발지지 스탠스, 발별 로컬 CoP):
//     ∂CoP_x / ∂θ_anklePitch = −0.64 … −0.88 m/rad   (대표 −0.75)
//     ∂CoP_y / ∂θ_ankleRoll  = −1.99 … −3.52 m/rad   (대표 −2.5)
//   둘 다 음수이므로, CoP 를 목표 쪽으로 옮기려면 (측정−목표) 에 **+k** 를 곱해 더한다.
//   완전 보상 이득은 1/0.75 ≈ 1.33 (pitch), 1/2.5 ≈ 0.40 (roll) 이고,
//   기본값은 그 20~35% 수준으로 잡아 과도 응답만 순응하도록 한다.
//
// CoP 는 센서 프레임에서 바로 풀 수 있어 FK 가 필요 없다:
//     m = p × f,  p_z = kFtToSoleZ  ⇒  cop_x = (p_z·f_x − m_y)/f_z,  cop_y = (m_x + p_z·f_y)/f_z
//   f 와 m 이 함께 부호 반전돼도 비율이 보존되므로 센서 부호 규약과 무관하다
//   (MuJoCo 는 본 모델에서 정지 시 f_z < 0).
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

#include "util/MathUtil.h"
#include "util/RobotDefs.h"

namespace kin {

class AnkleAdmittance {
public:
    struct Params {
        double enable  = 1.0;    // 0 = off
        double kPitch  = 0.05;   // [rad/m] CoP x → ankle pitch (루프이득 0.05·0.75 ≈ 0.04)
        double kRoll   = 0.015;  // [rad/m] CoP y → ankle roll  (루프이득 0.015·2.5 ≈ 0.04)
        double tau     = 0.05;   // 발목 보정 1차 지연 [s]
        double clamp   = 0.10;   // 발목 보정 한계 [rad]
        double ftTau   = 0.02;   // F/T 저역통과 시정수 [s]
        double fzMin   = 30.0;   // 이 하중 미만이면 공중으로 보고 0 으로 되돌림 [N]
        // 하중 스케일링: 보정량에 (|Fz| / fzNominal) 을 곱한다.
        //   모멘트 기반(Δθ = −k_M·M)은 M ≈ −cop·Fz 라 실효이득이 자동으로 하중에 비례한다.
        //   CoP 기반은 그 성질이 없어서, 발이 가볍게 실린 순간(착지 직후·이지 직전)에
        //   같은 크기로 발목을 꺾어 과보정한다. 이 항을 넣으면 모멘트 기반과 등가가
        //   되면서 부호 규약 독립성은 유지된다.
        //   fzNominal = 체중의 절반(양발 균등 분담). 0 이면 스케일링 off(고정 이득).
        double fzNominal = 469.0;   // ≈ 95.62 kg · 9.81 / 2
        // 목표 CoP = 발목 원점(= 발목 모멘트 0). 발 box 중심(+0.03)이 아니다!
        //   정지 시 자연 CoP 는 발목 기준 약 −0.019 m 라, 발 중심을 목표로 삼으면 49 mm 를
        //   상시로 밀어야 해서 정적 평형과 싸우고 로봇이 뒤로 밀린다(실측: 10 s 에 dx −0.07 m).
        //   발목 원점을 목표로 두면 정적 오프셋이 0.05·0.019 ≈ 0.001 rad(0.05°) 로 무시할 수준이 되어
        //   정적 평형을 건드리지 않고 과도 성분에만 순응한다.
        double copRefX = 0.0;
        double copRefY = 0.0;
    };

    void init(double dt, const Params& p) { dt_ = dt; p_ = p; reset(); }

    void reset() {
        for (int i = 0; i < 2; ++i) {
            dPitch_[i] = 0.0; dRoll_[i] = 0.0;
            ftF_[i].setZero(); haveFt_[i] = false;
            cop_[i].setZero(); fz_[i] = 0.0; loaded_[i] = false;
        }
    }

    bool enabled() const { return p_.enable > 0.5; }

    // 매 제어주기 1회. ftL/ftR 는 센서 site 프레임, 센서 원점 기준 (fx,fy,fz,mx,my,mz).
    void update(const Vector6d& ftL, const Vector6d& ftR) {
        updateFoot(0, ftL);
        updateFoot(1, ftR);
        // off 면 보정 출력만 0 으로 (CoP/하중 진단값은 그대로 유지해 A/B 비교에 쓴다).
        if (!enabled()) { dPitch_[0] = dPitch_[1] = dRoll_[0] = dRoll_[1] = 0.0; }
    }

    // 출력 관절 지령에 더할 발목 보정 [rad].
    double pitch(Side s) const { return dPitch_[s == Side::Left ? 0 : 1]; }
    double roll (Side s) const { return dRoll_ [s == Side::Left ? 0 : 1]; }

    // 진단용: 필터링된 발별 로컬 CoP / 수직 하중 / 접지 여부.
    const Eigen::Vector2d& cop(Side s) const { return cop_[s == Side::Left ? 0 : 1]; }
    double fz(Side s)     const { return fz_[s == Side::Left ? 0 : 1]; }
    bool   loaded(Side s) const { return loaded_[s == Side::Left ? 0 : 1]; }

private:
    void updateFoot(int i, const Vector6d& ft) {
        // F/T 저역통과 — CoP 는 나눗셈이라 잡음이 증폭되므로 wrench 단계에서 거른다.
        const double af = dt_ / std::max(dt_, p_.ftTau);
        if (!haveFt_[i]) { ftF_[i] = ft; haveFt_[i] = true; }
        else             { ftF_[i] += af * (ft - ftF_[i]); }

        const Vector3d f = ftF_[i].head(3), m = ftF_[i].tail(3);
        const double a = dt_ / std::max(dt_, p_.tau);

        fz_[i] = std::fabs(f.z());
        loaded_[i] = (fz_[i] >= p_.fzMin);
        if (!loaded_[i]) {
            // 공중(스윙) 발: 보정을 0 으로 부드럽게 되돌려 착지 시 튐을 막는다.
            dPitch_[i] += a * (0.0 - dPitch_[i]);
            dRoll_ [i] += a * (0.0 - dRoll_ [i]);
            return;
        }

        // 센서 프레임에서 발바닥 평면 위 CoP.
        cop_[i].x() = (kFtToSoleZ * f.x() - m.y()) / f.z();
        cop_[i].y() = (m.x() + kFtToSoleZ * f.y()) / f.z();
        // 접촉 불량 시 발 밖으로 튀는 값이 나오므로 발 크기로 제한.
        cop_[i].x() = clamp(cop_[i].x(), kFootCenterX - kFootHalfLen,   kFootCenterX + kFootHalfLen);
        cop_[i].y() = clamp(cop_[i].y(), kFootCenterY - kFootHalfWidth, kFootCenterY + kFootHalfWidth);

        // ∂CoP/∂θ < 0 이므로 (측정 − 목표) 에 +k. 1차 지연으로 순응(damping control).
        //   하중 스케일 w = |Fz|/fzNominal 을 곱해 모멘트 기반과 등가로 만든다
        //   (가볍게 실린 발은 자동으로 보정이 작아지고, 공중이면 0 으로 사라진다).
        const double w = (p_.fzNominal > 1.0) ? (fz_[i] / p_.fzNominal) : 1.0;
        const double tgtPitch = w * p_.kPitch * (cop_[i].x() - p_.copRefX);
        const double tgtRoll  = w * p_.kRoll  * (cop_[i].y() - p_.copRefY);
        dPitch_[i] += a * (tgtPitch - dPitch_[i]);
        dRoll_ [i] += a * (tgtRoll  - dRoll_ [i]);
        dPitch_[i] = clamp(dPitch_[i], -p_.clamp, p_.clamp);
        dRoll_ [i] = clamp(dRoll_ [i], -p_.clamp, p_.clamp);
    }

    static double clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }

    Params p_;
    double dt_ = 0.002;
    Vector6d ftF_[2];
    bool     haveFt_[2] = {false, false};
    Eigen::Vector2d cop_[2];
    double   fz_[2] = {0.0, 0.0};
    bool     loaded_[2] = {false, false};
    double   dPitch_[2] = {0.0, 0.0};
    double   dRoll_[2]  = {0.0, 0.0};
};

}  // namespace kin
