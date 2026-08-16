#pragma once
// RobotIO 의 시뮬레이터 백엔드 — MujocoEnv 를 래핑한 완전 구현.
//
// 물리는 모델 네이티브 timestep(0.5ms)으로 돌리고, 제어는 controlDt(기본 2ms)마다
// 수행한다(= 4 물리 서브스텝). 렌더링은 sim 시간 1/60초마다 수행하며 vsync 로
// 대략 실시간 페이싱된다.
#include "config/WalkingConfig.h"
#include "io/RobotIO.h"
#include "simulator/MujocoEnv.h"

namespace kin {

class SimIO : public RobotIO {
public:
    explicit SimIO(const std::string& model_path, bool with_viewer = true)
        : model_path_(model_path), with_viewer_(with_viewer) {}

    bool init() override;
    bool read(RobotState& out) override;
    void writeJointTargets(const VectorXd& qDesJoints) override;
    void step() override;
    bool running() override;
    double controlDt() const override { return control_dt_; }
    VelocityCommand velocityCommand() override;
    void render() override;
    void setStatus(const std::string& text) override { env_.setStatusText(text); }

    // 모터 위치 서보 제어 이득 설정(init 전에 호출). 기본값은 gConfig.servoKp/Kv.
    void setServoGains(double kp, double kv) { servo_kp_ = kp; servo_kv_ = kv; }
    void setServoIntegral(double ki, double clampRad) { servo_ki_ = ki; i_clamp_ = clampRad; }
    void setGravityComp(double g) { grav_comp_ = g; }
    // 속도 피드포워드 계수(0=off, 1=완전보상). MuJoCo 위치서보의 −kv·q̇ 를 상쇄한다.
    void setVelFeedforward(double a) { kv_ff_ = a; }

    // 측정 COM (sim 의 subtree_com, world x,y).
    Eigen::Vector2d measuredCom() const;
    // 측정 COM (world x,y,z) — 그래픽 마커용.
    Eigen::Vector3d measuredCom3() const;

    // 측정 ZMP = 양발 **F/T 센서** wrench 로부터 구한 지면 CoP (world x,y).
    //   실제 로봇과 동일한 신호원(F/T)을 쓰므로 RealIO 로 그대로 이식된다.
    //   양발 모두 하중이 없으면(공중) false.
    bool measuredZmp(Eigen::Vector2d& zmp) const;

    // 검증용: MuJoCo 접촉력(mj_contactForce) 기반 CoP. 시뮬레이터 전용.
    //   measuredZmp() 의 F/T 경로가 맞는지 대조하는 데 쓴다(정지 시 두 값의 차 ≈ 1.7 mm).
    bool measuredZmpContact(Eigen::Vector2d& zmp) const;

    MujocoEnv& env() { return env_; }

private:
    MujocoEnv   env_;
    std::string model_path_;
    bool        with_viewer_ = true;

    // 모터 위치 서보 제어 이득(gConfig 에서 기본값, setServoGains 로 오버라이드).
    double servo_kp_ = config::gConfig.servoKp;
    double servo_kv_ = config::gConfig.servoKv;
    double servo_ki_     = config::gConfig.servoKi;         // 적분 I텀
    double i_clamp_      = config::gConfig.servoIClampRad;  // anti-windup(위치 오프셋)
    double grav_comp_    = config::gConfig.gravityComp;     // 중력보상 FF 계수
    double   kv_ff_      = config::gConfig.servoKvFF;       // 속도 FF 계수
    VectorXd eint_;                                         // 관절별 적분 오차 ∫e
    VectorXd qdes_prev_;                                    // 직전 tick 위치 지령(q̇_des 산출용)
    bool     have_qdes_prev_ = false;

    // 발 F/T 센서: sensordata 주소와 site id(월드 포즈용). 없으면 -1.
    int ft_adr_[4]  = {-1, -1, -1, -1};   // [LF_force, LF_torque, RF_force, RF_torque]
    int ft_site_[2] = {-1, -1};           // [LF_FT, RF_FT]
    bool ft_ok_ = false;                  // 4개 센서 + 2개 site 를 모두 찾았는가

    double control_dt_ = config::gConfig.controlDt;   // 제어 주기 [s]
    int    substeps_   = 4;                            // control_dt / physics_timestep
    double last_render_sim_ = -1e9;
    int    base_body_ = 1;                             // subtree_com 루트(base_link)

    VelocityCommand cmd_;          // 램프된 현재 명령

    // 텔레옵 속도 한계/램프(gConfig 에서).
    double v_fwd_max_ = config::gConfig.vFwdMax;
    double v_lat_max_ = config::gConfig.vLatMax;
    double v_yaw_max_ = config::gConfig.vYawMax;
    double v_accel_   = config::gConfig.vAccel;
    double yaw_accel_ = config::gConfig.yawAccel;
};

}  // namespace kin
