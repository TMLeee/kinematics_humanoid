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

    // 모터 위치 서보 제어 이득 설정(init 전에 호출). 기본값은 config::kServoKp/Kv.
    void setServoGains(double kp, double kv) { servo_kp_ = kp; servo_kv_ = kv; }

    // 측정 COM (sim 의 subtree_com, world x,y).
    Eigen::Vector2d measuredCom() const;
    // 측정 ZMP = 지면 접촉 CoP (world x,y). 접촉 없으면(공중) false.
    bool measuredZmp(Eigen::Vector2d& zmp) const;

    MujocoEnv& env() { return env_; }

private:
    MujocoEnv   env_;
    std::string model_path_;
    bool        with_viewer_ = true;

    // 모터 위치 서보 제어 이득(config 에서 기본값, setServoGains 로 오버라이드).
    double servo_kp_ = config::kServoKp;
    double servo_kv_ = config::kServoKv;

    double control_dt_ = config::kControlDt;   // 제어 주기 [s]
    int    substeps_   = 4;                     // control_dt / physics_timestep
    double last_render_sim_ = -1e9;
    int    base_body_ = 1;                      // subtree_com 루트(base_link)

    VelocityCommand cmd_;          // 램프된 현재 명령

    // 텔레옵 속도 한계/램프(config 에서).
    double v_fwd_max_ = config::kVfwdMax;
    double v_lat_max_ = config::kVlatMax;
    double v_yaw_max_ = config::kVyawMax;
    double v_accel_   = config::kVaccel;
    double yaw_accel_ = config::kYawAccel;
};

}  // namespace kin
