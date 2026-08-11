#pragma once
// RobotIO 의 실제 로봇 백엔드 — [스켈레톤/틀만].
//
// 요구사항 8: 실제 로봇 연결도 고려해 베이스(RobotIO)를 상속한 틀만 둔다.
//   실제 구현(EtherCAT/CAN 통신, 상태 추정, 안전 관리 등)은 이후 채운다.
//   지금은 컴파일만 되며 동작하지 않는다(사용 시 경고).
#include "io/RobotIO.h"

namespace kin {

class RealIO : public RobotIO {
public:
    explicit RealIO(double control_dt = 0.002) : control_dt_(control_dt) {}

    bool init() override;                                   // TODO: 통신 개방/초기화
    bool read(RobotState& out) override;                   // TODO: 엔코더/IMU/FT 수신
    void writeJointTargets(const VectorXd& qDesJoints) override;  // TODO: 목표 송신
    void step() override;                                  // TODO: 제어주기 동기(RT)
    bool running() override { return running_; }
    double controlDt() const override { return control_dt_; }
    VelocityCommand velocityCommand() override;            // TODO: 텔레옵/조이스틱

private:
    double control_dt_ = 0.002;
    bool   running_ = false;
    // TODO: 통신 핸들(EtherCAT master 등), 상태 추정기, 안전 감시 멤버.
};

}  // namespace kin
