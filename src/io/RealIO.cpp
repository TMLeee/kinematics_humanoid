#include "io/RealIO.h"

#include <cstdio>

// [스켈레톤] 실제 로봇 I/O. 현재는 틀만 존재하며 동작하지 않는다.
namespace kin {

bool RealIO::init() {
    std::fprintf(stderr,
        "[RealIO] not implemented — use SimIO. (실 로봇 통신/상태추정 구현 예정)\n");
    running_ = false;
    return false;   // 아직 미구현
}

bool RealIO::read(RobotState& out) {
    // TODO: 엔코더 -> q(관절), IMU/상태추정 -> base pose, FT 센서 -> ftLeft/ftRight.
    out.valid = false;
    return false;
}

void RealIO::writeJointTargets(const VectorXd& /*qDesJoints*/) {
    // TODO: 관절 목표각(또는 토크)을 드라이버로 송신.
}

void RealIO::step() {
    // TODO: 실시간 제어주기 동기화(예: clock_nanosleep 로 control_dt 유지).
}

VelocityCommand RealIO::velocityCommand() {
    // TODO: 조이스틱/텔레옵 입력 매핑.
    return VelocityCommand{};
}

}  // namespace kin
