#pragma once
// 보행 상태/자료구조 정의(발걸음 생성기와 제어기가 공유).
#include "util/MathUtil.h"
#include "util/RobotDefs.h"

namespace kin {

// 보행 위상.
enum class GaitPhase {
    Stand,          // 정지(양발 지지, 스윙 없음)
    DoubleSupport,  // 양발 지지(ZMP 이동 구간)
    SingleSupport   // 한발 지지(스윙발 이동 구간)
};

// 지면 상의 2D 포즈(발 착지/토르소).
struct Pose2 {
    double x = 0.0, y = 0.0, yaw = 0.0;
};

// 스윙발 목표(world 3D + yaw).
struct FootPose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

}  // namespace kin
