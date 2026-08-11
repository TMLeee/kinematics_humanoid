#pragma once
// ROS 인터페이스 — [자리만 / 구현 제외].
//
// 요구사항 9: ROS 는 최종(외부) 인터페이스에만 쓸 것이므로 여기서는 "위치"만
//   고려하고 구현은 제외한다. 실제 로봇/상위 시스템과 연동할 때, 아래와 같은
//   퍼블리셔/서브스크라이버를 이 계층에 붙인다(빌드에는 포함하지 않음).
//
//   구독(sub):  /cmd_vel (geometry_msgs/Twist)   -> VelocityCommand
//               /walk_enable (std_msgs/Bool)      -> 보행 on/off
//   발행(pub):  /joint_states (sensor_msgs/JointState)
//               /com, /zmp, /footsteps            -> 디버그/모니터링
//
// 설계 메모: ROS 노드는 제어 루프와 분리(별도 스레드/프로세스)하고,
//   VelocityCommand / RobotState 만 락-프리 큐로 주고받아 제어 주기를 방해하지
//   않게 한다. 아래 클래스는 그 연결 지점을 표시하는 자리표시자이다.
//
// #ifdef USE_ROS  ... 로 감싸 실제 빌드에서 완전히 제외한다.
#if defined(USE_ROS)

namespace kin {

class RosInterface {
public:
    // TODO: rclcpp/ros::NodeHandle 초기화, cmd_vel 구독, joint_states 발행 등.
    bool init();
    void spinOnce();
    // VelocityCommand latestCommand() const;
    // void publishState(const RobotState& s);
};

}  // namespace kin

#endif  // USE_ROS
