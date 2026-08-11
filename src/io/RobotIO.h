#pragma once
// 로봇 입출력(I/O) 추상 베이스 클래스.
//
// 요구사항 8: 출력은 시뮬레이터로 연결되지만 실제 로봇 연결도 고려한다.
//   제어기는 이 인터페이스로만 로봇과 "현재값(측정)"·"목표값(명령)"을 주고받는다.
//   - 시뮬레이터: SimIO (MujocoEnv 래핑, 완전 구현)
//   - 실제 로봇:  RealIO (스켈레톤/틀만 — 요구사항: 구현 제외)
//
// 요구사항 9: ROS 는 최종 인터페이스에만 쓰므로 여기서는 위치만 고려한다
//   (io/RosInterface.h 에 자리만 두고 구현은 제외).
#include <Eigen/Dense>

#include "util/MathUtil.h"
#include "util/RobotDefs.h"

namespace kin {

// 사용자 이동 명령(요구사항 6: w/s=전후, a/d=좌우 게걸음).
struct VelocityCommand {
    double vx   = 0.0;   // 전(+)/후(-) 속도 [m/s]
    double vy   = 0.0;   // 좌(+)/우(-) 게걸음 속도 [m/s]
    double vyaw = 0.0;   // 제자리 회전 속도 [rad/s] (옵션: q/e)

    bool walk = false;        // 유효 보행 상태(HumanoidController 가 설정; footstep 이 읽음)
    bool spaceEdge = false;   // Space 눌림(엣지): 보행 시작/정지 토글 요청
    bool prepareEdge = false; // 'h' 눌림(엣지): 보행준비 자세로 이동 후 WBC 시작
    bool stop = false;        // 'x': 즉시 정지
};

// 로봇 측정 상태(현재값).
struct RobotState {
    VectorXd q;          // nq(=40): [base_pos(3), base_quat(w,x,y,z), joints(33)]
    VectorXd dq;         // nv(=39): [base_lin(3), base_ang(3), joints(33)]
    Vector6d ftLeft  = Vector6d::Zero();   // 왼발 F/T (fx,fy,fz,mx,my,mz)
    Vector6d ftRight = Vector6d::Zero();   // 오른발 F/T
    bool     valid = false;

    RobotState() : q(VectorXd::Zero(kNq)), dq(VectorXd::Zero(kNv)) {}
};

class RobotIO {
public:
    virtual ~RobotIO() = default;

    virtual bool init() = 0;

    // 최신 측정 상태를 읽어 out 에 채운다. 성공 시 true.
    virtual bool read(RobotState& out) = 0;

    // 목표 관절각(33) 을 로봇/시뮬레이터에 쓴다.
    virtual void writeJointTargets(const VectorXd& qDesJoints) = 0;

    // 한 제어주기 진행. sim: 물리 스텝, real: 통신/주기 동기화.
    virtual void step() = 0;

    // 루프 유지 여부(창 닫힘/통신 종료 등).
    virtual bool running() = 0;

    // 제어 주기 [s].
    virtual double controlDt() const = 0;

    // 사용자 이동 명령(키보드/텔레옵).
    virtual VelocityCommand velocityCommand() = 0;

    // 시각화(sim 전용). 기본 no-op.
    virtual void render() {}

    // 상태 텍스트 표시(sim 오버레이). 기본 no-op.
    virtual void setStatus(const std::string& /*text*/) {}
};

}  // namespace kin
