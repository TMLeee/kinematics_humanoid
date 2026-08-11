#pragma once
// MuJoCo 시뮬레이션 환경 래퍼.
// 모델(XML) 로드, 스텝 진행, GLFW 기반 렌더링/마우스 카메라 조작을 담당한다.
#include <mujoco/mujoco.h>
#include <chrono>
#include <string>

#include "viz/WalkPlotter.h"

struct GLFWwindow;

class MujocoEnv {
public:
    MujocoEnv() = default;
    ~MujocoEnv();

    // XML 모델을 로드하고 데이터 버퍼를 준비한다. 실패 시 false.
    // 모델에 키프레임이 있으면 0번 키프레임 자세로 초기화한다.
    bool load(const std::string& xml_path);

    // 지정한 키프레임 자세로 상태를 리셋한다(qpos/qvel/ctrl). 범위를 벗어나면 무시.
    void resetToKeyframe(int key = 0);

    // 렌더링 창을 연다 (headless 로 쓰려면 호출하지 않는다).
    bool initViewer(const std::string& title = "kin_humanoid");

    // 초기(자유) 카메라 각도/위치를 설정한다. initViewer 이후 호출.
    //   azimuth/elevation: 도(deg), distance: lookat 로부터의 거리(m),
    //   (cx,cy,cz): 바라보는 지점 lookat(m).
    void setCamera(double azimuth, double elevation, double distance,
                   double cx, double cy, double cz);

    // 현재 카메라 파라미터를 터미널에 출력한다(뷰어에서 'P' 키).
    void printCamera() const;

    // 현재 카메라 뷰를 파일에 저장한다('P' 키에서 호출). 다음 실행 때 자동 복원된다.
    void saveCamera() const;

    // 저장된 카메라 뷰가 있으면 불러와 적용한다(있으면 true). initViewer 에서 호출.
    bool loadCamera();

    // 충돌 지오메트리(그룹) 렌더링 on/off. 기본은 group 2(충돌 프리미티브) 숨김.
    void setGeomGroupVisible(int group, bool visible);

    // 모든 관절 액추에이터를 위치제어(position servo) 모드로 전환한다.
    // 로드된 모델의 gain/bias 파라미터를 직접 수정하므로 XML/서브모듈은 건드리지 않는다.
    //   force = kp*(ctrl - q) - kv*qdot   → ctrl 이 목표 관절각(rad)이 된다.
    // 기존 ctrlrange(토크 한계)는 forcerange 로 옮겨 실제 토크 포화를 유지한다.
    // kinematics-level 제어는 이 모드 위에서 목표 관절각을 d->ctrl 에 쓴다.
    void setJointPositionMode(double kp, double kv);

    // 현재 관절 자세를 위치 목표로 잡는다(현재 자세 유지). setJointPositionMode 후 호출.
    void holdCurrentPose();

    // 물리 한 스텝 진행.
    void step();

    // 현재 상태를 화면에 그린다. 창이 닫혔으면 false.
    bool render();

    bool viewerShouldClose() const;

    // --- 키보드 텔레옵 폴링 (요구사항 6: w/s 전후, a/d 좌우 게걸음) ---
    // GLFW 를 헤더 밖으로 노출하지 않기 위해 의미 있는 키 상태만 POD 로 돌려준다.
    struct KeyInput {
        bool w = false, a = false, s = false, d = false;   // 이동
        bool q = false, e = false;                         // 좌/우 회전(옵션)
        bool space = false;                                // 보행 on/off 토글(엣지)
        bool h = false;                                    // 보행준비 자세/WBC 시작(엣지)
        bool x = false;                                    // 정지
    };
    KeyInput pollKeys();

    // 화면 하단에 표시할 상태 텍스트(제어기 상태/명령 등)를 설정한다.
    void setStatusText(const std::string& text) { status_ = text; }

    // 실시간 보행 그래프. 매 tick 데이터를 push 하고, 'G' 키로 표시 토글.
    kin::WalkPlotter& walkPlotter() { return plotter_; }

    mjModel* model() { return m_; }
    mjData*  data()  { return d_; }

    // --- 마우스 콜백 내부 처리 (GLFW 트램폴린에서 호출) ---
    void onMouseButton(int button, int action, int mods);
    void onMouseMove(double xpos, double ypos);
    void onScroll(double yoffset);
    void onKey(int key, int action, int mods);

private:
    mjModel* m_ = nullptr;
    mjData*  d_ = nullptr;

    GLFWwindow* window_ = nullptr;
    mjvCamera cam_{};
    mjvOption opt_{};
    mjvScene  scn_{};
    mjrContext con_{};
    bool viewer_ = false;

    // 마우스 상태
    bool   btn_left_ = false, btn_middle_ = false, btn_right_ = false;
    double last_x_ = 0.0, last_y_ = 0.0;

    // 키보드 상태(space/h 엣지 검출용) / 하단 상태 텍스트
    bool   space_prev_ = false;
    bool   h_prev_ = false;
    std::string status_;

    // 실시간 보행 그래프
    kin::WalkPlotter plotter_;
    bool             show_plots_ = true;   // 기본 표시('G' 로 토글)

    // --- 좌상단 오버레이(실시간 배율/FPS/시뮬 시간) 통계 ---
    void drawOverlay(const mjrRect& viewport);
    using Clock = std::chrono::steady_clock;
    bool   stat_init_ = false;
    Clock::time_point stat_wall_{};   // 마지막 통계 갱신 벽시계 시각
    double stat_sim_  = 0.0;          // 마지막 통계 갱신 시점의 sim 시간
    int    stat_frames_ = 0;          // 갱신 구간 누적 프레임 수
    double disp_rtf_ = 0.0;           // 표시용 실시간 배율
    double disp_fps_ = 0.0;           // 표시용 FPS
};
