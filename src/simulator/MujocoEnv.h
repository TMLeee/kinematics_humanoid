#pragma once
// MuJoCo 시뮬레이션 환경 래퍼.
// 모델(XML) 로드, 스텝 진행, GLFW 기반 렌더링/마우스 카메라 조작을 담당한다.
#include <mujoco/mujoco.h>
#include <string>

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

    // 충돌 지오메트리(그룹) 렌더링 on/off. 기본은 group 2(충돌 프리미티브) 숨김.
    void setGeomGroupVisible(int group, bool visible);

    // 물리 한 스텝 진행.
    void step();

    // 현재 상태를 화면에 그린다. 창이 닫혔으면 false.
    bool render();

    bool viewerShouldClose() const;

    mjModel* model() { return m_; }
    mjData*  data()  { return d_; }

    // --- 마우스 콜백 내부 처리 (GLFW 트램폴린에서 호출) ---
    void onMouseButton(int button, int action, int mods);
    void onMouseMove(double xpos, double ypos);
    void onScroll(double yoffset);

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
};
