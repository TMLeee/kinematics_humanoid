#include "MujocoEnv.h"

#include <GLFW/glfw3.h>
#include <cstdio>
#include <fstream>

// ---- GLFW 정적 트램폴린: 윈도우 user pointer 로 MujocoEnv 인스턴스를 찾는다 ----
namespace {
// 저장된 카메라 뷰 파일(작업 디렉터리 기준). 'P' 로 저장하면 다음 실행 때 복원된다.
constexpr const char* kCameraFile = "camera_view.cfg";

MujocoEnv* envOf(GLFWwindow* w) {
    return static_cast<MujocoEnv*>(glfwGetWindowUserPointer(w));
}
void mouseButtonCb(GLFWwindow* w, int button, int action, int mods) {
    if (auto* e = envOf(w)) e->onMouseButton(button, action, mods);
}
void cursorPosCb(GLFWwindow* w, double x, double y) {
    if (auto* e = envOf(w)) e->onMouseMove(x, y);
}
void scrollCb(GLFWwindow* w, double /*xoff*/, double yoff) {
    if (auto* e = envOf(w)) e->onScroll(yoff);
}
void keyCb(GLFWwindow* w, int key, int /*scancode*/, int action, int mods) {
    if (auto* e = envOf(w)) e->onKey(key, action, mods);
}
}  // namespace

MujocoEnv::~MujocoEnv() {
    if (viewer_) {
        mjr_freeContext(&con_);
        mjv_freeScene(&scn_);
    }
    if (d_) mj_deleteData(d_);
    if (m_) mj_deleteModel(m_);
    if (window_) glfwTerminate();
}

bool MujocoEnv::load(const std::string& xml_path) {
    char error[1000] = "";
    m_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m_) {
        std::fprintf(stderr, "[MujocoEnv] load failed: %s\n", error);
        return false;
    }
    d_ = mj_makeData(m_);
    if (!d_) return false;

    // 키프레임이 있으면 0번(정상 서기) 자세로 초기화한다.
    // 기본 qpos0 는 골반이 원점(z=0)이라 발이 바닥을 관통 → 접촉력 폭발한다.
    if (m_->nkey > 0) resetToKeyframe(0);
    return true;
}

void MujocoEnv::resetToKeyframe(int key) {
    if (!m_ || !d_) return;
    if (key < 0 || key >= m_->nkey) return;
    mj_resetDataKeyframe(m_, d_, key);
    mj_forward(m_, d_);   // 파생량(접촉 등) 재계산
}

bool MujocoEnv::initViewer(const std::string& title) {
    if (!glfwInit()) {
        std::fprintf(stderr, "[MujocoEnv] glfwInit failed\n");
        return false;
    }
    window_ = glfwCreateWindow(1200, 900, title.c_str(), nullptr, nullptr);
    if (!window_) {
        std::fprintf(stderr, "[MujocoEnv] window creation failed\n");
        return false;
    }
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjr_defaultContext(&con_);

    mjv_makeScene(m_, &scn_, 2000);
    mjr_makeContext(m_, &con_, mjFONTSCALE_150);

    plotter_.init();

    // 초기 카메라: 기본값을 잡은 뒤, 저장된 뷰(camera_view.cfg)가 있으면 그것으로 덮어쓴다.
    // 뷰어에서 'P' 를 누르면 현재 뷰가 저장되어 다음 실행 때 이 위치로 열린다.
    setCamera(/*azimuth=*/127.0, /*elevation=*/-17.0, /*distance=*/3.500,
              /*lookat=*/0.557, 0.330, 0.610);
    if (loadCamera())
        std::printf("[camera] loaded saved view from %s\n", kCameraFile);

    // 충돌 프리미티브(class "cls" = group 2)는 회색 박스로 보이므로 숨긴다.
    setGeomGroupVisible(2, false);
    // 바닥 평면(geom "ground")은 group 3 인데 MuJoCo 기본 옵션은
    // geomgroup=[1,1,1,0,0,0] 이라 group 3 이 꺼져 있어 안 보인다 → 켜준다.
    setGeomGroupVisible(3, true);

    // 마우스 콜백 등록
    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, mouseButtonCb);
    glfwSetCursorPosCallback(window_, cursorPosCb);
    glfwSetScrollCallback(window_, scrollCb);
    glfwSetKeyCallback(window_, keyCb);

    viewer_ = true;
    return true;
}

void MujocoEnv::setCamera(double azimuth, double elevation, double distance,
                          double cx, double cy, double cz) {
    cam_.type      = mjCAMERA_FREE;
    cam_.azimuth   = azimuth;
    cam_.elevation = elevation;
    cam_.distance  = distance;
    cam_.lookat[0] = cx;
    cam_.lookat[1] = cy;
    cam_.lookat[2] = cz;
}

void MujocoEnv::printCamera() const {
    std::printf("[camera] azimuth=%.1f elevation=%.1f distance=%.3f "
                "lookat=(%.3f, %.3f, %.3f)\n",
                cam_.azimuth, cam_.elevation, cam_.distance,
                cam_.lookat[0], cam_.lookat[1], cam_.lookat[2]);
}

void MujocoEnv::saveCamera() const {
    std::ofstream f(kCameraFile);
    if (!f) {
        std::fprintf(stderr, "[camera] save failed: %s\n", kCameraFile);
        return;
    }
    f << cam_.azimuth   << ' ' << cam_.elevation << ' ' << cam_.distance << ' '
      << cam_.lookat[0] << ' ' << cam_.lookat[1] << ' ' << cam_.lookat[2] << '\n';
    std::printf("[camera] saved view -> %s\n", kCameraFile);
}

bool MujocoEnv::loadCamera() {
    std::ifstream f(kCameraFile);
    if (!f) return false;
    double az, el, dist, cx, cy, cz;
    if (!(f >> az >> el >> dist >> cx >> cy >> cz)) return false;
    setCamera(az, el, dist, cx, cy, cz);
    return true;
}

void MujocoEnv::setGeomGroupVisible(int group, bool visible) {
    if (group < 0 || group >= mjNGROUP) return;
    opt_.geomgroup[group] = visible ? 1 : 0;
}

void MujocoEnv::setJointPositionMode(double kp, double kv) {
    if (!m_) return;
    for (int i = 0; i < m_->nu; ++i) {
        // 관절 트랜스미션만 위치 서보로 바꾼다(tocabi 액추에이터는 모두 해당).
        if (m_->actuator_trntype[i] != mjTRN_JOINT) continue;

        // 기존 ctrlrange(토크 한계) → forcerange 로 이전하여 토크 포화를 보존.
        if (m_->actuator_ctrllimited[i]) {
            m_->actuator_forcerange[2 * i]     = m_->actuator_ctrlrange[2 * i];
            m_->actuator_forcerange[2 * i + 1] = m_->actuator_ctrlrange[2 * i + 1];
            m_->actuator_forcelimited[i]       = 1;
            // ctrl 은 이제 위치 명령이므로 토크 범위로 클램프하지 않는다.
            m_->actuator_ctrllimited[i]        = 0;
        }

        // position 서보: force = kp*ctrl + (-kp*q - kv*qdot).
        m_->actuator_gaintype[i] = mjGAIN_FIXED;
        m_->actuator_biastype[i] = mjBIAS_AFFINE;
        for (int k = 0; k < mjNGAIN; ++k) m_->actuator_gainprm[i * mjNGAIN + k] = 0.0;
        for (int k = 0; k < mjNBIAS; ++k) m_->actuator_biasprm[i * mjNBIAS + k] = 0.0;
        m_->actuator_gainprm[i * mjNGAIN + 0] =  kp;
        m_->actuator_biasprm[i * mjNBIAS + 1] = -kp;
        m_->actuator_biasprm[i * mjNBIAS + 2] = -kv;
    }
}

void MujocoEnv::holdCurrentPose() {
    if (!m_ || !d_) return;
    for (int i = 0; i < m_->nu; ++i) {
        if (m_->actuator_trntype[i] != mjTRN_JOINT) continue;
        const int jnt = m_->actuator_trnid[2 * i];
        d_->ctrl[i] = d_->qpos[m_->jnt_qposadr[jnt]];
    }
}

void MujocoEnv::step() {
    mj_step(m_, d_);
}

bool MujocoEnv::render() {
    if (!viewer_ || glfwWindowShouldClose(window_)) return false;

    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

    mjv_updateScene(m_, d_, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
    mjr_render(viewport, &scn_, &con_);
    drawOverlay(viewport);
    if (show_plots_) plotter_.render(viewport, &con_);

    glfwSwapBuffers(window_);
    glfwPollEvents();
    return true;
}

// 좌상단에 실시간 배율 / FPS / 시뮬레이션 시간을 작은 글씨로 표시한다.
// 벽시계 대비 sim 시간 진행률로 실시간 배율을, 프레임 수로 FPS 를 약 0.5초마다 갱신한다.
void MujocoEnv::drawOverlay(const mjrRect& viewport) {
    const auto now = Clock::now();
    if (!stat_init_) {
        stat_wall_ = now;
        stat_sim_  = d_->time;
        stat_init_ = true;
    }
    ++stat_frames_;
    const double wall_dt =
        std::chrono::duration<double>(now - stat_wall_).count();
    if (wall_dt >= 0.5) {
        disp_fps_ = stat_frames_ / wall_dt;
        disp_rtf_ = (d_->time - stat_sim_) / wall_dt;   // sim진행/실제경과 = 실시간 배율
        stat_frames_ = 0;
        stat_wall_   = now;
        stat_sim_    = d_->time;
    }

    char labels[128];
    char values[128];
    std::snprintf(labels, sizeof(labels), "Real-time\nFPS\nSim time");
    std::snprintf(values, sizeof(values), "%.2fx\n%.1f\n%.2f s",
                  disp_rtf_, disp_fps_, d_->time);
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, labels, values, &con_);

    // 하단: 조작 도움말 + 제어기 상태 텍스트.
    static const char* kHelp =
        "[H] ready-pose  [Space] walk  [W/S] fwd/back  [A/D] strafe  [Q/E] turn  [X] stop  [G] graph";
    if (!status_.empty())
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, status_.c_str(), "", &con_);
    mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMRIGHT, viewport, kHelp, "", &con_);
}

bool MujocoEnv::viewerShouldClose() const {
    return viewer_ && window_ && glfwWindowShouldClose(window_);
}

// ---- 마우스 카메라 조작 ----
void MujocoEnv::onMouseButton(int /*button*/, int /*action*/, int /*mods*/) {
    // 현재 눌린 버튼 상태를 폴링해 갱신한다.
    btn_left_   = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS;
    btn_middle_ = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    btn_right_  = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS;
    glfwGetCursorPos(window_, &last_x_, &last_y_);
}

void MujocoEnv::onMouseMove(double xpos, double ypos) {
    if (!btn_left_ && !btn_middle_ && !btn_right_) {
        last_x_ = xpos; last_y_ = ypos;
        return;
    }
    const double dx = xpos - last_x_;
    const double dy = ypos - last_y_;
    last_x_ = xpos; last_y_ = ypos;

    int win_w = 1, win_h = 1;
    glfwGetWindowSize(window_, &win_w, &win_h);

    const bool shift =
        glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT)  == GLFW_PRESS ||
        glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action;
    if (btn_right_)        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;   // 팬(이동)
    else if (btn_left_)    action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V; // 회전
    else                   action = mjMOUSE_ZOOM;                             // 가운데=줌

    mjv_moveCamera(m_, action, dx / win_h, dy / win_h, &scn_, &cam_);
}

void MujocoEnv::onScroll(double yoffset) {
    // 휠 스크롤 = 줌.
    mjv_moveCamera(m_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, &scn_, &cam_);
}

void MujocoEnv::onKey(int key, int action, int /*mods*/) {
    if (action != GLFW_PRESS) return;
    // 'P': 현재 카메라 뷰를 출력하고 파일에 저장 → 다음 실행 때 이 위치로 열린다.
    if (key == GLFW_KEY_P) {
        printCamera();
        saveCamera();
    }
    // 'G': 보행 그래프 표시 토글.
    if (key == GLFW_KEY_G) show_plots_ = !show_plots_;
}

MujocoEnv::KeyInput MujocoEnv::pollKeys() {
    KeyInput k;
    if (!viewer_ || !window_) return k;
    auto down = [&](int key) { return glfwGetKey(window_, key) == GLFW_PRESS; };
    k.w = down(GLFW_KEY_W);
    k.a = down(GLFW_KEY_A);
    k.s = down(GLFW_KEY_S);
    k.d = down(GLFW_KEY_D);
    k.q = down(GLFW_KEY_Q);
    k.e = down(GLFW_KEY_E);
    k.x = down(GLFW_KEY_X);
    // space / h 는 눌린 순간(엣지)만 true → 토글/트리거용.
    bool space_now = down(GLFW_KEY_SPACE);
    k.space = space_now && !space_prev_;
    space_prev_ = space_now;
    bool h_now = down(GLFW_KEY_H);
    k.h = h_now && !h_prev_;
    h_prev_ = h_now;
    return k;
}
