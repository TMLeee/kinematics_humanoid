#pragma once
// 보행 디버그용 실시간 그래프(뷰어 오버레이). MuJoCo 내장 mjvFigure/mjr_figure 사용
// → 외부 플로팅 라이브러리 의존성 없음.
//
// 두 개의 시계열 그래프(X=시간):
//   - 상단: Lateral (Y 방향)   — 보행 시작 흔들림 분석의 핵심
//   - 하단: Sagittal (X 방향)
// 각 그래프에 5개 신호를 겹쳐 그린다:
//   footstep(지지발), ZMP ref, COM ref, ZMP 측정(CoP), COM 측정.
#include <mujoco/mujoco.h>
#include <Eigen/Core>
#include <cstring>

namespace kin {

class WalkPlotter {
public:
    static constexpr int kLines = 5;   // footstep, zmpRef, comRef, zmpMeas, comMeas
    enum Line { L_FOOT = 0, L_ZMPREF, L_COMREF, L_ZMPMEAS, L_COMMEAS };

    void init() {
        const char* names[kLines] = {"footstep", "zmp_ref", "com_ref", "zmp_meas", "com_meas"};
        const float col[kLines][3] = {
            {0.55f, 0.55f, 0.55f},  // footstep - gray
            {0.90f, 0.20f, 0.20f},  // zmp_ref  - red
            {0.20f, 0.45f, 1.00f},  // com_ref  - blue
            {1.00f, 0.60f, 0.10f},  // zmp_meas - orange
            {0.10f, 0.80f, 0.75f},  // com_meas - cyan
        };
        setupFig(figY_, "Lateral (Y)  [m] vs time [s]", names, col);
        setupFig(figX_, "Sagittal (X)  [m] vs time [s]", names, col);
        t0_ = 0.0; have_t0_ = false; count_ = 0;
    }

    // 매 제어 tick 호출. 내부에서 decim_ 배수로 솎아 ~100 Hz 로 기록.
    void push(double t,
              const Eigen::Vector2d& footstep, const Eigen::Vector2d& zmpRef,
              const Eigen::Vector2d& comRef,   const Eigen::Vector2d& comMeas,
              const Eigen::Vector2d& zmpMeas,  bool zmpValid) {
        if (count_++ % decim_ != 0) return;
        if (!have_t0_) { t0_ = t; have_t0_ = true; }
        float tt = static_cast<float>(t - t0_);
        add(figX_, L_FOOT, tt, footstep.x());
        add(figX_, L_ZMPREF, tt, zmpRef.x());
        add(figX_, L_COMREF, tt, comRef.x());
        add(figX_, L_COMMEAS, tt, comMeas.x());
        if (zmpValid) add(figX_, L_ZMPMEAS, tt, zmpMeas.x());

        add(figY_, L_FOOT, tt, footstep.y());
        add(figY_, L_ZMPREF, tt, zmpRef.y());
        add(figY_, L_COMREF, tt, comRef.y());
        add(figY_, L_COMMEAS, tt, comMeas.y());
        if (zmpValid) add(figY_, L_ZMPMEAS, tt, zmpMeas.y());
    }

    // 뷰어 화면 우측에 두 그래프를 세로로 배치해 렌더링.
    void render(const mjrRect& vp, const mjrContext* con) {
        int pw = vp.width * 42 / 100;      // 패널 폭 ≈ 42%
        if (pw < 200) pw = vp.width / 2;
        int left = vp.left + vp.width - pw;
        int hh = vp.height / 2;
        mjrRect rY{left, vp.bottom + hh, pw, vp.height - hh};
        mjrRect rX{left, vp.bottom, pw, hh};
        mjr_figure(rY, &figY_, con);
        mjr_figure(rX, &figX_, con);
    }

    void clear() { for (int l = 0; l < kLines; ++l) { figX_.linepnt[l] = 0; figY_.linepnt[l] = 0; } have_t0_ = false; }

private:
    static void setupFig(mjvFigure& fig, const char* title,
                         const char* names[kLines], const float col[kLines][3]) {
        mjv_defaultFigure(&fig);
        std::strncpy(fig.title, title, sizeof(fig.title) - 1);
        std::strncpy(fig.xlabel, "time", sizeof(fig.xlabel) - 1);
        std::strncpy(fig.xformat, "%.1f", sizeof(fig.xformat) - 1);
        std::strncpy(fig.yformat, "%.2f", sizeof(fig.yformat) - 1);
        fig.flg_extend = 1;      // 데이터에 맞춰 자동 범위 확장
        fig.flg_legend = 1;
        fig.figurergba[3] = 0.75f;   // 반투명 배경
        fig.gridsize[0] = 5; fig.gridsize[1] = 5;
        fig.linewidth = 1.5f;        // (figure 당 스칼라)
        for (int l = 0; l < kLines; ++l) {
            std::strncpy(fig.linename[l], names[l], sizeof(fig.linename[l]) - 1);
            fig.linergb[l][0] = col[l][0];
            fig.linergb[l][1] = col[l][1];
            fig.linergb[l][2] = col[l][2];
            fig.linepnt[l] = 0;
        }
        // 자동 범위(range=0 이면 자동).
        fig.range[0][0] = fig.range[0][1] = 0;
        fig.range[1][0] = fig.range[1][1] = 0;
    }

    static void add(mjvFigure& fig, int line, float x, float y) {
        int pnt = fig.linepnt[line];
        if (pnt >= mjMAXLINEPNT) {   // 링 버퍼: 가득 차면 한 칸 밀기
            for (int i = 0; i < mjMAXLINEPNT - 1; ++i) {
                fig.linedata[line][2 * i]     = fig.linedata[line][2 * i + 2];
                fig.linedata[line][2 * i + 1] = fig.linedata[line][2 * i + 3];
            }
            pnt = mjMAXLINEPNT - 1;
        }
        fig.linedata[line][2 * pnt]     = x;
        fig.linedata[line][2 * pnt + 1] = y;
        fig.linepnt[line] = pnt + 1;
    }

    mjvFigure figX_{};
    mjvFigure figY_{};
    double t0_ = 0.0;
    bool   have_t0_ = false;
    long   count_ = 0;
    int    decim_ = 5;   // 500 Hz -> 100 Hz 기록 (≈10초 창)
};

}  // namespace kin
