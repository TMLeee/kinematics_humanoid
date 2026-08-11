# =====================================================================
#  kin_humanoid 빌드
#
#  병렬 빌드: `make -j$(nproc)` 로 CPU 코어 수만큼 동시 컴파일한다.
#  (VS Code tasks.json 이 nproc 값을 읽어 -j 인자로 넘긴다)
#
#  타깃:
#    make            # = make main  (뷰어 포함 앱 -> ./main)
#    make headless_test   # 헤드리스 보행 검증 하네스 -> ./headless_test
#    make clean
#
#  오브젝트 파일은 build/ 아래에 저장되며 .gitignore 로 무시된다.
#  main(-O0 -g) 과 headless_test(-O2) 는 최적화 옵션이 달라
#  오브젝트 디렉터리를 분리한다(build/main, build/test).
# =====================================================================

CXX      := g++

# 공통 include / 링크 경로
INCLUDES := -Isrc -I/opt/mujoco/mujoco/include -I/usr/include/eigen3 -I/usr/local/include
LDFLAGS  := -L/opt/mujoco/mujoco/lib -L/usr/local/lib \
            -Wl,-rpath,/opt/mujoco/mujoco/lib -Wl,-rpath,/usr/local/lib

# 헤더 변경 시에도 증분 재빌드가 정확하도록 의존성(.d) 자동 생성
DEPFLAGS := -MMD -MP
COMMON   := -std=c++17 -fdiagnostics-color=always $(INCLUDES) $(DEPFLAGS)

# ---- main app (뷰어) ----
MAIN_CXXFLAGS := $(COMMON) -g -O0
MAIN_LDLIBS   := -lmujoco -lglfw -lGLEW -lGL -lrbdl -lrbdl_urdfreader -lm -ldl -lpthread
MAIN_SRCS := \
    src/main.cpp \
    src/config/WalkingConfig.cpp \
    src/simulator/MujocoEnv.cpp \
    src/model/MujocoModel.cpp \
    src/model/RbdlModel.cpp \
    src/io/SimIO.cpp \
    src/io/RealIO.cpp \
    src/controller/FootstepGenerator.cpp \
    src/controller/WholeBodyIK.cpp \
    src/controller/HumanoidController.cpp
MAIN_OBJS := $(MAIN_SRCS:%.cpp=build/main/%.o)

# ---- headless 검증 하네스 ----
TEST_CXXFLAGS := $(COMMON) -O2
TEST_LDLIBS   := -lmujoco -lglfw -lGLEW -lGL -lm -ldl -lpthread
TEST_SRCS := \
    test/headless_walk_test.cpp \
    src/config/WalkingConfig.cpp \
    src/simulator/MujocoEnv.cpp \
    src/model/MujocoModel.cpp \
    src/io/SimIO.cpp \
    src/controller/FootstepGenerator.cpp \
    src/controller/WholeBodyIK.cpp \
    src/controller/HumanoidController.cpp
TEST_OBJS := $(TEST_SRCS:%.cpp=build/test/%.o)

.PHONY: all test clean
all: main

# ---- 링크 ----
main: $(MAIN_OBJS)
	$(CXX) $(MAIN_OBJS) -o $@ $(LDFLAGS) $(MAIN_LDLIBS)

test: headless_test
headless_test: $(TEST_OBJS)
	$(CXX) $(TEST_OBJS) -o $@ $(LDFLAGS) $(TEST_LDLIBS)

# ---- 컴파일 (오브젝트 디렉터리 분리) ----
build/main/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(MAIN_CXXFLAGS) -c $< -o $@

build/test/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(TEST_CXXFLAGS) -c $< -o $@

clean:
	rm -rf build main headless_test

# ---- 자동 생성된 헤더 의존성 포함 ----
-include $(MAIN_OBJS:.o=.d) $(TEST_OBJS:.o=.d)
