#include "config/WalkingConfig.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

// 외부 JSON 라이브러리 없이, "key": number 형태의 평면 JSON 만 다루는 최소 파서/기록기.
// (중첩 객체/배열 없음. 알 수 없는 키는 무시, 없는 키는 기존 값 유지.)
namespace kin {
namespace config {

namespace {
// 문자열에서 "key" 뒤의 숫자 값을 추출. 찾으면 out 에 넣고 true.
bool findNumber(const std::string& s, const std::string& key, double& out) {
    const std::string pat = "\"" + key + "\"";
    size_t p = s.find(pat);
    if (p == std::string::npos) return false;
    p = s.find(':', p + pat.size());
    if (p == std::string::npos) return false;
    ++p;
    while (p < s.size() && (s[p] == ' ' || s[p] == '\t' || s[p] == '\n' || s[p] == '\r'))
        ++p;
    const char* start = s.c_str() + p;
    char* end = nullptr;
    double v = std::strtod(start, &end);
    if (end == start) return false;
    out = v;
    return true;
}

// 편의 매크로: 각 필드를 키로 로드.
void assignAll(const std::string& s, WalkingConfig& c) {
    double v;
    auto D = [&](const char* k, double& f) { if (findNumber(s, k, v)) f = v; };
    auto I = [&](const char* k, int& f)    { if (findNumber(s, k, v)) f = static_cast<int>(v + (v >= 0 ? 0.5 : -0.5)); };

    D("controlDt", c.controlDt);
    D("servoKp", c.servoKp);            D("servoKv", c.servoKv);
    D("servoKi", c.servoKi);            D("servoIClampRad", c.servoIClampRad);
    D("anklePitchKp", c.anklePitchKp);  D("anklePitchKv", c.anklePitchKv);
    D("ankleRollKp", c.ankleRollKp);    D("ankleRollKv", c.ankleRollKv);
    D("gravityComp", c.gravityComp);          D("servoKvFF", c.servoKvFF);
    D("stepPeriod", c.stepPeriod);
    D("stepPeriodStart", c.stepPeriodStart);
    D("stepPeriodEnd", c.stepPeriodEnd);
    I("startRampSteps", c.startRampSteps);
    D("doubleSupportRatio", c.doubleSupportRatio);
    D("stepHeight", c.stepHeight);
    D("halfWidth", c.halfWidth);
    D("maxStridePerStep", c.maxStridePerStep);
    D("maxSwayPerStep", c.maxSwayPerStep);
    D("minFootClearance", c.minFootClearance);
    D("sidestepLeadOnly", c.sidestepLeadOnly);
    D("comHeight", c.comHeight);
    D("previewSec", c.previewSec);
    D("gravity", c.gravity);
    D("previewQe", c.previewQe);        D("previewR", c.previewR);
    D("comMassScale", c.comMassScale);
    D("closedLoop", c.closedLoop);
    D("footRefAnkle", c.footRefAnkle);
    D("comMeasPlanted", c.comMeasPlanted);
    D("floatingBase", c.floatingBase);  D("dsBothFeet", c.dsBothFeet);
    D("lamContact", c.lamContact);      D("ikCompare", c.ikCompare);
    D("pelvisBodyFrame", c.pelvisBodyFrame);
    D("kpPelvisYaw", c.kpPelvisYaw);
    D("pelvisImuRollPitch", c.pelvisImuRollPitch);
    D("baseAnchorPlan", c.baseAnchorPlan);
    D("ankleAdmAtFootTarget", c.ankleAdmAtFootTarget);
    D("ankleAdmSign", c.ankleAdmSign);
    D("kpContact", c.kpContact);
    D("kpCom", c.kpCom);                D("kpSwing", c.kpSwing);
    D("kpHand", c.kpHand);              D("kpPelvis", c.kpPelvis);
    D("kpWaist", c.kpWaist);
    D("lamSupport", c.lamSupport);      D("lamCom", c.lamCom);
    D("lamSwing", c.lamSwing);          D("lamHand", c.lamHand);
    D("lamPelvis", c.lamPelvis);        D("lamWaist", c.lamWaist);
    D("ankleAdmEnable", c.ankleAdmEnable);   D("ankleAdmKPitch", c.ankleAdmKPitch);
    D("ankleAdmKRoll", c.ankleAdmKRoll);     D("ankleAdmTau", c.ankleAdmTau);
    D("ankleAdmClamp", c.ankleAdmClamp);     D("ankleAdmFtTau", c.ankleAdmFtTau);
    D("ankleAdmFzMin", c.ankleAdmFzMin);       D("ankleAdmFzNom", c.ankleAdmFzNom);
    D("vFwdMax", c.vFwdMax);            D("vLatMax", c.vLatMax);
    D("vYawMax", c.vYawMax);            D("vAccel", c.vAccel);
    D("yawAccel", c.yawAccel);
}
}  // namespace

bool loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f) return false;
    std::stringstream ss;
    ss << f.rdbuf();
    assignAll(ss.str(), gConfig);
    return true;
}

bool saveToJson(const std::string& path) {
    std::ofstream f(path);
    if (!f) return false;
    const WalkingConfig& c = gConfig;
    f << "{\n";
    f << "  \"_comment\": \"kin_humanoid 보행 제어 설정. 값만 바꿔 재실행하면 반영됨(재컴파일 불필요).\",\n\n";
    f << "  \"controlDt\": " << c.controlDt << ",\n\n";
    f << "  \"servoKp\": " << c.servoKp << ",\n";
    f << "  \"servoKv\": " << c.servoKv << ",\n";
    f << "  \"servoKi\": " << c.servoKi << ",\n";
    f << "  \"servoIClampRad\": " << c.servoIClampRad << ",\n";
    f << "  \"anklePitchKp\": " << c.anklePitchKp << ",\n";
    f << "  \"anklePitchKv\": " << c.anklePitchKv << ",\n";
    f << "  \"ankleRollKp\": " << c.ankleRollKp << ",\n";
    f << "  \"ankleRollKv\": " << c.ankleRollKv << ",\n";
    f << "  \"gravityComp\": " << c.gravityComp << ",\n";
    f << "  \"servoKvFF\": " << c.servoKvFF << ",\n\n";
    f << "  \"stepPeriod\": " << c.stepPeriod << ",\n";
    f << "  \"stepPeriodStart\": " << c.stepPeriodStart << ",\n";
    f << "  \"stepPeriodEnd\": " << c.stepPeriodEnd << ",\n";
    f << "  \"startRampSteps\": " << c.startRampSteps << ",\n";
    f << "  \"doubleSupportRatio\": " << c.doubleSupportRatio << ",\n";
    f << "  \"stepHeight\": " << c.stepHeight << ",\n";
    f << "  \"halfWidth\": " << c.halfWidth << ",\n";
    f << "  \"maxStridePerStep\": " << c.maxStridePerStep << ",\n";
    f << "  \"maxSwayPerStep\": " << c.maxSwayPerStep << ",\n";
    f << "  \"minFootClearance\": " << c.minFootClearance << ",\n";
    f << "  \"sidestepLeadOnly\": " << c.sidestepLeadOnly << ",\n\n";
    f << "  \"comHeight\": " << c.comHeight << ",\n";
    f << "  \"previewSec\": " << c.previewSec << ",\n";
    f << "  \"gravity\": " << c.gravity << ",\n";
    f << "  \"previewQe\": " << c.previewQe << ",\n";
    f << "  \"previewR\": " << c.previewR << ",\n";
    f << "  \"comMassScale\": " << c.comMassScale << ",\n";
    f << "  \"closedLoop\": " << c.closedLoop << ",\n";
    f << "  \"footRefAnkle\": " << c.footRefAnkle << ",\n";
    f << "  \"comMeasPlanted\": " << c.comMeasPlanted << ",\n\n";
    f << "  \"floatingBase\": " << c.floatingBase << ",\n";
    f << "  \"dsBothFeet\": " << c.dsBothFeet << ",\n";
    f << "  \"lamContact\": " << c.lamContact << ",\n";
    f << "  \"ikCompare\": " << c.ikCompare << ",\n";
    f << "  \"pelvisBodyFrame\": " << c.pelvisBodyFrame << ",\n";
    f << "  \"kpPelvisYaw\": " << c.kpPelvisYaw << ",\n";
    f << "  \"pelvisImuRollPitch\": " << c.pelvisImuRollPitch << ",\n\n";
    f << "  \"baseAnchorPlan\": " << c.baseAnchorPlan << ",\n";
    f << "  \"ankleAdmAtFootTarget\": " << c.ankleAdmAtFootTarget << ",\n";
    f << "  \"ankleAdmSign\": " << c.ankleAdmSign << ",\n\n";
    f << "  \"kpContact\": " << c.kpContact << ",\n";
    f << "  \"kpCom\": " << c.kpCom << ",\n";
    f << "  \"kpSwing\": " << c.kpSwing << ",\n";
    f << "  \"kpHand\": " << c.kpHand << ",\n";
    f << "  \"kpPelvis\": " << c.kpPelvis << ",\n";
    f << "  \"kpWaist\": " << c.kpWaist << ",\n\n";
    f << "  \"lamSupport\": " << c.lamSupport << ",\n";
    f << "  \"lamCom\": " << c.lamCom << ",\n";
    f << "  \"lamSwing\": " << c.lamSwing << ",\n";
    f << "  \"lamHand\": " << c.lamHand << ",\n";
    f << "  \"lamPelvis\": " << c.lamPelvis << ",\n";
    f << "  \"lamWaist\": " << c.lamWaist << ",\n\n";
    f << "  \"ankleAdmEnable\": " << c.ankleAdmEnable << ",\n";
    f << "  \"ankleAdmKPitch\": " << c.ankleAdmKPitch << ",\n";
    f << "  \"ankleAdmKRoll\": " << c.ankleAdmKRoll << ",\n";
    f << "  \"ankleAdmTau\": " << c.ankleAdmTau << ",\n";
    f << "  \"ankleAdmClamp\": " << c.ankleAdmClamp << ",\n";
    f << "  \"ankleAdmFtTau\": " << c.ankleAdmFtTau << ",\n";
    f << "  \"ankleAdmFzMin\": " << c.ankleAdmFzMin << ",\n";
    f << "  \"ankleAdmFzNom\": " << c.ankleAdmFzNom << ",\n\n";
    f << "  \"vFwdMax\": " << c.vFwdMax << ",\n";
    f << "  \"vLatMax\": " << c.vLatMax << ",\n";
    f << "  \"vYawMax\": " << c.vYawMax << ",\n";
    f << "  \"vAccel\": " << c.vAccel << ",\n";
    f << "  \"yawAccel\": " << c.yawAccel << "\n";
    f << "}\n";
    return true;
}

}  // namespace config
}  // namespace kin
