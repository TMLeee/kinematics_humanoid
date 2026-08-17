#include "controller/WholeBodyIK.h"

namespace kin {

MatrixXd WholeBodyIK::baseSlave(const MatrixXd& Jsupport, int nJoints, double lambda) {
    // Jsupport: 6×nv,  nv = kBaseV + nJoints.
    MatrixXd Jb = Jsupport.leftCols(kBaseV);          // 6×6 (베이스)
    MatrixXd Jj = Jsupport.rightCols(nJoints);        // 6×nJoints (관절)
    // v_base = -Jb^{-1} Jj v_joint  (특이점 안전을 위해 DLS 역행렬)
    return -dampedPinv(Jb, lambda) * Jj;              // 6×nJoints
}

MatrixXd WholeBodyIK::reduce(const MatrixXd& Jtask, const MatrixXd& S) {
    const int nJoints = static_cast<int>(S.cols());
    MatrixXd Jbase  = Jtask.leftCols(kBaseV);         // m×6
    MatrixXd Jjoint = Jtask.rightCols(nJoints);       // m×nJoints
    return Jjoint + Jbase * S;                        // m×nJoints (support-consistent)
}

VectorXd WholeBodyIK::solve(const std::vector<Task>& tasks, int nJoints,
                            std::vector<TaskDiag>* diag) {
    VectorXd dq = VectorXd::Zero(nJoints);
    MatrixXd N  = MatrixXd::Identity(nJoints, nJoints);   // null-space projector
    if (diag) { diag->clear(); diag->reserve(tasks.size()); }

    for (const Task& t : tasks) {
        if (t.J.rows() == 0) { if (diag) diag->push_back(TaskDiag{}); continue; }
        const MatrixXd& Ji = t.J;                 // m×nJoints
        MatrixXd JiN = Ji * N;                    // 현재 null space 로 투영
        if (diag) {
            Eigen::JacobiSVD<MatrixXd> svd(JiN);
            const VectorXd sv = svd.singularValues();
            TaskDiag d;
            d.rows = static_cast<int>(Ji.rows());
            d.sigMax = sv(0);
            d.sigMin = sv(sv.size() - 1);
            diag->push_back(d);
        }
        MatrixXd JiN_pinv = dampedPinv(JiN, t.lambda);   // nJoints×m
        // 상위 task 결과(dq)를 방해하지 않으면서 이 task 오차를 최소화.
        dq += JiN_pinv * (t.xdot - Ji * dq);
        N  -= JiN_pinv * JiN;                     // null-space 갱신
    }
    return dq;
}

}  // namespace kin
