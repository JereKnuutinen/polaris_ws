//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleratiotest.h
//
// Code generation for function 'feasibleratiotest'
//

#ifndef FEASIBLERATIOTEST_H
#define FEASIBLERATIOTEST_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
double feasibleratiotest(
    const ::coder::array<double, 1U> &solution_xstar,
    const ::coder::array<double, 1U> &solution_searchDir,
    ::coder::array<double, 2U> &workspace, int workingset_nVar,
    int workingset_ldA, const ::coder::array<double, 1U> &workingset_Aineq,
    const ::coder::array<double, 1U> &workingset_bineq,
    const ::coder::array<double, 1U> &workingset_lb,
    const ::coder::array<double, 1U> &workingset_ub,
    const ::coder::array<int, 1U> &workingset_indexLB,
    const ::coder::array<int, 1U> &workingset_indexUB,
    const int workingset_sizes[5], const int workingset_isActiveIdx[6],
    const ::coder::array<bool, 1U> &workingset_isActiveConstr,
    const int workingset_nWConstr[5], bool isPhaseOne, double tolcon,
    bool &newBlocking, int &constrType, int &constrIdx);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (feasibleratiotest.h)
