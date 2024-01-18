//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// printInfo.cpp
//
// Code generation for function 'printInfo'
//

// Include files
#include "printInfo.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace display {
void printInfo(bool newBlocking, int PROBLEM_TYPE, double alpha,
               double stepNorm, int activeConstrChangedType,
               int localActiveConstrIdx, int activeSetChangeID,
               double solution_fstar, double solution_firstorderopt,
               double solution_maxConstr, int solution_iterations,
               const ::coder::array<int, 1U> &workingset_indexLB,
               const ::coder::array<int, 1U> &workingset_indexUB,
               int workingset_nActiveConstr)
{
  std::printf("%5i  %14.6e  %14.6e  %14.6e", solution_iterations,
              solution_fstar, solution_firstorderopt, solution_maxConstr);
  std::fflush(stdout);
  std::printf("  ");
  std::fflush(stdout);
  if (std::isnan(alpha)) {
    std::printf("       -      ");
    std::fflush(stdout);
  } else {
    std::printf("%14.6e", alpha);
    std::fflush(stdout);
  }
  std::printf("  ");
  std::fflush(stdout);
  std::printf("%14.6e", stepNorm);
  std::fflush(stdout);
  std::printf("    ");
  std::fflush(stdout);
  if (newBlocking || (activeSetChangeID == -1)) {
    int b_localActiveConstrIdx;
    if (newBlocking) {
      activeSetChangeID = 1;
    }
    b_localActiveConstrIdx = localActiveConstrIdx;
    switch (activeSetChangeID) {
    case -1:
      std::printf("-");
      std::fflush(stdout);
      break;
    case 1:
      std::printf("+");
      std::fflush(stdout);
      break;
    default:
      std::printf(" ");
      std::fflush(stdout);
      break;
    }
    switch (activeConstrChangedType) {
    case 3:
      std::printf("AINEQ");
      std::fflush(stdout);
      break;
    case 4:
      std::printf("LOWER");
      std::fflush(stdout);
      b_localActiveConstrIdx = workingset_indexLB[localActiveConstrIdx - 1];
      break;
    case 5:
      std::printf("UPPER");
      std::fflush(stdout);
      b_localActiveConstrIdx = workingset_indexUB[localActiveConstrIdx - 1];
      break;
    default:
      std::printf("SAME ");
      std::fflush(stdout);
      b_localActiveConstrIdx = -1;
      break;
    }
    std::printf("(%-5i)", b_localActiveConstrIdx);
    std::fflush(stdout);
  } else {
    std::printf(" SAME ");
    std::fflush(stdout);
    std::printf("(%-5i)", -1);
    std::fflush(stdout);
  }
  std::printf("           ");
  std::fflush(stdout);
  std::printf("%5i", workingset_nActiveConstr);
  std::fflush(stdout);
  std::printf("    ");
  std::fflush(stdout);
  switch (PROBLEM_TYPE) {
  case 1:
    std::printf("Phase One");
    std::fflush(stdout);
    break;
  case 2:
    std::printf("Regularized");
    std::fflush(stdout);
    break;
  case 4:
    std::printf("Phase One Reg");
    std::fflush(stdout);
    break;
  default:
    std::printf("Normal");
    std::fflush(stdout);
    break;
  }
  std::printf("\n");
  std::fflush(stdout);
}

} // namespace display
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (printInfo.cpp)
