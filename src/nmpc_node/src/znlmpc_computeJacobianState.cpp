//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// znlmpc_computeJacobianState.cpp
//
// Code generation for function 'znlmpc_computeJacobianState'
//

// Include files
#include "znlmpc_computeJacobianState.h"
#include "dynamic_model_nmpc.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
void znlmpc_computeJacobianState(const double f0[14], double x0[14],
                                 double u0[2], double Jx[196], double Jmv[28])
{
  double f[14];
  double xa[14];
  double ua[2];
  double dx;
  for (int k{0}; k < 14; k++) {
    dx = std::abs(x0[k]);
    xa[k] = dx;
    if (dx < 1.0) {
      xa[k] = 1.0;
    }
  }
  for (int k{0}; k < 14; k++) {
    dx = 1.0E-6 * xa[k];
    x0[k] += dx;
    dynamic_model_nmpc(x0, u0, f);
    x0[k] -= dx;
    for (int i{0}; i < 14; i++) {
      Jx[i + 14 * k] = (f[i] - f0[i]) / dx;
    }
  }
  ua[0] = std::abs(u0[0]);
  ua[1] = std::abs(u0[1]);
  if (ua[0] < 1.0) {
    ua[0] = 1.0;
  }
  if (ua[1] < 1.0) {
    ua[1] = 1.0;
  }
  for (int k{0}; k < 2; k++) {
    dx = 1.0E-6 * ua[k];
    u0[k] += dx;
    dynamic_model_nmpc(x0, u0, f);
    u0[k] -= dx;
    for (int i{0}; i < 14; i++) {
      Jmv[i + 14 * k] = (f[i] - f0[i]) / dx;
    }
  }
}

} // namespace coder

// End of code generation (znlmpc_computeJacobianState.cpp)
