//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// solve.h
//
// Code generation for function 'solve'
//

#ifndef SOLVE_H
#define SOLVE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace CholManager {
void solve(const f_struct_T &obj, ::coder::array<double, 1U> &rhs);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (solve.h)
