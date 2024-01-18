//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// partialColLDL3_.h
//
// Code generation for function 'partialColLDL3_'
//

#ifndef PARTIALCOLLDL3__H
#define PARTIALCOLLDL3__H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct f_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void partialColLDL3_(f_struct_T &obj, int LD_offset, int NColsRemain,
                     double REG_PRIMAL);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (partialColLDL3_.h)
