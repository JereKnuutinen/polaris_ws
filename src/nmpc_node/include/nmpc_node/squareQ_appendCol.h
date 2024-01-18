//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// squareQ_appendCol.h
//
// Code generation for function 'squareQ_appendCol'
//

#ifndef SQUAREQ_APPENDCOL_H
#define SQUAREQ_APPENDCOL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void squareQ_appendCol(e_struct_T &obj, const ::coder::array<double, 1U> &vec,
                       int iv0);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (squareQ_appendCol.h)
