//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// BFGSUpdate.h
//
// Code generation for function 'BFGSUpdate'
//

#ifndef BFGSUPDATE_H
#define BFGSUPDATE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
bool BFGSUpdate(int nvar, double Bk[12769],
                const ::coder::array<double, 1U> &sk,
                ::coder::array<double, 1U> &yk,
                ::coder::array<double, 2U> &workspace);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (BFGSUpdate.h)
