//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// enu2lla.h
//
// Code generation for function 'enu2lla'
//

#ifndef ENU2LLA_H
#define ENU2LLA_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void b_enu2lla(const double xyzENU[6], const double b_lla0[3], double lla[6]);

void enu2lla(const double xyzENU[3], const double b_lla0[3], double lla[3]);

} // namespace coder

#endif
// End of code generation (enu2lla.h)
