//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// NMPC_Node_rtwutil.cpp
//
// Code generation for function 'NMPC_Node_rtwutil'
//

// Include files
#include "NMPC_Node_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
int div_nzp_s32(int numerator)
{
  int quotient;
  unsigned int tempAbsQuotient;
  if (numerator < 0) {
    tempAbsQuotient = ~static_cast<unsigned int>(numerator) + 1U;
  } else {
    tempAbsQuotient = static_cast<unsigned int>(numerator);
  }
  tempAbsQuotient /= 50U;
  if (numerator < 0) {
    quotient = -static_cast<int>(tempAbsQuotient);
  } else {
    quotient = static_cast<int>(tempAbsQuotient);
  }
  return quotient;
}

// End of code generation (NMPC_Node_rtwutil.cpp)
