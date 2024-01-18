//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fmincon.h
//
// Code generation for function 'fmincon'
//

#ifndef FMINCON_H
#define FMINCON_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}
struct m_struct_T;

// Function Declarations
namespace coder {
double fmincon(const anonymous_function &fun, const double x0[113],
               const ::coder::array<double, 2U> &Aineq,
               const ::coder::array<double, 1U> &bineq, const double lb[113],
               const double ub[113], const anonymous_function &nonlcon,
               double x[113], double &exitflag, m_struct_T &output);

}

#endif
// End of code generation (fmincon.h)
