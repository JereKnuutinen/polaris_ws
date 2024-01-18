//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct1.h
//
// Code generation for function 'factoryConstruct1'
//

#ifndef FACTORYCONSTRUCT1_H
#define FACTORYCONSTRUCT1_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}
struct o_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun,
                      const anonymous_function &nonlin, int mCineq,
                      const double lb[113], const double ub[113],
                      o_struct_T &obj);

}
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (factoryConstruct1.h)
