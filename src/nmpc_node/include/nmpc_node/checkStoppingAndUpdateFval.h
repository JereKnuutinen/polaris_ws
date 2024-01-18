//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// checkStoppingAndUpdateFval.h
//
// Code generation for function 'checkStoppingAndUpdateFval'
//

#ifndef CHECKSTOPPINGANDUPDATEFVAL_H
#define CHECKSTOPPINGANDUPDATEFVAL_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

struct h_struct_T;

struct g_struct_T;

struct i_struct_T;

struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace stopping {
void checkStoppingAndUpdateFval(
    int &activeSetChangeID, const ::coder::array<double, 1U> &f,
    d_struct_T &solution, h_struct_T &memspace, const g_struct_T &objective,
    i_struct_T &workingset, e_struct_T &qrmanager,
    double options_ConstraintTolerance, double options_ObjectiveLimit,
    bool options_IterDisplayQP, int runTimeOptions_MaxIterations,
    double runTimeOptions_ConstrRelTolFactor, bool updateFval);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (checkStoppingAndUpdateFval.h)
