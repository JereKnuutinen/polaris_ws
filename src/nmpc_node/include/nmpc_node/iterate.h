//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// iterate.h
//
// Code generation for function 'iterate'
//

#ifndef ITERATE_H
#define ITERATE_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct d_struct_T;

struct h_struct_T;

struct i_struct_T;

struct e_struct_T;

struct f_struct_T;

struct g_struct_T;

struct n_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void iterate(const double H[12769], const ::coder::array<double, 1U> &f,
             d_struct_T &solution, h_struct_T &memspace, i_struct_T &workingset,
             e_struct_T &qrmanager, f_struct_T &cholmanager,
             g_struct_T &objective, const char options_SolverName[7],
             double options_StepTolerance, double options_ConstraintTolerance,
             double options_ObjectiveLimit, double options_PricingTolerance,
             bool options_IterDisplayQP, const n_struct_T &runTimeOptions);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (iterate.h)
