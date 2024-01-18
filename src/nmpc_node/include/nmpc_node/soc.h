//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// soc.h
//
// Code generation for function 'soc'
//

#ifndef SOC_H
#define SOC_H

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
namespace fminconsqp {
namespace step {
bool soc(const double Hessian[12769], const ::coder::array<double, 1U> &grad,
         d_struct_T &b_TrialState, h_struct_T &memspace, i_struct_T &WorkingSet,
         e_struct_T &b_QRManager, f_struct_T &b_CholManager,
         g_struct_T &QPObjective, const n_struct_T &qpoptions);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (soc.h)
